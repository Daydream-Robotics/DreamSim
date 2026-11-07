// Runs robot-core's MPC path following for DreamSim, called from Python through ctypes
// (scripts/Motion/mpc_bridge.py).
//
// Loads a JerryIO file with robot-core's own Path::buildAllPathsFromJerryIO and solves with
// its MPC, so paths, reference generation, tuning and delay compensation all come from
// robot-core. Only the follower loop is copied: the Nano's PathFollower::step talks to the
// brain through SerialLink, so it can't be linked here. Keep mpc_bridge_step in sync with
// jetson-nano/src/motion/pathFollower.cpp.
//
// Frames and units are robot-core's: JerryIO field frame (field-centre origin, +x forward,
// +y left), inches, theta 0 along +y and pi/2 along +x, wheel speeds in rad/s at the wheel.
#include "config/constants.h"
#include "motion/control/mpc.hpp"
#include "motion/pathing/paths.hpp"

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#ifdef _WIN32
#define BRIDGE_API extern "C" __declspec(dllexport)
#else
#define BRIDGE_API extern "C"
#endif

// mpc_bridge_step results
enum BridgeStatus {
    BRIDGE_RUNNING = 0,
    BRIDGE_FINISHED = 1,
    BRIDGE_ERROR = -1,
};

struct Bridge {
    std::vector<ALS_Path> paths;
    std::vector<PathFlag> flags;
    Pose startPose{0.0, 0.0, 0.0};

    MPC mpc;
    const ALS_Path* path = nullptr;
    PathFlag flag = PathFlag::FORWARDS;
    std::size_t currentSampleIdx = 0;
    bool finished = true;
};

// Loads every path in a JerryIO file. Returns null if the file has no paths.
BRIDGE_API Bridge* mpc_bridge_create(const char* pathFile) {
    auto bridge = std::make_unique<Bridge>();
    bridge->paths = Path::buildAllPathsFromJerryIO(pathFile, bridge->startPose, bridge->flags);
    if (bridge->paths.empty()) {
        std::fprintf(stderr, "[MPC bridge] no paths loaded from %s\n", pathFile);
        return nullptr;
    }
    return bridge.release();
}

BRIDGE_API void mpc_bridge_destroy(Bridge* bridge) {
    delete bridge;
}

BRIDGE_API int mpc_bridge_path_count(const Bridge* bridge) {
    return static_cast<int>(bridge->paths.size());
}

// The file's starting pose: the first point of the first path, facing the file's heading.
BRIDGE_API void mpc_bridge_start_pose(const Bridge* bridge, double* x, double* y, double* theta) {
    *x = bridge->startPose.x;
    *y = bridge->startPose.y;
    *theta = bridge->startPose.theta;
}

// Starts following a path (PathFollower::setPath). Returns 0, or -1 for a bad index.
BRIDGE_API int mpc_bridge_set_path(Bridge* bridge, int index) {
    if (index < 0 || index >= static_cast<int>(bridge->paths.size())) {
        return BRIDGE_ERROR;
    }
    bridge->path = &bridge->paths[index];
    bridge->flag = bridge->flags[index];
    bridge->currentSampleIdx = 0;
    bridge->finished = false;
    bridge->mpc.reset();
    return 0;
}

// One control step (PathFollower::step). Writes the drive voltages (V) and the distance left
// along the path (in). Returns BRIDGE_RUNNING, BRIDGE_FINISHED (stop the drive) or
// BRIDGE_ERROR (no path set).
//
// omegaLeft/omegaRight are the measured wheel speeds and age is how old they are (s). The MPC
// predicts the robot forward by that age plus MPC_COMMAND_DELAY, so the caller must apply the
// returned voltages MPC_COMMAND_DELAY after this call.
BRIDGE_API int mpc_bridge_step(Bridge* bridge, double x, double y, double theta,
                               double omegaLeft, double omegaRight, double batteryVolts, double age,
                               double* vLeft, double* vRight, double* distanceLeft) {
    *vLeft = 0.0;
    *vRight = 0.0;
    *distanceLeft = 0.0;
    if (!bridge->path) {
        return BRIDGE_ERROR;
    }
    const ALS_Path& path = *bridge->path;
    if (bridge->finished || !path.isValid() || path.getSamples().empty()) {
        bridge->finished = true;
        return BRIDGE_FINISHED;
    }

    DriveState drive{omegaLeft, omegaRight, batteryVolts, age};
    Pose currentPose{x, y, theta};
    bridge->mpc.predict(currentPose, drive);

    // only search a short way ahead
    const std::vector<Sample>& samples = path.getSamples();
    const double searchLimit = samples[std::min(bridge->currentSampleIdx, samples.size() - 1)].s + PATH_SEARCH_AHEAD;
    std::size_t searchEnd = bridge->currentSampleIdx;
    while (searchEnd < samples.size() && samples[searchEnd].s <= searchLimit) {
        searchEnd++;
    }
    bridge->currentSampleIdx = path.findClosestSampleIndex({currentPose.x, currentPose.y}, bridge->currentSampleIdx, searchEnd);

    if (bridge->currentSampleIdx >= samples.size()) {
        std::fprintf(stderr, "[MPC bridge] sample index %zu out of bounds (size %zu)\n", bridge->currentSampleIdx, samples.size());
        bridge->finished = true;
        return BRIDGE_FINISHED;
    }

    *distanceLeft = path.getTotalLength() - samples[bridge->currentSampleIdx].s;
    if (*distanceLeft < END_TOLERANCE) {
        bridge->finished = true;
        return BRIDGE_FINISHED;
    }

    // compute returns mV in voltage mode, and 0 V after a failed solve
    WheelVelocities out = bridge->mpc.compute(currentPose, drive, path, bridge->currentSampleIdx, bridge->flag);
    if (out.input != ControlMode::INPUT_VOLTAGE) {
        std::fprintf(stderr, "[MPC bridge] controller returned velocities, expected voltages\n");
        return BRIDGE_ERROR;
    }
    *vLeft = out.left / 1000.0;
    *vRight = out.right / 1000.0;
    return BRIDGE_RUNNING;
}
