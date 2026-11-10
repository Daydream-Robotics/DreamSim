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
#include "utils/mathHelpers.hpp"

#include <algorithm>
#include <cmath>
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
    // still following, but the solve failed so the voltages are 0 V (robot-core's failsafe)
    BRIDGE_SOLVE_FAILED = 2,
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

// Number of spline samples in a path, or -1 for a bad index.
BRIDGE_API int mpc_bridge_sample_count(const Bridge* bridge, int index) {
    if (index < 0 || index >= static_cast<int>(bridge->paths.size())) {
        return BRIDGE_ERROR;
    }
    return static_cast<int>(bridge->paths[index].getSamples().size());
}

// Copies up to capacity of a path's spline samples (x, y in inches, v in the file's speed
// units): the exact points the MPC follows, for drawing. Returns the number copied, or -1.
BRIDGE_API int mpc_bridge_samples(const Bridge* bridge, int index, double* x, double* y,
                                  double* v, int capacity) {
    if (index < 0 || index >= static_cast<int>(bridge->paths.size())) {
        return BRIDGE_ERROR;
    }
    const std::vector<Sample>& samples = bridge->paths[index].getSamples();
    const int count = std::min(capacity, static_cast<int>(samples.size()));
    for (int i = 0; i < count; i++) {
        x[i] = samples[i].x;
        y[i] = samples[i].y;
        v[i] = samples[i].v;
    }
    return count;
}

// Curvature speed cap at arc length sq. Copy of capAt in robot-core's
// jetson-nano/src/motion/control/mpcReference.cpp: keep in sync.
static double referenceCapAt(const ALS_Path& path, double sq, double sEnd, double vStraightMax) {
    InterpSample a = path.sampleAtArcLength(sq);
    InterpSample b = path.sampleAtArcLength(std::min(sq + 3.0, sEnd));
    double ds = std::max(1e-3, b.s - a.s);
    double kappa = std::abs(utils::wrapAngle(b.theta - a.theta)) / ds;
    return vStraightMax / (1.0 + kappa * TRACK_WIDTH / 2.0);
}

// Braking-aware cap. Copy of brakingCapAt in mpcReference.cpp: keep in sync.
static double referenceBrakingCapAt(const ALS_Path& path, double sq, double vNow, double sEnd, double vStraightMax) {
    double cap = referenceCapAt(path, sq, sEnd, vStraightMax);
    double brakeDist = (vNow * vNow) / (2.0 * MPC_A_MAX) + 2.0;
    for (double d = 2.0; d <= brakeDist; d += 2.0) {
        double sAhead = sq + d;
        if (sAhead >= sEnd) {
            break;
        }
        double c = referenceCapAt(path, sAhead, sEnd, vStraightMax);
        cap = std::min(cap, std::sqrt(c * c + 2.0 * MPC_A_MAX * d));
    }
    double dEnd = std::max(0.0, sEnd - sq);
    return std::min(cap, std::sqrt(2.0 * MPC_A_MAX * dEnd));
}

// The speed (in/s) robot-core's MPC reference targets at each of a path's spline samples:
// its speed profile stepped along the whole path from rest at the MPC period, as if the robot
// tracked it perfectly (same steps as MPCReference::build). Writes up to capacity values.
// Returns the number written, or -1 for a bad index.
BRIDGE_API int mpc_bridge_reference_speeds(const Bridge* bridge, int index, double* speeds, int capacity) {
    if (index < 0 || index >= static_cast<int>(bridge->paths.size())) {
        return BRIDGE_ERROR;
    }
    const ALS_Path& path = bridge->paths[index];
    const std::vector<Sample>& samples = path.getSamples();
    const double sEnd = samples.back().s;
    const double h = MPC_SAMPLE_PERIOD;
    const double r = DRIVE_WHEEL_DIAMETER_INCHES / 2.0;
    const double vStraightMax = 0.9 * (MPC_MOTOR_B / MPC_MOTOR_A) * 12.0 * r;

    // (arc length, speed from there) at each reference step
    std::vector<std::pair<double, double>> profile;
    double s = 0.0;
    double v = MPC_V_REF_MIN;
    while (profile.size() < 1000000) {
        double vFloor = (sEnd - s < 6.0) ? 1.0 : MPC_V_REF_MIN;
        double vCap = referenceBrakingCapAt(path, s, v, sEnd, vStraightMax);
        double vTarget = std::min(vCap, v + MPC_A_MAX * h);
        v = std::max({vTarget, v - MPC_A_MAX * h, vFloor});
        profile.emplace_back(s, v);
        s += v * h;
        if (s >= sEnd) {
            break;
        }
    }

    // read the profile at each sample's arc length
    const int count = std::min(capacity, static_cast<int>(samples.size()));
    std::size_t step = 0;
    for (int i = 0; i < count; i++) {
        while (step + 1 < profile.size() && profile[step + 1].first <= samples[i].s) {
            step++;
        }
        speeds[i] = profile[step].second;
    }
    return count;
}

// The path point the follower last matched the robot to (in, field frame): the closest-point
// search in mpc_bridge_step, run on the delay-compensated pose. Returns 0, or -1 with no path.
BRIDGE_API int mpc_bridge_closest_point(const Bridge* bridge, double* x, double* y) {
    if (!bridge->path || bridge->path->getSamples().empty()) {
        return BRIDGE_ERROR;
    }
    const std::vector<Sample>& samples = bridge->path->getSamples();
    const Sample& closest = samples[std::min(bridge->currentSampleIdx, samples.size() - 1)];
    *x = closest.x;
    *y = closest.y;
    return 0;
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

// One control step (PathFollower::step). Writes the drive voltages (V), the distance left
// along the path (in), and how far the measured pose is from the closest path sample (in).
// Returns BRIDGE_RUNNING, BRIDGE_SOLVE_FAILED (0 V this step), BRIDGE_FINISHED (stop the
// drive) or BRIDGE_ERROR (no path set).
//
// omegaLeft/omegaRight are the measured wheel speeds and age is how old they are (s). The MPC
// predicts the robot forward by that age plus MPC_COMMAND_DELAY, so the caller must apply the
// returned voltages MPC_COMMAND_DELAY after this call.
BRIDGE_API int mpc_bridge_step(Bridge* bridge, double x, double y, double theta,
                               double omegaLeft, double omegaRight, double batteryVolts, double age,
                               double* vLeft, double* vRight, double* distanceLeft, double* offPath) {
    *vLeft = 0.0;
    *vRight = 0.0;
    *distanceLeft = 0.0;
    *offPath = 0.0;
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

    const Sample& closest = samples[bridge->currentSampleIdx];
    *distanceLeft = path.getTotalLength() - closest.s;
    *offPath = std::hypot(x - closest.x, y - closest.y);
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
    // a failed solve returns exactly 0 V on both sides; a real solve practically never does
    if (out.left == 0.0 && out.right == 0.0) {
        return BRIDGE_SOLVE_FAILED;
    }
    return BRIDGE_RUNNING;
}
