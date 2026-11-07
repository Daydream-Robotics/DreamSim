"""Run robot-core's MPC over a JerryIO file without Isaac Sim.

Drives the same autonomous.py and drive_model.py physics as
drive_controller.py, on a flat 2D rigid body: no PhysX, collisions, walls or
tipping. A quick check after changing robot-core or the drive model.

usage (from the repo root, after scripts\\build_native.bat):
    python scripts/Motion/test_mpc_bridge.py [path.jerryio.txt ...]

With no files it runs config.MPC_PATH_FILE.
"""

import math
import sys
from pathlib import Path

import autonomous
import drive_config as config
import drive_model as model


def run(path_file):
    """Drive every path in path_file. Returns the list of PathResults."""
    run = autonomous.Autonomous(path_file, log=lambda line: print("  " + line))
    x, y, heading = run.start_pose()

    # World frame: metres, Isaac Sim yaw (0 along +x, anticlockwise).
    pos_x, pos_y = x * config.INCH, y * config.INCH
    yaw = model.yaw_from_heading(heading)
    vel_x = vel_y = yaw_rate = 0.0
    dt = 1.0 / config.PHYSICS_STEPS_PER_SECOND
    inertia = config.ROBOT_INERTIA[2]

    applied = (0.0, 0.0, True)
    while not (run.done and applied[:2] == (0.0, 0.0)):
        forward = vel_x * math.cos(yaw) + vel_y * math.sin(yaw)
        sideways = -vel_x * math.sin(yaw) + vel_y * math.cos(yaw)
        omega_left, omega_right = model.wheel_speeds(forward, yaw_rate)
        applied = run.physics_step(
            pos_x / config.INCH, pos_y / config.INCH,
            model.heading_from_yaw(yaw), omega_left, omega_right,
        )

        force_x, force_y, torque = model.drive_wrench(
            applied[0], applied[1], forward, sideways, yaw_rate, dt,
            applied[2],
        )
        accel_x = (force_x * math.cos(yaw) - force_y * math.sin(yaw))
        accel_y = (force_x * math.sin(yaw) + force_y * math.cos(yaw))
        vel_x += accel_x / config.ROBOT_MASS * dt
        vel_y += accel_y / config.ROBOT_MASS * dt
        yaw_rate += torque / inertia * dt
        pos_x += vel_x * dt
        pos_y += vel_y * dt
        yaw += yaw_rate * dt

    run.close()
    return run.results


def main():
    files = [Path(arg) for arg in sys.argv[1:]] or [config.MPC_PATH_FILE]
    print(
        f"Brake mode {config.DRIVE_BRAKE_MODE}, sideways friction "
        f"{config.WHEEL_SIDEWAYS_FRICTION}, inertia scale "
        f"{config.ROBOT_INERTIA_SCALE}, {config.PHYSICS_STEPS_PER_SECOND} "
        "physics steps/s"
    )
    for path_file in files:
        print(f"{path_file.name}:")
        results = run(path_file)
        finished = sum(result.finished for result in results)
        failed = sum(result.failed_solves for result in results)
        worst = max(result.worst_off_path for result in results)
        print(
            f"  {finished}/{len(results)} paths finished, worst {worst:.2f} "
            f"in off the path, {failed} failed solves"
        )


if __name__ == "__main__":
    main()
