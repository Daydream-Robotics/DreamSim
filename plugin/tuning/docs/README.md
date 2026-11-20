# DreamSim MPC Tuning

An Isaac Sim extension for tuning robot-core's MPC constants
(`shared/include/config/constants.h` in the robot-core submodule) against the
sim. The physics constants in `scripts/Motion/drivetrain_config.py` stay
fixed while tuning.

## Loading it in Isaac Sim

Add `plugin/` to Isaac Sim's extension search paths once:

1. **Window → Extensions**, then the ☰ menu (top left) → **Settings**.
2. Under **Extension Search Paths**, add the repo's `plugin` folder, e.g.
   `C:\Users\<you>\DreamSim\plugin`.
3. Search the Extensions window for **tuning**, enable **DreamSim MPC
   Tuning**, and tick **Autoload** to have it on at every start.

Or launch with it enabled:

```
C:\isaacsim\isaac-sim.bat --ext-folder C:\Users\<you>\DreamSim\plugin --enable tuning
```

Open the window from **Window → DreamSim → MPC Tuning**.

Kit watches the extension's Python files and reloads the extension when they
change, so edits show up without restarting Isaac Sim.

## Overrides

The window lists every constant in `constants.h` under its `[CATEGORY]`
banner. Constants the sim applies at runtime (`TUNABLE` in
`scripts/Motion/tunable_constants.py`) get a toggle and a value: ticking it
overrides `constants.h` for sim runs from the next autonomous run (P),
without rebuilding. Overrides are saved to `tuning_overrides.json` in the
repo root (gitignored) and never change `constants.h`. Other constants show
why they're locked: physical (shared with the sim's physics or measured),
compile-time (the MPC's horizons) or not simulated.

The window also checks that the built bridge matches `constants.h`, and says
to run `scripts\build_native.bat` if it doesn't.

## Trials

The **Trial** section runs autonomous once, with the current overrides, and
shows how it went: finished, timed out, tipped, cancelled or failed, with
each path's time and error and the values it ran with.

It needs the drive controller loaded (run `scripts/Motion/drive_controller.py`
in the Script Editor) and the simulation playing. The controller registers
itself in `scripts/Motion/controller_registry.py`; the window starts runs
through `DriveController.run_trial(on_done, overrides, path_file, timeout)`,
which the optimizer will use too. `on_done` gets an
`autonomous.TrialResult`, always on a later app update, never inside a
physics step.

## Layout

```
plugin/tuning/
├── config/extension.toml      # extension metadata and dependencies
├── dreamsim/tuning/           # Python package (module dreamsim.tuning)
│   ├── __init__.py
│   ├── extension.py           # menu entry and window
│   └── constants_file.py      # parses constants.h by its [CATEGORY] banners
└── docs/README.md
```
