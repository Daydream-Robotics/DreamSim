```
.
├── isaac-sim/          # Main simulation root (from whiteboard)
│   ├── README.md       # This file
│   │
│   ├── src/            # Core Simulation Logic & Scripts
│   │   │               # "anything robot do"
│   │   ├── train_ppo.py  # PPO training script
│   │   ├── drive_model.py # Robot drivetrain model
│   │   └── perception.py # Perception module
│   │
│   ├── assets/         # Importable Simulation Assets
│   │   │               # "anything import"
│   │   ├── robot/      # Robot meshes, USDs, & configs
│   │   └── field/      # Field USDs, colliders, & props
│   │
│   └── plugin/         # Editor Plugins & Extensions
│                       # "anything editor do"
│       ├── reset_button # Extension for resetting the simulation
│       └── run_auton   # Extension for running autonomous routines
│
└── docs/               # (Suggested addition for documentation)
```

## Driving a robot

1. Open `scenes/FELD_WITH_PHYSICS_NO_TAPE.usd` with **File → Open** (not drag-in).
2. Tag the robot: Property panel → Add Attribute → `dreamsim:controllable`, uniform Bool, ticked.
3. Run `scripts/Motion/drive_controller.py` in the Script Editor.
4. **W/S** drive, **A/D** turn, **Space** stop. Keep the mouse off the viewport so W/E/R don't switch tools.
