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
