# **ISAAC SIM & LAB SETUP (WINDOWS 11 | standalone Isaac Sim + uv)**

## **[System Requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)**
## **[Documentation](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html)**

Versions: **Isaac Sim 6.1** (standalone) + **Isaac Lab v3.0.0-EA** (the Isaac Lab tag that targets 6.1), Python 3.12, torch 2.11 (CUDA 12.8). All Python deps are pinned in `uv.lock`.

---

## **STEPS**

### **1. Install Isaac Sim 6.1 standalone**
Unzip the Isaac Sim 6.1 Windows standalone build to `C:\isaacsim` and run `C:\isaacsim\post_install.bat`.
Somewhere else? Set `ISAACSIM_PATH` to that folder and swap it into the paths below.

### **2. [Install uv](https://docs.astral.sh/uv/getting-started/installation/)**
```bash
winget install --id=astral-sh.uv -e
```

### **3. Clone with submodules**
```bash
git clone --recurse-submodules https://github.com/Daydream-Robotics/DreamSim.git
cd DreamSim
# already cloned? pull in robot-core and its OSQP submodules:
git submodule update --init --recursive
```

### **4. Create the environment on Isaac Sim's own Python**
The venv is built on Isaac Sim's bundled interpreter and linked to the standalone install, so Isaac Sim is never installed twice. Isaac Lab and its deps (torch etc.) go into the venv.
```bash
uv venv --python C:\isaacsim\kit\python\python.exe
uv sync
uv run python scripts/link_isaacsim.py
```
`link_isaacsim.py` only needs re-running if the venv is deleted or Isaac Sim moves.

### **5. Verify**
```bash
uv run python -c "from isaacsim import SimulationApp; app = SimulationApp({'headless': True}); app.close()"
```

### **Native build (robot-core MPC)**
Needs the MSVC compiler from [VS Build Tools 2022](https://visualstudio.microsoft.com/visual-cpp-build-tools/) ("Desktop development with C++"). CMake and Ninja come from the uv dev group, so run from a "Developer PowerShell for VS 2022":
```bash
uv run cmake -S native/robotcore_mpc/third_party/robotcore/jetson-nano -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
uv run cmake --build build --target robotcore_mpc
```
