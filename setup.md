# **ISAAC SIM & LAB SETUP (WINDOWS 11 | pip)**

## **[System Requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)**
## **[Documentation](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html)**

---

## **STEPS**

### **1. [Install Miniconda](https://www.anaconda.com/docs/getting-started/miniconda/install/windows-cli-install)**

### **2. Create an environment in Anaconda Prompt**
```bash
conda create -n env_isaaclab python=3.11
conda activate env_isaaclab
python -m pip install --upgrade pip
```

### **3. Install dependencies**
```bash
pip install "isaacsim[all,extscache]==5.1.0" --extra-index-url https://pypi.nvidia.com
pip install -U torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
# VERIFY:
isaacsim
```

### **4. Clone Isaac Lab repo into workspace**
```bash
git clone git@github.com:isaac-sim/IsaacLab.git --branch main
cd IsaacLab
# UTILITIES:
isaaclab.bat --help
```

### **5. Install Isaac Lab**
```bash
isaaclab.bat --install
# or
isaaclab.bat -i
```

### **6. Verify Installation from top of repo**
```bash
isaaclab.bat -p scripts\tutorials\00_sim\create_empty.py
```
