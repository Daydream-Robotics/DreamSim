"""Point the uv venv at the standalone Isaac Sim install instead of pip-installing a second copy.

Writes a .pth into the venv's site-packages that puts <isaacsim>/site on sys.path, so Python
runs Isaac Sim's own sitecustomize.py at startup (Kit paths, DLL dirs, python_packages), and
sets the env vars python.bat would (ISAAC_PATH, CARB_APP_PATH, EXP_PATH).

usage: uv run python scripts/link_isaacsim.py [ISAACSIM_PATH]   (default: $ISAACSIM_PATH or C:/isaacsim)
"""

import os
import sys
import sysconfig
from pathlib import Path

isaacsim = Path(sys.argv[1] if len(sys.argv) > 1 else os.environ.get("ISAACSIM_PATH", "C:/isaacsim")).resolve()
site_dir = isaacsim / "site"
if not (site_dir / "sitecustomize.py").is_file():
    sys.exit(f"no Isaac Sim install at {isaacsim} (missing site/sitecustomize.py)")
if sys.prefix == sys.base_prefix:
    sys.exit("run this inside the project venv: uv run python scripts/link_isaacsim.py")

pth = Path(sysconfig.get_paths()["purelib"]) / "_isaacsim_standalone.pth"
env = {"ISAAC_PATH": isaacsim, "CARB_APP_PATH": isaacsim / "kit", "EXP_PATH": isaacsim / "apps"}
# .pth lines starting with "import" are executed at startup
setenv = "; ".join(f"os.environ.setdefault({k!r}, {str(v)!r})" for k, v in env.items())
pth.write_text(f"{site_dir}\nimport os; {setenv}\n")
print(f"linked {pth} -> {site_dir}")
