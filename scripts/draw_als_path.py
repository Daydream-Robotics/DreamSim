"""Draw robot-core ALS paths on the field as speed-coloured USD curves.

Runs build/als_dump (the real robot-core ALS_Path) on a JerryIO file and writes each path as a
flat ribbon UsdGeom.Mesh under /World/ALSPaths, coloured by path speed v (red slow -> green fast).
Field frame: JerryIO GPS coords, field-center origin, +X/+Y as in JerryIO.

usage:
    uv run python scripts/draw_als_path.py [--path FILE.jerryio.txt] [--stage FIELD.usd]
        [--height 0.02] [--width 0.015] [--yaw-deg 0] [--headless --save OUT.usd]
"""

import argparse
import csv
import io
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PATH = ROOT / "native/robotcore_mpc/third_party/robotcore/common-utils/paths/unit_tests/auton/auton.path.jerryio.txt"
ALS_DUMP = ROOT / "build" / ("als_dump.exe" if sys.platform == "win32" else "als_dump")

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--path", type=Path, default=DEFAULT_PATH, help="JerryIO path file")
parser.add_argument("--stage", type=Path, default=None, help="USD stage to draw on (default: empty stage)")
parser.add_argument("--height", type=float, default=0.02, help="line height above z=0 (m)")
parser.add_argument("--width", type=float, default=0.02, help="line width (m)")
parser.add_argument("--yaw-deg", type=float, default=0.0, help="rotate paths about Z to line up with the field USD")
parser.add_argument("--spacing", type=float, default=0.25, help="ALS sample spacing (in)")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--save", type=Path, default=None, help="save the stage here after drawing")
args = parser.parse_args()


def load_samples():
    if not ALS_DUMP.exists():
        sys.exit(f"{ALS_DUMP} not found, build it first (see setup.md, Native build)")
    out = subprocess.run([str(ALS_DUMP), str(args.path), str(args.spacing)], capture_output=True, text=True)
    if out.returncode != 0:
        sys.exit(f"als_dump failed: {out.stderr.strip()}")
    paths = {}
    for row in csv.DictReader(io.StringIO(out.stdout)):
        paths.setdefault(int(row["path"]), []).append((float(row["x"]), float(row["y"]), float(row["v"])))
    return [paths[k] for k in sorted(paths)]


paths = load_samples()

# Kit must be up before pxr/omni imports
from isaacsim import SimulationApp

app = SimulationApp({"headless": args.headless})

import omni.usd
from pxr import UsdGeom

# the ribbon drawing is shared with scripts/Motion/drive_controller.py
sys.path.insert(0, str(ROOT / "scripts" / "Motion"))
import path_display  # noqa: E402

ctx = omni.usd.get_context()
if args.stage:
    ctx.open_stage(str(args.stage.resolve()))
else:
    ctx.new_stage()
app.update()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
v_min, v_max = path_display.draw_paths(
    stage, paths, height=args.height, width=args.width, yaw_deg=args.yaw_deg
)

print(f"[ALS] drew {len(paths)} paths ({sum(map(len, paths))} samples) from {args.path.name}")
print(f"[ALS] colour: v={v_min:.1f} (red) -> v={v_max:.1f} (green), in the path file's speed units")

if args.save:
    stage.Export(str(args.save.resolve()))
    print(f"[ALS] saved {args.save}")

if not args.headless:
    while app.is_running():
        app.update()
app.close()
