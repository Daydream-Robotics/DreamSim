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
import math
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PATH = ROOT / "native/robotcore_mpc/third_party/robotcore/common-utils/paths/unit_tests/auton/auton.path.jerryio.txt"
ALS_DUMP = ROOT / "build" / ("als_dump.exe" if sys.platform == "win32" else "als_dump")
INCH = 0.0254

# slow -> fast: red, yellow, green (through yellow so the middle doesn't go brown)
SPEED_COLORS = [(0.9, 0.0, 0.0), (1.0, 0.85, 0.0), (0.0, 0.8, 0.0)]

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


def speed_color(t):
    t = min(max(t, 0.0), 1.0) * (len(SPEED_COLORS) - 1)
    i = min(int(t), len(SPEED_COLORS) - 2)
    f = t - i
    return tuple(a + (b - a) * f for a, b in zip(SPEED_COLORS[i], SPEED_COLORS[i + 1]))


paths = load_samples()

# Kit must be up before pxr/omni imports
from isaacsim import SimulationApp

app = SimulationApp({"headless": args.headless})

import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

ctx = omni.usd.get_context()
if args.stage:
    ctx.open_stage(str(args.stage.resolve()))
else:
    ctx.new_stage()
app.update()
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
to_stage = INCH / UsdGeom.GetStageMetersPerUnit(stage)

v_all = [v for p in paths for _, _, v in p]
v_min, v_max = min(v_all), max(v_all)
v_span = (v_max - v_min) or 1.0
c, s = math.cos(math.radians(args.yaw_deg)), math.sin(math.radians(args.yaw_deg))

root = Sdf.Path("/World/ALSPaths")
if stage.GetPrimAtPath(root):
    stage.RemovePrim(root)
UsdGeom.Xform.Define(stage, root)

# displayColor alone renders black in RTX (and the stage may have no lights), so bind an
# emissive preview material that reads the per-vertex displayColor
material = UsdShade.Material.Define(stage, root.AppendChild("SpeedColor"))
reader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("displayColor"))
reader.CreateIdAttr("UsdPrimvarReader_float3")
reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("displayColor")
reader_out = reader.CreateOutput("result", Sdf.ValueTypeNames.Float3)
surface = UsdShade.Shader.Define(stage, material.GetPath().AppendChild("surface"))
surface.CreateIdAttr("UsdPreviewSurface")
surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(reader_out)
surface.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(reader_out)
surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")

z = args.height / UsdGeom.GetStageMetersPerUnit(stage)
half_w = args.width / 2 / UsdGeom.GetStageMetersPerUnit(stage)

# flat ribbon mesh instead of BasisCurves: RTX's curve shaders take minutes to compile on
# first use and stall the app, plain meshes use the already-compiled pipeline
for i, samples in enumerate(paths):
    xy = [((x * c - y * s) * to_stage, (x * s + y * c) * to_stage) for x, y, _ in samples]
    pts, colors = [], []
    for j, (px, py) in enumerate(xy):
        # tangent from neighbours, ribbon edges offset along its left normal
        ax, ay = xy[max(j - 1, 0)]
        bx, by = xy[min(j + 1, len(xy) - 1)]
        tx, ty = bx - ax, by - ay
        norm = math.hypot(tx, ty) or 1.0
        nx, ny = -ty / norm * half_w, tx / norm * half_w
        pts += [Gf.Vec3f(px + nx, py + ny, z), Gf.Vec3f(px - nx, py - ny, z)]
        col = Gf.Vec3f(*speed_color((samples[j][2] - v_min) / v_span))
        colors += [col, col]

    mesh = UsdGeom.Mesh.Define(stage, root.AppendChild(f"path_{i}"))
    mesh.CreatePointsAttr(Vt.Vec3fArray(pts))
    # quad j: right_j, right_j+1, left_j+1, left_j (counter-clockwise seen from +Z)
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4] * (len(xy) - 1)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([k for j in range(len(xy) - 1) for k in (2 * j + 1, 2 * j + 3, 2 * j + 2, 2 * j)]))
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex).Set(Vt.Vec3fArray(colors))
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

print(f"[ALS] drew {len(paths)} paths ({sum(map(len, paths))} samples) from {args.path.name}")
print(f"[ALS] colour: v={v_min:.1f} (red) -> v={v_max:.1f} (green), in the path file's speed units")

if args.save:
    stage.Export(str(args.save.resolve()))
    print(f"[ALS] saved {args.save}")

if not args.headless:
    while app.is_running():
        app.update()
app.close()
