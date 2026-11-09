"""Draw ALS paths on the field as speed-coloured ribbons.

Used by drive_controller.py (the path autonomous is following) and
scripts/draw_als_path.py (standalone). Each path becomes a flat
UsdGeom.Mesh coloured by path speed v: red slow, yellow, green fast.
Field frame: JerryIO coordinates in inches, field-centre origin.

Needs pxr, so it only runs inside Isaac Sim / Kit.
"""

import math

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt


INCH = 0.0254
DEFAULT_ROOT = "/World/ALSPaths"

# slow -> fast: red, yellow, green (through yellow so the middle doesn't go
# brown)
SPEED_COLORS = [(0.9, 0.0, 0.0), (1.0, 0.85, 0.0), (0.0, 0.8, 0.0)]


def speed_color(t):
    """Colour for t in [0, 1] along SPEED_COLORS."""
    t = min(max(t, 0.0), 1.0) * (len(SPEED_COLORS) - 1)
    i = min(int(t), len(SPEED_COLORS) - 2)
    f = t - i
    return tuple(
        a + (b - a) * f for a, b in zip(SPEED_COLORS[i], SPEED_COLORS[i + 1])
    )


def draw_paths(stage, paths, height=0.02, width=0.02, yaw_deg=0.0,
               root=DEFAULT_ROOT, session=False):
    """Draw paths, replacing any drawn before under root.

    paths: one list of (x in, y in, v) samples per path.
    height, width: ribbon height above z=0 and width (m).
    yaw_deg: rotate about Z to line paths up with the field USD.
    session: draw in the stage's session layer, so saving the scene doesn't
    save the drawing into it.

    Returns (v_min, v_max) for the colour scale.
    """
    if session:
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            return _draw(stage, paths, height, width, yaw_deg, root)
    return _draw(stage, paths, height, width, yaw_deg, root)


def _draw(stage, paths, height, width, yaw_deg, root):
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    to_stage = INCH / meters_per_unit

    v_all = [v for path in paths for _, _, v in path]
    v_min, v_max = min(v_all), max(v_all)
    v_span = (v_max - v_min) or 1.0
    c = math.cos(math.radians(yaw_deg))
    s = math.sin(math.radians(yaw_deg))

    root = Sdf.Path(root)
    if stage.GetPrimAtPath(root):
        stage.RemovePrim(root)
    UsdGeom.Xform.Define(stage, root)

    # displayColor alone renders black in RTX (and the stage may have no
    # lights), so bind an emissive preview material that reads the
    # per-vertex displayColor
    material = UsdShade.Material.Define(stage, root.AppendChild("SpeedColor"))
    reader = UsdShade.Shader.Define(
        stage, material.GetPath().AppendChild("displayColor")
    )
    reader.CreateIdAttr("UsdPrimvarReader_float3")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("displayColor")
    reader_out = reader.CreateOutput("result", Sdf.ValueTypeNames.Float3)
    surface = UsdShade.Shader.Define(
        stage, material.GetPath().AppendChild("surface")
    )
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput(
        "diffuseColor", Sdf.ValueTypeNames.Color3f
    ).ConnectToSource(reader_out)
    surface.CreateInput(
        "emissiveColor", Sdf.ValueTypeNames.Color3f
    ).ConnectToSource(reader_out)
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    material.CreateSurfaceOutput().ConnectToSource(
        surface.ConnectableAPI(), "surface"
    )

    z = height / meters_per_unit
    half_w = width / 2 / meters_per_unit

    # flat ribbon mesh instead of BasisCurves: RTX's curve shaders take
    # minutes to compile on first use and stall the app, plain meshes use the
    # already-compiled pipeline
    for i, samples in enumerate(paths):
        xy = [
            ((x * c - y * s) * to_stage, (x * s + y * c) * to_stage)
            for x, y, _ in samples
        ]
        points, colors = [], []
        for j, (px, py) in enumerate(xy):
            # tangent from neighbours, ribbon edges offset along its left
            # normal
            ax, ay = xy[max(j - 1, 0)]
            bx, by = xy[min(j + 1, len(xy) - 1)]
            tx, ty = bx - ax, by - ay
            norm = math.hypot(tx, ty) or 1.0
            nx, ny = -ty / norm * half_w, tx / norm * half_w
            points += [
                Gf.Vec3f(px + nx, py + ny, z), Gf.Vec3f(px - nx, py - ny, z)
            ]
            color = Gf.Vec3f(*speed_color((samples[j][2] - v_min) / v_span))
            colors += [color, color]

        mesh = UsdGeom.Mesh.Define(stage, root.AppendChild(f"path_{i}"))
        mesh.CreatePointsAttr(Vt.Vec3fArray(points))
        # quad j: right_j, right_j+1, left_j+1, left_j (counter-clockwise
        # seen from +Z)
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4] * (len(xy) - 1)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray([
            k for j in range(len(xy) - 1)
            for k in (2 * j + 1, 2 * j + 3, 2 * j + 2, 2 * j)
        ]))
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex).Set(
            Vt.Vec3fArray(colors)
        )
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

    return v_min, v_max
