#!/usr/bin/env python3
"""Build what fire_extinguisher_inspect.usda needs from Poly Haven's model.

Poly Haven's `korean_fire_extinguisher_01` is one mesh. Its damage variants need
to remove the hose, take the extinguisher out of its stand, and so on, so this
splits that mesh into parts along its connected pieces and writes them to
./fire_extinguisher/parts.usdc:

    /FireExtinguisher
        stand                    the floor stand
        extinguisher/cylinder    the body, its yellow strap
        extinguisher/head        valve, gauge, hose coupling
        extinguisher/levers      the carrying handle and the squeeze lever
        extinguisher/hose        hose and nozzle
        extinguisher/tag         the hanging inspection tag
        _materials               the model's own (UsdPreviewSurface; MaterialX blocked)

Each part keeps the model's points, UVs and normals, so the parts together render
exactly as the model does. It also writes a corroded copy of the body textures
(./fire_extinguisher/textures/*_corroded_2k.png): rust blotches and streaks over
the red paint, rougher where rusted; and the powder spilled by the
`discharged` variant (./fire_extinguisher/textures/powder_1k.png).

Run by fetch_fire_extinguisher.sh with Isaac Sim's bundled Python (pxr, numpy
and PIL). Deterministic, and a file whose contents come out the same is left
untouched: Isaac holds parts.usdc memory-mapped, and replacing it under a
running Isaac crashed it (2026-11-10).
"""
import io
import os
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

HERE = Path(__file__).resolve().parent / "fire_extinguisher"
NAME = "korean_fire_extinguisher_01"
SRC = HERE / f"{NAME}_2k.usdc"
OUT = HERE / "parts.usdc"
TEX = HERE / "textures"


def replace_if_changed(path: Path, data: bytes) -> bool:
    """Write `data` to `path` unless it already holds exactly that; True if written."""
    if path.exists() and path.read_bytes() == data:
        return False
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)
    return True


def save_png(path: Path, img: Image.Image) -> str:
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return "wrote" if replace_if_changed(path, buf.getvalue()) else "kept"


def components(counts, indices, n_points):
    """Connected pieces of the mesh: faces that share a vertex are one piece."""
    parent = np.arange(n_points)

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    offsets = np.concatenate([[0], np.cumsum(counts)])
    for f in range(len(counts)):
        verts = indices[offsets[f]:offsets[f + 1]]
        root = find(verts[0])
        for v in verts[1:]:
            rv = find(v)
            if rv != root:
                parent[rv] = root
    roots = np.array([find(indices[offsets[f]]) for f in range(len(counts))])
    return [np.flatnonzero(roots == r) for r in np.unique(roots)], offsets


def classify(faces, points, verts, material):
    """Which part a piece belongs to, from where it sits on the model.

    The model stands on z=0 with its gauge facing -y, the hose down its -x side
    and the levers reaching out over +x. Sizes are in metres.
    """
    lo, hi = points[verts].min(0), points[verts].max(0)
    if material == "paper":
        return "tag"
    if hi[2] < 0.13 and hi[0] - lo[0] > 0.25:
        return "stand"
    if hi[0] < -0.035 and lo[2] < 0.55:
        return "hose"                  # hose, its end fitting, the nozzle
    if lo[2] < 0.5:
        return "cylinder"              # the body, the strap, a fitting on it
    if hi[0] - lo[0] > 0.12 and lo[2] > 0.55:
        return "levers"                # the two long black levers
    return "head"


def write_part(stage, path, src_mesh, faces, offsets, counts, indices, points, normals, st,
               face_material, materials):
    corners = np.concatenate([np.arange(offsets[f], offsets[f + 1]) for f in faces])
    used, remap = np.unique(indices[corners], return_inverse=True)
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points[used].astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(counts[faces].astype(np.int32)))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(remap.astype(np.int32)))
    mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(normals[corners].astype(np.float32)))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
    uv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                 UsdGeom.Tokens.faceVarying)
    uv.Set(Vt.Vec2fArray.FromNumpy(st[corners].astype(np.float32)))
    mesh.CreateSubdivisionSchemeAttr(src_mesh.GetSubdivisionSchemeAttr().Get())
    mesh.CreateOrientationAttr(src_mesh.GetOrientationAttr().Get())
    mesh.CreateDoubleSidedAttr(src_mesh.GetDoubleSidedAttr().Get())
    lo, hi = points[used].min(0), points[used].max(0)
    mesh.CreateExtentAttr([Gf.Vec3f(*map(float, lo)), Gf.Vec3f(*map(float, hi))])
    names = face_material[faces]
    binding = UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
    if len(set(names)) == 1:
        binding.Bind(materials[names[0]])
        return
    for name in sorted(set(names)):
        subset = UsdGeom.Subset.CreateGeomSubset(
            mesh, name, UsdGeom.Tokens.face, Vt.IntArray.FromNumpy(np.flatnonzero(names == name).astype(np.int32)),
            UsdShade.Tokens.materialBind)
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(materials[name])


def build_parts():
    src = Usd.Stage.Open(str(SRC))
    mesh = UsdGeom.Mesh(src.GetPrimAtPath(f"/{NAME}/{NAME}/_2_low"))
    points = np.array(mesh.GetPointsAttr().Get(), dtype=np.float64)
    counts = np.array(mesh.GetFaceVertexCountsAttr().Get())
    indices = np.array(mesh.GetFaceVertexIndicesAttr().Get())
    normals = np.array(mesh.GetNormalsAttr().Get())
    st = np.array(UsdGeom.PrimvarsAPI(mesh).GetPrimvar("st").ComputeFlattened())
    if len(normals) != len(indices) or len(st) != len(indices):
        sys.exit("expected face-varying normals and UVs; has Poly Haven's export changed?")
    face_material = np.empty(len(counts), dtype=object)
    for subset in UsdGeom.Subset.GetAllGeomSubsets(mesh):
        face_material[np.array(subset.GetIndicesAttr().Get())] = subset.GetPrim().GetName().rsplit("_", 1)[-1]

    pieces, offsets = components(counts, indices, len(points))
    parts = {}
    for faces in pieces:
        verts = np.unique(np.concatenate([indices[offsets[f]:offsets[f + 1]] for f in faces]))
        part = classify(faces, points, verts, face_material[faces[0]])
        parts.setdefault(part, []).append(faces)

    scratch = OUT.with_name("parts.new.usdc")
    scratch.unlink(missing_ok=True)
    out = Usd.Stage.CreateNew(str(scratch))
    UsdGeom.SetStageUpAxis(out, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(out, 1.0)
    root = UsdGeom.Xform.Define(out, "/FireExtinguisher")
    out.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetMetadata("comment", f"Generated by make_fire_extinguisher.py from {SRC.name}.")
    mats = out.DefinePrim("/FireExtinguisher/_materials", "Scope")
    mats.GetReferences().AddReference(f"./{SRC.name}", f"/{NAME}/_materials")
    materials = {}
    for kind in ("body", "glass", "paper"):
        prim = out.OverridePrim(f"/FireExtinguisher/_materials/{NAME}_{kind}")
        # RTX prefers the MaterialX network and renders it untextured; keep the
        # UsdPreviewSurface one.
        prim.CreateAttribute("outputs:mtlx:surface", Sdf.ValueTypeNames.Token).SetConnections([])
        materials[kind] = UsdShade.Material(prim)
    UsdGeom.Xform.Define(out, "/FireExtinguisher/extinguisher")
    report = []
    for part in ("stand", "cylinder", "head", "levers", "hose", "tag"):
        if part not in parts:
            sys.exit(f"no piece of the model was classified as {part!r}; has the model changed?")
        faces = np.sort(np.concatenate(parts[part]))
        path = "/FireExtinguisher/stand" if part == "stand" else f"/FireExtinguisher/extinguisher/{part}"
        write_part(out, path, mesh, faces, offsets, counts, indices, points, normals, st,
                   face_material, materials)
        report.append(f"{part} {len(faces)}")
    out.GetRootLayer().Save()
    del out
    data = scratch.read_bytes()
    scratch.unlink()
    verb = "wrote" if replace_if_changed(OUT, data) else "kept (unchanged)"
    print(f"{verb} {OUT} ({', '.join(report)} faces)")


def noise(shape, scale, rng, octaves=4):
    """Fractal value noise in [0, 1], `scale` pixels per cell at the coarsest octave."""
    h, w = shape
    total, amp, norm = np.zeros(shape), 1.0, 0.0
    for o in range(octaves):
        cell = max(scale / 2 ** o, 2)
        grid = rng.random((int(h / cell) + 2, int(w / cell) + 2)).astype(np.float32)
        total += amp * np.asarray(Image.fromarray(grid).resize((w, h), Image.BICUBIC))
        norm += amp
        amp *= 0.5
    total /= norm
    return (total - total.min()) / (total.max() - total.min())


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0, 1)
    return t * t * (3 - 2 * t)


def build_corroded():
    rng = np.random.default_rng(7)
    diff = Image.open(TEX / f"{NAME}_body_diff_2k.png")
    rgba = np.asarray(diff).astype(np.float32) / 255
    rgb = rgba[..., :3]
    shape = rgb.shape[:2]
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    paint = smoothstep(1.2, 1.8, r / (np.maximum(g, b) + 0.02)) * smoothstep(0.12, 0.25, r)
    blotch = noise(shape, 160, rng)
    # Streaks run down the texture's v axis, which is the body's height.
    streak = np.asarray(Image.fromarray(noise((shape[0] // 16, shape[1]), 24, rng).astype(np.float32))
                        .resize(shape[::-1], Image.BICUBIC))
    # Most of the paint: a robot's camera sees a 0.2 m body as ~80 px, where
    # light rust read as "staining" to the VLM (2026-11-10).
    cover = np.maximum(smoothstep(0.36, 0.46, blotch), 0.9 * smoothstep(0.55, 0.65, streak)) * paint
    tone = noise(shape, 40, rng)[..., None]
    rust = (1 - tone) * np.array([0.30, 0.14, 0.05]) + tone * np.array([0.62, 0.33, 0.10])
    pits = smoothstep(0.70, 0.85, noise(shape, 12, rng))[..., None] * 0.5
    rust = rust * (1 - pits)
    # What paint is left is sun-faded and dull.
    faded = rgb * 0.75 + 0.25 * rgb.mean(-1, keepdims=True) + 0.04
    out = faded * (1 - cover[..., None]) + rust * cover[..., None]
    rgba = np.concatenate([out, rgba[..., 3:]], -1) if rgba.shape[-1] == 4 else out
    v1 = save_png(TEX / f"{NAME}_body_diff_corroded_2k.png",
                  Image.fromarray((np.clip(rgba, 0, 1) * 255).astype(np.uint8)))
    rough_img = Image.open(TEX / f"{NAME}_body_rough_2k.png")       # 16-bit grey
    rough = np.asarray(rough_img).astype(np.float32) / (255 if rough_img.mode == "L" else 65535)
    rough = rough * (1 - cover) + np.maximum(rough, 0.9) * cover
    v2 = save_png(TEX / f"{NAME}_body_rough_corroded_2k.png", Image.fromarray((rough * 255).astype(np.uint8)))
    print(f"{v1}/{v2} {TEX}/{NAME}_body_{{diff,rough}}_corroded_2k.png "
          f"({cover.mean():.0%} of the texture rusted)")


# The powder decal's floor area, x and y ranges in metres: the quad that
# fire_extinguisher_inspect.usda's _Powder lays down. It stops short of the
# wall the hospital stage stands the extinguisher against (+x 0.23).
POWDER_EXTENT = ((-1.1, 0.2), (-0.7, 0.8))
# Where the drift is centred and which way it runs: out of the hose side (-x)
# and back along the stand, to the left of it in the robot's view and as far
# from the camera as the extinguisher. In front of it, it was below the frame
# of a camera 1.5 m off (the floor is in view only from 1.3 m out).
POWDER_CENTRE = (-0.30, 0.10)
POWDER_DIRECTION = (-0.75, 0.66)


def build_powder(size=1024):
    """Dry-chemical powder spilled in front of the stand, as an RGBA cut-out.

    A drift that is densest beside the stand and runs out from its hose side,
    a thinner haze around it, and clumps.
    The alpha is dithered per pixel, so the cut-out's edge is grainy like
    powder rather than a hard outline.
    """
    rng = np.random.default_rng(11)
    (x0, x1), (y0, y1) = POWDER_EXTENT
    xs = np.linspace(x0, x1, size)                     # not square in metres: u and v
    ys = np.linspace(y1, y0, size)                     # stretch; image row 0 = +y edge (v = 1)
    X, Y = np.meshgrid(xs, ys)
    d = np.array(POWDER_DIRECTION) / np.hypot(*POWDER_DIRECTION)
    cx0, cy0 = POWDER_CENTRE
    rx, ry = X - cx0, Y - cy0                           # from the drift's centre
    along = rx * d[0] + ry * d[1]
    across = -rx * d[1] + ry * d[0]
    drift = np.exp(-(((along - 0.12) / 0.36) ** 2 + (across / 0.22) ** 2))
    haze = 0.45 * np.exp(-((along / 0.62) ** 2 + (across / 0.42) ** 2))
    density = np.maximum(drift, haze) * (0.45 + 0.55 * noise((size, size), 90, rng))
    for _ in range(40):                                 # clumps
        a = rng.normal(0, 0.9)
        r = rng.uniform(0.25, 0.7)
        cx = cx0 + r * (d[0] * np.cos(a) - d[1] * np.sin(a))
        cy = cy0 + r * (d[1] * np.cos(a) + d[0] * np.sin(a))
        s = rng.uniform(0.012, 0.04)
        density = np.maximum(density, 0.8 * np.exp(-(((X - cx) ** 2 + (Y - cy) ** 2) / s ** 2)))
    alpha = (rng.random((size, size)) < smoothstep(0.12, 0.55, density)).astype(np.float32)
    shade = 0.92 + 0.08 * noise((size, size), 30, rng)
    rgb = np.stack([0.90 * shade, 0.88 * shade, 0.79 * shade], -1)
    rgba = np.concatenate([rgb, alpha[..., None]], -1)
    verb = save_png(TEX / "powder_1k.png", Image.fromarray((rgba * 255).astype(np.uint8)))
    print(f"{verb} {TEX}/powder_1k.png ({alpha.mean():.0%} covered)")


if __name__ == "__main__":
    if not SRC.exists():
        sys.exit(f"{SRC} is missing: run fetch_fire_extinguisher.sh")
    build_parts()
    build_corroded()
    build_powder()
