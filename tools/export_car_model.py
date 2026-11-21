#!/usr/bin/env python3
"""Bake RaceCar.glb (CC0, OpenGameArt/mabaci) into a generated C++ header.

The GLB stores the body with a mirroring node transform and the four wheels as
separate meshes. This script:
  * bakes node transforms into the vertices,
  * splits body / wheels and centres the wheel geometry on its own origin,
  * scales to game units (wheel radius 0.33 m) and centres the car on its wheelbase,
  * samples the livery texture (RaceCar.jpg) into per-vertex grayscale so the
    per-car team tint keeps the paint detail,
  * emits src/vehicle/CarModelData.h with plain arrays consumed by Vehicle::Render.

Regenerate with:  python tools/export_car_model.py
"""

import json
import struct
import sys
from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
GLB_PATH = ROOT / "assets" / "models" / "racecar" / "RaceCar.glb"
TEX_PATH = ROOT / "assets" / "models" / "racecar" / "RaceCar.jpg"
OUT_PATH = ROOT / "src" / "vehicle" / "CarModelData.h"

WHEEL_RADIUS = 0.33          # must match VehicleParams::wheelRadius
SCALE = 1.5                  # visual scale of the whole car

COMP_SIZE = {5120: 1, 5121: 1, 5122: 2, 5123: 2, 5125: 4, 5126: 4}
COMP_FMT = {5120: "b", 5121: "B", 5122: "h", 5123: "H", 5125: "I", 5126: "f"}
NUM_COMPONENTS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


def read_glb(path):
    data = path.read_bytes()
    magic, _ver, length = struct.unpack_from("<III", data, 0)
    assert magic == 0x46546C67, "not a GLB file"
    off = 12
    gltf, bin_chunk = None, None
    while off < length:
        clen, ctype = struct.unpack_from("<II", data, off)
        chunk = data[off + 8 : off + 8 + clen]
        if ctype == 0x4E4F534A:
            gltf = json.loads(chunk)
        else:
            bin_chunk = chunk
        off += 8 + clen
    return gltf, bin_chunk


def accessor_values(gltf, bin_chunk, index):
    acc = gltf["accessors"][index]
    view = gltf["bufferViews"][acc["bufferView"]]
    comp_size = COMP_SIZE[acc["componentType"]]
    ncomp = NUM_COMPONENTS[acc["type"]]
    stride = view.get("byteStride") or comp_size * ncomp
    base = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
    fmt = COMP_FMT[acc["componentType"]]
    out = []
    for i in range(acc["count"]):
        vals = struct.unpack_from("<" + fmt * ncomp, bin_chunk, base + i * stride)
        out.append([float(v) for v in vals])
    return out


def node_matrix(node):
    t = node.get("translation", [0.0, 0.0, 0.0])
    r = node.get("rotation", [0.0, 0.0, 0.0, 1.0])  # x, y, z, w
    s = node.get("scale", [1.0, 1.0, 1.0])
    x, y, z, w = r
    # rotation matrix (column j = image of basis j)
    rot = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    # M = T * R * S  ->  columns scaled by s
    m = [[rot[i][j] * s[j] for j in range(3)] for i in range(3)]
    det = (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )
    return m, t, det


def transform_point(m, t, p):
    return [
        m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2] + t[0],
        m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2] + t[1],
        m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2] + t[2],
    ]


def extract_mesh(gltf, bin_chunk, mesh_index):
    """Return (positions, uvs, indices) with node transform baked in later."""
    prim = gltf["meshes"][mesh_index]["primitives"][0]
    positions = accessor_values(gltf, bin_chunk, prim["attributes"]["POSITION"])
    if "TEXCOORD_0" in prim["attributes"]:
        uvs = accessor_values(gltf, bin_chunk, prim["attributes"]["TEXCOORD_0"])
    else:
        uvs = [[0.0, 0.0] for _ in positions]
    if "indices" in prim:
        idx = [int(v[0]) for v in accessor_values(gltf, bin_chunk, prim["indices"])]
    else:
        idx = list(range(len(positions)))
    return positions, uvs, idx


def recompute_normals(positions, indices):
    normals = [[0.0, 0.0, 0.0] for _ in positions]
    for i in range(0, len(indices), 3):
        a, b, c = indices[i], indices[i + 1], indices[i + 2]
        pa, pb, pc = positions[a], positions[b], positions[c]
        u = [pb[j] - pa[j] for j in range(3)]
        v = [pc[j] - pa[j] for j in range(3)]
        n = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ]
        for p in (a, b, c):
            for j in range(3):
                normals[p][j] += n[j]
    for n in normals:
        length = (n[0] ** 2 + n[1] ** 2 + n[2] ** 2) ** 0.5
        if length > 1e-12:
            for j in range(3):
                n[j] /= length
    return normals


def bounds(positions):
    lo = [min(p[i] for p in positions) for i in range(3)]
    hi = [max(p[i] for p in positions) for i in range(3)]
    return lo, hi


def main():
    gltf, bin_chunk = read_glb(GLB_PATH)

    nodes = {n.get("name"): n for n in gltf["nodes"]}
    body_node = nodes["body"]
    wheel_names = ["RearLeftTire", "FrontLeftTire", "RearRightTire", "FrontRightTire"]

    meshes = {}
    for name, node in [("body", body_node)] + [(n, nodes[n]) for n in wheel_names]:
        m, t, det = node_matrix(node)
        positions, uvs, indices = extract_mesh(gltf, bin_chunk, node["mesh"])
        baked = [transform_point(m, t, p) for p in positions]
        if det < 0:
            # Mirroring transform inverts winding: (a,b,c) -> (a,c,b).
            fixed = []
            for i in range(0, len(indices), 3):
                fixed += [indices[i], indices[i + 2], indices[i + 1]]
            indices = fixed
        meshes[name] = (baked, uvs, indices)

    # Wheel-base centre in baked space (front/rear wheel mean z).
    wheel_z = []
    wheel_centres = {}
    for name in wheel_names:
        positions, _, _ = meshes[name]
        lo, hi = bounds(positions)
        centre = [(lo[i] + hi[i]) * 0.5 for i in range(3)]
        wheel_centres[name] = centre
        wheel_z.append(centre[2])
    z_mid = (min(wheel_z) + max(wheel_z)) * 0.5

    # Game scale: make the exported wheel radius match WHEEL_RADIUS.
    wheel_lo, wheel_hi = bounds(meshes[wheel_names[0]][0])
    wheel_radius_model = (wheel_hi[1] - wheel_lo[1]) * 0.5
    scale = WHEEL_RADIUS / wheel_radius_model

    def place(positions, centre_z_shift):
        return [[p[0] * scale, p[1] * scale, (p[2] - z_mid) * scale + centre_z_shift]
                for p in positions]

    out = {}
    positions, uvs, indices = meshes["body"]
    body_pos = place(positions, 0.0)
    out["body"] = (body_pos, uvs, indices)

    positions, uvs, indices = meshes[wheel_names[0]]
    c = wheel_centres[wheel_names[0]]
    wheel_pos = [[(p[0] - c[0]) * scale, (p[1] - c[1]) * scale, (p[2] - c[2]) * scale]
                 for p in positions]
    out["wheel"] = (wheel_pos, uvs, indices)

    # Wheel anchor positions in model space (ground plane at y = 0).
    anchors = {}
    for name in wheel_names:
        c = wheel_centres[name]
        anchors[name] = (c[0] * scale, c[1] * scale, (c[2] - z_mid) * scale)

    # Emit header.
    def fmt_floats(vals, per_line=9):
        lines = []
        row = []
        for v in vals:
            row.append(f"{v:.6f}f")
            if len(row) == per_line:
                lines.append(", ".join(row))
                row = []
        if row:
            lines.append(", ".join(row))
        return ",\n    ".join(lines)

    # Livery bake: sample the texture per vertex and keep its luminance as a
    # grayscale value. The renderer multiplies this by the per-car team tint,
    # so paint detail survives while each car keeps its colour.
    tex = Image.open(TEX_PATH).convert("RGB")
    tw, th = tex.size
    texels = tex.load()

    def bake_colors(uvs):
        out_bytes = []
        for u, v in uvs:
            x = min(tw - 1, max(0, int(u * tw)))
            y = min(th - 1, max(0, int(v * th)))
            r, g, b = texels[x, y]
            lum = (0.299 * r + 0.587 * g + 0.114 * b) / 255.0
            gray = int(min(255, max(0, (0.35 + 0.65 * lum) * 255)))
            out_bytes += [gray, gray, gray, 255]
        return out_bytes

    def mesh_block(cpp_name, positions, uvs, indices):
        pos_flat = [c for p in positions for c in p]
        normals = recompute_normals(positions, indices)
        nrm_flat = [c for n in normals for c in n]
        colors = bake_colors(uvs)
        return (
            f"const float k{cpp_name}Positions[] = {{\n    {fmt_floats(pos_flat)}\n}};\n\n"
            f"const float k{cpp_name}Normals[] = {{\n    {fmt_floats(nrm_flat)}\n}};\n\n"
            f"const uint8_t k{cpp_name}Colors[] = {{\n    "
            + ", ".join(str(c) for c in colors)
            + "\n};\n\n"
            f"const uint16_t k{cpp_name}Indices[] = {{\n    "
            + ", ".join(str(i) for i in indices)
            + "\n};\n"
        )

    body_pos, body_uvs, body_idx = out["body"]
    wheel_pos, wheel_uvs, wheel_idx = out["wheel"]

    header = []
    header.append("// Generated by tools/export_car_model.py — do not edit.")
    header.append("// Source: assets/models/racecar/RaceCar.glb (CC0, OpenGameArt / mabaci).")
    header.append("#pragma once")
    header.append("")
    header.append("#include <cstdint>")
    header.append("")
    header.append("namespace apex {")
    header.append("namespace car_model {")
    header.append("")
    header.append("struct MeshBlob {")
    header.append("    const float* positions;")
    header.append("    const float* normals;")
    header.append("    const uint8_t* colors;   // RGBA per vertex, livery baked as grayscale")
    header.append("    const uint16_t* indices;")
    header.append("    int vertexCount;")
    header.append("    int indexCount;")
    header.append("};")
    header.append("")
    header.append(mesh_block("Body", body_pos, body_uvs, body_idx))
    header.append("")
    header.append(mesh_block("Wheel", wheel_pos, wheel_uvs, wheel_idx))
    header.append("")
    header.append(f"inline constexpr MeshBlob kBody{{kBodyPositions, kBodyNormals, kBodyColors, "
                  f"kBodyIndices, {len(body_pos)}, {len(body_idx)}}};")
    header.append(f"inline constexpr MeshBlob kWheel{{kWheelPositions, kWheelNormals, kWheelColors, "
                  f"kWheelIndices, {len(wheel_pos)}, {len(wheel_idx)}}};")
    header.append("")
    header.append("// Wheel anchors in model space (origin = wheelbase centre on the ground).")
    for name in wheel_names:
        a = anchors[name]
        key = name.replace("Tire", "")
        header.append(f"inline constexpr float kAnchor{key}[3] = {{"
                      f"{a[0]:.6f}f, {a[1]:.6f}f, {a[2]:.6f}f}};")
    header.append("")
    header.append("}  // namespace car_model")
    header.append("}  // namespace apex")

    OUT_PATH.write_text("\n".join(header) + "\n", encoding="utf-8")

    body_lo, body_hi = bounds(body_pos)
    print(f"scale={scale:.4f}  wheel_radius={wheel_radius_model * scale:.4f}")
    print(f"body bounds lo={[round(v, 3) for v in body_lo]} hi={[round(v, 3) for v in body_hi]}")
    for name in wheel_names:
        print(f"anchor {name}: {[round(v, 3) for v in anchors[name]]}")
    print(f"body: {len(body_pos)} verts / {len(body_idx) // 3} tris")
    print(f"wheel: {len(wheel_pos)} verts / {len(wheel_idx) // 3} tris")
    print(f"wrote {OUT_PATH}")


if __name__ == "__main__":
    sys.exit(main())
