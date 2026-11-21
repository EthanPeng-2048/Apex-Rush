#!/usr/bin/env python3
"""Authoring tool for the test circuit map data.

track.csv stores SPARSE authored waypoints (the real circuit data form: a
centerline polyline with per-side runoff bands). The engine smooths it with
Catmull-Rom at load time — the layout is DATA, not runtime generation.

Real circuit data later plugs into the same CSV format via a converter
(GPS/sim polyline -> track.csv), no engine changes needed.

Usage: python tools/make_test_track.py   (writes assets/maps/test_circuit/)
"""
import math
import os

# Hand-authored centerline waypoints (closed loop). Columns:
# x, z, y, width, grass_left, gravel_left, grass_right, gravel_right
# "right" follows the running direction; big runoff sits outside fast corners.
WAYPOINTS = [
    #  x      z      y    width  gl    vl    gr    vr
    (0.0,    0.0,   0.0, 15.0, 10.0,  6.0, 10.0,  6.0),   # start line
    (0.0,  640.0,   1.0, 15.0, 10.0,  6.0, 10.0,  6.0),   # end of main straight
    (150.0, 830.0,  1.6, 13.0, 24.0, 20.0,  8.0,  0.0),   # T1 right (wide outside L)
    (430.0, 870.0,  2.0, 13.0, 12.0,  8.0, 12.0,  8.0),
    (580.0, 700.0,  2.8, 12.5, 10.0,  0.0, 20.0, 14.0),   # esse left (outside R)
    (520.0, 470.0,  3.4, 12.5, 20.0, 14.0, 10.0,  0.0),   # esse right (outside L)
    (720.0, 300.0,  4.2, 14.0, 22.0, 18.0, 10.0,  0.0),   # fast right (outside L)
    (880.0,  40.0,  5.0, 14.5, 12.0,  8.0, 12.0,  8.0),
    (840.0,-290.0,  5.2, 14.0, 12.0,  8.0, 12.0,  8.0),   # back straight end
    (620.0,-430.0,  4.4, 13.0, 22.0, 18.0, 10.0,  0.0),   # fast right 2 (outside L)
    (380.0,-490.0,  3.2, 13.0, 12.0,  8.0, 12.0,  8.0),
    (230.0,-370.0,  2.4, 12.0, 26.0, 22.0,  8.0,  0.0),   # hairpin (wide outside L)
    (310.0,-180.0,  1.6, 13.0, 10.0,  0.0, 20.0, 16.0),   # slow left (outside R)
    (170.0, -60.0,  0.8, 13.5, 12.0,  8.0, 12.0,  8.0),
    # Final corner onto the main straight: authored on a ~60 m radius arc
    # (entry heading west, exit heading north at the start line). Sparse
    # points here used to let the spline fold a 5 m hook.
    (105.0, -60.0,  0.5, 13.5, 14.0, 10.0, 12.0,  8.0),
    ( 60.0, -60.0,  0.3, 13.5, 16.0, 12.0, 12.0,  8.0),   # corner entry
    ( 18.0, -42.0,  0.1, 14.0, 20.0, 16.0, 10.0,  0.0),   # apex -> exit north
]

TREE_STEP = 1
OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "assets", "maps", "test_circuit")


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    n = len(WAYPOINTS)

    # Lap length estimate for tree placement (polygon perimeter).
    perim = 0.0
    for i in range(n):
        a, b = WAYPOINTS[i], WAYPOINTS[(i + 1) % n]
        perim += math.hypot(b[0] - a[0], b[1] - a[1])

    with open(os.path.join(OUT_DIR, "track.csv"), "w", encoding="utf-8") as f:
        f.write("# x, z, y, width, grass_left, gravel_left, grass_right, gravel_right\n")
        f.write("# sparse authored waypoints; engine smooths with Catmull-Rom\n")
        for w in WAYPOINTS:
            f.write(",".join(f"{v:.2f}" for v in w) + "\n")

    # Terrain height posts around the loop (deterministic authored undulation).
    xs = [w[0] for w in WAYPOINTS]
    zs = [w[1] for w in WAYPOINTS]
    min_x, max_x = min(xs) - 150.0, max(xs) + 150.0
    min_z, max_z = min(zs) - 150.0, max(zs) + 150.0
    with open(os.path.join(OUT_DIR, "terrain.csv"), "w", encoding="utf-8") as f:
        f.write("# x, z, y   (height posts, interpolated by the engine)\n")
        step = 70.0
        gx = min_x
        while gx <= max_x:
            gz = min_z
            while gz <= max_z:
                y = (1.8 * math.sin(gx / 160.0) * math.cos(gz / 200.0)
                     + 2.4 * math.sin((gx + gz) / 280.0))
                f.write(f"{gx:.1f},{gz:.1f},{y:.2f}\n")
                gz += step
            gx += step

    # Trees outside the runoff bands, alternating sides (dense polyline walk).
    # The engine snaps trees to the terrain height; y here is a hint only.
    with open(os.path.join(OUT_DIR, "decor.csv"), "w", encoding="utf-8") as f:
        f.write("# type, x, z, y, scale\n")
        for i in range(n):
            a, b = WAYPOINTS[i], WAYPOINTS[(i + 1) % n]
            dx, dz = b[0] - a[0], b[1] - a[1]
            seg = math.hypot(dx, dz)
            ux, uz = dx / max(1e-9, seg), dz / max(1e-9, seg)
            rx, rz = uz, -ux
            steps = max(1, int(seg / 14.0))
            for k in range(steps):
                t = (k + 0.5) / steps
                px, pz = a[0] + dx * t, a[1] + dz * t
                side = 1.0 if (k + i) % 2 == 0 else -1.0
                # Bands interpolate along the segment; the tree sits beyond
                # the outer band edge (road/2 + grass + gravel) plus a gap.
                w = a[3] + (b[3] - a[3]) * t
                gl = a[4] + (b[4] - a[4]) * t
                vl = a[5] + (b[5] - a[5]) * t
                gr = a[6] + (b[6] - a[6]) * t
                vr = a[7] + (b[7] - a[7]) * t
                edge = w * 0.5 + (gr + vr if side > 0 else gl + vl)
                off = edge + 12.0 + ((k + i) % 4) * 7.0
                scale = 0.9 + ((k + i) % 3) * 0.25
                f.write(f"tree,{px + rx * side * off:.1f},{pz + rz * side * off:.1f},"
                        f"0.00,{scale:.2f}\n")

    print(f"wrote {OUT_DIR}: {n} waypoints, lap ~{perim:.0f} m")


if __name__ == "__main__":
    main()
