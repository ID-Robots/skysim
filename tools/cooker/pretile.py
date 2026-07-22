#!/usr/bin/env python3
"""Demo-map generator: deterministic toy city -> per-tile OBJ files for tile_cooker.

Map frame is ENU (x east, y north, z up), meters, origin at the vehicle home point.
Buildings are axis-aligned boxes on a 100 m grid with cross streets through the origin
(the origin block stays clear for takeoff/landing). Faces are CCW-wound viewed from
outside; tile_cooker converts ENU -> Jolt and relies on that outward winding
(mesh collision is one-sided, CLAUDE.md "Jolt-specific rules").

Usage: python3 tools/cooker/pretile.py <out_dir> [--tile-size 250] [--extent 250]
Real scanned-city input (decimate + tile a photogrammetry mesh) replaces this at M5;
the OBJ-per-tile contract stays the same.
"""
from __future__ import annotations

import argparse
import os
from typing import TextIO

FOOTPRINT_HALF = 20.0  # 40 m x 40 m buildings
GRID_STEP = 100.0


def building_height(i: int, j: int) -> float:
    """Deterministic pseudo-random height, 20..50 m."""
    return 20.0 + 10.0 * ((i * 7 + j * 13) % 4)


def buildings(extent: float) -> list[tuple[float, float, float]]:
    """(cx_east, cy_north, height) for every building inside +/-extent."""
    out = []
    n = int(extent // GRID_STEP)
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            if i == 0 or j == 0:  # cross streets through origin, keeps home clear
                continue
            out.append((i * GRID_STEP, j * GRID_STEP, building_height(i, j)))
    return out


def emit_box(f: TextIO, base: int, x0: float, x1: float, y0: float, y1: float,
             z0: float, z1: float) -> int:
    """Write one axis-aligned box: 8 vertices, 12 CCW-outward triangles. Returns new base."""
    verts = [
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),  # bottom ring
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),  # top ring
    ]
    # Quads with outward CCW winding (right-hand normal points out of the box).
    quads = [
        (1, 2, 6, 5),  # south face  (y = y0), normal -y
        (2, 3, 7, 6),  # east face   (x = x1), normal +x
        (3, 4, 8, 7),  # north face  (y = y1), normal +y
        (4, 1, 5, 8),  # west face   (x = x0), normal -x
        (5, 6, 7, 8),  # top         (z = z1), normal +z
        (4, 3, 2, 1),  # bottom      (z = z0), normal -z
    ]
    for v in verts:
        f.write(f"v {v[0]:.3f} {v[1]:.3f} {v[2]:.3f}\n")
    for a, b, c, d in quads:
        f.write(f"f {base + a} {base + b} {base + c}\n")
        f.write(f"f {base + a} {base + c} {base + d}\n")
    return base + 8


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir")
    ap.add_argument("--tile-size", type=float, default=250.0)
    ap.add_argument("--extent", type=float, default=250.0, help="map covers +/-extent meters")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    ts = args.tile_size
    n_tiles = int(2 * args.extent // ts)
    tile_of = lambda coord: min(int((coord + args.extent) // ts), n_tiles - 1)  # noqa: E731

    # Bucket buildings into tiles by center point.
    tiles: dict[tuple[int, int], list[tuple[float, float, float]]] = {}
    for cx, cy, h in buildings(args.extent):
        tiles.setdefault((tile_of(cx), tile_of(cy)), []).append((cx, cy, h))

    count = 0
    for (ti, tj), blds in sorted(tiles.items()):
        path = os.path.join(args.out_dir, f"tile_{ti}_{tj}.obj")
        with open(path, "w") as f:
            f.write(f"# skysim demo tile ({ti},{tj}) — ENU meters, CCW outward winding\n")
            base = 0
            for cx, cy, h in blds:
                base = emit_box(f, base,
                                cx - FOOTPRINT_HALF, cx + FOOTPRINT_HALF,
                                cy - FOOTPRINT_HALF, cy + FOOTPRINT_HALF,
                                0.0, h)
        print(f"{path}: {len(blds)} buildings")
        count += 1
    print(f"wrote {count} tiles to {args.out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
