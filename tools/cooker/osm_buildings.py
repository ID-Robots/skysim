#!/usr/bin/env python3
"""Real-city map generator: OSM building footprints -> per-tile OBJ files for tile_cooker.

This is the production counterpart to pretile.py's synthetic grid. It pulls real
building footprints and heights from OpenStreetMap (free and redistributable —
ODbL) for a radius around a WGS84 anchor, projects them into the map frame, and
extrudes each footprint into a closed prism.

Map frame matches pretile.py: ENU (x east, y north, z up), meters, origin at the
anchor — which must be the same point the vehicle is spawned at (ArduPilot
``--home``), or the simulated buildings will not line up with what the operator
sees on the map. The anchor is recorded in ``anchor.json`` next to the OBJ files
so the cook step can stamp it into index.json instead of leaving the tile set
un-georeferenced.

Faces are CCW-wound viewed from outside; tile_cooker converts ENU -> Jolt and
relies on that outward winding (mesh collision is one-sided).

Heights, in order of preference:
  1. ``height`` / ``building:height`` tag (meters)
  2. ``building:levels`` x LEVEL_HEIGHT_M
  3. DEFAULT_HEIGHT_M

Usage:
  python3 tools/cooker/osm_buildings.py out_dir --lat 42.1403890 --lon 24.7645490 \
      --radius 5000 [--tile-size 250] [--cache buildings.json]

Notes / limits:
  - Only the outer ring of each building is used; courtyards are filled in. For
    collision purposes a solid building is the conservative choice.
  - Buildings straddling a tile boundary are assigned whole to the tile holding
    their centroid, so a tile's geometry can extend slightly past its nominal
    bounds. The streamer keeps neighbouring tiles resident, so this is safe.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import urllib.error
import urllib.parse
import urllib.request
from typing import TextIO

OVERPASS_URLS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
]
LEVEL_HEIGHT_M = 3.0
DEFAULT_HEIGHT_M = 8.0
MIN_HEIGHT_M = 2.0
EARTH_RADIUS_M = 6378137.0


# ---------------------------------------------------------------------------- fetch


def build_query(lat: float, lon: float, radius_m: float) -> str:
    return f"""
[out:json][timeout:300];
(
  way["building"](around:{radius_m},{lat},{lon});
  relation["building"](around:{radius_m},{lat},{lon});
);
out body geom;
""".strip()


def fetch_osm(lat: float, lon: float, radius_m: float) -> dict:
    """Query Overpass, falling back to the mirror if the primary is busy."""
    query = build_query(lat, lon, radius_m)
    data = urllib.parse.urlencode({"data": query}).encode()
    last_error: Exception | None = None

    for url in OVERPASS_URLS:
        try:
            print(f"querying {url} ({radius_m:.0f} m around {lat},{lon}) ...")
            request = urllib.request.Request(url, data=data, headers={"User-Agent": "skysim-cooker/1.0"})
            with urllib.request.urlopen(request, timeout=300) as response:
                return json.loads(response.read().decode())
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as e:
            print(f"  {url} failed: {e}")
            last_error = e

    raise SystemExit(f"all Overpass endpoints failed: {last_error}")


# ---------------------------------------------------------------------------- geometry


def parse_height(tags: dict) -> float:
    """Metres, from whichever tag OSM happens to carry for this building."""
    for key in ("height", "building:height"):
        raw = tags.get(key)
        if raw:
            try:
                # tags look like "12", "12.5", "12 m"
                return max(MIN_HEIGHT_M, float(str(raw).split()[0].replace(",", ".")))
            except ValueError:
                pass

    for key in ("building:levels", "levels"):
        raw = tags.get(key)
        if raw:
            try:
                return max(MIN_HEIGHT_M, float(str(raw).replace(",", ".")) * LEVEL_HEIGHT_M)
            except ValueError:
                pass

    return DEFAULT_HEIGHT_M


def to_enu(lat: float, lon: float, anchor_lat: float, anchor_lon: float) -> tuple[float, float]:
    """WGS84 -> local ENU metres about the anchor (equirectangular; exact enough at city scale)."""
    lat_rad = math.radians(anchor_lat)
    east = math.radians(lon - anchor_lon) * EARTH_RADIUS_M * math.cos(lat_rad)
    north = math.radians(lat - anchor_lat) * EARTH_RADIUS_M
    return east, north


def signed_area(poly: list[tuple[float, float]]) -> float:
    """Positive when the ring is counter-clockwise viewed from +z."""
    total = 0.0
    for i, (x0, y0) in enumerate(poly):
        x1, y1 = poly[(i + 1) % len(poly)]
        total += x0 * y1 - x1 * y0
    return total / 2.0


def _point_in_triangle(px, py, ax, ay, bx, by, cx, cy) -> bool:
    d1 = (px - bx) * (ay - by) - (ax - bx) * (py - by)
    d2 = (px - cx) * (by - cy) - (bx - cx) * (py - cy)
    d3 = (px - ax) * (cy - ay) - (cx - ax) * (py - ay)
    has_neg = d1 < 0 or d2 < 0 or d3 < 0
    has_pos = d1 > 0 or d2 > 0 or d3 > 0
    return not (has_neg and has_pos)


def ear_clip(poly: list[tuple[float, float]]) -> list[tuple[int, int, int]]:
    """Triangulate a simple CCW polygon. Returns index triples into ``poly``.

    Real footprints are frequently concave, so a fan triangulation would produce
    triangles outside the building and give it phantom collision geometry.
    """
    n = len(poly)
    if n < 3:
        return []

    indices = list(range(n))
    triangles: list[tuple[int, int, int]] = []
    guard = 0

    while len(indices) > 3 and guard < 4 * n:
        guard += 1
        clipped = False
        for k in range(len(indices)):
            i_prev = indices[(k - 1) % len(indices)]
            i_curr = indices[k]
            i_next = indices[(k + 1) % len(indices)]
            ax, ay = poly[i_prev]
            bx, by = poly[i_curr]
            cx, cy = poly[i_next]

            # Convex corner? (CCW polygon => positive cross product)
            if (bx - ax) * (cy - ay) - (by - ay) * (cx - ax) <= 0:
                continue

            # No other vertex inside the candidate ear
            if any(
                _point_in_triangle(poly[i][0], poly[i][1], ax, ay, bx, by, cx, cy)
                for i in indices
                if i not in (i_prev, i_curr, i_next)
            ):
                continue

            triangles.append((i_prev, i_curr, i_next))
            indices.pop(k)
            clipped = True
            break

        if not clipped:
            break  # degenerate ring; emit what we have

    if len(indices) == 3:
        triangles.append((indices[0], indices[1], indices[2]))
    return triangles


def emit_prism(f: TextIO, base: int, ring: list[tuple[float, float]], height: float) -> int:
    """Write one extruded footprint: walls + top + bottom, CCW outward. Returns new base."""
    if len(ring) < 3:
        return base

    # Normalise to CCW so the winding rules below hold
    if signed_area(ring) < 0:
        ring = list(reversed(ring))

    n = len(ring)
    for x, y in ring:  # bottom ring first, then top ring
        f.write(f"v {x:.3f} {y:.3f} 0.000\n")
    for x, y in ring:
        f.write(f"v {x:.3f} {y:.3f} {height:.3f}\n")

    # Side walls: for CCW ring, (bottom_i, bottom_i+1, top_i+1, top_i) faces outward
    for i in range(n):
        j = (i + 1) % n
        b_i, b_j = base + 1 + i, base + 1 + j
        t_i, t_j = base + 1 + n + i, base + 1 + n + j
        f.write(f"f {b_i} {b_j} {t_j}\n")
        f.write(f"f {b_i} {t_j} {t_i}\n")

    # Caps: top CCW from above (+z), bottom reversed (-z)
    for a, b, c in ear_clip(ring):
        f.write(f"f {base + 1 + n + a} {base + 1 + n + b} {base + 1 + n + c}\n")
        f.write(f"f {base + 1 + c} {base + 1 + b} {base + 1 + a}\n")

    return base + 2 * n


# ---------------------------------------------------------------------------- main


def extract_buildings(payload: dict, anchor_lat: float, anchor_lon: float):
    """OSM elements -> [(ring_in_enu, height)], deduplicated by OSM id."""
    seen: set[tuple[str, int]] = set()
    out = []

    for element in payload.get("elements", []):
        key = (element.get("type", "?"), element.get("id", 0))
        if key in seen:
            continue
        seen.add(key)

        geometry = element.get("geometry")
        if not geometry:
            continue  # relations without inline geometry — skipped, see module docstring

        ring = [to_enu(p["lat"], p["lon"], anchor_lat, anchor_lon) for p in geometry if "lat" in p]
        # Overpass closes ways by repeating the first node
        if len(ring) > 1 and ring[0] == ring[-1]:
            ring = ring[:-1]
        if len(ring) < 3:
            continue

        out.append((ring, parse_height(element.get("tags", {}))))

    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_dir")
    ap.add_argument("--lat", type=float, required=True, help="anchor latitude (must match the SITL --home)")
    ap.add_argument("--lon", type=float, required=True, help="anchor longitude")
    ap.add_argument("--alt", type=float, default=0.0, help="anchor altitude, metres")
    ap.add_argument("--radius", type=float, default=5000.0, help="metres around the anchor")
    ap.add_argument("--tile-size", type=float, default=250.0)
    ap.add_argument("--cache", help="read/write the raw Overpass response here to avoid refetching")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    payload = None
    if args.cache and os.path.exists(args.cache):
        print(f"using cached Overpass response: {args.cache}")
        with open(args.cache) as f:
            payload = json.load(f)
    if payload is None:
        payload = fetch_osm(args.lat, args.lon, args.radius)
        if args.cache:
            with open(args.cache, "w") as f:
                json.dump(payload, f)
            print(f"cached Overpass response -> {args.cache}")

    buildings = extract_buildings(payload, args.lat, args.lon)
    print(f"{len(buildings)} building footprints")
    if not buildings:
        raise SystemExit("no buildings found — check the anchor and radius")

    # Bucket into tiles by centroid. Tile indices are signed, so the map can grow
    # in any direction without renumbering.
    tiles: dict[tuple[int, int], list] = {}
    for ring, height in buildings:
        cx = sum(p[0] for p in ring) / len(ring)
        cy = sum(p[1] for p in ring) / len(ring)
        tiles.setdefault((math.floor(cx / args.tile_size), math.floor(cy / args.tile_size)), []).append((ring, height))

    total_faces = 0
    for (ti, tj), items in sorted(tiles.items()):
        # 'm' prefix keeps negative indices filename-safe and sortable
        name = f"tile_{'m' if ti < 0 else ''}{abs(ti)}_{'m' if tj < 0 else ''}{abs(tj)}.obj"
        path = os.path.join(args.out_dir, name)
        with open(path, "w") as f:
            f.write(f"# skysim OSM tile ({ti},{tj}) — ENU metres about {args.lat},{args.lon}, CCW outward\n")
            base = 0
            for ring, height in items:
                base = emit_prism(f, base, ring, height)
            total_faces += base
        print(f"{path}: {len(items)} buildings")

    anchor_path = os.path.join(args.out_dir, "anchor.json")
    with open(anchor_path, "w") as f:
        json.dump(
            {
                "origin_wgs84": [args.lat, args.lon, args.alt],
                "crs": "ENU metres about origin_wgs84",
                "radius_m": args.radius,
                "tile_size_m": args.tile_size,
                "source": "OpenStreetMap (ODbL)",
                "buildings": len(buildings),
            },
            f,
            indent=2,
        )

    print(f"wrote {len(tiles)} tiles ({total_faces} vertices) to {args.out_dir}")
    print(f"anchor recorded in {anchor_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
