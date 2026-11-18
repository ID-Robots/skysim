#!/usr/bin/env python3
"""Real-city map generator: the buildings on the SkyHub map -> per-tile OBJ files for tile_cooker.

The operator plans and watches flights on a Mapbox map whose 3D buildings are the Mapbox
Streets v8 ``building`` layer. skysim must fly through those same buildings, at the same
heights — the Arbiter's map context (the gateway's ``GET /map/structures``) reads every
height it reasons with from skysim's geometry — so this extrudes what that layer draws, from
one of two sources:

``--source mapbox``
    The layer itself: Mapbox Streets v8 vector tiles at zoom 16 (the zoom that holds every
    building), with ``height``, ``min_height``, ``extrude`` and ``underground`` as Mapbox
    serves them. Needs a Mapbox access token (``--mapbox-token`` or ``MAPBOX_ACCESS_TOKEN``).

``--source osm`` (default)
    OpenStreetMap through Overpass (free and redistributable — ODbL), read the way Mapbox
    documents it reads OSM for that layer
    (https://docs.mapbox.com/data/tilesets/reference/mapbox-streets-v8/#building):

    * top: ``height`` in metres — 12, 12.5 m, 12m, 40 ft and 40'6" all parse; else
      ``building:levels`` x 3 m ("We estimate 3 meters per level");
    * bottom: ``min_height`` (a tower on a podium, a bridge);
    * also read, though Mapbox's page does not name them: ``building:height`` for a missing
      ``height``, ``building:min_level`` x 3 m for a missing ``min_height``, and the old
      ``levels`` for a missing ``building:levels``;
    * ``building:part`` ways and relations are buildings of their own, and an outline with
      parts inside it (not in its courtyard) is not extruded (Mapbox's ``extrude=false``): its
      parts carry the heights;
    * multipolygon relations: every outer ring;
    * ``location=underground`` buildings are left out.

    A building with no height and no levels gets ``--default-height`` (8 m). OSM has neither
    for most of a city — 9 213 of the 10 277 OSM buildings within 2.5 km of the Plovdiv anchor
    (2026-11-18) — so a deployed world should be cooked from Mapbox; ``anchor.json`` counts how
    many heights were guessed.

Map frame matches pretile.py: ENU (x east, y north, z up), meters, origin at the
anchor — which must be the same point the vehicle is spawned at (ArduPilot
``--home``), or the simulated buildings will not line up with what the operator
sees on the map. The anchor is recorded in ``anchor.json`` next to the OBJ files
so the cook step can stamp it into index.json instead of leaving the tile set
un-georeferenced.

Faces are CCW-wound viewed from outside; tile_cooker converts ENU -> Jolt and
relies on that outward winding (mesh collision is one-sided).

Usage:
  python3 tools/cooker/osm_buildings.py out_dir --lat 42.1403890 --lon 24.7645490 \
      --radius 5000 [--tile-size 250] [--cache buildings.json]
  MAPBOX_ACCESS_TOKEN=pk... python3 tools/cooker/osm_buildings.py out_dir --source mapbox \
      --lat 42.1403890 --lon 24.7645490 --radius 5000 [--cache mapbox_tiles/]

Notes / limits:
  - Only the outer rings of each building are used; courtyards are filled in. For
    collision purposes a solid building is the conservative choice.
  - Buildings straddling a tile boundary are assigned whole to the tile holding
    their centroid, so a tile's geometry can extend slightly past its nominal
    bounds. The streamer keeps neighbouring tiles resident, so this is safe.
  - The ground is flat: every building stands on NED z = 0, the height of the anchor.
  - Mapbox cuts a building that crosses a vector tile's edge into one piece per tile;
    each piece is clipped to its tile and extruded on its own, so the pieces meet
    at the tile seam.
"""

from __future__ import annotations

import argparse
import gzip
import http.client
import json
import math
import os
import re
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import Counter
from dataclasses import dataclass
from typing import TextIO

import mvt  # tools/cooker/mvt.py

OVERPASS_URLS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
]
MAPBOX_TILESET = "mapbox.mapbox-streets-v8"
MAPBOX_ZOOM = 16  # "all buildings are included in zoom level 16 and up"
LEVEL_HEIGHT_M = 3.0  # Mapbox's estimate per building:levels
DEFAULT_HEIGHT_M = 8.0
EARTH_RADIUS_M = 6378137.0


@dataclass
class Prism:
    """One footprint ring (ENU metres) extruded from base_m up to top_m."""

    ring: list[tuple[float, float]]
    base_m: float
    top_m: float


# ---------------------------------------------------------------------------- fetch


def build_query(lat: float, lon: float, radius_m: float) -> str:
    around = f"(around:{radius_m},{lat},{lon})"
    return f"""
[out:json][timeout:300];
(
  way["building"]{around};
  relation["building"]{around};
  way["building:part"]{around};
  relation["building:part"]{around};
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
                payload = json.loads(response.read().decode())
        except (OSError, http.client.HTTPException, json.JSONDecodeError) as e:
            print(f"  {url} failed: {e}")
            last_error = e
            continue
        # A query that times out or runs out of memory still answers 200, with what it had
        # so far and a remark: a partial city, which must not be cooked or cached.
        remark = str(payload.get("remark", ""))
        if "error" in remark.lower():
            print(f"  {url} failed: {remark}")
            last_error = RuntimeError(remark)
            continue
        payload["skysim_query"] = {"lat": lat, "lon": lon, "radius_m": radius_m}
        return payload

    raise SystemExit(f"all Overpass endpoints failed: {last_error}")


def tile_range(lat: float, lon: float, radius_m: float, zoom: int = MAPBOX_ZOOM) -> list[tuple[int, int]]:
    """Every web-mercator tile (x, y) at ``zoom`` that the square around the anchor touches."""
    dlat = math.degrees(radius_m / EARTH_RADIUS_M)
    dlon = math.degrees(radius_m / (EARTH_RADIUS_M * math.cos(math.radians(lat))))
    x0, y0 = lat_lon_to_tile(lat + dlat, lon - dlon, zoom)  # north-west corner
    x1, y1 = lat_lon_to_tile(lat - dlat, lon + dlon, zoom)  # south-east corner
    return [(x, y) for x in range(math.floor(x0), math.floor(x1) + 1) for y in range(math.floor(y0), math.floor(y1) + 1)]


def lat_lon_to_tile(lat: float, lon: float, zoom: int) -> tuple[float, float]:
    """Fractional web-mercator tile coordinates."""
    n = 2.0**zoom
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def tile_to_lat_lon(zoom: int, x: float, y: float) -> tuple[float, float]:
    """Inverse of lat_lon_to_tile."""
    n = 2.0**zoom
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1.0 - 2.0 * y / n))))
    return lat, lon


def mapbox_cache_path(cache_dir: str, tileset: str, zoom: int, x: int, y: int) -> str:
    """Where one tile is kept: per tileset, so a mistyped one cannot stand in for the right one."""
    return os.path.join(cache_dir, f"{tileset}_{zoom}_{x}_{y}.pbf")


def fetch_mapbox_tile(zoom: int, x: int, y: int, token: str, tileset: str) -> bytes:
    """One vector tile, uncompressed; b"" for a tile Mapbox has no data for."""
    url = f"https://api.mapbox.com/v4/{tileset}/{zoom}/{x}/{y}.vector.pbf"
    request = urllib.request.Request(
        url + "?" + urllib.parse.urlencode({"access_token": token}), headers={"User-Agent": "skysim-cooker/1.0"}
    )
    error: Exception | None = None
    for attempt in range(4):
        try:
            with urllib.request.urlopen(request, timeout=60) as response:
                data = response.read()
            return gzip.decompress(data) if data[:2] == b"\x1f\x8b" else data
        except urllib.error.HTTPError as e:
            if e.code == 404:
                return b""
            if e.code in (401, 403):
                raise SystemExit(f"Mapbox refused {url} ({e.code}): check the access token") from None
            if e.code != 429 and e.code < 500:
                raise SystemExit(f"Mapbox {url} failed: HTTP {e.code}") from None
            error = e
        except (OSError, EOFError, http.client.HTTPException) as e:  # reset mid-read, bad gzip
            error = e
        time.sleep(2.0 * (attempt + 1))
    raise SystemExit(f"Mapbox {url} failed: {error}")


def load_mapbox_tiles(
    lat: float, lon: float, radius_m: float, token: str | None, tileset: str, cache_dir: str | None
) -> list[tuple[int, int, int, bytes]]:
    """(zoom, x, y, tile) for the area, from the cache directory where it has them."""
    tiles = []
    if cache_dir:
        os.makedirs(cache_dir, exist_ok=True)
    wanted = tile_range(lat, lon, radius_m)
    print(f"{len(wanted)} Mapbox tiles at z{MAPBOX_ZOOM} for {radius_m:.0f} m around {lat},{lon}")
    for x, y in wanted:
        path = mapbox_cache_path(cache_dir, tileset, MAPBOX_ZOOM, x, y) if cache_dir else None
        if path and os.path.exists(path):
            with open(path, "rb") as f:
                data = f.read()
            if data[:2] == b"\x1f\x8b":  # stored as served
                data = gzip.decompress(data)
        else:
            if not token:
                raise SystemExit(
                    f"no Mapbox access token (--mapbox-token or MAPBOX_ACCESS_TOKEN) to fetch tile {MAPBOX_ZOOM}/{x}/{y}"
                )
            data = fetch_mapbox_tile(MAPBOX_ZOOM, x, y, token, tileset)
            if path:  # whole or not at all: an interrupted cook must not leave half a tile
                with open(path + ".part", "wb") as f:
                    f.write(data)
                os.replace(path + ".part", path)
        tiles.append((MAPBOX_ZOOM, x, y, data))
    return tiles


# ---------------------------------------------------------------------------- heights

_METRES = re.compile(r"^(\d+(?:\.\d+)?)\s*(?:m|metres?|meters?)?$")
_FEET = re.compile(r"^(\d+(?:\.\d+)?)\s*(?:ft|feet|')$")
_FEET_INCHES = re.compile(r"^(\d+(?:\.\d+)?)\s*'\s*(\d+(?:\.\d+)?)\s*\"$")
_NUMBER = re.compile(r"^(\d+(?:\.\d+)?)$")


def parse_length(raw: object) -> float | None:
    """Metres from an OSM length (12, 12.5 m, 12m, 12,5, 40 ft, 40'6"); None if it is not one."""
    text = str(raw).strip().lower().replace(",", ".")
    if m := _METRES.match(text):
        return float(m.group(1))
    if m := _FEET.match(text):
        return float(m.group(1)) * 0.3048
    if m := _FEET_INCHES.match(text):
        return float(m.group(1)) * 0.3048 + float(m.group(2)) * 0.0254
    return None


def parse_count(raw: object) -> float | None:
    """A level count ("5", "5.5"); None if it is not one."""
    m = _NUMBER.match(str(raw).strip().replace(",", "."))
    return float(m.group(1)) if m else None


def building_extent(tags: dict, default_height_m: float = DEFAULT_HEIGHT_M) -> tuple[float, float, str]:
    """(bottom, top, which rule gave the top) in metres above the ground, from OSM tags.

    The rules Mapbox Streets v8 documents for its building layer's ``height`` and
    ``min_height``: an explicit height, else 3 m per level; the bottom of a part that does
    not reach the ground from ``min_height`` or ``building:min_level``.
    """
    top: float | None = None
    rule = "default"
    for key in ("height", "building:height"):
        value = parse_length(tags[key]) if tags.get(key) else None
        if value is not None and value > 0:
            top, rule = value, "height"
            break
    if top is None:
        for key in ("building:levels", "levels"):
            value = parse_count(tags[key]) if tags.get(key) else None
            if value is not None and value > 0:
                top, rule = value * LEVEL_HEIGHT_M, "levels"
                break

    bottom = parse_length(tags["min_height"]) if tags.get("min_height") else None
    if bottom is None:
        min_level = parse_count(tags["building:min_level"]) if tags.get("building:min_level") else None
        bottom = min_level * LEVEL_HEIGHT_M if min_level is not None else 0.0

    if top is None:
        # Nothing known: the default, and at least one level above a raised bottom.
        top = max(default_height_m, bottom + LEVEL_HEIGHT_M)
    return bottom, top, rule


# ---------------------------------------------------------------------------- geometry


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


def point_in_ring(x: float, y: float, ring: list[tuple[float, float]]) -> bool:
    """Even-odd rule."""
    inside = False
    for i, (x0, y0) in enumerate(ring):
        x1, y1 = ring[(i + 1) % len(ring)]
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            inside = not inside
    return inside


def interior_point(ring: list[tuple[float, float]]) -> tuple[float, float]:
    """A point inside the ring: its centroid when that is inside, else the middle of the widest
    stretch of a horizontal line through it (an L- or U-shaped footprint's centroid is not)."""
    area = signed_area(ring)
    if abs(area) > 1e-9:
        cx = cy = 0.0
        for i, (x0, y0) in enumerate(ring):
            x1, y1 = ring[(i + 1) % len(ring)]
            cross = x0 * y1 - x1 * y0
            cx += (x0 + x1) * cross
            cy += (y0 + y1) * cross
        cx, cy = cx / (6.0 * area), cy / (6.0 * area)
    else:
        cx = sum(p[0] for p in ring) / len(ring)
        cy = sum(p[1] for p in ring) / len(ring)
    if point_in_ring(cx, cy, ring):
        return cx, cy
    crossings = []
    for i, (x0, y0) in enumerate(ring):
        x1, y1 = ring[(i + 1) % len(ring)]
        if (y0 > cy) != (y1 > cy):
            crossings.append(x0 + (cy - y0) * (x1 - x0) / (y1 - y0))
    crossings.sort()
    spans = [(crossings[k + 1] - crossings[k], crossings[k]) for k in range(0, len(crossings) - 1, 2)]
    if not spans:
        return cx, cy
    width, start = max(spans)
    return start + width / 2.0, cy


def stitch_rings(ways: list[list[tuple[float, float]]]) -> list[list[tuple[float, float]]]:
    """Join a multipolygon's member ways into closed rings; a ring may be split over several
    ways, in either direction. A ring that cannot be closed is dropped."""
    pending = [list(w) for w in ways if len(w) >= 2]
    rings = []
    while pending:
        ring = pending.pop(0)
        while ring[0] != ring[-1]:
            for k, way in enumerate(pending):
                if way[0] == ring[-1]:
                    ring.extend(way[1:])
                    break
                if way[-1] == ring[-1]:
                    ring.extend(reversed(way[:-1]))
                    break
            else:
                ring = []
                break
            pending.pop(k)
        if len(ring) >= 4:
            rings.append(ring[:-1])  # drop the repeated closing node
    return rings


def dedupe_ring(ring: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """The ring without repeated consecutive points (last against first too): ear_clip takes a
    point on an ear's corner for one inside it, and would leave the roof out."""
    out: list[tuple[float, float]] = []
    for point in ring:
        if not out or abs(point[0] - out[-1][0]) > 1e-9 or abs(point[1] - out[-1][1]) > 1e-9:
            out.append(point)
    while len(out) > 1 and abs(out[0][0] - out[-1][0]) <= 1e-9 and abs(out[0][1] - out[-1][1]) <= 1e-9:
        out.pop()
    return out


def _clip_half_plane(
    ring: list[tuple[float, float]], axis: int, bound: float, keep_above: bool
) -> list[list[tuple[float, float]]]:
    """A counter-clockwise ring cut to one side of the line ``p[axis] == bound``: one ring per
    piece left. Sutherland-Hodgman would join the pieces of a concave footprint along the line,
    and that joint, extruded, is a wall across the gap between them."""
    sign = 1.0 if keep_above else -1.0

    def depth(p: tuple[float, float]) -> float:  # >= 0 on the kept side
        return (p[axis] - bound) * sign

    def along(p: tuple[float, float]) -> float:  # along the line, the kept side on the left
        return p[0] * sign if axis == 1 else -p[1] * sign

    inside = [depth(p) >= 0.0 for p in ring]
    if all(inside):
        return [ring]
    if not any(inside):
        return []
    first = inside.index(False)  # start outside: every piece then runs entry -> exit
    ring = ring[first:] + ring[:first]
    inside = inside[first:] + inside[:first]

    chains: list[list[tuple[float, float]]] = []
    current: list[tuple[float, float]] = []
    for i, a in enumerate(ring):
        b = ring[(i + 1) % len(ring)]
        if inside[i] != inside[(i + 1) % len(ring)]:
            t = depth(a) / (depth(a) - depth(b))
            point = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
            point[axis] = bound
            if inside[(i + 1) % len(ring)]:
                current = [(point[0], point[1])]
            else:
                chains.append([*current, (point[0], point[1])])
                current = []
        if inside[(i + 1) % len(ring)]:
            current.append(b)

    # Along the line, the polygon's inside runs from each exit to the nearest entry after it:
    # that is the edge which closes a piece.
    pieces = []
    used = [False] * len(chains)
    for start in range(len(chains)):
        if used[start]:
            continue
        piece: list[tuple[float, float]] = []
        c = start
        while not used[c]:
            used[c] = True
            piece.extend(chains[c])
            leave = along(chains[c][-1])
            later = [k for k in range(len(chains)) if (not used[k] or k == start) and along(chains[k][0]) >= leave - 1e-9]
            c = min(later, key=lambda k: along(chains[k][0])) if later else start
        pieces.append(piece)
    return pieces


def clip_ring(ring: list[tuple[float, float]], lo: float, hi: float) -> list[list[tuple[float, float]]]:
    """The ring cut to the square [lo, hi] x [lo, hi]: the pieces left, each a ring of its own."""
    ring = dedupe_ring(ring)
    if signed_area(ring) < 0:
        ring = list(reversed(ring))
    pieces = [ring]
    for axis, bound, keep_above in ((0, lo, True), (0, hi, False), (1, lo, True), (1, hi, False)):
        pieces = [cut for piece in pieces for cut in _clip_half_plane(piece, axis, bound, keep_above)]
    # A vertex on the line is both kept and cut: drop the copy, and what has no area left.
    pieces = [dedupe_ring(piece) for piece in pieces]
    return [piece for piece in pieces if len(piece) >= 3 and abs(signed_area(piece)) > 1e-9]


def _point_in_triangle(
    px: float, py: float, ax: float, ay: float, bx: float, by: float, cx: float, cy: float
) -> bool:
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


def emit_prism(f: TextIO, base: int, ring: list[tuple[float, float]], height: float, bottom: float = 0.0) -> int:
    """Write one extruded footprint, ``bottom`` up to ``height`` metres: walls + top + bottom,
    CCW outward. Returns the new vertex base."""
    ring = dedupe_ring(ring)
    if len(ring) < 3:
        return base

    # Normalise to CCW so the winding rules below hold
    if signed_area(ring) < 0:
        ring = list(reversed(ring))

    n = len(ring)
    for x, y in ring:  # bottom ring first, then top ring
        f.write(f"v {x:.3f} {y:.3f} {bottom:.3f}\n")
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


# ---------------------------------------------------------------------------- sources


def _yes(value: object) -> bool:
    return value is not None and value != "no"


def _element_rings(element: dict) -> tuple[list[list[tuple[float, float]]], list[list[tuple[float, float]]]]:
    """(lat, lon) outer rings and holes of an Overpass way or multipolygon relation, unclosed."""
    if element.get("type") == "way":
        ring = [(p["lat"], p["lon"]) for p in element.get("geometry") or [] if p and "lat" in p]
        # Overpass closes ways by repeating the first node
        if len(ring) > 1 and ring[0] == ring[-1]:
            ring = ring[:-1]
        return ([ring] if len(ring) >= 3 else []), []

    def members(roles: tuple[str, ...]) -> list[list[tuple[float, float]]]:
        return [
            [(p["lat"], p["lon"]) for p in member.get("geometry") or [] if p and "lat" in p]
            for member in element.get("members") or []
            if member.get("type") == "way" and member.get("role", "outer") in roles
        ]

    return stitch_rings(members(("outer", ""))), stitch_rings(members(("inner",)))


def extract_buildings(
    payload: dict, anchor_lat: float, anchor_lon: float, default_height_m: float = DEFAULT_HEIGHT_M
) -> tuple[list[Prism], Counter]:
    """OSM elements -> prisms, deduplicated by OSM id, and a count of what was done with them."""
    counts: Counter = Counter()
    seen: set[tuple[str, int]] = set()
    Ring = list[tuple[float, float]]
    outlines: list[tuple[list[Ring], list[Ring], dict]] = []
    parts: list[tuple[list[Ring], list[Ring], dict]] = []

    for element in payload.get("elements", []):
        key = (element.get("type", "?"), element.get("id", 0))
        if key in seen:
            continue
        seen.add(key)

        tags = element.get("tags") or {}
        is_part = _yes(tags.get("building:part"))
        if not is_part and not _yes(tags.get("building")):
            continue
        if tags.get("location") == "underground":
            counts["underground"] += 1
            continue
        outers, inners = (
            [[to_enu(lat, lon, anchor_lat, anchor_lon) for lat, lon in ring] for ring in rings]
            for rings in _element_rings(element)
        )
        if not outers:
            counts["no_geometry"] += 1
            continue
        (parts if is_part else outlines).append((outers, inners, tags))

    # An outline with building:part inside it is Mapbox's extrude=false: only its parts are drawn.
    cell = 50.0
    grid: dict[tuple[int, int], list[int]] = {}
    for i, (rings, _, _) in enumerate(outlines):
        xs = [p[0] for ring in rings for p in ring]
        ys = [p[1] for ring in rings for p in ring]
        for gx in range(math.floor(min(xs) / cell), math.floor(max(xs) / cell) + 1):
            for gy in range(math.floor(min(ys) / cell), math.floor(max(ys) / cell) + 1):
                grid.setdefault((gx, gy), []).append(i)
    # Only a part that will be drawn replaces anything, and only the outer ring it stands in:
    # another ring of the same multipolygon, with no part of its own, stays.
    part_extents = [building_extent(tags, default_height_m) for _, _, tags in parts]
    replaced: set[tuple[int, int]] = set()  # (outline, outer ring)
    for (rings, _, _), (bottom, top, _) in zip(parts, part_extents):
        if top <= bottom:
            continue
        for ring in rings:
            px, py = interior_point(ring)
            for i in grid.get((math.floor(px / cell), math.floor(py / cell)), []):
                outers, inners, _ = outlines[i]
                if any(point_in_ring(px, py, r) for r in inners):
                    continue  # a part standing in the courtyard is not this building's
                for k, outer in enumerate(outers):
                    if point_in_ring(px, py, outer):
                        replaced.add((i, k))

    prisms: list[Prism] = []
    for i, (outers, _, tags) in enumerate(outlines):
        kept = [ring for k, ring in enumerate(outers) if (i, k) not in replaced]
        if len(kept) < len(outers):
            counts["outline_with_parts"] += 1
        if not kept:
            continue
        bottom, top, rule = building_extent(tags, default_height_m)
        if top <= bottom:
            counts["degenerate"] += 1
            continue
        counts[rule] += 1
        prisms.extend(Prism(ring, bottom, top) for ring in kept)
    for (rings, _, _), (bottom, top, rule) in zip(parts, part_extents):
        if top <= bottom:
            counts["degenerate"] += 1
            continue
        counts[rule] += 1
        prisms.extend(Prism(ring, bottom, top) for ring in rings)
    return prisms, counts


def _flag(value: object) -> bool | None:
    """Mapbox's 'true'/'false' fields (strings in Streets v8; booleans tolerated)."""
    if value is None:
        return None
    return str(value).lower() == "true"


def _number(value: object) -> float | None:
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None


def extract_mapbox_buildings(
    tiles: list[tuple[int, int, int, bytes]],
    anchor_lat: float,
    anchor_lon: float,
    radius_m: float,
    default_height_m: float = DEFAULT_HEIGHT_M,
) -> tuple[list[Prism], Counter]:
    """Mapbox Streets v8 ``building`` features -> prisms, and a count of what was done with them.

    Counts are per tile feature: a building cut by a tile edge counts once per piece.
    """
    counts: Counter = Counter()
    prisms: list[Prism] = []
    for zoom, tx, ty, data in tiles:
        if not data:
            continue
        try:
            layer = mvt.decode_layers(data).get("building")
            if layer is not None and layer.extent <= 0:
                raise ValueError(f"extent {layer.extent}")
            features = [(f, mvt.polygon_rings(f.geometry)) for f in layer.features] if layer else []
        except (ValueError, UnicodeDecodeError) as e:
            raise SystemExit(f"Mapbox tile {zoom}/{tx}/{ty} is not a vector tile ({e}): delete it from the cache") from None
        if layer is None:
            continue
        extent = float(layer.extent)
        for feature, rings in features:
            if feature.type != mvt.GEOM_POLYGON:
                continue
            props = feature.properties
            if _flag(props.get("underground")):
                counts["underground"] += 1
                continue
            if _flag(props.get("extrude")) is False:
                counts["outline_with_parts"] += 1
                continue
            top = _number(props.get("height"))
            bottom = _number(props.get("min_height")) or 0.0
            rule = "mapbox"
            if top is None or top <= 0:
                top, rule = max(default_height_m, bottom + LEVEL_HEIGHT_M), "default"
            if top <= bottom:
                counts["degenerate"] += 1
                continue
            pieces = 0
            for ring in rings:
                if mvt.ring_area(ring) <= 0:
                    continue  # a hole: courtyards are filled in
                for clipped in clip_ring([(float(x), float(y)) for x, y in ring], 0.0, extent):
                    if abs(signed_area(clipped)) < 1e-6:
                        continue
                    enu = [
                        to_enu(*tile_to_lat_lon(zoom, tx + px / extent, ty + py / extent), anchor_lat, anchor_lon)
                        for px, py in clipped
                    ]
                    cx = sum(p[0] for p in enu) / len(enu)
                    cy = sum(p[1] for p in enu) / len(enu)
                    if math.hypot(cx, cy) > radius_m:
                        continue
                    prisms.append(Prism(enu, bottom, top))
                    pieces += 1
            if pieces:
                counts[rule] += 1
    return prisms, counts


# ---------------------------------------------------------------------------- main


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_dir")
    ap.add_argument("--lat", type=float, required=True, help="anchor latitude (must match the SITL --home)")
    ap.add_argument("--lon", type=float, required=True, help="anchor longitude")
    ap.add_argument("--alt", type=float, default=0.0, help="anchor altitude, metres")
    ap.add_argument("--radius", type=float, default=5000.0, help="metres around the anchor")
    ap.add_argument("--tile-size", type=float, default=250.0)
    ap.add_argument(
        "--source", choices=("osm", "mapbox"), default="osm", help="building data: OpenStreetMap or Mapbox Streets v8"
    )
    ap.add_argument(
        "--cache",
        help="osm: read/write the raw Overpass response in this file; mapbox: read/write the vector tiles "
        "in this directory (avoids refetching)",
    )
    ap.add_argument("--mapbox-token", default=os.environ.get("MAPBOX_ACCESS_TOKEN"), help="default: $MAPBOX_ACCESS_TOKEN")
    ap.add_argument("--mapbox-tileset", default=MAPBOX_TILESET)
    ap.add_argument(
        "--default-height",
        type=float,
        default=DEFAULT_HEIGHT_M,
        help="metres for a building the source gives no height for",
    )
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    if args.source == "mapbox":
        tiles = load_mapbox_tiles(args.lat, args.lon, args.radius, args.mapbox_token, args.mapbox_tileset, args.cache)
        prisms, counts = extract_mapbox_buildings(tiles, args.lat, args.lon, args.radius, args.default_height)
        source = f"Mapbox Streets v8 ({args.mapbox_tileset}, z{MAPBOX_ZOOM} vector tiles)"
    else:
        payload = None
        if args.cache and os.path.exists(args.cache):
            print(f"using cached Overpass response: {args.cache}")
            with open(args.cache) as f:
                payload = json.load(f)
            query = payload.get("skysim_query")  # absent in a hand-made or pre-TASK-2679 cache
            wanted = {"lat": args.lat, "lon": args.lon, "radius_m": args.radius}
            if query and any(not math.isclose(float(query.get(k, math.nan)), v) for k, v in wanted.items()):
                raise SystemExit(f"{args.cache} was fetched for {query}, not {wanted}: delete it or match it")
        if payload is None:
            payload = fetch_osm(args.lat, args.lon, args.radius)
            if args.cache:
                with open(args.cache, "w") as f:
                    json.dump(payload, f)
                print(f"cached Overpass response -> {args.cache}")
        prisms, counts = extract_buildings(payload, args.lat, args.lon, args.default_height)
        source = "OpenStreetMap (ODbL), heights by the Mapbox Streets v8 rules"

    extruded = sum(counts[rule] for rule in ("height", "levels", "mapbox", "default"))
    print(f"{extruded} buildings, {len(prisms)} footprints; heights: " + ", ".join(f"{k} {v}" for k, v in sorted(counts.items())))
    if counts["default"]:
        print(f"  {counts['default']} of {extruded} buildings had no height in the source: {args.default_height:g} m assumed")
    if not prisms:
        hint = " and --mapbox-tileset" if args.source == "mapbox" else ""
        raise SystemExit(f"no buildings found — check the anchor and radius{hint}")

    # Bucket into tiles by centroid. Tile indices are signed, so the map can grow
    # in any direction without renumbering.
    tiles_out: dict[tuple[int, int], list[Prism]] = {}
    for prism in prisms:
        cx = sum(p[0] for p in prism.ring) / len(prism.ring)
        cy = sum(p[1] for p in prism.ring) / len(prism.ring)
        tiles_out.setdefault((math.floor(cx / args.tile_size), math.floor(cy / args.tile_size)), []).append(prism)

    def tile_name(ti: int, tj: int) -> str:
        # 'm' prefix keeps negative indices filename-safe and sortable
        return f"tile_{'m' if ti < 0 else ''}{abs(ti)}_{'m' if tj < 0 else ''}{abs(tj)}.obj"

    total_vertices = 0
    for (ti, tj), items in sorted(tiles_out.items()):
        path = os.path.join(args.out_dir, tile_name(ti, tj))
        with open(path, "w") as f:
            f.write(f"# skysim building tile ({ti},{tj}) — ENU metres about {args.lat},{args.lon}, CCW outward\n")
            base = 0
            for prism in items:
                base = emit_prism(f, base, prism.ring, prism.top_m, prism.base_m)
            total_vertices += base
        print(f"{path}: {len(items)} footprints")
    # Not deleted here (the directory may hold someone's own OBJs), but tile_cooker would cook them too.
    written = {tile_name(ti, tj) for ti, tj in tiles_out}
    stale = sorted(n for n in os.listdir(args.out_dir) if n.startswith("tile_") and n.endswith(".obj") and n not in written)
    if stale:
        print(
            f"warning: {len(stale)} tile_*.obj in {args.out_dir} are not from this cook (e.g. {stale[0]}): "
            "remove them before running tile_cooker on *.obj"
        )

    anchor_path = os.path.join(args.out_dir, "anchor.json")
    with open(anchor_path, "w") as f:
        json.dump(
            {
                "origin_wgs84": [args.lat, args.lon, args.alt],
                "crs": "ENU metres about origin_wgs84",
                "radius_m": args.radius,
                "tile_size_m": args.tile_size,
                "source": source,
                "buildings": extruded,
                "footprints": len(prisms),
                # How each building's height was found (height / levels tag, Mapbox's height,
                # or the default), and what was left out (outlines whose parts were drawn
                # instead, underground, ...).
                "heights": dict(sorted(counts.items())),
                "default_height_m": args.default_height,
            },
            f,
            indent=2,
        )

    print(f"wrote {len(tiles_out)} tiles ({total_vertices} vertices) to {args.out_dir}")
    print(f"anchor recorded in {anchor_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
