#!/usr/bin/env python3
"""ctest 'building_heights': simulated buildings stand as tall as the map's (TASK-2679).

Three real buildings in central Plovdiv, fixed in tests/fixtures/plovdiv_buildings_osm.json
(OpenStreetMap contributors, ODbL; Overpass snapshot of 2026-11-18), are cooked into a world
twice — from the OSM data by the Mapbox Streets v8 rules (tools/cooker/osm_buildings.py), and from
Mapbox Streets v8 vector tiles made here with the attributes that layer gives them
(``--source mapbox``) — and each world is loaded into skysim and probed the way the gateway's
``GET /map/structures`` probes it for the Arbiter, through ``POST /mission/check``:

  * a vertical leg down onto the roof stops within TOLERANCE_M of the Mapbox height (Mapbox
    rounds heights to whole metres; OBJ vertices are written to the millimetre);
  * a level leg toward the building 1 m below that height hits its wall, 1 m above passes over.

The buildings (OSM ids):
  * way 1578993375, a 12-level ``building:part`` (36 m) inside the outline way 125444465
    (building=government, 2 levels): the tower is drawn, the outline is not (Mapbox
    ``extrude=false``). The cooker before TASK-2679 extruded only the outline, at 6 m.
  * relation 7292387, "Четвърти корпус" (building=university, multipolygon with a courtyard,
    5 levels = 15 m). Before: missing — every multipolygon relation was dropped.
  * way 99064086, "РРС Пловдив" (height=29): an explicit height, 29 m before and after.

Before the worlds: unit checks of the tag rules, ring stitching and clipping, and the vector
tile decoder. Usage: building_heights_smoke.py <skysim> <tile_cooker>
(API ports 18696/18697, instances 52/53.)
"""

from __future__ import annotations

import gzip
import importlib.util
import io
import json
import math
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time
import types
import urllib.error
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
COOKER_DIR = REPO / "tools" / "cooker"
FIXTURE = REPO / "tests" / "fixtures" / "plovdiv_buildings_osm.json"
# The world's anchor: README's, and the gateway's default SITL_HOME_LOCATION.
ANCHOR_LAT, ANCHOR_LON = 42.1403890, 24.7645490
RADIUS_M = 2000.0
EARTH_RADIUS_M = 6378137.0
TOLERANCE_M = 0.5
FAILURES: list[str] = []
CHECKS = [0]

# (name, OSM element whose footprint is probed, Mapbox height in metres)
BUILDINGS = [
    ("government tower (way 1578993375)", ("way", 1578993375), 36.0),
    ("Четвърти корпус (relation 7292387)", ("relation", 7292387), 15.0),
    ("РРС Пловдив (way 99064086)", ("way", 99064086), 29.0),
]
OUTLINE = ("way", 125444465)  # the tower's outline: not drawn


def check(cond: bool, what: str) -> None:
    CHECKS[0] += 1
    if not cond:
        FAILURES.append(what)
        print(f"FAIL: {what}")


def load_cooker() -> types.ModuleType:
    sys.path.insert(0, str(COOKER_DIR))  # osm_buildings imports its sibling mvt
    spec = importlib.util.spec_from_file_location("osm_buildings", COOKER_DIR / "osm_buildings.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["osm_buildings"] = module
    spec.loader.exec_module(module)
    return module


# ---------------------------------------------------------------------------- fixture geometry


def ned(lat: float, lon: float) -> tuple[float, float]:
    """(north, east) metres about the anchor: the gateway's wgs84_to_ned, written out again here."""
    north = math.radians(lat - ANCHOR_LAT) * EARTH_RADIUS_M
    east = math.radians(lon - ANCHOR_LON) * EARTH_RADIUS_M * math.cos(math.radians(ANCHOR_LAT))
    return north, east


def lat_lon_ring(points: list[dict]) -> list[tuple[float, float]]:
    ring = [(p["lat"], p["lon"]) for p in points]
    return ring[:-1] if ring[0] == ring[-1] else ring


def footprint(element: dict) -> tuple[list[tuple[float, float]], list[list[tuple[float, float]]]]:
    """(outer ring, holes) in (lat, lon); every fixture building has one closed outer way."""
    if element["type"] == "way":
        return lat_lon_ring(element["geometry"]), []
    outer = [lat_lon_ring(m["geometry"]) for m in element["members"] if m["role"] == "outer"]
    holes = [lat_lon_ring(m["geometry"]) for m in element["members"] if m["role"] == "inner"]
    assert len(outer) == 1
    return outer[0], holes


def inside(x: float, y: float, ring: list[tuple[float, float]]) -> bool:
    result = False
    for i, (x0, y0) in enumerate(ring):
        x1, y1 = ring[(i + 1) % len(ring)]
        if (y0 > y) != (y1 > y) and x < x0 + (y - y0) * (x1 - x0) / (y1 - y0):
            result = not result
    return result


def edge_distance(x: float, y: float, ring: list[tuple[float, float]]) -> float:
    best = math.inf
    for i, (x0, y0) in enumerate(ring):
        x1, y1 = ring[(i + 1) % len(ring)]
        dx, dy = x1 - x0, y1 - y0
        t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / (dx * dx + dy * dy or 1.0)))
        best = min(best, math.hypot(x - x0 - t * dx, y - y0 - t * dy))
    return best


def probe_point(
    outer: list[tuple[float, float]], holes: list[list[tuple[float, float]]], exclude: list[list[tuple[float, float]]]
) -> tuple[float, float]:
    """The point (north, east) of a 0.5 m grid inside ``outer`` and outside ``holes`` and
    ``exclude`` that lies farthest from all of their edges: a roof, not a wall or a courtyard."""
    rings = [outer, *holes, *exclude]
    ns = [p[0] for p in outer]
    es = [p[1] for p in outer]
    best = (-1.0, 0.0, 0.0)
    n = min(ns)
    while n <= max(ns):
        e = min(es)
        while e <= max(es):
            if inside(n, e, outer) and not any(inside(n, e, r) for r in holes + exclude):
                d = min(edge_distance(n, e, r) for r in rings)
                best = max(best, (d, n, e))
            e += 0.5
        n += 0.5
    check(best[0] >= 2.0, f"probe point at least 2 m inside its footprint ({best[0]:.1f} m)")
    return best[1], best[2]


# ---------------------------------------------------------------------------- vector tiles


def _varint(n: int) -> bytes:
    out = bytearray()
    while True:
        byte = n & 0x7F
        n >>= 7
        if n:
            out.append(byte | 0x80)
        else:
            out.append(byte)
            return bytes(out)


def _field(number: int, payload: bytes) -> bytes:
    return _varint(number << 3 | 2) + _varint(len(payload)) + payload


def _uint(number: int, value: int) -> bytes:
    return _varint(number << 3) + _varint(value)


def _zigzag(n: int) -> int:
    return n << 1 if n >= 0 else (-n << 1) - 1


def _mvt_value(value: object) -> bytes:
    if isinstance(value, str):
        return _field(1, value.encode())
    if isinstance(value, bool):
        return _uint(7, int(value))
    if isinstance(value, int):
        return _uint(5, value) if value >= 0 else _uint(6, _zigzag(value))
    return _varint(3 << 3 | 1) + struct.pack("<d", float(value))


def _area(ring: list[tuple[int, int]]) -> float:
    return sum(ring[i][0] * ring[(i + 1) % len(ring)][1] - ring[(i + 1) % len(ring)][0] * ring[i][1]
               for i in range(len(ring))) / 2.0


def encode_tile(layers: dict[str, list[tuple[dict, list[list[tuple[int, int]]]]]], extent: int = 4096) -> bytes:
    """A Mapbox vector tile (spec 2.1): per layer, polygon features (properties, rings) whose
    first ring is the exterior and the rest holes, in tile coordinates."""
    tile = b""
    for name, features in layers.items():
        keys: list[str] = []
        values: list[object] = []
        body = b""
        for fid, (props, rings) in enumerate(features, 1):
            tags = []
            for key, value in props.items():
                if key not in keys:
                    keys.append(key)
                if (type(value), value) not in [(type(v), v) for v in values]:
                    values.append(value)
                tags += [keys.index(key), [(type(v), v) for v in values].index((type(value), value))]
            geometry: list[int] = []
            x = y = 0
            for k, ring in enumerate(rings):
                # Exterior rings have a positive area in tile coordinates (y down), holes negative.
                if (_area(ring) > 0) != (k == 0):
                    ring = list(reversed(ring))
                geometry += [1 | 1 << 3, _zigzag(ring[0][0] - x), _zigzag(ring[0][1] - y)]
                x, y = ring[0]
                geometry.append(2 | (len(ring) - 1) << 3)
                for px, py in ring[1:]:
                    geometry += [_zigzag(px - x), _zigzag(py - y)]
                    x, y = px, py
                geometry.append(7 | 1 << 3)
            packed_tags = b"".join(_varint(t) for t in tags)
            packed_geometry = b"".join(_varint(g) for g in geometry)
            feature = _uint(1, fid) + _field(2, packed_tags) + _uint(3, 3) + _field(4, packed_geometry)
            body += _field(2, feature)
        layer = _uint(15, 2) + _field(1, name.encode()) + body
        layer += b"".join(_field(3, k.encode()) for k in keys)
        layer += b"".join(_field(4, _mvt_value(v)) for v in values)
        layer += _uint(5, extent)
        tile += _field(3, layer)
    return tile


def write_mapbox_tiles(
    cooker: types.ModuleType, cache: Path, features: list[tuple[dict, list[list[tuple[float, float]]]]]
) -> int:
    """Mapbox's z16 tiles for the cooker's whole area into its cache directory, the way Mapbox cuts
    them: a building goes into every tile its box overlaps, with coordinates past the tile's edge
    (the cooker clips). Tiles with no building are empty; every other one is gzipped, as served."""
    zoom = cooker.MAPBOX_ZOOM
    extent = 4096
    cache.mkdir(parents=True, exist_ok=True)
    written = 0
    for tx, ty in cooker.tile_range(ANCHOR_LAT, ANCHOR_LON, RADIUS_M):
        in_tile = []
        for props, rings in features:
            tile_rings = []
            for ring in rings:
                coords = []
                for lat, lon in ring:
                    fx, fy = cooker.lat_lon_to_tile(lat, lon, zoom)
                    coords.append((round((fx - tx) * extent), round((fy - ty) * extent)))
                tile_rings.append(coords)
            xs = [p[0] for p in tile_rings[0]]
            ys = [p[1] for p in tile_rings[0]]
            if max(xs) > 0 and min(xs) < extent and max(ys) > 0 and min(ys) < extent:
                in_tile.append((props, tile_rings))
        data = b""
        if in_tile:
            # A road layer first: the cooker must pick the building layer by name.
            road = ({"class": "street"}, [[(0, 0), (10, 0), (10, 10)]])
            data = gzip.compress(encode_tile({"road": [road], "building": in_tile}, extent))
            written += 1
        Path(cooker.mapbox_cache_path(str(cache), cooker.MAPBOX_TILESET, zoom, tx, ty)).write_bytes(data)
    return written


# ---------------------------------------------------------------------------- unit checks


def faces(cooker: types.ModuleType, ring: list[tuple[float, float]]) -> tuple[int, int]:
    """(walls, caps) emit_prism writes for one ring."""
    out = io.StringIO()
    cooker.emit_prism(out, 0, ring, 10.0)
    lines = out.getvalue().splitlines()
    n = sum(1 for line in lines if line.startswith("v ")) // 2
    total = sum(1 for line in lines if line.startswith("f "))
    return 2 * n, total - 2 * n


def unit_checks(cooker: types.ModuleType, payload: dict) -> None:
    import mvt  # tools/cooker, on sys.path through load_cooker

    for raw, metres in [("12", 12.0), ("12.5 m", 12.5), ("12m", 12.0), ("12,5", 12.5), ("40 ft", 12.192),
                        ("40'6\"", 12.3444), ("29 metres", 29.0)]:
        value = cooker.parse_length(raw)
        check(value is not None and abs(value - metres) < 1e-6, f"parse_length({raw!r}) = {metres} ({value})")
    for raw in ("tall", "20;25", "", "-3"):
        check(cooker.parse_length(raw) is None, f"parse_length({raw!r}) is not a length")

    extent = cooker.building_extent
    check(extent({"building:levels": "12"}) == (0.0, 36.0, "levels"), "12 levels: 36 m, Mapbox's 3 m a level")
    check(extent({"height": "29", "building:levels": "5"}) == (0.0, 29.0, "height"), "an explicit height wins")
    check(extent({"building:height": "21 m"}) == (0.0, 21.0, "height"), "building:height")
    check(extent({"building:levels": "6", "building:min_level": "2"}) == (6.0, 18.0, "levels"),
          "building:min_level raises the bottom")
    check(extent({"height": "30", "min_height": "10"}) == (10.0, 30.0, "height"), "min_height")
    check(extent({}) == (0.0, cooker.DEFAULT_HEIGHT_M, "default"), "no height: the default, and says so")
    check(extent({"height": "?", "building:levels": "3"}) == (0.0, 9.0, "levels"), "unparseable height: levels")

    square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
    halves = [[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0)], [(0.0, 0.0), (0.0, 10.0), (10.0, 10.0)]]
    rings = cooker.stitch_rings(halves)
    check(len(rings) == 1 and abs(abs(cooker.signed_area(rings[0])) - 100.0) < 1e-9,
          "two ways, one reversed, stitch into one ring")
    check(cooker.stitch_rings([[(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]]) == [], "an unclosable ring is dropped")
    clipped = cooker.clip_ring([(-10.0, -10.0), (20.0, -10.0), (20.0, 20.0), (-10.0, 20.0)], 0.0, 10.0)
    check(len(clipped) == 1 and abs(abs(cooker.signed_area(clipped[0])) - 100.0) < 1e-9, "clipped to the tile")
    # A footprint with a notch across the tile edge: two pieces below it, no wall across the gap
    # between them; one piece above it. 96000 m2 less the 44000 m2 notch, split 38400 / 13600.
    notched = [(100.0, 4000.0), (300.0, 4000.0), (300.0, 4110.0), (700.0, 4110.0), (700.0, 4000.0),
               (900.0, 4000.0), (900.0, 4120.0), (100.0, 4120.0)]
    below = cooker.clip_ring(notched, 0.0, 4096.0)
    above = cooker.clip_ring([(x, y - 4096.0) for x, y in notched], 0.0, 4096.0)
    check(sorted(round(abs(cooker.signed_area(p))) for p in below) == [19200, 19200]
          and all(len(p) == 4 and max(x for x, _ in p) - min(x for x, _ in p) == 200.0 for p in below),
          f"a notch across the tile edge: two 200 m wide pieces below, nothing across the gap ({below})")
    check(len(above) == 1 and round(abs(cooker.signed_area(above[0]))) == 13600, f"and one piece above ({above})")
    l_shape = [(0.0, 0.0), (10.0, 0.0), (10.0, 2.0), (2.0, 2.0), (2.0, 10.0), (0.0, 10.0)]
    check(cooker.point_in_ring(*cooker.interior_point(l_shape), l_shape), "interior point of an L")
    check(cooker.point_in_ring(*cooker.interior_point(square), square), "interior point of a square")

    # The fixture by the Mapbox rules: the tower part replaces its outline, the relation is there.
    prisms, counts = cooker.extract_buildings(payload, ANCHOR_LAT, ANCHOR_LON)
    check(len(prisms) == 3, f"three footprints from the fixture ({len(prisms)})")
    check(counts["outline_with_parts"] == 1 and counts["levels"] == 2 and counts["height"] == 1,
          f"counts: one outline replaced by its part, two by levels, one by height ({dict(counts)})")
    check(sorted(p.top_m for p in prisms) == [15.0, 29.0, 36.0], "fixture heights 15, 29 and 36 m")
    underground = {"elements": [{"type": "way", "id": 1, "tags": {"building": "yes", "location": "underground"},
                                 "geometry": [{"lat": 42.14, "lon": 24.76}, {"lat": 42.1401, "lon": 24.76},
                                              {"lat": 42.1401, "lon": 24.7601}, {"lat": 42.14, "lon": 24.76}]}]}
    prisms, counts = cooker.extract_buildings(underground, ANCHOR_LAT, ANCHOR_LON)
    check(not prisms and counts["underground"] == 1, "an underground building is left out")

    # The decoder against the encoder above, and Mapbox's flags.
    ring = [(100, 100), (300, 100), (300, 300), (100, 300)]
    hole = [(150, 150), (250, 150), (250, 250), (150, 250)]
    tile = encode_tile({"building": [
        ({"height": 36, "min_height": 3, "extrude": "true", "underground": "false", "type": "building:part"},
         [ring, hole]),
        ({"extrude": "false", "underground": "false", "type": "government"}, [ring]),
        ({"height": 10, "extrude": "true", "underground": "true"}, [ring]),
        ({"height": 12.5, "extrude": "true", "underground": "false", "note": -7}, [ring]),
    ]})
    layer = mvt.decode_layers(tile)["building"]
    check(layer.extent == 4096 and len(layer.features) == 4, "decoded layer and features")
    first = layer.features[0]
    check(first.properties.get("height") == 36 and first.properties.get("type") == "building:part",
          f"decoded properties ({first.properties})")
    check(layer.features[3].properties.get("note") == -7 and layer.features[3].properties.get("height") == 12.5,
          "decoded negative and double values")
    decoded = mvt.polygon_rings(first.geometry)
    check(len(decoded) == 2 and mvt.ring_area(decoded[0]) > 0 > mvt.ring_area(decoded[1]),
          "exterior ring then hole, told apart by their area")
    zoom, tx, ty = 16, 37278, 24254
    prisms, counts = cooker.extract_mapbox_buildings([(zoom, tx, ty, tile)], *cooker.tile_to_lat_lon(zoom, tx, ty),
                                                     radius_m=1000.0)
    check(len(prisms) == 2 and sorted(p.top_m for p in prisms) == [12.5, 36.0], "Mapbox heights as served")
    check(any(p.base_m == 3.0 for p in prisms), "Mapbox min_height is the bottom")
    check(counts["outline_with_parts"] == 1 and counts["underground"] == 1, f"extrude=false, underground ({counts})")

    # A vertex exactly on the tile edge: clipped without a doubled point, so the roof is there.
    seam = cooker.clip_ring([(4000.0, 0.0), (4096.0, 0.0), (4200.0, 0.0), (4200.0, 100.0), (4096.0, 100.0),
                             (4000.0, 100.0)], 0.0, 4096.0)
    walls, caps = faces(cooker, seam[0]) if len(seam) == 1 else (0, 0)
    check(len(seam) == 1 and len(seam[0]) == 4 and caps == 4, f"seam vertex: {seam}, {caps} cap triangles")
    walls, caps = faces(cooker, [(0.0, 0.0), (0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)])
    check(walls == 8 and caps == 4, f"repeated points dropped before extruding ({walls} walls, {caps} caps)")

    # One building in two tiles, each tile holding all of it (as Mapbox's buffer does): two pieces
    # that add up to the building, not two copies of it.
    zoom, tx, ty = 16, 37278, 24254
    lat0, lon0 = cooker.tile_to_lat_lon(zoom, tx + 1, ty)
    square = [(3996, 100), (4196, 100), (4196, 300), (3996, 300)]
    left = encode_tile({"building": [({"height": 20, "extrude": "true"}, [square])]})
    right = encode_tile({"building": [({"height": 20, "extrude": "true"}, [[(x - 4096, y) for x, y in square]])]})
    prisms, _ = cooker.extract_mapbox_buildings([(zoom, tx, ty, left), (zoom, tx + 1, ty, right)], lat0, lon0, 1000.0)
    corners = [cooker.tile_to_lat_lon(zoom, tx + x / 4096, ty + y / 4096) for x, y in square]
    whole = abs(cooker.signed_area([cooker.to_enu(lat, lon, lat0, lon0) for lat, lon in corners]))
    total = sum(abs(cooker.signed_area(p.ring)) for p in prisms)
    check(len(prisms) == 2 and abs(total - whole) < 0.01 * whole,
          f"a building across a tile edge: {len(prisms)} pieces, {total:.1f} of {whole:.1f} m2")
    try:
        cooker.extract_mapbox_buildings([(zoom, tx, ty, b"\x1a\x05\x0a\x03bu")], lat0, lon0, 1000.0)
        check(False, "a truncated tile is refused")
    except SystemExit as e:
        check("delete it from the cache" in str(e), f"a truncated tile names itself ({e})")

    # A part standing in a multipolygon's courtyard is not that building's part.
    def way(ring: list[tuple[float, float]]) -> list[dict]:
        return [{"lat": lat, "lon": lon} for lat, lon in [*ring, ring[0]]]

    outer = [(42.1400, 24.7600), (42.1400, 24.7610), (42.1410, 24.7610), (42.1410, 24.7600)]
    yard = [(42.1403, 24.7603), (42.1403, 24.7607), (42.1407, 24.7607), (42.1407, 24.7603)]
    kiosk = [(42.1404, 24.7604), (42.1404, 24.7606), (42.1406, 24.7606), (42.1406, 24.7604)]
    courtyard = {"elements": [
        {"type": "relation", "id": 1, "tags": {"type": "multipolygon", "building": "yes", "building:levels": "4"},
         "members": [{"type": "way", "ref": 2, "role": "outer", "geometry": way(outer)},
                     {"type": "way", "ref": 3, "role": "inner", "geometry": way(yard)}]},
        {"type": "way", "id": 4, "tags": {"building:part": "yes", "building:levels": "1"}, "geometry": way(kiosk)},
    ]}
    prisms, counts = cooker.extract_buildings(courtyard, ANCHOR_LAT, ANCHOR_LON)
    check(sorted(p.top_m for p in prisms) == [3.0, 12.0] and not counts["outline_with_parts"],
          f"a courtyard part leaves its building standing ({dict(counts)})")

    # Two outer rings, a part in one: only that ring gives way. A part that would not be drawn
    # (bottom at its top) replaces nothing.
    east = [(lat, lon + 0.002) for lat, lon in outer]
    tower = {"type": "way", "id": 5, "tags": {"building:part": "yes", "building:levels": "10"}, "geometry": way(kiosk)}
    two = {"elements": [
        {"type": "relation", "id": 6, "tags": {"type": "multipolygon", "building": "yes", "building:levels": "2"},
         "members": [{"type": "way", "ref": 7, "role": "outer", "geometry": way(outer)},
                     {"type": "way", "ref": 8, "role": "outer", "geometry": way(east)}]},
        tower,
    ]}
    prisms, counts = cooker.extract_buildings(two, ANCHOR_LAT, ANCHOR_LON)
    check(sorted(p.top_m for p in prisms) == [6.0, 30.0], f"a part replaces only its own outer ring ({prisms})")
    flat = dict(tower, tags={"building:part": "yes", "height": "10", "min_height": "10"})
    prisms, counts = cooker.extract_buildings({"elements": [two["elements"][0], flat]}, ANCHOR_LAT, ANCHOR_LON)
    check(sorted(p.top_m for p in prisms) == [6.0, 6.0] and counts["degenerate"] == 1,
          f"a part with no height replaces nothing ({dict(counts)})")


# ---------------------------------------------------------------------------- skysim


def stop(proc: subprocess.Popen) -> int:
    proc.send_signal(signal.SIGTERM)
    try:
        return proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        return proc.wait()


def mission_check(port: int, waypoints: list[list[float]]) -> dict:
    body = json.dumps({"waypoints_ned": waypoints, "clearance_m": 0.0}).encode()
    request = urllib.request.Request(f"http://127.0.0.1:{port}/mission/check", data=body, method="POST")
    with urllib.request.urlopen(request, timeout=15) as response:
        return json.loads(response.read().decode())


def roof(port: int, north: float, east: float) -> float | None:
    """The gateway's roof probe: straight down from 100 m to 1 m above the ground."""
    result = mission_check(port, [[north, east, -100.0], [north, east, -1.0]])
    check(result.get("checked") is True, "skysim had building geometry for the probe")
    hits = result.get("hits") or []
    return -hits[0]["position_ned"][2] if hits else None


def wall(port: int, north: float, east: float, height: float) -> bool:
    """The gateway's fan leg: level, at ``height``, from 150 m south up to the probe point."""
    result = mission_check(port, [[north - 150.0, east, -height], [north, east, -height]])
    return bool(result.get("hits"))


def probe_world(sim: str, tiles: Path, port: int, instance: int, label: str, probes: dict) -> None:
    proc = subprocess.Popen([sim, "--vehicles", "0", "--base-instance", str(instance), "--time-mode", "interactive",
                             "--api-port", str(port), "--tiles", str(tiles)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        deadline = time.time() + 30
        while True:
            try:
                urllib.request.urlopen(f"http://127.0.0.1:{port}/metrics", timeout=2).close()
                break
            except (urllib.error.URLError, ConnectionError):
                check(time.time() < deadline and proc.poll() is None, f"{label}: skysim answers")
                if FAILURES:
                    return
                time.sleep(0.2)

        for name, (north, east), expected in probes["buildings"]:
            top = roof(port, north, east)
            print(f"{label}: {name}: roof at {top if top is None else round(top, 2)} m, Mapbox {expected:g} m")
            check(top is not None and abs(top - expected) <= TOLERANCE_M,
                  f"{label}: {name}: roof {top} m within {TOLERANCE_M} m of {expected:g} m")
            check(wall(port, north, east, expected - 1.0), f"{label}: {name}: wall hit 1 m below the top")
            check(not wall(port, north, east, expected + 1.0), f"{label}: {name}: clear 1 m above the top")
        north, east = probes["outline"]
        top = roof(port, north, east)
        print(f"{label}: the tower's outline, off the tower: roof {top}")
        check(top is None, f"{label}: the outline whose part is drawn is not extruded ({top})")
    finally:
        check(stop(proc) == 0, f"{label}: skysim clean shutdown")


def cook(cooker_bin: str, args: list[str], obj: Path, tiles: Path, env: dict) -> dict:
    run = subprocess.run([sys.executable, str(COOKER_DIR / "osm_buildings.py"), str(obj), "--lat", str(ANCHOR_LAT),
                          "--lon", str(ANCHOR_LON), "--radius", str(RADIUS_M), *args],
                         capture_output=True, text=True, timeout=300, check=False, env=env)
    check(run.returncode == 0, f"osm_buildings.py {' '.join(args)}: {run.stdout[-400:]} {run.stderr[-400:]}")
    objs = sorted(str(p) for p in obj.glob("*.obj"))
    run = subprocess.run([cooker_bin, str(tiles), "--anchor", f"{ANCHOR_LAT},{ANCHOR_LON},0", *objs],
                         capture_output=True, text=True, timeout=300, check=False)
    check(run.returncode == 0, f"tile_cooker: {run.stderr.strip()}")
    return json.loads((obj / "anchor.json").read_text()) if (obj / "anchor.json").exists() else {}


def main() -> int:
    sim, cooker_bin = sys.argv[1], sys.argv[2]
    cooker = load_cooker()
    payload = json.loads(FIXTURE.read_text())
    elements = {(e["type"], e["id"]): e for e in payload["elements"]}

    unit_checks(cooker, payload)

    # Where to probe: each roof, and the tower's outline away from the tower.
    tower_outer, _ = footprint(elements[BUILDINGS[0][1]])

    def to_ned(ring: list[tuple[float, float]]) -> list[tuple[float, float]]:
        return [ned(lat, lon) for lat, lon in ring]

    buildings = []
    for name, key, expected in BUILDINGS:
        outer, holes = footprint(elements[key])
        buildings.append((name, probe_point(to_ned(outer), [to_ned(h) for h in holes], []), expected))
    outline_outer, _ = footprint(elements[OUTLINE])
    probes = {"buildings": buildings, "outline": probe_point(to_ned(outline_outer), [], [to_ned(tower_outer)])}

    # Mapbox's features for the same buildings: what its building layer serves for them.
    mapbox_features = []
    for key, props in [
        (OUTLINE, {"extrude": "false", "underground": "false", "type": "government"}),
        (BUILDINGS[0][1], {"extrude": "true", "underground": "false", "type": "building:part", "height": 36,
                           "min_height": 0, "building_id": 125444465}),
        (BUILDINGS[1][1], {"extrude": "true", "underground": "false", "type": "university", "height": 15,
                           "min_height": 0}),
        (BUILDINGS[2][1], {"extrude": "true", "underground": "false", "type": "building", "height": 29,
                           "min_height": 0}),
    ]:
        outer, holes = footprint(elements[key])
        mapbox_features.append((props, [outer, *holes]))

    env = {k: v for k, v in os.environ.items() if k != "MAPBOX_ACCESS_TOKEN"}  # never the network
    with tempfile.TemporaryDirectory(prefix="skysim_heights_") as tmp_name:
        tmp = Path(tmp_name)
        other = dict(payload, skysim_query={"lat": ANCHOR_LAT, "lon": ANCHOR_LON, "radius_m": 5000.0})
        (tmp / "other.json").write_text(json.dumps(other))
        refused = subprocess.run([sys.executable, str(COOKER_DIR / "osm_buildings.py"), str(tmp / "other_obj"),
                                  "--lat", str(ANCHOR_LAT), "--lon", str(ANCHOR_LON), "--radius", str(RADIUS_M),
                                  "--cache", str(tmp / "other.json")], capture_output=True, text=True, timeout=60,
                                 check=False, env=env)
        check(refused.returncode != 0 and "was fetched for" in refused.stderr,
              f"an Overpass cache of another area is refused ({refused.stderr.strip()})")
        anchor = cook(cooker_bin, ["--cache", str(FIXTURE)], tmp / "osm_obj", tmp / "osm_tiles", env)
        check(anchor.get("heights", {}).get("outline_with_parts") == 1, f"anchor.json counts ({anchor.get('heights')})")
        if not FAILURES:
            probe_world(sim, tmp / "osm_tiles", 18696, 52, "osm", probes)

        written = write_mapbox_tiles(cooker, tmp / "mapbox_cache", mapbox_features)
        check(written >= 3, f"Mapbox tiles with buildings ({written})")
        # Clipped to their tiles, the pieces of each building add up to its footprint.
        tiles = cooker.load_mapbox_tiles(ANCHOR_LAT, ANCHOR_LON, RADIUS_M, None, cooker.MAPBOX_TILESET,
                                         str(tmp / "mapbox_cache"))
        pieces, _ = cooker.extract_mapbox_buildings(tiles, ANCHOR_LAT, ANCHOR_LON, RADIUS_M)
        for name, key, expected in BUILDINGS:
            outer, _ = footprint(elements[key])
            area = abs(cooker.signed_area([cooker.to_enu(lat, lon, ANCHOR_LAT, ANCHOR_LON) for lat, lon in outer]))
            got = sum(abs(cooker.signed_area(p.ring)) for p in pieces if p.top_m == expected)
            check(abs(got - area) < 0.02 * area, f"mapbox: {name}: {got:.0f} of {area:.0f} m2 of footprint")
        anchor = cook(cooker_bin, ["--source", "mapbox", "--cache", str(tmp / "mapbox_cache")], tmp / "mapbox_obj",
                      tmp / "mapbox_tiles", env)
        check(anchor.get("heights", {}).get("mapbox", 0) >= 3, f"anchor.json counts ({anchor.get('heights')})")
        if not FAILURES:
            probe_world(sim, tmp / "mapbox_tiles", 18697, 53, "mapbox", probes)

    if FAILURES:
        print(f"{len(FAILURES)} of {CHECKS[0]} checks failed")
        return 1
    print(f"building heights: OK ({CHECKS[0]} checks)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
