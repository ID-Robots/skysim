#!/usr/bin/env python3
"""Survey 3D scans -> skysim building tiles, merged with the OSM / Mapbox prisms (TASK-2735).

Where a survey has been flown and reconstructed, skysim should fly against the scan rather than
the extruded map footprints. This takes one or more OpenDroneMap models plus a building tile set
from ``osm_buildings.py`` and writes the merged per-tile OBJ set for ``tile_cooker``:

* each scan is moved into skysim's map frame (ENU metres about the anchor, the SITL ``--home``),
  decimated into a collision mesh, and cut to the 250 m tile grid;
* every OSM prism whose whole footprint lies inside a scan's ground coverage is dropped (the
  scan has that building); every other prism is kept as it is, so the world outside the
  scanned areas does not change.

Scan input (``--scan``, repeatable), each one of:

* an ODM project directory, or NodeODM's ``all.zip``: the model is the first of
  ``odm_texturing/odm_textured_model_geo.{obj,glb}`` then ``odm_texturing_25d/...``, and its
  georeference ``odm_georeferencing/coords.txt``;
* a model file (``.obj`` or ``.glb``) with ``coords.txt`` beside it, or in
  ``../odm_georeferencing/``;
* any of those with ``@LAT,LON`` appended: the model's origin in degrees, for a model with no
  coords.txt. That is what the gateway stores for a reconstruction (``model_origin`` beside
  ``model.glb`` or the smaller ``model_map.glb`` — same frame, coords.txt itself is not kept).
  The UTM zone is the one in which the origin is the GLB's ``CESIUM_RTC`` centre, else the
  standard zone of its longitude; ``@LAT,LON,34N`` names it. With a coords.txt as well, the
  coords.txt grid is used and the origin only checked against it.

ODM's georeferenced model is metres east (x), north (y) and up (z) of a UTM point (coords.txt:
the projection, then that easting and northing; the GLB's ``CESIUM_RTC`` centre). Every vertex
goes UTM -> WGS84 -> ENU about the anchor by the same equirectangular projection
``osm_buildings.py`` uses, so the grid convergence (~1.5 degrees in Plovdiv) and the UTM scale
are taken out and a scan lines up with the prisms around it. Heights: skysim's ground is the
plane z = 0, so the scan's own ground level (``--ground-z``; by default the height below which
a fifth of its level, upward-facing surface lies) is moved to 0.

Collision mesh, per scan:

1. Orientation: mesh collision is one-sided, so faces must point into the air. The mesh's
   signed volume over a plane below it says which way it is wound; a negative one (inside out)
   is flipped whole. ``--two-sided`` writes every triangle both ways for a mesh whose faces are
   not consistently wound at all.
2. Decimation by vertex clustering: vertices are merged per ``--cell`` cube (1 m) to the mean of
   their members; triangles that collapse are dropped, the rest keep their winding. Each
   vertex moves at most a cell diagonal (1.7 m at 1 m; a plane's members average out, and the
   ODM fixture's walls stay a median 0.13 m from the truth).
   Over ``--max-triangles`` the cell grows by half until under it (an error past 10 m).
3. Ground: triangles wholly below ``--ground-cut`` (1 m) are dropped. skysim has its own ground
   slab, and a landing on a ground triangle of a tile would count as a building strike.
4. Tiles: each triangle goes whole to the tile holding its centroid, like a whole building
   in ``osm_buildings.py``; the streamer keeps neighbouring tiles resident.

Coverage (which OSM prisms the scan replaces): every clustered vertex of the scan — ground,
roofs, walls, trees — rasterised on ``--coverage-cell`` (2 m) squares, gaps of up to two cells closed, then shrunk by
``--margin`` (5 m, rounded up to whole cells) — the edge of a reconstruction is its worst part. A
prism is dropped only when every square its footprint touches is covered, and only by a scan that
kept triangles.

Usage:
  python3 tools/cooker/osm_buildings.py osm_obj --lat 42.1403890 --lon 24.7645490 --radius 5000
  python3 tools/cooker/scan_tiles.py world_obj --buildings osm_obj \\
      --scan odm_project/ --scan model.glb@42.1449,24.7469
  ./build/tile_cooker world_tiles --anchor 42.1403890,24.7645490,0 world_obj/*.obj

Limits:
  - ODM models only: a UTM frame with a coords.txt offset (or an origin). Other georeferences
    would need their own reader.
  - GLB: core glTF 2.0, float positions, triangle lists. ODM's own GLB is Draco-compressed
    (its obj2glb default), read only with the DracoPy package (and numpy) installed; otherwise
    pass the ODM OBJ or the gateway's ``model_map.glb``.
  - The mesh is an open surface (ODM reconstructs what the cameras saw): walls no camera saw are
    missing, so a vehicle can pass through them. Courtyards, eaves and trees are kept as seen.
    The demo-city survey fixture (nadir + one -60 degree grid, ODM 3.x, 298 533 triangles ->
    78 687 at 1 m) puts roofs within 0.2 m and walls a median 0.13 m from the true boxes, but
    13 of 60 wall probes found no wall: the faces no oblique looked at.
  - Coverage is where the scan has any vertex (ground or roof), not which walls it saw: a
    building at the edge of a survey is replaced even when its outward wall is missing. Fly a building beyond the area
    to replace, or raise ``--margin``.
  - skysim's ground is flat. A scan on a slope keeps its relief: ground more than
    ``--ground-cut`` above its ground level stays as geometry, and ground below it is not.
  - Pure Python: a 200 000-triangle ODM model (ODM's default ``--mesh-size``) takes seconds.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import struct
import sys
import zipfile
from array import array
from dataclasses import dataclass, field
from pathlib import Path

import osm_buildings  # tools/cooker/osm_buildings.py: the same ENU projection and tile names

Vec3 = tuple[float, float, float]
Tri = tuple[int, int, int]

ODM_MODELS = (
    "odm_texturing/odm_textured_model_geo.obj",
    "odm_texturing/odm_textured_model_geo.glb",
    "odm_texturing_25d/odm_textured_model_geo.obj",
    "odm_texturing_25d/odm_textured_model_geo.glb",
)
ODM_COORDS = "odm_georeferencing/coords.txt"

# WGS84 and the UTM constants
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)
_EP2 = _E2 / (1.0 - _E2)
_K0 = 0.9996

DEFAULT_CELL_M = 1.0
DEFAULT_MAX_TRIANGLES = 300_000
MAX_CELL_M = 10.0  # the coarsest decimation --max-triangles may grow the cell to
DEFAULT_GROUND_CUT_M = 1.0
DEFAULT_COVERAGE_CELL_M = 2.0
DEFAULT_MARGIN_M = 5.0
GROUND_PERCENTILE = 0.2  # of the level, upward-facing area
LEVEL_NZ = 0.9  # |normal z| of a "level" face


class ScanError(Exception):
    """A scan that cannot be read or placed, in words fit for the command line."""


# ---------------------------------------------------------------------------- georeference


def utm_to_latlon(easting: float, northing: float, zone: int, northern: bool) -> tuple[float, float]:
    """Inverse transverse Mercator on WGS84 (Snyder, *Map Projections*, eqs. 8-12 to 8-25)."""
    x = easting - 500000.0
    y = northing if northern else northing - 10000000.0
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)
    e1 = (1.0 - math.sqrt(1.0 - _E2)) / (1.0 + math.sqrt(1.0 - _E2))
    mu = y / _K0 / (_A * (1.0 - _E2 / 4.0 - 3.0 * _E2**2 / 64.0 - 5.0 * _E2**3 / 256.0))
    phi1 = (
        mu
        + (3.0 * e1 / 2.0 - 27.0 * e1**3 / 32.0) * math.sin(2.0 * mu)
        + (21.0 * e1**2 / 16.0 - 55.0 * e1**4 / 32.0) * math.sin(4.0 * mu)
        + (151.0 * e1**3 / 96.0) * math.sin(6.0 * mu)
        + (1097.0 * e1**4 / 512.0) * math.sin(8.0 * mu)
    )
    sin1, cos1, tan1 = math.sin(phi1), math.cos(phi1), math.tan(phi1)
    n1 = _A / math.sqrt(1.0 - _E2 * sin1**2)
    t1 = tan1**2
    c1 = _EP2 * cos1**2
    r1 = _A * (1.0 - _E2) / (1.0 - _E2 * sin1**2) ** 1.5
    d = x / (n1 * _K0)
    lat = phi1 - (n1 * tan1 / r1) * (
        d**2 / 2.0
        - (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * c1**2 - 9.0 * _EP2) * d**4 / 24.0
        + (61.0 + 90.0 * t1 + 298.0 * c1 + 45.0 * t1**2 - 252.0 * _EP2 - 3.0 * c1**2) * d**6 / 720.0
    )
    lon = (
        lon0
        + (
            d
            - (1.0 + 2.0 * t1 + c1) * d**3 / 6.0
            + (5.0 - 2.0 * c1 + 28.0 * t1 - 3.0 * c1**2 + 8.0 * _EP2 + 24.0 * t1**2) * d**5 / 120.0
        )
        / cos1
    )
    return math.degrees(lat), math.degrees(lon)


def latlon_to_utm(lat: float, lon: float, zone: int | None = None) -> tuple[float, float, int, bool]:
    """Forward transverse Mercator: ``(easting, northing, zone, northern)``; the standard zone
    for ``lon`` unless one is given (ODM picks the standard zone of its images)."""
    zone = int(zone or (math.floor((lon + 180.0) / 6.0) % 60) + 1)
    phi = math.radians(lat)
    lam = math.radians(lon)
    lam0 = math.radians((zone - 1) * 6 - 180 + 3)
    n = _A / math.sqrt(1.0 - _E2 * math.sin(phi) ** 2)
    t = math.tan(phi) ** 2
    c = _EP2 * math.cos(phi) ** 2
    a = math.cos(phi) * ((lam - lam0 + math.pi) % (2.0 * math.pi) - math.pi)  # the short way round the antimeridian
    m = _A * (
        (1.0 - _E2 / 4.0 - 3.0 * _E2**2 / 64.0 - 5.0 * _E2**3 / 256.0) * phi
        - (3.0 * _E2 / 8.0 + 3.0 * _E2**2 / 32.0 + 45.0 * _E2**3 / 1024.0) * math.sin(2.0 * phi)
        + (15.0 * _E2**2 / 256.0 + 45.0 * _E2**3 / 1024.0) * math.sin(4.0 * phi)
        - (35.0 * _E2**3 / 3072.0) * math.sin(6.0 * phi)
    )
    easting = (
        _K0 * n * (a + (1.0 - t + c) * a**3 / 6.0 + (5.0 - 18.0 * t + t**2 + 72.0 * c - 58.0 * _EP2) * a**5 / 120.0)
    )
    northing = _K0 * (
        m
        + n
        * math.tan(phi)
        * (
            a**2 / 2.0
            + (5.0 - t + 9.0 * c + 4.0 * c**2) * a**4 / 24.0
            + (61.0 - 58.0 * t + t**2 + 600.0 * c - 330.0 * _EP2) * a**6 / 720.0
        )
    )
    northern = lat >= 0.0
    if not northern:
        northing += 10000000.0
    return easting + 500000.0, northing, zone, northern


@dataclass(frozen=True)
class Georef:
    """Where a model's (0, 0) is: a UTM easting and northing in a zone."""

    zone: int
    northern: bool
    easting: float
    northing: float
    source: str

    def to_enu(self, x: float, y: float, anchor_lat: float, anchor_lon: float) -> tuple[float, float]:
        lat, lon = utm_to_latlon(self.easting + x, self.northing + y, self.zone, self.northern)
        lon = anchor_lon + (lon - anchor_lon + 180.0) % 360.0 - 180.0  # the short way round the antimeridian
        return osm_buildings.to_enu(lat, lon, anchor_lat, anchor_lon)


def parse_coords(text: str) -> Georef:
    """ODM's coords.txt: the projection ("WGS84 UTM 35N" or "EPSG:32635"), then easting northing."""
    lines = [line.strip() for line in text.splitlines() if line.strip()]
    if len(lines) < 2:
        raise ScanError("coords.txt: expected a projection line and an offset line")
    zone: tuple[int, bool] | None = None
    match = re.search(r"UTM\s*(\d{1,2})\s*([NS])", lines[0], re.IGNORECASE)
    if match:
        zone = int(match.group(1)), match.group(2).upper() == "N"
    else:
        match = re.search(r"EPSG:\s*(\d+)", lines[0], re.IGNORECASE)
        epsg = int(match.group(1)) if match else 0
        if 32601 <= epsg <= 32660:
            zone = epsg - 32600, True
        elif 32701 <= epsg <= 32760:
            zone = epsg - 32700, False
    if zone is None or not 1 <= zone[0] <= 60:
        raise ScanError(f"coords.txt: not a WGS84 UTM projection: {lines[0]!r}")
    try:
        easting, northing = (float(v) for v in lines[1].split()[:2])
    except ValueError:
        raise ScanError(f"coords.txt: bad offset line {lines[1]!r}") from None
    if not (math.isfinite(easting) and math.isfinite(northing)):
        raise ScanError(f"coords.txt: bad offset line {lines[1]!r}")
    return Georef(zone[0], zone[1], easting, northing, "coords.txt")


def georef_from_origin(lat: float, lon: float, zone: int | None = None, northern: bool | None = None) -> Georef:
    """The model's origin in degrees (the gateway's ``model_origin``) as a UTM offset: in ``zone``,
    else the standard zone of ``lon`` (ODM's zone is its first image's, which can be the
    neighbouring one near a zone edge)."""
    easting, northing, zone, north = latlon_to_utm(lat, lon, zone)
    if northern is not None and northern != north:
        northing += -10000000.0 if northern else 10000000.0
        north = northern
    return Georef(zone, north, easting, northing, f"origin {lat:.8f},{lon:.8f} (UTM {zone}{'N' if north else 'S'})")


# ---------------------------------------------------------------------------- model readers


def read_obj(text: str) -> tuple[list[Vec3], list[Tri]]:
    """'v x y z' and 'f a b c ...' (1-based or negative, optional /vt/vn, polygons fanned)."""
    verts: list[Vec3] = []
    faces: list[Tri] = []
    for line_no, line in enumerate(text.splitlines(), 1):
        if line.startswith("v "):
            parts = line.split()
            try:
                verts.append((float(parts[1]), float(parts[2]), float(parts[3])))
            except (IndexError, ValueError):
                raise ScanError(f"OBJ line {line_no}: bad vertex") from None
        elif line.startswith("f "):
            idx = []
            for tok in line.split()[1:]:
                try:
                    v = int(tok.split("/", 1)[0])
                except ValueError:
                    raise ScanError(f"OBJ line {line_no}: bad face") from None
                v = v - 1 if v > 0 else len(verts) + v
                if not 0 <= v < len(verts):
                    raise ScanError(f"OBJ line {line_no}: face index out of range")
                idx.append(v)
            faces.extend((idx[0], idx[k], idx[k + 1]) for k in range(1, len(idx) - 1))
    return verts, faces


_IDENTITY = (1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
_COMPONENTS = {5121: ("B", 1), 5123: ("H", 2), 5125: ("I", 4), 5126: ("f", 4)}
_WIDTHS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


def _node_matrix(node: dict) -> tuple[float, ...]:
    """A glTF node's local transform, column-major."""
    if "matrix" in node:
        return tuple(float(v) for v in node["matrix"])
    tx, ty, tz = (float(v) for v in node.get("translation", (0, 0, 0)))
    qx, qy, qz, qw = (float(v) for v in node.get("rotation", (0, 0, 0, 1)))
    sx, sy, sz = (float(v) for v in node.get("scale", (1, 1, 1)))
    r = (
        (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
        (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
        (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)),
    )
    s = (sx, sy, sz)
    m = [0.0] * 16
    for col in range(3):
        for row in range(3):
            m[col * 4 + row] = r[row][col] * s[col]
    m[12], m[13], m[14], m[15] = tx, ty, tz, 1.0
    return tuple(m)


def _multiply(a: tuple[float, ...], b: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(sum(a[k * 4 + row] * b[col * 4 + k] for k in range(4)) for col in range(4) for row in range(4))


def _accessor(doc: dict, binary: bytes, index: int) -> array:
    acc = doc["accessors"][index]
    if "sparse" in acc or acc.get("normalized"):
        raise ScanError("GLB: sparse or normalized (quantized) accessors are not supported")
    if acc["componentType"] not in _COMPONENTS or acc["type"] not in _WIDTHS:
        raise ScanError(f"GLB: accessor type {acc['componentType']}/{acc['type']} not supported")
    fmt, size = _COMPONENTS[acc["componentType"]]
    width, count = _WIDTHS[acc["type"]], int(acc["count"])
    out = array(fmt)
    if "bufferView" not in acc:
        out.extend([0] * (count * width))
        return out
    view = doc["bufferViews"][acc["bufferView"]]
    if view.get("buffer", 0) != 0:
        raise ScanError("GLB: geometry in an external buffer")
    start = int(view.get("byteOffset", 0)) + int(acc.get("byteOffset", 0))
    stride = int(view.get("byteStride") or size * width)
    end = start + stride * (count - 1) + size * width if count else start
    if end > int(view.get("byteOffset", 0)) + int(view["byteLength"]) or end > len(binary):
        raise ScanError("GLB: an accessor runs past its buffer")
    if stride == size * width:
        out.frombytes(binary[start:end])
    else:
        for i in range(count):
            out.extend(struct.unpack_from(f"<{width}{fmt}", binary, start + i * stride))
    if sys.byteorder == "big":
        out.byteswap()
    return out


def read_glb(data: bytes) -> tuple[list[Vec3], list[Tri], dict]:
    """Every triangle a GLB's default scene draws, node transforms baked in.

    Returns (vertices, faces, top-level extensions)."""
    if len(data) < 12:
        raise ScanError("GLB: too short")
    magic, version, length = struct.unpack_from("<4sII", data, 0)
    if magic != b"glTF" or version != 2:
        raise ScanError("not a glTF 2.0 binary")
    doc: dict | None = None
    binary = b""
    offset = 12
    while offset + 8 <= min(length, len(data)):
        chunk_len, chunk_type = struct.unpack_from("<II", data, offset)
        chunk = data[offset + 8 : offset + 8 + chunk_len]
        offset += 8 + chunk_len
        if chunk_type == 0x4E4F534A and doc is None:  # JSON
            doc = json.loads(chunk.decode("utf-8"))
        elif chunk_type == 0x004E4942 and not binary:  # BIN
            binary = chunk
    if doc is None:
        raise ScanError("GLB: no JSON chunk")

    nodes = doc.get("nodes") or []
    scenes = doc.get("scenes") or []
    if scenes:
        roots = scenes[doc.get("scene", 0)].get("nodes") or []
    else:
        children = {c for node in nodes for c in node.get("children") or []}
        roots = [i for i in range(len(nodes)) if i not in children]
    verts: list[Vec3] = []
    faces: list[Tri] = []
    stack = [(root, _IDENTITY, 0) for root in roots]
    while stack:
        index, parent, depth = stack.pop()
        if depth > 64:
            raise ScanError("GLB: node tree too deep (a cycle?)")
        node = nodes[index]
        m = _multiply(parent, _node_matrix(node))
        stack.extend((child, m, depth + 1) for child in node.get("children") or [])
        if "mesh" not in node:
            continue
        det = (
            m[0] * (m[5] * m[10] - m[9] * m[6])
            - m[4] * (m[1] * m[10] - m[9] * m[2])
            + m[8] * (m[1] * m[6] - m[5] * m[2])
        )
        for prim in doc["meshes"][node["mesh"]].get("primitives") or []:
            mode = prim.get("mode", 4)
            if mode in (5, 6):
                raise ScanError("GLB: triangle strips and fans are not supported")
            if mode != 4:
                continue  # points / lines: nothing to collide with
            draco = (prim.get("extensions") or {}).get("KHR_draco_mesh_compression")
            if draco:
                positions, indices = _draco(doc, binary, draco)
            else:
                at = doc["accessors"][prim["attributes"]["POSITION"]]
                if at.get("type") != "VEC3" or at.get("componentType") != 5126:
                    raise ScanError("GLB: POSITION must be float VEC3")
                if "indices" in prim:
                    at = doc["accessors"][prim["indices"]]
                    if at.get("type") != "SCALAR" or at.get("componentType") not in (5121, 5123, 5125):
                        raise ScanError("GLB: indices must be unsigned SCALAR")
                positions = _accessor(doc, binary, prim["attributes"]["POSITION"])
                indices = (
                    _accessor(doc, binary, prim["indices"])
                    if "indices" in prim
                    else array("I", range(len(positions) // 3))
                )
            base = len(verts)
            for i in range(0, len(positions) - 2, 3):
                x, y, z = positions[i], positions[i + 1], positions[i + 2]
                verts.append(
                    (
                        m[0] * x + m[4] * y + m[8] * z + m[12],
                        m[1] * x + m[5] * y + m[9] * z + m[13],
                        m[2] * x + m[6] * y + m[10] * z + m[14],
                    )
                )
            nverts = len(verts) - base
            for i in range(0, len(indices) - 2, 3):
                a, b, c = indices[i], indices[i + 1], indices[i + 2]
                if max(a, b, c) >= nverts:
                    raise ScanError("GLB: a triangle index points past its vertices")
                # A mirroring node flips which side glTF shows: keep the same side outward.
                faces.append((base + a, base + c, base + b) if det < 0 else (base + a, base + b, base + c))
    return verts, faces, doc.get("extensions") or {}


def _draco(doc: dict, binary: bytes, ext: dict) -> tuple[list[float], list[int]]:
    try:
        import DracoPy  # type: ignore[import-not-found]  # optional
    except ImportError:
        raise ScanError(
            "GLB is Draco-compressed: install DracoPy, or pass the ODM OBJ "
            "(odm_texturing/odm_textured_model_geo.obj) or the gateway's model_map.glb"
        ) from None
    view = doc["bufferViews"][ext["bufferView"]]
    start = int(view.get("byteOffset", 0))
    mesh = DracoPy.decode(binary[start : start + int(view["byteLength"])])
    return [float(v) for v in list(mesh.points.reshape(-1))], [int(v) for v in list(mesh.faces.reshape(-1))]


@dataclass
class Scan:
    """One model as read: vertices and faces in its own frame, and where that frame is."""

    name: str
    verts: list[Vec3]
    faces: list[Tri]
    georef: Georef


def _model_bytes(path: Path) -> tuple[str, bytes, str | None]:
    """(model name, model bytes, coords.txt text or None) for a project dir, zip or model file."""
    if path.is_dir():
        for rel in ODM_MODELS:
            if (path / rel).is_file():
                coords = path / ODM_COORDS
                return str(path / rel), (path / rel).read_bytes(), coords.read_text() if coords.is_file() else None
        raise ScanError(f"{path}: no ODM textured model in it ({', '.join(ODM_MODELS)})")
    if path.suffix.lower() == ".zip":
        with zipfile.ZipFile(path) as bundle:
            names = set(bundle.namelist())
            for rel in ODM_MODELS:
                if rel in names:
                    coords = bundle.read(ODM_COORDS).decode(errors="replace") if ODM_COORDS in names else None
                    return f"{path}:{rel}", bundle.read(rel), coords
        raise ScanError(f"{path}: no ODM textured model in the archive")
    if path.suffix.lower() in (".obj", ".glb") and path.is_file():
        for coords in (path.parent / "coords.txt", path.parent.parent / ODM_COORDS):
            if coords.is_file():
                return str(path), path.read_bytes(), coords.read_text()
        return str(path), path.read_bytes(), None
    raise ScanError(f"{path}: not an ODM project directory, all.zip, .obj or .glb")


def load_scan(spec: str) -> Scan:
    """``PATH``, ``PATH@LAT,LON`` or ``PATH@LAT,LON,ZONE`` (see the module docstring)."""
    origin = None
    zone: tuple[int, bool] | None = None
    match = re.fullmatch(r"(.+)@\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*(?:,\s*(\d{1,2})\s*([NSns])\s*)?", spec)
    if match:
        spec, origin = match.group(1), (float(match.group(2)), float(match.group(3)))
        if not (-90.0 <= origin[0] <= 90.0 and -180.0 <= origin[1] <= 180.0):
            raise ScanError(f"{spec}: origin {origin} is not a latitude,longitude")
        if match.group(4):
            zone = int(match.group(4)), match.group(5).upper() == "N"
            if not 1 <= zone[0] <= 60:
                raise ScanError(f"{spec}: UTM zone {zone[0]} does not exist")
    name, data, coords = _model_bytes(Path(spec))
    if data[:4] == b"glTF":
        verts, faces, extensions = read_glb(data)
        centre = (extensions.get("CESIUM_RTC") or {}).get("center")
        centre = tuple(float(c) for c in centre[:2]) if centre and len(centre) >= 2 else None
    else:
        verts, faces = read_obj(data.decode("utf-8", errors="replace"))
        centre = None

    if coords is not None:
        # The model's own grid: coords.txt's zone and offset, whatever an origin says.
        georef = parse_coords(coords)
        if origin is not None:
            there = georef_from_origin(*origin, georef.zone, georef.northern)
            if math.hypot(there.easting - georef.easting, there.northing - georef.northing) > 1.0:
                raise ScanError(f"{name}: @{origin[0]},{origin[1]} is not the coords.txt offset ({georef.source})")
    elif origin is not None:
        if zone is None and centre is not None:
            # ODM writes its UTM offset as the RTC centre: the zone in which the origin is that point.
            std = latlon_to_utm(*origin)[2]
            zone = next(
                (
                    (z, origin[0] >= 0.0)
                    for z in (std, std % 60 + 1, (std - 2) % 60 + 1)
                    if math.dist(latlon_to_utm(*origin, z)[:2], centre) <= 1.0
                ),
                None,
            )
        georef = georef_from_origin(*origin, *(zone or (None, None)))
    else:
        raise ScanError(f"{name}: no coords.txt found; append @LAT,LON[,ZONE] (the model's origin)")
    if centre is not None and math.hypot(centre[0] - georef.easting, centre[1] - georef.northing) > 1.0:
        # A georeference that disagrees with the model's own is the wrong one.
        raise ScanError(
            f"{name}: CESIUM_RTC centre {centre[0]:.1f} {centre[1]:.1f} is not the {georef.source} offset "
            f"{georef.easting:.1f} {georef.northing:.1f}"
        )
    if not faces:
        raise ScanError(f"{name}: no triangles")
    if not all(math.isfinite(c) for v in verts for c in v):
        raise ScanError(f"{name}: vertices that are not numbers")
    return Scan(name, verts, faces, georef)


# ---------------------------------------------------------------------------- collision mesh


def _cross_z_and_mean_z(verts: list[Vec3], face: Tri) -> tuple[float, float, float]:
    """(twice the signed projected area, mean height, |normal|) of one triangle."""
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = verts[face[0]], verts[face[1]], verts[face[2]]
    ux, uy, uz = bx - ax, by - ay, bz - az
    vx, vy, vz = cx - ax, cy - ay, cz - az
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    return nz, (az + bz + cz) / 3.0, math.sqrt(nx * nx + ny * ny + nz * nz)


def is_inside_out(verts: list[Vec3], faces: list[Tri]) -> bool:
    """True when the mesh's signed volume over a plane below it is negative: its faces point
    down / inward. Positive for an upward-facing surface and for a closed outward shell alike."""
    floor_z = min(v[2] for v in verts) - 1.0
    volume = 0.0
    for face in faces:
        nz, mean_z, _ = _cross_z_and_mean_z(verts, face)
        volume += nz * (mean_z - floor_z)
    return volume < 0.0


def ground_level(verts: list[Vec3], faces: list[Tri], percentile: float = GROUND_PERCENTILE) -> float:
    """The height below which ``percentile`` of the mesh's level, upward-facing area lies."""
    level = []
    for face in faces:
        nz, mean_z, norm = _cross_z_and_mean_z(verts, face)
        if norm > 0.0 and nz >= LEVEL_NZ * norm:
            level.append((mean_z, nz))
    if not level:
        raise ScanError("no level, upward-facing surface to find the ground from: give --ground-z")
    level.sort()
    goal = percentile * sum(a for _, a in level)
    run = 0.0
    for z, a in level:
        run += a
        if run >= goal:
            return z
    return level[-1][0]


def cluster(verts: list[Vec3], faces: list[Tri], cell: float) -> tuple[list[Vec3], list[Tri]]:
    """Vertex-clustering decimation: one vertex per occupied ``cell`` cube (the mean of its
    members); collapsed and duplicate triangles dropped, winding kept."""
    key_of: dict[tuple[int, int, int], int] = {}
    sums: list[list[float]] = []
    remap = [0] * len(verts)
    for i, (x, y, z) in enumerate(verts):
        key = (math.floor(x / cell), math.floor(y / cell), math.floor(z / cell))
        k = key_of.get(key)
        if k is None:
            k = key_of[key] = len(sums)
            sums.append([0.0, 0.0, 0.0, 0.0])
        s = sums[k]
        s[0] += x
        s[1] += y
        s[2] += z
        s[3] += 1.0
        remap[i] = k
    out_faces: list[Tri] = []
    seen: set[Tri] = set()
    for a, b, c in faces:
        a, b, c = remap[a], remap[b], remap[c]
        if a == b or b == c or a == c:
            continue
        lowest = min(a, b, c)  # rotation-invariant key: the same triangle the same way round
        key3 = (a, b, c) if a == lowest else (b, c, a) if b == lowest else (c, a, b)
        if key3 in seen:
            continue
        seen.add(key3)
        out_faces.append((a, b, c))
    return [(s[0] / s[3], s[1] / s[3], s[2] / s[3]) for s in sums], out_faces


def _dilate(cells: set[tuple[int, int]], r: int) -> set[tuple[int, int]]:
    return {(i + di, j + dj) for i, j in cells for di in range(-r, r + 1) for dj in range(-r, r + 1)}


def _erode(cells: set[tuple[int, int]], r: int) -> set[tuple[int, int]]:
    offsets = [(di, dj) for di in range(-r, r + 1) for dj in range(-r, r + 1)]
    return {(i, j) for i, j in cells if all((i + di, j + dj) in cells for di, dj in offsets)}


@dataclass
class Coverage:
    """The ground a scan covers: squares of ``cell`` metres in the ENU map frame."""

    cell: float
    cells: set[tuple[int, int]] = field(default_factory=set)

    def covers(self, x: float, y: float) -> bool:
        return (math.floor(x / self.cell), math.floor(y / self.cell)) in self.cells

    @property
    def area_m2(self) -> float:
        return len(self.cells) * self.cell * self.cell


def coverage_of(points: list[tuple[float, float]], cell: float, margin: float) -> Coverage:
    """Squares holding a point; gaps of up to two squares closed (dilate by one, erode by one), then
    shrunk by ``margin`` rounded up to whole squares."""
    cells = {(math.floor(x / cell), math.floor(y / cell)) for x, y in points}
    shrink = 1 + math.ceil(margin / cell)  # undo the closing's dilation, then the margin
    return Coverage(cell, _erode(_dilate(cells, 1), shrink))


@dataclass
class ScanMesh:
    """A scan ready for the tiles: triangles in skysim ENU (z up from its ground), and stats."""

    triangles: list[tuple[Vec3, Vec3, Vec3]]
    coverage: Coverage
    stats: dict


def build_scan_mesh(
    scan: Scan,
    anchor_lat: float,
    anchor_lon: float,
    *,
    cell: float = DEFAULT_CELL_M,
    max_triangles: int = DEFAULT_MAX_TRIANGLES,
    ground_z: float | None = None,
    ground_cut: float = DEFAULT_GROUND_CUT_M,
    coverage_cell: float = DEFAULT_COVERAGE_CELL_M,
    margin: float = DEFAULT_MARGIN_M,
    two_sided: bool = False,
) -> ScanMesh:
    faces = scan.faces
    flipped = is_inside_out(scan.verts, faces)
    if flipped:
        faces = [(a, c, b) for a, b, c in faces]
    ground = ground_level(scan.verts, faces) if ground_z is None else ground_z

    # Coarser than this a collision mesh no longer has a building's shape, and the cell would grow
    # until every face collapsed (an empty mesh that meets any budget).
    largest = max(MAX_CELL_M, cell)
    start_cell = cell
    while True:  # in the model's own frame: UTM grid metres, which are metres to within 0.04 %
        verts, clustered = cluster(scan.verts, faces, cell)
        kept = [
            f
            for f in clustered
            if max(verts[f[0]][2], verts[f[1]][2], verts[f[2]][2]) - ground >= ground_cut
            and _cross_z_and_mean_z(verts, f)[2] > 2e-6  # more than a square millimetre: not a line
        ]
        if max_triangles <= 0 or len(kept) * (2 if two_sided else 1) <= max_triangles:
            if kept or cell == start_cell:
                break
            raise ScanError(f"--max-triangles {max_triangles} is met only by collapsing the scan to nothing")
        if cell >= largest:
            raise ScanError(
                f"cannot decimate to --max-triangles {max_triangles} with cells up to {largest:g} m: "
                f"{len(kept)} triangles left"
            )
        cell = min(cell * 1.5, largest)  # the last try is the largest cell itself

    # Into the map frame, every clustered vertex (the coverage needs the ground's too).
    enu = []
    for x, y, z in verts:
        e, n = scan.georef.to_enu(x, y, anchor_lat, anchor_lon)
        enu.append((e, n, z - ground))
    # Clustered vertices are about a cell apart: a finer coverage grid would be all gaps.
    coverage = coverage_of([(e, n) for e, n, _ in enu], max(coverage_cell, cell), margin)
    triangles = [(enu[a], enu[b], enu[c]) for a, b, c in kept]
    if two_sided:
        triangles += [(p, r, q) for p, q, r in triangles]
    xs = [p[0] for t in triangles for p in t] or [0.0]
    ys = [p[1] for t in triangles for p in t] or [0.0]
    zs = [p[2] for t in triangles for p in t] or [0.0]
    stats = {
        "source": scan.name,
        "georeference": scan.georef.source,
        "utm": f"{scan.georef.zone}{'N' if scan.georef.northern else 'S'} "
        f"{scan.georef.easting:.3f} {scan.georef.northing:.3f}",
        "source_triangles": len(scan.faces),
        "flipped": flipped,
        "ground_z_model_m": round(ground, 3),
        "cell_m": round(cell, 3),
        "clustered_triangles": len(clustered),
        "triangles": len(triangles),
        "two_sided": two_sided,
        "bounds_enu_m": [
            round(min(xs), 2),
            round(min(ys), 2),
            round(min(zs), 2),
            round(max(xs), 2),
            round(max(ys), 2),
            round(max(zs), 2),
        ],
        "coverage_m2": round(coverage.area_m2),
    }
    return ScanMesh(triangles, coverage, stats)


# ---------------------------------------------------------------------------- OSM tiles


@dataclass
class Solid:
    """One connected piece of a tile OBJ (from osm_buildings.py: one extruded footprint)."""

    verts: list[Vec3]
    faces: list[Tri]


def solids(verts: list[Vec3], faces: list[Tri]) -> list[Solid]:
    """Split a tile's mesh into its connected pieces (faces sharing vertices), each renumbered."""
    parent = list(range(len(verts)))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for a, b, c in faces:
        ra, rb, rc = find(a), find(b), find(c)
        parent[rb] = ra
        parent[find(rc)] = ra
    groups: dict[int, list[Tri]] = {}
    for face in faces:
        groups.setdefault(find(face[0]), []).append(face)
    out = []
    for group in groups.values():
        local: dict[int, int] = {}
        for face in group:
            for v in face:
                local.setdefault(v, len(local))
        out.append(Solid([verts[v] for v in local], [(local[a], local[b], local[c]) for a, b, c in group]))
    return out


def _overlaps_square(tri: list[tuple[float, float]], x0: float, y0: float, size: float) -> bool:
    """Whether a 2D triangle (or the segment a wall projects to) and an axis-aligned square share
    more than a boundary: separating axes, the square's two and the triangle's edge normals."""
    eps = 1e-9
    if max(p[0] for p in tri) <= x0 + eps or min(p[0] for p in tri) >= x0 + size - eps:
        return False
    if max(p[1] for p in tri) <= y0 + eps or min(p[1] for p in tri) >= y0 + size - eps:
        return False
    corners = ((x0, y0), (x0 + size, y0), (x0, y0 + size), (x0 + size, y0 + size))
    for i in range(3):
        (px, py), (qx, qy) = tri[i], tri[(i + 1) % 3]
        nx, ny = qy - py, px - qx
        if nx == 0.0 and ny == 0.0:
            continue
        t = [nx * x + ny * y for x, y in tri]
        c = [nx * x + ny * y for x, y in corners]
        if max(t) <= min(c) + eps * math.hypot(nx, ny) or min(t) >= max(c) - eps * math.hypot(nx, ny):
            return False
    return True


def solid_inside(solid: Solid, coverage: Coverage) -> bool:
    """Every coverage square the solid's footprint touches is covered: vertices first (cheap),
    then every square each face's projection (a cap's area, a wall's line) overlaps."""
    if not all(coverage.covers(x, y) for x, y, _ in solid.verts):
        return False
    size = coverage.cell
    for face in solid.faces:
        tri = [(solid.verts[k][0], solid.verts[k][1]) for k in face]
        for i in range(math.floor(min(p[0] for p in tri) / size), math.floor(max(p[0] for p in tri) / size) + 1):
            for j in range(math.floor(min(p[1] for p in tri) / size), math.floor(max(p[1] for p in tri) / size) + 1):
                if (i, j) not in coverage.cells and _overlaps_square(tri, i * size, j * size, size):
                    return False
    return True


def tile_name(ti: int, tj: int) -> str:
    """osm_buildings.py's tile file name, so a scan and the prisms of one tile share a file."""
    return f"tile_{'m' if ti < 0 else ''}{abs(ti)}_{'m' if tj < 0 else ''}{abs(tj)}.obj"


def write_tile(path: Path, header: str, pieces: list[Solid], triangles: list[tuple[Vec3, Vec3, Vec3]]) -> None:
    with path.open("w") as f:
        f.write(f"# {header}\n")
        base = 0
        for piece in pieces:
            f.writelines(f"v {x:.3f} {y:.3f} {z:.3f}\n" for x, y, z in piece.verts)
            f.writelines(f"f {base + a + 1} {base + b + 1} {base + c + 1}\n" for a, b, c in piece.faces)
            base += len(piece.verts)
        for tri in triangles:
            f.writelines(f"v {x:.3f} {y:.3f} {z:.3f}\n" for x, y, z in tri)
            f.write(f"f {base + 1} {base + 2} {base + 3}\n")
            base += 3


# ---------------------------------------------------------------------------- main


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out_dir", type=Path)
    ap.add_argument(
        "--scan",
        action="append",
        required=True,
        metavar="PATH[@LAT,LON[,ZONE]]",
        help="ODM project dir, all.zip, or .obj/.glb (repeatable)",
    )
    ap.add_argument("--buildings", type=Path, help="osm_buildings.py output (tile_*.obj + anchor.json) to merge with")
    ap.add_argument("--lat", type=float, help="anchor latitude (default: --buildings' anchor.json)")
    ap.add_argument("--lon", type=float, help="anchor longitude")
    ap.add_argument("--alt", type=float, help="anchor altitude, metres")
    ap.add_argument("--tile-size", type=float, help="default: --buildings' tile_size_m, else 250")
    ap.add_argument("--cell", type=float, default=DEFAULT_CELL_M, help="decimation cell, metres (default 1)")
    ap.add_argument(
        "--max-triangles",
        type=int,
        default=DEFAULT_MAX_TRIANGLES,
        help="per scan; the cell grows until under it (0: no limit; default 300000)",
    )
    ap.add_argument("--ground-z", type=float, help="model z of skysim's ground (default: found per scan)")
    ap.add_argument(
        "--ground-cut",
        type=float,
        default=DEFAULT_GROUND_CUT_M,
        help="drop triangles wholly below this height above the ground, metres (default 1)",
    )
    ap.add_argument("--coverage-cell", type=float, default=DEFAULT_COVERAGE_CELL_M, help="metres (default 2)")
    ap.add_argument(
        "--margin",
        type=float,
        default=DEFAULT_MARGIN_M,
        help="coverage shrunk by this before replacing prisms, metres (default 5)",
    )
    ap.add_argument("--two-sided", action="store_true", help="write every scan triangle both ways round")
    args = ap.parse_args(argv)
    for name in ("lat", "lon", "alt", "tile_size", "cell", "ground_z", "ground_cut", "coverage_cell", "margin"):
        if getattr(args, name) is not None and not math.isfinite(getattr(args, name)):
            ap.error(f"--{name.replace('_', '-')} must be a finite number")
    for name in ("cell", "coverage_cell"):
        if not getattr(args, name) > 0.0:
            ap.error(f"--{name.replace('_', '-')} must be positive")
    if args.margin < 0.0:
        ap.error("--margin must not be negative")

    base_anchor: dict = {}
    if args.buildings is not None:
        if not args.buildings.is_dir():
            ap.error(f"--buildings {args.buildings}: no such directory")
        if args.buildings.resolve() == args.out_dir.resolve():
            ap.error("out_dir must not be the --buildings directory")
        anchor_path = args.buildings / "anchor.json"
        if anchor_path.is_file():
            base_anchor = json.loads(anchor_path.read_text())
    origin = list(base_anchor.get("origin_wgs84") or [])
    if args.lat is not None and args.lon is not None:
        given = [args.lat, args.lon, args.alt if args.alt is not None else (origin[2] if len(origin) > 2 else 0.0)]
        if origin and (abs(origin[0] - given[0]) > 1e-7 or abs(origin[1] - given[1]) > 1e-7):
            ap.error(f"--lat/--lon {given[:2]} differ from {args.buildings}/anchor.json {origin[:2]}")
        origin = given
    elif args.lat is not None or args.lon is not None:
        ap.error("give both --lat and --lon")
    if len(origin) < 2:
        ap.error("no anchor: give --lat/--lon, or --buildings with an anchor.json")
    anchor_lat, anchor_lon = float(origin[0]), float(origin[1])
    anchor_alt = float(args.alt) if args.alt is not None else float(origin[2]) if len(origin) > 2 else 0.0
    if not all(math.isfinite(v) for v in (anchor_lat, anchor_lon, anchor_alt)):
        ap.error("the anchor (--lat/--lon/--alt or anchor.json) must be finite numbers")
    base_tile = base_anchor.get("tile_size_m")
    if args.tile_size and base_tile and not math.isclose(float(args.tile_size), float(base_tile)):
        ap.error(f"--tile-size {args.tile_size:g} differs from {args.buildings}/anchor.json tile_size_m {base_tile}")
    tile_size = float(args.tile_size or base_tile or 250.0)
    if not 0.0 < tile_size < math.inf:
        ap.error("--tile-size must be positive and finite")

    meshes = []
    for spec in args.scan:
        try:
            scan = load_scan(spec)
            mesh = build_scan_mesh(
                scan,
                anchor_lat,
                anchor_lon,
                cell=args.cell,
                max_triangles=args.max_triangles,
                ground_z=args.ground_z,
                ground_cut=args.ground_cut,
                coverage_cell=args.coverage_cell,
                margin=args.margin,
                two_sided=args.two_sided,
            )
        except (OSError, ScanError, zipfile.BadZipFile, KeyError, IndexError, ValueError) as e:
            raise SystemExit(f"scan_tiles: {spec}: {e}") from None
        s = mesh.stats
        print(
            f"{spec}: {s['source_triangles']} triangles -> {s['triangles']} (cell {s['cell_m']} m, ground at "
            f"model z {s['ground_z_model_m']} m{', flipped' if s['flipped'] else ''}), "
            f"covers {s['coverage_m2']} m2"
        )
        if not mesh.triangles:
            # Nothing would stand where its prisms stood: the scan must not clear the map.
            print(f"warning: {spec}: no triangles above --ground-cut; it replaces no footprints")
            mesh.coverage.cells.clear()
        meshes.append(mesh)

    # The prisms: kept unless wholly inside some scan's coverage.
    pieces_by_tile: dict[str, list[Solid]] = {}
    total = replaced = 0
    replaced_by = [0] * len(meshes)
    if args.buildings is not None:
        for path in sorted(args.buildings.glob("tile_*.obj")):
            try:
                verts, faces = read_obj(path.read_text())
            except ScanError as e:
                raise SystemExit(f"scan_tiles: {path}: {e}") from None
            kept = []
            for piece in solids(verts, faces):
                total += 1
                inside = next((k for k, m in enumerate(meshes) if solid_inside(piece, m.coverage)), None)
                if inside is None:
                    kept.append(piece)
                else:
                    replaced += 1
                    replaced_by[inside] += 1
            if kept:
                pieces_by_tile[path.name] = kept
    for mesh, count in zip(meshes, replaced_by, strict=True):
        mesh.stats["footprints_replaced"] = count

    triangles_by_tile: dict[str, list[tuple[Vec3, Vec3, Vec3]]] = {}
    for mesh in meshes:
        for tri in mesh.triangles:
            cx = (tri[0][0] + tri[1][0] + tri[2][0]) / 3.0
            cy = (tri[0][1] + tri[1][1] + tri[2][1]) / 3.0
            name = tile_name(math.floor(cx / tile_size), math.floor(cy / tile_size))
            triangles_by_tile.setdefault(name, []).append(tri)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    names = sorted(set(pieces_by_tile) | set(triangles_by_tile))
    for name in names:  # before any is written: a tile of another origin is not ours to overwrite
        if (args.out_dir / name).is_file():
            with (args.out_dir / name).open() as f:
                if not f.readline().startswith("# skysim merged tile "):
                    raise SystemExit(f"scan_tiles: {args.out_dir / name} is not from scan_tiles.py: not overwriting it")
    for name in names:
        write_tile(
            args.out_dir / name,
            f"skysim merged tile {name} (scan_tiles.py) — ENU metres about {anchor_lat},{anchor_lon}; OSM prisms "
            f"({len(pieces_by_tile.get(name, []))}) + scan triangles ({len(triangles_by_tile.get(name, []))})",
            pieces_by_tile.get(name, []),
            triangles_by_tile.get(name, []),
        )
    stale = []
    for path in sorted(args.out_dir.glob("tile_*.obj")):
        if path.name in names:
            continue
        with path.open() as f:
            ours = f.readline().startswith("# skysim merged tile ")
        if ours:  # an earlier run's tile with no geometry now: tile_cooker would cook it as phantom geometry
            path.unlink()
            print(f"removed {path}: left by an earlier run")
        else:
            stale.append(path.name)
    if stale:
        print(
            f"warning: {len(stale)} tile_*.obj in {args.out_dir} are not from this run (e.g. {stale[0]}): "
            "remove them before running tile_cooker on *.obj"
        )

    anchor = {
        "origin_wgs84": [anchor_lat, anchor_lon, anchor_alt],
        "crs": "ENU metres about origin_wgs84",
        "tile_size_m": tile_size,
        # osm_buildings.py's own record of the prisms before the merge (source, heights, ...).
        "buildings": base_anchor or None,
        "footprints": total,
        "footprints_replaced_by_scans": replaced,
        "scans": [m.stats for m in meshes],
    }
    with (args.out_dir / "anchor.json").open("w") as f:
        json.dump(anchor, f, indent=2)
    print(f"{replaced} of {total} footprints replaced by scans; wrote {len(names)} tiles to {args.out_dir}")
    print(f"next: tile_cooker <tiles> --anchor {anchor_lat},{anchor_lon},{anchor_alt:g} {args.out_dir}/*.obj")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
