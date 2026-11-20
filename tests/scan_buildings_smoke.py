#!/usr/bin/env python3
"""ctest 'scan_buildings': survey scans replace the map's prisms where we have them (TASK-2735).

A synthetic OpenDroneMap project stands in for a flown survey: a 160 m ground sheet with a
scanned building (16 x 12 m, 18 m) on the footprint of the real OSM building РРС Пловдив (way
99064086, 29 m in tests/fixtures/plovdiv_buildings_osm.json) and a 25 m mast on the 250 m tile
seam, written the way ODM writes them — UTM zone 35N metres about a coords.txt offset, the scan's
ground at a datum height of 151.3 m. tools/cooker/scan_tiles.py merges it with the three OSM
buildings cooked by osm_buildings.py, tile_cooker cooks the result, and skysim answers
``POST /mission/check`` against it:

  * the scanned building's roof is at 18 m (not OSM's 29), its walls stop a level leg, and the
    part of the OSM footprint the scan does not cover is empty: that prism was dropped;
  * the mast is in both tiles of the seam it straddles; the scan's ground is not geometry;
  * the two OSM buildings outside the scan are still there at 36 and 15 m.

Before the world: unit checks of the projection (against pyproj references and an anchor round
trip), coords.txt, the OBJ and GLB readers, orientation, decimation and the coverage test.
Usage: scan_buildings_smoke.py <skysim> <tile_cooker>   (API port 18712, instance 57.)
"""

from __future__ import annotations

import importlib.util
import json
import math
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
ANCHOR_LAT, ANCHOR_LON = 42.1403890, 24.7645490  # README's, and the gateway's SITL_HOME_LOCATION
RADIUS_M = 2000.0
EARTH_RADIUS_M = 6378137.0
DATUM_M = 151.3  # the scan's ground in its own frame
BOX_HALF = (8.0, 6.0)  # east, north
BOX_HEIGHT = 18.0
MAST_EAST, MAST_HALF, MAST_HEIGHT = -1500.0, 3.0, 25.0  # on the seam between tile columns -7 and -6
SHEET_HALF = 80.0
API_PORT, INSTANCE = 18712, 57
FAILURES: list[str] = []
CHECKS = [0]

Vec3 = tuple[float, float, float]


def check(cond: bool, what: str) -> None:
    CHECKS[0] += 1
    if not cond:
        FAILURES.append(what)
        print(f"FAIL: {what}")


def load(name: str) -> types.ModuleType:
    if str(COOKER_DIR) not in sys.path:
        sys.path.insert(0, str(COOKER_DIR))
    spec = importlib.util.spec_from_file_location(name, COOKER_DIR / f"{name}.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def enu_to_latlon(east: float, north: float) -> tuple[float, float]:
    """The inverse of osm_buildings.to_enu, written out again here."""
    lat = ANCHOR_LAT + math.degrees(north / EARTH_RADIUS_M)
    lon = ANCHOR_LON + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(ANCHOR_LAT))))
    return lat, lon


# ---------------------------------------------------------------------------- the synthetic scan


class Mesh:
    def __init__(self) -> None:
        self.verts: list[Vec3] = []
        self.faces: list[tuple[int, int, int]] = []
        self._index: dict[tuple[int, int, int], int] = {}

    def vertex(self, p: Vec3) -> int:
        key = (round(p[0] * 1000), round(p[1] * 1000), round(p[2] * 1000))
        if key not in self._index:
            self._index[key] = len(self.verts)
            self.verts.append(p)
        return self._index[key]

    def patch(self, origin: Vec3, du: Vec3, dv: Vec3, nu: int, nv: int) -> None:
        """A grid of nu x nv quads; outward is du x dv."""

        def at(i: int, j: int) -> int:
            return self.vertex(tuple(origin[k] + du[k] * i / nu + dv[k] * j / nv for k in range(3)))

        for i in range(nu):
            for j in range(nv):
                self.faces.append((at(i, j), at(i + 1, j), at(i + 1, j + 1)))
                self.faces.append((at(i, j), at(i + 1, j + 1), at(i, j + 1)))

    def box(self, x0: float, x1: float, y0: float, y1: float, h: float, step: float) -> None:
        """Walls and roof, open below (a scan sees no floor), CCW outward."""
        corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1], strict=True):
            length = math.hypot(bx - ax, by - ay)
            self.patch((ax, ay, 0.0), (bx - ax, by - ay, 0.0), (0.0, 0.0, h), round(length / step), round(h / step))
        self.patch(
            (x0, y0, h), (x1 - x0, 0.0, 0.0), (0.0, y1 - y0, 0.0), round((x1 - x0) / step), round((y1 - y0) / step)
        )


def scan_mesh(centre: tuple[float, float]) -> Mesh:
    """The scene in ENU about the anchor, z up from the ground."""
    ce, cn = centre
    mesh = Mesh()
    boxes = [
        (ce - BOX_HALF[0], ce + BOX_HALF[0], cn - BOX_HALF[1], cn + BOX_HALF[1], BOX_HEIGHT),
        (MAST_EAST - MAST_HALF, MAST_EAST + MAST_HALF, cn - MAST_HALF, cn + MAST_HALF, MAST_HEIGHT),
    ]
    for x0, x1, y0, y1, h in boxes:
        mesh.box(x0, x1, y0, y1, h, 0.5)
    step = 2.0
    n = round(2 * SHEET_HALF / step)
    for i in range(n):
        for j in range(n):
            x, y = ce - SHEET_HALF + i * step, cn - SHEET_HALF + j * step
            if any(x0 - step < x < x1 and y0 - step < y < y1 for x0, x1, y0, y1, _ in boxes):
                continue  # no ground under a building

            def z(px: float, py: float) -> float:
                return 0.08 * math.sin(0.7 * px) * math.cos(0.9 * py)  # reconstruction noise

            a = mesh.vertex((x, y, z(x, y)))
            b = mesh.vertex((x + step, y, z(x + step, y)))
            c = mesh.vertex((x + step, y + step, z(x + step, y + step)))
            d = mesh.vertex((x, y + step, z(x, y + step)))
            mesh.faces += [(a, b, c), (a, c, d)]
    return mesh


def to_model(scan_tiles: types.ModuleType, mesh: Mesh, offset: tuple[float, float]) -> list[Vec3]:
    """ENU vertices -> ODM's frame: UTM 35N metres about ``offset``, z above the datum."""
    out = []
    for e, n, u in mesh.verts:
        easting, northing, zone, _ = scan_tiles.latlon_to_utm(*enu_to_latlon(e, n))
        assert zone == 35
        out.append((easting - offset[0], northing - offset[1], u + DATUM_M))
    return out


def write_obj(path: Path, verts: list[Vec3], faces: list[tuple[int, int, int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        f.write("mtllib odm_textured_model_geo.mtl\nusemtl material0000\n")
        f.writelines(f"v {x:.4f} {y:.4f} {z:.4f}\n" for x, y, z in verts)
        f.writelines(f"vt 0.5 0.5\nf {a + 1}/1 {b + 1}/1 {c + 1}/1\n" for a, b, c in faces)


def glb_bytes(doc: dict, binary: bytes) -> bytes:
    payload = json.dumps(doc).encode()
    payload += b" " * (-len(payload) % 4)
    binary += b"\0" * (-len(binary) % 4)
    body = struct.pack("<II", len(payload), 0x4E4F534A) + payload + struct.pack("<II", len(binary), 0x004E4942) + binary
    return struct.pack("<4sII", b"glTF", 2, 12 + len(body)) + body


def write_glb(
    path: Path, verts: list[Vec3], faces: list[tuple[int, int, int]], offset: tuple[float, float], mirror: bool = False
) -> None:
    """The same model as a GLB: node translation + 90 degree rotation about z (+ a mirroring
    scale of x), CESIUM_RTC."""
    t = (10.0, -20.0, 5.0)
    sx = -1.0 if mirror else 1.0
    local = [(sx * (y - t[1]), -(x - t[0]), z - t[2]) for x, y, z in verts]  # S^-1 R^-1 (p - t), R = +90 about z
    if mirror:  # mirrored, the stored triangles show the other side: the node mirrors them back
        faces = [(a, c, b) for a, b, c in faces]
    positions = b"".join(struct.pack("<3f", *p) for p in local)
    indices = b"".join(struct.pack("<3I", *f) for f in faces)
    lo = [min(p[k] for p in local) for k in range(3)]
    hi = [max(p[k] for p in local) for k in range(3)]
    s = math.sqrt(0.5)
    doc = {
        "asset": {"version": "2.0"},
        "extensionsUsed": ["CESIUM_RTC"],
        "extensions": {"CESIUM_RTC": {"center": [offset[0], offset[1], 0.0]}},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [
            {"children": [1]},
            {"mesh": 0, "translation": list(t), "rotation": [0.0, 0.0, s, s], "scale": [sx, 1.0, 1.0]},
        ],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1, "mode": 4}]}],
        "buffers": [{"byteLength": len(positions) + len(indices)}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": len(positions), "target": 34962},
            {"buffer": 0, "byteOffset": len(positions), "byteLength": len(indices), "target": 34963},
        ],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": len(local), "type": "VEC3", "min": lo, "max": hi},
            {"bufferView": 1, "componentType": 5125, "count": 3 * len(faces), "type": "SCALAR"},
        ],
    }
    path.write_bytes(glb_bytes(doc, positions + indices))


# ---------------------------------------------------------------------------- unit checks


def normal_z(t: tuple[Vec3, Vec3, Vec3]) -> float:
    (ax, ay, _), (bx, by, _), (cx, cy, _) = t
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)


def unit_checks(st: types.ModuleType, ob: types.ModuleType, tmp: Path) -> None:
    # The projection against pyproj 3 (EPSG:4326 -> 32635 / 32734), computed once in the
    # opendronemap/odm image: (lat, lon, easting, northing, zone, northern).
    for lat, lon, e_ref, n_ref, zone_ref, north_ref in [
        (42.1403890, 24.7645490, 315268.0004, 4667782.2288, 35, True),  # the anchor
        (42.1452774, 24.7468000, 313815.5223, 4668363.6083, 35, True),  # РРС Пловдив
        (41.9000000, 24.0100000, 251975.6441, 4642997.4767, 35, True),
        (-33.9200000, 18.4200000, 261488.8264, 6243716.3330, 34, False),
    ]:
        e, n, zone, northern = st.latlon_to_utm(lat, lon)
        check(
            abs(e - e_ref) < 0.002 and abs(n - n_ref) < 0.002 and (zone, northern) == (zone_ref, north_ref),
            f"UTM of {lat},{lon}: {e:.4f} {n:.4f} {zone}{northern}, pyproj {e_ref} {n_ref}",
        )
        back = st.utm_to_latlon(e, n, zone, northern)
        check(abs(back[0] - lat) < 1e-8 and abs(back[1] - lon) < 1e-8, f"UTM round trip of {lat},{lon}: {back}")

    # A known point lands at its ENU place, through coords.txt and through an origin.
    target = (-1460.0, 545.0)  # metres east, north of the anchor
    lat, lon = enu_to_latlon(*target)
    easting, northing, _, _ = st.latlon_to_utm(lat, lon)
    for georef in (
        st.parse_coords("WGS84 UTM 35N\n313800 4668300\n"),
        st.parse_coords("EPSG:32635\n313800 4668300"),
        st.georef_from_origin(*st.utm_to_latlon(313800.0, 4668300.0, 35, True)),
    ):
        got = georef.to_enu(easting - georef.easting, northing - georef.northing, ANCHOR_LAT, ANCHOR_LON)
        check(math.dist(got, target) < 0.01, f"{georef.source}: ({lat}, {lon}) lands at {got}, not {target}")
        # The UTM grid is turned ~1.5 degrees from north here: a model placed without it is metres off.
        grid = (easting - 315268.0004, northing - 4667782.2288)
        check(math.dist(grid, target) > 10.0, f"grid convergence matters ({grid} vs {target})")
    for bad in ("WGS84 UTM 35N\n", "EPSG:4326\n24.7 42.1", "WGS84 UTM 35N\nnan 1", "WGS84 UTM 61N\n1 2"):
        try:
            st.parse_coords(bad)
            check(False, f"coords.txt {bad!r} refused")
        except st.ScanError:
            pass

    # OBJ: polygons fanned, /vt suffixes, negative indices.
    verts, faces = st.read_obj("v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nvt 0 0\nf 1/1 2/1 3/1 4/1\nf -4 -3 -2\n")
    check(faces == [(0, 1, 2), (0, 2, 3), (0, 1, 2)], f"OBJ faces {faces}")

    # GLB: node transforms baked in, the same vertices as the OBJ; a Draco primitive is refused
    # with a way out (unless DracoPy is installed).
    mesh = Mesh()
    mesh.box(0.0, 4.0, 0.0, 3.0, 5.0, 1.0)
    write_glb(tmp / "unit.glb", mesh.verts, mesh.faces, (313800.0, 4668300.0))
    gverts, gfaces, ext = st.read_glb((tmp / "unit.glb").read_bytes())
    worst = max(math.dist(a, b) for a, b in zip(gverts, mesh.verts, strict=True))
    check(gfaces == mesh.faces and worst < 1e-4, f"GLB reads back the model (worst {worst:.2e} m)")
    check(ext.get("CESIUM_RTC", {}).get("center") == [313800.0, 4668300.0, 0.0], f"GLB extensions {ext}")
    write_glb(tmp / "mirror.glb", mesh.verts, mesh.faces, (313800.0, 4668300.0), mirror=True)
    mverts, mfaces, _ = st.read_glb((tmp / "mirror.glb").read_bytes())
    worst = max(math.dist(a, b) for a, b in zip(mverts, mesh.verts, strict=True))
    check(
        worst < 1e-4
        and not st.is_inside_out(mverts, mfaces)
        and sorted(map(sorted, mfaces)) == sorted(map(sorted, mesh.faces)),
        f"a mirroring GLB node keeps the outward side (worst {worst:.2e} m)",
    )
    strip = json.loads(
        (tmp / "unit.glb").read_bytes()[20 : 20 + struct.unpack_from("<I", (tmp / "unit.glb").read_bytes(), 12)[0]]
    )
    strip["meshes"][0]["primitives"][0]["mode"] = 5
    try:
        st.read_glb(glb_bytes(strip, b""))
        check(False, "a triangle strip is refused, not skipped")
    except st.ScanError as e:
        check("strip" in str(e), f"triangle strip refusal: {e}")
    draco_doc = {
        "asset": {"version": "2.0"},
        "nodes": [{"mesh": 0}],
        "bufferViews": [{"buffer": 0, "byteLength": 4}],
        "meshes": [
            {
                "primitives": [
                    {"attributes": {"POSITION": 0}, "extensions": {"KHR_draco_mesh_compression": {"bufferView": 0}}}
                ]
            }
        ],
    }
    if importlib.util.find_spec("DracoPy") is None:
        try:
            st.read_glb(glb_bytes(draco_doc, b"\0\0\0\0"))
            check(False, "a Draco GLB without DracoPy is refused")
        except st.ScanError as e:
            check("DracoPy" in str(e) and ".obj" in str(e), f"Draco refusal says what to do: {e}")

    # Orientation: an inside-out box is found and flipped; decimation keeps the winding.
    inverted = [(a, c, b) for a, b, c in mesh.faces]
    check(
        not st.is_inside_out(mesh.verts, mesh.faces) and st.is_inside_out(mesh.verts, inverted),
        "an inside-out mesh is told from an outward one",
    )
    fine = Mesh()
    fine.patch((0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (0.0, 10.0, 0.0), 40, 40)  # 0.25 m quads, facing up
    cverts, cfaces = st.cluster(fine.verts, fine.faces, 1.0)
    up = all(normal_z((cverts[a], cverts[b], cverts[c])) > 0 for a, b, c in cfaces)
    check(len(cfaces) < len(fine.faces) / 8 and up, f"decimation: {len(fine.faces)} -> {len(cfaces)}, all up: {up}")

    # Coverage: a prism wholly inside is replaced; one across its edge, one outside, and one whose
    # corners are covered but whose middle is a hole are kept.
    cov = st.Coverage(2.0, {(i, j) for i in range(50) for j in range(50)} - {(25, 25)})
    tile = tmp / "unit_osm.obj"
    with tile.open("w") as f:
        base = 0
        for ring in (
            [(10, 10), (20, 10), (20, 20), (10, 20)],
            [(95, 10), (105, 10), (105, 20), (95, 20)],
            [(200, 0), (210, 0), (210, 10), (200, 10)],
            [(44, 44), (58, 44), (58, 58), (44, 58)],
        ):
            base = ob.emit_prism(f, base, [(float(x), float(y)) for x, y in ring], 10.0)
    pieces = st.solids(*st.read_obj(tile.read_text()))
    inside = [st.solid_inside(p, cov) for p in pieces]
    check(inside == [True, False, False, False], f"prisms inside the coverage: {inside}")

    # coverage_of: gaps of two cells closed, three not; the margin in whole cells.
    points = [(x + 0.5, y + 0.5) for x in range(100) for y in range(100) if not (40 <= x < 44 or 60 <= x < 66)]
    closed = st.coverage_of(points, 2.0, 0.0)
    check(closed.covers(41.0, 50.0) and not closed.covers(62.0, 50.0), "coverage closes 2-cell gaps, not 3-cell ones")
    shrunk = st.coverage_of(points, 2.0, 5.0)
    check(
        not shrunk.covers(5.0, 50.0) and shrunk.covers(7.0, 50.0) and shrunk.covers(90.0, 50.0),
        "coverage shrinks by the margin rounded up to whole cells (6 m)",
    )


# ---------------------------------------------------------------------------- the world


def stop(proc: subprocess.Popen) -> int:
    proc.send_signal(signal.SIGTERM)
    try:
        return proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        return proc.wait()


def mission_check(waypoints: list[list[float]]) -> dict:
    body = json.dumps({"waypoints_ned": waypoints, "clearance_m": 0.0}).encode()
    request = urllib.request.Request(f"http://127.0.0.1:{API_PORT}/mission/check", data=body, method="POST")
    with urllib.request.urlopen(request, timeout=15) as response:
        return json.loads(response.read().decode())


def roof(east: float, north: float) -> float | None:
    """Straight down from 100 m to 1 m above the ground (the gateway's roof probe)."""
    result = mission_check([[north, east, -100.0], [north, east, -1.0]])
    check(result.get("checked") is True, "skysim had building geometry for the probe")
    hits = result.get("hits") or []
    return -hits[0]["position_ned"][2] if hits else None


def level_hit(start: tuple[float, float], end: tuple[float, float], height: float) -> tuple[float, float] | None:
    """(east, north) of the first hit on a level leg at ``height``, or None."""
    result = mission_check([[start[1], start[0], -height], [end[1], end[0], -height]])
    hits = result.get("hits") or []
    return (hits[0]["position_ned"][1], hits[0]["position_ned"][0]) if hits else None


def run(cmd: list[str], what: str) -> str:
    done = subprocess.run(cmd, capture_output=True, text=True, timeout=300, check=False)
    check(done.returncode == 0, f"{what}: {done.stdout[-600:]} {done.stderr[-600:]}")
    return done.stdout


def main() -> int:
    sim, cooker_bin = sys.argv[1], sys.argv[2]
    ob = load("osm_buildings")
    st = load("scan_tiles")
    payload = json.loads(FIXTURE.read_text())
    elements = {(e["type"], e["id"]): e for e in payload["elements"]}

    def ring_of(key: tuple[str, int]) -> list[tuple[float, float]]:
        element = elements[key]
        geometry = (
            element["geometry"]
            if element["type"] == "way"
            else next(m["geometry"] for m in element["members"] if m["role"] == "outer")
        )
        ring = [ob.to_enu(p["lat"], p["lon"], ANCHOR_LAT, ANCHOR_LON) for p in geometry]
        return ring[:-1] if ring[0] == ring[-1] else ring

    rrs = ring_of(("way", 99064086))
    centre = (sum(p[0] for p in rrs) / len(rrs), sum(p[1] for p in rrs) / len(rrs))
    # Inside the OSM footprint, 2.5 m off the scanned box: only the dropped prism was there.
    off_box = (centre[0], centre[1] + BOX_HALF[1] + 2.5)
    check(ob.point_in_ring(*off_box, rrs) and ob.point_in_ring(*centre, rrs), "probe points inside the OSM footprint")
    others = [
        ("government tower", ring_of(("way", 1578993375)), 36.0),
        ("Четвърти корпус", ring_of(("relation", 7292387)), 15.0),
    ]

    with tempfile.TemporaryDirectory(prefix="skysim_scan_") as tmp_name:
        tmp = Path(tmp_name)
        unit_checks(st, ob, tmp)

        # The ODM project: model + coords.txt, offset a whole-metre UTM point as ODM writes it.
        mesh = scan_mesh(centre)
        e0, n0, _, _ = st.latlon_to_utm(*enu_to_latlon(*centre))
        offset = (float(math.floor(e0)), float(math.floor(n0)))
        project = tmp / "odm_project"
        model = to_model(st, mesh, offset)
        write_obj(project / "odm_texturing" / "odm_textured_model_geo.obj", model, mesh.faces)
        (project / "odm_georeferencing").mkdir(parents=True)
        (project / "odm_georeferencing" / "coords.txt").write_text(f"WGS84 UTM 35N\n{offset[0]:.0f} {offset[1]:.0f}\n")
        # The gateway's form of the same scan: model.glb plus model_origin.
        write_glb(tmp / "model.glb", model, mesh.faces, offset)
        origin_lat, origin_lon = st.utm_to_latlon(offset[0], offset[1], 35, True)

        obj_scan = st.load_scan(str(project))
        from_obj = st.build_scan_mesh(obj_scan, ANCHOR_LAT, ANCHOR_LON)
        from_glb = st.build_scan_mesh(
            st.load_scan(f"{tmp / 'model.glb'}@{origin_lat:.9f},{origin_lon:.9f}"), ANCHOR_LAT, ANCHOR_LON
        )
        s = from_obj.stats
        print(
            f"synthetic scan: {s['source_triangles']} -> {s['triangles']} triangles, ground at model z "
            f"{s['ground_z_model_m']}, covers {s['coverage_m2']} m2"
        )
        check(abs(s["ground_z_model_m"] - DATUM_M) < 0.1 and not s["flipped"], f"ground found at the datum ({s})")
        check(s["triangles"] < s["source_triangles"] / 4, f"decimated ({s['source_triangles']} -> {s['triangles']})")
        glb_bounds = from_glb.stats["bounds_enu_m"]
        check(
            abs(from_glb.stats["triangles"] - s["triangles"]) <= 0.01 * s["triangles"]
            and all(abs(a - b) < 0.05 for a, b in zip(glb_bounds, s["bounds_enu_m"], strict=True)),
            f"GLB@origin and OBJ+coords.txt agree ({from_glb.stats['triangles']} vs {s['triangles']} triangles, "
            f"{glb_bounds} vs {s['bounds_enu_m']})",
        )
        # A GLB whose RTC centre is not the coords.txt offset beside it is refused.
        (tmp / "mismatch").mkdir()
        (tmp / "mismatch" / "model.glb").write_bytes((tmp / "model.glb").read_bytes())
        (tmp / "mismatch" / "coords.txt").write_text(f"WGS84 UTM 35N\n{offset[0] + 100:.0f} {offset[1]:.0f}\n")
        try:
            st.load_scan(str(tmp / "mismatch" / "model.glb"))
            check(False, "a GLB whose CESIUM_RTC disagrees with coords.txt is refused")
        except st.ScanError as e:
            check("CESIUM_RTC" in str(e), f"RTC mismatch refusal: {e}")
        lo, hi = s["bounds_enu_m"][:3], s["bounds_enu_m"][3:]
        expect_lo = (MAST_EAST - MAST_HALF, centre[1] - BOX_HALF[1])
        expect_hi = (centre[0] + BOX_HALF[0], centre[1] + BOX_HALF[1], MAST_HEIGHT)
        check(
            abs(lo[0] - expect_lo[0]) < 1.0
            and abs(lo[1] - expect_lo[1]) < 1.0
            and all(abs(hi[k] - expect_hi[k]) < 1.0 for k in range(3)),
            f"scan lands where it was flown: {s['bounds_enu_m']} vs {expect_lo} {expect_hi}",
        )
        inverted = st.Scan("inverted", obj_scan.verts, [(a, c, b) for a, b, c in obj_scan.faces], obj_scan.georef)
        flipped = st.build_scan_mesh(inverted, ANCHOR_LAT, ANCHOR_LON)
        check(
            flipped.stats["flipped"] and flipped.stats["triangles"] == s["triangles"], "an inside-out scan is flipped"
        )

        # OSM prisms -> merged with the scan -> cooked.
        run(
            [
                sys.executable,
                str(COOKER_DIR / "osm_buildings.py"),
                str(tmp / "osm_obj"),
                "--lat",
                str(ANCHOR_LAT),
                "--lon",
                str(ANCHOR_LON),
                "--radius",
                str(RADIUS_M),
                "--cache",
                str(FIXTURE),
            ],
            "osm_buildings.py",
        )
        # A scan with nothing above the ground cut clears no footprints.
        empty = run(
            [
                sys.executable,
                str(COOKER_DIR / "scan_tiles.py"),
                str(tmp / "empty_obj"),
                "--buildings",
                str(tmp / "osm_obj"),
                "--scan",
                str(project),
                "--ground-cut",
                "100",
            ],
            "scan_tiles.py --ground-cut 100",
        )
        empty_anchor = json.loads((tmp / "empty_obj" / "anchor.json").read_text())
        check(
            "replaces no footprints" in empty and empty_anchor["footprints_replaced_by_scans"] == 0,
            f"a scan with no triangles replaces nothing ({empty_anchor['footprints_replaced_by_scans']})",
        )
        # An earlier run's tile is removed; a tile of someone else's is only warned about.
        (tmp / "world_obj").mkdir()
        (tmp / "world_obj" / "tile_50_50.obj").write_text("# skysim merged tile tile_50_50.obj (old)\nv 0 0 0\n")
        (tmp / "world_obj" / "tile_51_51.obj").write_text("# mine\nv 0 0 0\n")
        out = run(
            [
                sys.executable,
                str(COOKER_DIR / "scan_tiles.py"),
                str(tmp / "world_obj"),
                "--buildings",
                str(tmp / "osm_obj"),
                "--scan",
                str(project),
            ],
            "scan_tiles.py",
        )
        print(out.strip())
        check(
            not (tmp / "world_obj" / "tile_50_50.obj").exists() and "tile_51_51.obj" in out,
            "stale tiles: an earlier run's removed, a foreign one warned about",
        )
        (tmp / "world_obj" / "tile_51_51.obj").unlink()
        anchor = json.loads((tmp / "world_obj" / "anchor.json").read_text())
        check(
            anchor.get("footprints_replaced_by_scans") == 1
            and anchor.get("footprints") == 3
            and anchor["origin_wgs84"][:2] == [ANCHOR_LAT, ANCHOR_LON]
            and anchor["scans"][0]["footprints_replaced"] == 1
            and anchor["scans"][0]["triangles"] == s["triangles"]
            and (anchor.get("buildings") or {}).get("heights"),
            f"anchor.json: OSM's record plus the scan's ({anchor.get('footprints_replaced_by_scans')})",
        )
        objs = sorted(str(p) for p in (tmp / "world_obj").glob("*.obj"))
        names = [Path(p).name for p in objs]
        check("tile_m7_2.obj" in names and "tile_m6_2.obj" in names, f"the mast is cut to both tiles: {names}")
        run([cooker_bin, str(tmp / "tiles"), "--anchor", f"{ANCHOR_LAT},{ANCHOR_LON},0", *objs], "tile_cooker")
        if FAILURES:
            return report()

        proc = subprocess.Popen(
            [
                sim,
                "--vehicles",
                "0",
                "--base-instance",
                str(INSTANCE),
                "--time-mode",
                "interactive",
                "--api-port",
                str(API_PORT),
                "--tiles",
                str(tmp / "tiles"),
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        try:
            deadline = time.time() + 30
            while True:
                try:
                    urllib.request.urlopen(f"http://127.0.0.1:{API_PORT}/metrics", timeout=2).close()
                    break
                except (urllib.error.URLError, ConnectionError):
                    if time.time() > deadline or proc.poll() is not None:
                        check(False, "skysim answers")
                        return report()
                    time.sleep(0.2)

            top = roof(*centre)
            print(f"scanned building: roof {top} m (scan {BOX_HEIGHT:g} m, OSM 29 m)")
            check(top is not None and abs(top - BOX_HEIGHT) < 0.5, f"scanned roof at {BOX_HEIGHT:g} m: {top}")
            gone = roof(*off_box)
            check(gone is None, f"the replaced OSM prism is gone off the scanned box: {gone}")
            south = level_hit((centre[0], centre[1] - 150.0), centre, 10.0)
            east = level_hit((centre[0] + 150.0, centre[1]), centre, 10.0)
            print(f"scanned walls hit at {south} (south) and {east} (east)")
            check(
                south is not None and abs(south[1] - (centre[1] - BOX_HALF[1])) < 1.0,
                f"a level leg from the south stops at the scanned south wall: {south}",
            )
            check(
                east is not None and abs(east[0] - (centre[0] + BOX_HALF[0])) < 1.0,
                f"a level leg from the east stops at the scanned east wall: {east}",
            )
            check(
                level_hit((centre[0], centre[1] - 150.0), centre, BOX_HEIGHT + 1.5) is None,
                "and passes 1.5 m over its roof",
            )
            for side in (-1.5, 1.5):
                mast = roof(MAST_EAST + side, centre[1])
                check(mast is not None and abs(mast - MAST_HEIGHT) < 0.5, f"mast at {side:+} m of the seam: {mast}")
            ground = roof(centre[0] - 40.0, centre[1] - 40.0)
            check(ground is None, f"the scan's ground is not geometry: {ground}")
            for name, ring, height in others:
                top = roof(*ob.interior_point(ring))
                print(f"OSM {name}: roof {top} m")
                check(top is not None and abs(top - height) < 0.5, f"OSM {name} outside the scan kept: {top}")
        finally:
            check(stop(proc) == 0, "skysim clean shutdown")
    return report()


def report() -> int:
    if FAILURES:
        print(f"scan_buildings: {len(FAILURES)} of {CHECKS[0]} checks failed")
        return 1
    print(f"scan_buildings: all {CHECKS[0]} checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
