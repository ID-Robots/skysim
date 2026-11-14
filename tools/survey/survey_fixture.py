#!/usr/bin/env python3
"""Fly a virtual survey camera over a cooked city without ArduPilot: the reconstruction fixture.

Plans a survey grid over a block of the world (serpentine lanes, given altitude, front and side
overlap and camera pitch, optionally a crosshatch), renders every frame with the server's own
still renderer (``survey_still``, the code behind GET /instances/<n>/still.jpg), and writes what
a photogrammetry run wants beside them:

* ``<name>.jpg`` with EXIF: GPS position and altitude, image direction, focal length (and its
  35 mm equivalent), pixel size, Make "SkyHub", Model "skysim survey camera",
  ImageDescription "SIMULATED IMAGERY";
* ``geo.txt`` for OpenDroneMap: ``EPSG:4326`` then ``name lon lat alt`` (x = longitude first).
  No camera angles: ODM 3.6 turns geo.txt yaw/pitch/roll into omega/phi/kappa through a frame
  that is not local level away from the equator (opendm/photo.py compute_opk), so angles there
  would hand ``--sfm-algorithm triangulation`` rotations tens of degrees off. The exact camera
  angles are in manifest.json;
* ``manifest.json``: the plan, every frame's pose, and each file's SHA-256.

Geotags come from ``survey_still``, which converts NED to lat/lon exactly as the server's pose
headers and ArduPilot's own GPS do, so the fixture is geotagged the way a flown survey is.

    tools/survey/survey_fixture.py --out build/survey_fixture            # 300 x 200 m, nadir
    tools/survey/survey_fixture.py --out build/odm/images --crosshatch -60   # + 3D obliques

With no ``--tiles`` it cooks the demo city (tools/cooker/pretile.py, 1.2 km) once into the build
directory. Same arguments, same bytes: the CI test runs it twice and compares.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import math
import os
import struct
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

# The nominal sensor behind the EXIF focal length: a 1-inch type, 13.2 mm wide. Only the ratio
# of focal length to sensor width means anything for a rendered pinhole; FocalLengthIn35mmFilm
# carries the same ratio in the form photogrammetry tools read first.
SENSOR_WIDTH_MM = 13.2
# Start of the fixture's deterministic clock (EXIF DateTime), one frame every two seconds.
CLOCK_START = (2026, 11, 14, 10, 0, 0)


@dataclasses.dataclass(frozen=True)
class Frame:
    name: str
    north: float
    east: float
    down: float
    yaw: float
    pitch: float


@dataclasses.dataclass(frozen=True)
class Grid:
    axis: str  # "ew": lanes run east-west; "ns": lanes run north-south
    pitch: float


def footprint(alt: float, hfov: float, width: int, height: int) -> tuple[float, float]:
    """Nadir ground footprint (across track = image width, along track = image height), m."""
    across = 2.0 * alt * math.tan(math.radians(hfov) / 2.0)
    return across, across * height / width


def plan_grid(grid: Grid, index: int, args: argparse.Namespace) -> list[Frame]:
    """One serpentine grid over the block; the camera faces along each lane."""
    across_fp, along_fp = footprint(args.alt, args.hfov, args.image[0], args.image[1])
    side_step = across_fp * (1.0 - args.side_overlap)
    front_step = along_fp * (1.0 - args.front_overlap)
    size_e, size_n = args.size
    along_len, across_len = (size_e, size_n) if grid.axis == "ew" else (size_n, size_e)
    # An oblique camera looks ahead of the aircraft; lanes run on past the block by that much
    # so the far edge is seen from both directions.
    ahead = args.alt * math.tan(math.radians(90.0 + grid.pitch))
    along_len += 2.0 * ahead

    lanes = max(1, math.ceil(across_len / side_step) + 1)
    shots = max(1, math.ceil(along_len / front_step) + 1)
    lane_gap = across_len / (lanes - 1) if lanes > 1 else 0.0
    shot_gap = along_len / (shots - 1) if shots > 1 else 0.0
    centre_n, centre_e = args.center

    frames: list[Frame] = []
    for lane in range(lanes):
        offset = -across_len / 2.0 + lane * lane_gap
        forward = lane % 2 == 0
        for shot in range(shots):
            k = shot if forward else shots - 1 - shot
            along = -along_len / 2.0 + k * shot_gap
            if grid.axis == "ew":
                north, east = centre_n + offset, centre_e + along
                yaw = 90.0 if forward else 270.0
            else:
                north, east = centre_n + along, centre_e + offset
                yaw = 0.0 if forward else 180.0
            name = f"g{index}_{grid.axis}{abs(round(grid.pitch)):02d}_l{lane:02d}_{shot:03d}.jpg"
            frames.append(Frame(name, round(north, 4), round(east, 4), -args.alt, yaw, grid.pitch))
    return frames


def plan(args: argparse.Namespace) -> tuple[list[Grid], list[Frame]]:
    """The main grid along the block's long side at --pitch, plus the crosshatch if asked."""
    long_axis = "ew" if args.size[0] >= args.size[1] else "ns"
    grids = [Grid(long_axis, args.pitch)]
    if args.crosshatch is not None:
        for axis in ("ew", "ns"):
            g = Grid(axis, args.crosshatch)
            if g not in grids:
                grids.append(g)
    frames: list[Frame] = []
    for i, grid in enumerate(grids):
        frames.extend(plan_grid(grid, i, args))
    return grids, frames


# --- EXIF --------------------------------------------------------------------------------------

ASCII, BYTE, SHORT, LONG, RATIONAL, UNDEFINED = 2, 1, 3, 4, 5, 7
Value = str | bytes | list[int] | list[tuple[int, int]]


def _encode(kind: int, value: Value) -> tuple[bytes, int]:
    if kind == ASCII:
        assert isinstance(value, str)
        raw = value.encode("ascii") + b"\0"
        return raw, len(raw)
    if kind in (BYTE, UNDEFINED):
        raw = bytes(value) if isinstance(value, list) else value
        assert isinstance(raw, bytes)
        return raw, len(raw)
    assert isinstance(value, list)
    if kind == SHORT:
        return b"".join(struct.pack("<H", v) for v in value), len(value)
    if kind == LONG:
        return b"".join(struct.pack("<I", v) for v in value), len(value)
    return b"".join(struct.pack("<II", n, d) for n, d in value), len(value)  # type: ignore[misc]


def _ifd(entries: list[tuple[int, int, Value]], offset: int) -> bytes:
    """A TIFF IFD at `offset` (from the TIFF header), its out-of-line values after it."""
    entries = sorted(entries, key=lambda e: e[0])
    size = 2 + 12 * len(entries) + 4
    head = bytearray(struct.pack("<H", len(entries)))
    data = bytearray()
    for tag, kind, value in entries:
        raw, count = _encode(kind, value)
        if len(raw) <= 4:
            head += struct.pack("<HHI", tag, kind, count) + raw.ljust(4, b"\0")
        else:
            head += struct.pack("<HHII", tag, kind, count, offset + size + len(data))
            data += raw + (b"\0" if len(raw) % 2 else b"")
    head += struct.pack("<I", 0)
    return bytes(head + data)


def _dms(deg: float) -> list[tuple[int, int]]:
    """Degrees as degrees, minutes and seconds to 1e-4 s (about 3 mm)."""
    total = round(abs(deg) * 3600 * 10000)
    d, rest = divmod(total, 3600 * 10000)
    m, s = divmod(rest, 60 * 10000)
    return [(d, 1), (m, 1), (s, 10000)]


def exif_app1(lat: float, lon: float, alt: float, yaw: float, width: int, height: int, hfov: float,
              stamp: str) -> bytes:
    """An APP1 Exif segment carrying the frame's geotag and camera."""
    focal_mm = SENSOR_WIDTH_MM / 2.0 / math.tan(math.radians(hfov) / 2.0)
    focal_35 = round(18.0 / math.tan(math.radians(hfov) / 2.0))
    px_per_cm = width / (SENSOR_WIDTH_MM / 10.0)
    ifd0: list[tuple[int, int, Value]] = [
        (0x010E, ASCII, "SIMULATED IMAGERY"),
        (0x010F, ASCII, "SkyHub"),
        (0x0110, ASCII, "skysim survey camera"),
        (0x0112, SHORT, [1]),
        (0x011A, RATIONAL, [(72, 1)]),
        (0x011B, RATIONAL, [(72, 1)]),
        (0x0128, SHORT, [2]),
        (0x0131, ASCII, "skysim survey_fixture"),
        (0x0132, ASCII, stamp),
    ]
    exif: list[tuple[int, int, Value]] = [
        (0x9000, UNDEFINED, b"0230"),
        (0x9003, ASCII, stamp),
        (0x920A, RATIONAL, [(round(focal_mm * 1000), 1000)]),
        (0xA002, LONG, [width]),
        (0xA003, LONG, [height]),
        (0xA20E, RATIONAL, [(round(px_per_cm * 1000), 1000)]),
        (0xA20F, RATIONAL, [(round(px_per_cm * 1000), 1000)]),
        (0xA210, SHORT, [3]),
        (0xA405, SHORT, [focal_35]),
    ]
    gps: list[tuple[int, int, Value]] = [
        (0x0000, BYTE, [2, 3, 0, 0]),
        (0x0001, ASCII, "N" if lat >= 0 else "S"),
        (0x0002, RATIONAL, _dms(lat)),
        (0x0003, ASCII, "E" if lon >= 0 else "W"),
        (0x0004, RATIONAL, _dms(lon)),
        (0x0005, BYTE, [0 if alt >= 0 else 1]),
        (0x0006, RATIONAL, [(round(abs(alt) * 1000), 1000)]),
        (0x0010, ASCII, "T"),
        (0x0011, RATIONAL, [(round(yaw * 100), 100)]),
    ]
    # IFD0's size does not depend on the pointer values (LONGs sit inline), so lay it out once
    # with placeholders to find where the sub-IFDs go, then again with the real offsets.
    pointers: list[tuple[int, int, Value]] = [(0x8769, LONG, [0]), (0x8825, LONG, [0])]
    exif_at = 8 + len(_ifd(ifd0 + pointers, 8))
    exif_ifd = _ifd(exif, exif_at)
    gps_at = exif_at + len(exif_ifd)
    pointers = [(0x8769, LONG, [exif_at]), (0x8825, LONG, [gps_at])]
    tiff = b"II*\0" + struct.pack("<I", 8) + _ifd(ifd0 + pointers, 8) + exif_ifd + _ifd(gps, gps_at)
    payload = b"Exif\0\0" + tiff
    return b"\xff\xe1" + struct.pack(">H", len(payload) + 2) + payload


def with_exif(jpeg: bytes, app1: bytes) -> bytes:
    """The JPEG with its JFIF APP0 (if any) replaced by the Exif APP1, which must come first."""
    if jpeg[:2] != b"\xff\xd8":
        raise ValueError("not a JPEG")
    rest = jpeg[2:]
    if rest[:2] == b"\xff\xe0":
        rest = rest[2 + struct.unpack(">H", rest[2:4])[0]:]
    return b"\xff\xd8" + app1 + rest


def read_gps(jpeg: bytes) -> tuple[float, float, float]:
    """(lat, lon, alt) from a JPEG's Exif GPS IFD — enough of a reader to check the writer."""
    if jpeg[2:4] != b"\xff\xe1" or jpeg[6:12] != b"Exif\0\0":
        raise ValueError("no Exif APP1 first")
    tiff = jpeg[12:]

    def entries(at: int) -> dict[int, tuple[int, int, int]]:
        (count,) = struct.unpack_from("<H", tiff, at)
        out = {}
        for i in range(count):
            tag, kind, n, value = struct.unpack_from("<HHII", tiff, at + 2 + 12 * i)
            out[tag] = (kind, n, value)
        return out

    def rationals(entry: tuple[int, int, int]) -> list[float]:
        _, n, at = entry
        return [a / b for a, b in (struct.unpack_from("<II", tiff, at + 8 * i) for i in range(n))]

    def ref(entry: tuple[int, int, int]) -> str:
        return chr(entry[2] & 0xFF)

    gps = entries(entries(8)[0x8825][2])
    d, m, s = rationals(gps[0x0002])
    lat = (d + m / 60 + s / 3600) * (1 if ref(gps[0x0001]) == "N" else -1)
    d, m, s = rationals(gps[0x0004])
    lon = (d + m / 60 + s / 3600) * (1 if ref(gps[0x0003]) == "E" else -1)
    alt = rationals(gps[0x0006])[0] * (1 if (gps[0x0005][2] & 0xFF) == 0 else -1)
    return lat, lon, alt


# --- the run -----------------------------------------------------------------------------------


def stamp_for(index: int) -> str:
    y, mo, d, h, mi, s = CLOCK_START
    seconds = h * 3600 + mi * 60 + s + 2 * index
    return f"{y:04d}:{mo:02d}:{d:02d} {seconds // 3600 % 24:02d}:{seconds // 60 % 60:02d}:{seconds % 60:02d}"


def demo_tiles(build: Path, cooker: Path) -> Path:
    """The demo city of tools/getting_started.sh, cooked once into the build directory.

    Reused only once a cook has completed (a marker written after tile_cooker succeeds): a
    cook interrupted half-way would otherwise be taken for the whole city on the next run.
    """
    base = build / "survey_demo"
    tiles = base / "tiles"
    done = base / "COOKED"
    if done.is_file() and tiles.is_dir():
        return tiles
    for stale in (*tiles.glob("*"), *(base / "obj").glob("*")):
        stale.unlink()
    done.unlink(missing_ok=True)
    obj = base / "obj"
    subprocess.run([sys.executable, str(REPO / "tools" / "cooker" / "pretile.py"), str(obj),
                    "--extent", "600", "--tile-size", "300"], check=True, stdout=subprocess.DEVNULL)
    subprocess.run([str(cooker), str(tiles), *sorted(str(p) for p in obj.glob("*.obj"))], check=True,
                   stdout=subprocess.DEVNULL)
    done.write_text("tools/cooker/pretile.py --extent 600 --tile-size 300\n")
    return tiles


def pair(text: str, kind: type, sep: str) -> tuple:
    parts = text.split(sep)
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(f"expected two values separated by '{sep}': {text}")
    return kind(parts[0]), kind(parts[1])


def parse_args(argv: list[str]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", required=True, type=Path, help="output directory (frames, geo.txt)")
    ap.add_argument("--tiles", type=Path, help="cooked tile directory (default: cook the demo city)")
    ap.add_argument("--build-dir", type=Path, default=REPO / "build", help="where survey_still is built")
    ap.add_argument("--survey-still", type=Path, help="survey_still binary (default: in --build-dir)")
    ap.add_argument("--tile-cooker", type=Path, help="tile_cooker binary (default: in --build-dir)")
    ap.add_argument("--center", type=lambda t: pair(t, float, ","), default=(200.0, 250.0),
                    help="block centre, NED north,east metres from home (default 200,250)")
    ap.add_argument("--size", type=lambda t: pair(t, float, "x"), default=(300.0, 200.0),
                    help="block size, east x north metres (default 300x200)")
    ap.add_argument("--alt", type=float, default=100.0, help="height above home, m (default 100)")
    ap.add_argument("--front-overlap", type=float, default=0.8, help="fraction (default 0.8)")
    ap.add_argument("--side-overlap", type=float, default=0.7, help="fraction (default 0.7)")
    ap.add_argument("--pitch", type=float, default=-90.0, help="camera pitch of the main grid (default -90)")
    ap.add_argument("--crosshatch", type=float, default=None, metavar="PITCH",
                    help="also fly the block along both axes at this camera pitch (a 3D-model capture)")
    ap.add_argument("--image", type=lambda t: pair(t, int, "x"), default=(1600, 1200),
                    help="frame size WxH (default 1600x1200)")
    ap.add_argument("--hfov", type=float, default=73.7, help="horizontal field of view, deg (default 73.7)")
    ap.add_argument("--quality", type=int, default=90, help="JPEG quality (default 90)")
    ap.add_argument("--seed", type=int, default=1, help="texture seed, as SKYSIM_TEXTURE_SEED (default 1)")
    ap.add_argument("--home", default="42.1403890,24.7645490,0",
                    help="lat,lon,alt of the NED origin (default: skysim's --spawn-home)")
    ap.add_argument("--threads", type=int, default=0, help="render threads (default: all cores)")
    args = ap.parse_args(argv)
    for name in ("front_overlap", "side_overlap"):
        if not 0.0 <= getattr(args, name) < 1.0:
            ap.error(f"--{name.replace('_', '-')} must be a fraction in [0, 1)")
    for pitch in (args.pitch, args.crosshatch):
        if pitch is not None and not -90.0 <= pitch <= -30.0:
            ap.error("pitches must be between -90 and -30 (a survey camera looks down)")
    if args.alt <= 0 or args.size[0] <= 0 or args.size[1] <= 0:
        ap.error("--alt and --size must be positive")
    # The server's own bounds (render::StillRequest), so a fixture frame is one the still
    # route could have served.
    if not 10.0 <= args.hfov <= 120.0:
        ap.error("--hfov must be between 10 and 120")
    if not all(64 <= v <= 4096 for v in args.image):
        ap.error("--image sides must be between 64 and 4096")
    if not 1 <= args.quality <= 100:
        ap.error("--quality must be between 1 and 100")
    if not 0 <= args.seed <= 0xFFFFFFFF:
        ap.error("--seed must be between 0 and 4294967295")
    if not 0 <= args.threads <= 256:
        ap.error("--threads must be between 0 and 256")
    return args


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    # Absolute, so a relative path (./survey_still) runs that file rather than a PATH lookup.
    still = (args.survey_still or args.build_dir / "survey_still").resolve()
    cooker = (args.tile_cooker or args.build_dir / "tile_cooker").resolve()
    if not os.access(still, os.X_OK):
        print(f"survey_fixture: no {still}; build it: cmake --build {args.build_dir} --target survey_still "
              "tile_cooker", file=sys.stderr)
        return 2
    tiles = args.tiles or demo_tiles(args.build_dir, cooker)

    grids, frames = plan(args)
    args.out.mkdir(parents=True, exist_ok=True)
    # A smaller plan into the same directory must not leave old frames behind — but only
    # frames this tool wrote there (listed in its previous manifest) are anyone's to delete.
    previous = args.out / "manifest.json"
    if previous.is_file():
        try:
            owned = [img["name"] for img in json.loads(previous.read_text()).get("images", [])]
        except (ValueError, AttributeError, TypeError, KeyError):
            owned = []
        for name in owned:
            if isinstance(name, str) and Path(name).name == name:
                (args.out / name).unlink(missing_ok=True)

    poses = "".join(f"{f.name},{f.north},{f.east},{f.down},{f.yaw},{f.pitch},0\n" for f in frames)
    cmd = [str(still), "--tiles", str(tiles), "--out", str(args.out), "--seed", str(args.seed),
           "--size", f"{args.image[0]}x{args.image[1]}", "--hfov", str(args.hfov),
           "--quality", str(args.quality), "--home", args.home, "--threads", str(args.threads)]
    run = subprocess.run(cmd, input=poses, capture_output=True, text=True, check=False)
    if run.returncode != 0:
        print(run.stderr, file=sys.stderr)
        return run.returncode
    rows = [line.split(",") for line in run.stdout.splitlines()[1:]]
    if len(rows) != len(frames):
        print(f"survey_fixture: rendered {len(rows)} of {len(frames)} frames", file=sys.stderr)
        return 1

    geo = ["EPSG:4326"]
    manifest_frames = []
    render_ms = []
    for index, (frame, row) in enumerate(zip(frames, rows, strict=True)):
        name, lat, lon, alt_amsl = row[0], float(row[1]), float(row[2]), float(row[3])
        yaw, pitch, roll, ms = float(row[5]), float(row[6]), float(row[7]), float(row[8])
        path = args.out / name
        jpeg = with_exif(path.read_bytes(), exif_app1(lat, lon, alt_amsl, yaw, args.image[0], args.image[1],
                                                       args.hfov, stamp_for(index)))
        path.write_bytes(jpeg)
        geo.append(f"{name} {lon:.9f} {lat:.9f} {alt_amsl:.3f}")
        render_ms.append(ms)
        manifest_frames.append({
            "name": name, "north": frame.north, "east": frame.east, "down": frame.down,
            "lat": lat, "lon": lon, "alt_amsl": alt_amsl, "yaw": yaw, "pitch": pitch, "roll": roll,
            "sha256": hashlib.sha256(jpeg).hexdigest(),
        })
    (args.out / "geo.txt").write_text("\n".join(geo) + "\n")
    manifest = {
        "simulated": True,
        "source": "skysim survey_fixture",
        "planned": len(frames),
        "frames": len(manifest_frames),
        "grids": [dataclasses.asdict(g) for g in grids],
        "params": {"center_ne": list(args.center), "size_en": list(args.size), "alt": args.alt,
                   "front_overlap": args.front_overlap, "side_overlap": args.side_overlap,
                   "image": list(args.image), "hfov": args.hfov, "quality": args.quality, "seed": args.seed,
                   "home": args.home},
        "images": manifest_frames,
    }
    (args.out / "manifest.json").write_text(json.dumps(manifest, indent=1) + "\n")
    mean = sum(render_ms) / len(render_ms) if render_ms else 0.0
    print(f"survey_fixture: {len(frames)} frames in {len(grids)} grid(s) "
          f"({', '.join(f'{g.axis}@{g.pitch:g}' for g in grids)}), mean render {mean:.0f} ms, in {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
