#!/usr/bin/env python3
"""ctest 'survey_fixture': the reconstruction fixture is complete, geotagged and deterministic.

Cooks a small demo city, then runs tools/survey/survey_fixture.py twice into two directories
with a nadir grid plus a crosshatch, and checks what a photogrammetry run depends on: every
planned frame is there, geo.txt lists them longitude first, each frame's EXIF GPS agrees with
geo.txt, and the second run wrote the same bytes as the first. ODM itself is not run here.

Usage: survey_fixture_smoke.py <survey_still> <tile_cooker>
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
FIXTURE = REPO / "tools" / "survey" / "survey_fixture.py"
FAILURES: list[str] = []


def check(cond: bool, what: str) -> None:
    if not cond:
        FAILURES.append(what)
        print(f"FAIL: {what}")


def load_fixture_module():  # noqa: ANN201 - a module object
    spec = importlib.util.spec_from_file_location("survey_fixture", FIXTURE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules["survey_fixture"] = module
    spec.loader.exec_module(module)
    return module


def run_fixture(still: str, tiles: Path, out: Path, *extra: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(FIXTURE), "--out", str(out), "--tiles", str(tiles), "--survey-still", still,
         "--center", "50,150", "--size", "120x80", "--alt", "60", "--image", "256x192",
         "--crosshatch", "-60", "--threads", "2", *extra],
        capture_output=True, text=True, timeout=600, check=False)


def digests(directory: Path) -> dict[str, str]:
    return {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(directory.glob("*.jpg"))}


def main() -> int:
    still, cooker = sys.argv[1], sys.argv[2]
    fixture = load_fixture_module()
    with tempfile.TemporaryDirectory(prefix="skysim_fixture_") as tmp_name:
        tmp = Path(tmp_name)
        obj = tmp / "obj"
        tiles = tmp / "tiles"
        subprocess.run([sys.executable, str(REPO / "tools" / "cooker" / "pretile.py"), str(obj),
                        "--extent", "250", "--tile-size", "250"], check=True, capture_output=True)
        subprocess.run([cooker, str(tiles), *sorted(str(p) for p in obj.glob("*.obj"))], check=True,
                       capture_output=True)

        first = run_fixture(still, tiles, tmp / "a")
        check(first.returncode == 0, f"fixture run ({first.stderr.strip()})")
        second = run_fixture(still, tiles, tmp / "b")
        check(second.returncode == 0, "second fixture run")
        if FAILURES:
            return 1

        manifest = json.loads((tmp / "a" / "manifest.json").read_text())
        frames = digests(tmp / "a")
        check(manifest["planned"] == manifest["frames"] == len(frames), "every planned frame written")
        check(len(manifest["grids"]) == 3, "nadir grid plus a two-way crosshatch")
        # Nadir over 120 x 80 m at 60 m, 80/70 %: footprint 89.9 x 67.5 m, so lanes 27.0 m and
        # shots 13.5 m apart: 4 lanes of 10. Obliques run on past the block.
        args = fixture.parse_args(["--out", "x", "--center", "50,150", "--size", "120x80", "--alt", "60",
                                   "--image", "256x192", "--crosshatch", "-60"])
        _, planned = fixture.plan(args)
        check(len(planned) == len(frames), f"frame count matches the plan ({len(planned)} vs {len(frames)})")
        nadir = [f for f in planned if f.pitch == -90.0]
        check(len(nadir) == 40, f"nadir grid is 4 lanes x 10 ({len(nadir)})")
        check({f.yaw for f in planned if f.pitch == -60.0} == {0.0, 90.0, 180.0, 270.0},
              "obliques look all four ways")

        # Determinism: same arguments, same bytes.
        check(frames == digests(tmp / "b"), "two runs wrote identical frames")
        check((tmp / "a" / "geo.txt").read_text() == (tmp / "b" / "geo.txt").read_text(), "identical geo.txt")

        # geo.txt: EPSG:4326, longitude first, one line per frame, matching each frame's EXIF.
        lines = (tmp / "a" / "geo.txt").read_text().splitlines()
        check(lines[0] == "EPSG:4326", "geo.txt projection header")
        check(len(lines) == len(frames) + 1, "geo.txt line per frame")
        for line in lines[1:]:
            name, lon, lat, alt = line.split()
            check(abs(float(lon) - 24.76) < 0.05 and abs(float(lat) - 42.14) < 0.05,
                  f"{name}: x is longitude, y latitude")
            exif_lat, exif_lon, exif_alt = fixture.read_gps((tmp / "a" / name).read_bytes())
            check(abs(exif_lat - float(lat)) < 1e-6 and abs(exif_lon - float(lon)) < 1e-6,
                  f"{name}: EXIF GPS matches geo.txt")
            check(abs(exif_alt - float(alt)) < 1e-3 and abs(exif_alt - 60.0) < 1e-3, f"{name}: EXIF altitude")
        jpeg = (tmp / "a" / lines[1].split()[0]).read_bytes()
        check(b"SIMULATED IMAGERY" in jpeg[:2048] and b"skysim survey camera" in jpeg[:2048],
              "frames are marked simulated in EXIF")

        # A narrower plan into the same directory leaves no stale frames behind.
        third = run_fixture(still, tiles, tmp / "a", "--size", "60x40")
        check(third.returncode == 0, "re-run into the same directory")
        rerun = json.loads((tmp / "a" / "manifest.json").read_text())
        check(len(digests(tmp / "a")) == rerun["frames"] < manifest["frames"], "old frames cleared")

        bad = run_fixture(still, tiles, tmp / "c", "--front-overlap", "1.2")
        check(bad.returncode == 2, "an impossible overlap is refused")

    if FAILURES:
        print(f"survey_fixture: {len(FAILURES)} failure(s)")
        return 1
    print("survey_fixture: all checks OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
