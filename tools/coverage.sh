#!/usr/bin/env bash
# Unit-test line coverage over src/ (gcovr). Usage: tools/coverage.sh [--min 60]
# Builds into build-cov/ with SKYSIM_COVERAGE=ON, runs ctest, reports, and fails when the
# total line coverage over src/ is below the threshold.
set -euo pipefail
cd "$(dirname "$0")/.."

MIN=${2:-60}
if [[ "${1:-}" == "--min" ]]; then MIN=$2; fi

cmake -B build-cov -G Ninja -DCMAKE_BUILD_TYPE=Debug -DSKYSIM_COVERAGE=ON > /dev/null
cmake --build build-cov -j
# Stale .gcda from previous runs would double-count; start clean every time.
find build-cov -name '*.gcda' -delete
ctest --test-dir build-cov --output-on-failure

gcovr --root . --filter 'src/' build-cov \
      --exclude-throw-branches \
      --print-summary \
      --html-details build-cov/coverage.html \
      --fail-under-line "$MIN"
echo "coverage report: build-cov/coverage.html (threshold ${MIN}% lines)"
