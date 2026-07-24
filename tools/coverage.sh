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

# gcov intermittently emits negative branch counts for heavily-threaded code
# ("branch 1 taken -5", GCC bug 68080), which aborts the whole report and fails CI
# for a reason unrelated to the change under test. Warn once per file instead.
gcovr --root . --filter 'src/' build-cov \
      --exclude-throw-branches \
      --gcov-ignore-parse-errors negative_hits.warn_once_per_file \
      --print-summary \
      --html-details build-cov/coverage.html \
      --fail-under-line "$MIN"
echo "coverage report: build-cov/coverage.html (threshold ${MIN}% lines)"
