#!/usr/bin/env bash
# Full simulator coverage over src/ (gcovr).
# Usage: tools/coverage.sh [--min-line 85] [--min-branch 70] [--min-function 95]
set -euo pipefail
cd "$(dirname "$0")/.."

LINE_MIN=85
BRANCH_MIN=70
FUNCTION_MIN=95
COVERAGE_BUILD_DIR=${SKYSIM_COVERAGE_BUILD_DIR:-build-cov}

while (($#)); do
    case "$1" in
        --min|--min-line)
            [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
            LINE_MIN=$2
            shift 2
            ;;
        --min-branch)
            [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
            BRANCH_MIN=$2
            shift 2
            ;;
        --min-function)
            [[ $# -ge 2 ]] || { echo "$1 requires a value" >&2; exit 2; }
            FUNCTION_MIN=$2
            shift 2
            ;;
        *)
            echo "unknown option: $1" >&2
            exit 2
            ;;
    esac
done

for threshold in "$LINE_MIN" "$BRANCH_MIN" "$FUNCTION_MIN"; do
    if ! [[ "$threshold" =~ ^[0-9]+$ ]] || ((threshold > 100)); then
        echo "coverage thresholds must be integers from 0 to 100" >&2
        exit 2
    fi
done

cmake -S . -B "$COVERAGE_BUILD_DIR" -G Ninja \
      -DCMAKE_BUILD_TYPE=Debug -DSKYSIM_COVERAGE=ON > /dev/null
cmake --build "$COVERAGE_BUILD_DIR" --parallel
# Stale .gcda from previous runs would double-count; start clean every time.
find "$COVERAGE_BUILD_DIR" -name '*.gcda' -delete
# Benchmarks are timing gates, not correctness coverage, and instrumentation changes their timing.
ctest --test-dir "$COVERAGE_BUILD_DIR" --label-exclude '^performance$' --output-on-failure

# gcov intermittently emits negative branch counts for heavily-threaded code
# ("branch 1 taken -5", GCC bug 68080), which aborts the whole report and fails CI
# for a reason unrelated to the change under test. Warn once per file instead.
gcovr --root . --filter '^src/' "$COVERAGE_BUILD_DIR" \
      --exclude-throw-branches \
      --gcov-ignore-parse-errors negative_hits.warn_once_per_file \
      --print-summary \
      --html-details "$COVERAGE_BUILD_DIR/coverage.html" \
      --cobertura "$COVERAGE_BUILD_DIR/coverage.xml" --cobertura-pretty \
      --json-summary-pretty --json-summary "$COVERAGE_BUILD_DIR/coverage.json" \
      --txt "$COVERAGE_BUILD_DIR/coverage.txt" \
      --fail-under-line "$LINE_MIN" \
      --fail-under-branch "$BRANCH_MIN" \
      --fail-under-function "$FUNCTION_MIN"

echo "coverage reports: $COVERAGE_BUILD_DIR/coverage.html, coverage.xml, coverage.json"
echo "coverage gates: lines >= ${LINE_MIN}%, branches >= ${BRANCH_MIN}%, functions >= ${FUNCTION_MIN}%"
