#!/usr/bin/env bash
# See skysim work in about a minute, without building ArduPilot first.
#
# Getting started with a lockstep physics server usually means building an
# autopilot, cooking a world and wiring three processes together before anything
# happens on screen. That is a long way to go to find out whether the thing
# builds. This cooks a small city, renders it from a few camera poses, and drops
# the images in one directory — no SITL, no network, no autopilot.
#
# If ARDUPILOT_ROOT is set it goes on to fly a real vehicle through that same
# world, so the second half only costs you anything once you want it.
#
#   tools/getting_started.sh              # render only
#   ARDUPILOT_ROOT=~/ardupilot tools/getting_started.sh --fly
set -euo pipefail

cd "$(dirname "$0")/.."

BUILD_DIR="${BUILD_DIR:-build}"
OUT_DIR="${OUT_DIR:-$BUILD_DIR/getting_started}"
# Its own world rather than build/demo_map, which the quick start above cooks at
# the default extent: this one wants a city big enough to have a horizon.
WORLD_OBJ="$OUT_DIR/world_obj"
WORLD_TILES="$OUT_DIR/world/tiles"
FLY=0
[ "${1:-}" = "--fly" ] && FLY=1
# Overridable because a machine already running skysim owns the default ports,
# and "getting started" should not mean "first stop whatever you were doing".
INSTANCE="${INSTANCE:-0}"
API_PORT="${API_PORT:-8642}"

say() { printf '\n\033[1m%s\033[0m\n' "$*"; }

# --- 1. Build -----------------------------------------------------------------
if [ ! -x "$BUILD_DIR/skysim" ] || [ ! -x "$BUILD_DIR/render_probe" ]; then
    say "Building skysim"
    cmake -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null
    cmake --build "$BUILD_DIR" -j
fi

# --- 2. Cook a world ----------------------------------------------------------
# Skipped when the tiles are already there: cooking is deterministic and the
# whole point of this script is that re-running it is cheap.
if [ ! -d "$WORLD_TILES" ] || [ -z "$(ls -A "$WORLD_TILES" 2>/dev/null)" ]; then
    say "Cooking a demo city"
    # 1.2 km across, in 300 m tiles. Big enough that the far blocks sit out at the
    # draw distance and fade, which is the only way to see that the haze is doing
    # anything; the default 250 m map fits inside one clear view.
    python3 tools/cooker/pretile.py "$WORLD_OBJ" --extent 600 --tile-size 300
    ./"$BUILD_DIR"/tile_cooker "$WORLD_TILES" "$WORLD_OBJ"/*.obj
fi
TILE_COUNT=$(ls -1 "$WORLD_TILES"/*.jshape 2>/dev/null | wc -l)

# --- 3. Render it -------------------------------------------------------------
say "Rendering the world from four camera poses"
mkdir -p "$OUT_DIR"
rm -f "$OUT_DIR"/*.jpg # or renaming a pose leaves the old one behind, listed as if it were fresh

# Four poses that between them show the whole palette: a building from below, a
# skyline at roof height, the grid from above, and a long view into the haze.
# All of them stand off the origin, which pretile.py deliberately leaves clear
# for takeoff — parked there the camera looks down an empty street.
render() { # name north east alt pitch yaw
    ./"$BUILD_DIR"/render_probe \
        --tiles "$WORLD_TILES" --raster --threads 4 --size 640x360 \
        --north "$2" --east "$3" --alt "$4" --pitch "$5" --yaw "$6" \
        --out "$OUT_DIR/$1.jpg" | tail -1
}
render 1-street-level  -160 -160    9   -3   45
render 2-skyline       -320 -320   55  -10   40
render 3-avenue        -500    0   14   -2    0
render 4-long-view     -520 -520  120  -12   38

# --- 4. Optionally fly through it ---------------------------------------------
if [ "$FLY" = "1" ]; then
    if [ -z "${ARDUPILOT_ROOT:-}" ]; then
        echo "  --fly needs ARDUPILOT_ROOT set to an ArduPilot checkout built for SITL" >&2
        exit 2
    fi

    say "Starting skysim + one vehicle (Ctrl-C to stop)"
    LOG="$OUT_DIR/fly.log"
    SITL_PID="" # named in the trap below before it is ever assigned
    # 400 Hz physics against a 200 Hz autopilot scheduler: ArduPilot will not arm
    # unless the gyro rate is at least 1.8x its loop rate, and on the JSON
    # backend the gyro rate is the physics rate.
    ./"$BUILD_DIR"/skysim \
        --vehicles 1 --base-instance "$INSTANCE" --dt 0.0025 --time-mode interactive \
        --tiles "$WORLD_TILES" --api-port "$API_PORT" \
        --camera-fps 10 --camera-size 320x180 >"$LOG" 2>&1 &
    SKYSIM_PID=$!
    trap 'kill $SKYSIM_PID $SITL_PID 2>/dev/null || true' EXIT

    # Poll for the thing we actually need, rather than sleeping a guessed number
    # of seconds at it. A fixed sleep reports a camera URL for a process that
    # died on a port clash thirty seconds ago.
    #
    # give_up_after tenths, pid to watch, message, then the test as extra args.
    wait_for() {
        local tries="$1" pid="$2" what="$3"
        shift 3
        for _ in $(seq "$tries"); do
            if "$@"; then return 0; fi
            if ! kill -0 "$pid" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done
        echo >&2
        sed 's/^/  /' "$LOG" >&2
        echo >&2 "  $what"
        exit 1
    }

    # Deadlines on both: a server that accepts the connection and then says
    # nothing would hang curl forever, and wait_for would never get to retry or
    # to notice the process had died.
    probe() { curl -sf --connect-timeout 1 --max-time 2 "http://127.0.0.1:$API_PORT/vehicles"; }
    api_up() { probe >/dev/null; }
    vehicle_connected() { probe | grep -q '"connected":true'; }

    wait_for 100 $SKYSIM_PID \
        "skysim never came up. If a port is taken: INSTANCE=1 API_PORT=8643 $0 --fly" api_up

    # --serial0 tcp:0 or the autopilot blocks on "Waiting for connection ...."
    # for a ground station that is never coming, and never sends a servo packet.
    "$ARDUPILOT_ROOT/build/sitl/bin/arducopter" --model json:127.0.0.1 -I "$INSTANCE" \
        --home 42.1403890,24.7645490,0,0 \
        --defaults "$ARDUPILOT_ROOT/Tools/autotest/default_params/copter.parm" \
        --serial0 tcp:0 --sim-address 127.0.0.1 >>"$LOG" 2>&1 &
    SITL_PID=$!

    wait_for 300 $SITL_PID \
        "the autopilot never completed the JSON handshake (is instance $INSTANCE already flying?)" \
        vehicle_connected

    say "Flying. The vehicle's camera is live at:"
    echo "  http://127.0.0.1:$API_PORT/instances/$INSTANCE/camera.mjpg   (open in a browser)"
    echo "  http://127.0.0.1:$API_PORT/vehicles                  (state)"
    echo "  http://127.0.0.1:$API_PORT/metrics                   (tick timing)"
    echo "  MAVLink: 127.0.0.1:$((5760 + 10 * INSTANCE))  (connect QGroundControl or SkyHub)"
    wait $SKYSIM_PID
    exit 0
fi

# --- 5. Say what happened -----------------------------------------------------
say "Done — $TILE_COUNT tiles, 4 images in $OUT_DIR"
ls -1 "$OUT_DIR"/*.jpg | sed 's/^/  /'
cat <<EOF

  Open one:      xdg-open $OUT_DIR/2-skyline.jpg
  Run the tests: ctest --test-dir $BUILD_DIR --output-on-failure
  Fly a vehicle: ARDUPILOT_ROOT=~/ardupilot $0 --fly
EOF
