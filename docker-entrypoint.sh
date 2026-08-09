#!/bin/bash
# Translate SKYSIM_* environment into skysim CLI flags.
#
# Container platforms configure through the environment, but skysim only takes
# flags (--tiles, --stream-radius, ...), so the mapping lives here rather than
# forcing every caller to hand-write an argv.
set -euo pipefail

args=(--api-bind "${SKYSIM_API_BIND:-0.0.0.0}" --api-port "${SKYSIM_API_PORT:-8642}")

# One mapping, used by everything below: set the variable, get the flag; leave it
# unset and the binary's own default stands, so a default lives in one place
# rather than being restated here.
add_if_set() { # env-var-name flag
    local value="${!1:-}"
    if [[ -n "$value" ]]; then
        args+=("$2" "$value")
    fi
}

# Vehicles are normally spawned on demand over the control plane, so default to
# starting with none rather than skysim's built-in default of one.
args+=(--vehicles "${SKYSIM_VEHICLES:-0}")

if [[ -n "${SKYSIM_TILES:-}" ]]; then
    if [[ -d "${SKYSIM_TILES}" ]]; then
        args+=(--tiles "${SKYSIM_TILES}")
        args+=(--stream-radius "${SKYSIM_STREAM_RADIUS:-1500}")
        args+=(--stream-max "${SKYSIM_STREAM_MAX:-128}")
    else
        # Loud, but not fatal: a flat world still flies, it just has no buildings,
        # and failing the container would take the whole SITL task down with it.
        echo "skysim: SKYSIM_TILES=${SKYSIM_TILES} is not a directory — starting with no buildings" >&2
    fi
fi

add_if_set SKYSIM_DT --dt

# Time mode, which a deployment could not set at all before.
#
# skysim defaults to strict because that is what determinism and CI replays need:
# a barrier every tick, and an abort when a vehicle misses it. A long-lived server
# wants the opposite — vehicles join and leave, some of them lag, and none of that
# should take the fleet down. Without this the only way to say so was
# SKYSIM_EXTRA_ARGS, so every deployment quietly ran strict.
add_if_set SKYSIM_TIME_MODE --time-mode

add_if_set SKYSIM_SPAWN_HOME --spawn-home

# Camera. SKYSIM_CAMERA_FPS is the switch — without --camera-fps the render
# service is never built and every other camera setting is inert, which is
# exactly how a deployment ends up serving a control plane with no pictures on
# it. The rest only apply once it is on.
if [[ -n "${SKYSIM_CAMERA_FPS:-}" ]]; then
    args+=(--camera-fps "${SKYSIM_CAMERA_FPS}")
    add_if_set SKYSIM_CAMERA_SIZE --camera-size
    add_if_set SKYSIM_CAMERA_QUALITY --camera-quality
    add_if_set SKYSIM_CAMERA_THREADS --camera-threads
    add_if_set SKYSIM_CAMERA_FOV --camera-fov
    add_if_set SKYSIM_CAMERA_PITCH --camera-pitch
    add_if_set SKYSIM_CAMERA_RANGE --camera-range
fi

if [[ -n "${SKYSIM_EXTRA_ARGS:-}" ]]; then
    # shellcheck disable=SC2206 # deliberate word splitting: caller supplies flags
    extra=(${SKYSIM_EXTRA_ARGS})
    args+=("${extra[@]}")
fi

echo "skysim: exec skysim ${args[*]}"
exec skysim "${args[@]}"
