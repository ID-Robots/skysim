#!/bin/bash
# Translate SKYSIM_* environment into skysim CLI flags.
#
# Container platforms configure through the environment, but skysim only takes
# flags (--tiles, --stream-radius, ...), so the mapping lives here rather than
# forcing every caller to hand-write an argv.
set -euo pipefail

args=(--api-bind "${SKYSIM_API_BIND:-0.0.0.0}" --api-port "${SKYSIM_API_PORT:-8642}")

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

if [[ -n "${SKYSIM_DT:-}" ]]; then
    args+=(--dt "${SKYSIM_DT}")
fi

if [[ -n "${SKYSIM_SPAWN_HOME:-}" ]]; then
    args+=(--spawn-home "${SKYSIM_SPAWN_HOME}")
fi

if [[ -n "${SKYSIM_EXTRA_ARGS:-}" ]]; then
    # shellcheck disable=SC2206 # deliberate word splitting: caller supplies flags
    extra=(${SKYSIM_EXTRA_ARGS})
    args+=("${extra[@]}")
fi

echo "skysim: exec skysim ${args[*]}"
exec skysim "${args[@]}"
