#pragma once
// Control plane for SkyHub: REST (spawn/despawn/list/metrics) + WS state stream.
// Reads published snapshots only; writes go through the world command queue.
// Planned dep: cpp-httplib (header-only) — add via FetchContent at M4.
namespace skysim::api {
// TODO(M4): POST /vehicles, DELETE /vehicles/{id}, GET /vehicles, GET /metrics, WS /state.
}  // namespace skysim::api
