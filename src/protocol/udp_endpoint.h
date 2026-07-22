#pragma once
// Per-vehicle UDP endpoint: binds 9002 + 10*instance, rx thread fills a latest-wins mailbox,
// reply goes to last sender. No world access from rx threads (CLAUDE.md invariant 5).
#include <cstdint>
#include <memory>
#include <optional>
#include "protocol/packets.h"

namespace skysim::protocol {

class UdpEndpoint {
  public:
    explicit UdpEndpoint(int instance); // binds; throws on port conflict (spawn-time only)
    ~UdpEndpoint();
    UdpEndpoint(const UdpEndpoint &) = delete;
    UdpEndpoint &operator=(const UdpEndpoint &) = delete;

    // Tick thread only. Consumes the mailbox (nullopt if nothing new since last call), parses
    // and classifies against this endpoint's sequence state, which advances on Ok/Gap and
    // resets on Reboot (docs/PROTOCOL.md frame_count semantics).
    std::optional<ParsedServos> take_latest();

    // Tick thread only. Sends to the address:port of the newest datagram; no-op before the
    // first datagram. Never blocks the tick beyond a non-blocking sendto.
    void send_reply(const char *json, size_t len);

    int port() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace skysim::protocol
