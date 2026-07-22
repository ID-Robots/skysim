#pragma once
// Per-vehicle UDP endpoint: binds 9002 + 10*instance, rx thread fills a latest-wins mailbox,
// reply goes to last sender. No world access from rx threads (CLAUDE.md invariant 5).
#include <cstdint>
#include <optional>
#include "protocol/packets.h"

namespace skysim::protocol {

class UdpEndpoint {
public:
    explicit UdpEndpoint(int instance);  // binds; throws on port conflict (spawn-time only)
    ~UdpEndpoint();
    std::optional<ParsedServos> take_latest();          // tick thread; consumes mailbox
    void send_reply(const char* json, size_t len);      // to last sender addr:port
    // TODO(M1): implement with a small rx thread or one shared epoll loop for all endpoints.
};

}  // namespace skysim::protocol
