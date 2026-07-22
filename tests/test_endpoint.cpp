// UdpEndpoint: real-socket roundtrip — mailbox latest-wins, classification against sequence
// state, reply addressing, and the answered-duplicate rule (lockstep desync guard).
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <optional>
#include <thread>

#include "protocol/packets.h"
#include "protocol/udp_endpoint.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

constexpr int kInstance = 90; // port 9902 — far from anything live
constexpr int kPort = 9002 + 10 * kInstance;

struct TestPeer {
    int fd;
    sockaddr_in sim_addr{};
    TestPeer() {
        fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        timeval tv{};
        tv.tv_sec = 2;
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        sim_addr.sin_family = AF_INET;
        sim_addr.sin_port = htons(kPort);
        inet_pton(AF_INET, "127.0.0.1", &sim_addr.sin_addr);
    }
    ~TestPeer() { ::close(fd); }

    void send_frame(uint32_t frame_count, uint16_t magic = skysim::protocol::kServoMagic16,
                    size_t size = sizeof(skysim::protocol::ServoPacket16)) {
        skysim::protocol::ServoPacket16 pkt{};
        pkt.magic = magic;
        pkt.frame_rate = 800;
        pkt.frame_count = frame_count;
        for (int i = 0; i < 16; ++i) {
            pkt.pwm[i] = static_cast<uint16_t>(1500 + i);
        }
        ::sendto(fd, &pkt, size, 0, reinterpret_cast<sockaddr *>(&sim_addr), sizeof(sim_addr));
    }

    std::string recv_reply() {
        char buf[2048];
        const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        return n > 0 ? std::string(buf, static_cast<size_t>(n)) : std::string();
    }
};

// The rx thread stores asynchronously: poll take_latest until something arrives.
std::optional<skysim::protocol::ParsedServos> wait_take(skysim::protocol::UdpEndpoint &ep,
                                                        int timeout_ms = 1000) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (auto p = ep.take_latest()) {
            return p;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return std::nullopt;
}

void settle() { std::this_thread::sleep_for(std::chrono::milliseconds(60)); }

} // namespace

int main() {
    using skysim::protocol::FrameStatus;
    skysim::protocol::UdpEndpoint ep(kInstance);
    CHECK(ep.port() == kPort);
    TestPeer peer;

    // Empty mailbox and reply-before-any-sender are safe no-ops.
    CHECK(!ep.take_latest().has_value());
    ep.send_reply("x\n", 2);

    // First frame: kOk, fields parsed.
    peer.send_frame(0);
    auto p = wait_take(ep);
    CHECK(p.has_value() && p->status == FrameStatus::kOk);
    CHECK(p->frame_count == 0 && p->frame_rate_hz == 800 && p->channel_count == 16);
    CHECK(p->pwm[15] == 1515);

    // Reply goes back to the sender's address:port.
    ep.send_reply("{\"t\":1}\n", 8);
    CHECK(peer.recv_reply() == "{\"t\":1}\n");

    // Ok -> next frame; gap; reboot.
    peer.send_frame(1);
    p = wait_take(ep);
    CHECK(p.has_value() && p->status == FrameStatus::kOk);
    peer.send_frame(5);
    p = wait_take(ep);
    CHECK(p.has_value() && p->status == FrameStatus::kGap);
    peer.send_frame(0);
    p = wait_take(ep);
    CHECK(p.has_value() && p->status == FrameStatus::kReboot);
    peer.send_frame(1);
    p = wait_take(ep); // sequence resumed after reboot reset
    CHECK(p.has_value() && p->status == FrameStatus::kOk);

    // Duplicate that arrives AFTER our reply: unanswered -> must be re-replied.
    ep.send_reply("{\"t\":2}\n", 8);
    peer.recv_reply();
    peer.send_frame(1);
    p = wait_take(ep);
    CHECK(p.has_value() && p->status == FrameStatus::kDuplicate);
    CHECK(ep.last_taken_unanswered());

    // Duplicate that arrived BEFORE our reply: already answered -> must NOT be re-replied.
    peer.send_frame(2);
    p = wait_take(ep);
    CHECK(p.has_value() && p->status == FrameStatus::kOk);
    peer.send_frame(2); // ArduPilot re-send lands while we are still "stepping"
    settle();           // let the rx thread store it BEFORE we reply
    ep.send_reply("{\"t\":3}\n", 8);
    peer.recv_reply();
    p = wait_take(ep);
    CHECK(p.has_value() && p->status == FrameStatus::kDuplicate);
    CHECK(!ep.last_taken_unanswered());

    // Latest-wins: two frames land between polls; only the newest is seen.
    peer.send_frame(3);
    peer.send_frame(4);
    settle();
    p = wait_take(ep);
    CHECK(p.has_value() && p->frame_count == 4);
    CHECK(!ep.take_latest().has_value()); // frame 3 was overwritten, not queued

    // Corruption classification straight through the socket path.
    peer.send_frame(7, 0x1234);
    p = wait_take(ep);
    CHECK(p.has_value() && p->status == FrameStatus::kBadMagic);
    peer.send_frame(7, skysim::protocol::kServoMagic16, 17);
    p = wait_take(ep);
    CHECK(p.has_value() && p->status == FrameStatus::kBadSize);

    // Port conflict is a constructor failure (spawn-time), not a late surprise.
    bool threw = false;
    try {
        skysim::protocol::UdpEndpoint dup(kInstance);
    } catch (const std::exception &) {
        threw = true;
    }
    CHECK(threw);

    if (g_failures == 0) {
        std::printf("test_endpoint: all checks OK\n");
        return 0;
    }
    std::printf("test_endpoint: %d failure(s)\n", g_failures);
    return 1;
}
