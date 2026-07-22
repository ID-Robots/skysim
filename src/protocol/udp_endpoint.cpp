// See udp_endpoint.h. POSIX sockets; one rx thread per endpoint (M1 — a shared epoll loop
// can replace this at M4 scale without touching the interface).
#include "protocol/udp_endpoint.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace skysim::protocol {

namespace {
constexpr int kBasePort = 9002; // 9002 + 10*I, SITL_cmdline.cpp:255,437-438 @ Copter-4.7.0
constexpr int kPortStride = 10;
// Largest valid servo datagram is 72 bytes; a bigger mailbox lets us receive-and-reject
// oversized garbage without truncating it into a "valid" size.
constexpr size_t kMailboxBytes = 256;
} // namespace

struct UdpEndpoint::Impl {
    int fd = -1;
    int port = 0;
    std::thread rx_thread;
    std::atomic<bool> stop{false};

    // Mailbox: written by rx thread, drained by tick thread. Latest-wins.
    std::mutex mutex;
    uint8_t buf[kMailboxBytes];
    size_t len = 0;
    sockaddr_in sender{};
    uint64_t seq = 0; // bumped per datagram stored

    // Tick-thread state (single writer: only take_latest / send_reply touch these).
    uint64_t consumed_seq = 0;
    std::optional<uint32_t> last_frame_count;
    sockaddr_in reply_addr{};
    bool have_reply_addr = false;

    void rx_loop() {
        uint8_t local[kMailboxBytes];
        while (!stop.load(std::memory_order_relaxed)) {
            sockaddr_in from{};
            socklen_t from_len = sizeof(from);
            const ssize_t n =
                ::recvfrom(fd, local, sizeof(local), 0, reinterpret_cast<sockaddr *>(&from), &from_len);
            if (n <= 0) {
                continue; // timeout (SO_RCVTIMEO) or transient error; re-check stop flag
            }
            std::lock_guard<std::mutex> lock(mutex);
            std::memcpy(buf, local, static_cast<size_t>(n));
            len = static_cast<size_t>(n);
            sender = from;
            ++seq;
        }
    }
};

UdpEndpoint::UdpEndpoint(int instance) : impl_(std::make_unique<Impl>()) {
    impl_->port = kBasePort + kPortStride * instance;

    impl_->fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (impl_->fd < 0) {
        throw std::runtime_error("UdpEndpoint: socket(): " + std::string(std::strerror(errno)));
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY); // remote SITLs allowed (DESIGN.md process model)
    addr.sin_port = htons(static_cast<uint16_t>(impl_->port));
    if (::bind(impl_->fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        const int err = errno;
        ::close(impl_->fd);
        impl_->fd = -1;
        throw std::runtime_error("UdpEndpoint: bind(" + std::to_string(impl_->port) +
                                 "): " + std::string(std::strerror(err)));
    }

    timeval tv{};
    tv.tv_usec = 100 * 1000; // rx thread wakes 10x/s to check the stop flag
    ::setsockopt(impl_->fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    impl_->rx_thread = std::thread([this] { impl_->rx_loop(); });
}

UdpEndpoint::~UdpEndpoint() {
    if (!impl_) {
        return;
    }
    impl_->stop.store(true, std::memory_order_relaxed);
    if (impl_->rx_thread.joinable()) {
        impl_->rx_thread.join();
    }
    if (impl_->fd >= 0) {
        ::close(impl_->fd);
    }
}

std::optional<ParsedServos> UdpEndpoint::take_latest() {
    uint8_t local[kMailboxBytes];
    size_t local_len = 0;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        if (impl_->seq == impl_->consumed_seq) {
            return std::nullopt;
        }
        impl_->consumed_seq = impl_->seq;
        local_len = impl_->len;
        std::memcpy(local, impl_->buf, local_len);
        impl_->reply_addr = impl_->sender;
        impl_->have_reply_addr = true;
    }

    const auto bytes = std::as_bytes(std::span(local, local_len));
    ParsedServos parsed = parse_servo_datagram(bytes, impl_->last_frame_count);
    switch (parsed.status) {
    case FrameStatus::kOk:
    case FrameStatus::kGap:
    case FrameStatus::kReboot: // reboot resets sequence state; pose is kept by the caller
        impl_->last_frame_count = parsed.frame_count;
        break;
    case FrameStatus::kDuplicate:
    case FrameStatus::kBadMagic:
    case FrameStatus::kBadSize:
        break;
    }
    return parsed;
}

void UdpEndpoint::send_reply(const char *json, size_t len) {
    if (!impl_->have_reply_addr) {
        return;
    }
    ::sendto(impl_->fd, json, len, MSG_DONTWAIT, reinterpret_cast<const sockaddr *>(&impl_->reply_addr),
             sizeof(impl_->reply_addr));
}

int UdpEndpoint::port() const { return impl_->port; }

} // namespace skysim::protocol
