#include "video/rtsp_server.h"

#include "video/rtp_h264.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <random>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace skysim::video {
namespace {

using Clock = std::chrono::steady_clock;

constexpr size_t kMaxRequestBytes = 16 * 1024; // headers + body of one request
constexpr size_t kMaxQueuedFrames = 64;        // publish() → server thread, all streams
constexpr auto kSenderReportEvery = std::chrono::seconds(5);
constexpr auto kKeyframeAskEvery = std::chrono::seconds(1); // per session, while it waits for one
constexpr int kMaxDatagramsPerWake = 64;                    // UDP read per poll() wakeup
constexpr auto kAcceptBackoff = std::chrono::seconds(1);    // after accept() runs out of fds
const char kPublic[] = "OPTIONS, DESCRIBE, SETUP, PLAY, PAUSE, TEARDOWN, GET_PARAMETER, SET_PARAMETER";

std::string_view trim(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
        s.remove_prefix(1);
    }
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) {
        s.remove_suffix(1);
    }
    return s;
}

bool iequals(std::string_view a, std::string_view b) {
    return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](char x, char y) {
               return std::tolower(static_cast<unsigned char>(x)) ==
                      std::tolower(static_cast<unsigned char>(y));
           });
}

// A whole decimal number in [lo, hi], or nullopt.
std::optional<int> whole(std::string_view s, int lo, int hi) {
    int v = 0;
    const auto r = std::from_chars(s.data(), s.data() + s.size(), v);
    if (s.empty() || r.ec != std::errc() || r.ptr != s.data() + s.size() || v < lo || v > hi) {
        return std::nullopt;
    }
    return v;
}

// "a-b" or "a" (b = a + 1), both in [lo, hi].
bool port_pair(std::string_view s, int lo, int hi, int &a, int &b) {
    const size_t dash = s.find('-');
    const auto first = whole(s.substr(0, dash), lo, hi);
    if (!first) {
        return false;
    }
    a = *first;
    if (dash == std::string_view::npos) {
        b = a + 1;
        return b <= hi;
    }
    const auto second = whole(s.substr(dash + 1), lo, hi);
    if (!second) {
        return false;
    }
    b = *second;
    return true;
}

std::string hex(uint64_t v, int digits) {
    char buf[17];
    std::snprintf(buf, sizeof(buf), "%0*llx", digits, static_cast<unsigned long long>(v));
    return buf;
}

std::string address_text(const sockaddr_storage &addr) {
    char buf[INET6_ADDRSTRLEN] = "0.0.0.0";
    if (addr.ss_family == AF_INET6) {
        inet_ntop(AF_INET6, &reinterpret_cast<const sockaddr_in6 &>(addr).sin6_addr, buf, sizeof(buf));
    } else {
        inet_ntop(AF_INET, &reinterpret_cast<const sockaddr_in &>(addr).sin_addr, buf, sizeof(buf));
    }
    return buf;
}

sockaddr_storage with_port(sockaddr_storage addr, int port) {
    if (addr.ss_family == AF_INET6) {
        reinterpret_cast<sockaddr_in6 &>(addr).sin6_port = htons(static_cast<uint16_t>(port));
    } else {
        reinterpret_cast<sockaddr_in &>(addr).sin_port = htons(static_cast<uint16_t>(port));
    }
    return addr;
}

int port_of(const sockaddr_storage &addr) {
    return addr.ss_family == AF_INET6 ? ntohs(reinterpret_cast<const sockaddr_in6 &>(addr).sin6_port)
                                      : ntohs(reinterpret_cast<const sockaddr_in &>(addr).sin_port);
}

socklen_t length_of(const sockaddr_storage &addr) {
    return addr.ss_family == AF_INET6 ? sizeof(sockaddr_in6) : sizeof(sockaddr_in);
}

bool same_host(const sockaddr_storage &a, const sockaddr_storage &b) {
    if (a.ss_family != b.ss_family) {
        return false;
    }
    if (a.ss_family == AF_INET6) {
        return std::memcmp(&reinterpret_cast<const sockaddr_in6 &>(a).sin6_addr,
                           &reinterpret_cast<const sockaddr_in6 &>(b).sin6_addr, sizeof(in6_addr)) == 0;
    }
    return reinterpret_cast<const sockaddr_in &>(a).sin_addr.s_addr ==
           reinterpret_cast<const sockaddr_in &>(b).sin_addr.s_addr;
}

void set_nonblocking(int fd) { fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK); }

const char *reason(int code) {
    switch (code) {
    case 200:
        return "OK";
    case 400:
        return "Bad Request";
    case 404:
        return "Not Found";
    case 454:
        return "Session Not Found";
    case 455:
        return "Method Not Valid in This State";
    case 461:
        return "Unsupported Transport";
    case 501:
        return "Not Implemented";
    default:
        return "Error";
    }
}

struct Request {
    std::string method;
    std::string url;
    std::vector<std::pair<std::string, std::string>> headers;

    std::string_view header(std::string_view name) const {
        for (const auto &[k, v] : headers) {
            if (iequals(k, name)) {
                return v;
            }
        }
        return {};
    }
};

// Parses the head of one request (up to, not including, the blank line). False when the
// request line is not "METHOD URL RTSP/1.x".
bool parse_head(std::string_view head, Request &req) {
    const size_t eol = head.find("\r\n");
    const std::string_view line = head.substr(0, eol);
    const size_t a = line.find(' ');
    const size_t b = a == std::string_view::npos ? a : line.find(' ', a + 1);
    if (b == std::string_view::npos || line.substr(b + 1, 7) != "RTSP/1.") {
        return false;
    }
    req.method = std::string(line.substr(0, a));
    req.url = std::string(line.substr(a + 1, b - a - 1));
    size_t pos = eol == std::string_view::npos ? head.size() : eol + 2;
    while (pos < head.size()) {
        size_t end = head.find("\r\n", pos);
        if (end == std::string_view::npos) {
            end = head.size();
        }
        const std::string_view h = head.substr(pos, end - pos);
        const size_t colon = h.find(':');
        if (colon != std::string_view::npos) {
            req.headers.emplace_back(std::string(trim(h.substr(0, colon))),
                                     std::string(trim(h.substr(colon + 1))));
        }
        pos = end + 2;
    }
    return true;
}

} // namespace

std::optional<int> parse_stream_path(std::string_view url) {
    const size_t scheme = url.find("://");
    if (scheme != std::string_view::npos) {
        const size_t path = url.find('/', scheme + 3);
        url = path == std::string_view::npos ? std::string_view{} : url.substr(path);
    }
    url = url.substr(0, url.find('?'));
    constexpr std::string_view kPrefix = "/instances/";
    constexpr std::string_view kName = "/main.264";
    if (url.substr(0, kPrefix.size()) != kPrefix) {
        return std::nullopt;
    }
    url.remove_prefix(kPrefix.size());
    const size_t digits = url.find('/');
    const auto instance = whole(url.substr(0, digits), 0, 999999999);
    if (!instance || digits == std::string_view::npos) {
        return std::nullopt;
    }
    url.remove_prefix(digits);
    if (url.substr(0, kName.size()) != kName || (url.size() > kName.size() && url[kName.size()] != '/')) {
        return std::nullopt;
    }
    return instance;
}

std::optional<TransportSpec> parse_transport(std::string_view header) {
    while (!header.empty()) {
        const size_t comma = header.find(',');
        const std::string_view spec = trim(header.substr(0, comma));
        header = comma == std::string_view::npos ? std::string_view{} : header.substr(comma + 1);

        TransportSpec t;
        bool ok = true;
        bool multicast = false;
        bool have_channels = false;
        bool have_client_ports = false;
        size_t pos = 0;
        bool first = true;
        while (ok && pos <= spec.size()) {
            const size_t semi = spec.find(';', pos);
            const std::string_view part = trim(spec.substr(pos, semi - pos));
            pos = semi == std::string_view::npos ? spec.size() + 1 : semi + 1;
            if (first) {
                first = false;
                if (iequals(part, "RTP/AVP/TCP")) {
                    t.tcp = true;
                } else if (!iequals(part, "RTP/AVP") && !iequals(part, "RTP/AVP/UDP")) {
                    ok = false;
                }
                continue;
            }
            const size_t eq = part.find('=');
            const std::string_view key = part.substr(0, eq);
            const std::string_view value =
                eq == std::string_view::npos ? std::string_view{} : part.substr(eq + 1);
            if (iequals(key, "multicast")) {
                multicast = true;
            } else if (iequals(key, "interleaved")) {
                ok = port_pair(value, 0, 255, t.rtp_channel, t.rtcp_channel);
                have_channels = ok;
            } else if (iequals(key, "client_port")) {
                ok = port_pair(value, 1, 65535, t.client_rtp_port, t.client_rtcp_port);
                have_client_ports = ok;
            }
        }
        if (!ok || multicast) {
            continue;
        }
        if (t.tcp) {
            if (!have_channels) {
                t.rtp_channel = 0;
                t.rtcp_channel = 1;
            }
            return t;
        }
        if (have_client_ports) { // UDP goes nowhere without them, whatever else the spec says
            return t;
        }
    }
    return std::nullopt;
}

struct RtspServer::Impl {
    struct Session {
        std::string id;
        int instance{0};
        TransportSpec transport;
        std::string url; // as SETUP named it, for RTP-Info
        RtpPacketizer rtp;
        uint32_t ts_offset{0};
        bool playing{false};
        bool need_idr{true};
        bool reported{false}; // a sender report has gone out
        Clock::time_point last_report{};
        Clock::time_point last_keyframe_ask{};

        Session(uint32_t ssrc, uint16_t seq) : rtp(ssrc, seq) {}
    };

    struct Connection {
        int fd{-1};
        sockaddr_storage peer{};
        sockaddr_storage local{};
        std::string in;
        std::string out;
        size_t out_off{0};
        Clock::time_point last_heard{};
        Clock::time_point last_progress{};
        std::unique_ptr<Session> session;
        bool closing{false}; // close once the response queued last is sent

        size_t pending() const { return out.size() - out_off; }
    };

    Config cfg;
    Hooks hooks;
    int listen_fd{-1};
    int wake_fd{-1};
    int rtp_fd{-1};
    int rtcp_fd{-1};
    int port{0};
    int rtp_port{0};
    std::thread thread;
    std::atomic<bool> stop{false};
    const Clock::time_point epoch{Clock::now()};
    std::mt19937_64 rng{std::random_device{}()};

    std::mutex queue_mutex;
    std::deque<std::pair<int, std::shared_ptr<const AccessUnit>>> queue;
    std::vector<int> dropped_instances; // streams that lost a queued frame since the last drain

    Clock::time_point accept_paused_until{}; // server thread only

    std::vector<std::unique_ptr<Connection>> connections; // server thread only

    std::atomic<size_t> stat_connections{0};
    std::atomic<size_t> stat_playing{0};
    std::atomic<uint64_t> stat_frames_sent{0};
    std::atomic<uint64_t> stat_frames_dropped{0};
    std::atomic<uint64_t> stat_bytes_sent{0};
    std::atomic<uint64_t> stat_closed_flooding{0};
    std::atomic<uint64_t> stat_closed_not_reading{0};

    ~Impl() {
        for (const int fd : {listen_fd, wake_fd, rtp_fd, rtcp_fd}) {
            if (fd >= 0) {
                ::close(fd);
            }
        }
    }

    void open_sockets();
    void run();
    void poke() const {
        const uint64_t one = 1;
        [[maybe_unused]] const ssize_t n = ::write(wake_fd, &one, sizeof(one));
    }

    uint32_t rtp_now(const Session &s, Clock::time_point t) const {
        const auto us = std::chrono::duration_cast<std::chrono::microseconds>(t - epoch).count();
        return s.ts_offset + static_cast<uint32_t>(us * 9 / 100);
    }

    void accept_all(Clock::time_point now);
    bool read_from(Connection &c, Clock::time_point now);
    bool flush(Connection &c, Clock::time_point now);
    void handle_requests(Connection &c);
    void handle(Connection &c, const Request &req, const std::string &body);
    void respond(Connection &c, int code, std::string_view cseq, const std::string &headers = {},
                 const std::string &body = {});
    void end_session(Connection &c);
    void close_connection(Connection &c);
    void deliver(int instance, const AccessUnit &au, Clock::time_point now);
    void send_report(Connection &c, Clock::time_point now);
    void send_rtcp(Connection &c, const std::vector<uint8_t> &packet);
    void drain_udp(int fd);
    void ask_keyframe(Session &s, Clock::time_point now);
    std::string sdp(const Connection &c, int instance, const StreamInfo &info) const;
};

void RtspServer::Impl::open_sockets() {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE | AI_NUMERICHOST | AI_NUMERICSERV;
    addrinfo *res = nullptr;
    const std::string service = std::to_string(cfg.port);
    if (getaddrinfo(cfg.bind.c_str(), service.c_str(), &hints, &res) != 0 || res == nullptr) {
        throw std::runtime_error("RtspServer: bad bind address " + cfg.bind);
    }
    sockaddr_storage addr{};
    std::memcpy(&addr, res->ai_addr, res->ai_addrlen);
    const int family = res->ai_family;
    freeaddrinfo(res);

    listen_fd = ::socket(family, SOCK_STREAM | SOCK_CLOEXEC, 0);
    const int yes = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    if (listen_fd < 0 || ::bind(listen_fd, reinterpret_cast<const sockaddr *>(&addr), length_of(addr)) != 0 ||
        ::listen(listen_fd, 16) != 0) {
        throw std::runtime_error("RtspServer: cannot bind " + cfg.bind + ":" + std::to_string(cfg.port) +
                                 ": " + std::strerror(errno));
    }
    sockaddr_storage bound{};
    socklen_t len = sizeof(bound);
    getsockname(listen_fd, reinterpret_cast<sockaddr *>(&bound), &len);
    port = port_of(bound);
    set_nonblocking(listen_fd);

    // The UDP pair every UDP session is served from: RTP on an even port, RTCP on the next.
    for (int attempt = 0; attempt < 64 && rtp_fd < 0; ++attempt) {
        const int a = ::socket(family, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        const sockaddr_storage any = with_port(addr, 0);
        sockaddr_storage got{};
        socklen_t got_len = sizeof(got);
        if (a < 0 || ::bind(a, reinterpret_cast<const sockaddr *>(&any), length_of(any)) != 0 ||
            getsockname(a, reinterpret_cast<sockaddr *>(&got), &got_len) != 0 || port_of(got) % 2 != 0 ||
            port_of(got) == 65534) {
            if (a >= 0) {
                ::close(a);
            }
            continue;
        }
        const int b = ::socket(family, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        const sockaddr_storage next = with_port(addr, port_of(got) + 1);
        if (b < 0 || ::bind(b, reinterpret_cast<const sockaddr *>(&next), length_of(next)) != 0) {
            ::close(a);
            if (b >= 0) {
                ::close(b);
            }
            continue;
        }
        rtp_fd = a;
        rtcp_fd = b;
        rtp_port = port_of(got);
    }
    if (rtp_fd < 0) {
        throw std::runtime_error("RtspServer: no free UDP port pair for RTP/RTCP");
    }
    set_nonblocking(rtp_fd);
    set_nonblocking(rtcp_fd);
    wake_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (wake_fd < 0) {
        throw std::runtime_error("RtspServer: eventfd failed");
    }
}

void RtspServer::Impl::run() {
    std::vector<pollfd> fds;
    std::deque<std::pair<int, std::shared_ptr<const AccessUnit>>> batch;
    while (!stop.load(std::memory_order_relaxed)) {
        fds.clear();
        const auto before = Clock::now();
        const bool accepting = before >= accept_paused_until;
        fds.push_back({listen_fd, static_cast<short>(accepting ? POLLIN : 0), 0});
        fds.push_back({wake_fd, POLLIN, 0});
        fds.push_back({rtp_fd, POLLIN, 0});
        fds.push_back({rtcp_fd, POLLIN, 0});
        for (const auto &c : connections) {
            fds.push_back({c->fd, static_cast<short>(POLLIN | (c->pending() > 0 ? POLLOUT : 0)), 0});
        }
        // With nobody connected there is nothing to time out and no report to send: sleep until
        // a connection or a frame arrives. Otherwise wake once a second for both.
        const int timeout_ms = connections.empty() && accepting ? -1 : 1000;
        if (::poll(fds.data(), fds.size(), timeout_ms) < 0) {
            if (errno != EINTR) {
                // Not expected (EFAULT/EINVAL are bugs, ENOMEM passes): say so and carry on rather
                // than leave the port bound with nobody answering it.
                std::fprintf(stderr, "skysim: rtsp: poll failed: %s\n", std::strerror(errno));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            continue;
        }
        const auto now = Clock::now();
        if ((fds[1].revents & POLLIN) != 0) {
            uint64_t count = 0;
            [[maybe_unused]] const ssize_t n = ::read(wake_fd, &count, sizeof(count));
            std::vector<int> lost;
            {
                std::lock_guard<std::mutex> lock(queue_mutex);
                batch.swap(queue);
                lost.swap(dropped_instances);
            }
            // A stream that lost a frame in the queue: its receivers' next frames may reference
            // it, so they wait for an IDR, and one is asked for.
            for (const auto &c : connections) {
                if (c->session && c->session->playing &&
                    std::find(lost.begin(), lost.end(), c->session->instance) != lost.end()) {
                    c->session->need_idr = true;
                    ask_keyframe(*c->session, now);
                }
            }
            for (const auto &[instance, au] : batch) {
                deliver(instance, *au, now);
            }
            batch.clear();
        }
        if ((fds[2].revents & POLLIN) != 0) {
            drain_udp(rtp_fd);
        }
        if ((fds[3].revents & POLLIN) != 0) {
            drain_udp(rtcp_fd);
        }
        // Connections in the poll set are the first fds.size() - 4 of `connections`; any accepted
        // below are polled next time round.
        for (size_t i = 4; i < fds.size(); ++i) {
            Connection &c = *connections[i - 4];
            if (c.fd < 0) {
                continue;
            }
            bool alive = true;
            if ((fds[i].revents & (POLLIN | POLLHUP | POLLERR)) != 0) {
                alive = read_from(c, now);
                if (alive) {
                    handle_requests(c);
                }
            }
            if (alive && c.fd >= 0 && c.pending() > 0) {
                alive = flush(c, now);
            }
            if (alive && c.closing && c.pending() == 0) {
                alive = false;
            }
            // Silence: a connection with nothing playing over it that has not been heard from
            // in 1.5 timeouts, or one whose receiver has taken nothing for a whole timeout.
            const auto timeout = std::chrono::milliseconds(1000LL * cfg.session_timeout_s);
            const bool tcp_playing = c.session && c.session->playing && c.session->transport.tcp;
            if (alive && ((!tcp_playing && now - c.last_heard > timeout * 3 / 2) ||
                          (c.pending() > 0 && now - c.last_progress > timeout))) {
                alive = false;
            }
            if (!alive) {
                close_connection(c);
            }
        }
        connections.erase(std::remove_if(connections.begin(), connections.end(),
                                         [](const std::unique_ptr<Connection> &c) { return c->fd < 0; }),
                          connections.end());
        if (accepting && (fds[0].revents & POLLIN) != 0) {
            accept_all(now);
        }
        for (const auto &c : connections) {
            if (c->session && c->session->playing && c->session->reported &&
                now - c->session->last_report >= kSenderReportEvery) {
                send_report(*c, now);
            }
        }
        stat_connections.store(connections.size(), std::memory_order_relaxed);
    }
    for (const auto &c : connections) {
        ::close(c->fd);
    }
    connections.clear();
}

void RtspServer::Impl::drain_udp(int fd) {
    // RTCP receiver reports (and anything else) sent to the server pair. They are not needed,
    // only taken as a sign of life from whoever sent them.
    uint8_t buf[2048];
    sockaddr_storage from{};
    socklen_t len = sizeof(from);
    const auto now = Clock::now();
    // Bounded per wakeup, so a flood at the pair cannot keep this thread from everyone else.
    for (int n = 0; n < kMaxDatagramsPerWake &&
                    ::recvfrom(fd, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&from), &len) >= 0;
         ++n) {
        for (const auto &c : connections) {
            if (c->session && !c->session->transport.tcp && same_host(c->peer, from) &&
                (port_of(from) == c->session->transport.client_rtcp_port ||
                 port_of(from) == c->session->transport.client_rtp_port)) {
                c->last_heard = now;
            }
        }
        len = sizeof(from);
    }
}

void RtspServer::Impl::accept_all(Clock::time_point now) {
    while (true) {
        sockaddr_storage peer{};
        socklen_t len = sizeof(peer);
        const int fd =
            ::accept4(listen_fd, reinterpret_cast<sockaddr *>(&peer), &len, SOCK_NONBLOCK | SOCK_CLOEXEC);
        if (fd < 0) {
            if (errno == EMFILE || errno == ENFILE || errno == ENOBUFS || errno == ENOMEM) {
                // The connection stays queued and the listen socket readable: stop polling it for a
                // while instead of waking for it on every pass.
                accept_paused_until = now + kAcceptBackoff;
            }
            return;
        }
        if (connections.size() >= cfg.max_connections) {
            ::close(fd);
            continue;
        }
        const int yes = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
        auto c = std::make_unique<Connection>();
        c->fd = fd;
        c->peer = peer;
        socklen_t local_len = sizeof(c->local);
        getsockname(fd, reinterpret_cast<sockaddr *>(&c->local), &local_len);
        c->last_heard = now;
        c->last_progress = now;
        connections.push_back(std::move(c));
    }
}

bool RtspServer::Impl::read_from(Connection &c, Clock::time_point now) {
    char buf[8192];
    while (true) {
        const ssize_t n = ::recv(c.fd, buf, sizeof(buf), 0);
        if (n > 0) {
            c.in.append(buf, static_cast<size_t>(n));
            c.last_heard = now;
            if (c.in.size() > 4 * kMaxRequestBytes) {
                stat_closed_flooding.fetch_add(1, std::memory_order_relaxed);
                return false;
            }
            continue;
        }
        if (n == 0) {
            return false;
        }
        return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
    }
}

bool RtspServer::Impl::flush(Connection &c, Clock::time_point now) {
    while (c.pending() > 0) {
        const ssize_t n = ::send(c.fd, c.out.data() + c.out_off, c.pending(), MSG_NOSIGNAL | MSG_DONTWAIT);
        if (n > 0) {
            c.out_off += static_cast<size_t>(n);
            c.last_progress = now;
            stat_bytes_sent.fetch_add(static_cast<uint64_t>(n), std::memory_order_relaxed);
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            break;
        }
        return false;
    }
    if (c.pending() == 0) {
        c.out.clear();
        c.out_off = 0;
    } else if (c.out_off > 256 * 1024) {
        c.out.erase(0, c.out_off);
        c.out_off = 0;
    }
    return true;
}

void RtspServer::Impl::handle_requests(Connection &c) {
    // Responses are bounded too, not only media: a peer that pipelines requests and does not
    // read the answers is dropped once its backlog passes what a playing TCP receiver can have
    // (max_backlog_bytes plus a frame, well under twice that).
    const size_t max_pending = 2 * cfg.max_backlog_bytes + 64 * 1024;
    while (!c.in.empty() && !c.closing) {
        if (c.pending() > max_pending) {
            stat_closed_not_reading.fetch_add(1, std::memory_order_relaxed);
            c.out.clear();
            c.out_off = 0;
            c.in.clear();
            c.closing = true; // nothing pending: closed by the poll loop straight away
            return;
        }
        if (c.in[0] == '$') {
            // Interleaved binary data from the receiver (its RTCP over TCP): skipped whole.
            if (c.in.size() < 4) {
                return;
            }
            const size_t len = (static_cast<uint8_t>(c.in[2]) << 8) | static_cast<uint8_t>(c.in[3]);
            if (c.in.size() < 4 + len) {
                return;
            }
            c.in.erase(0, 4 + len);
            continue;
        }
        const size_t head_end = c.in.find("\r\n\r\n");
        if (head_end == std::string::npos) {
            if (c.in.size() > kMaxRequestBytes) {
                respond(c, 400, {});
                c.closing = true;
                c.in.clear();
            }
            return;
        }
        Request req;
        if (!parse_head(std::string_view(c.in).substr(0, head_end), req)) {
            respond(c, 400, {});
            c.closing = true;
            c.in.clear();
            return;
        }
        size_t body_len = 0;
        const std::string_view length = req.header("Content-Length");
        if (!length.empty()) {
            const auto v = whole(length, 0, static_cast<int>(kMaxRequestBytes));
            if (!v) {
                respond(c, 400, req.header("CSeq"));
                c.closing = true;
                c.in.clear();
                return;
            }
            body_len = static_cast<size_t>(*v);
        }
        if (c.in.size() < head_end + 4 + body_len) {
            return;
        }
        const std::string body = c.in.substr(head_end + 4, body_len);
        c.in.erase(0, head_end + 4 + body_len);
        handle(c, req, body);
    }
}

void RtspServer::Impl::respond(Connection &c, int code, std::string_view cseq, const std::string &headers,
                               const std::string &body) {
    std::string r = "RTSP/1.0 " + std::to_string(code) + " " + reason(code) + "\r\n";
    if (!cseq.empty()) {
        r += "CSeq: " + std::string(cseq) + "\r\n";
    }
    r += "Server: skysim\r\n";
    r += headers;
    if (!body.empty()) {
        r += "Content-Length: " + std::to_string(body.size()) + "\r\n";
    }
    r += "\r\n";
    r += body;
    c.out += r;
}

std::string RtspServer::Impl::sdp(const Connection &c, int instance, const StreamInfo &info) const {
    const bool v6 = c.local.ss_family == AF_INET6;
    const std::string ip = v6 ? "IP6" : "IP4";
    char fps[32];
    std::snprintf(fps, sizeof(fps), "%g", info.fps);
    std::string s;
    s += "v=0\r\n";
    s += "o=- " +
         std::to_string(static_cast<unsigned long long>(
             std::chrono::duration_cast<std::chrono::seconds>(epoch.time_since_epoch()).count())) +
         " 1 IN " + ip + " " + address_text(c.local) + "\r\n";
    s += "s=skysim instance " + std::to_string(instance) + " camera\r\n";
    s += "c=IN " + ip + (v6 ? " ::" : " 0.0.0.0") + "\r\n";
    s += "t=0 0\r\n";
    s += "a=tool:skysim\r\n";
    s += "a=type:broadcast\r\n";
    s += "a=control:*\r\n";
    s += "a=range:npt=0-\r\n";
    s += "m=video 0 RTP/AVP " + std::to_string(kRtpPayloadType) + "\r\n";
    s += "b=AS:" + std::to_string(info.bitrate_kbps) + "\r\n";
    s += "a=rtpmap:" + std::to_string(kRtpPayloadType) + " H264/90000\r\n";
    s += "a=fmtp:" + std::to_string(kRtpPayloadType) +
         " packetization-mode=1;profile-level-id=" + profile_level_id(info.sps) +
         ";sprop-parameter-sets=" + base64(info.sps) + "," + base64(info.pps) + "\r\n";
    s += "a=framerate:" + std::string(fps) + "\r\n";
    s += "a=x-dimensions:" + std::to_string(info.width) + "," + std::to_string(info.height) + "\r\n";
    s += "a=control:trackID=0\r\n";
    return s;
}

void RtspServer::Impl::handle(Connection &c, const Request &req, const std::string &body) {
    (void)body; // GET/SET_PARAMETER bodies are keep-alive padding here
    const std::string_view cseq = req.header("CSeq");
    const std::string_view session_header =
        trim(req.header("Session").substr(0, req.header("Session").find(';')));
    const auto session_matches = [&] {
        return c.session && (session_header.empty() || session_header == c.session->id);
    };
    const std::string session_line =
        c.session ? "Session: " + c.session->id + ";timeout=" + std::to_string(cfg.session_timeout_s) + "\r\n"
                  : std::string{};

    if (req.method == "OPTIONS") {
        respond(c, 200, cseq, std::string("Public: ") + kPublic + "\r\n");
        return;
    }
    if (req.method == "GET_PARAMETER" || req.method == "SET_PARAMETER") {
        if (!session_header.empty() && !session_matches()) {
            respond(c, 454, cseq);
            return;
        }
        respond(c, 200, cseq, session_line);
        return;
    }
    if (req.method == "DESCRIBE") {
        const auto instance = parse_stream_path(req.url);
        const auto info = instance && hooks.describe ? hooks.describe(*instance) : std::nullopt;
        if (!info) {
            respond(c, 404, cseq);
            return;
        }
        std::string base = req.url.substr(0, req.url.find('?'));
        if (base.empty() || base.back() != '/') {
            base += '/';
        }
        respond(c, 200, cseq, "Content-Type: application/sdp\r\nContent-Base: " + base + "\r\n",
                sdp(c, *instance, *info));
        return;
    }
    if (req.method == "SETUP") {
        const auto instance = parse_stream_path(req.url);
        if (!instance || !hooks.describe || !hooks.describe(*instance)) {
            respond(c, 404, cseq);
            return;
        }
        if (!session_header.empty() && !session_matches()) {
            respond(c, 454, cseq);
            return;
        }
        if (c.session && (c.session->instance != *instance || c.session->playing)) {
            // One stream per connection, and no transport change mid-play.
            respond(c, 455, cseq, session_line);
            return;
        }
        const auto transport = parse_transport(req.header("Transport"));
        if (!transport) {
            respond(c, 461, cseq);
            return;
        }
        if (!c.session) {
            c.session = std::make_unique<Session>(static_cast<uint32_t>(rng()), static_cast<uint16_t>(rng()));
            c.session->id = hex(rng(), 16);
            c.session->ts_offset = static_cast<uint32_t>(rng());
        }
        Session &s = *c.session;
        s.instance = *instance;
        s.transport = *transport;
        s.url = req.url;
        const std::string ssrc = hex(s.rtp.ssrc(), 8);
        std::string reply_transport;
        if (transport->tcp) {
            reply_transport = "RTP/AVP/TCP;unicast;interleaved=" + std::to_string(transport->rtp_channel) +
                              "-" + std::to_string(transport->rtcp_channel) + ";ssrc=" + ssrc;
        } else {
            reply_transport = "RTP/AVP;unicast;client_port=" + std::to_string(transport->client_rtp_port) +
                              "-" + std::to_string(transport->client_rtcp_port) +
                              ";server_port=" + std::to_string(rtp_port) + "-" +
                              std::to_string(rtp_port + 1) + ";ssrc=" + ssrc;
        }
        respond(c, 200, cseq,
                "Transport: " + reply_transport + "\r\nSession: " + s.id +
                    ";timeout=" + std::to_string(cfg.session_timeout_s) + "\r\n");
        return;
    }
    if (req.method == "PLAY" || req.method == "PAUSE" || req.method == "TEARDOWN") {
        if (!session_matches()) {
            respond(c, c.session ? 454 : 455, cseq);
            return;
        }
        Session &s = *c.session;
        if (req.method == "PLAY") {
            if (!s.playing) {
                s.playing = true;
                s.need_idr = true;
                stat_playing.fetch_add(1, std::memory_order_relaxed);
                if (hooks.playing) {
                    hooks.playing(s.instance, true);
                }
                if (hooks.keyframe) {
                    hooks.keyframe(s.instance);
                }
                s.last_keyframe_ask = Clock::now();
            }
            const std::string info = "Range: npt=0.000-\r\nRTP-Info: url=" + s.url +
                                     ";seq=" + std::to_string(s.rtp.next_seq()) +
                                     ";rtptime=" + std::to_string(rtp_now(s, Clock::now())) + "\r\n";
            respond(c, 200, cseq, session_line + info);
            return;
        }
        if (req.method == "PAUSE") {
            if (s.playing) {
                s.playing = false;
                stat_playing.fetch_sub(1, std::memory_order_relaxed);
                if (hooks.playing) {
                    hooks.playing(s.instance, false);
                }
            }
            respond(c, 200, cseq, session_line);
            return;
        }
        respond(c, 200, cseq, session_line);
        end_session(c);
        return;
    }
    respond(c, 501, cseq, std::string("Public: ") + kPublic + "\r\n");
}

void RtspServer::Impl::end_session(Connection &c) {
    if (!c.session) {
        return;
    }
    if (c.session->playing) {
        stat_playing.fetch_sub(1, std::memory_order_relaxed);
        if (hooks.playing && !stop.load(std::memory_order_relaxed)) {
            hooks.playing(c.session->instance, false);
        }
    }
    c.session.reset();
}

void RtspServer::Impl::close_connection(Connection &c) {
    end_session(c);
    ::close(c.fd);
    c.fd = -1;
}

void RtspServer::Impl::send_rtcp(Connection &c, const std::vector<uint8_t> &packet) {
    const Session &s = *c.session;
    if (s.transport.tcp) {
        const uint8_t head[4] = {'$', static_cast<uint8_t>(s.transport.rtcp_channel),
                                 static_cast<uint8_t>(packet.size() >> 8),
                                 static_cast<uint8_t>(packet.size())};
        c.out.append(reinterpret_cast<const char *>(head), 4);
        c.out.append(reinterpret_cast<const char *>(packet.data()), packet.size());
        return;
    }
    const sockaddr_storage to = with_port(c.peer, s.transport.client_rtcp_port);
    if (::sendto(rtcp_fd, packet.data(), packet.size(), MSG_DONTWAIT, reinterpret_cast<const sockaddr *>(&to),
                 length_of(to)) > 0) {
        stat_bytes_sent.fetch_add(packet.size(), std::memory_order_relaxed);
    }
}

void RtspServer::Impl::send_report(Connection &c, Clock::time_point now) {
    Session &s = *c.session;
    // The two clocks read back to back: this report is what maps RTP time (the steady clock
    // the frames were stamped with) onto wall-clock time.
    const uint64_t ntp = ntp_time(std::chrono::system_clock::now());
    const uint32_t ts = rtp_now(s, Clock::now());
    send_rtcp(c, rtcp_sender_report(s.rtp.ssrc(), ntp, ts, s.rtp.packets(), s.rtp.octets(), "skysim"));
    s.reported = true;
    s.last_report = now;
}

void RtspServer::Impl::ask_keyframe(Session &s, Clock::time_point now) {
    if (hooks.keyframe && now - s.last_keyframe_ask >= kKeyframeAskEvery) {
        hooks.keyframe(s.instance);
        s.last_keyframe_ask = now;
    }
}

void RtspServer::Impl::deliver(int instance, const AccessUnit &au, Clock::time_point now) {
    for (const auto &cp : connections) {
        Connection &c = *cp;
        if (c.fd < 0 || !c.session || !c.session->playing || c.session->instance != instance) {
            continue;
        }
        Session &s = *c.session;
        if (s.need_idr && !au.idr) {
            // Nothing before an IDR is decodable for this receiver. Ask again (once a second)
            // until one comes: an earlier ask may have fallen inside the rate limit.
            ask_keyframe(s, now);
            continue;
        }
        if (s.transport.tcp && c.pending() > cfg.max_backlog_bytes) {
            stat_frames_dropped.fetch_add(1, std::memory_order_relaxed);
            s.need_idr = true;
            ask_keyframe(s, now);
            continue;
        }
        s.need_idr = false;
        if (!s.reported) {
            send_report(c, now); // before the first frame, so latency is measurable from it
        }
        const uint32_t ts = rtp_now(s, au.captured);
        if (s.transport.tcp) {
            s.rtp.packetize(au, ts, [&](const uint8_t *p, size_t n) {
                const uint8_t head[4] = {'$', static_cast<uint8_t>(s.transport.rtp_channel),
                                         static_cast<uint8_t>(n >> 8), static_cast<uint8_t>(n)};
                c.out.append(reinterpret_cast<const char *>(head), 4);
                c.out.append(reinterpret_cast<const char *>(p), n);
            });
            if (!flush(c, now)) {
                c.closing = true; // the poll loop sees the dead socket and closes it
            }
        } else {
            const sockaddr_storage to = with_port(c.peer, s.transport.client_rtp_port);
            s.rtp.packetize(au, ts, [&](const uint8_t *p, size_t n) {
                if (::sendto(rtp_fd, p, n, MSG_DONTWAIT, reinterpret_cast<const sockaddr *>(&to),
                             length_of(to)) > 0) {
                    stat_bytes_sent.fetch_add(n, std::memory_order_relaxed);
                }
            });
        }
        stat_frames_sent.fetch_add(1, std::memory_order_relaxed);
    }
}

RtspServer::RtspServer(const Config &cfg, Hooks hooks) : impl_(std::make_unique<Impl>()) {
    impl_->cfg = cfg;
    impl_->hooks = std::move(hooks);
    impl_->open_sockets();
    impl_->thread = std::thread([this] { impl_->run(); });
}

RtspServer::~RtspServer() {
    impl_->stop.store(true, std::memory_order_relaxed);
    impl_->poke();
    if (impl_->thread.joinable()) {
        impl_->thread.join();
    }
}

int RtspServer::port() const { return impl_->port; }
int RtspServer::udp_rtp_port() const { return impl_->rtp_port; }

void RtspServer::publish(int instance, std::shared_ptr<const AccessUnit> au) {
    if (!au) {
        return;
    }
    bool dropped = false;
    {
        std::lock_guard<std::mutex> lock(impl_->queue_mutex);
        impl_->queue.emplace_back(instance, std::move(au));
        if (impl_->queue.size() > kMaxQueuedFrames) {
            const int lost = impl_->queue.front().first;
            impl_->queue.pop_front();
            if (std::find(impl_->dropped_instances.begin(), impl_->dropped_instances.end(), lost) ==
                impl_->dropped_instances.end()) {
                impl_->dropped_instances.push_back(lost);
            }
            dropped = true;
        }
    }
    if (dropped) {
        impl_->stat_frames_dropped.fetch_add(1, std::memory_order_relaxed);
    }
    impl_->poke();
}

RtspServer::Stats RtspServer::stats() const {
    Stats s;
    s.connections = impl_->stat_connections.load(std::memory_order_relaxed);
    s.playing = impl_->stat_playing.load(std::memory_order_relaxed);
    s.frames_sent = impl_->stat_frames_sent.load(std::memory_order_relaxed);
    s.frames_dropped = impl_->stat_frames_dropped.load(std::memory_order_relaxed);
    s.bytes_sent = impl_->stat_bytes_sent.load(std::memory_order_relaxed);
    s.closed_flooding = impl_->stat_closed_flooding.load(std::memory_order_relaxed);
    s.closed_not_reading = impl_->stat_closed_not_reading.load(std::memory_order_relaxed);
    return s;
}

} // namespace skysim::video
