#pragma once
// A blocking RTSP/RTP test client for test_video and test_video_api: just enough to drive the
// server through DESCRIBE/SETUP/PLAY over TCP or UDP and put the H.264 back together.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace rtsp_test {

struct Response {
    int status{0};
    std::map<std::string, std::string> headers; // lower-case names
    std::string body;

    std::string header(const std::string &name) const {
        auto it = headers.find(name);
        return it == headers.end() ? std::string{} : it->second;
    }
    // The session id without ";timeout=".
    std::string session() const {
        const std::string s = header("session");
        return s.substr(0, s.find(';'));
    }
};

inline std::string lower(std::string s) {
    for (char &c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

class Client {
  public:
    explicit Client(int port, const char *host = "127.0.0.1", int family = AF_INET, int rcvbuf = 0) {
        fd_ = ::socket(family, SOCK_STREAM, 0);
        if (rcvbuf > 0) {
            setsockopt(fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
        }
        sockaddr_storage addr{};
        socklen_t len = 0;
        if (family == AF_INET6) {
            auto &a = reinterpret_cast<sockaddr_in6 &>(addr);
            a.sin6_family = AF_INET6;
            a.sin6_port = htons(static_cast<uint16_t>(port));
            inet_pton(AF_INET6, host, &a.sin6_addr);
            len = sizeof(a);
        } else {
            auto &a = reinterpret_cast<sockaddr_in &>(addr);
            a.sin_family = AF_INET;
            a.sin_port = htons(static_cast<uint16_t>(port));
            inet_pton(AF_INET, host, &a.sin_addr);
            len = sizeof(a);
        }
        connected_ = ::connect(fd_, reinterpret_cast<sockaddr *>(&addr), len) == 0;
    }
    ~Client() { close(); }
    Client(const Client &) = delete;
    Client &operator=(const Client &) = delete;

    bool connected() const { return connected_; }
    int fd() const { return fd_; }
    void close() {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }

    void send_raw(const std::string &text) {
        size_t off = 0;
        while (off < text.size()) {
            const ssize_t n = ::send(fd_, text.data() + off, text.size() - off, MSG_NOSIGNAL);
            if (n <= 0) {
                return;
            }
            off += static_cast<size_t>(n);
        }
    }

    std::optional<Response> request(const std::string &method, const std::string &url,
                                    const std::string &headers = {}, const std::string &body = {},
                                    int timeout_ms = 3000) {
        std::string req =
            method + " " + url + " RTSP/1.0\r\nCSeq: " + std::to_string(++cseq_) + "\r\n" + headers;
        if (!body.empty()) {
            req += "Content-Length: " + std::to_string(body.size()) + "\r\n";
        }
        req += "\r\n" + body;
        send_raw(req);
        return response(timeout_ms);
    }

    // The next response; interleaved frames met on the way go to packets().
    std::optional<Response> response(int timeout_ms = 3000) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (true) {
            take_interleaved();
            const size_t end = buf_.find("\r\n\r\n");
            if (end != std::string::npos && !buf_.empty() && buf_[0] != '$') {
                Response r;
                const std::string head = buf_.substr(0, end);
                size_t pos = head.find("\r\n");
                const std::string line = head.substr(0, pos);
                if (line.rfind("RTSP/1.0 ", 0) == 0) {
                    r.status = std::atoi(line.c_str() + 9);
                }
                while (pos != std::string::npos && pos + 2 < head.size()) {
                    const size_t next = head.find("\r\n", pos + 2);
                    const std::string h =
                        head.substr(pos + 2, next == std::string::npos ? std::string::npos : next - pos - 2);
                    const size_t colon = h.find(':');
                    if (colon != std::string::npos) {
                        size_t v = colon + 1;
                        while (v < h.size() && h[v] == ' ') {
                            ++v;
                        }
                        r.headers[lower(h.substr(0, colon))] = h.substr(v);
                    }
                    pos = next;
                }
                const size_t length = static_cast<size_t>(std::atoi(r.header("content-length").c_str()));
                if (buf_.size() >= end + 4 + length) {
                    r.body = buf_.substr(end + 4, length);
                    buf_.erase(0, end + 4 + length);
                    return r;
                }
            }
            if (!read_more(deadline)) {
                return std::nullopt;
            }
        }
    }

    // Reads until `n` interleaved packets have arrived on `channel` (or any channel when -1).
    bool wait_packets(size_t n, int channel, int timeout_ms) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (true) {
            take_interleaved();
            size_t have = 0;
            for (const auto &p : packets_) {
                have += (channel < 0 || p.first == channel) ? 1 : 0;
            }
            if (have >= n) {
                return true;
            }
            if (!read_more(deadline)) {
                return false;
            }
        }
    }

    // True when the server closes the connection within the timeout.
    bool closed_by_peer(int timeout_ms) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            pollfd p{fd_, POLLIN, 0};
            if (::poll(&p, 1, 50) > 0) {
                char tmp[65536];
                const ssize_t n = ::recv(fd_, tmp, sizeof(tmp), 0);
                if (n <= 0) {
                    return true;
                }
                buf_.append(tmp, static_cast<size_t>(n));
            }
        }
        return false;
    }

    // Reads until nothing has arrived for `quiet_ms`, then parses what came.
    void settle(int quiet_ms = 150) {
        while (true) {
            pollfd p{fd_, POLLIN, 0};
            if (::poll(&p, 1, quiet_ms) <= 0) {
                break;
            }
            char tmp[65536];
            const ssize_t n = ::recv(fd_, tmp, sizeof(tmp), 0);
            if (n <= 0) {
                break;
            }
            buf_.append(tmp, static_cast<size_t>(n));
        }
        take_interleaved();
    }

    std::vector<std::pair<int, std::vector<uint8_t>>> &packets() { return packets_; }

  private:
    bool read_more(std::chrono::steady_clock::time_point deadline) {
        const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        if (left.count() <= 0) {
            return false;
        }
        pollfd p{fd_, POLLIN, 0};
        if (::poll(&p, 1, static_cast<int>(left.count())) <= 0) {
            return false;
        }
        char tmp[65536];
        const ssize_t n = ::recv(fd_, tmp, sizeof(tmp), 0);
        if (n <= 0) {
            return false;
        }
        buf_.append(tmp, static_cast<size_t>(n));
        return true;
    }

    void take_interleaved() {
        while (buf_.size() >= 4 && buf_[0] == '$') {
            const size_t len = (static_cast<uint8_t>(buf_[2]) << 8) | static_cast<uint8_t>(buf_[3]);
            if (buf_.size() < 4 + len) {
                return;
            }
            packets_.emplace_back(
                static_cast<uint8_t>(buf_[1]),
                std::vector<uint8_t>(buf_.begin() + 4, buf_.begin() + 4 + static_cast<long>(len)));
            buf_.erase(0, 4 + len);
        }
    }

    int fd_{-1};
    bool connected_{false};
    int cseq_{0};
    std::string buf_;
    std::vector<std::pair<int, std::vector<uint8_t>>> packets_;
};

// Puts RTP/H.264 (single NAL + FU-A) back into access units, split at the marker bit.
struct Depacketizer {
    struct Frame {
        uint32_t timestamp{0};
        std::vector<std::vector<uint8_t>> nals;
        bool has(uint8_t type) const {
            for (const auto &n : nals) {
                if (!n.empty() && (n[0] & 0x1F) == type) {
                    return true;
                }
            }
            return false;
        }
    };
    std::vector<Frame> frames;
    std::vector<std::vector<uint8_t>> current;
    std::vector<uint8_t> fu;
    uint16_t last_seq{0};
    bool have_seq{false};
    size_t seq_gaps{0};
    uint32_t ssrc{0};

    void push(const std::vector<uint8_t> &pkt) {
        if (pkt.size() < 13 || (pkt[0] >> 6) != 2 || (pkt[1] & 0x7F) != 96) {
            return;
        }
        const uint16_t seq = static_cast<uint16_t>((pkt[2] << 8) | pkt[3]);
        if (have_seq && seq != static_cast<uint16_t>(last_seq + 1)) {
            ++seq_gaps;
        }
        have_seq = true;
        last_seq = seq;
        const uint32_t ts =
            (uint32_t{pkt[4]} << 24) | (uint32_t{pkt[5]} << 16) | (uint32_t{pkt[6]} << 8) | pkt[7];
        ssrc = (uint32_t{pkt[8]} << 24) | (uint32_t{pkt[9]} << 16) | (uint32_t{pkt[10]} << 8) | pkt[11];
        const uint8_t *p = pkt.data() + 12;
        const size_t n = pkt.size() - 12;
        const uint8_t type = p[0] & 0x1F;
        if (type == 28 && n >= 2) {
            if ((p[1] & 0x80) != 0) {
                fu.assign(1, static_cast<uint8_t>((p[0] & 0xE0) | (p[1] & 0x1F)));
            }
            fu.insert(fu.end(), p + 2, p + n);
            if ((p[1] & 0x40) != 0) {
                current.push_back(fu);
                fu.clear();
            }
        } else if (type >= 1 && type <= 23) {
            current.emplace_back(p, p + n);
        }
        if ((pkt[1] & 0x80) != 0) {
            frames.push_back({ts, current});
            current.clear();
        }
    }
};

} // namespace rtsp_test
