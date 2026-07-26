// Reply-path microbenchmark: build_state_json + UdpEndpoint::send_reply per vehicle, serial
// vs fanned across a ThreadPool. This is the kernel-bound reply path the io pool targets.
// Usage: reply_bench [N=100] [ticks=2000] [io_threads=3] [--max-pooled-p99-us N]
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

#include "core/thread_pool.h"
#include "protocol/packets.h"
#include "protocol/udp_endpoint.h"

using Clock = std::chrono::steady_clock;

namespace {
double pct(std::vector<double> &v, double p) {
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, static_cast<size_t>(v.size() * p))];
}

struct Slot {
    std::unique_ptr<skysim::protocol::UdpEndpoint> ep;
    char json[1024];
    size_t json_len = 0;
    skysim::protocol::VehicleTruth truth{};
};

void build_and_send(Slot &s) {
    s.truth.timestamp_s += 1.0 / 800.0;
    s.json_len = skysim::protocol::build_state_json(s.truth, s.json, sizeof(s.json));
    s.ep->send_reply(s.json, s.json_len);
}
} // namespace

int main(int argc, char **argv) {
    int n = 100;
    int ticks = 2000;
    int io_threads = 3;
    double max_pooled_p99_us = -1.0;
    int positional = 0;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--max-pooled-p99-us") == 0 && i + 1 < argc) {
            max_pooled_p99_us = std::atof(argv[++i]);
        } else if (argv[i][0] == '-') {
            std::fprintf(stderr, "unknown or incomplete option: %s\n", argv[i]);
            return 2;
        } else if (positional++ == 0) {
            n = std::atoi(argv[i]);
        } else if (positional == 2) {
            ticks = std::atoi(argv[i]);
        } else if (positional == 3) {
            io_threads = std::atoi(argv[i]);
        } else {
            std::fprintf(stderr, "unexpected argument: %s\n", argv[i]);
            return 2;
        }
    }
    if (n <= 0 || ticks <= 0 || io_threads <= 0 || max_pooled_p99_us == 0.0) {
        std::fprintf(stderr, "N, ticks, io_threads, and --max-pooled-p99-us (when set) must be positive\n");
        return 2;
    }
    const int base = 400; // instances 400..: ports 9002+10*I, far from anything

    std::vector<Slot> fleet(static_cast<size_t>(n));
    std::atomic<bool> stop{false};
    std::vector<std::thread> drains;
    std::vector<sockaddr_in> drain_addrs;
    drain_addrs.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        auto &s = fleet[static_cast<size_t>(i)];
        s.ep = std::make_unique<skysim::protocol::UdpEndpoint>(base + i);
        s.truth.quat_wxyz[0] = 1.0;
        s.truth.accel_body[2] = -9.81;
        // A peer that sends one servo (so the endpoint learns the reply address) then drains.
        int peer = ::socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in dst{};
        dst.sin_family = AF_INET;
        dst.sin_port = htons(static_cast<uint16_t>(9002 + 10 * (base + i)));
        inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);
        skysim::protocol::ServoPacket16 pkt{};
        pkt.magic = skysim::protocol::kServoMagic16;
        ::sendto(peer, &pkt, sizeof(pkt), 0, reinterpret_cast<sockaddr *>(&dst), sizeof(dst));
        sockaddr_in peer_addr{};
        socklen_t peer_addr_len = sizeof(peer_addr);
        if (::getsockname(peer, reinterpret_cast<sockaddr *>(&peer_addr), &peer_addr_len) != 0) {
            std::perror("getsockname");
            return 1;
        }
        peer_addr.sin_addr = dst.sin_addr;
        drain_addrs.push_back(peer_addr);
        drains.emplace_back([peer, &stop] {
            char b[512];
            while (!stop.load()) {
                ::recv(peer, b, sizeof(b), 0);
            }
            ::close(peer);
        });
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); // let endpoints latch the peer
    for (auto &s : fleet) {
        (void)s.ep->take_latest(); // consume the priming servo so send_reply has an address
    }

    auto measure = [&](skysim::core::ThreadPool *pool) {
        std::vector<double> t;
        t.reserve(ticks);
        constexpr size_t kThresh = 48;
        for (int k = 0; k < ticks; ++k) {
            const auto a = Clock::now();
            if (pool && fleet.size() > kThresh) {
                pool->parallel_for(fleet.size(), [&](size_t b, size_t e) {
                    for (size_t i = b; i < e; ++i) {
                        build_and_send(fleet[i]);
                    }
                });
            } else {
                for (auto &s : fleet) {
                    build_and_send(s);
                }
            }
            t.push_back(std::chrono::duration<double, std::micro>(Clock::now() - a).count());
        }
        return t;
    };

    auto serial = measure(nullptr);
    skysim::core::ThreadPool pool(io_threads);
    auto parallel = measure(&pool);

    const double serial_p50 = pct(serial, 0.5);
    const double serial_p99 = pct(serial, 0.99);
    const double pooled_p50 = pct(parallel, 0.5);
    const double pooled_p99 = pct(parallel, 0.99);
    std::printf("reply_bench: N=%d ticks=%d io_threads=%d\n", n, ticks, io_threads);
    std::printf("  serial   p50=%7.1f  p99=%7.1f us\n", serial_p50, serial_p99);
    std::printf("  pooled   p50=%7.1f  p99=%7.1f us  (%.2fx p50)\n", pooled_p50, pooled_p99,
                serial_p50 / pooled_p50);

    stop.store(true);
    for (int i = 0; i < n; ++i) {
        // nudge each drain out of recv
        int w = ::socket(AF_INET, SOCK_DGRAM, 0);
        char z = 0;
        ::sendto(w, &z, 1, 0, reinterpret_cast<sockaddr *>(&drain_addrs[static_cast<size_t>(i)]),
                 sizeof(drain_addrs[static_cast<size_t>(i)]));
        ::close(w);
    }
    for (auto &d : drains) {
        d.join();
    }
    if (max_pooled_p99_us > 0.0) {
        const bool pass = pooled_p99 <= max_pooled_p99_us;
        std::printf("  performance gate: pooled p99 %.1f us <= %.1f us => %s\n", pooled_p99,
                    max_pooled_p99_us, pass ? "PASS" : "FAIL");
        return pass ? 0 : 1;
    }
    return 0;
}
