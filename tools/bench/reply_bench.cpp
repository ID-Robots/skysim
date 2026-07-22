// Reply-path microbenchmark: build_state_json + UdpEndpoint::send_reply per vehicle, serial
// vs fanned across a ThreadPool. This is the kernel-bound reply path the io pool targets.
// Usage: reply_bench [N=100] [ticks=2000] [io_threads=3]
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
    const int n = argc > 1 ? std::atoi(argv[1]) : 100;
    const int ticks = argc > 2 ? std::atoi(argv[2]) : 2000;
    const int io_threads = argc > 3 ? std::atoi(argv[3]) : 3;
    const int base = 400; // instances 400..: ports 9002+10*I, far from anything

    std::vector<Slot> fleet(static_cast<size_t>(n));
    std::atomic<bool> stop{false};
    std::vector<std::thread> drains;
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

    std::printf("reply_bench: N=%d ticks=%d io_threads=%d\n", n, ticks, io_threads);
    std::printf("  serial   p50=%7.1f  p99=%7.1f us\n", pct(serial, 0.5), pct(serial, 0.99));
    std::printf("  pooled   p50=%7.1f  p99=%7.1f us  (%.2fx p50)\n", pct(parallel, 0.5), pct(parallel, 0.99),
                pct(serial, 0.5) / pct(parallel, 0.5));

    stop.store(true);
    for (int i = 0; i < n; ++i) {
        // nudge each drain out of recv
        int w = ::socket(AF_INET, SOCK_DGRAM, 0);
        sockaddr_in dst{};
        dst.sin_family = AF_INET;
        dst.sin_port = htons(static_cast<uint16_t>(9002 + 10 * (base + i)));
        inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);
        char z = 0;
        ::sendto(w, &z, 1, 0, reinterpret_cast<sockaddr *>(&dst), sizeof(dst));
        ::close(w);
    }
    for (auto &d : drains) {
        d.join();
    }
    return 0;
}
