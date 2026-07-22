// Protocol hot-path microbenchmark: build_state_json + parse_servo_datagram throughput.
// These run once per vehicle per tick on the tick thread, so at 200 veh * 800 Hz that is
// 160k calls/s each. Usage: proto_bench [iterations=2000000]
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "protocol/packets.h"

using Clock = std::chrono::steady_clock;

int main(int argc, char **argv) {
    const long iters = argc > 1 ? std::atol(argv[1]) : 2'000'000;

    // --- build_state_json ---
    skysim::protocol::VehicleTruth t{};
    t.timestamp_s = 12.3456;
    t.gyro_rps[0] = 0.01;
    t.gyro_rps[1] = -0.02;
    t.gyro_rps[2] = 0.03;
    t.accel_body[2] = -9.81;
    t.pos_ned_m[0] = 12.5;
    t.pos_ned_m[1] = -7.25;
    t.pos_ned_m[2] = -50.0;
    t.vel_ned_mps[0] = 3.5;
    t.quat_wxyz[0] = 0.98;
    t.quat_wxyz[3] = 0.2;
    t.rangefinder_m[0] = 41.3;
    t.rangefinder_count = 1;

    char buf[1024];
    volatile size_t sink = 0;
    auto b0 = Clock::now();
    for (long i = 0; i < iters; ++i) {
        t.timestamp_s += 1e-6;
        sink += skysim::protocol::build_state_json(t, buf, sizeof(buf));
    }
    auto b1 = Clock::now();
    const double build_ns = std::chrono::duration<double, std::nano>(b1 - b0).count() / iters;

    // --- parse_servo_datagram ---
    skysim::protocol::ServoPacket16 pkt{};
    pkt.magic = skysim::protocol::kServoMagic16;
    pkt.frame_rate = 800;
    for (int k = 0; k < 16; ++k) {
        pkt.pwm[k] = static_cast<uint16_t>(1500 + k);
    }
    std::vector<std::byte> raw(sizeof(pkt));
    std::memcpy(raw.data(), &pkt, sizeof(pkt));
    volatile uint32_t psink = 0;
    auto p0 = Clock::now();
    for (long i = 0; i < iters; ++i) {
        std::memcpy(raw.data() + 4, &i, 4); // vary frame_count
        auto r = skysim::protocol::parse_servo_datagram(raw, static_cast<uint32_t>(i - 1));
        psink += r.pwm[0];
    }
    auto p1 = Clock::now();
    const double parse_ns = std::chrono::duration<double, std::nano>(p1 - p0).count() / iters;

    std::printf("proto_bench (%ld iters):\n", iters);
    std::printf("  build_state_json:     %6.1f ns/call  (%.2f M/s)\n", build_ns, 1000.0 / build_ns);
    std::printf("  parse_servo_datagram: %6.1f ns/call  (%.2f M/s)\n", parse_ns, 1000.0 / parse_ns);
    std::printf("  per-vehicle reply cost ~ %.1f ns; at 200 veh*800Hz = %.1f%% of one core\n",
                build_ns + parse_ns, (build_ns + parse_ns) * 200 * 800 / 1e9 * 100);
    (void)sink;
    (void)psink;
    return 0;
}
