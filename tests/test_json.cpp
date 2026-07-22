// M1: build_state_json golden checks — compact single-line JSON obeying ArduPilot's strstr
// parser rules (docs/PROTOCOL.md): no spaces, '\n'-terminated, keytable field names.
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>

#include "protocol/packets.h"

namespace {

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

} // namespace

int main() {
    skysim::protocol::VehicleTruth t{};
    t.timestamp_s = 12.3456;
    t.gyro_rps[0] = 0.01;
    t.gyro_rps[1] = -0.02;
    t.gyro_rps[2] = 0.03;
    t.accel_body[0] = 0.0;
    t.accel_body[1] = 0.0;
    t.accel_body[2] = -9.81;
    t.pos_ned_m[0] = 1.5;
    t.pos_ned_m[1] = -2.5;
    t.pos_ned_m[2] = -10.0;
    t.vel_ned_mps[0] = 0.5;
    t.vel_ned_mps[1] = 0.0;
    t.vel_ned_mps[2] = -1.0;
    t.quat_wxyz[0] = 1.0;
    t.quat_wxyz[1] = 0.0;
    t.quat_wxyz[2] = 0.0;
    t.quat_wxyz[3] = 0.0;

    char buf[1024];
    const size_t n = skysim::protocol::build_state_json(t, buf, sizeof(buf));
    CHECK(n > 0);
    const std::string s(buf, n);

    const std::string expected =
        "{\"timestamp\":12.345600,"
        "\"imu\":{\"gyro\":[0.010000,-0.020000,0.030000],\"accel_body\":[0.000000,0.000000,-9.810000]},"
        "\"position\":[1.500000,-2.500000,-10.000000],\"velocity\":[0.500000,0.000000,-1.000000],"
        "\"quaternion\":[1.0000000,0.0000000,0.0000000,0.0000000]}\n";
    if (s != expected) {
        std::printf("FAIL golden mismatch:\n  got: %s  want: %s", s.c_str(), expected.c_str());
        ++g_failures;
    }

    // Parser-rule invariants (hold for any state, not just the golden one).
    CHECK(s.find(' ') == std::string::npos);
    CHECK(s.back() == '\n');
    CHECK(s.find('\n') == s.size() - 1);
    CHECK(s.find("\"velocity\"") != std::string::npos);
    CHECK(s.find("velocity") == s.rfind("velocity")); // exactly one "velocity" substring

    // Rangefinders: optional keys appended with exact keytable spelling rng_1..rng_6.
    t.rangefinder_m = {1.25, 2.5, 3.75, 5.0, 6.25, 7.5};
    const size_t n2 = skysim::protocol::build_state_json(t, buf, sizeof(buf));
    CHECK(n2 > n);
    const std::string s2(buf, n2);
    CHECK(s2.find("\"rng_1\":1.2500") != std::string::npos);
    CHECK(s2.find("\"rng_6\":7.5000") != std::string::npos);
    CHECK(s2.find(' ') == std::string::npos);
    CHECK(s2.back() == '\n');

    // Overflow contract: returns 0 when the buffer is too small.
    char tiny[32];
    CHECK(skysim::protocol::build_state_json(t, tiny, sizeof(tiny)) == 0);

    // Non-finite guard (PROTOCOL.md: no NaN/Inf ever reaches the wire — ArduPilot's
    // skip-to-digit parser walks over "nan"/"inf" and desyncs the whole field scan).
    skysim::protocol::VehicleTruth bad{};
    bad.timestamp_s = 1.0;
    bad.gyro_rps[0] = std::numeric_limits<double>::quiet_NaN();
    bad.vel_ned_mps[2] = std::numeric_limits<double>::infinity();
    bad.pos_ned_m[1] = -std::numeric_limits<double>::infinity();
    bad.quat_wxyz[0] = std::numeric_limits<double>::quiet_NaN(); // whole quat -> identity
    const size_t n3 = skysim::protocol::build_state_json(bad, buf, sizeof(buf));
    CHECK(n3 > 0);
    const std::string s3(buf, n3);
    CHECK(s3.find("nan") == std::string::npos);
    CHECK(s3.find("inf") == std::string::npos);
    CHECK(s3.find("\"quaternion\":[1.0000000,0.0000000,0.0000000,0.0000000]") != std::string::npos);

    if (g_failures == 0) {
        std::printf("test_json: all checks OK\n");
        return 0;
    }
    std::printf("test_json: %d failure(s)\n", g_failures);
    return 1;
}
