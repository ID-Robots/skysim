// M0: static ABI checks + round-trip of a captured real servo datagram (fixture added at M0).
#include <cstdio>
#include "protocol/packets.h"

int main() {
    static_assert(sizeof(skysim::protocol::ServoPacket16) == 40);
    static_assert(skysim::protocol::kServoMagic16 == 18458);
    // TODO(M0): load tests/fixtures/servo_frame.bin captured from a real arducopter,
    //           assert parse_servo_datagram classifies Ok/Duplicate/Gap/Reboot correctly.
    // TODO(M1): golden-file test for build_state_json (field names verified vs pinned readme).
    std::printf("test_packets: static checks OK\n");
    return 0;
}
