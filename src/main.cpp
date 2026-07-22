// skysim entry point. M1 wires: config -> World -> UdpEndpoints -> tick loop.
#include <cstdio>

int main(int argc, char** argv) {
    (void)argc; (void)argv;
    // TODO(M1): parse --vehicles, --time-mode {strict|interactive}, --dt, --base-port (9002),
    //           --tiles <dir>; construct core::World; run tick loop per docs/DESIGN.md.
    std::printf("skysim: scaffold build OK — implement per docs/MILESTONES.md\n");
    return 0;
}
