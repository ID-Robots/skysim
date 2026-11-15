// The camera as H.264 over RTSP (TASK-2561): bitstream helpers, the x264 wrapper, RTP/RTCP
// packets, the RTSP server over real sockets (TCP interleaved and UDP), the video service that
// ties them to the renderer, and the renderer's video demand.
#include "render/render_service.h"
#include "render/scene.h"
#include "video/h264.h"
#include "video/h264_encoder.h"
#include "video/rtp_h264.h"
#include "video/rtsp_server.h"
#include "video/video_service.h"

#include "rtsp_test_client.h"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <set>
#include <stdexcept>
#include <thread>

namespace {

using namespace std::chrono_literals;
using namespace skysim::video;
using skysim::render::Image;

int g_failures = 0;

#define CHECK(cond)                                                                                          \
    do {                                                                                                     \
        if (!(cond)) {                                                                                       \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                                      \
            ++g_failures;                                                                                    \
        }                                                                                                    \
    } while (0)

// A test card that moves with `t`: gradients plus a bright square, so P-frames carry motion.
Image card(int w, int h, int t) {
    Image img;
    img.width = w;
    img.height = h;
    img.rgb.resize(static_cast<size_t>(w) * h * 3);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            uint8_t *p = &img.rgb[(static_cast<size_t>(y) * w + x) * 3];
            const bool square = ((x + 3 * t) % w) < w / 4 && y > h / 3 && y < 2 * h / 3;
            p[0] = square ? 250 : static_cast<uint8_t>(x * 255 / w);
            p[1] = square ? 250 : static_cast<uint8_t>(y * 255 / h);
            p[2] = static_cast<uint8_t>((x + y + t) & 0xFF);
        }
    }
    return img;
}

Image solid(int w, int h, uint8_t r, uint8_t g, uint8_t b) {
    Image img;
    img.width = w;
    img.height = h;
    for (int i = 0; i < w * h; ++i) {
        img.rgb.insert(img.rgb.end(), {r, g, b});
    }
    return img;
}

std::vector<uint8_t> bytes(std::initializer_list<int> v) {
    std::vector<uint8_t> out;
    for (int b : v) {
        out.push_back(static_cast<uint8_t>(b));
    }
    return out;
}

AccessUnit synthetic(bool idr, size_t slice_bytes, uint8_t fill = 0x55) {
    AccessUnit au;
    au.idr = idr;
    au.captured = std::chrono::steady_clock::now();
    if (idr) {
        au.nals.push_back(bytes({0x67, 0x42, 0xE0, 0x1F, 0xAA}));
        au.nals.push_back(bytes({0x68, 0xCE, 0x38, 0x80}));
    }
    std::vector<uint8_t> slice(slice_bytes, fill);
    slice[0] = idr ? 0x65 : 0x41;
    au.nals.push_back(slice);
    return au;
}

void test_bitstream_helpers() {
    // Annex B: 4- and 3-byte start codes, junk before the first, trailing zeros dropped.
    const auto stream = bytes({0xFF, 0, 0, 0, 1, 0x67, 1, 2, 0, 0, 1, 0x68, 3, 0, 0, 0, 0, 1, 0x65, 9, 9});
    const auto nals = split_annexb(stream.data(), stream.size());
    CHECK(nals.size() == 3);
    CHECK(nals.size() == 3 && nals[0] == bytes({0x67, 1, 2}) && nals[1] == bytes({0x68, 3}) &&
          nals[2] == bytes({0x65, 9, 9}));
    CHECK(split_annexb(stream.data(), 2).empty());
    CHECK(nal_type(bytes({0x65})) == kNalIdr && nal_type({}) == 0);

    CHECK(profile_level_id(bytes({0x67, 0x42, 0xE0, 0x1F})) == "42e01f");
    CHECK(profile_level_id(bytes({0x68, 0x42, 0xE0, 0x1F})).empty());
    CHECK(profile_level_id(bytes({0x67, 0x42})).empty());

    // RFC 4648 test vectors.
    const char *plain[] = {"", "f", "fo", "foo", "foob", "fooba", "foobar"};
    const char *coded[] = {"", "Zg==", "Zm8=", "Zm9v", "Zm9vYg==", "Zm9vYmE=", "Zm9vYmFy"};
    for (int i = 0; i < 7; ++i) {
        const std::string s = plain[i];
        CHECK(base64(std::vector<uint8_t>(s.begin(), s.end())) == coded[i]);
    }

    // Levels: the floor is 3.1, then the smallest that holds the frame and its macroblock rate.
    CHECK(level_for(256, 144, 10) == 31);
    CHECK(level_for(1280, 720, 30) == 31);
    CHECK(level_for(1280, 720, 60) == 40);
    CHECK(level_for(1920, 1080, 30) == 40);
    CHECK(level_for(1920, 1080, 60) == 42);
    CHECK(level_for(2560, 1440, 30) == 50);
    CHECK(level_for(3840, 2160, 30) == 51);
    CHECK(level_for(8192, 8192, 60) == 52);
    CHECK(level_for(8000, 16, 1) == 51); // 500 MBs wide: past sqrt(8 * MaxFS) of every level below

    // SPS parsing: not an SPS, truncated, a High-profile layout with a scaling matrix (not parsed),
    // and a POC type 1 SPS with an over-long cycle.
    CHECK(!parse_sps(bytes({0x68, 0x42, 0xE0, 0x1F})).ok);
    CHECK(!parse_sps(bytes({0x67, 0x42, 0xE0})).ok);
    CHECK(!parse_sps(bytes({0x67, 0x42, 0xE0, 0x1F, 0x00})).ok); // ue() runs off the end
    // profile 100, id 0, chroma 1, depths 0, bypass 0, scaling matrix 1 -> not parsed
    CHECK(!parse_sps(bytes({0x67, 0x64, 0x00, 0x1F, 0xAD})).ok);
    // profile 66, sps id 0, log2_max_frame_num 0, poc type 1, always_zero 0, se 0, se 0, cycle 300
    CHECK(!parse_sps(bytes({0x67, 0x42, 0xE0, 0x1F, 0xD3, 0x00, 0x96, 0x80})).ok);
}

void test_encoder() {
    std::string error;
    CHECK(H264Encoder::create({8, 8, 10.0, 500}, &error) == nullptr && !error.empty());
    CHECK(H264Encoder::create({64, 48, 0.0, 500}, &error) == nullptr);
    CHECK(H264Encoder::create({64, 48, 10.0, 0}, &error) == nullptr);
    CHECK(H264Encoder::create({64, 48, 10.0, 500, "warp-speed"}, &error) == nullptr &&
          error == "unknown x264 preset");
    CHECK(H264Encoder::create({64, 48, 10.0, 500}) != nullptr); // no error pointer needed

    auto enc = H264Encoder::create({256, 144, 10.0, 1200}, &error);
    CHECK(enc != nullptr);
    if (!enc) {
        return;
    }
    // The SDP's profile-level-id: Constrained Baseline with constraint_set2, level 3.1.
    CHECK(profile_level_id(enc->sps()) == "42e01f");
    const SpsInfo sps = parse_sps(enc->sps());
    CHECK(sps.ok && sps.profile_idc == 66 && sps.level_idc == 31);
    CHECK((sps.constraint_flags & 0xE0) == 0xE0);
    CHECK(sps.width == 256 && sps.height == 144 && sps.frame_mbs_only && sps.direct_8x8_inference);
    // VUI timing: time_scale / (2 * num_units_in_tick) is the frame rate.
    CHECK(sps.timing_info && sps.num_units_in_tick > 0 &&
          std::fabs(sps.time_scale / (2.0 * sps.num_units_in_tick) - 10.0) < 1e-9);
    CHECK(nal_type(enc->pps()) == kNalPps);
    CHECK(enc->config().preset == "veryfast" && enc->width() == 256 && enc->height() == 144);

    AccessUnit au;
    size_t p_frames = 0;
    for (int i = 0; i < 20; ++i) {
        const bool force = i == 0 || i == 12;
        CHECK(enc->encode(card(256, 144, i), force, au));
        bool sps_seen = false;
        bool pps_seen = false;
        bool idr_seen = false;
        for (const auto &nal : au.nals) {
            const uint8_t t = nal_type(nal);
            sps_seen = sps_seen || t == kNalSps;
            pps_seen = pps_seen || t == kNalPps;
            idr_seen = idr_seen || t == kNalIdr;
            if (t == kNalSps) {
                CHECK(nal == enc->sps()); // the in-band SPS says 42e01f too
            }
        }
        // IDRs exactly where forced (x264's own GOP is infinite), and every one carries SPS + PPS.
        CHECK(au.idr == force && idr_seen == force);
        CHECK(!force || (sps_seen && pps_seen));
        CHECK(force || (!sps_seen && !pps_seen));
        p_frames += force ? 0 : 1;
    }
    CHECK(p_frames == 18 && enc->frames() == 20);

    // Too small a picture is refused; a larger one is cropped to the configured size.
    CHECK(!enc->encode(card(128, 72, 0), false, au) && au.nals.empty());
    CHECK(enc->encode(card(300, 200, 21), false, au));

    // Bitrate in place, no IDR: a lower target spends fewer bytes on the same content.
    CHECK(!enc->set_bitrate(0));
    CHECK(enc->set_bitrate(150) && enc->config().bitrate_kbps == 150);
    const auto spend = [](int kbps) {
        auto e = H264Encoder::create({256, 144, 10.0, kbps});
        size_t total = 0;
        AccessUnit a;
        for (int i = 0; i < 30; ++i) {
            e->encode(card(256, 144, i * 7), i == 0, a);
            total += a.bytes();
        }
        return total;
    };
    const size_t low = spend(100);
    const size_t high = spend(3000);
    std::printf("test_video: 30 frames at 100 kbps = %zu bytes, at 3000 kbps = %zu bytes\n", low, high);
    CHECK(low * 2 < high);

    // Odd sizes round down to even; a fractional rate is a rational in the VUI.
    auto odd = H264Encoder::create({255, 143, 7.5, 400});
    CHECK(odd && odd->width() == 254 && odd->height() == 142);
    if (odd) {
        const SpsInfo o = parse_sps(odd->sps());
        CHECK(o.ok && o.width == 254 && o.height == 142);
        CHECK(std::fabs(o.time_scale / (2.0 * o.num_units_in_tick) - 7.5) < 1e-9);
        CHECK(odd->encode(card(255, 143, 1), true, au) && au.idr);
    }
    // A frame size past level 3.1 is signalled at a higher level, still Constrained Baseline.
    auto big = H264Encoder::create({1920, 1080, 30.0, 4000});
    CHECK(big && profile_level_id(big->sps()) == "42e028");

    // Colour conversion: BT.601 limited range.
    uint8_t y[4];
    uint8_t u[1];
    uint8_t v[1];
    rgb_to_i420(solid(2, 2, 255, 255, 255), 2, 2, y, u, v);
    CHECK(y[0] == 235 && y[3] == 235 && u[0] == 128 && v[0] == 128);
    rgb_to_i420(solid(2, 2, 0, 0, 0), 2, 2, y, u, v);
    CHECK(y[0] == 16 && u[0] == 128 && v[0] == 128);
    rgb_to_i420(solid(2, 2, 255, 0, 0), 2, 2, y, u, v);
    CHECK(y[0] == 82 && u[0] == 90 && v[0] == 240);
}

void test_rtp() {
    AccessUnit au;
    au.nals.push_back(std::vector<uint8_t>(100, 0x11));
    au.nals[0][0] = 0x67;
    au.nals.push_back({});                               // empty NALs are skipped
    au.nals.push_back(std::vector<uint8_t>(5000, 0x22)); // FU-A
    au.nals.back()[0] = 0x65;
    au.nals.back()[4999] = 0x99;
    RtpPacketizer rtp(0xDEADBEEF, 65534);
    std::vector<std::vector<uint8_t>> packets;
    rtp.packetize(au, 123456, [&](const uint8_t *p, size_t n) { packets.emplace_back(p, p + n); });
    // 1 single NAL packet + ceil(4999 / 1398) = 4 fragments.
    CHECK(packets.size() == 5);
    CHECK(rtp.packets() == 5 && rtp.next_seq() == 3 && rtp.ssrc() == 0xDEADBEEF);
    rtsp_test::Depacketizer depack;
    for (size_t i = 0; i < packets.size(); ++i) {
        const auto &p = packets[i];
        CHECK(p.size() <= kRtpHeaderBytes + kRtpMaxPayload);
        CHECK(p[0] == 0x80 && (p[1] & 0x7F) == 96);
        CHECK(((p[1] & 0x80) != 0) == (i + 1 == packets.size())); // marker on the last only
        CHECK(p[4] == 0x00 && p[5] == 0x01 && p[6] == 0xE2 && p[7] == 0x40);
        CHECK(p[8] == 0xDE && p[11] == 0xEF);
        depack.push(p);
    }
    CHECK(packets[1][12] == ((0x65 & 0xE0) | 28) && packets[1][13] == (0x80 | 5)); // FU-A start
    CHECK(packets[4][13] == (0x40 | 5));                                           // FU-A end
    CHECK(depack.seq_gaps == 0);                                                   // 65534, 65535, 0, 1, 2
    CHECK(depack.frames.size() == 1 && depack.frames[0].nals.size() == 2);
    CHECK(depack.frames.size() == 1 && depack.frames[0].nals[0] == au.nals[0] &&
          depack.frames[0].nals[1] == au.nals[2]);
    CHECK(rtp.octets() == 100 + 4999 + 4 * 2);

    // Trailing empty entries do not take the marker with them, single NAL or FU-A.
    for (const size_t size : {50, 3000}) {
        AccessUnit trailing;
        trailing.nals.push_back(std::vector<uint8_t>(size, 0x41));
        trailing.nals.push_back({});
        trailing.nals.push_back({});
        RtpPacketizer one(7, 0);
        std::vector<std::vector<uint8_t>> out;
        one.packetize(trailing, 0, [&](const uint8_t *p, size_t n) { out.emplace_back(p, p + n); });
        CHECK(!out.empty() && (out.back()[1] & 0x80) != 0);
        size_t marked = 0;
        for (const auto &p : out) {
            marked += (p[1] & 0x80) != 0 ? 1 : 0;
        }
        CHECK(marked == 1);
    }

    // A tiny MTU still fragments correctly (floored at 16 bytes of payload).
    RtpPacketizer tiny(1, 0, 4);
    size_t count = 0;
    tiny.packetize(au, 0, [&](const uint8_t *, size_t n) {
        CHECK(n <= kRtpHeaderBytes + 16);
        ++count;
    });
    CHECK(count > 300);

    // NTP: the Unix epoch is 2208988800 s after 1900; half a second is 0x80000000.
    const auto epoch = std::chrono::system_clock::time_point{} + 500ms;
    CHECK(ntp_time(epoch) == ((2208988800ULL << 32) | 0x80000000ULL));
    const auto sr = rtcp_sender_report(0x01020304, 0x1122334455667788ULL, 90000, 7, 999, "skysim");
    CHECK(sr.size() == 28 + 4 + 16);
    CHECK(sr[0] == 0x80 && sr[1] == 200 && sr[2] == 0 && sr[3] == 6);
    CHECK(sr[4] == 1 && sr[7] == 4 && sr[8] == 0x11 && sr[15] == 0x88);
    CHECK(sr[16] == 0 && sr[17] == 1 && sr[18] == 0x5F && sr[19] == 0x90);
    CHECK(sr[23] == 7 && sr[26] == 0x03 && sr[27] == 0xE7);
    CHECK(sr[28] == 0x81 && sr[29] == 202 && sr[31] == 4 && sr[36] == 1 && sr[37] == 6 && sr[38] == 's');
}

void test_parsers() {
    CHECK(parse_stream_path("rtsp://10.0.0.1:8554/instances/3/main.264") == 3);
    CHECK(parse_stream_path("rtsp://h/instances/0/main.264/trackID=0") == 0);
    CHECK(parse_stream_path("/instances/12/main.264?x=1") == 12);
    CHECK(parse_stream_path("/instances/12/main.264/") == 12);
    CHECK(!parse_stream_path("rtsp://h:8554/main.264"));
    CHECK(!parse_stream_path("rtsp://h:8554"));
    CHECK(!parse_stream_path("/instances/x/main.264"));
    CHECK(!parse_stream_path("/instances/1"));
    CHECK(!parse_stream_path("/instances/1/main.265"));
    CHECK(!parse_stream_path("/instances/1/main.2640"));
    CHECK(!parse_stream_path("/instances/99999999999/main.264"));
    CHECK(!parse_stream_path("*"));

    auto t = parse_transport("RTP/AVP;unicast;client_port=5000-5001");
    CHECK(t && !t->tcp && t->client_rtp_port == 5000 && t->client_rtcp_port == 5001);
    t = parse_transport("RTP/AVP/UDP;unicast;client_port=6000");
    CHECK(t && !t->tcp && t->client_rtp_port == 6000 && t->client_rtcp_port == 6001);
    t = parse_transport("RTP/AVP/TCP;unicast;interleaved=2-3");
    CHECK(t && t->tcp && t->rtp_channel == 2 && t->rtcp_channel == 3);
    t = parse_transport("rtp/avp/tcp;unicast");
    CHECK(t && t->tcp && t->rtp_channel == 0 && t->rtcp_channel == 1);
    // A list: multicast and secure profiles are skipped for the first one this server serves.
    t = parse_transport("RTP/AVP;multicast;port=5000-5001, RTP/SAVP;unicast;client_port=1-2 ,"
                        " RTP/AVP/TCP;interleaved=4");
    CHECK(t && t->tcp && t->rtp_channel == 4 && t->rtcp_channel == 5);
    CHECK(!parse_transport("RTP/AVP;unicast"));                 // UDP needs client ports
    CHECK(!parse_transport("RTP/AVP;unicast;interleaved=0-1")); // ...channels are not ports
    CHECK(!parse_transport("RTP/AVP;unicast;client_port=0-1")); // out of range
    CHECK(!parse_transport("RTP/AVP;unicast;client_port=65535"));
    CHECK(!parse_transport("RTP/AVP;unicast;client_port=a-b"));
    CHECK(!parse_transport("RTP/AVP;unicast;client_port=5000-x"));
    CHECK(!parse_transport("RTP/AVP/TCP;interleaved=255"));
    CHECK(!parse_transport(""));
}

// A UDP pair bound on loopback, as a receiver's client_port.
struct UdpPair {
    int rtp{-1};
    int rtcp{-1};
    int port{0};
    UdpPair() {
        for (int attempt = 0; attempt < 50 && rtp < 0; ++attempt) {
            int a = ::socket(AF_INET, SOCK_DGRAM, 0);
            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            socklen_t len = sizeof(addr);
            ::bind(a, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
            getsockname(a, reinterpret_cast<sockaddr *>(&addr), &len);
            const int p = ntohs(addr.sin_port);
            int b = ::socket(AF_INET, SOCK_DGRAM, 0);
            addr.sin_port = htons(static_cast<uint16_t>(p + 1));
            if (p % 2 == 0 && ::bind(b, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0) {
                rtp = a;
                rtcp = b;
                port = p;
            } else {
                ::close(a);
                ::close(b);
            }
        }
    }
    ~UdpPair() {
        ::close(rtp);
        ::close(rtcp);
    }
    std::optional<std::vector<uint8_t>> recv(int fd, int timeout_ms) const {
        pollfd p{fd, POLLIN, 0};
        if (::poll(&p, 1, timeout_ms) <= 0) {
            return std::nullopt;
        }
        std::vector<uint8_t> buf(65536);
        const ssize_t n = ::recv(fd, buf.data(), buf.size(), 0);
        if (n <= 0) {
            return std::nullopt;
        }
        buf.resize(static_cast<size_t>(n));
        return buf;
    }
};

void test_rtsp_server() {
    std::mutex mutex;
    std::vector<std::pair<int, bool>> plays;
    std::atomic<int> keyframes{0};
    RtspServer::Hooks hooks;
    hooks.describe = [](int instance) -> std::optional<RtspServer::StreamInfo> {
        if (instance != 7) {
            return std::nullopt;
        }
        return RtspServer::StreamInfo{
            bytes({0x67, 0x42, 0xE0, 0x1F, 0xAA}), bytes({0x68, 0xCE, 0x38, 0x80}), 256, 144, 10.0, 1200};
    };
    hooks.playing = [&](int instance, bool on) {
        std::lock_guard<std::mutex> lock(mutex);
        plays.emplace_back(instance, on);
    };
    hooks.keyframe = [&](int) { ++keyframes; };
    RtspServer::Config cfg;
    cfg.bind = "127.0.0.1";
    cfg.port = 0;
    cfg.session_timeout_s = 1;
    cfg.max_backlog_bytes = 64 * 1024;
    RtspServer server(cfg, hooks);
    const int port = server.port();
    CHECK(port > 0 && server.udp_rtp_port() > 0 && server.udp_rtp_port() % 2 == 0);
    const auto plays_now = [&] {
        std::lock_guard<std::mutex> lock(mutex);
        return plays;
    };

    // A second server on the same port, or on an address that is not one, cannot start.
    bool threw = false;
    try {
        RtspServer::Config same = cfg;
        same.port = port;
        RtspServer clash(same, {});
    } catch (const std::runtime_error &) {
        threw = true;
    }
    CHECK(threw);
    threw = false;
    try {
        RtspServer::Config bad = cfg;
        bad.bind = "not-an-address";
        RtspServer nope(bad, {});
    } catch (const std::runtime_error &) {
        threw = true;
    }
    CHECK(threw);

    const std::string url = "rtsp://127.0.0.1:" + std::to_string(port) + "/instances/7/main.264";
    {
        rtsp_test::Client c(port);
        CHECK(c.connected());
        auto r = c.request("OPTIONS", "*");
        CHECK(r && r->status == 200 && r->header("public").find("DESCRIBE") != std::string::npos);
        CHECK(r && r->header("cseq") == "1" && r->header("server") == "skysim");

        r = c.request("DESCRIBE", "rtsp://127.0.0.1/instances/8/main.264");
        CHECK(r && r->status == 404);
        r = c.request("DESCRIBE", "rtsp://127.0.0.1/nothing");
        CHECK(r && r->status == 404);
        r = c.request("DESCRIBE", url, "Accept: application/sdp\r\n");
        CHECK(r && r->status == 200 && r->header("content-type") == "application/sdp");
        CHECK(r && r->header("content-base") == url + "/");
        CHECK(r && r->body.find("m=video 0 RTP/AVP 96\r\n") != std::string::npos);
        CHECK(r && r->body.find("a=rtpmap:96 H264/90000") != std::string::npos);
        CHECK(r &&
              r->body.find(
                  "packetization-mode=1;profile-level-id=42e01f;sprop-parameter-sets=Z0LgH6o=,aM44gA==") !=
                  std::string::npos);
        CHECK(r && r->body.find("a=framerate:10\r\n") != std::string::npos);
        CHECK(r && r->body.find("a=control:trackID=0") != std::string::npos);
        CHECK(r && r->body.find("b=AS:1200") != std::string::npos);

        r = c.request("PLAY", url);
        CHECK(r && r->status == 455); // no session yet
        r = c.request("SETUP", url + "/trackID=0");
        CHECK(r && r->status == 461); // no Transport header
        r = c.request("SETUP", "rtsp://127.0.0.1/instances/8/main.264/trackID=0",
                      "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n");
        CHECK(r && r->status == 404);
        r = c.request("SETUP", url + "/trackID=0", "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n");
        CHECK(r && r->status == 200);
        const std::string session = r ? r->session() : std::string{};
        CHECK(session.size() == 16);
        CHECK(r && r->header("session").find(";timeout=1") != std::string::npos);
        CHECK(r && r->header("transport").rfind("RTP/AVP/TCP;unicast;interleaved=0-1;ssrc=", 0) == 0);
        r = c.request("SETUP", url + "/trackID=0",
                      "Session: ffff\r\nTransport: RTP/AVP/TCP;interleaved=0-1\r\n");
        CHECK(r && r->status == 454);
        r = c.request("PLAY", url, "Session: wrong\r\n");
        CHECK(r && r->status == 454);

        // A P-frame published before anyone plays goes nowhere.
        server.publish(7, std::make_shared<AccessUnit>(synthetic(false, 500)));
        server.publish(7, nullptr);

        r = c.request("PLAY", url, "Session: " + session + "\r\nRange: npt=0.000-\r\n");
        CHECK(r && r->status == 200 && r->header("rtp-info").find("seq=") != std::string::npos);
        CHECK(r && r->header("rtp-info").find("url=" + url + "/trackID=0") == 0);
        CHECK(r && r->header("range") == "npt=0.000-");
        CHECK(plays_now().size() == 1 && plays_now()[0] == std::make_pair(7, true));
        CHECK(keyframes.load() == 1);                               // a new receiver needs an IDR
        r = c.request("PLAY", url, "Session: " + session + "\r\n"); // already playing: just OK
        CHECK(r && r->status == 200 && plays_now().size() == 1);
        r = c.request("SETUP", url + "/trackID=0",
                      "Session: " + session + "\r\nTransport: RTP/AVP/TCP;interleaved=0-1\r\n");
        CHECK(r && r->status == 455); // no transport change mid-play

        // Nothing until an IDR; then the sender report first, then every packet of the frame.
        server.publish(7, std::make_shared<AccessUnit>(synthetic(false, 500)));
        server.publish(8, std::make_shared<AccessUnit>(synthetic(true, 500))); // another stream
        server.publish(7, std::make_shared<AccessUnit>(synthetic(true, 3000)));
        server.publish(7, std::make_shared<AccessUnit>(synthetic(false, 200)));
        CHECK(c.wait_packets(1, 1, 2000));
        CHECK(c.wait_packets(6, 0, 2000)); // SPS, PPS, 3 FU-A fragments, P-frame
        c.settle();
        rtsp_test::Depacketizer depack;
        bool report_first = !c.packets().empty() && c.packets()[0].first == 1;
        for (const auto &[channel, data] : c.packets()) {
            if (channel == 0) {
                depack.push(data);
            } else {
                CHECK(data.size() >= 28 && data[1] == 200);
            }
        }
        CHECK(report_first);
        CHECK(depack.frames.size() == 2);
        CHECK(depack.frames.size() == 2 && depack.frames[0].has(kNalSps) && depack.frames[0].has(kNalPps) &&
              depack.frames[0].has(kNalIdr) && depack.frames[0].nals[2].size() == 3000);
        CHECK(depack.frames.size() == 2 && depack.frames[1].has(kNalSlice));
        CHECK(depack.seq_gaps == 0);
        c.packets().clear();

        // Keep-alives with and without a body; interleaved data from the receiver is skipped.
        r = c.request("GET_PARAMETER", url, "Session: " + session + "\r\n");
        CHECK(r && r->status == 200 && r->session() == session);
        r = c.request("SET_PARAMETER", url, "Content-Type: text/parameters\r\n", "x: 1\r\n");
        CHECK(r && r->status == 200);
        r = c.request("GET_PARAMETER", url, "Session: nope\r\n");
        CHECK(r && r->status == 454);
        c.send_raw(std::string("$\x01\x00\x04", 4) + "abcd");
        r = c.request("OPTIONS", url);
        CHECK(r && r->status == 200);
        r = c.request("ANNOUNCE", url);
        CHECK(r && r->status == 501);

        // PAUSE stops it, PLAY starts it again.
        r = c.request("PAUSE", url, "Session: " + session + "\r\n");
        CHECK(r && r->status == 200 && plays_now().back() == std::make_pair(7, false));
        r = c.request("PAUSE", url, "Session: " + session + "\r\n");
        CHECK(r && r->status == 200);
        r = c.request("PLAY", url, "Session: " + session + "\r\n");
        CHECK(r && r->status == 200 && plays_now().back() == std::make_pair(7, true));

        const auto st = server.stats();
        CHECK(st.connections == 1 && st.playing == 1 && st.frames_sent == 2 && st.bytes_sent > 3000);

        r = c.request("TEARDOWN", url, "Session: " + session + "\r\n");
        CHECK(r && r->status == 200 && plays_now().back() == std::make_pair(7, false));
        r = c.request("PLAY", url, "Session: " + session + "\r\n");
        CHECK(r && r->status == 455);
    }

    // No IDR within a second of PLAY: the receiver asks again, and its RTP-Info is whole however
    // long the URL it set up with.
    {
        rtsp_test::Client c(port);
        const std::string long_url = url + "/" + std::string(400, 't');
        auto r = c.request("SETUP", long_url, "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n");
        CHECK(r && r->status == 200);
        r = c.request("PLAY", url);
        CHECK(r && r->status == 200 && r->header("rtp-info").find("url=" + long_url + ";seq=") == 0 &&
              r->header("rtp-info").find(";rtptime=") != std::string::npos);
        const int asked = keyframes.load();
        server.publish(7, std::make_shared<AccessUnit>(synthetic(false, 100)));
        std::this_thread::sleep_for(100ms);
        CHECK(keyframes.load() == asked); // within the second: no second ask yet
        std::this_thread::sleep_for(1000ms);
        server.publish(7, std::make_shared<AccessUnit>(synthetic(false, 100)));
        std::this_thread::sleep_for(100ms);
        CHECK(keyframes.load() == asked + 1);
        // A burst the server thread may not drain in time: whatever it drops, the receiver
        // resynchronises on the next IDR.
        for (int i = 0; i < 300; ++i) {
            server.publish(7, std::make_shared<AccessUnit>(synthetic(i == 0, 50)));
        }
        server.publish(7, std::make_shared<AccessUnit>(synthetic(true, 60)));
        CHECK(c.wait_packets(3, 0, 3000));
    }

    // A receiver that stops reading: its backlog fills, frames are dropped until the next IDR,
    // and an IDR is asked for; the connection itself survives.
    {
        rtsp_test::Client c(port, "127.0.0.1", AF_INET, 4096);
        c.request("SETUP", url, "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n");
        auto r = c.request("PLAY", url);
        CHECK(r && r->status == 200);
        std::this_thread::sleep_for(1100ms); // past the once-a-second limit on asking for IDRs
        const int asked = keyframes.load();
        server.publish(7, std::make_shared<AccessUnit>(synthetic(true, 1000)));
        for (int i = 0; i < 60; ++i) {
            server.publish(7, std::make_shared<AccessUnit>(synthetic(false, 150000)));
            std::this_thread::sleep_for(2ms);
        }
        std::this_thread::sleep_for(300ms);
        CHECK(server.stats().frames_dropped > 0);
        CHECK(keyframes.load() > asked);
        // Drain, then the next IDR gets through again (and the P-frame before it does not).
        c.settle(300);
        c.packets().clear();
        server.publish(7, std::make_shared<AccessUnit>(synthetic(false, 100)));
        server.publish(7, std::make_shared<AccessUnit>(synthetic(true, 100)));
        CHECK(c.wait_packets(3, 0, 3000));
        c.settle();
        rtsp_test::Depacketizer depack;
        for (const auto &[channel, data] : c.packets()) {
            if (channel == 0) {
                depack.push(data);
            }
        }
        CHECK(depack.frames.size() == 1 && depack.frames[0].has(kNalIdr));
    } // closing the connection ends its session

    // UDP: RTP to the receiver's client_port, RTCP to the port after it, from the server pair.
    {
        UdpPair udp;
        CHECK(udp.port > 0);
        rtsp_test::Client c(port);
        auto r = c.request("SETUP", url + "/trackID=0",
                           "Transport: RTP/AVP;unicast;client_port=" + std::to_string(udp.port) + "-" +
                               std::to_string(udp.port + 1) + "\r\n");
        CHECK(r && r->status == 200);
        CHECK(r && r->header("transport")
                           .find("server_port=" + std::to_string(server.udp_rtp_port()) + "-" +
                                 std::to_string(server.udp_rtp_port() + 1)) != std::string::npos);
        r = c.request("PLAY", url, "Session: " + (r ? r->session() : std::string{}) + "\r\n");
        CHECK(r && r->status == 200);
        server.publish(7, std::make_shared<AccessUnit>(synthetic(true, 2000)));
        const auto report = udp.recv(udp.rtcp, 2000);
        CHECK(report && report->size() >= 28 && (*report)[1] == 200);
        rtsp_test::Depacketizer depack;
        while (depack.frames.empty()) {
            const auto pkt = udp.recv(udp.rtp, 2000);
            if (!pkt) {
                break;
            }
            depack.push(*pkt);
        }
        CHECK(depack.frames.size() == 1 && depack.frames[0].has(kNalIdr));
        // A receiver report to the server pair is taken as a sign of life.
        sockaddr_in to{};
        to.sin_family = AF_INET;
        to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        to.sin_port = htons(static_cast<uint16_t>(server.udp_rtp_port() + 1));
        const uint8_t rr[8] = {0x80, 201, 0, 1, 0, 0, 0, 1};
        ::sendto(udp.rtcp, rr, sizeof(rr), 0, reinterpret_cast<sockaddr *>(&to), sizeof(to));
        to.sin_port = htons(static_cast<uint16_t>(server.udp_rtp_port()));
        ::sendto(udp.rtp, rr, sizeof(rr), 0, reinterpret_cast<sockaddr *>(&to), sizeof(to));
        std::this_thread::sleep_for(200ms);
        // A UDP session lives on its RTSP connection: it outlasts 1.5 timeouts only if heard from.
        CHECK(!c.closed_by_peer(1000));
        r = c.request("GET_PARAMETER", url);
        CHECK(r && r->status == 200);
        CHECK(c.closed_by_peer(3000)); // and nothing more: timed out
        std::this_thread::sleep_for(100ms);
        CHECK(plays_now().back() == std::make_pair(7, false));
    }

    // Protocol errors: a request line that is not RTSP, a bad Content-Length, a header that never
    // ends. Each gets a 400 and a closed connection.
    {
        rtsp_test::Client c(port);
        c.send_raw("GET / HTTP/1.1\r\nHost: x\r\n\r\n");
        auto r = c.response();
        CHECK(r && r->status == 400);
        CHECK(c.closed_by_peer(1000));
    }
    {
        rtsp_test::Client c(port);
        c.send_raw("OPTIONS * RTSP/1.0\r\nCSeq: 9\r\nContent-Length: lots\r\n\r\n");
        auto r = c.response();
        CHECK(r && r->status == 400 && r->header("cseq") == "9");
        CHECK(c.closed_by_peer(1000));
    }
    {
        rtsp_test::Client c(port);
        c.send_raw("OPTIONS * RTSP/1.0\r\nX: " + std::string(20000, 'a'));
        auto r = c.response();
        CHECK(r && r->status == 400);
        CHECK(c.closed_by_peer(1000));
    }
    {
        // Silence on a connection with no session closes it after 1.5 timeouts.
        rtsp_test::Client c(port);
        CHECK(c.closed_by_peer(3000));
    }

    // A peer that pipelines requests and never reads the answers is dropped once its unread
    // responses pass the bound, instead of growing them without limit.
    {
        RtspServer::Config tight = cfg;
        tight.max_backlog_bytes = 1000; // response bound: 2 * 1000 + 64 KiB
        RtspServer bounded(tight, hooks);
        rtsp_test::Client c(bounded.port(), "127.0.0.1", AF_INET, 4096);
        std::string burst;
        for (int i = 0; i < 600; ++i) {
            burst += "OPTIONS * RTSP/1.0\r\n\r\n";
        }
        bool closed = false;
        for (int round = 0; round < 400 && !closed; ++round) {
            closed = ::send(c.fd(), burst.data(), burst.size(), MSG_NOSIGNAL | MSG_DONTWAIT) < 0 &&
                     errno != EAGAIN && errno != EWOULDBLOCK;
            std::this_thread::sleep_for(3ms);
        }
        CHECK(closed || c.closed_by_peer(3000));
    }

    // At most max_connections at once: the one past it is closed straight away.
    {
        RtspServer::Config one = cfg;
        one.max_connections = 1;
        RtspServer small(one, hooks);
        rtsp_test::Client a(small.port());
        auto r = a.request("OPTIONS", "*");
        CHECK(r && r->status == 200);
        rtsp_test::Client b(small.port());
        CHECK(b.closed_by_peer(1000));
    }

    // IPv6, where the box has it.
    try {
        RtspServer::Config v6 = cfg;
        v6.bind = "::1";
        RtspServer server6(v6, hooks);
        rtsp_test::Client c(server6.port(), "::1", AF_INET6);
        auto r = c.request("DESCRIBE", "rtsp://[::1]/instances/7/main.264");
        CHECK(r && r->status == 200 && r->body.find("IN IP6 ::1") != std::string::npos);
    } catch (const std::runtime_error &e) {
        std::printf("test_video: no IPv6 loopback here (%s), skipping that case\n", e.what());
    }
}

void test_video_service() {
    std::atomic<uint32_t> vehicle_at_3{30};
    std::mutex mutex;
    std::vector<uint32_t> wanted;
    VideoService::Config cfg;
    cfg.width = 64;
    cfg.height = 48;
    cfg.max_fps = 10.0;
    cfg.defaults.fps = 25.0; // clamped to max_fps
    cfg.threads = 1;
    cfg.background_priority = false;
    cfg.rtsp.bind = "127.0.0.1";
    cfg.rtsp.port = 0;
    cfg.linger_s = 0.3;
    cfg.refresh_s = 0.05;
    VideoService::Hooks hooks;
    hooks.resolve = [&](int instance) -> uint32_t { return instance == 3 ? vehicle_at_3.load() : 0; };
    hooks.want_frames = [&](uint32_t id) {
        std::lock_guard<std::mutex> lock(mutex);
        wanted.push_back(id);
    };
    VideoService video(cfg, hooks);
    const int port = video.port();
    CHECK(port > 0);
    const std::string url = "rtsp://127.0.0.1:" + std::to_string(port) + "/instances/3/main.264";

    // Settings: defaults, clamping, and settings belonging to the vehicle they were set for.
    auto st = video.encoder(3, 30);
    CHECK(st.settings.bitrate_kbps == 1200 && st.settings.fps == 10.0 &&
          st.settings.keyframe_interval_s == 1.0);
    CHECK(st.width == 64 && st.height == 48 && st.max_fps == 10.0 && st.clients == 0);
    EncoderPatch patch;
    patch.bitrate_kbps = 99999.0;
    patch.fps = 0.2;
    patch.keyframe_interval_s = 0.01;
    st = video.encoder(3, 30, &patch);
    CHECK(st.settings.bitrate_kbps == kMaxBitrateKbps && st.settings.fps == kMinFps &&
          st.settings.keyframe_interval_s == kMinKeyframeIntervalS);
    patch = {};
    patch.bitrate_kbps = 1.0;
    patch.fps = 99.0;
    patch.keyframe_interval_s = 1e9;
    st = video.encoder(3, 30, &patch);
    CHECK(st.settings.bitrate_kbps == kMinBitrateKbps && st.settings.fps == 10.0 &&
          st.settings.keyframe_interval_s == kMaxKeyframeIntervalS);
    patch = {};
    patch.bitrate_kbps = std::nan("");
    patch.fps = 7.5;
    st = video.encoder(3, 30, &patch);
    CHECK(st.settings.bitrate_kbps == kMinBitrateKbps && st.settings.fps == 7.5); // NaN changes nothing
    st = video.encoder(3, 31); // another vehicle at the instance starts from defaults
    CHECK(st.settings.bitrate_kbps == 1200 && st.settings.fps == 10.0);
    st = video.encoder(3, 30);
    CHECK(st.settings.bitrate_kbps == 1200 && st.settings.keyframe_interval_s == 1.0);
    CHECK(video.request_keyframe(3, 30) == 0);

    rtsp_test::Client c(port);
    auto r = c.request("DESCRIBE", "rtsp://127.0.0.1/instances/9/main.264");
    CHECK(r && r->status == 404);
    r = c.request("DESCRIBE", url);
    CHECK(r && r->status == 200 && r->body.find("profile-level-id=42e01f") != std::string::npos);
    CHECK(r && r->body.find("a=x-dimensions:64,48") != std::string::npos);
    r = c.request("DESCRIBE", url); // the cached parameter sets
    CHECK(r && r->status == 200);
    r = c.request("SETUP", url + "/trackID=0", "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n");
    CHECK(r && r->status == 200);
    r = c.request("PLAY", url, "Session: " + (r ? r->session() : std::string{}) + "\r\n");
    CHECK(r && r->status == 200);
    std::this_thread::sleep_for(150ms);
    {
        std::lock_guard<std::mutex> lock(mutex);
        CHECK(wanted.size() >= 2 && wanted.front() == 30); // at PLAY, then on every refresh
    }
    CHECK(video.stats().streams == 1 && video.stats().clients == 1);
    CHECK(video.encoder(3, 30).clients == 1);

    // Pictures at the render rate: every one encoded, an IDR first and every second after.
    video.on_frame(99, std::make_shared<Image>(card(64, 48, 0)), 0.0, std::chrono::steady_clock::now());
    video.on_frame(30, nullptr, 0.0, std::chrono::steady_clock::now());
    video.on_frame(30, std::make_shared<Image>(), 0.0, std::chrono::steady_clock::now());
    const auto feed = [&](uint32_t id, int frames, double fps, int t0) {
        const auto start = std::chrono::steady_clock::now();
        for (int i = 0; i < frames; ++i) {
            const auto at = start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                        std::chrono::duration<double>(i / fps));
            std::this_thread::sleep_until(at);
            video.on_frame(id, std::make_shared<Image>(card(64, 48, t0 + i)), (t0 + i) / fps, at);
        }
        std::this_thread::sleep_for(100ms);
    };
    feed(30, 25, 10.0, 0);
    CHECK(c.wait_packets(25, 0, 3000));
    c.settle();
    rtsp_test::Depacketizer depack;
    for (const auto &[channel, data] : c.packets()) {
        if (channel == 0) {
            depack.push(data);
        }
    }
    c.packets().clear();
    CHECK(depack.frames.size() == 25);
    std::vector<size_t> idrs;
    for (size_t i = 0; i < depack.frames.size(); ++i) {
        if (depack.frames[i].has(kNalIdr)) {
            idrs.push_back(i);
            CHECK(depack.frames[i].has(kNalSps) && depack.frames[i].has(kNalPps));
        }
    }
    CHECK(idrs.size() == 3 && idrs[0] == 0 && idrs[1] == 10 && idrs[2] == 20);
    // RTP timestamps follow the capture clock: 100 ms apart at 90 kHz.
    if (depack.frames.size() == 25) {
        const uint32_t step = depack.frames[1].timestamp - depack.frames[0].timestamp;
        CHECK(step > 8500 && step < 9500);
    }
    auto before = video.stats();
    CHECK(before.frames_encoded == 25 && before.keyframes == 3 && before.bytes_encoded > 0);

    // A forced keyframe, a bitrate change (no IDR), then half the rate: every other picture.
    CHECK(video.request_keyframe(3, 30) == 1);
    patch = {};
    patch.bitrate_kbps = 300.0;
    video.encoder(3, 30, &patch);
    feed(30, 3, 10.0, 100);
    CHECK(c.wait_packets(3, 0, 3000));
    c.settle();
    depack = {};
    for (const auto &[channel, data] : c.packets()) {
        if (channel == 0) {
            depack.push(data);
        }
    }
    c.packets().clear();
    CHECK(depack.frames.size() == 3 && depack.frames[0].has(kNalIdr) && !depack.frames[1].has(kNalIdr));
    patch = {};
    patch.fps = 5.0;
    patch.keyframe_interval_s = 2.0;
    video.encoder(3, 30, &patch);
    before = video.stats();
    feed(30, 20, 10.0, 200);
    auto after = video.stats();
    CHECK(after.frames_encoded - before.frames_encoded == 10);
    c.wait_packets(10, 0, 3000);
    c.settle();
    depack = {};
    for (const auto &[channel, data] : c.packets()) {
        if (channel == 0) {
            depack.push(data);
        }
    }
    c.packets().clear();
    CHECK(depack.frames.size() == 10 && depack.frames[0].has(kNalIdr)); // new rate, new encoder
    if (depack.frames.size() == 10) {
        const uint32_t step = depack.frames[1].timestamp - depack.frames[0].timestamp;
        CHECK(step > 17000 && step < 19000);
    }

    // Pictures faster than the encoder takes them: a waiting picture is replaced, and counted.
    patch = {};
    patch.fps = 10.0;
    video.encoder(3, 30, &patch);
    before = video.stats();
    for (int i = 0; i < 40; ++i) {
        video.on_frame(30, std::make_shared<Image>(card(64, 48, i)), 0.0, std::chrono::steady_clock::now());
    }
    std::this_thread::sleep_for(200ms);
    after = video.stats();
    CHECK(after.frames_dropped > before.frames_dropped);
    CHECK(after.frames_encoded > before.frames_encoded);

    // Another vehicle takes the instance: its pictures now, defaults again, and an IDR.
    vehicle_at_3 = 32;
    std::this_thread::sleep_for(200ms);
    CHECK(video.encoder(3, 32).settings.fps == 10.0 &&
          video.encoder(3, 32).settings.keyframe_interval_s == 1.0);
    c.packets().clear();
    before = video.stats();
    video.on_frame(30, std::make_shared<Image>(card(64, 48, 0)), 0.0, std::chrono::steady_clock::now());
    std::this_thread::sleep_for(50ms);
    CHECK(video.stats().frames_encoded == before.frames_encoded);
    feed(32, 2, 10.0, 0);
    CHECK(c.wait_packets(2, 0, 3000));
    c.settle();
    depack = {};
    for (const auto &[channel, data] : c.packets()) {
        if (channel == 0) {
            depack.push(data);
        }
    }
    CHECK(!depack.frames.empty() && depack.frames[0].has(kNalIdr));
    vehicle_at_3 = 0; // despawned: the stream stays open with nothing to show
    std::this_thread::sleep_for(150ms);
    vehicle_at_3 = 32;

    // The last receiver goes: nothing is encoded, and after the linger the encoder is freed.
    CHECK(video.stats().encoders == 1);
    c.close();
    std::this_thread::sleep_for(150ms);
    CHECK(video.stats().clients == 0 && video.stats().streams == 0);
    before = video.stats();
    video.on_frame(32, std::make_shared<Image>(card(64, 48, 0)), 0.0, std::chrono::steady_clock::now());
    std::this_thread::sleep_for(500ms);
    CHECK(video.stats().frames_encoded == before.frames_encoded);
    CHECK(video.stats().encoders == 0);
    {
        rtsp_test::Client again(port);
        r = again.request("DESCRIBE", url);
        CHECK(r && r->status == 200);
    }
    const auto final_stats = video.stats();
    CHECK(final_stats.threads == 1 && final_stats.port == port);
    video.stop();
    video.stop(); // idempotent
    video.on_frame(32, std::make_shared<Image>(card(64, 48, 0)), 0.0, std::chrono::steady_clock::now());
    CHECK(video.stats().port == 0);
}

void test_render_video_demand() {
    using skysim::render::RenderService;
    std::mutex mutex;
    std::vector<std::pair<uint32_t, std::shared_ptr<const Image>>> pictures;
    RenderService::Config cfg;
    cfg.camera.width = 64;
    cfg.camera.height = 36;
    cfg.scene = skysim::render::load_scene("", 1);
    cfg.fps = 50.0;
    cfg.threads = 1;
    cfg.demand_window_s = 0.3;
    cfg.background_priority = false;
    cfg.video_sink = [&](uint32_t id, std::shared_ptr<const Image> image, double,
                         std::chrono::steady_clock::time_point) {
        std::lock_guard<std::mutex> lock(mutex);
        pictures.emplace_back(id, std::move(image));
    };
    RenderService service(cfg);
    const auto count = [&] {
        std::lock_guard<std::mutex> lock(mutex);
        return pictures.size();
    };
    CHECK(!service.want_video(7)); // not in the fleet: nothing recorded
    RenderService::Pose pose;
    pose.vehicle_id = 7;
    pose.position_ned = {0.0, 0.0, -20.0};
    service.publish_poses({pose});
    CHECK(service.want_video(7));
    std::this_thread::sleep_for(200ms);
    CHECK(count() >= 3);
    // Video alone makes no JPEG, and a parked vehicle is drawn once and re-sent.
    auto st = service.stats();
    CHECK(st.frames_drawn == 1 && st.frames_reused >= 2);
    CHECK(service.frame(7).jpeg.empty());
    {
        std::lock_guard<std::mutex> lock(mutex);
        CHECK(pictures.front().first == 7 && pictures.front().second->width == 64);
        CHECK(pictures.back().second == pictures.front().second); // the same picture, shared
    }
    // A JPEG viewer joining a picture already drawn gets a JPEG made from it, not a redraw.
    const auto jpeg = service.frame(7, 0, 2000ms);
    CHECK(!jpeg.jpeg.empty() && jpeg.captured_unix_us > 0);
    CHECK(service.stats().frames_drawn == 1);
    // JPEG demand lapses while video carries on: pictures keep coming, the JPEG is dropped.
    const size_t both = count();
    for (int i = 0; i < 10; ++i) {
        service.want_video(7);
        std::this_thread::sleep_for(60ms);
    }
    CHECK(count() > both);
    CHECK(service.stats().frames_drawn == 1); // still the one picture
    // And the other way round: video lapses, the JPEG viewer keeps the vehicle drawn.
    for (int i = 0; i < 10; ++i) {
        service.frame(7, 0, 0ms);
        std::this_thread::sleep_for(60ms);
    }
    const size_t lapsed = count();
    for (int i = 0; i < 5; ++i) {
        CHECK(!service.frame(7, 0, 100ms).jpeg.empty());
    }
    CHECK(count() == lapsed);
}

} // namespace

int main() {
    test_bitstream_helpers();
    test_encoder();
    test_rtp();
    test_parsers();
    test_rtsp_server();
    test_video_service();
    test_render_video_demand();
    if (g_failures != 0) {
        std::printf("test_video: %d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("test_video: OK\n");
    return 0;
}
