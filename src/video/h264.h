#pragma once
// H.264 bitstream helpers shared by the encoder, the RTP packetizer and the RTSP server: what an
// encoded frame is, how to split an Annex B byte stream into NAL units, and the few SPS fields
// an SDP offer has to quote. No codec library here — this layer only moves bytes around.
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace skysim::video {

// NAL unit types this code looks at (ITU-T H.264 Table 7-1).
inline constexpr uint8_t kNalSlice = 1;
inline constexpr uint8_t kNalIdr = 5;
inline constexpr uint8_t kNalSei = 6;
inline constexpr uint8_t kNalSps = 7;
inline constexpr uint8_t kNalPps = 8;
// RFC 6184 payload types carried in the NAL header's type field.
inline constexpr uint8_t kNalStapA = 24;
inline constexpr uint8_t kNalFuA = 28;

inline uint8_t nal_type(const std::vector<uint8_t> &nal) { return nal.empty() ? 0 : (nal[0] & 0x1F); }

// One encoded picture: its NAL units in decoding order, each without a start code. An IDR
// access unit always carries its SPS and PPS first (the stream repeats them on every IDR, so a
// receiver that joins late, or lost the SDP, can start decoding at the next one).
struct AccessUnit {
    std::vector<std::vector<uint8_t>> nals;
    bool idr{false};
    // When the picture was drawn (steady clock). RTP timestamps are derived from this, so the
    // spacing a receiver sees is the spacing the frames were drawn at.
    std::chrono::steady_clock::time_point captured{};
    // Sim time of the pose it was drawn from, for logs and tests.
    double sim_time_s{0.0};

    size_t bytes() const {
        size_t n = 0;
        for (const auto &nal : nals) {
            n += nal.size();
        }
        return n;
    }
};

// Splits an Annex B byte stream (00 00 01 or 00 00 00 01 start codes) into NAL units. Bytes
// before the first start code are dropped; trailing zero bytes of a NAL (trailing_zero_8bits)
// are dropped with the next start code.
std::vector<std::vector<uint8_t>> split_annexb(const uint8_t *data, size_t size);

// profile_idc, the constraint flags byte and level_idc of an SPS NAL, as RFC 6184's
// profile-level-id ("42e01f"). Empty if `sps` is not an SPS or is too short.
std::string profile_level_id(const std::vector<uint8_t> &sps);

// RFC 4648 base64 with padding, for sprop-parameter-sets.
std::string base64(const std::vector<uint8_t> &bytes);

// What an SPS says about the stream, for tests and for the encoder's own self-check. Parses
// the Baseline/Main/Extended SPS layout (profile_idc without the High-profile chroma fields)
// up to frame_cropping, plus the VUI timing fields when present. `ok` is false for anything
// else or a truncated NAL.
struct SpsInfo {
    bool ok{false};
    int profile_idc{0};
    int constraint_flags{0}; // constraint_set0..5 as the top six bits of the byte
    int level_idc{0};
    int width{0};  // in pixels, after cropping
    int height{0}; // in pixels, after cropping
    bool frame_mbs_only{false};
    bool direct_8x8_inference{false};
    bool timing_info{false};
    uint32_t num_units_in_tick{0};
    uint32_t time_scale{0};
    bool fixed_frame_rate{false};
};
SpsInfo parse_sps(const std::vector<uint8_t> &sps);

// The level (level_idc, e.g. 31 for 3.1) the stream is signalled at: the lowest of 3.1, 4.0,
// 4.1, 4.2, 5.0, 5.1, 5.2 whose frame-size and macroblock-rate limits (Table A-1) hold
// `width` x `height` at `fps`. 3.1 is the floor because it is the level of "42e01f", the
// profile-level-id browsers and Janus offer for H.264, and a lower level buys a receiver
// nothing. 52 when nothing fits.
int level_for(int width, int height, double fps);

} // namespace skysim::video
