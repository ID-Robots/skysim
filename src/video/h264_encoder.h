#pragma once
// A libx264 encoder configured the way the real camera on Observer 1 streams and the way
// browsers and Janus H.264 rooms take a stream without transcoding: Constrained Baseline
// (profile-level-id 42e01f at the default sizes), no B-frames, zerolatency (no lookahead, one
// frame in → one frame out), SPS and PPS in front of every IDR, ABR with a one-second VBV.
//
// IDRs are the caller's decision (encode()'s force_idr): x264's own GOP is set to infinite and
// its scene-cut detection off, so the keyframe interval is a wall-clock setting the caller
// enforces and can change between frames without restarting the stream. The bitrate can be
// changed in place too (set_bitrate); a new frame rate needs a new encoder, because it is in
// the SPS (VUI timing) and in the rate control's bits-per-frame.
//
// Not thread-safe: one encoder belongs to one stream, used by one thread at a time.
#include "render/image.h"
#include "video/h264.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace skysim::video {

struct EncoderConfig {
    int width{256}; // rounded down to even (4:2:0); the picture's top-left region is encoded
    int height{144};
    double fps{10.0};
    int bitrate_kbps{1200};
    // x264 speed preset. veryfast is what live streaming defaults to; at a pilot camera's
    // sizes it costs a millisecond or two per frame, next to tens for drawing the picture.
    std::string preset{"veryfast"};
};

class H264Encoder {
  public:
    // Null, with the reason in *error, when x264 refuses the parameters (a frame under 16x16
    // pixels, a non-positive rate, an unknown preset).
    static std::unique_ptr<H264Encoder> create(const EncoderConfig &cfg, std::string *error = nullptr);
    ~H264Encoder();
    H264Encoder(const H264Encoder &) = delete;
    H264Encoder &operator=(const H264Encoder &) = delete;

    // Encodes one RGB picture of at least the configured size (larger ones are cropped to the
    // top-left). Fills `out.nals` and `out.idr` and leaves its timestamps to the caller. False
    // when the picture is too small or x264 fails.
    //
    // Rate control budgets bitrate / fps per picture, so pictures arriving slower than the
    // stream's fps (a renderer that cannot keep up) use proportionally less of the bitrate.
    // Budgeting by real timestamps instead (x264's VFR input) needs each picture's successor
    // to know its duration: a frame of latency, which zerolatency exists to avoid.
    bool encode(const render::Image &image, bool force_idr, AccessUnit &out);

    // ABR target and VBV ceiling, applied from the next frame without an IDR.
    bool set_bitrate(int kbps);

    const EncoderConfig &config() const { return cfg_; }
    int width() const { return width_; }
    int height() const { return height_; }
    // The parameter sets every IDR carries, also what an SDP's sprop-parameter-sets quotes.
    const std::vector<uint8_t> &sps() const { return sps_; }
    const std::vector<uint8_t> &pps() const { return pps_; }
    uint64_t frames() const { return frames_; }

  private:
    struct Impl;
    H264Encoder();
    void fix_sps(std::vector<uint8_t> &sps) const;

    EncoderConfig cfg_;
    int width_{0};
    int height_{0};
    bool set2_{false}; // constraint_set2_flag may be raised (see fix_sps)
    std::vector<uint8_t> sps_;
    std::vector<uint8_t> pps_;
    uint64_t frames_{0};
    std::unique_ptr<Impl> impl_;
};

// RGB8 → I420 (BT.601, limited range: what the stream's VUI signals), over the top-left
// `width` x `height` (both even) of `rgb`. Chroma is the mean of each 2x2 block.
void rgb_to_i420(const render::Image &rgb, int width, int height, uint8_t *y, uint8_t *u, uint8_t *v);

} // namespace skysim::video
