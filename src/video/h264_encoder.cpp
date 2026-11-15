#include "video/h264_encoder.h"

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>

#include <x264.h>

namespace skysim::video {
namespace {

// x264's warnings and errors, one "skysim: x264:" line each; its info chatter is not wanted.
void log_x264(void *, int level, const char *fmt, va_list args) {
    if (level > X264_LOG_WARNING) {
        return;
    }
    char line[512];
    std::vsnprintf(line, sizeof(line), fmt, args);
    std::fprintf(stderr, "skysim: x264: %s", line);
}

// Frame rate as the rational x264 wants: whole rates exactly, others to a thousandth.
void fps_rational(double fps, uint32_t &num, uint32_t &den) {
    const double rounded = std::round(fps);
    if (std::fabs(fps - rounded) < 1e-9) {
        num = static_cast<uint32_t>(rounded);
        den = 1;
    } else {
        num = static_cast<uint32_t>(std::lround(fps * 1000.0));
        den = 1000;
    }
}

// x264 with b_annexb off prefixes each NAL with its 4-byte big-endian size instead of a start
// code; this strips it, so the NAL starts at its header byte.
std::vector<uint8_t> nal_bytes(const x264_nal_t &nal) {
    if (nal.i_payload <= 4) {
        return {};
    }
    return std::vector<uint8_t>(nal.p_payload + 4, nal.p_payload + nal.i_payload);
}

} // namespace

struct H264Encoder::Impl {
    x264_t *x264{nullptr};
    x264_picture_t picture{};
    std::vector<uint8_t> i420;

    ~Impl() {
        if (x264 != nullptr) {
            x264_encoder_close(x264);
        }
    }
};

H264Encoder::H264Encoder() : impl_(std::make_unique<Impl>()) {}
H264Encoder::~H264Encoder() = default;

std::unique_ptr<H264Encoder> H264Encoder::create(const EncoderConfig &cfg, std::string *error) {
    const auto fail = [error](const char *why) -> std::unique_ptr<H264Encoder> {
        if (error != nullptr) {
            *error = why;
        }
        return nullptr;
    };
    const int width = cfg.width & ~1;
    const int height = cfg.height & ~1;
    if (width < 16 || height < 16) {
        return fail("frame must be at least 16x16 pixels");
    }
    if (!(cfg.fps > 0.0) || cfg.fps > 240.0 || cfg.bitrate_kbps <= 0) {
        return fail("fps must be in (0, 240] and bitrate positive");
    }

    x264_param_t p;
    if (x264_param_default_preset(&p, cfg.preset.c_str(), "zerolatency") != 0) {
        return fail("unknown x264 preset");
    }
    p.pf_log = &log_x264;
    p.i_log_level = X264_LOG_WARNING;
    p.i_csp = X264_CSP_I420;
    p.i_width = width;
    p.i_height = height;
    fps_rational(cfg.fps, p.i_fps_num, p.i_fps_den);
    p.b_vfr_input = 0; // see encode(): VFR rate control costs a frame of latency
    // One thread, no slices: a picture this size encodes in a millisecond or two, and frame
    // threads would add a frame of latency each.
    p.i_threads = 1;
    p.b_sliced_threads = 0;
    // Keyframes are ours to place (wall-clock interval, forced IDRs), never x264's.
    p.i_keyint_max = X264_KEYINT_MAX_INFINITE;
    p.i_scenecut_threshold = 0;
    p.b_intra_refresh = 0;
    p.i_frame_reference = 1;
    p.i_bframe = 0;
    p.b_repeat_headers = 1;
    p.b_annexb = 0;
    p.b_aud = 0;
    p.i_level_idc = level_for(width, height, cfg.fps);
    // What rgb_to_i420 produces: BT.601 matrix, limited range. Signalled so a browser does not
    // guess BT.709 for it.
    p.vui.b_fullrange = 0;
    p.vui.i_colorprim = 6;
    p.vui.i_transfer = 6;
    p.vui.i_colmatrix = 6;
    p.rc.i_rc_method = X264_RC_ABR;
    p.rc.i_bitrate = cfg.bitrate_kbps;
    p.rc.i_vbv_max_bitrate = cfg.bitrate_kbps;
    p.rc.i_vbv_buffer_size = cfg.bitrate_kbps; // one second
    if (x264_param_apply_profile(&p, "baseline") != 0) {
        return fail("x264 refused the baseline profile");
    }

    std::unique_ptr<H264Encoder> enc(new H264Encoder());
    enc->cfg_ = cfg;
    enc->cfg_.width = width; // what is encoded, as the field's comment says
    enc->cfg_.height = height;
    enc->width_ = width;
    enc->height_ = height;
    enc->impl_->x264 = x264_encoder_open(&p);
    if (enc->impl_->x264 == nullptr) {
        return fail("x264_encoder_open failed");
    }

    x264_nal_t *nals = nullptr;
    int count = 0;
    if (x264_encoder_headers(enc->impl_->x264, &nals, &count) < 0) {
        return fail("x264_encoder_headers failed");
    }
    for (int i = 0; i < count; ++i) {
        if (nals[i].i_type == NAL_SPS) {
            enc->sps_ = nal_bytes(nals[i]);
        } else if (nals[i].i_type == NAL_PPS) {
            enc->pps_ = nal_bytes(nals[i]);
        }
    }
    // constraint_set2_flag says the stream also obeys the Extended profile's constraints. A
    // Constrained Baseline stream does whenever direct_8x8_inference_flag is 1 and it is
    // progressive (A.2.3), which x264 always writes for Baseline — so the SPS can say "42e0",
    // the profile-level-id WebRTC offers, instead of x264's "42c0". Raised only after checking.
    const SpsInfo info = parse_sps(enc->sps_);
    if (!info.ok || enc->pps_.empty()) {
        return fail("x264 produced no usable SPS/PPS");
    }
    enc->set2_ = info.profile_idc == 66 && info.direct_8x8_inference && info.frame_mbs_only;
    enc->fix_sps(enc->sps_);

    x264_picture_init(&enc->impl_->picture);
    auto &pic = enc->impl_->picture;
    const size_t luma = static_cast<size_t>(width) * height;
    enc->impl_->i420.resize(luma + luma / 2);
    pic.img.i_csp = X264_CSP_I420;
    pic.img.i_plane = 3;
    pic.img.i_stride[0] = width;
    pic.img.i_stride[1] = width / 2;
    pic.img.i_stride[2] = width / 2;
    pic.img.plane[0] = enc->impl_->i420.data();
    pic.img.plane[1] = enc->impl_->i420.data() + luma;
    pic.img.plane[2] = enc->impl_->i420.data() + luma + luma / 4;
    return enc;
}

void H264Encoder::fix_sps(std::vector<uint8_t> &sps) const {
    // Byte 2 is the constraint flags (after the NAL header and profile_idc); emulation
    // prevention cannot touch it, as neither 0xc0 nor 0xe0 is a 00/01/02/03 byte.
    if (set2_ && sps.size() > 3 && nal_type(sps) == kNalSps) {
        sps[2] |= 0x20;
    }
}

bool H264Encoder::encode(const render::Image &image, bool force_idr, AccessUnit &out) {
    out.nals.clear();
    out.idr = false;
    if (image.width < width_ || image.height < height_ ||
        image.rgb.size() < static_cast<size_t>(image.width) * image.height * 3) {
        return false;
    }
    auto &pic = impl_->picture;
    rgb_to_i420(image, width_, height_, pic.img.plane[0], pic.img.plane[1], pic.img.plane[2]);
    pic.i_type = force_idr ? X264_TYPE_IDR : X264_TYPE_AUTO;
    pic.i_pts = static_cast<int64_t>(frames_);
    x264_picture_t encoded;
    x264_nal_t *nals = nullptr;
    int count = 0;
    const int size = x264_encoder_encode(impl_->x264, &nals, &count, &pic, &encoded);
    if (size < 0) {
        return false;
    }
    ++frames_;
    bool has_sps = false;
    bool has_pps = false;
    for (int i = 0; i < count; ++i) {
        std::vector<uint8_t> nal = nal_bytes(nals[i]);
        if (nal.empty()) {
            continue;
        }
        const uint8_t type = nal_type(nal);
        has_sps = has_sps || type == kNalSps;
        has_pps = has_pps || type == kNalPps;
        out.idr = out.idr || type == kNalIdr;
        fix_sps(nal);
        out.nals.push_back(std::move(nal));
    }
    // b_repeat_headers puts them there; this only makes "every IDR carries SPS and PPS" a
    // property of this class rather than of an x264 option.
    if (out.idr && !(has_sps && has_pps)) {
        out.nals.insert(out.nals.begin(), {sps_, pps_});
    }
    return !out.nals.empty();
}

bool H264Encoder::set_bitrate(int kbps) {
    if (kbps <= 0) {
        return false;
    }
    x264_param_t p;
    x264_encoder_parameters(impl_->x264, &p);
    p.rc.i_bitrate = kbps;
    p.rc.i_vbv_max_bitrate = kbps;
    p.rc.i_vbv_buffer_size = kbps;
    if (x264_encoder_reconfig(impl_->x264, &p) != 0) {
        return false;
    }
    cfg_.bitrate_kbps = kbps;
    return true;
}

void rgb_to_i420(const render::Image &rgb, int width, int height, uint8_t *y, uint8_t *u, uint8_t *v) {
    const size_t stride = static_cast<size_t>(rgb.width) * 3;
    const uint8_t *src = rgb.rgb.data();
    for (int row = 0; row < height; row += 2) {
        const uint8_t *top = src + static_cast<size_t>(row) * stride;
        const uint8_t *bottom = top + stride;
        uint8_t *y_top = y + static_cast<size_t>(row) * width;
        uint8_t *y_bottom = y_top + width;
        uint8_t *u_row = u + static_cast<size_t>(row / 2) * (width / 2);
        uint8_t *v_row = v + static_cast<size_t>(row / 2) * (width / 2);
        for (int col = 0; col < width; col += 2) {
            int r_sum = 0;
            int g_sum = 0;
            int b_sum = 0;
            const uint8_t *px[4] = {top + col * 3, top + col * 3 + 3, bottom + col * 3, bottom + col * 3 + 3};
            uint8_t *out[4] = {y_top + col, y_top + col + 1, y_bottom + col, y_bottom + col + 1};
            for (int k = 0; k < 4; ++k) {
                const int r = px[k][0];
                const int g = px[k][1];
                const int b = px[k][2];
                *out[k] = static_cast<uint8_t>(((66 * r + 129 * g + 25 * b + 128) >> 8) + 16);
                r_sum += r;
                g_sum += g;
                b_sum += b;
            }
            const int r = (r_sum + 2) >> 2;
            const int g = (g_sum + 2) >> 2;
            const int b = (b_sum + 2) >> 2;
            u_row[col / 2] = static_cast<uint8_t>(((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128);
            v_row[col / 2] = static_cast<uint8_t>(((112 * r - 94 * g - 18 * b + 128) >> 8) + 128);
        }
    }
}

} // namespace skysim::video
