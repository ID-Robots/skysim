#include "video/h264.h"

#include <cmath>

namespace skysim::video {
namespace {

// Reads bits MSB first out of an RBSP (emulation prevention bytes already removed). Reading
// past the end sets `overrun` and yields zeros, so a parser checks once at the end instead of
// after every field.
class BitReader {
  public:
    explicit BitReader(const std::vector<uint8_t> &rbsp) : data_(rbsp) {}

    uint32_t bits(int n) {
        uint32_t v = 0;
        for (int i = 0; i < n; ++i) {
            v = (v << 1) | bit();
        }
        return v;
    }

    uint32_t bit() {
        if (pos_ >= data_.size() * 8) {
            overrun_ = true;
            return 0;
        }
        const uint32_t b = (data_[pos_ / 8] >> (7 - pos_ % 8)) & 1U;
        ++pos_;
        return b;
    }

    // ue(v): Exp-Golomb, at most 31 leading zeros (anything longer is not a valid SPS field).
    uint32_t ue() {
        int zeros = 0;
        while (bit() == 0) {
            if (overrun_ || ++zeros > 31) {
                overrun_ = true;
                return 0;
            }
        }
        if (zeros == 0) {
            return 0;
        }
        return ((1U << zeros) - 1U) + bits(zeros);
    }

    int32_t se() {
        const uint32_t k = ue();
        return (k & 1U) != 0 ? static_cast<int32_t>((k + 1) / 2) : -static_cast<int32_t>(k / 2);
    }

    bool overrun() const { return overrun_; }

  private:
    const std::vector<uint8_t> &data_;
    size_t pos_{0};
    bool overrun_{false};
};

// The NAL payload with its header byte and the emulation prevention bytes (00 00 03) removed.
std::vector<uint8_t> rbsp_of(const std::vector<uint8_t> &nal) {
    std::vector<uint8_t> out;
    out.reserve(nal.size());
    int zeros = 0;
    for (size_t i = 1; i < nal.size(); ++i) {
        const uint8_t b = nal[i];
        if (zeros >= 2 && b == 0x03) {
            zeros = 0;
            continue;
        }
        zeros = b == 0 ? zeros + 1 : 0;
        out.push_back(b);
    }
    return out;
}

bool high_profile_layout(int profile_idc) {
    switch (profile_idc) {
    case 100:
    case 110:
    case 122:
    case 244:
    case 44:
    case 83:
    case 86:
    case 118:
    case 128:
    case 138:
    case 139:
    case 134:
    case 135:
        return true;
    default:
        return false;
    }
}

} // namespace

std::vector<std::vector<uint8_t>> split_annexb(const uint8_t *data, size_t size) {
    std::vector<std::vector<uint8_t>> nals;
    size_t start = SIZE_MAX; // first byte of the NAL being collected
    size_t i = 0;
    const auto finish = [&](size_t end) {
        if (start == SIZE_MAX) {
            return;
        }
        while (end > start && data[end - 1] == 0) {
            --end;
        }
        if (end > start) {
            nals.emplace_back(data + start, data + end);
        }
    };
    while (i + 2 < size) {
        if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1) {
            finish(i);
            i += 3;
            start = i;
            continue;
        }
        ++i;
    }
    finish(size);
    return nals;
}

std::string profile_level_id(const std::vector<uint8_t> &sps) {
    if (sps.size() < 4 || nal_type(sps) != kNalSps) {
        return {};
    }
    char text[8];
    static const char kHex[] = "0123456789abcdef";
    for (int i = 0; i < 3; ++i) {
        text[2 * i] = kHex[sps[1 + i] >> 4];
        text[2 * i + 1] = kHex[sps[1 + i] & 0x0F];
    }
    return std::string(text, 6);
}

std::string base64(const std::vector<uint8_t> &bytes) {
    static const char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve((bytes.size() + 2) / 3 * 4);
    size_t i = 0;
    for (; i + 2 < bytes.size(); i += 3) {
        const uint32_t v = (uint32_t{bytes[i]} << 16) | (uint32_t{bytes[i + 1]} << 8) | bytes[i + 2];
        out += kAlphabet[(v >> 18) & 63];
        out += kAlphabet[(v >> 12) & 63];
        out += kAlphabet[(v >> 6) & 63];
        out += kAlphabet[v & 63];
    }
    const size_t rest = bytes.size() - i;
    if (rest > 0) {
        const uint32_t v = (uint32_t{bytes[i]} << 16) | (rest > 1 ? uint32_t{bytes[i + 1]} << 8 : 0U);
        out += kAlphabet[(v >> 18) & 63];
        out += kAlphabet[(v >> 12) & 63];
        out += rest > 1 ? kAlphabet[(v >> 6) & 63] : '=';
        out += '=';
    }
    return out;
}

SpsInfo parse_sps(const std::vector<uint8_t> &sps) {
    SpsInfo info;
    if (sps.size() < 4 || nal_type(sps) != kNalSps) {
        return info;
    }
    const std::vector<uint8_t> rbsp = rbsp_of(sps);
    BitReader r(rbsp);
    info.profile_idc = static_cast<int>(r.bits(8));
    info.constraint_flags = static_cast<int>(r.bits(8));
    info.level_idc = static_cast<int>(r.bits(8));
    r.ue(); // seq_parameter_set_id
    uint32_t chroma_format_idc = 1;
    if (high_profile_layout(info.profile_idc)) {
        chroma_format_idc = r.ue();
        if (chroma_format_idc == 3) {
            r.bit(); // separate_colour_plane_flag
        }
        r.ue();  // bit_depth_luma_minus8
        r.ue();  // bit_depth_chroma_minus8
        r.bit(); // qpprime_y_zero_transform_bypass_flag
        if (r.bit() != 0) {
            return info; // seq_scaling_matrix_present_flag: not needed here, not parsed
        }
    }
    r.ue(); // log2_max_frame_num_minus4
    const uint32_t poc_type = r.ue();
    if (poc_type == 0) {
        r.ue(); // log2_max_pic_order_cnt_lsb_minus4
    } else if (poc_type == 1) {
        r.bit(); // delta_pic_order_always_zero_flag
        r.se();  // offset_for_non_ref_pic
        r.se();  // offset_for_top_to_bottom_field
        const uint32_t cycle = r.ue();
        if (cycle > 255) {
            return info;
        }
        for (uint32_t k = 0; k < cycle; ++k) {
            r.se();
        }
    }
    r.ue();  // max_num_ref_frames
    r.bit(); // gaps_in_frame_num_value_allowed_flag
    const uint32_t width_mbs = r.ue() + 1;
    const uint32_t height_units = r.ue() + 1;
    info.frame_mbs_only = r.bit() != 0;
    if (!info.frame_mbs_only) {
        r.bit(); // mb_adaptive_frame_field_flag
    }
    info.direct_8x8_inference = r.bit() != 0;
    uint32_t crop[4] = {0, 0, 0, 0};
    if (r.bit() != 0) {
        for (uint32_t &c : crop) {
            c = r.ue();
        }
    }
    const uint32_t crop_x = chroma_format_idc == 0 || chroma_format_idc == 3 ? 1 : 2;
    const uint32_t crop_y = (chroma_format_idc == 1 ? 2 : 1) * (info.frame_mbs_only ? 1 : 2);
    info.width = static_cast<int>(width_mbs * 16 - crop_x * (crop[0] + crop[1]));
    info.height =
        static_cast<int>((info.frame_mbs_only ? 1 : 2) * height_units * 16 - crop_y * (crop[2] + crop[3]));
    if (r.bit() != 0) { // vui_parameters_present_flag
        if (r.bit() != 0) {
            if (r.bits(8) == 255) { // aspect_ratio_idc == Extended_SAR
                r.bits(16);
                r.bits(16);
            }
        }
        if (r.bit() != 0) {
            r.bit(); // overscan_appropriate_flag
        }
        if (r.bit() != 0) { // video_signal_type_present_flag
            r.bits(3);
            r.bit();
            if (r.bit() != 0) {
                r.bits(24);
            }
        }
        if (r.bit() != 0) { // chroma_loc_info_present_flag
            r.ue();
            r.ue();
        }
        info.timing_info = r.bit() != 0;
        if (info.timing_info) {
            info.num_units_in_tick = r.bits(32);
            info.time_scale = r.bits(32);
            info.fixed_frame_rate = r.bit() != 0;
        }
    }
    info.ok = !r.overrun();
    return info;
}

int level_for(int width, int height, double fps) {
    struct Level {
        int idc;
        double max_mbps; // macroblocks per second
        double max_fs;   // macroblocks per frame
    };
    static constexpr Level kLevels[] = {
        {31, 108000, 3600},  {40, 245760, 8192},  {41, 245760, 8192},   {42, 522240, 8704},
        {50, 589824, 22080}, {51, 983040, 36864}, {52, 2073600, 36864},
    };
    const double w_mbs = std::ceil(width / 16.0);
    const double h_mbs = std::ceil(height / 16.0);
    const double fs = w_mbs * h_mbs;
    for (const Level &l : kLevels) {
        // A.3.1: the frame must fit, and neither side may exceed sqrt(8 * MaxFS) macroblocks.
        const double side = std::sqrt(8.0 * l.max_fs);
        if (fs <= l.max_fs && w_mbs <= side && h_mbs <= side && fs * fps <= l.max_mbps) {
            return l.idc;
        }
    }
    return 52;
}

} // namespace skysim::video
