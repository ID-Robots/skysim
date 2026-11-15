#include "video/rtp_h264.h"

#include <algorithm>

namespace skysim::video {
namespace {

void put16(uint8_t *p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v >> 8);
    p[1] = static_cast<uint8_t>(v);
}

void put32(uint8_t *p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);
    p[3] = static_cast<uint8_t>(v);
}

} // namespace

RtpPacketizer::RtpPacketizer(uint32_t ssrc, uint16_t first_seq, size_t max_payload)
    : ssrc_(ssrc), seq_(first_seq), max_payload_(std::max<size_t>(max_payload, 16)) {
    buffer_.resize(kRtpHeaderBytes + max_payload_);
}

void RtpPacketizer::send(const uint8_t *head, size_t head_size, const uint8_t *body, size_t body_size,
                         bool marker, uint32_t timestamp, const Emit &emit) {
    uint8_t *p = buffer_.data();
    p[0] = 0x80; // V=2, no padding, no extension, no CSRC
    p[1] = static_cast<uint8_t>((marker ? 0x80 : 0x00) | kRtpPayloadType);
    put16(p + 2, seq_);
    put32(p + 4, timestamp);
    put32(p + 8, ssrc_);
    std::copy(head, head + head_size, p + kRtpHeaderBytes);
    std::copy(body, body + body_size, p + kRtpHeaderBytes + head_size);
    const size_t payload = head_size + body_size;
    ++seq_;
    ++packets_;
    octets_ += static_cast<uint32_t>(payload);
    emit(p, kRtpHeaderBytes + payload);
}

void RtpPacketizer::packetize(const AccessUnit &au, uint32_t timestamp, const Emit &emit) {
    // The marker goes on the last packet of the last NAL that is sent, so a trailing empty
    // entry cannot leave the access unit unterminated.
    size_t last = au.nals.size();
    while (last > 0 && au.nals[last - 1].empty()) {
        --last;
    }
    for (size_t n = 0; n < au.nals.size(); ++n) {
        const std::vector<uint8_t> &nal = au.nals[n];
        if (nal.empty()) {
            continue;
        }
        const bool last_nal = n + 1 == last;
        if (nal.size() <= max_payload_) {
            send(nal.data(), 0, nal.data(), nal.size(), last_nal, timestamp, emit);
            continue;
        }
        // FU-A: the NAL header's F and NRI bits travel in the indicator, its type in the FU
        // header; the header byte itself is not sent.
        const uint8_t head[2] = {static_cast<uint8_t>((nal[0] & 0xE0) | kNalFuA),
                                 static_cast<uint8_t>(nal[0] & 0x1F)};
        const size_t chunk = max_payload_ - 2;
        for (size_t off = 1; off < nal.size(); off += chunk) {
            const size_t len = std::min(chunk, nal.size() - off);
            const bool first = off == 1;
            const bool end = off + len == nal.size();
            const uint8_t fu[2] = {
                head[0], static_cast<uint8_t>(head[1] | (first ? 0x80 : 0x00) | (end ? 0x40 : 0x00))};
            send(fu, 2, nal.data() + off, len, last_nal && end, timestamp, emit);
        }
    }
}

uint64_t ntp_time(std::chrono::system_clock::time_point t) {
    constexpr uint64_t kUnixToNtp = 2208988800ULL;
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
    const uint64_t sec = static_cast<uint64_t>(ns / 1000000000LL) + kUnixToNtp;
    const uint64_t frac = (static_cast<uint64_t>(ns % 1000000000LL) << 32) / 1000000000ULL;
    return (sec << 32) | frac;
}

std::vector<uint8_t> rtcp_sender_report(uint32_t ssrc, uint64_t ntp, uint32_t rtp_timestamp, uint32_t packets,
                                        uint32_t octets, std::string_view cname) {
    const size_t name = std::min<size_t>(cname.size(), 255);
    // SDES chunk: SSRC, CNAME item (type, length, text), END, padded to a 32-bit boundary.
    const size_t chunk = (4 + 2 + name + 1 + 3) / 4 * 4;
    std::vector<uint8_t> out(28 + 4 + chunk, 0);
    uint8_t *sr = out.data();
    sr[0] = 0x80; // V=2, RC=0
    sr[1] = 200;  // SR
    put16(sr + 2, 6);
    put32(sr + 4, ssrc);
    put32(sr + 8, static_cast<uint32_t>(ntp >> 32));
    put32(sr + 12, static_cast<uint32_t>(ntp));
    put32(sr + 16, rtp_timestamp);
    put32(sr + 20, packets);
    put32(sr + 24, octets);
    uint8_t *sdes = sr + 28;
    sdes[0] = 0x81; // V=2, SC=1
    sdes[1] = 202;  // SDES
    put16(sdes + 2, static_cast<uint32_t>((4 + chunk) / 4 - 1));
    put32(sdes + 4, ssrc);
    sdes[8] = 1; // CNAME
    sdes[9] = static_cast<uint8_t>(name);
    std::copy(cname.data(), cname.data() + name, sdes + 10);
    return out;
}

} // namespace skysim::video
