#pragma once
// RTP for H.264 (RFC 6184, packetization-mode=1) and the RTCP sender report that goes with it
// (RFC 3550). One packetizer per receiver: each RTSP session has its own SSRC and sequence
// numbers, so a receiver that joins late or drops out does not disturb anyone else's.
#include "video/h264.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string_view>
#include <vector>

namespace skysim::video {

inline constexpr uint8_t kRtpPayloadType = 96; // dynamic, mapped to H264/90000 in the SDP
inline constexpr uint32_t kRtpClockHz = 90000;
inline constexpr size_t kRtpHeaderBytes = 12;
// Payload bytes per packet: an RTP packet then fits a 1500-byte Ethernet MTU with IPv4/IPv6,
// UDP and a tunnel's overhead to spare, and over TCP it is one interleaved frame.
inline constexpr size_t kRtpMaxPayload = 1400;

class RtpPacketizer {
  public:
    // One RTP packet: 12-byte header followed by its payload, valid during the call only.
    using Emit = std::function<void(const uint8_t *packet, size_t size)>;

    RtpPacketizer(uint32_t ssrc, uint16_t first_seq, size_t max_payload = kRtpMaxPayload);

    // Every NAL of the access unit, in order: a NAL that fits goes as a single NAL unit
    // packet, a larger one as FU-A fragments. The marker bit is set on the access unit's last
    // packet, which is how a receiver knows the picture is complete.
    void packetize(const AccessUnit &au, uint32_t timestamp, const Emit &emit);

    uint32_t ssrc() const { return ssrc_; }
    uint16_t next_seq() const { return seq_; }
    // Totals for the sender report: packets sent and payload octets (headers excluded).
    uint32_t packets() const { return packets_; }
    uint32_t octets() const { return octets_; }

  private:
    void send(const uint8_t *head, size_t head_size, const uint8_t *body, size_t body_size, bool marker,
              uint32_t timestamp, const Emit &emit);

    uint32_t ssrc_;
    uint16_t seq_;
    size_t max_payload_;
    uint32_t packets_{0};
    uint32_t octets_{0};
    std::vector<uint8_t> buffer_;
};

// 64-bit NTP timestamp (seconds since 1900 in the high word, binary fraction in the low).
uint64_t ntp_time(std::chrono::system_clock::time_point t);

// A compound RTCP packet: sender report (no report blocks) followed by an SDES with CNAME.
// The report maps `rtp_timestamp` to wall-clock `ntp`, which is what lets a receiver turn RTP
// timestamps into capture times — and so measure latency, or line two streams up.
std::vector<uint8_t> rtcp_sender_report(uint32_t ssrc, uint64_t ntp, uint32_t rtp_timestamp, uint32_t packets,
                                        uint32_t octets, std::string_view cname);

} // namespace skysim::video
