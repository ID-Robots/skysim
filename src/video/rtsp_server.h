#pragma once
// A small RTSP 1.0 server (RFC 2326) for live H.264, one stream per vehicle instance at
// rtsp://<host>:<port>/instances/<instance>/main.264 — the layout of the SIYI ZR30's own
// rtsp://<cam>:8554/main.264, prefixed by the instance the rest of SkyHub already keys on.
//
// Enough RTSP for ffmpeg/ffprobe, GStreamer's rtspsrc, VLC/live555 and Janus' RTSP source:
// OPTIONS, DESCRIBE (SDP with sprop-parameter-sets), SETUP over RTP/AVP/UDP unicast or
// RTP/AVP/TCP interleaved, PLAY, PAUSE, TEARDOWN, and GET/SET_PARAMETER as keep-alives. No
// authentication, multicast, recording or seeking: a camera on a private network, live only.
//
// One thread does all the socket work, with poll(): reading requests, answering them, and
// sending every receiver its RTP. Frames come in through publish() from any thread, which only
// queues a pointer and pokes an eventfd, so an encoder is never held up by a slow receiver.
// A receiver that cannot keep up loses frames, never the stream: over TCP its backlog is
// bounded, and once that is full frames are dropped until the next IDR, which is asked for (and
// asked for again, once a second, until it comes); over UDP a full socket buffer drops the
// datagram. Frames the server thread could not drain in time (publish() keeps 64) are dropped
// the same way: everyone playing that stream waits for, and asks for, the next IDR.
#include "video/h264.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace skysim::video {

class RtspServer {
  public:
    // What DESCRIBE answers with for one stream.
    struct StreamInfo {
        std::vector<uint8_t> sps;
        std::vector<uint8_t> pps;
        int width{0};
        int height{0};
        double fps{0.0};
        int bitrate_kbps{0};
    };

    // Called on the server thread. None of them may block for long: every receiver's packets
    // wait while they run. (VideoService's describe opens an encoder only when the parameter
    // sets for a stream's frame rate were never made — normally they are made off this thread.)
    struct Hooks {
        // The stream for an instance, or nullopt when there is none (404).
        std::function<std::optional<StreamInfo>(int instance)> describe;
        // A session started (true) or stopped (false) playing an instance's stream.
        std::function<void(int instance, bool playing)> playing;
        // A receiver of this stream needs an IDR: it has just started, or dropped frames.
        std::function<void(int instance)> keyframe;
    };

    struct Config {
        std::string bind{"0.0.0.0"};
        int port{8554}; // 0: any free port (tests read it back from port())
        size_t max_connections{64};
        // Advertised in every Session header. A connection with no playing TCP session that
        // says nothing for 1.5x this long (no request, no RTCP) is closed.
        int session_timeout_s{60};
        // Bytes a TCP receiver may have queued before frames are dropped for it.
        size_t max_backlog_bytes{1u << 20};
    };

    struct Stats {
        size_t connections{0};
        size_t playing{0};
        uint64_t frames_sent{0};    // access units, counted once per receiver
        uint64_t frames_dropped{0}; // access units a receiver could not take
        uint64_t bytes_sent{0};     // RTP and RTCP, all receivers
        // Connections closed for misbehaving: requests piling up unanswered faster than they
        // could be read (input past 64 KiB), or answers piling up unread (see handle_requests).
        uint64_t closed_flooding{0};
        uint64_t closed_not_reading{0};
    };

    // Binds and starts serving. Throws std::runtime_error when the address cannot be bound.
    RtspServer(const Config &cfg, Hooks hooks);
    ~RtspServer();
    RtspServer(const RtspServer &) = delete;
    RtspServer &operator=(const RtspServer &) = delete;

    int port() const;
    // The server's RTP/RTCP UDP pair (even/odd), what SETUP answers as server_port.
    int udp_rtp_port() const;

    // Hands one encoded frame of an instance's stream to everyone playing it. Never blocks on
    // the network; frames queued faster than the server thread drains them are dropped oldest
    // first.
    void publish(int instance, std::shared_ptr<const AccessUnit> au);

    Stats stats() const;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// The instance in a stream URL — rtsp://host:port/instances/<n>/main.264, optionally followed
// by "/" and a track name (what a client appends from the SDP's a=control). Absolute or
// path-only; a query string is ignored. nullopt for anything else.
std::optional<int> parse_stream_path(std::string_view url);

// The first transport in a SETUP Transport header (a comma-separated list) this server can
// serve: RTP/AVP[/UDP];unicast;client_port=a[-b], or RTP/AVP/TCP;interleaved=a[-b].
struct TransportSpec {
    bool tcp{false};
    int rtp_channel{0}; // TCP: interleaved channels
    int rtcp_channel{1};
    int client_rtp_port{0}; // UDP: where the receiver listens
    int client_rtcp_port{0};
};
std::optional<TransportSpec> parse_transport(std::string_view header);

} // namespace skysim::video
