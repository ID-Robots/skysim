"""A small RTSP client for skysim's H.264 camera streams (TASK-2561), stdlib only.

DESCRIBE / SETUP (RTP over the RTSP connection, interleaved) / PLAY / TEARDOWN, RTP/H.264
reassembly (single NAL units and FU-A, RFC 6184), and the RTCP sender reports that map RTP
timestamps to wall-clock capture times. Used by tests/rtsp_smoke.py to look inside the stream
(SPS/PPS on every IDR, keyframe spacing) and by tools/bench/camera_bench.py to measure latency.
"""
from __future__ import annotations

import socket
import struct
import time
import urllib.parse
from dataclasses import dataclass, field

NTP_UNIX_OFFSET = 2208988800


@dataclass
class Frame:
    """One access unit: its NAL units (no start codes), RTP timestamp, and arrival wall time."""

    rtp_ts: int
    nals: list[bytes]
    received: float
    size: int = 0

    def types(self) -> set[int]:
        return {n[0] & 0x1F for n in self.nals if n}

    @property
    def idr(self) -> bool:
        return 5 in self.types()


@dataclass
class Response:
    status: int
    headers: dict[str, str] = field(default_factory=dict)
    body: str = ""


class RtspClient:
    def __init__(self, url: str, timeout: float = 5.0) -> None:
        self.url = url
        parts = urllib.parse.urlsplit(url)
        self.sock = socket.create_connection((parts.hostname or "127.0.0.1", parts.port or 554), timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.buf = b""
        self.cseq = 0
        self.session = ""
        self.sdp = ""
        self.frames: list[Frame] = []
        self.packets = 0
        self._nals: list[bytes] = []
        self._fu: bytearray | None = None
        self._size = 0
        # The latest sender report: (RTP timestamp, wall-clock seconds since the Unix epoch).
        self.report: tuple[int, float] | None = None
        self._responses: list[Response] = []

    # --- RTSP ---------------------------------------------------------------------------------
    def request(self, method: str, url: str, headers: dict[str, str] | None = None) -> Response:
        self.cseq += 1
        lines = [f"{method} {url} RTSP/1.0", f"CSeq: {self.cseq}", "User-Agent: skysim-rtsp-client"]
        if self.session:
            lines.append(f"Session: {self.session}")
        for k, v in (headers or {}).items():
            lines.append(f"{k}: {v}")
        self.sock.sendall(("\r\n".join(lines) + "\r\n\r\n").encode())
        deadline = time.monotonic() + 10.0
        while not self._responses:
            if time.monotonic() > deadline or not self._pump():
                raise TimeoutError(f"no response to {method}")
        return self._responses.pop(0)

    def describe(self) -> Response:
        r = self.request("DESCRIBE", self.url, {"Accept": "application/sdp"})
        self.sdp = r.body
        return r

    def play(self) -> Response:
        r = self.request("SETUP", self.url + "/trackID=0", {"Transport": "RTP/AVP/TCP;unicast;interleaved=0-1"})
        if r.status != 200:
            return r
        self.session = r.headers.get("session", "").split(";")[0]
        return self.request("PLAY", self.url, {"Range": "npt=0.000-"})

    def close(self) -> None:
        try:
            if self.session:
                self.request("TEARDOWN", self.url)
        except (OSError, TimeoutError):
            pass
        self.sock.close()

    # --- media --------------------------------------------------------------------------------
    def read_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if not self._pump(max(deadline - time.monotonic(), 0.01)):
                break

    def read_frames(self, count: int, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while len(self.frames) < count and time.monotonic() < deadline:
            self._pump(max(deadline - time.monotonic(), 0.01))
        return len(self.frames) >= count

    def capture_time(self, frame: Frame) -> float | None:
        """Wall-clock time the frame was drawn, from the latest sender report."""
        if self.report is None:
            return None
        rtp, wall = self.report
        delta = (frame.rtp_ts - rtp + (1 << 31)) % (1 << 32) - (1 << 31)
        return wall + delta / 90000.0

    # --- plumbing -----------------------------------------------------------------------------
    def _pump(self, timeout: float = 5.0) -> bool:
        self.sock.settimeout(timeout)
        try:
            data = self.sock.recv(262144)
        except TimeoutError:
            return False
        if not data:
            return False
        now = time.time()
        self.buf += data
        while self.buf:
            if self.buf[:1] == b"$":
                if len(self.buf) < 4:
                    break
                channel = self.buf[1]
                (length,) = struct.unpack(">H", self.buf[2:4])
                if len(self.buf) < 4 + length:
                    break
                packet = self.buf[4 : 4 + length]
                self.buf = self.buf[4 + length :]
                if channel == 0:
                    self._rtp(packet, now)
                else:
                    self._rtcp(packet, now)
                continue
            end = self.buf.find(b"\r\n\r\n")
            if end < 0:
                break
            head = self.buf[:end].decode(errors="replace").split("\r\n")
            headers = {}
            for line in head[1:]:
                if ":" in line:
                    k, v = line.split(":", 1)
                    headers[k.strip().lower()] = v.strip()
            length = int(headers.get("content-length", "0"))
            if len(self.buf) < end + 4 + length:
                break
            body = self.buf[end + 4 : end + 4 + length].decode(errors="replace")
            self.buf = self.buf[end + 4 + length :]
            status = int(head[0].split()[1]) if head[0].startswith("RTSP/") else 0
            self._responses.append(Response(status, headers, body))
        return True

    def _rtp(self, packet: bytes, now: float) -> None:
        if len(packet) < 13:
            return
        self.packets += 1
        marker = packet[1] & 0x80
        (ts,) = struct.unpack(">I", packet[4:8])
        payload = packet[12:]
        self._size += len(payload)
        kind = payload[0] & 0x1F
        if kind == 28 and len(payload) >= 2:
            if payload[1] & 0x80:
                self._fu = bytearray([(payload[0] & 0xE0) | (payload[1] & 0x1F)])
            if self._fu is not None:
                self._fu += payload[2:]
                if payload[1] & 0x40:
                    self._nals.append(bytes(self._fu))
                    self._fu = None
        elif 1 <= kind <= 23:
            self._nals.append(payload)
        if marker:
            self.frames.append(Frame(ts, self._nals, now, self._size))
            self._nals = []
            self._size = 0

    def _rtcp(self, packet: bytes, now: float) -> None:
        if len(packet) >= 20 and packet[1] == 200:
            msw, lsw, rtp = struct.unpack(">III", packet[8:20])
            self.report = (rtp, msw - NTP_UNIX_OFFSET + lsw / 4294967296.0)
