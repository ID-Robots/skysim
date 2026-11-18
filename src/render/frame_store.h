#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iterator>
#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

namespace skysim::render {

// Latest camera frame per vehicle, handed from the render thread to the HTTP threads.
//
// The two sides run at unrelated rates — one draws when it can, the others serve
// whoever is connected — so the handover is a slot rather than a channel: one
// encoded JPEG per vehicle, replaced whole, copied out under the lock.
//
// Frames are dropped rather than queued. A viewer that falls behind wants the
// newest picture, not a backlog of stale ones.
class FrameStore {
  public:
    struct Frame {
        std::vector<uint8_t> jpeg;
        double sim_time_s{0.0};
        // Which publish this was, counted across the store from 1 (0: no frame). A viewer that
        // remembers the last one it sent can wait for the next without re-reading the slot on a
        // timer, and tell a new frame from an old one even while sim time stands still.
        uint64_t seq{0};
        // Wall-clock time the picture was drawn (or re-sent unchanged), microseconds since the
        // Unix epoch; 0 when unknown. What an MJPEG part's X-Skysim-Capture-Unix-Usec carries,
        // so a receiver can measure the feed's latency.
        int64_t captured_unix_us{0};
        // The vehicle's picture revision this JPEG was drawn for (VehicleCameraState::picture_revision):
        // the intrinsics and GET .../camera name the same number while they describe this picture.
        // It counts within the vehicle the slot is keyed by: the routes that serve the frame name
        // that vehicle too (X-Skysim-Vehicle), since a replacement at one instance starts again.
        uint64_t picture_revision{0};
    };

    void publish(uint32_t vehicle_id, std::vector<uint8_t> jpeg, double sim_time_s,
                 int64_t captured_unix_us = 0, uint64_t picture_revision = 0) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            Frame &slot = frames_[vehicle_id];
            slot.jpeg = std::move(jpeg);
            slot.sim_time_s = sim_time_s;
            slot.seq = ++seq_;
            slot.captured_unix_us = captured_unix_us;
            slot.picture_revision = picture_revision;
        }
        published_.notify_all();
    }

    // Empty if that vehicle has never had a frame rendered.
    Frame get(uint32_t vehicle_id) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = frames_.find(vehicle_id);
        if (it == frames_.end()) {
            return {};
        }
        return it->second;
    }

    // The vehicle's frame as soon as it is newer than `after_seq`, or whatever the slot holds
    // (possibly nothing) once `deadline` passes. The JPEG is copied once, when it is returned.
    Frame wait_newer(uint32_t vehicle_id, uint64_t after_seq,
                     std::chrono::steady_clock::time_point deadline) const {
        std::unique_lock<std::mutex> lock(mutex_);
        const auto newer = [&] {
            auto it = frames_.find(vehicle_id);
            return it != frames_.end() && it->second.seq > after_seq;
        };
        published_.wait_until(lock, deadline, newer);
        auto it = frames_.find(vehicle_id);
        return it == frames_.end() ? Frame{} : it->second;
    }

    void erase(uint32_t vehicle_id) {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.erase(vehicle_id);
    }

    // Drop every vehicle not in `keep`.
    //
    // Without this a despawned vehicle's last frame outlives it: the camera route
    // keeps serving a picture for an aircraft that no longer exists, instead of
    // the 404 the caller needs to see, and the map grows for the life of the
    // process as ids churn.
    void retain(const std::vector<uint32_t> &keep) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto it = frames_.begin(); it != frames_.end();) {
            it = std::find(keep.begin(), keep.end(), it->first) == keep.end() ? frames_.erase(it) : std::next(it);
        }
    }

  private:
    mutable std::mutex mutex_;
    mutable std::condition_variable published_;
    std::unordered_map<uint32_t, Frame> frames_;
    uint64_t seq_{0};
};

} // namespace skysim::render
