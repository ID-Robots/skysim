#pragma once

#include <algorithm>
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
    };

    void publish(uint32_t vehicle_id, std::vector<uint8_t> jpeg, double sim_time_s) {
        std::lock_guard<std::mutex> lock(mutex_);
        Frame &slot = frames_[vehicle_id];
        slot.jpeg = std::move(jpeg);
        slot.sim_time_s = sim_time_s;
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
    std::unordered_map<uint32_t, Frame> frames_;
};

} // namespace skysim::render
