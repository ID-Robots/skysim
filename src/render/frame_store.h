#pragma once

#include <cstdint>
#include <mutex>
#include <unordered_map>
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

  private:
    mutable std::mutex mutex_;
    std::unordered_map<uint32_t, Frame> frames_;
};

} // namespace skysim::render
