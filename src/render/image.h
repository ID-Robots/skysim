#pragma once

#include <cstdint>
#include <vector>

namespace skysim::render {

// An RGB8 image, tightly packed, row-major from the top-left.
//
// Its own header because everything downstream of a renderer wants it — the JPEG
// encoder, the frame store, the service — and none of them should have to
// include a particular renderer to get it.
struct Image {
    int width{0};
    int height{0};
    std::vector<uint8_t> rgb;

    bool empty() const { return rgb.empty(); }
};

} // namespace skysim::render
