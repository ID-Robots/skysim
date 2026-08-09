#pragma once

#include "render/image.h"

#include <cstdint>
#include <vector>

namespace skysim::render {

// Encode an RGB8 image as baseline JPEG.
//
// JPEG rather than raw frames because the consumer is a GStreamer pipeline in
// another container: a 256x144 frame is ~110 kB raw and a few kB compressed, and
// at this resolution the encode is cheaper than pushing the raw bytes through a
// socket would be.
//
// quality is 1-100; returns an empty vector if the image is empty.
std::vector<uint8_t> encode_jpeg(const Image &image, int quality = 70);

} // namespace skysim::render
