#pragma once
// A picture drawn smaller than it is delivered, scaled up (TASK-2661).
//
// A vehicle's picture has one size for its whole life (render/camera_mode.h): a camera drawing
// its cheap normal mode draws fewer pixels and this scales them to the picture size, so whoever
// reads the stream sees one picture size and one set of intrinsics, landing or not.
#include "core/thread_pool.h"
#include "render/image.h"

namespace skysim::render {

// `src` scaled to `width` x `height` with bilinear filtering, pixel centres aligned (a pixel of the
// result samples the source at the same place in the frame), rows spread over `pool` (null: on
// the calling thread). The same picture whatever the pool. An empty source gives an empty image.
Image resize_bilinear(const Image &src, int width, int height, core::ThreadPool *pool = nullptr);

} // namespace skysim::render
