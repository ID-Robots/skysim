#include "render/jpeg.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
// Writing to a file is never wanted here — frames go straight out over HTTP —
// and pulling in stdio widens what a physics server links against for nothing.
#define STBI_WRITE_NO_STDIO
#include <stb_image_write.h>

namespace skysim::render {
namespace {

void append_chunk(void *context, void *data, int size) {
    auto *out = static_cast<std::vector<uint8_t> *>(context);
    const auto *bytes = static_cast<const uint8_t *>(data);
    out->insert(out->end(), bytes, bytes + size);
}

} // namespace

std::vector<uint8_t> encode_jpeg(const Image &image, int quality) {
    std::vector<uint8_t> out;
    if (image.empty() || image.width <= 0 || image.height <= 0) {
        return out;
    }

    out.reserve(static_cast<size_t>(image.width) * image.height / 4);
    if (stbi_write_jpg_to_func(&append_chunk, &out, image.width, image.height, 3, image.rgb.data(), quality) == 0) {
        out.clear();
    }
    return out;
}

} // namespace skysim::render
