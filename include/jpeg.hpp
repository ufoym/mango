#include "jpeg_core.hpp"

// ----------------------------------------------------------------------
// util
// ----------------------------------------------------------------------

#include <fstream>

mango::ConstMemory read(const char * filename) {
    mango::ConstMemory memory;

    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file.is_open())
        printf("error opening the input file `%s`\n", filename);

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    char * buffer = new char[size];
    if (!file.read(buffer, size))
        printf("error reading the input file `%s`\n", filename);
    file.close();

    memory.size = size;
    memory.address = reinterpret_cast<unsigned char*>(buffer);
    return memory;
}


// ----------------------------------------------------------------------
// image load & save
// ----------------------------------------------------------------------

unsigned char * mango_load_jpeg(
    const char* filename,
    int & width,
    int & height,
    int & channels,
    const bool simd = true,
    const bool multithread = true
) {
    mango::image::ImageDecodeOptions decode_options;
    decode_options.simd = simd;
    decode_options.multithread = multithread;

    mango::ConstMemory memory = read(filename);
    mango::jpeg::Parser parser(memory);
    mango::image::ImageHeader & header = parser.header;
    const size_t stride = header.width * header.format.bytes();
    mango::image::Surface bitmap(
        header.width,
        header.height,
        header.format,
        stride,
        new unsigned char[header.height * stride]
    );
    mango::image::ImageDecodeStatus status = parser.decode(bitmap, decode_options);
    MANGO_UNREFERENCED(status);
    width  = header.width;
    height = header.height;
    channels = header.format.bytes();
    return bitmap.image;
}

void mango_save_jpeg(
    const char* filename,
    unsigned char * data,
    const int width,
    const int height,
    const int channels,
    const int quality = 90,
    const bool simd = true,
    const bool multithread = true
) {
    mango::image::ImageEncodeOptions encode_options;
    encode_options.quality = quality / 100.f;
    encode_options.simd = simd;
    encode_options.multithread = multithread;
    mango::filesystem::FileStream file(filename, mango::Stream::WRITE);

    auto format = channels > 1 ? Format(24, Format::UNORM, Format::BGR, 8, 8, 8)
                               : LuminanceFormat(8, Format::UNORM, 8, 0);
    const size_t stride = width * format.bytes();
    mango::image::Surface bitmap(width, height, format, stride, data);
    mango::image::ImageEncodeStatus status = mango::jpeg::encodeImage(file, bitmap, encode_options);
    MANGO_UNREFERENCED(status);
}
