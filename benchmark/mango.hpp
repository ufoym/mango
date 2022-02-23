#include <mango/mango.hpp>

// ----------------------------------------------------------------------
// util
// ----------------------------------------------------------------------

#include <fstream>

mango::ConstMemory read(const char * filename) {
    mango::ConstMemory memory;

    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file.is_open())
        printf("error opening the input file\n");

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    char * buffer = new char[size];
    if (!file.read(buffer, size))
        printf("error reading the input file\n");
    file.close();

    memory.size = size;
    memory.address = reinterpret_cast<unsigned char*>(buffer);
    return memory;
}


// ----------------------------------------------------------------------
// image load & save
// ----------------------------------------------------------------------

mango::image::Surface mango_load_jpeg(
    const char * filename,
    const bool simd = true,
    const bool multithread = true
) {
    mango::image::ImageDecodeOptions decode_options;
    decode_options.simd = simd;
    decode_options.multithread = multithread;

    mango::image::Surface bitmap;

    mango::ConstMemory memory = read(filename);
    mango::image::ImageDecoder decoder(memory, filename);
    if (decoder.isDecoder()) {
        mango::image::ImageHeader header = decoder.header();
        bitmap.format = header.format;
        if (decode_options.palette) {
            decode_options.palette->size = 0;
            if (header.palette)
                bitmap.format = mango::image::IndexedFormat(8);
        }
        bitmap.width  = header.width;
        bitmap.height = header.height;
        bitmap.stride = header.width * bitmap.format.bytes();
        bitmap.image  = new unsigned char[header.height * bitmap.stride];
        mango::image::ImageDecodeStatus status = decoder.decode(bitmap, decode_options, 0, 0, 0);
        MANGO_UNREFERENCED(status);
    }

    return bitmap;
}

void mango_save_jpeg(
    const char* filename,
    const mango::image::Surface& bitmap,
    const float quality = 0.7f,
    const bool simd = true,
    const bool multithread = true
) {
    mango::image::ImageEncodeOptions encode_options;
    encode_options.quality = quality;
    encode_options.simd = simd;
    encode_options.multithread = multithread;
    bitmap.save(filename, encode_options);
}
