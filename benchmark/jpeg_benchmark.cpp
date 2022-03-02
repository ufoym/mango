/*
    MANGO Multimedia Development Platform
    Copyright (C) 2012-2021 Twilight Finland 3D Oy Ltd. All rights reserved.
*/
#include <jpeg.hpp>

#define TEST_LIBJPEG
#define TEST_STB
#define TEST_JPEG_COMPRESSOR

// ----------------------------------------------------------------------
// libjpeg
// ----------------------------------------------------------------------

#include <jpeglib.h>
#include <jerror.h>

unsigned char * load_jpeg(
    const char* filename,
    int & width,
    int & height,
    int & channels)
{
    FILE* file = fopen(filename, "rb" );
    if (!file)
        return nullptr;

    struct jpeg_decompress_struct info;
    struct jpeg_error_mgr err;

    info.err = jpeg_std_error(&err);
    jpeg_create_decompress(&info);

    jpeg_stdio_src(&info, file);
    jpeg_read_header(&info, TRUE);

    jpeg_start_decompress(&info);

    width = info.output_width;
    height = info.output_height;
    channels = info.num_components;
    unsigned long dataSize = width * height * channels;

    // read scanlines one at a time & put bytes in jdata[] array (assumes an RGB image)
    unsigned char *data = new unsigned char[dataSize];;
    unsigned char *rowptr[ 1 ]; // array or pointers
    for ( ; info.output_scanline < info.output_height ; )
    {
        rowptr[ 0 ] = data + info.output_scanline * width * channels;
        jpeg_read_scanlines( &info, rowptr, 1 );
    }
    jpeg_finish_decompress(&info);
    fclose(file);
    return data;
}

void save_jpeg(
    const char* filename,
    unsigned char * data,
    const int width,
    const int height,
    const int channels,
    const int quality = 90)
{
    struct jpeg_compress_struct cinfo;
    jpeg_create_compress(&cinfo);

    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);

    FILE * outfile;
    if ((outfile = fopen(filename, "wb")) == NULL)
    {
        fprintf(stderr, "can't open %s\n", filename);
        exit(1);
    }
    jpeg_stdio_dest(&cinfo, outfile);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = channels;
    cinfo.in_color_space = JCS_RGB;

    bool progressive = false;

    jpeg_set_defaults(&cinfo);
    if (progressive)
        jpeg_simple_progression(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    JSAMPROW row_pointer[1];
    const int stride = width * channels;
    while (cinfo.next_scanline < cinfo.image_height)
    {
        row_pointer[0] = data + cinfo.next_scanline * stride;
        int x = jpeg_write_scanlines(&cinfo, row_pointer, 1);
        (void) x;
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    fclose(outfile);
}

// ----------------------------------------------------------------------
// stb
// ----------------------------------------------------------------------

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

mango::image::Surface stb_load_jpeg(const char* filename)
{
    int width, height, bpp;
    unsigned char* rgb = stbi_load(filename, &width, &height, &bpp, 3);

    return mango::image::Surface(
        width, height, mango::image::Format(24, mango::image::Format::UNORM,
        mango::image::Format::RGB, 8, 8, 8), width * 3, rgb);
}

void stb_save_jpeg(const char* filename, const mango::image::Surface& surface)
{
    stbi_write_jpg(filename, surface.width, surface.height, 3, surface.image, surface.width*3);
    stbi_image_free(surface.image);
}

// ----------------------------------------------------------------------
// jpeg-compressor
// ----------------------------------------------------------------------

#ifdef TEST_JPEG_COMPRESSOR

#include "jpeg-compressor/jpgd.h"
#include "jpeg-compressor/jpge.h"

mango::image::Surface jpgd_load(const char* filename)
{
    int width;
    int height;
    int comps;
    unsigned char* image = jpgd::decompress_jpeg_image_from_file(filename, &width, &height, &comps, 4);
    return mango::image::Surface(
        width, height, mango::image::Format(32, mango::image::Format::UNORM,
        mango::image::Format::BGRA, 8, 8, 8, 8), width * 4, image);
}

void jpge_save(const char* filename, const mango::image::Surface& surface)
{
    jpge::compress_image_to_jpeg_file(filename, surface.width, surface.height, 4, surface.image);
    free(surface.image);
}

#endif

// ----------------------------------------------------------------------
// print
// ----------------------------------------------------------------------

void print(const char* name, std::uint64_t load, std::uint64_t save)
{
    printf("%s", name);
    printf("%7d.%d ms ", int(load / 1000), int(load % 1000) / 100);
    printf("%7d.%d ms ", int(save / 1000), int(save % 1000) / 100);
    printf("\n");
}

// ----------------------------------------------------------------------
// main()
// ----------------------------------------------------------------------

int main(int argc, const char* argv[])
{
    const char* default_filename = "../../test/images/jpeg444.jpg";
    const char* filename = argc > 1 ? argv[1] : default_filename;

    printf("%s\n", mango::getSystemInfo().c_str());
    printf("----------------------------------------------\n");
    printf("                load         save             \n");
    printf("----------------------------------------------\n");

    std::uint64_t time0;
    std::uint64_t time1;
    std::uint64_t time2;

    // ------------------------------------------------------------------

#ifdef TEST_LIBJPEG

    int width = 0, height = 0, channels = 0;

    time0 = mango::Time::us();
    unsigned char * img = load_jpeg(filename, width, height, channels);

    time1 = mango::Time::us();
    save_jpeg("output-libjpeg.jpg", img, width, height, channels);

    time2 = mango::Time::us();
    print("libjpeg: ", time1 - time0, time2 - time1);
    delete [] img;

#endif

    // ------------------------------------------------------------------

#ifdef TEST_STB

    time0 = mango::Time::us();
    mango::image::Surface s_stb = stb_load_jpeg(filename);

    time1 = mango::Time::us();
    stb_save_jpeg("output-stb.jpg", s_stb);

    time2 = mango::Time::us();
    print("stb:     ", time1 - time0, time2 - time1);

#endif

    // ------------------------------------------------------------------

#ifdef TEST_JPEG_COMPRESSOR

    time0 = mango::Time::us();
    mango::image::Surface s_jpgd = jpgd_load(filename);

    time1 = mango::Time::us();
    jpge_save("output-jpge.jpg", s_jpgd);

    time2 = mango::Time::us();
    print("jpgd:    ", time1 - time0, time2 - time1);

#endif

    // ------------------------------------------------------------------

    time0 = mango::Time::us();
    mango::image::Surface bitmap = mango_load_jpeg(filename);

    time1 = mango::Time::us();
    mango_save_jpeg("output-mango.jpg", bitmap);

    time2 = mango::Time::us();
    print("mango:   ", time1 - time0, time2 - time1);
}
