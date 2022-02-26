/*
    MANGO Multimedia Development Platform
    Copyright (C) 2012-2021 Twilight Finland 3D Oy Ltd. All rights reserved.
*/
#pragma once

#include <cstddef>
#include <string>
#include <mango/core/configure.hpp>
#include <mango/core/memory.hpp>
#include <mango/filesystem/file.hpp>
#include <mango/image/format.hpp>

namespace mango::image
{

    class Surface
    {
    public:
        Format  format;
        u8*     image;
        size_t  stride;
        int     width;
        int     height;

        Surface();
        Surface(const Surface& surface);
        Surface(int width, int height, const Format& format, size_t stride, const void* image);
        Surface(const Surface& source, int x, int y, int width, int height);
        ~Surface();

        Surface& operator = (const Surface& surface);

        u8* address(int x = 0, int y = 0) const
        {
            u8* scan = image + y * stride;
            return scan + x * format.bytes();
        }

        template <typename SampleType>
        SampleType* address(int x = 0, int y = 0) const
        {
            SampleType* scan = reinterpret_cast<SampleType*>(image + y * stride);
            return scan + x;
        }

        void clear(float red, float green, float blue, float alpha) const;
        void clear(Color color) const;
        void blit(int x, int y, const Surface& source) const;
        void xflip() const;
        void yflip() const;
    };

    class Bitmap : private NonCopyable, public Surface
    {
    public:
        Bitmap(int width, int height, const Format& format, size_t stride = 0);
        Bitmap(const Surface& source, const Format& format);
        Bitmap(Bitmap&& bitmap);
        ~Bitmap();

        Bitmap& operator = (Bitmap&& bitmap);
    };

} // namespace mango::image
