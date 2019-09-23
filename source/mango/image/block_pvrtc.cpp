/*
    MANGO Multimedia Development Platform
    Copyright (C) 2012-2019 Twilight Finland 3D Oy Ltd. All rights reserved.
*/
#include <mango/image/compression.hpp>

namespace
{
    using namespace mango;

	// ----------------------------------------------------------------------------
    // pvrtc
    // ----------------------------------------------------------------------------

    //
    // Copyright (c) Imagination Technologies Limited.
    //
    // PVRTC decompressor (C) Imagination Technologies Limited.
    // Adapted and optimized for MANGO in December 2016.

    struct Pixel32
    {
        u8 red, green, blue, alpha;
    };

    struct Pixel128S
    {
        s32 red, green, blue, alpha;
    };

    struct PVRTCWord
    {
        u32 u32ModulationData;
        u32 u32ColorData;
    };

    constexpr u32 PUNCHTHROUGH_ALPHA = 0x10;

    static Pixel32 getColorA(u32 u32ColorData)
    {
        Pixel32 color;

        if ((u32ColorData & 0x8000) != 0)
        {
            // Opaque Color Mode - RGB 554
            color.red   = (u8)((u32ColorData & 0x7c00) >> 10); // 5->5 bits
            color.green = (u8)((u32ColorData & 0x3e0)  >> 5); // 5->5 bits
            color.blue  = (u8)(u32ColorData  & 0x1e) | ((u32ColorData & 0x1e) >> 4); // 4->5 bits
            color.alpha = (u8)0xf;// 0->4 bits
        }
        else
        {
            // Transparent Color Mode - ARGB 3443
            color.red   = (u8)((u32ColorData & 0xf00)  >> 7) | ((u32ColorData & 0xf00) >> 11); // 4->5 bits
            color.green = (u8)((u32ColorData & 0xf0)   >> 3) | ((u32ColorData & 0xf0)  >> 7); // 4->5 bits
            color.blue  = (u8)((u32ColorData & 0xe)    << 1) | ((u32ColorData & 0xe)   >> 2); // 3->5 bits
            color.alpha = (u8)((u32ColorData & 0x7000) >> 11);// 3->4 bits - note 0 at right
        }
        
        return color;
    }

    static Pixel32 getColorB(u32 u32ColorData)
    {
        Pixel32 color;

        if (u32ColorData & 0x80000000)
        {
            // Opaque Color Mode - RGB 555
            color.red   = (u8)((u32ColorData & 0x7c000000) >> 26); // 5->5 bits
            color.green = (u8)((u32ColorData & 0x3e00000)  >> 21); // 5->5 bits
            color.blue  = (u8)((u32ColorData & 0x1f0000)   >> 16); // 5->5 bits
            color.alpha = (u8)0xf;// 0 bits
        }
        else
        {
            // Transparent Color Mode - ARGB 3444
            color.red   = (u8)(((u32ColorData & 0xf000000)  >> 23) | ((u32ColorData & 0xf000000) >> 27)); // 4->5 bits
            color.green = (u8)(((u32ColorData & 0xf00000)   >> 19) | ((u32ColorData & 0xf00000)  >> 23)); // 4->5 bits
            color.blue  = (u8)(((u32ColorData & 0xf0000)    >> 15) | ((u32ColorData & 0xf0000)   >> 19)); // 4->5 bits
            color.alpha = (u8)((u32ColorData & 0x70000000) >> 27);// 3->4 bits - note 0 at right
        }

        return color;
    }

    static void interpolateColors(Pixel32 color[4], Pixel128S* pPixel, u8 ui8Bpp)
    {
        const u32 ui32WordWidth = (ui8Bpp == 2) ? 8 : 4;
        const u32 ui32WordHeight = 4;

        //Convert to int 32.
        Pixel128S hP = { (s32)color[0].red, (s32)color[0].green, (s32)color[0].blue, (s32)color[0].alpha };
        Pixel128S hQ = { (s32)color[1].red, (s32)color[1].green, (s32)color[1].blue, (s32)color[1].alpha };
        Pixel128S hR = { (s32)color[2].red, (s32)color[2].green, (s32)color[2].blue, (s32)color[2].alpha };
        Pixel128S hS = { (s32)color[3].red, (s32)color[3].green, (s32)color[3].blue, (s32)color[3].alpha };

        //Get vectors.
        Pixel128S QminusP = {hQ.red - hP.red, hQ.green - hP.green, hQ.blue - hP.blue, hQ.alpha - hP.alpha};
        Pixel128S SminusR = {hS.red - hR.red, hS.green - hR.green, hS.blue - hR.blue, hS.alpha - hR.alpha};

        //Multiply colors.
        hP.red		*=	ui32WordWidth;
        hP.green	*=	ui32WordWidth;
        hP.blue		*=	ui32WordWidth;
        hP.alpha	*=	ui32WordWidth;
        hR.red		*=	ui32WordWidth;
        hR.green	*=	ui32WordWidth;
        hR.blue		*=	ui32WordWidth;
        hR.alpha	*=	ui32WordWidth;

        if (ui8Bpp == 2)
        {
            //Loop through pixels to achieve results.
            for (unsigned int x = 0; x < ui32WordWidth; x++)
            {
                Pixel128S result = {4 * hP.red, 4 * hP.green, 4 * hP.blue, 4 * hP.alpha};
                Pixel128S dY = {hR.red - hP.red, hR.green - hP.green, hR.blue - hP.blue, hR.alpha - hP.alpha};

                for (unsigned int y = 0; y < ui32WordHeight; y++)
                {
                    pPixel[y * ui32WordWidth + x].red   = (s32)((result.red   >> 7) + (result.red   >> 2));
                    pPixel[y * ui32WordWidth + x].green = (s32)((result.green >> 7) + (result.green >> 2));
                    pPixel[y * ui32WordWidth + x].blue  = (s32)((result.blue  >> 7) + (result.blue  >> 2));
                    pPixel[y * ui32WordWidth + x].alpha = (s32)((result.alpha >> 5) + (result.alpha >> 1));

                    result.red   += dY.red;
                    result.green += dY.green;
                    result.blue  += dY.blue;
                    result.alpha += dY.alpha;
                }

                hP.red		+= QminusP.red;
                hP.green	+= QminusP.green;
                hP.blue		+= QminusP.blue;
                hP.alpha	+= QminusP.alpha;

                hR.red		+= SminusR.red;
                hR.green	+= SminusR.green;
                hR.blue		+= SminusR.blue;
                hR.alpha	+= SminusR.alpha;
            }
        }
        else
        {
            //Loop through pixels to achieve results.
            for (unsigned int y = 0; y < ui32WordHeight; y++)
            {
                Pixel128S result = {4 * hP.red, 4 * hP.green, 4 * hP.blue, 4 * hP.alpha};
                Pixel128S dY = {hR.red - hP.red, hR.green - hP.green, hR.blue - hP.blue, hR.alpha - hP.alpha};

                for (unsigned int x = 0; x < ui32WordWidth; x++)
                {
                    pPixel[x * ui32WordWidth + y].red   = (s32)((result.red   >> 6) + (result.red   >> 1));
                    pPixel[x * ui32WordWidth + y].green = (s32)((result.green >> 6) + (result.green >> 1));
                    pPixel[x * ui32WordWidth + y].blue  = (s32)((result.blue  >> 6) + (result.blue  >> 1));
                    pPixel[x * ui32WordWidth + y].alpha = (s32)((result.alpha >> 4) + (result.alpha));

                    result.red   += dY.red;
                    result.green += dY.green;
                    result.blue  += dY.blue;
                    result.alpha += dY.alpha;
                }

                hP.red   += QminusP.red;
                hP.green += QminusP.green;
                hP.blue  += QminusP.blue;
                hP.alpha += QminusP.alpha;

                hR.red   += SminusR.red;
                hR.green += SminusR.green;
                hR.blue  += SminusR.blue;
                hR.alpha += SminusR.alpha;
            }
        }
    }

    static void unpackModulations(const PVRTCWord& word, int offsetX, int offsetY, u8 i32ModulationValues[8][16], u8 ui8Bpp)
    {
        u32 WordModMode = word.u32ColorData & 0x1;
        u32 ModulationBits = word.u32ModulationData;

        const u8 modulation_table[] =
        {
            0, 3, 5, 8,
            0, 4, 4 | PUNCHTHROUGH_ALPHA, 8
        };

        // Unpack differently depending on 2bpp or 4bpp modes.
        if (ui8Bpp == 2)
        {
            // WordModeMode: 0 = store, 1 = HV, 2 = H, 3 = V
            if (WordModMode)
            {
                if (ModulationBits & 0x1)
                {
                    WordModMode += ((ModulationBits >> 20) & 1) + 1;
                    ModulationBits = (ModulationBits & ~0x00100000) | ((ModulationBits & 0x00200000) >> 1);
                }

                ModulationBits = (ModulationBits & ~0x00000001) | ((ModulationBits & 0x00000002) >> 1);

                // Store mode in 2 MSB
                WordModMode <<= 6;

                for (int y = 0; y < 4; y++)
                {
                    u8* dest = &i32ModulationValues[y + offsetY][0 + offsetX];
                    const int s = y & 1;
                    for (int x = 0; x < 4; x++)
                    {
                        dest[1-s] = WordModMode;
                        dest[0+s] = modulation_table[ModulationBits & 3];
                        dest += 2;
                        ModulationBits >>= 2;
                    }
                }
            }
            else
            {
                // else if direct encoded 2bit mode - i.e. 1 mode bit per pixel
                for (int y = 0; y < 4; y++)
                {
                    u8* dest = &i32ModulationValues[y + offsetY][0 + offsetX];
                    for (int x = 0; x < 8; x++)
                    {
                        dest[x] = (ModulationBits & 1) * 8;
                        ModulationBits >>= 1;
                    }
                }
            }
        }
        else
        {
            const u8* table = modulation_table + WordModMode * 4;
            for (int y = 0; y < 4; y++)
            {
                u8* dest = &i32ModulationValues[y + offsetY][0 + offsetX];
                for (int x = 0; x < 4; x++)
                {
                    dest[x] = table[ModulationBits & 3];
                    ModulationBits >>= 2;
                }
            }
        }
    }

    static s32 getModulationValues(u8 i32ModulationValues[8][16], u32 xPos, u32 yPos, u8 ui8Bpp)
    {
        int value = i32ModulationValues[yPos][xPos];
        if (ui8Bpp == 2)
        {
            const int mode = value >> 6;
            if (mode == 0)
            {
                value &= 0x0f;
            }
            else
            {
                if (((xPos ^ yPos) & 1) == 0)
                {
                    // if this is a stored value
                    value &= 0x0f;
                }
                else if (mode == 3)
                {
                    // else it's V-Only
                    value = (i32ModulationValues[yPos - 1][xPos] +
                             i32ModulationValues[yPos + 1][xPos] + 1) / 2;
                }
                else if (mode == 2)
                {
                    // else if H-Only
                    value = (i32ModulationValues[yPos][xPos - 1] +
                             i32ModulationValues[yPos][xPos + 1] + 1) / 2;
                }
                else
                {
                    // if H&V interpolation...
                    value = (i32ModulationValues[yPos - 1][xPos] +
                             i32ModulationValues[yPos + 1][xPos] +
                             i32ModulationValues[yPos][xPos - 1] +
                             i32ModulationValues[yPos][xPos + 1] + 2) / 4;
                }
            }
        }

        return value;
    }

    constexpr int lerp(int a, int b, int mod)
    {
        return a + ((b - a) * mod) / 8;
    }

    static void pvrtcGetDecompressedPixels(u8 i32ModulationValues[8][16],
                                           Pixel128S upscaledColorA[32],
                                           Pixel128S upscaledColorB[32],
                                           u8* pColorData, int stride,
                                           int xoffset, int yoffset, int width, int height,
                                           u8 ui8Bpp)
    {
        const u32 ui32WordWidth = (ui8Bpp == 2) ? 8 : 4;
        const u32 ui32WordHeight = 4;

        xoffset = xoffset * ui32WordWidth - ui32WordWidth / 2;
        yoffset = yoffset * ui32WordHeight - ui32WordHeight / 2;
        const int xmask = width - 1;
        const int ymask = height - 1;

        for (unsigned int y = 0; y < ui32WordHeight; y++)
        {
            const Pixel128S* colorA = upscaledColorA + y * ui32WordWidth;
            const Pixel128S* colorB = upscaledColorB + y * ui32WordWidth;

            const int sy = ((yoffset + y) & ymask) * stride;
            Pixel32* dest = reinterpret_cast<Pixel32 *>(pColorData + sy);

            for (unsigned int x = 0; x < ui32WordWidth; x++)
            {
                s32 mod = getModulationValues(i32ModulationValues, x + ui32WordWidth / 2, y + ui32WordHeight / 2, ui8Bpp);
                mod &= 0xf;

                Pixel128S result;
                result.red   = lerp(colorA[x].red,   colorB[x].red,   mod);
                result.green = lerp(colorA[x].green, colorB[x].green, mod);
                result.blue  = lerp(colorA[x].blue,  colorB[x].blue,  mod);
                result.alpha = mod & PUNCHTHROUGH_ALPHA ? 0 : lerp(colorA[x].alpha, colorB[x].alpha, mod);

                const int offset = (xoffset + x) & xmask;
                dest[offset].red   = (u8)result.red;
                dest[offset].green = (u8)result.green;
                dest[offset].blue  = (u8)result.blue;
                dest[offset].alpha = (u8)result.alpha;
            }
        }
    }

    constexpr unsigned int wrapWordIndex(unsigned int numWords, int word)
    {
        //return ((word + numWords) % numWords);
        return word & (numWords - 1); // numWords must be power of two
    }

    static void moveModulationValues(u8 i32ModulationValues[8][16], u32 ui32WordWidth, u8 ui8bpp)
    {
        u32* d = (u32*) &i32ModulationValues[0][0];
        u32* s = (u32*) &i32ModulationValues[0][ui32WordWidth];
        for (int i = 0; i < 8; ++i)
        {
            d[0] = s[0];
            if (ui8bpp)
                d[1] = s[1];
            d += 4;
            s += 4;
        }
    }

    static void pvrtc_decompress(const u8* pCompressedData,
                               u8* pDecompressedData,
                               int stride,
                               u32 ui32Width,
                               u32 ui32Height,
                               u8 ui8Bpp)
    {
        const u32 ui32WordWidth = (ui8Bpp == 2) ? 8 : 4;
        const u32 ui32WordHeight = 4;

        PVRTCWord* pWordMembers = (PVRTCWord*)pCompressedData;

        // Calculate number of words
        int i32NumXWords = (int)(ui32Width / ui32WordWidth);
        int i32NumYWords = (int)(ui32Height / ui32WordHeight);

        // For each row of words
        for (int wordY = 0; wordY < i32NumYWords; wordY++)
        {
            int x0 = i32NumXWords - 1;
            int x1 = 0;
            int y0 = wrapWordIndex(i32NumYWords, wordY - 1);
            int y1 = wrapWordIndex(i32NumYWords, wordY);

            PVRTCWord* P = pWordMembers + u32_interleave_bits(y0, x0);
            PVRTCWord* Q = pWordMembers + u32_interleave_bits(y0, x1);
            PVRTCWord* R = pWordMembers + u32_interleave_bits(y1, x0);
            PVRTCWord* S = pWordMembers + u32_interleave_bits(y1, x1);

            u8 i32ModulationValues[8][16];

            unpackModulations(*P, 0, 0,                          i32ModulationValues, ui8Bpp);
            unpackModulations(*Q, ui32WordWidth, 0,              i32ModulationValues, ui8Bpp);
            unpackModulations(*R, 0, ui32WordHeight,             i32ModulationValues, ui8Bpp);
            unpackModulations(*S, ui32WordWidth, ui32WordHeight, i32ModulationValues, ui8Bpp);

            Pixel32 colorA[4];
            Pixel32 colorB[4];

            colorA[0] = getColorA(P->u32ColorData);
            colorA[1] = getColorA(Q->u32ColorData);
            colorA[2] = getColorA(R->u32ColorData);
            colorA[3] = getColorA(S->u32ColorData);
            colorB[0] = getColorB(P->u32ColorData);
            colorB[1] = getColorB(Q->u32ColorData);
            colorB[2] = getColorB(R->u32ColorData);
            colorB[3] = getColorB(S->u32ColorData);

            // for each column of words
            for (int wordX = 0; wordX < i32NumXWords; wordX++)
            {
                Pixel128S upscaledColorA[32];
                Pixel128S upscaledColorB[32];
 
                // Bilinear upscale image data from 2x2 -> 4x4
                interpolateColors(colorA, upscaledColorA, ui8Bpp);
                interpolateColors(colorB, upscaledColorB, ui8Bpp);
 
                // assemble 4 words into struct to get decompressed pixels from
                pvrtcGetDecompressedPixels(i32ModulationValues, upscaledColorA, upscaledColorB,
                                           pDecompressedData, stride, wordX, wordY, ui32Width, ui32Height, ui8Bpp);
 
                x1 = wrapWordIndex(i32NumXWords, wordX + 1);
 
                P = Q;
                R = S;
                Q = pWordMembers + u32_interleave_bits(y0, x1);
                S = pWordMembers + u32_interleave_bits(y1, x1);

                moveModulationValues(i32ModulationValues, ui32WordWidth, ui8Bpp);
                unpackModulations(*Q, ui32WordWidth, 0,              i32ModulationValues, ui8Bpp);
                unpackModulations(*S, ui32WordWidth, ui32WordHeight, i32ModulationValues, ui8Bpp);

                colorA[0] = colorA[1];
                colorA[1] = getColorA(Q->u32ColorData);
                colorA[2] = colorA[3];
                colorA[3] = getColorA(S->u32ColorData);

                colorB[0] = colorB[1];
                colorB[1] = getColorB(Q->u32ColorData);
                colorB[2] = colorB[3];
                colorB[3] = getColorB(S->u32ColorData);
            }
        }
    }

	// ----------------------------------------------------------------------------
    // pvrtc2
    // ----------------------------------------------------------------------------

    // https://s3.amazonaws.com/pvr-sdk-live/sdk-documentation/PVRTC%20Specification%20and%20User%20Guide.pdf
    // http://sv-journal.org/2014-1/06/en/index.php?lang=en#7-3

    constexpr u32 pvrtc2_extend(u32 value, int from, int to)
    {
        return value * ((1 << to) - 1) / ((1 << from) - 1);
    }

    constexpr u32 pvrtc2_alpha0(u32 alpha)
    {
        alpha = (alpha << 1) | 0;
        return (alpha << 4) | alpha;
    }

    constexpr u32 pvrtc2_alpha1(u32 alpha)
    {
        alpha = (alpha << 1) | 1;
        return (alpha << 4) | alpha;
    }

    static inline
    ColorRGBA pvrtc2_lerp(ColorRGBA a, ColorRGBA b, int mod)
    {
        // TODO: specification
        // mod(0): a
        // mod(1): (a*5 + b*3) / 8
        // mod(2): (a*3 + b*5) / 8
        // mod(3): b
        ColorRGBA c;
        switch (mod)
        {
            case 0:
                c = a;
                break;
            case 1:
                c.r = (a.r * 5 + b.r * 3) / 8;
                c.g = (a.g * 5 + b.g * 3) / 8;
                c.b = (a.b * 5 + b.b * 3) / 8;
                c.a = (a.a * 5 + b.a * 3) / 8;
                break;
            case 2:
                c.r = (a.r * 3 + b.r * 5) / 8;
                c.g = (a.g * 3 + b.g * 5) / 8;
                c.b = (a.b * 3 + b.b * 5) / 8;
                c.a = (a.a * 3 + b.a * 5) / 8;
                break;
            case 3:
                c = b;
                break;
        }
        return c;
    }

    static inline
    ColorRGBA pvrtc2_punch(ColorRGBA a, ColorRGBA b, int mod)
    {
        ColorRGBA c;
        switch (mod)
        {
            case 0:
                c = a;
                break;
            case 1:
                c.r = (a.r + b.r) / 2;
                c.g = (a.g + b.g) / 2;
                c.b = (a.b + b.b) / 2;
                c.a = (a.a + b.a) / 2;
                break;
            case 2:
                // punch-through (TODO: should this be a, b, or 0?)
                c = ColorRGBA(0, 0, 0, 0);
                //c.r = (a.r + b.r) / 2;
                //c.g = (a.g + b.g) / 2;
                //c.b = (a.b + b.b) / 2;
                //c.a = 0;
                break;
            case 3:
                c = b;
                break;
        }
        return c;
    }

    struct BlockPVRTC2
    {
        ColorRGBA a;
        ColorRGBA b;

        // --------------------------------------------
        // hard   mode    decoder
        // --------------------------------------------
        //   0      0     bilinear
        //   0      1     punch-through alpha
        //   1      0     non-interpolated
        //   1      1     local palette

        u32 mode;
        u32 hard;
        u32 modulation;

        BlockPVRTC2(const u8* data)
        {
            modulation = uload32le(data + 0);
            u32 packed = uload32le(data + 4);

            mode = packed & 0x0001;
            hard = packed & 0x8000;

            u32 opacity = packed & 0x80000000;

            if (opacity)
            {
                a.b = pvrtc2_extend((packed >>  1) & 0x0f, 4, 8);
                a.g = pvrtc2_extend((packed >>  5) & 0x1f, 5, 8);
                a.r = pvrtc2_extend((packed >> 10) & 0x1f, 5, 8);
                a.a = 0xff;

                b.b = pvrtc2_extend((packed >> 16) & 0x1f, 5, 8);
                b.g = pvrtc2_extend((packed >> 21) & 0x1f, 5, 8);
                b.r = pvrtc2_extend((packed >> 26) & 0x1f, 5, 8);
                b.a = 0xff;
            }
            else
            {
                a.b = pvrtc2_extend((packed >>  1) & 0x07, 3, 8);
                a.g = pvrtc2_extend((packed >>  4) & 0x0f, 4, 8);
                a.r = pvrtc2_extend((packed >>  8) & 0x0f, 4, 8);
                //a.a = pvrtc2_alpha1((packed >> 12) & 0x07);
                a.a = pvrtc2_extend((packed >> 12) & 0x07, 3, 8);

                b.b = pvrtc2_extend((packed >> 16) & 0xf, 4, 8);
                b.g = pvrtc2_extend((packed >> 20) & 0xf, 4, 8);
                b.r = pvrtc2_extend((packed >> 24) & 0xf, 4, 8);
                //b.a = pvrtc2_alpha0((packed >> 28) & 0x7);
                b.a = pvrtc2_extend((packed >> 28) & 0x7, 3, 8);
            }
        }
    };

    void pvrtc2_quad_debug(u8* image, int stride, ColorRGBA color)
    {
        u32* scan0 = reinterpret_cast<u32*>(image + stride * 0);
        u32* scan1 = reinterpret_cast<u32*>(image + stride * 1);
        scan0[0] = color;
        scan0[1] = color;
        scan1[0] = color;
        scan1[1] = color;
    }

    void pvrtc2_quad_nearest(u8* image, int stride, ColorRGBA a, ColorRGBA b, u32 modulation)
    {
        u32* scan0 = reinterpret_cast<u32*>(image + stride * 0);
        u32* scan1 = reinterpret_cast<u32*>(image + stride * 1);
        scan0[0] = pvrtc2_lerp(a, b, (modulation >>  0) & 3);
        scan0[1] = pvrtc2_lerp(a, b, (modulation >>  2) & 3);
        scan1[0] = pvrtc2_lerp(a, b, (modulation >>  8) & 3);
        scan1[1] = pvrtc2_lerp(a, b, (modulation >> 10) & 3);
    }

    void pvrtc2_quad_palette(u8* image, int stride, const ColorRGBA* palette, u32 modulation)
    {
        u32* scan0 = reinterpret_cast<u32*>(image + stride * 0);
        u32* scan1 = reinterpret_cast<u32*>(image + stride * 1);
        scan0[0] = palette[((modulation >>  0) & 3) + 0];
        scan0[1] = palette[((modulation >>  2) & 3) + 4];
        scan1[0] = palette[((modulation >>  8) & 3) + 8];
        scan1[1] = palette[((modulation >> 10) & 3) + 12];
    }

    void pvrtc2_quad_bilinear(u8* image, int stride, int u0, int v0,
        const BlockPVRTC2& block0,
        const BlockPVRTC2& block1,
        const BlockPVRTC2& block2,
        const BlockPVRTC2& block3,
        u32 modulation)
    {
        ColorRGBA a[4];
        ColorRGBA b[4];

        for (int y = 0; y < 2; ++y)
        {
            for (int x = 0; x < 2; ++x)
            {
                constexpr int N = 16;
                constexpr int S = 4;

                int u = u0 + x;
                int v = v0 + y;
                int w0 = (S - u) * (S - v);
                int w1 = u * (S - v);
                int w2 = (S - u) * v;
                int w3 = u * v;

                a[y * 2 + x].r = (block0.a.r * w0 + block1.a.r * w1 + block2.a.r * w2 + block3.a.r * w3) / N;
                a[y * 2 + x].g = (block0.a.g * w0 + block1.a.g * w1 + block2.a.g * w2 + block3.a.g * w3) / N;
                a[y * 2 + x].b = (block0.a.b * w0 + block1.a.b * w1 + block2.a.b * w2 + block3.a.b * w3) / N;
                a[y * 2 + x].a = (block0.a.a * w0 + block1.a.a * w1 + block2.a.a * w2 + block3.a.a * w3) / N;

                b[y * 2 + x].r = (block0.b.r * w0 + block1.b.r * w1 + block2.b.r * w2 + block3.b.r * w3) / N;
                b[y * 2 + x].g = (block0.b.g * w0 + block1.b.g * w1 + block2.b.g * w2 + block3.b.g * w3) / N;
                b[y * 2 + x].b = (block0.b.b * w0 + block1.b.b * w1 + block2.b.b * w2 + block3.b.b * w3) / N;
                b[y * 2 + x].a = (block0.b.a * w0 + block1.b.a * w1 + block2.b.a * w2 + block3.b.a * w3) / N;
            }
        }

        u32* scan0 = reinterpret_cast<u32*>(image + stride * 0);
        u32* scan1 = reinterpret_cast<u32*>(image + stride * 1);
        scan0[0] = pvrtc2_lerp(a[0], b[0], (modulation >>  0) & 3);
        scan0[1] = pvrtc2_lerp(a[1], b[1], (modulation >>  2) & 3);
        scan1[0] = pvrtc2_lerp(a[2], b[2], (modulation >>  8) & 3);
        scan1[1] = pvrtc2_lerp(a[3], b[3], (modulation >> 10) & 3);
    }

    void pvrtc2_quad_punchthrough(u8* image, int stride, int u0, int v0,
        const BlockPVRTC2& block0,
        const BlockPVRTC2& block1,
        const BlockPVRTC2& block2,
        const BlockPVRTC2& block3,
        u32 modulation)
    {
        ColorRGBA a[4];
        ColorRGBA b[4];

        for (int y = 0; y < 2; ++y)
        {
            for (int x = 0; x < 2; ++x)
            {
                constexpr int N = 16;
                constexpr int S = 4;

                int u = u0 + x;
                int v = v0 + y;
                int w0 = (S - u) * (S - v);
                int w1 = u * (S - v);
                int w2 = (S - u) * v;
                int w3 = u * v;

                a[y * 2 + x].r = (block0.a.r * w0 + block1.a.r * w1 + block2.a.r * w2 + block3.a.r * w3) / N;
                a[y * 2 + x].g = (block0.a.g * w0 + block1.a.g * w1 + block2.a.g * w2 + block3.a.g * w3) / N;
                a[y * 2 + x].b = (block0.a.b * w0 + block1.a.b * w1 + block2.a.b * w2 + block3.a.b * w3) / N;
                a[y * 2 + x].a = (block0.a.a * w0 + block1.a.a * w1 + block2.a.a * w2 + block3.a.a * w3) / N;

                b[y * 2 + x].r = (block0.b.r * w0 + block1.b.r * w1 + block2.b.r * w2 + block3.b.r * w3) / N;
                b[y * 2 + x].g = (block0.b.g * w0 + block1.b.g * w1 + block2.b.g * w2 + block3.b.g * w3) / N;
                b[y * 2 + x].b = (block0.b.b * w0 + block1.b.b * w1 + block2.b.b * w2 + block3.b.b * w3) / N;
                b[y * 2 + x].a = (block0.b.a * w0 + block1.b.a * w1 + block2.b.a * w2 + block3.b.a * w3) / N;
            }
        }

        u32* scan0 = reinterpret_cast<u32*>(image + stride * 0);
        u32* scan1 = reinterpret_cast<u32*>(image + stride * 1);
        scan0[0] = pvrtc2_punch(a[0], b[0], (modulation >>  0) & 3);
        scan0[1] = pvrtc2_punch(a[1], b[1], (modulation >>  2) & 3);
        scan1[0] = pvrtc2_punch(a[2], b[2], (modulation >>  8) & 3);
        scan1[1] = pvrtc2_punch(a[3], b[3], (modulation >> 10) & 3);
    }

    void pvrtc2_decompress(const u8* data,
                           u8* image,
                           int stride,
                           u32 width,
                           u32 height,
                           u8 bpp)
    {
        const u32 block_width = bpp == 2 ? 8 : 4;
        const u32 block_height = 4;
        const u32 xblocks = ceil_div(width, block_width);
        const u32 yblocks = ceil_div(height, block_height);

        for (int y0 = 0; y0 < yblocks; ++y0)
        {
            for (int x0 = 0; x0 < xblocks; ++x0)
            {
                int x1 = (x0 + 1) % xblocks;
                int y1 = (y0 + 1) % yblocks;
                const u8* p0 = data + (y0 * xblocks + x0) * 8;
                const u8* p1 = data + (y0 * xblocks + x1) * 8;
                const u8* p2 = data + (y1 * xblocks + x0) * 8;
                const u8* p3 = data + (y1 * xblocks + x1) * 8;

                BlockPVRTC2 block0(p0); // P block
                BlockPVRTC2 block1(p1); // Q block
                BlockPVRTC2 block2(p2); // R block
                BlockPVRTC2 block3(p3); // S block

                u8* scan0 = image + ((y0 * block_height + 2) % height) * stride + ((x0 * block_width + 2) % width) * 4;
                u8* scan1 = image + ((y0 * block_height + 2) % height) * stride + ((x0 * block_width + 4) % width) * 4;
                u8* scan2 = image + ((y0 * block_height + 4) % height) * stride + ((x0 * block_width + 2) % width) * 4;
                u8* scan3 = image + ((y0 * block_height + 4) % height) * stride + ((x0 * block_width + 4) % width) * 4;

                if (!block0.hard)
                {
                    if (!block0.mode)
                    {
#if 1
                        ColorRGBA color(0, 255, 0, 255);
                        pvrtc2_quad_debug(scan0, stride, color);
                        pvrtc2_quad_debug(scan1, stride, color);
                        pvrtc2_quad_debug(scan2, stride, color);
                        pvrtc2_quad_debug(scan3, stride, color);
#else
                        // status: ?
                        pvrtc2_quad_bilinear(scan0, stride, 0, 0, block0, block1, block2, block3, block0.modulation >> 20);
                        pvrtc2_quad_bilinear(scan1, stride, 2, 0, block0, block1, block2, block3, block1.modulation >> 16);
                        pvrtc2_quad_bilinear(scan2, stride, 0, 2, block0, block1, block2, block3, block2.modulation >> 4);
                        pvrtc2_quad_bilinear(scan3, stride, 2, 2, block0, block1, block2, block3, block3.modulation >> 0);
#endif
                    }
                    else
                    {
#if 1
                        ColorRGBA color(0, 0, 255, 255);
                        pvrtc2_quad_debug(scan0, stride, color);
                        pvrtc2_quad_debug(scan1, stride, color);
                        pvrtc2_quad_debug(scan2, stride, color);
                        pvrtc2_quad_debug(scan3, stride, color);
#else
                        // status: ?
                        pvrtc2_quad_punchthrough(scan0, stride, 0, 0, block0, block1, block2, block3, block0.modulation >> 20);
                        pvrtc2_quad_punchthrough(scan1, stride, 2, 0, block0, block1, block2, block3, block1.modulation >> 16);
                        pvrtc2_quad_punchthrough(scan2, stride, 0, 2, block0, block1, block2, block3, block2.modulation >> 4);
                        pvrtc2_quad_punchthrough(scan3, stride, 2, 2, block0, block1, block2, block3, block3.modulation >> 0);
#endif
                    }
                }
                else
                {
                    if (!block0.mode)
                    {
#if 1
                        ColorRGBA color(255, 255, 255, 255);
                        pvrtc2_quad_debug(scan0, stride, color);
                        pvrtc2_quad_debug(scan1, stride, color);
                        pvrtc2_quad_debug(scan2, stride, color);
                        pvrtc2_quad_debug(scan3, stride, color);
#else
                        // status: ?
                        pvrtc2_quad_nearest(scan0, stride, block0.a, block0.b, block0.modulation >> 20);
                        pvrtc2_quad_nearest(scan1, stride, block1.a, block1.b, block1.modulation >> 16);
                        pvrtc2_quad_nearest(scan2, stride, block2.a, block2.b, block2.modulation >> 4);
                        pvrtc2_quad_nearest(scan3, stride, block3.a, block3.b, block3.modulation >> 0);
#endif
                    }
                    else
                    {
#if 0
                        ColorRGBA color(255, 255, 0, 255);
                        pvrtc2_quad_debug(scan0, stride, color);
                        pvrtc2_quad_debug(scan1, stride, color);
                        pvrtc2_quad_debug(scan2, stride, color);
                        pvrtc2_quad_debug(scan3, stride, color);
#else
                        ColorRGBA palette[64];

                        // P

                        palette[ 0] = block0.a;
                        palette[ 1] = (block0.a * 5 + block0.b * 3) / 8;
                        palette[ 2] = (block0.a * 3 + block0.b * 5) / 8;
                        palette[ 3] = block0.b;

                        palette[ 4] = block0.a;
                        palette[ 5] = block0.b;
                        palette[ 6] = block1.a;
                        palette[ 7] = block1.b;

                        palette[ 8] = block0.a;
                        palette[ 9] = block0.b;
                        palette[10] = block2.a;
                        palette[11] = block2.b;

                        palette[12] = block0.a;
                        palette[13] = block0.b;
                        palette[14] = block1.a;
                        palette[15] = block2.b;

                        // Q

                        palette[16] = block0.a;
                        palette[17] = block0.b;
                        palette[18] = block1.a;
                        palette[19] = block1.b;

                        palette[20] = block0.a;
                        palette[21] = block0.b;
                        palette[22] = block1.a;
                        palette[23] = block1.b;

                        palette[24] = block0.a;
                        palette[25] = block0.b;
                        palette[26] = block1.a;
                        palette[27] = block1.b;

                        palette[28] = block3.a;
                        palette[29] = block0.b;
                        palette[30] = block1.a;
                        palette[31] = block1.b;

                        // R

                        palette[32] = block0.a;
                        palette[33] = block0.b;
                        palette[34] = block2.a;
                        palette[35] = block2.b;

                        palette[36] = block0.a;
                        palette[37] = block0.b;
                        palette[38] = block2.a;
                        palette[39] = block2.b;

                        palette[40] = block0.a;
                        palette[41] = block0.b;
                        palette[42] = block2.a;
                        palette[43] = block2.b;

                        palette[44] = block0.a;
                        palette[45] = block3.b;
                        palette[46] = block2.a;
                        palette[47] = block2.b;

                        // S

                        palette[48] = block0.a;
                        palette[49] = block3.b;
                        palette[50] = block2.a;
                        palette[51] = block1.b;

                        palette[52] = block3.a;
                        palette[53] = block3.b;
                        palette[54] = block1.a;
                        palette[55] = block1.b;

                        palette[56] = block3.a;
                        palette[57] = block3.b;
                        palette[58] = block2.a;
                        palette[59] = block2.b;

                        palette[60] = block3.a;
                        palette[61] = block3.b;
                        palette[62] = block2.a;
                        palette[63] = block1.b;

                        for (int i = 4; i < 64; i += 4)
                        {
                            //std::swap(palette[i + 0], palette[i + 1]);
                            //std::swap(palette[i + 2], palette[i + 3]);
                        }

                        // status: ?
                        pvrtc2_quad_palette(scan0, stride, palette + 0 * 16, block0.modulation >> 20);
                        pvrtc2_quad_palette(scan1, stride, palette + 1 * 16, block1.modulation >> 16);
                        pvrtc2_quad_palette(scan2, stride, palette + 2 * 16, block2.modulation >> 4);
                        pvrtc2_quad_palette(scan3, stride, palette + 3 * 16, block3.modulation >> 0);
#endif
                    }
                }
            }
        }
    }

} // namespace

namespace mango
{

    void decode_block_pvrtc(const TextureCompressionInfo& info, u8* out, const u8* in, int stride)
    {
        switch (info.compression)
        {
            case TextureCompression::PVRTC_RGB_2BPP:
            case TextureCompression::PVRTC_RGBA_2BPP:
            case TextureCompression::PVRTC_SRGB_2BPP:
            case TextureCompression::PVRTC_SRGB_ALPHA_2BPP:
                pvrtc_decompress(in, out, stride, info.width, info.height, 2);
                break;

            case TextureCompression::PVRTC_RGB_4BPP:
            case TextureCompression::PVRTC_RGBA_4BPP:
            case TextureCompression::PVRTC_SRGB_4BPP:
            case TextureCompression::PVRTC_SRGB_ALPHA_4BPP:
                pvrtc_decompress(in, out, stride, info.width, info.height, 4);
                break;

            case TextureCompression::PVRTC2_RGBA_2BPP:
                pvrtc2_decompress(in, out, stride, info.width, info.height, 2);
                break;

            case TextureCompression::PVRTC2_RGBA_4BPP:
                pvrtc2_decompress(in, out, stride, info.width, info.height, 4);
                break;

            default:
                // incorrect compression
                break;
        }
    }

} // namespace mango
