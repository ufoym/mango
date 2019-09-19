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
        ColorRGBA c;
        c.r = a.r + ((b.r - a.r) * mod) / 3;
        c.g = a.g + ((b.g - a.g) * mod) / 3;
        c.b = a.b + ((b.b - a.b) * mod) / 3;
        c.a = a.a + ((b.a - a.a) * mod) / 3;
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
                a.b = pvrtc2_extend((packed >> 16) & 0x1f, 5, 8);
                a.g = pvrtc2_extend((packed >> 21) & 0x1f, 5, 8);
                a.r = pvrtc2_extend((packed >> 26) & 0x1f, 5, 8);
                a.a = 0xff;

                b.b = pvrtc2_extend((packed >>  1) & 0x0f, 4, 8);
                b.g = pvrtc2_extend((packed >>  5) & 0x1f, 5, 8);
                b.r = pvrtc2_extend((packed >> 10) & 0x1f, 5, 8);
                b.a = 0xff;
            }
            else
            {
                a.b = pvrtc2_extend((packed >> 16) & 0xf, 4, 8);
                a.g = pvrtc2_extend((packed >> 20) & 0xf, 4, 8);
                a.r = pvrtc2_extend((packed >> 24) & 0xf, 4, 8);
                a.a = pvrtc2_alpha0((packed >> 28) & 0x7);

                b.b = pvrtc2_extend((packed >>  1) & 0x07, 3, 8);
                b.g = pvrtc2_extend((packed >>  4) & 0x0f, 4, 8);
                b.r = pvrtc2_extend((packed >>  8) & 0x0f, 4, 8);
                b.a = pvrtc2_alpha1((packed >> 12) & 0x07);
            }
        }
    };

    static void pvrtc2_decompress(const u8* data,
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

        // TODO: handle edge blocks (see yblocks - 1, xblocks - 1)
        image += stride * 2 + 2 * 4;

        for (int y0 = 0; y0 < yblocks - 1; ++y0)
        {
            for (int x0 = 0; x0 < xblocks - 1; ++x0)
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

                ColorRGBA acolor[4];

                acolor[0] = block0.a;
                acolor[1] = block1.a;
                acolor[2] = block2.a;
                acolor[3] = block3.a;

                ColorRGBA bcolor[4];

                bcolor[0] = block0.b;
                bcolor[1] = block1.b;
                bcolor[2] = block2.b;
                bcolor[3] = block3.b;

                u32 modulation = block0.modulation;

                for (int y = 0; y < block_height; ++y)
                {
                    u32* scan = reinterpret_cast<u32*>(image + (y0 * block_height + y) * stride + (x0 * block_width * 4));

                    for (int x = 0; x < block_width; ++x)
                    {
                        int mod = modulation & 3;
                        modulation >>= 2;

                        int offset = (y & 2) + (x >> 1);
                        //offset = 0;
                        ColorRGBA a = acolor[offset];
                        ColorRGBA b = bcolor[offset];
                        ColorRGBA c = pvrtc2_lerp(b, a, mod);
                        scan[x] = c;
                    }
                }

                /*
                u32 modulation = block0.modulation;
                u8* block_image = image + x0 * block_width * 4;

                ColorRGBA a = block0.a;
                ColorRGBA b = block0.b;

                for (int y = 0; y < block_height; ++y)
                {
                    u32* scan = reinterpret_cast<u32*>(block_image + y * stride);

                    for (int x = 0; x < block_width; ++x)
                    {
                        int mod = modulation & 3;
                        modulation >>= 2;
#if 1
                        if (block0.isBilinear())
                        {
                            const int m = 3;//bpp == 2 ? 7 : 3;
                            const int s = m * 3;

                            int w0 = (m - x) * (3 - y);
                            int w1 = x * (3 - y);
                            int w2 = (m - x) * y;
                            int w3 = x * y;

                            int r0 = (block0.a.r * w0 + block1.a.r * w1 + block2.a.r * w2 + block3.a.r * w3) / s;
                            int g0 = (block0.a.g * w0 + block1.a.g * w1 + block2.a.g * w2 + block3.a.g * w3) / s;
                            int b0 = (block0.a.b * w0 + block1.a.b * w1 + block2.a.b * w2 + block3.a.b * w3) / s;
                            int a0 = (block0.a.a * w0 + block1.a.a * w1 + block2.a.a * w2 + block3.a.a * w3) / s;
                            a.r = r0;
                            a.g = g0;
                            a.b = b0;
                            a.a = a0;

                            int r1 = (block0.b.r * w0 + block1.b.r * w1 + block2.b.r * w2 + block3.b.r * w3) / s;
                            int g1 = (block0.b.g * w0 + block1.b.g * w1 + block2.b.g * w2 + block3.b.g * w3) / s;
                            int b1 = (block0.b.b * w0 + block1.b.b * w1 + block2.b.b * w2 + block3.b.b * w3) / s;
                            int a1 = (block0.b.a * w0 + block1.b.a * w1 + block2.b.a * w2 + block3.b.a * w3) / s;
                            b.r = r1;
                            b.g = g1;
                            b.b = b1;
                            b.a = a1;
                        }
#endif
                        ColorRGBA c;
                        c. r = pvrtc2_lerp(b.r, a.r, mod);
                        c. g = pvrtc2_lerp(b.g, a.g, mod);
                        c. b = pvrtc2_lerp(b.b, a.b, mod);
                        c. a = pvrtc2_lerp(b.a, a.a, mod);

                        if (mod == 2 && block0.isPunchthrough())
                        {
                            //c.a = 0;
                        }

#if 1
                        switch (block0.mode)
                        {
                            case BlockPVRTC2::BILINEAR:
                                // GREEN
                                //c = ColorRGBA(0, 255, 0, 255);
                                break;
                            case BlockPVRTC2::PUNCHTHROUGH_ALPHA:
                                // BLUE
                                //c = ColorRGBA(0, 0, 255, 255);
                                break;
                            case BlockPVRTC2::NON_INTERPOLATED:
                                // WHITE
                                //c = ColorRGBA(255, 255, 255, 255);
                                break;
                            case BlockPVRTC2::LOCAL_PALETTE:
                                // YELLOW
                                c = ColorRGBA(255, 255, 0, 255);
                                break;
                        }
#endif

                        scan[x] = c;
                    }
                }
                */
            }

            //image += stride * block_height;
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
