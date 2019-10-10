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

#if 0 // TODO: should use these (specification conforming)
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
#endif

    static inline
    ColorRGBA pvrtc2_lerp2(ColorRGBA a, ColorRGBA b, int mod)
    {
        return mod ? b : a;
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

        BlockPVRTC2(const BlockPVRTC2& block)
        {
            a = block.a;
            b = block.b;
            mode = block.mode;
            hard = block.hard;
            modulation = block.modulation;
        }
    };

    void pvrtc2_2bit_bilinear(ColorRGBA* a, ColorRGBA* b, int u0, int v0, const BlockPVRTC2* blocks)
    {
        for (int y = 0; y < 2; ++y)
        {
            for (int x = 0; x < 4; ++x)
            {
                constexpr int U = 8;
                constexpr int V = 4;
                constexpr int N = 32;

                int u = u0 + x;
                int v = v0 + y;
                int w0 = (U - u) * (V - v);
                int w1 = u * (V - v);
                int w2 = (U - u) * v;
                int w3 = u * v;

                a->r = (blocks[0].a.r * w0 + blocks[1].a.r * w1 + blocks[2].a.r * w2 + blocks[3].a.r * w3) / N;
                a->g = (blocks[0].a.g * w0 + blocks[1].a.g * w1 + blocks[2].a.g * w2 + blocks[3].a.g * w3) / N;
                a->b = (blocks[0].a.b * w0 + blocks[1].a.b * w1 + blocks[2].a.b * w2 + blocks[3].a.b * w3) / N;
                a->a = (blocks[0].a.a * w0 + blocks[1].a.a * w1 + blocks[2].a.a * w2 + blocks[3].a.a * w3) / N;
                ++a;

                b->r = (blocks[0].b.r * w0 + blocks[1].b.r * w1 + blocks[2].b.r * w2 + blocks[3].b.r * w3) / N;
                b->g = (blocks[0].b.g * w0 + blocks[1].b.g * w1 + blocks[2].b.g * w2 + blocks[3].b.g * w3) / N;
                b->b = (blocks[0].b.b * w0 + blocks[1].b.b * w1 + blocks[2].b.b * w2 + blocks[3].b.b * w3) / N;
                b->a = (blocks[0].b.a * w0 + blocks[1].b.a * w1 + blocks[2].b.a * w2 + blocks[3].b.a * w3) / N;
                ++b;
            }
        }
    }

    void pvrtc2_4bit_bilinear(ColorRGBA* a, ColorRGBA* b, int u0, int v0, const BlockPVRTC2* blocks)
    {
        for (int y = 0; y < 2; ++y)
        {
            for (int x = 0; x < 2; ++x)
            {
                constexpr int U = 4;
                constexpr int V = 4;
                constexpr int N = 16;

                int u = u0 + x;
                int v = v0 + y;
                int w0 = (U - u) * (V - v);
                int w1 = u * (V - v);
                int w2 = (U - u) * v;
                int w3 = u * v;

                a->r = (blocks[0].a.r * w0 + blocks[1].a.r * w1 + blocks[2].a.r * w2 + blocks[3].a.r * w3) / N;
                a->g = (blocks[0].a.g * w0 + blocks[1].a.g * w1 + blocks[2].a.g * w2 + blocks[3].a.g * w3) / N;
                a->b = (blocks[0].a.b * w0 + blocks[1].a.b * w1 + blocks[2].a.b * w2 + blocks[3].a.b * w3) / N;
                a->a = (blocks[0].a.a * w0 + blocks[1].a.a * w1 + blocks[2].a.a * w2 + blocks[3].a.a * w3) / N;
                ++a;

                b->r = (blocks[0].b.r * w0 + blocks[1].b.r * w1 + blocks[2].b.r * w2 + blocks[3].b.r * w3) / N;
                b->g = (blocks[0].b.g * w0 + blocks[1].b.g * w1 + blocks[2].b.g * w2 + blocks[3].b.g * w3) / N;
                b->b = (blocks[0].b.b * w0 + blocks[1].b.b * w1 + blocks[2].b.b * w2 + blocks[3].b.b * w3) / N;
                b->a = (blocks[0].b.a * w0 + blocks[1].b.a * w1 + blocks[2].b.a * w2 + blocks[3].b.a * w3) / N;
                ++b;
            }
        }
    }

    void pvrtc2_quad2_bilinear(u8* image, int stride, int u0, int v0,
                               const BlockPVRTC2* blocks,
                               u32 modulation, bool interpolated)
    {
        ColorRGBA a[8];
        ColorRGBA b[8];
        pvrtc2_2bit_bilinear(a, b, u0, v0, blocks);

        u32* scan0 = reinterpret_cast<u32*>(image + stride * 0);
        u32* scan1 = reinterpret_cast<u32*>(image + stride * 1);

        if (interpolated)
        {
            // TODO: interpolated modulation
            /*
            scan0[0] = pvrtc2_lerp(a[0], b[0], (modulation >>  0) & 3);
            scan0[1] = pvrtc2_lerp(a[1], b[1], (modulation >>  2) & 3);
            scan0[2] = pvrtc2_lerp(a[2], b[2], (modulation >>  4) & 3);
            scan0[3] = pvrtc2_lerp(a[3], b[3], (modulation >>  6) & 3);

            scan1[0] = pvrtc2_lerp(a[4], b[4], (modulation >>  8) & 3);
            scan1[1] = pvrtc2_lerp(a[5], b[5], (modulation >> 10) & 3);
            scan1[2] = pvrtc2_lerp(a[6], b[6], (modulation >> 12) & 3);
            scan1[3] = pvrtc2_lerp(a[7], b[7], (modulation >> 14) & 3);
            */
            scan0[0] = 0;
            scan0[1] = 0;
            scan0[2] = 0;
            scan0[3] = 0;

            scan1[0] = 0;
            scan1[1] = 0;
            scan1[2] = 0;
            scan1[3] = 0;
        }
        else
        {
            scan0[0] = pvrtc2_lerp2(a[0], b[0], (modulation >>  0) & 1);
            scan0[1] = pvrtc2_lerp2(a[1], b[1], (modulation >>  1) & 1);
            scan0[2] = pvrtc2_lerp2(a[2], b[2], (modulation >>  2) & 1);
            scan0[3] = pvrtc2_lerp2(a[3], b[3], (modulation >>  3) & 1);

            scan1[0] = pvrtc2_lerp2(a[4], b[4], (modulation >>  8) & 1);
            scan1[1] = pvrtc2_lerp2(a[5], b[5], (modulation >>  9) & 1);
            scan1[2] = pvrtc2_lerp2(a[6], b[6], (modulation >> 10) & 1);
            scan1[3] = pvrtc2_lerp2(a[7], b[7], (modulation >> 11) & 1);
        }
    }

    void pvrtc2_quad2_nearest(u8* image, int stride, ColorRGBA a, ColorRGBA b,
                              u32 modulation, bool interpolated)
    {
        u32* scan0 = reinterpret_cast<u32*>(image + stride * 0);
        u32* scan1 = reinterpret_cast<u32*>(image + stride * 1);

        if (interpolated)
        {
            // TODO: interpolated modulation
            scan0[0] = 0;
            scan0[1] = 0;
            scan0[2] = 0;
            scan0[3] = 0;

            scan1[0] = 0;
            scan1[1] = 0;
            scan1[2] = 0;
            scan1[3] = 0;
            return;
        }

        scan0[0] = pvrtc2_lerp2(a, b, (modulation >>  0) & 1);
        scan0[1] = pvrtc2_lerp2(a, b, (modulation >>  1) & 1);
        scan0[2] = pvrtc2_lerp2(a, b, (modulation >>  2) & 1);
        scan0[3] = pvrtc2_lerp2(a, b, (modulation >>  3) & 1);

        scan1[0] = pvrtc2_lerp2(a, b, (modulation >>  8) & 1);
        scan1[1] = pvrtc2_lerp2(a, b, (modulation >>  9) & 1);
        scan1[2] = pvrtc2_lerp2(a, b, (modulation >> 10) & 1);
        scan1[3] = pvrtc2_lerp2(a, b, (modulation >> 11) & 1);
    }

    void pvrtc2_quad4_bilinear(u8* image, int stride, int u0, int v0,
        const BlockPVRTC2* blocks,
        u32 modulation)
    {
        ColorRGBA a[4];
        ColorRGBA b[4];
        pvrtc2_4bit_bilinear(a, b, u0, v0, blocks);

        u32* scan0 = reinterpret_cast<u32*>(image + stride * 0);
        u32* scan1 = reinterpret_cast<u32*>(image + stride * 1);
        scan0[0] = pvrtc2_lerp(a[0], b[0], (modulation >>  0) & 3);
        scan0[1] = pvrtc2_lerp(a[1], b[1], (modulation >>  2) & 3);
        scan1[0] = pvrtc2_lerp(a[2], b[2], (modulation >>  8) & 3);
        scan1[1] = pvrtc2_lerp(a[3], b[3], (modulation >> 10) & 3);
    }

    void pvrtc2_quad4_punchthrough(u8* image, int stride, int u0, int v0,
        const BlockPVRTC2* blocks,
        u32 modulation)
    {
        ColorRGBA a[4];
        ColorRGBA b[4];
        pvrtc2_4bit_bilinear(a, b, u0, v0, blocks);

        u32* scan0 = reinterpret_cast<u32*>(image + stride * 0);
        u32* scan1 = reinterpret_cast<u32*>(image + stride * 1);
        scan0[0] = pvrtc2_punch(a[0], b[0], (modulation >>  0) & 3);
        scan0[1] = pvrtc2_punch(a[1], b[1], (modulation >>  2) & 3);
        scan1[0] = pvrtc2_punch(a[2], b[2], (modulation >>  8) & 3);
        scan1[1] = pvrtc2_punch(a[3], b[3], (modulation >> 10) & 3);
    }

    void pvrtc2_quad4_nearest(u8* image, int stride, ColorRGBA a, ColorRGBA b, u32 modulation)
    {
        u32* scan0 = reinterpret_cast<u32*>(image + stride * 0);
        u32* scan1 = reinterpret_cast<u32*>(image + stride * 1);
        scan0[0] = pvrtc2_lerp(a, b, (modulation >>  0) & 3);
        scan0[1] = pvrtc2_lerp(a, b, (modulation >>  2) & 3);
        scan1[0] = pvrtc2_lerp(a, b, (modulation >>  8) & 3);
        scan1[1] = pvrtc2_lerp(a, b, (modulation >> 10) & 3);
    }

    void pvrtc2_quad4_palette(u8* image, int stride, int index, const BlockPVRTC2* blocks, u32 modulation)
    {
        ColorRGBA palette[16];

        switch (index)
        {
            case 0:
                palette[ 0] = blocks[0].a;
                palette[ 1] = (blocks[0].a * 5 + blocks[0].b * 3) / 8;
                palette[ 2] = (blocks[0].a * 3 + blocks[0].b * 5) / 8;
                palette[ 3] = blocks[0].b;

                palette[ 4] = blocks[0].a;
                palette[ 5] = blocks[0].b;
                palette[ 6] = blocks[1].a;
                palette[ 7] = blocks[1].b;

                palette[ 8] = blocks[0].a;
                palette[ 9] = blocks[0].b;
                palette[10] = blocks[2].a;
                palette[11] = blocks[2].b;

                palette[12] = blocks[0].a;
                palette[13] = blocks[0].b;
                palette[14] = blocks[1].a;
                palette[15] = blocks[2].b;
                break;
            case 1:
                palette[ 0] = blocks[0].a;
                palette[ 1] = blocks[0].b;
                palette[ 2] = blocks[1].a;
                palette[ 3] = blocks[1].b;

                palette[ 4] = blocks[0].a;
                palette[ 5] = blocks[0].b;
                palette[ 6] = blocks[1].a;
                palette[ 7] = blocks[1].b;

                palette[ 8] = blocks[0].a;
                palette[ 9] = blocks[0].b;
                palette[10] = blocks[1].a;
                palette[11] = blocks[1].b;

                palette[12] = blocks[3].a;
                palette[13] = blocks[0].b;
                palette[14] = blocks[1].a;
                palette[15] = blocks[1].b;
                break;
            case 2:
                palette[ 0] = blocks[0].a;
                palette[ 1] = blocks[0].b;
                palette[ 2] = blocks[2].a;
                palette[ 3] = blocks[2].b;

                palette[ 4] = blocks[0].a;
                palette[ 5] = blocks[0].b;
                palette[ 6] = blocks[2].a;
                palette[ 7] = blocks[2].b;

                palette[ 8] = blocks[0].a;
                palette[ 9] = blocks[0].b;
                palette[10] = blocks[2].a;
                palette[11] = blocks[2].b;

                palette[12] = blocks[0].a;
                palette[13] = blocks[3].b;
                palette[14] = blocks[2].a;
                palette[15] = blocks[2].b;
                break;
            case 3:
                palette[ 0] = blocks[0].a;
                palette[ 1] = blocks[3].b;
                palette[ 2] = blocks[2].a;
                palette[ 3] = blocks[1].b;

                palette[ 4] = blocks[3].a;
                palette[ 5] = blocks[3].b;
                palette[ 6] = blocks[1].a;
                palette[ 7] = blocks[1].b;

                palette[ 8] = blocks[3].a;
                palette[ 9] = blocks[3].b;
                palette[10] = blocks[2].a;
                palette[11] = blocks[2].b;

                palette[12] = blocks[3].a;
                palette[13] = blocks[3].b;
                palette[14] = blocks[2].a;
                palette[15] = blocks[1].b;
                break;
        }

        u32* scan0 = reinterpret_cast<u32*>(image + stride * 0);
        u32* scan1 = reinterpret_cast<u32*>(image + stride * 1);
        scan0[0] = palette[((modulation >>  0) & 3) + 0];
        scan0[1] = palette[((modulation >>  2) & 3) + 4];
        scan1[0] = palette[((modulation >>  8) & 3) + 8];
        scan1[1] = palette[((modulation >> 10) & 3) + 12];
    }

    void pvrtc2_quad_debug(u8* image, int xsize, int stride, ColorRGBA color)
    {
        u32* scan0 = reinterpret_cast<u32*>(image + stride * 0);
        u32* scan1 = reinterpret_cast<u32*>(image + stride * 1);
        for (int x = 0; x < xsize; ++x)
        {
            scan0[x] = color;
            scan1[x] = color;
        }
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
        const u32 quad_width = block_width / 2;
        const u32 quad_height = block_height / 2;
        const u32 xblocks = ceil_div(width, block_width);
        const u32 yblocks = ceil_div(height, block_height);

        for (int y0 = 0; y0 < yblocks; ++y0)
        {
            for (int x0 = 0; x0 < xblocks; ++x0)
            {
                int x1 = (x0 + 1) % xblocks;
                int y1 = (y0 + 1) % yblocks;

                BlockPVRTC2 blocks[] =
                {
                    BlockPVRTC2(data + (y0 * xblocks + x0) * 8),
                    BlockPVRTC2(data + (y0 * xblocks + x1) * 8),
                    BlockPVRTC2(data + (y1 * xblocks + x0) * 8),
                    BlockPVRTC2(data + (y1 * xblocks + x1) * 8),
                };

                const int mods[] =
                {
                    20, 16, 4, 0
                };

                for (int y = 0; y < 2; ++y)
                {
                    for (int x = 0; x < 2; ++x)
                    {
                        int index = y * 2 + x;
                        const BlockPVRTC2& block = blocks[index];
                        u32 modulation = block.modulation >> mods[index];

                        u8* scan = image + ((y0 * block_height + y * quad_height + quad_height) % height) * stride + 
                                           ((x0 * block_width  + x * quad_width + quad_width) % width) * 4;

                        ColorRGBA color;
                        bool debug = false;

                        int mode = (!!blocks[0].hard) * 2 + !!block.mode;
                        switch (mode)
                        {
                            case 0:
                                color = ColorRGBA(0, 255, 0, 255);
                                debug = false;
                                if (bpp == 2)
                                    pvrtc2_quad2_bilinear(scan, stride, x * 4, y * 2, blocks, modulation, false);
                                else
                                    pvrtc2_quad4_bilinear(scan, stride, x * 2, y * 2, blocks, modulation);
                                break;
                            case 1:
                                color = ColorRGBA(0, 0, 255, 255);
                                debug = false;
                                if (bpp == 2)
                                    pvrtc2_quad2_bilinear(scan, stride, x * 4, y * 2, blocks, modulation, true);
                                else
                                    pvrtc2_quad4_punchthrough(scan, stride, x * 2, y * 2, blocks, modulation);
                                break;
                            case 2:
                                color = ColorRGBA(255, 0, 0, 255);
                                debug = false;
                                if (bpp == 2)
                                    pvrtc2_quad2_nearest(scan, stride, block.a, block.b, modulation, false);
                                else
                                    pvrtc2_quad4_nearest(scan, stride, block.a, block.b, modulation);
                                break;
                            case 3:
                                color = ColorRGBA(255, 255, 0, 255);
                                debug = false;
                                if (bpp == 2)
                                    pvrtc2_quad2_nearest(scan, stride, block.a, block.b, modulation, true);
                                else
                                    pvrtc2_quad4_palette(scan, stride, index, blocks, modulation);
                                break;
                        }

                        if (debug)
                        {
                            pvrtc2_quad_debug(scan, block_width, stride, color);
                            pvrtc2_quad_debug(scan, block_width, stride, color);
                            pvrtc2_quad_debug(scan, block_width, stride, color);
                            pvrtc2_quad_debug(scan, block_width, stride, color);
                        }
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
