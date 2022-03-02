/*
    MANGO Multimedia Development Platform
    Copyright (C) 2012-2021 Twilight Finland 3D Oy Ltd. All rights reserved.
*/
#pragma once

#include <cmath>
#include <vector>
#include <string>
#include <cstdio>
#include <cstddef>
#include <cstring>

#include <mango/math/vector.hpp>

#include <mango/core/timer.hpp>
#include <mango/core/system.hpp>
#include <mango/core/endian.hpp>
#include <mango/core/stream.hpp>
#include <mango/core/buffer.hpp>
#include <mango/core/memory.hpp>
#include <mango/core/string.hpp>
#include <mango/core/thread.hpp>
#include <mango/core/pointer.hpp>
#include <mango/core/cpuinfo.hpp>
#include <mango/core/configure.hpp>
#include <mango/core/exception.hpp>

#include <mango/image/exif.hpp>
#include <mango/image/fourcc.hpp>
#include <mango/image/color.hpp>
#include <mango/image/format.hpp>
#include <mango/image/blitter.hpp>
#include <mango/image/surface.hpp>
#include <mango/image/quantize.hpp>
#include <mango/image/compression.hpp>


namespace mango::image
{
    struct ImageHeader : Status
    {
        int     width = 0;   // width
        int     height = 0;  // height
        int     depth = 0;   // depth
        int     levels = 0;  // mipmap levels
        int     faces = 0;   // cubemap faces
        bool    palette = false; // palette is available
        bool    premultiplied = false; // alpha is premultiplied
        Format  format; // preferred format (fastest available "direct" decoding is possible)
        TextureCompression compression = TextureCompression::NONE;
    };

    struct ImageDecodeStatus : Status
    {
        bool direct = false;

        // animation information
        // NOTE: we would love to simply return number of animation frames in the ImageHeader
        //       but some formats do not provide this information without decompressing frames
        //       until running out of data.
        int current_frame_index = 0;
        int next_frame_index = 0;

        // animation frame duration in (numerator / denominator) seconds
        int frame_delay_numerator = 1;    // 1 frame...
        int frame_delay_denominator = 60; // ... every 60th of a second
    };

    struct ImageDecodeOptions
    {
        // request indexed decoding
        // - palette is resolved into the provided palette object
        // - decode() destination surface must be indexed
        Palette* palette = nullptr; // enable indexed decoding by pointing to a palette

        bool simd = true;
        bool multithread = true;
    };

    struct ImageEncodeStatus : Status
    {
        bool direct = false;
    };

    struct ImageEncodeOptions
    {
        Palette palette;

        ConstMemory icc; // jpeg, png

        float quality = 0.90f; // jpeg: [0.0, 1.0]
        int compression = 5; // png: [0, 10]
        bool filtering = false; // png
        bool dithering = true; // gif
        bool lossless = false; // webp

        bool simd = true; // jpeg
        bool multithread = true; // jpeg
    };
}


// ----------------------------------------------------------------------------
// main
// ----------------------------------------------------------------------------

namespace mango::jpeg
{

    // ----------------------------------------------------------------------------
    // Specifications
    // ----------------------------------------------------------------------------

    // https://www.w3.org/Graphics/JPEG/itu-t81.pdf

    // ----------------------------------------------------------------------------
    // markers
    // ----------------------------------------------------------------------------

    enum jpegMarker
    {
        // Start of Frame markers, non-differential, Huffman coding
        MARKER_SOF0     = 0xffc0,  // Baseline DCT
        MARKER_SOF1     = 0xffc1,  // Extended sequential DCT
        MARKER_SOF2     = 0xffc2,  // Progressive DCT
        MARKER_SOF3     = 0xffc3,  // Lossless (sequential)

        // Huffman table specification
        MARKER_DHT      = 0xffc4,  // Define Huffman table(s)

        // Start of Frame markers, differential, Huffman coding
        MARKER_SOF5     = 0xffc5,  // Differential sequential DCT
        MARKER_SOF6     = 0xffc6,  // Differential progressive DCT
        MARKER_SOF7     = 0xffc7,  // Differential lossless (sequential)

        // Start of Frame markers, non-differential, arithmetic coding
        MARKER_JPG      = 0xffc8,  // Reserved for JPEG extensions
        MARKER_SOF9     = 0xffc9,  // Extended sequential DCT
        MARKER_SOF10    = 0xffca,  // Progressive DCT
        MARKER_SOF11    = 0xffcb,  // Lossless (sequential)

        // Arithmetic coding conditioning specification
        MARKER_DAC      = 0xffcc,  // Define arithmetic coding conditioning(s)

        // Start of Frame markers, differential, arithmetic coding
        MARKER_SOF13    = 0xffcd,  // Differential sequential DCT
        MARKER_SOF14    = 0xffce,  // Differential progressive DCT
        MARKER_SOF15    = 0xffcf,  // Differential lossless (sequential)

        // Restart interval termination
        MARKER_RST0     = 0xffd0,  // Restart with modulo 8 count 0
        MARKER_RST1     = 0xffd1,  // Restart with modulo 8 count 1
        MARKER_RST2     = 0xffd2,  // Restart with modulo 8 count 2
        MARKER_RST3     = 0xffd3,  // Restart with modulo 8 count 3
        MARKER_RST4     = 0xffd4,  // Restart with modulo 8 count 4
        MARKER_RST5     = 0xffd5,  // Restart with modulo 8 count 5
        MARKER_RST6     = 0xffd6,  // Restart with modulo 8 count 6
        MARKER_RST7     = 0xffd7,  // Restart with modulo 8 count 7

        // Other markers
        MARKER_SOI      = 0xffd8,  // Start of image
        MARKER_EOI      = 0xffd9,  // End of image
        MARKER_SOS      = 0xffda,  // Start of scan
        MARKER_DQT      = 0xffdb,  // Define quantization table(s)
        MARKER_DNL      = 0xffdc,  // Define number of lines
        MARKER_DRI      = 0xffdd,  // Define restart interval
        MARKER_DHP      = 0xffde,  // Define hierarchical progression
        MARKER_EXP      = 0xffdf,  // Expand reference component(s)
        MARKER_APP0     = 0xffe0,  // Reserved for application segments
        MARKER_APP1     = 0xffe1,  // Reserved for application segments
        MARKER_APP2     = 0xffe2,  // Reserved for application segments
        MARKER_APP3     = 0xffe3,  // Reserved for application segments
        MARKER_APP4     = 0xffe4,  // Reserved for application segments
        MARKER_APP5     = 0xffe5,  // Reserved for application segments
        MARKER_APP6     = 0xffe6,  // Reserved for application segments
        MARKER_APP7     = 0xffe7,  // Reserved for application segments
        MARKER_APP8     = 0xffe8,  // Reserved for application segments
        MARKER_APP9     = 0xffe9,  // Reserved for application segments
        MARKER_APP10    = 0xffea,  // Reserved for application segments
        MARKER_APP11    = 0xffeb,  // Reserved for application segments
        MARKER_APP12    = 0xffec,  // Reserved for application segments
        MARKER_APP13    = 0xffed,  // Reserved for application segments
        MARKER_APP14    = 0xffee,  // Reserved for application segments
        MARKER_APP15    = 0xffef,  // Reserved for application segments
        MARKER_JPG0     = 0xfff0,  // Reserved for JPEG extensions
        MARKER_JPG1     = 0xfff1,  // Reserved for JPEG extensions
        MARKER_JPG2     = 0xfff2,  // Reserved for JPEG extensions
        MARKER_JPG3     = 0xfff3,  // Reserved for JPEG extensions
        MARKER_JPG4     = 0xfff4,  // Reserved for JPEG extensions
        MARKER_JPG5     = 0xfff5,  // Reserved for JPEG extensions
        MARKER_JPG6     = 0xfff6,  // Reserved for JPEG extensions
        MARKER_JPG7     = 0xfff7,  // Reserved for JPEG extensions
        MARKER_JPG8     = 0xfff8,  // Reserved for JPEG extensions
        MARKER_JPG9     = 0xfff9,  // Reserved for JPEG extensions
        MARKER_JPG10    = 0xfffa,  // Reserved for JPEG extensions
        MARKER_JPG11    = 0xfffb,  // Reserved for JPEG extensions
        MARKER_JPG12    = 0xfffc,  // Reserved for JPEG extensions
        MARKER_JPG13    = 0xfffd,  // Reserved for JPEG extensions
        MARKER_COM      = 0xfffe,  // Comment

        // Reserved markers
        MARKER_TEM      = 0xff01,  // For temporary private use in arithmetic coding
        MARKER_RES      = 0xff02   // Reserved (0x02 .. 0xbf)
    };

    // ----------------------------------------------------------------------------
    // types
    // ----------------------------------------------------------------------------

    using mango::u8;
    using mango::u16;
    using mango::u32;
    using mango::u64;
    using mango::s16;

    using mango::Stream;
    using mango::ThreadPool;
    using mango::Memory;
    using mango::ConstMemory;

    using mango::image::Format;
    using mango::image::LuminanceFormat;
    using mango::image::Surface;
    using mango::image::Bitmap;
    using mango::image::ImageHeader;
    using mango::image::ImageDecodeStatus;
    using mango::image::ImageEncodeStatus;
    using mango::image::ImageDecodeOptions;
    using mango::image::ImageEncodeOptions;

#ifdef MANGO_CPU_64BIT

    using DataType = u64;

    #define bextr mango::u64_extract_bits

    #define JPEG_REGISTER_BITS  64
    #define JPEG_REGISTER_BYTES 8
    #define JPEG_REGISTER_FILL  6

#else

    using DataType = u32;

    #define bextr mango::u32_extract_bits

    #define JPEG_REGISTER_BITS  32
    #define JPEG_REGISTER_BYTES 4
    #define JPEG_REGISTER_FILL  2

#endif

    constexpr int JPEG_MAX_BLOCKS_IN_MCU  = 10;  // Maximum # of blocks per MCU in the JPEG specification
    constexpr int JPEG_MAX_SAMPLES_IN_MCU = 64 * JPEG_MAX_BLOCKS_IN_MCU;
    constexpr int JPEG_MAX_COMPS_IN_SCAN  = 4;   // JPEG limit on # of components in one scan
    constexpr int JPEG_NUM_ARITH_TBLS     = 16;  // Arith-coding tables are numbered 0..15
    constexpr int JPEG_DC_STAT_BINS       = 64;  // ...
    constexpr int JPEG_AC_STAT_BINS       = 256; // ...
    constexpr int JPEG_HUFF_LOOKUP_BITS   = 8;   // Huffman look-ahead table log2 size
    constexpr int JPEG_HUFF_LOOKUP_SIZE   = (1 << JPEG_HUFF_LOOKUP_BITS);

    // supported external data formats (encode from, decode to)
    enum SampleType
    {
        JPEG_U8_Y,
        JPEG_U8_BGR,
        JPEG_U8_RGB,
        JPEG_U8_BGRA,
        JPEG_U8_RGBA,
    };

    enum class ColorSpace
    {
        CMYK = 0,
        YCBCR = 1,
        YCCK = 2
    };

    struct SampleFormat
    {
        SampleType sample;
        Format format;
    };

    struct QuantTable
    {
        s16* table;  // Quantization table
        int  bits;   // Quantization table precision (8 or 16 bits)
    };

    struct BitBuffer
    {
        const u8* ptr;
        const u8* end;

        DataType data;
        int remain;

        void restart();
        void fill();

        void ensure()
        {
            if (remain < 16)
            {
                fill();
            }
        }

        int getBits(int nbits)
        {
            ensure();
            return int(bextr(data, remain -= nbits, nbits));
        }

        int peekBits(int nbits)
        {
            return int(bextr(data, remain - nbits, nbits));
        }

        int extend(int value, int nbits) const
        {
            return value - ((((value + value) >> nbits) - 1) & ((1 << nbits) - 1));
            //return value - int(bextr(((value + value) >> nbits) - 1, nbits, nbits));
        }

        int receive(int nbits)
        {
            int value = getBits(nbits);
            return extend(value, nbits);
        }
    };

    struct HuffTable
    {
        u8 size[17];
        u8 value[256];

        // acceleration tables
        DataType maxcode[18];
        DataType valueOffset[19];
        u8 lookupSize[JPEG_HUFF_LOOKUP_SIZE];
        u8 lookupValue[JPEG_HUFF_LOOKUP_SIZE];

        bool configure();
        int decode(BitBuffer& buffer) const;
    };

    struct Huffman
    {
        int last_dc_value[JPEG_MAX_COMPS_IN_SCAN];
        int eob_run;

        HuffTable table[2][JPEG_MAX_COMPS_IN_SCAN];

        void restart();
    };

    struct Arithmetic
    {
        u32 c;
        u32 a;
        int ct;

        int last_dc_value[JPEG_MAX_COMPS_IN_SCAN]; // Last DC coef for each component
        int dc_context[JPEG_MAX_COMPS_IN_SCAN]; // Context index for DC conditioning

        u8 dc_L[JPEG_NUM_ARITH_TBLS]; // L values for DC arith-coding tables
        u8 dc_U[JPEG_NUM_ARITH_TBLS]; // U values for DC arith-coding tables
        u8 ac_K[JPEG_NUM_ARITH_TBLS]; // K values for AC arith-coding tables

        u8 dc_stats[JPEG_NUM_ARITH_TBLS][JPEG_DC_STAT_BINS];
        u8 ac_stats[JPEG_NUM_ARITH_TBLS][JPEG_AC_STAT_BINS];
        u8 fixed_bin[4]; // Statistics bin for coding with fixed probability 0.5

        Arithmetic();
        ~Arithmetic();

        void restart(BitBuffer& buffer);
    };

    struct Frame
    {
        int compid; // Component identifier
        int Hsf;    // Horizontal sampling factor
        int Vsf;    // Vertical sampling factor
        int Tq;     // Quantization table destination selector
        int offset;
    };

    struct DecodeBlock
    {
        int offset;
        int pred;
        int dc;
        int ac;
    };

    struct DecodeState
    {
        BitBuffer buffer;
        Huffman huffman;
        Arithmetic arithmetic;

        DecodeBlock block[JPEG_MAX_BLOCKS_IN_MCU];
        int blocks;
        int comps_in_scan;

        const u8* zigzagTable;

        int spectralStart;
        int spectralEnd;
        int successiveHigh;
        int successiveLow;

        void (*decode)(s16* output, DecodeState* state);
    };

    struct Block
    {
        s16* qt;
    };

    struct ProcessState
    {
        // NOTE: this is just quantization tables
        Block block[JPEG_MAX_BLOCKS_IN_MCU];
        int blocks;

        Frame frame[JPEG_MAX_COMPS_IN_SCAN];
        int frames;
        ColorSpace colorspace = ColorSpace::CMYK; // default

        void (*idct) (u8* dest, const s16* data, const s16* qt);

        void (*process            ) (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
        void (*process_y          ) (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
        void (*process_cmyk       ) (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
        void (*process_ycbcr      ) (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
        void (*process_ycbcr_8x8  ) (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
        void (*process_ycbcr_8x16 ) (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
        void (*process_ycbcr_16x8 ) (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
        void (*process_ycbcr_16x16) (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    };

    // ----------------------------------------------------------------------------
    // Parser
    // ----------------------------------------------------------------------------

    class Parser
    {
    protected:
        QuantTable quantTable[JPEG_MAX_COMPS_IN_SCAN];

        AlignedStorage<s16> quantTableVector;
        AlignedStorage<s16> blockVector;

        std::vector<Frame> frames;
        Frame* scanFrame = nullptr; // current Progressive AC scan frame

        DecodeState decodeState;
        ProcessState processState;

        int restartInterval;
        int restartCounter;

        int m_hardware_concurrency;

        std::string m_encoding;
        std::string m_compression;
        std::string m_idct_name;
        std::string m_ycbcr_name;

        const Surface* m_surface;

        int width;  // Image width, does include alignment
        int height; // Image height, does include alignment
        int xsize;  // Image width, does NOT include alignment
        int ysize;  // Image height, does NOT include alignment
        int precision; // 8 or 12 bits
        int components; // 1..4

        bool is_baseline = true;
        bool is_progressive = false;
        bool is_multiscan = false;
        bool is_arithmetic = false;
        bool is_lossless = false;
        bool is_differential = false;

        int Hmax;
        int Vmax;
        int blocks_in_mcu;
        int xblock;
        int yblock;
        int xmcu;
        int ymcu;
        int mcus;

        bool isJPEG(ConstMemory memory) const;

        const u8* stepMarker(const u8* p, const u8* end) const;
        const u8* seekMarker(const u8* p, const u8* end) const;
        const u8* seekRestartMarker(const u8* p, const u8* end) const;

        void processSOI();
        void processEOI();
        void processCOM(const u8* p);
        void processTEM(const u8* p);
        void processRES(const u8* p);
        void processJPG(const u8* p);
        void processJPG(const u8* p, u16 marker);
        void processAPP(const u8* p, u16 marker);
        void processSOF(const u8* p, u16 marker);
        const u8* processSOS(const u8* p, const u8* end);
        void processDQT(const u8* p);
        void processDNL(const u8* p);
        void processDRI(const u8* p);
        void processDHT(const u8* p, const u8* end);
        void processDAC(const u8* p);
        void processDHP(const u8* p);
        void processEXP(const u8* p);

        void parse(ConstMemory memory, bool decode);

        void restart();
        bool handleRestart();

        void decodeLossless();
        void decodeSequential();
        void decodeSequentialST();
        void decodeSequentialMT(int N);
        void decodeMultiScan();
        void decodeProgressive();
        void decodeProgressiveDC();
        void decodeProgressiveAC();
        void finishProgressive();

        void process_range(int y0, int y1, const s16* data);
        void process_and_clip(u8* dest, size_t stride, const s16* data, int width, int height);

        int getTaskSize(int count) const;
        void configureCPU(SampleType sample, const ImageDecodeOptions& options);
        std::string getInfo() const;

    public:
        ImageHeader header;
        ConstMemory exif_memory; // Exif block, if one is present
        ConstMemory scan_memory; // Scan block
        Buffer icc_buffer; // ICC color profile block, if one is present

        Parser(ConstMemory memory);
        ~Parser();

        ImageDecodeStatus decode(const Surface& target, const ImageDecodeOptions& options);
    };

    // ----------------------------------------------------------------------------
    // functions
    // ----------------------------------------------------------------------------

    using ProcessFunc = void (*)(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);

    void huff_decode_mcu_lossless   (s16* output, DecodeState* state);
    void huff_decode_mcu            (s16* output, DecodeState* state);
    void huff_decode_dc_first       (s16* output, DecodeState* state);
    void huff_decode_dc_refine      (s16* output, DecodeState* state);
    void huff_decode_ac_first       (s16* output, DecodeState* state);
    void huff_decode_ac_refine      (s16* output, DecodeState* state);

    void arith_decode_mcu_lossless  (s16* output, DecodeState* state);
    void arith_decode_mcu           (s16* output, DecodeState* state);
    void arith_decode_dc_first      (s16* output, DecodeState* state);
    void arith_decode_dc_refine     (s16* output, DecodeState* state);
    void arith_decode_ac_first      (s16* output, DecodeState* state);
    void arith_decode_ac_refine     (s16* output, DecodeState* state);

    void idct8                          (u8* dest, const s16* data, const s16* qt);
    void idct12                         (u8* dest, const s16* data, const s16* qt);

    void process_y_8bit                 (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_y_24bit                (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_y_32bit                (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_cmyk_bgra              (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_8bit             (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);

    void process_ycbcr_bgr              (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgr_8x8          (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgr_8x16         (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgr_16x8         (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgr_16x16        (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);

    void process_ycbcr_rgb              (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgb_8x8          (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgb_8x16         (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgb_16x8         (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgb_16x16        (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);

    void process_ycbcr_bgra             (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgra_8x8         (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgra_8x16        (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgra_16x8        (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgra_16x16       (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);

    void process_ycbcr_rgba             (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgba_8x8         (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgba_8x16        (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgba_16x8        (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgba_16x16       (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);

#if defined(MANGO_ENABLE_NEON)

    void idct_neon                      (u8* dest, const s16* data, const s16* qt);

    void process_ycbcr_bgra_8x8_neon    (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgra_8x16_neon   (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgra_16x8_neon   (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgra_16x16_neon  (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);

    void process_ycbcr_rgba_8x8_neon    (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgba_8x16_neon   (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgba_16x8_neon   (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgba_16x16_neon  (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);

    void process_ycbcr_bgr_8x8_neon     (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgr_8x16_neon    (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgr_16x8_neon    (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgr_16x16_neon   (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);

    void process_ycbcr_rgb_8x8_neon     (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgb_8x16_neon    (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgb_16x8_neon    (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgb_16x16_neon   (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);

#endif // MANGO_ENABLE_NEON

#if defined(MANGO_ENABLE_SSE2)

    void idct_sse2                      (u8* dest, const s16* data, const s16* qt);

    void process_ycbcr_bgra_8x8_sse2    (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgra_8x16_sse2   (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgra_16x8_sse2   (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgra_16x16_sse2  (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);

    void process_ycbcr_rgba_8x8_sse2    (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgba_8x16_sse2   (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgba_16x8_sse2   (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgba_16x16_sse2  (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);

#endif // MANGO_ENABLE_SSE2

#if defined(MANGO_ENABLE_SSE4_1)

    void process_ycbcr_bgr_8x8_ssse3    (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgr_8x16_ssse3   (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgr_16x8_ssse3   (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_bgr_16x16_ssse3  (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);

    void process_ycbcr_rgb_8x8_ssse3    (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgb_8x16_ssse3   (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgb_16x8_ssse3   (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);
    void process_ycbcr_rgb_16x16_ssse3  (u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height);

#endif // MANGO_ENABLE_SSE4_1

    SampleFormat getSampleFormat(const Format& format);
    ImageEncodeStatus encodeImage(Stream& stream, const Surface& surface, const ImageEncodeOptions& options);

} // namespace mango::jpeg

// ----------------------------------------------------------------------------
// arithmetic
// ----------------------------------------------------------------------------

namespace
{
    using namespace mango;
    using namespace jpeg;

    const u32 jpeg_aritab[] =
    {
        0x5a1d0181, 0x2586020e, 0x11140310, 0x080b0412, 0x03d80514, 0x01da0617,
        0x00e50719, 0x006f081c, 0x0036091e, 0x001a0a21, 0x000d0b23, 0x00060c09,
        0x00030d0a, 0x00010d0c, 0x5a7f0f8f, 0x3f251024, 0x2cf21126, 0x207c1227,
        0x17b91328, 0x1182142a, 0x0cef152b, 0x09a1162d, 0x072f172e, 0x055c1830,
        0x04061931, 0x03031a33, 0x02401b34, 0x01b11c36, 0x01441d38, 0x00f51e39,
        0x00b71f3b, 0x008a203c, 0x0068213e, 0x004e223f, 0x003b2320, 0x002c0921,
        0x5ae125a5, 0x484c2640, 0x3a0d2741, 0x2ef12843, 0x261f2944, 0x1f332a45,
        0x19a82b46, 0x15182c48, 0x11772d49, 0x0e742e4a, 0x0bfb2f4b, 0x09f8304d,
        0x0861314e, 0x0706324f, 0x05cd3330, 0x04de3432, 0x040f3532, 0x03633633,
        0x02d43734, 0x025c3835, 0x01f83936, 0x01a43a37, 0x01603b38, 0x01253c39,
        0x00f63d3a, 0x00cb3e3b, 0x00ab3f3d, 0x008f203d, 0x5b1241c1, 0x4d044250,
        0x412c4351, 0x37d84452, 0x2fe84553, 0x293c4654, 0x23794756, 0x1edf4857,
        0x1aa94957, 0x174e4a48, 0x14244b48, 0x119c4c4a, 0x0f6b4d4a, 0x0d514e4b,
        0x0bb64f4d, 0x0a40304d, 0x583251d0, 0x4d1c5258, 0x438e5359, 0x3bdd545a,
        0x34ee555b, 0x2eae565c, 0x299a575d, 0x25164756, 0x557059d8, 0x4ca95a5f,
        0x44d95b60, 0x3e225c61, 0x38245d63, 0x32b45e63, 0x2e17565d, 0x56a860df,
        0x4f466165, 0x47e56266, 0x41cf6367, 0x3c3d6468, 0x375e5d63, 0x52316669,
        0x4c0f676a, 0x4639686b, 0x415e6367, 0x56276ae9, 0x50e76b6c, 0x4b85676d,
        0x55976d6e, 0x504f6b6f, 0x5a106fee, 0x55226d70, 0x59eb6ff0, 0x5a1d7171
    };

    inline u8 get_byte(BitBuffer& buffer)
    {
        // guard against corrupted bit-streams
        if (buffer.ptr >= buffer.end)
            return 0;

        u8 value = *buffer.ptr++;
        if (value == 0xff)
        {
            // skip stuff byte (0x00)
            ++buffer.ptr;
        }

        return value;
    }

    int arith_decode(Arithmetic& e, BitBuffer& buffer, u8* st)
    {
        while (e.a < 0x8000L)
        {
            if (--e.ct < 0)
            {
                int data = get_byte(buffer);
                e.c = (e.c << 8) | data;
                e.ct += 8;
            }

            e.a <<= 1;
        }

        // Fetch values from our compact representation of Table D.2:
        // Qe values and probability estimation state machine
        int sv = *st;

        u32 qe = jpeg_aritab[sv & 0x7F];
        u8 nextLPS = qe & 0xFF; qe >>= 8;
        u8 nextMPS = qe & 0xFF; qe >>= 8;

        // Decode & estimation procedures per sections D.2.4 & D.2.5
        u32 temp = e.a - qe;
        e.a = temp;
        temp <<= e.ct;

        if (e.c >= temp)
        {
            e.c -= temp;

            // Conditional LPS (less probable symbol) exchange
            if (e.a < qe)
            {
                e.a = qe;
                *st = (sv & 0x80) ^ nextMPS; // Estimate_after_MPS
            }
            else
            {
                e.a = qe;
                *st = (sv & 0x80) ^ nextLPS; // Estimate_after_LPS
                sv ^= 0x80; // Exchange LPS/MPS
            }
        }
        else
        {
            if (e.a < 0x8000L)
            {
                // Conditional MPS (more probable symbol) exchange
                if (e.a < qe)
                {
                    *st = (sv & 0x80) ^ nextLPS; // Estimate_after_LPS
                    sv ^= 0x80; // Exchange LPS/MPS
                }
                else
                {
                    *st = (sv & 0x80) ^ nextMPS; // Estimate_after_MPS
                }
            }
        }

        return sv >> 7;
    }

} // namespace

namespace mango::jpeg
{

    // ----------------------------------------------------------------------------
    // arithmetic decoder
    // ----------------------------------------------------------------------------

    void arith_decode_mcu_lossless(s16* output, DecodeState* state)
    {
        Arithmetic& arithmetic = state->arithmetic;
        BitBuffer& buffer = state->buffer;
        DecodeBlock* block = state->block;

        for (int j = 0; j < state->blocks; ++j)
        {
            const int ci = block->pred;

            // DC
            int tbl = block->dc;
            u8* st = arithmetic.dc_stats[tbl] + arithmetic.dc_context[ci];

            if (arith_decode(arithmetic, buffer, st) == 0)
            {
                arithmetic.dc_context[ci] = 0;
            }
            else
            {
                int sign;
                int v, m;

                sign = arith_decode(arithmetic, buffer, st + 1);
                st += 2;
                st += sign;

                if ((m = arith_decode(arithmetic, buffer, st)) != 0)
                {
                    st = arithmetic.dc_stats[tbl] + 20;
                    while (arith_decode(arithmetic, buffer, st))
                    {
                        m <<= 1;
                        ++st;
                    }
                }

                if (m < (int) ((1L << arithmetic.dc_L[tbl]) >> 1))
                {
                    // zero diff category
                    arithmetic.dc_context[ci] = 0;
                }
                else if (m > (int) ((1L << arithmetic.dc_U[tbl]) >> 1))
                {
                    // large diff category
                    arithmetic.dc_context[ci] = 12 + (sign * 4);
                }
                else
                {
                    // small diff category
                    arithmetic.dc_context[ci] = 4 + (sign * 4);
                }

                v = m;

                st += 14;
                while (m >>= 1)
                {
                    if (arith_decode(arithmetic, buffer, st))
                        v |= m;
                }

                v += 1; if (sign) v = -v;
                arithmetic.last_dc_value[ci] += v;
            }

            output[j] = s16(arithmetic.last_dc_value[ci]);
        }
    }

    void arith_decode_mcu(s16* output, DecodeState* state)
    {
        const u8* zigzagTable = state->zigzagTable;
        Arithmetic& arithmetic = state->arithmetic;
        BitBuffer& buffer = state->buffer;
        DecodeBlock* block = state->block;

        std::memset(output, 0, state->blocks * 64 * sizeof(s16));

        const int end = state->spectralEnd;

        for (int j = 0; j < state->blocks; ++j)
        {
            const int ci = block->pred;

            // DC
            int tbl = block->dc;
            u8* st = arithmetic.dc_stats[tbl] + arithmetic.dc_context[ci];

            if (arith_decode(arithmetic, buffer, st) == 0)
            {
                arithmetic.dc_context[ci] = 0;
            }
            else
            {
                int sign;
                int v, m;

                sign = arith_decode(arithmetic, buffer, st + 1);
                st += 2;
                st += sign;

                if ((m = arith_decode(arithmetic, buffer, st)) != 0)
                {
                    st = arithmetic.dc_stats[tbl] + 20;
                    while (arith_decode(arithmetic, buffer, st))
                    {
                        m <<= 1;
                        ++st;
                    }
                }

                if (m < (int) ((1L << arithmetic.dc_L[tbl]) >> 1))
                {
                    // zero diff category
                    arithmetic.dc_context[ci] = 0;
                }
                else if (m > (int) ((1L << arithmetic.dc_U[tbl]) >> 1))
                {
                    // large diff category
                    arithmetic.dc_context[ci] = 12 + (sign * 4);
                }
                else
                {
                    // small diff category
                    arithmetic.dc_context[ci] = 4 + (sign * 4);
                }

                v = m;

                st += 14;
                while (m >>= 1)
                {
                    if (arith_decode(arithmetic, buffer, st))
                        v |= m;
                }

                v += 1; if (sign) v = -v;
                arithmetic.last_dc_value[ci] += v;
            }

            output[0] = s16(arithmetic.last_dc_value[ci]);

            // AC
            tbl = block->ac;
            u8* ac_stats = arithmetic.ac_stats[tbl];
            u8 ac_K = arithmetic.ac_K[tbl];

            for (int k = 1; k <= end; k++)
            {
                int sign;
                int v, m;

                st = ac_stats + 3 * (k - 1);

                if (arith_decode(arithmetic, buffer, st))
                    break;

                while (arith_decode(arithmetic, buffer, st + 1) == 0)
                {
                    st += 3;
                    ++k;
                }

                sign = arith_decode(arithmetic, buffer, arithmetic.fixed_bin);
                st += 2;

                if ((m = arith_decode(arithmetic, buffer, st)) != 0)
                {
                    if (arith_decode(arithmetic, buffer, st))
                    {
                        m <<= 1;
                        st = ac_stats + (k <= ac_K ? 189 : 217);

                        while (arith_decode(arithmetic, buffer, st))
                        {
                            m <<= 1;
                            ++st;
                        }
                    }
                }
                v = m;

                st += 14;
                while (m >>= 1)
                {
                    if (arith_decode(arithmetic, buffer, st))
                    {
                        v |= m;
                    }
                }
                v += 1; if (sign) v = -v;

                output[zigzagTable[k]] = s16(v);
            }

            ++block;
            output += 64;
        }
    }

    void arith_decode_dc_first(s16* output, DecodeState* state)
    {
        Arithmetic& arithmetic = state->arithmetic;
        BitBuffer& buffer = state->buffer;
        DecodeBlock* block = state->block;

        for (int j = 0; j < state->blocks; ++j)
        {
            s16* dest = output + block->offset;
            const int ci = block->pred;

            std::memset(dest, 0, 64 * sizeof(s16));

            int tbl = block->dc;
            u8* st = arithmetic.dc_stats[tbl] + arithmetic.dc_context[ci];

            int sign;
            int v, m;

            if (arith_decode(arithmetic, buffer, st) == 0)
            {
                arithmetic.dc_context[ci] = 0;
            }
            else
            {
                sign = arith_decode(arithmetic, buffer, st + 1);
                st += 2; st += sign;

                if ((m = arith_decode(arithmetic, buffer, st)) != 0)
                {
                    st = arithmetic.dc_stats[tbl] + 20; // Table F.4: X1 = 20
                    while (arith_decode(arithmetic, buffer, st))
                    {
                        m += m;
                        ++st;
                    }
                }

                // Section F.1.4.4.1.2: Establish dc_context conditioning category
                if (m < (int) ((1L << arithmetic.dc_L[tbl]) >> 1))
                {
                    // zero diff category
                    arithmetic.dc_context[ci] = 0;
                }
                else if (m > (int) ((1L << arithmetic.dc_U[tbl]) >> 1))
                {
                    // large diff category
                    arithmetic.dc_context[ci] = 12 + (sign * 4);
                }
                else
                {
                    // small diff category
                    arithmetic.dc_context[ci] = 4 + (sign * 4);
                }

                v = m;

                // Figure F.24: Decoding the magnitude bit pattern of v
                st += 14;
                while (m >>= 1)
                {
                    if (arith_decode(arithmetic, buffer, st))
                        v |= m;
                }

                v += 1; if (sign) v = -v;
                arithmetic.last_dc_value[ci] += v;
            }

            dest[0] = s16(arithmetic.last_dc_value[ci] << state->successiveLow);
            ++block;
        }
    }

    void arith_decode_dc_refine(s16* output, DecodeState* state)
    {
        Arithmetic& arithmetic = state->arithmetic;
        BitBuffer& buffer = state->buffer;
        u8* st = arithmetic.fixed_bin;

        for (int j = 0; j < state->blocks; ++j)
        {
            s16* dest = output + state->block[j].offset;

            // Encoded data is simply the next bit of the two's-complement DC value
            if (arith_decode(arithmetic, buffer, st))
            {
                dest[0] |= (1 << state->successiveLow);
            }
        }
    }

    void arith_decode_ac_first(s16* output, DecodeState* state)
    {
        const u8* zigzagTable = state->zigzagTable;
        Arithmetic& arithmetic = state->arithmetic;
        BitBuffer& buffer = state->buffer;

        const int start = state->spectralStart;
        const int end = state->spectralEnd;

        u8* ac_stats = arithmetic.ac_stats[state->block[0].ac];
        u8 ac_K = arithmetic.ac_K[state->block[0].ac];

        // Figure F.20: Decode_AC_coefficients
        for (int k = start; k <= end; k++)
        {
            u8* st = ac_stats + 3 * (k - 1);

            if (arith_decode(arithmetic, buffer, st))
                break; // EOB flag

            while (arith_decode(arithmetic, buffer, st + 1) == 0)
            {
                st += 3;
                ++k;
            }

            // Figure F.21: Decoding nonzero value v
            // Figure F.22: Decoding the sign of v
            int sign = arith_decode(arithmetic, buffer, arithmetic.fixed_bin);
            st += 2;

            int m;

            // Figure F.23: Decoding the magnitude category of v
            if ((m = arith_decode(arithmetic, buffer, st)) != 0)
            {
                if (arith_decode(arithmetic, buffer, st))
                {
                    m <<= 1;
                    st = ac_stats + (k <= ac_K ? 189 : 217);

                    while (arith_decode(arithmetic, buffer, st))
                    {
                        m += m;
                        ++st;
                    }
                }
            }

            int v = m;

            // Figure F.24: Decoding the magnitude bit pattern of v
            st += 14;
            while (m >>= 1)
            {
                if (arith_decode(arithmetic, buffer, st))
                    v |= m;
            }

            v += 1; if (sign) v = -v;

            // Scale and output coefficient in natural order
            output[zigzagTable[k]] = s16(v << state->successiveLow);
        }
    }

    void arith_decode_ac_refine(s16* output, DecodeState* state)
    {
        const u8* zigzagTable = state->zigzagTable;
        Arithmetic& arithmetic = state->arithmetic;
        BitBuffer& buffer = state->buffer;

        const int start = state->spectralStart;
        const int end = state->spectralEnd;

        u8* ac_stats = arithmetic.ac_stats[state->block[0].ac];

        int p1 = 1 << state->successiveLow; //  1 in the bit position being coded
        int m1 = (-1) << state->successiveLow; // -1 in the bit position being coded

        int kex;

        // Establish EOBx (previous stage end-of-block) index
        for (kex = end; kex > 0; kex--)
        {
            if (output[zigzagTable[kex]])
                break;
        }

        for (int k = start; k <= end; k++)
        {
            u8* st = ac_stats + 3 * (k - 1);

            if (k > kex)
            {
                if (arith_decode(arithmetic, buffer, st))
                    break; // EOB flag
            }

            s16* coef = output + zigzagTable[k];
            for (;;)
            {
                if (*coef)
                {
                    // previously nonzero coef
                    if (arith_decode(arithmetic, buffer, st + 2))
                    {
                        if (*coef < 0)
                            *coef += s16(m1);
                        else
                            *coef += s16(p1);
                    }
                    break;
                }

                if (arith_decode(arithmetic, buffer, st + 1))
                {
                    // newly nonzero coef
                    if (arith_decode(arithmetic, buffer, arithmetic.fixed_bin))
                        *coef = s16(m1);
                    else
                        *coef = s16(p1);

                    break;
                }

                st += 3;
                ++k;
            }
        }
    }

    // ----------------------------------------------------------------------------
    // Arithmetic
    // ----------------------------------------------------------------------------

    Arithmetic::Arithmetic()
    {
        fixed_bin[0] = 113;
        std::memset(dc_L, 0, JPEG_NUM_ARITH_TBLS);
        std::memset(dc_U, 1, JPEG_NUM_ARITH_TBLS);
        std::memset(ac_K, 5, JPEG_NUM_ARITH_TBLS);
    }

    Arithmetic::~Arithmetic()
    {
    }

    void Arithmetic::restart(BitBuffer& buffer)
    {
        u8 v0 = get_byte(buffer);
        u8 v1 = get_byte(buffer);

        c = (v0 << 8) | v1;
        a = 0x10000;
        ct = 0;

        std::memset(dc_stats, 0, JPEG_NUM_ARITH_TBLS * JPEG_DC_STAT_BINS);
        std::memset(ac_stats, 0, JPEG_NUM_ARITH_TBLS * JPEG_AC_STAT_BINS);

        for (int i = 0; i < JPEG_MAX_COMPS_IN_SCAN; ++i)
        {
            last_dc_value[i] = 0;
            dc_context[i] = 0;
        }
    }

} // namespace mango::jpeg

// ----------------------------------------------------------------------------
// decode
// ----------------------------------------------------------------------------

namespace mango::jpeg
{

    // ----------------------------------------------------------------------------
    // utilities
    // ----------------------------------------------------------------------------

    static const u8 g_zigzag_table_inverse [] =
    {
         0,  1,  8, 16,  9,  2,  3, 10,
        17, 24, 32, 25, 18, 11,  4,  5,
        12, 19, 26, 33, 40, 48, 41, 34,
        27, 20, 13,  6,  7, 14, 21, 28,
        35, 42, 49, 56, 57, 50, 43, 36,
        29, 22, 15, 23, 30, 37, 44, 51,
        58, 59, 52, 45, 38, 31, 39, 46,
        53, 60, 61, 54, 47, 55, 62, 63,
    };

    inline bool isRestartMarker(const u8* p)
    {
        bool is = false;

        if (p[0] == 0xff)
        {
            int index = p[1] - 0xd0;
            is = index >= 0 && index <= 7;
        }

        return is;
    }

    void jpegPrintMemory(const u8* ptr)
    {
        for (int i = 0; i < 8; ++i)
        {
            printf("%.2x ", ptr[i - 8]);
        }
        printf("| ");
        for (int i = 0; i < 8; ++i)
        {
            printf("%.2x ", ptr[i]);
        }
        printf("\n");
    }

    static
    const SampleFormat g_format_table [] =
    {
        { JPEG_U8_Y,    LuminanceFormat(8, Format::UNORM, 8, 0) },
        { JPEG_U8_BGR,  Format(24, Format::UNORM, Format::BGR, 8, 8, 8) },
        { JPEG_U8_RGB,  Format(24, Format::UNORM, Format::RGB, 8, 8, 8) },
        { JPEG_U8_BGRA, Format(32, Format::UNORM, Format::BGRA, 8, 8, 8, 8) },
        { JPEG_U8_RGBA, Format(32, Format::UNORM, Format::RGBA, 8, 8, 8, 8) },
    };

    SampleFormat getSampleFormat(const Format& format)
    {
        // set default format
        SampleFormat result { JPEG_U8_RGBA, Format(32, Format::UNORM, Format::RGBA, 8, 8, 8, 8) };

        // find better match
        for (auto sf : g_format_table)
        {
            if (format == sf.format)
            {
                result = sf;
                break;
            }
        }

        return result;
    }

    // ----------------------------------------------------------------------------
    // BitBuffer
    // ----------------------------------------------------------------------------

    void BitBuffer::restart()
    {
        data = 0;
        remain = 0;
    }

    void BitBuffer::fill()
    {
#if defined(MANGO_CPU_64BIT) && defined(MANGO_ENABLE_SSE2)
        if (ptr + 8 <= end)
        {
            u64 x = uload64(ptr);
            const __m128i ref = _mm_cmpeq_epi8(_mm_setzero_si128(), _mm_setzero_si128());
            u32 mask = _mm_movemask_epi8(_mm_cmpeq_epi8(_mm_set_epi64x(0, x), ref));
            if (!mask)
            {
                data = (data << 48) | (byteswap(x) >> 16);
                remain += 48;
                ptr += 6;
                return;
            }
        }
#endif

        for (int i = 0; i < JPEG_REGISTER_FILL; ++i)
        {
            const u8* x = ptr;

            int a = ptr < end ? *ptr++ : 0;
            if (a == 0xff)
            {
                int b = ptr < end ? *ptr++ : 0;
                if (b)
                {
                    // Found a marker; keep returning zero until it has been processed
                    ptr = x;
                    a = 0;
                }
            }

            remain += 8;
            data = (data << 8) | a;
        }
    }

    // ----------------------------------------------------------------------------
    // Parser
    // ----------------------------------------------------------------------------

    Parser::Parser(ConstMemory memory)
        : quantTableVector(64 * JPEG_MAX_COMPS_IN_SCAN)
    {
        restartInterval = 0;
        restartCounter = 0;

        for (int i = 0; i < JPEG_MAX_COMPS_IN_SCAN; ++i)
        {
            quantTable[i].table = quantTableVector.data() + i * 64;
        }

        m_surface = nullptr;

        if (isJPEG(memory))
        {
            parse(memory, false);
        }
        else
        {
            header.setError("Incorrect SOI marker.");
        }
    }

    Parser::~Parser()
    {
    }

    bool Parser::isJPEG(ConstMemory memory) const
    {
        if (!memory.address || memory.size < 4)
            return false;

        if (uload16be(memory.address) != MARKER_SOI)
            return false;

#if 0
        // Scan for EOI marker
        const u8* p = memory.address + memory.size - 2;
        for (int i = 0; i < 32; ++i, --p)
        {
            u16 marker = uload16be(p);
            if (marker == MARKER_EOI)
                return true;
        }

        return false;
#else
        // Let's not be so picky.. EOI marker is optional, right?
        // (A lot of JPEG writers think so and we just have to deal with it :)
        return true;
#endif
    }

    const u8* Parser::stepMarker(const u8* p, const u8* end) const
    {
        if (p + 2 > end)
            return end;

        u16 size = uload16be(p);
        p += size;

        if (p + 1 > end)
            return end;

        p += (p[1] == 0xff); // HACK: some really ancient jpeg encoders encode markers sometimes as
                             // (0xff, 0xff, ID) ; this will skip to the "correct" 0xff (the second one)
        return p;
    }

    const u8* Parser::seekRestartMarker(const u8* start, const u8* end) const
    {
        const u8* p = start;
        --end; // marker is two bytes: don't look at last byte

        while (p < end)
        {
            p = mango::memchr(p, 0xff, end - p);
            if (p[1])
            {
                return p; // found a marker
            }
            p += 2; // skip: 0xff, 0x00
        }

        //if (*p != 0xff)
        ++p; // skip last byte (warning! if it is 0xff a marker can be potentially missed)
        debugPrint("  Seek: %d bytes\n", int(p - start));

        return p;
    }

    void Parser::processSOI()
    {
        debugPrint("[ SOI ]\n");
        restartInterval = 0;
    }

    void Parser::processEOI()
    {
        debugPrint("[ EOI ]\n");
    }

    void Parser::processCOM(const u8* p)
    {
        debugPrint("[ COM ]\n");
        MANGO_UNREFERENCED(p);
    }

    void Parser::processTEM(const u8* p)
    {
        debugPrint("[ TEM ]\n");
        MANGO_UNREFERENCED(p);
    }

    void Parser::processRES(const u8* p)
    {
        debugPrint("[ RES ]\n");

        // Reserved for jpeg extensions
        MANGO_UNREFERENCED(p);
    }

    void Parser::processJPG(const u8* p)
    {
        debugPrint("[ JPG ]\n");

        // Reserved for jpeg extensions
        MANGO_UNREFERENCED(p);
    }

    void Parser::processJPG(const u8* p, u16 marker)
    {
        debugPrint("[ JPG%d ]\n", int(marker - MARKER_JPG0));

        // Reserved for jpeg extensions
        MANGO_UNREFERENCED(p);
        MANGO_UNREFERENCED(marker);
    }

    void Parser::processAPP(const u8* p, u16 marker)
    {
        debugPrint("[ APP%d ]\n", int(marker - MARKER_APP0));

        int size = uload16be(p) - 2;
        p += 2;

        switch (marker)
        {
            case MARKER_APP0:
            {
                const u8 magicJFIF[] = { 0x4a, 0x46, 0x49, 0x46, 0 }; // 'JFIF', 0
                const u8 magicJFXX[] = { 0x4a, 0x46, 0x58, 0x58, 0 }; // 'JFXX', 0

                if (!std::memcmp(p, magicJFIF, 5) || !std::memcmp(p, magicJFXX, 5))
                {
                    p += 5;
                    size -= 5;

                    debugPrint("  JFIF: %i bytes\n", size);

                    int version = p[0] * 100 + p[1];
                    int units = p[2]; // 0 - no units, 1 - dots per inch, 2 - dots per cm
                    int Xdensity = (p[3] << 16) | p[4];
                    int Ydensity = (p[5] << 16) | p[6];
                    int Xthumbnail = p[7];
                    int Ythumbnail = p[8];

                    const char* unit_str = "";

                    switch (units)
                    {
                        case 1: unit_str = "dpi"; break;
                        case 2: unit_str = "cpi"; break;
                    }

                    debugPrint("    version: %i\n", version);
                    debugPrint("    density: %i x %i %s\n", Xdensity, Ydensity, unit_str);
                    debugPrint("    thumbnail: %i x %i\n", Xthumbnail, Ythumbnail);

                    // TODO: process thumbnail / store JFIF block
                    MANGO_UNREFERENCED(version);
                    MANGO_UNREFERENCED(Xdensity);
                    MANGO_UNREFERENCED(Ydensity);
                    MANGO_UNREFERENCED(Xthumbnail);
                    MANGO_UNREFERENCED(Ythumbnail);
                    MANGO_UNREFERENCED(unit_str);
                }

                break;
            }

            case MARKER_APP1:
            {
                const u8 magicExif0[] = { 0x45, 0x78, 0x69, 0x66, 0, 0 }; // 'Exif', 0, 0
                const u8 magicExif255[] = { 0x45, 0x78, 0x69, 0x66, 0, 0xff }; // 'Exif', 0, 0xff

                if (!std::memcmp(p, magicExif0, 6) || !std::memcmp(p, magicExif255, 6))
                {
                    p += 6;
                    size -= 6;
                    exif_memory = ConstMemory(p, size);
                    debugPrint("  EXIF: %d bytes\n", size);
                }

                // TODO: detect and support XMP

                break;
            }

            case MARKER_APP2:
            {
                const u8 magicICC[] = { 0x49, 0x43, 0x43, 0x5f, 0x50, 0x52, 0x4f, 0x46, 0x49, 0x4c, 0x45, 0 }; // 'ICC_PROFILE', 0

                if (!std::memcmp(p, magicICC, 12))
                {
                    // skip magic
                    p += 12;
                    size -= 12;

                    // read sequence information
                    u8 sequence_number = p[0];
                    u8 sequence_total = p[1];
                    p += 2;
                    size -= 2;

                    debugPrint("  ICC: %d / %d (%d bytes)\n", sequence_number, sequence_total, size);
                    MANGO_UNREFERENCED(sequence_number);
                    MANGO_UNREFERENCED(sequence_total);

                    // append ICC segment (JPEG markers have a maximum size and are split)
                    icc_buffer.append(p, size);
                }

                break;
            }

            case MARKER_APP3:
            {
                const u8 magicMETA[] = { 0x4d, 0x45, 0x54, 0x41, 0, 0 }; // 'META', 0, 0
                const u8 magicMeta[] = { 0x4d, 0x65, 0x74, 0x61, 0, 0 }; // 'Meta', 0, 0

                if (!std::memcmp(p, magicMETA, 6) || !std::memcmp(p, magicMeta, 6))
                {
                    p += 6;
                    size -= 6;
                    exif_memory = ConstMemory(p, size);
                    debugPrint("  EXIF: %d bytes\n", size);
                }

                break;
            }

            case MARKER_APP14:
            {
                const u8 magicAdobe[] = { 0x41, 0x64, 0x6f, 0x62, 0x65 }; // 'Adobe'
                if (size == 12 && !std::memcmp(p, magicAdobe, 5))
                {
                    u16 version = uload16be(p + 5);
                    //u16 flags0 = uload16be(p + 7);
                    //u16 flags1 = uload16be(p + 9);
                    u8 color_transform = p[11]; // 0 - CMYK, 1 - YCbCr, 2 - YCCK
                    if (color_transform <= 2)
                    {
                        processState.colorspace = ColorSpace(color_transform);
                    }
                    debugPrint("  Version: %d\n", version);
                    debugPrint("  ColorTransform: %d\n", color_transform);
                    MANGO_UNREFERENCED(version);
                    MANGO_UNREFERENCED(color_transform);
                }
                break;
            }
        }
    }

    void Parser::processSOF(const u8* p, u16 marker)
    {
        debugPrint("[ SOF%d ]\n", int(marker - MARKER_SOF0));

        is_baseline = (marker == MARKER_SOF0);
        is_progressive = false;
        is_multiscan = false;
        is_arithmetic = false;
        is_lossless = false;
        is_differential = false;

        u16 length = uload16be(p + 0);
        precision = p[2];
        ysize = uload16be(p + 3);
        xsize = uload16be(p + 5);
        components = p[7];
        p += 8;

        debugPrint("  Image: %d x %d x %d\n", xsize, ysize, precision);

        u16 correct_length = 8 + 3 * components;
        if (length != correct_length)
        {
            header.setError("Incorrect chunk length (%d, should be %d).", length, correct_length);
            return;
        }

        if (xsize <= 0 || ysize <= 0 || xsize > 65535 || ysize > 65535)
        {
            // NOTE: ysize of 0 is allowed in the specs but we won't
            header.setError("Incorrect dimensions (%d x %d)", xsize, ysize);
            return;
        }

        if (components < 1 || components > 4)
        {
            // NOTE: only progressive is required to have 1..4 components,
            //       other modes allow 1..255 but we are extra strict here :)
            header.setError("Incorrect number of components (%d)", components);
            return;
        }

        is_arithmetic = marker > MARKER_SOF7;
        m_compression = is_arithmetic ? "Arithmetic" : "Huffman";

        const char* encoding = "";
        switch (marker)
        {
            // Huffman
            case MARKER_SOF0:  encoding = "Baseline DCT"; break;
            case MARKER_SOF1:  encoding = "Extended sequential DCT"; break;
            case MARKER_SOF2:  encoding = "Progressive DCT"; break;
            case MARKER_SOF3:  encoding = "Lossless"; break;
            case MARKER_SOF5:  encoding = "Differential sequential DCT"; break;
            case MARKER_SOF6:  encoding = "Differential progressive DCT"; break;
            case MARKER_SOF7:  encoding = "Differential lossless"; break;
            // Arithmetic
            case MARKER_SOF9:  encoding = "Extended sequential DCT"; break;
            case MARKER_SOF10: encoding = "Progressive DCT"; break;
            case MARKER_SOF11: encoding = "Lossless"; break;
            case MARKER_SOF13: encoding = "Differential sequential DCT"; break;
            case MARKER_SOF14: encoding = "Differential progressive DCT"; break;
            case MARKER_SOF15: encoding = "Differential lossless"; break;
        }

        m_encoding = encoding;

        debugPrint("  Encoding: %s\n", m_encoding.c_str());
        debugPrint("  Compression: %s\n", m_compression.c_str());

        switch (marker)
        {
            case MARKER_SOF2:
            case MARKER_SOF6:
            case MARKER_SOF10:
            case MARKER_SOF14:
                is_progressive = true;
                break;
        }

        switch (marker)
        {
            case MARKER_SOF7:
            case MARKER_SOF15:
                is_differential = true;
                // fall-through
            case MARKER_SOF3:
            case MARKER_SOF11:
                is_lossless = true;
                break;
        }

        if (is_baseline)
        {
            if (precision != 8)
            {
                header.setError(makeString("Incorrect precision (%d, allowed: 8)", precision));
                return;
            }
        }
        else if (is_lossless)
        {
            if (precision < 2 || precision > 16)
            {
                header.setError(makeString("Incorrect precision (%d, allowed: 2..16)", precision));
                return;
            }
        }
        else
        {
            if (precision != 8 && precision != 12)
            {
                header.setError(makeString("Incorrect precision (%d, allowed: 8, 12)", precision));
                return;
            }
        }

        Hmax = 0;
        Vmax = 0;
        blocks_in_mcu = 0;
        int offset = 0;

        processState.frames = components;

        for (int i = 0; i < components; ++i)
        {
            if (offset >= JPEG_MAX_BLOCKS_IN_MCU)
            {
                header.setError("Incorrect blocks offset (%d >= %d).", offset, JPEG_MAX_BLOCKS_IN_MCU);
                return;
            }

            Frame& frame = processState.frame[i];

            frame.compid = p[0];
            u8 x = p[1];
            frame.Hsf = (x >> 4) & 0xf;
            frame.Vsf = (x >> 0) & 0xf;
            frame.Tq = p[2];
            frame.offset = offset;
            p += 3;

            u8 max_tq = is_lossless ? 0 : 3;
            if (frame.Tq > max_tq)
            {
                header.setError("Incorrect quantization table index (%d)", frame.Tq);
                return;
            }

            if (components == 1)
            {
                // Optimization: force block size to 8x8 with grayscale images
                frame.Hsf = 1;
                frame.Vsf = 1;
            }

            Hmax = std::max(Hmax, frame.Hsf);
            Vmax = std::max(Vmax, frame.Vsf);
            blocks_in_mcu += frame.Hsf * frame.Vsf;

            if (frame.Hsf < 1 || frame.Hsf > 4 || frame.Vsf < 1 || frame.Vsf > 4)
            {
                header.setError(makeString("Incorrect frame sampling rate (%d x %d)", frame.Hsf, frame.Vsf));
                return;
            }

            if (blocks_in_mcu > JPEG_MAX_BLOCKS_IN_MCU)
            {
                header.setError(makeString("Incorrect number of blocks in MCU (%d >= %d).", blocks_in_mcu, JPEG_MAX_BLOCKS_IN_MCU));
                return;
            }

            for (int y = 0; y < frame.Vsf; ++y)
            {
                for (int x = 0; x < frame.Hsf; ++x)
                {
                    processState.block[offset].qt = quantTable[frame.Tq].table;
                    if (!processState.block[offset].qt)
                    {
                        header.setError("No quantization table for index (%d)", frame.Tq);
                        return;
                    }

                    ++offset;
                }
            }

            debugPrint("  Frame: %d, compid: %d, Hsf: %d, Vsf: %d, Tq: %d, offset: %d\n",
                i, frame.compid, frame.Hsf, frame.Vsf, frame.Tq, frame.offset);

            frames.push_back(frame);
        }

        processState.blocks = offset;

        // Compute frame sampling factors against maximum sampling factor,
        // then convert them into power-of-two presentation.
        for (int i = 0; i < components; ++i)
        {
            Frame& frame = processState.frame[i];
            if (!frame.Hsf || !frame.Vsf)
            {
                header.setError("Incorrect sampling factors (%d x %d)", frame.Hsf, frame.Vsf);
                return;
            }
            frame.Hsf = u32_log2(Hmax / frame.Hsf);
            frame.Vsf = u32_log2(Vmax / frame.Vsf);
        }

        xblock = 8 * Hmax;
        yblock = 8 * Vmax;

        if (!xblock || !yblock)
        {
            header.setError("Incorrect dimensions (%d x %d)", xblock, yblock);
            return;
        }

        debugPrint("  Blocks per MCU: %d\n", blocks_in_mcu);
        debugPrint("  MCU size: %d x %d\n", xblock, yblock);

        // Align to next MCU boundary
        int xmask = xblock - 1;
        int ymask = yblock - 1;
        width  = (xsize + xmask) & ~xmask;
        height = (ysize + ymask) & ~ymask;

        // MCU resolution
        xmcu = width  / xblock;
        ymcu = height / yblock;
        mcus = xmcu * ymcu;

        debugPrint("  %d MCUs (%d x %d) -> (%d x %d)\n", mcus, xmcu, ymcu, xmcu * xblock, ymcu * yblock);
        debugPrint("  Image: %d x %d\n", xsize, ysize);

        // configure header
        header.width = xsize;
        header.height = ysize;
        header.format = components > 1 ? Format(24, Format::UNORM, Format::BGR, 8, 8, 8)
                                       : LuminanceFormat(8, Format::UNORM, 8, 0);

        MANGO_UNREFERENCED(length);
    }

    const u8* Parser::processSOS(const u8* p, const u8* end)
    {
        debugPrint("[ SOS ]\n");

        u16 length = uload16be(p);
        u8 components = p[2]; // Ns
        p += 3;

        u16 correct_length = 6 + 2 * components;
        if (length != correct_length)
        {
            header.setError("Incorrect chunk length (%d, should be %d).", length, correct_length);
            return p;
        }

        if (components < 1 || components > 4)
        {
            header.setError("Incorrect number of components (%d).", components);
            return p;
        }

        if (components != processState.frames && !is_progressive)
        {
            is_multiscan = true;

            // allocate blocks
            size_t num_blocks = size_t(mcus) * blocks_in_mcu;
            blockVector.resize(num_blocks * 64);
        }

        decodeState.comps_in_scan = components;

        debugPrint("  components: %i%s\n", components, is_multiscan ? " (MultiScan)" : "");
        MANGO_UNREFERENCED(length);

        decodeState.blocks = 0;

        for (int i = 0; i < components; ++i)
        {
            u8 cs = p[0]; // Scan component selector
            u8 x = p[1];
            p += 2;
            int dc = (x >> 4) & 0xf; // DC entropy coding table destination selector
            int ac = (x >> 0) & 0xf; // AC entropy coding table destination selector

            // default limits
            int max_dc = 3;
            int max_ac = 3;

            if (is_baseline)
            {
                max_dc = 1;
                max_ac = 1;
            }
            else if (is_lossless)
            {
                max_ac = 0;
            }

            if (dc > max_dc || ac > max_ac)
            {
                header.setError(makeString("Incorrect coding table selector (DC: %d, AC: %d).", dc, ac));
                return p;
            }

            // find frame
            Frame* frame = nullptr;
            int pred = 0;

            for (int j = 0; j < int(frames.size()); ++j)
            {
                if (frames[j].compid == cs)
                {
                    pred = j;
                    frame = &frames[j];
                }
            }

            if (!frame)
            {
                header.setError("Incorrect scan component selector (%d)", cs);
                return p;
            }

            scanFrame = frame;

            const int size = frame->Hsf * frame->Vsf;
            int offset = frame->offset;

            std::string cs_name;
            if (cs >= 32 && cs < 128)
            {
                cs_name = makeString(" (%c)", char(cs));
            }

            debugPrint("  Component: %i%s, DC: %i, AC: %i, offset: %d, size: %d\n",
                cs, cs_name.c_str(), dc, ac, frame->offset, size);

            for (int j = 0; j < size; ++j)
            {
                if (offset >= JPEG_MAX_BLOCKS_IN_MCU)
                {
                    header.setError("Incorrect number of blocks in MCU (%d >= %d).", offset, JPEG_MAX_BLOCKS_IN_MCU);
                    return p;
                }

                DecodeBlock& block = decodeState.block[decodeState.blocks];

                block.offset = offset * 64;
                block.pred = pred;
                block.dc = dc;
                block.ac = ac;

                debugPrint("      - offset: %d, pred: %d,\n", offset * 64, pred);
                ++offset;
                ++decodeState.blocks;
            }
        }

        int Ss = p[0];
        int Se = p[1];

        u8 x = p[2];
        int Al = (x >> 0) & 0xf;
        int Ah = (x >> 4) & 0xf;
        p += 3;

        decodeState.spectralStart = Ss;
        decodeState.spectralEnd = Se;
        decodeState.successiveLow = Al;
        decodeState.successiveHigh = Ah;

        debugPrint("  Spectral range: (%d, %d)\n", Ss, Se);

        // default limits
        int min_ss = 0;
        int max_ss = 0;

        int min_se = 63;
        int max_se = 63;

        int min_ah = 0;
        int max_ah = 0;

        int min_al = 0;
        int max_al = 0;

        if (is_progressive)
        {
            max_ss = 63;
            min_se = Ss;
            max_ah = 13;
            max_al = 13;
        }
        else if (is_lossless)
        {
            min_ss = 1; max_ss = 7;
            min_se = 0; max_se = 0;
            max_al = 15;
        }

        if (Ss < min_ss || Ss > max_ss ||
            Se < min_se || Se > max_se ||
            Ah < min_ah || Ah > max_ah ||
            Al < min_al || Al > max_al)
        {
            header.setError("Incorrect spectral range.");
            return p;
        }

        bool dc_scan = (decodeState.spectralStart == 0);
        bool refine_scan = (decodeState.successiveHigh != 0);

        decodeState.zigzagTable = g_zigzag_table_inverse;

        restartCounter = restartInterval;

        if (is_arithmetic)
        {
            Arithmetic& arithmetic = decodeState.arithmetic;

            decodeState.buffer.ptr = p;
            decodeState.buffer.end = end;

            // restart
            decodeState.buffer.restart();

            arithmetic.restart(decodeState.buffer);

            if (is_lossless)
            {
                decodeLossless();
            }
            else if (is_multiscan)
            {
                decodeState.decode = arith_decode_mcu;
                decodeMultiScan();
            }
            else if (is_progressive)
            {
                if (dc_scan)
                {
                    if (!refine_scan)
                    {
                        debugPrint("  * decode_dc_first()\n");
                        decodeState.decode = arith_decode_dc_first;
                    }
                    else
                    {
                        debugPrint("  * decode_dc_refine()\n");
                        decodeState.decode = arith_decode_dc_refine;
                    }
                }
                else
                {
                    if (!refine_scan)
                    {
                        debugPrint("  * decode_ac_first()\n");
                        decodeState.decode = arith_decode_ac_first;
                    }
                    else
                    {
                        debugPrint("  * decode_ac_refine()\n");
                        decodeState.decode = arith_decode_ac_refine;
                    }
                }
                decodeProgressive();
            }
            else
            {
                decodeState.decode = arith_decode_mcu;
                decodeSequential();
            }
        }
        else
        {
            Huffman& huffman = decodeState.huffman;

            decodeState.buffer.ptr = p;
            decodeState.buffer.end = end;

            // restart
            decodeState.buffer.restart();

            huffman.restart();

            if (is_lossless)
            {
                decodeLossless();
            }
            else if (is_multiscan)
            {
                decodeState.decode = huff_decode_mcu;
                decodeMultiScan();
            }
            else if (is_progressive)
            {
                if (dc_scan)
                {
                    if (!refine_scan)
                    {
                        debugPrint("  * decode_dc_first()\n");
                        decodeState.decode = huff_decode_dc_first;
                    }
                    else
                    {
                        debugPrint("  * decode_dc_refine()\n");
                        decodeState.decode = huff_decode_dc_refine;
                    }
                }
                else
                {
                    if (!refine_scan)
                    {
                        debugPrint("  * decode_ac_first()\n");
                        decodeState.decode = huff_decode_ac_first;
                    }
                    else
                    {
                        debugPrint("  * decode_ac_refine()\n");
                        decodeState.decode = huff_decode_ac_refine;
                    }
                }
                decodeProgressive();
            }
            else
            {
                decodeState.decode = huff_decode_mcu;
                decodeSequential();
            }
        }

        if (debugPrintIsEnable())
        {
            jpegPrintMemory(decodeState.buffer.ptr);
        }

        p = decodeState.buffer.ptr;

        return p;
    }

    void Parser::processDQT(const u8* p)
    {
        debugPrint("[ DQT ]\n");

        u16 Lq = uload16be(p); // Quantization table definition length
        p += 2;
        Lq -= 2;

        for ( ; Lq > 0; )
        {
            u8 x = *p++;
            u8 Pq = (x >> 4) & 0xf; // Quantization table element precision (0 = 8 bits, 1 = 16 bits)
            u8 Tq = (x >> 0) & 0xf; // Quantization table destination identifier (0..3)

            const int bits = (Pq + 1) * 8;
            debugPrint("  Quantization table #%i element precision: %i bits\n", Tq, bits);

            if (!is_lossless)
            {
                u8 max_pq = is_baseline ? 0 : 1;

                if (Pq > max_pq)
                {
                    header.setError("Incorrect quantization table element precision (%d bits)", Pq);
                    return;
                }

                if (Tq > 3)
                {
                    header.setError("Incorrect quantization table (%d)", Tq);
                    return;
                }
            }

            QuantTable& table = quantTable[Tq];
            table.bits = bits;

            switch (Pq)
            {
                case 0:
                    for (int i = 0; i < 64; ++i)
                    {
                        table.table[g_zigzag_table_inverse[i]] = *p++;
                    }
                    break;

                case 1:
                    for (int i = 0; i < 64; ++i)
                    {
                        table.table[g_zigzag_table_inverse[i]] = uload16be(p);
                        p += 2;
                    }
                    break;

                default:
                    break;
            }

            Lq -= (1 + (Pq + 1) * 64);
        }
    }

    void Parser::processDHT(const u8* p, const u8* end)
    {
        debugPrint("[ DHT ]\n");

        int Lh = uload16be(p); // Huffman table definition length
        p += 2;
        Lh -= 2;

        std::vector<HuffTable*> tables;

        for ( ; Lh > 0; )
        {
            u8 x = p[0];
            u8 Tc = (x >> 4) & 0xf; // Table class - 0 = DC table or lossless table, 1 = AC table.
            u8 Th = (x >> 0) & 0xf; // Huffman table identifier

            u8 max_tc = is_lossless ? 0 : 1;
            u8 max_th = is_baseline ? 1 : 3;

            if (Tc > max_tc)
            {
                header.setError(makeString("Incorrect huffman table class (%d)", Tc));
                return;
            }

            if (Th > max_th)
            {
                header.setError(makeString("Incorrect huffman table identifier (%d)", Th));
                return;
            }

            HuffTable& table = decodeState.huffman.table[Tc][Th];

            debugPrint("  Huffman table #%d table class: %d\n", Th, Tc);
            debugPrint("    codes: ");

            if (p >= end - 17)
            {
                header.setError("Data overflow.");
                return;
            }

            int count = 0;

            for (int i = 1; i <= 16; ++i)
            {
                u8 L = p[i]; // Number of Huffman codes of length i bits
                table.size[i] = L;
                count += L;
                debugPrint("%i ", L);
            }

            debugPrint("\n");

            p += 17;
            Lh -= 17;

            if (Lh < 0 || count > 256)
            {
                header.setError("Incorrect huffman table data.");
                return;
            }

            if (p >= end - count)
            {
                header.setError("Data overflow.");
                return;
            }

            std::memcpy(table.value, p, count);
            p += count;
            Lh -= count;

            if (Lh < 0)
            {
                header.setError("Incorrect huffman table data.");
                return;
            }

            tables.push_back(&table);
        }

        if (Lh != 0)
        {
            header.setError("Corrupted DHT data.");
            return;
        }

        // configure tables only after the data is determined to be correct
        for (HuffTable* table : tables)
        {
            if (!table->configure())
            {
                header.setError("Corrupted DHT - Huffman table generation failed.");
                return;
            }
        }
    }

    void Parser::processDAC(const u8* p)
    {
        debugPrint("[ DAC ]\n");

        u16 La = uload16be(p); // Arithmetic coding conditioning definition length
        p += 2;

        if (is_baseline)
        {
            header.setError("BaselineDCT does not support Arithmetic Coding tables.");
            return;
        }

        int n = (La - 2) / 2;

        debugPrint("  n: %i\n", n);

        if (n > 32)
        {
            header.setError("Too many DAC entries (%d).", n);
            return;
        }

        for (int i = 0; i < n; ++i)
        {
            u8 x = p[0];
            u8 Tc = (x >> 4) & 0xf; // Table class - 0 = DC table or lossless table, 1 = AC table
            u8 Tb = (x >> 0) & 0xf; // Arithmetic coding conditioning table destination identifier
            u8 Cs = p[1]; // Conditioning table value
            p += 2;

            u8 max_tc = is_lossless ? 0 : 1;
            u8 max_tb = 3;
            u8 min_cs = (Tc == 0 || is_lossless) ? 0 : 1;
            u8 max_cs = (Tc == 0 || is_lossless) ? 255 : 63;

            if (Tc > max_tc || Tb > max_tb)
            {
                header.setError(makeString("Incorrect Arithmetic table selector (Tc: %d, Tb: %d).", Tc, Tb));
                return;
            }

            if (Cs < min_cs || Cs > max_cs)
            {
                header.setError(makeString("Incorrect Arithmetic conditioning table (%d).", Cs));
                return;
            }

            switch (Tc)
            {
                case 0:
                    // DC table
                    decodeState.arithmetic.dc_L[Tb] = (Cs & 0xf);
                    decodeState.arithmetic.dc_U[Tb] = (Cs >> 4);
                    break;

                case 1:
                    // AC table
                    decodeState.arithmetic.ac_K[Tb] = Cs;
                    break;

                default:
                    header.setError("Incorrect Arithmetic table class (%d).", Tc);
                    return;
            }

            debugPrint("  Tc: %i, Tb: %i, Cs: %i\n", Tc, Tb, Cs);
        }
    }

    void Parser::processDNL(const u8* p)
    {
        debugPrint("[ DNL ]\n");

        u16 Ld = uload16be(p + 0); // Define number of lines segment length
        u16 NL = uload16be(p + 2); // Number of lines
        MANGO_UNREFERENCED(NL); // TODO: ysize = NL, no files to test with found yet..
        MANGO_UNREFERENCED(Ld);
    }

    void Parser::processDRI(const u8* p)
    {
        debugPrint("[ DRI ]\n");

        int Lh = uload16be(p + 0); // length
        if (Lh != 4)
        {
            // signal error
        }

        restartInterval = uload16be(p + 2); // number of MCU in restart interval
        debugPrint("  Restart interval: %i\n", restartInterval);
    }

    void Parser::processDHP(const u8* p)
    {
        debugPrint("[ DHP ]\n");

        // TODO: "Define Hierarchical Progression" marker
        MANGO_UNREFERENCED(p);
    }

    void Parser::processEXP(const u8* p)
    {
        debugPrint("[ EXP ]\n");

        u16 Le = uload16be(p); // Expand reference components segment length
        u8 x = p[2];
        u8 Eh = (x >> 4) & 0xf; // Expand horizontally
        u8 Ev = (x >> 0) & 0xf; // Expand vertically

        // Unsupported marker
        MANGO_UNREFERENCED(Le);
        MANGO_UNREFERENCED(Eh);
        MANGO_UNREFERENCED(Ev);
    }

    void Parser::parse(ConstMemory memory, bool decode)
    {
        const u8* end = memory.address + memory.size;
        const u8* p = memory.address;

        for ( ; p < end; )
        {
            if (!header)
            {
                // we are in error state -> abort parsing
                break;
            }

            u16 marker = uload16be(p);
            p += 2;

            u64 time0 = Time::us();

            switch (marker)
            {
                case MARKER_SOI:
                    processSOI();
                    break;

                case MARKER_EOI:
                    processEOI();
                    p = end; // terminate parsing
                    break;

                case MARKER_DHT:
                    processDHT(p, end);
                    p = stepMarker(p, end);
                    break;

                case MARKER_DAC:
                    processDAC(p);
                    p = stepMarker(p, end);
                    break;

                case MARKER_DQT:
                    processDQT(p);
                    p = stepMarker(p, end);
                    break;

                case MARKER_DNL:
                    processDNL(p);
                    p = stepMarker(p, end);
                    break;

                case MARKER_DRI:
                    processDRI(p);
                    p = stepMarker(p, end);
                    break;

                case MARKER_DHP:
                    processDHP(p);
                    p = stepMarker(p, end);
                    break;

                case MARKER_EXP:
                    processEXP(p);
                    p = stepMarker(p, end);
                    break;

                case MARKER_COM:
                    processCOM(p);
                    p = stepMarker(p, end);
                    break;

                case MARKER_TEM:
                    processTEM(p);
                    p = stepMarker(p, end);
                    break;

                case MARKER_RES:
                    processRES(p);
                    p = stepMarker(p, end);
                    break;

                case MARKER_APP0:
                case MARKER_APP1:
                case MARKER_APP2:
                case MARKER_APP3:
                case MARKER_APP4:
                case MARKER_APP5:
                case MARKER_APP6:
                case MARKER_APP7:
                case MARKER_APP8:
                case MARKER_APP9:
                case MARKER_APP10:
                case MARKER_APP11:
                case MARKER_APP12:
                case MARKER_APP13:
                case MARKER_APP14:
                case MARKER_APP15:
                    processAPP(p, marker);
                    p = stepMarker(p, end);
                    break;

                case MARKER_JPG:
                    processJPG(p);
                    p = stepMarker(p, end);
                    break;

                case MARKER_JPG0:
                case MARKER_JPG1:
                case MARKER_JPG2:
                case MARKER_JPG3:
                case MARKER_JPG4:
                case MARKER_JPG5:
                case MARKER_JPG6:
                case MARKER_JPG7:
                case MARKER_JPG8:
                case MARKER_JPG9:
                case MARKER_JPG10:
                case MARKER_JPG11:
                case MARKER_JPG12:
                case MARKER_JPG13:
                    processJPG(p, marker);
                    p = stepMarker(p, end);
                    break;

                case MARKER_SOF0:
                case MARKER_SOF1:
                case MARKER_SOF2:
                case MARKER_SOF3:
                case MARKER_SOF5:
                case MARKER_SOF6:
                case MARKER_SOF7:
                case MARKER_SOF9:
                case MARKER_SOF10:
                case MARKER_SOF11:
                case MARKER_SOF13:
                case MARKER_SOF14:
                case MARKER_SOF15:
                    processSOF(p, marker);
                    p = stepMarker(p, end);
                    if (!decode)
                    {
                        // parse header mode (no decoding)
                        scan_memory = ConstMemory(p, end - p);
                        p = end; // terminate parsing
                    }
                    break;

                case MARKER_RST0:
                case MARKER_RST1:
                case MARKER_RST2:
                case MARKER_RST3:
                case MARKER_RST4:
                case MARKER_RST5:
                case MARKER_RST6:
                case MARKER_RST7:
                    break;

                case MARKER_SOS:
                    if (decode)
                    {
                        p = processSOS(p, end);
                    }
                    break;

                default:
                    debugPrint("[ 0x%x ]\n", marker);
                    header.setError("Incorrect JPEG data.");
                    p = end; // terminate parsing
                    break;
            }

            u64 time1 = Time::us();
            debugPrint("  Time: %d us\n\n", int(time1 - time0));

            MANGO_UNREFERENCED(time0);
            MANGO_UNREFERENCED(time1);
        }
    }

    void Parser::restart()
    {
        if (is_arithmetic)
        {
            decodeState.arithmetic.restart(decodeState.buffer);
        }
        else
        {
            decodeState.huffman.restart();
        }

        // restart
        decodeState.buffer.restart();
    }

    bool Parser::handleRestart()
    {
        if (restartInterval > 0 && !--restartCounter)
        {
            restartCounter = restartInterval;

            if (isRestartMarker(decodeState.buffer.ptr))
            {
                restart();
                decodeState.buffer.ptr += 2;
                return true;
            }
        }

        return false;
    }

    void Parser::configureCPU(SampleType sample, const ImageDecodeOptions& options)
    {
        const char* simd = "";

        u64 flags = options.simd ? getCPUFlags() : 0;
        MANGO_UNREFERENCED(flags);

        // configure idct

        processState.idct = idct8;

#if defined(MANGO_ENABLE_NEON)
        if (flags & ARM_NEON)
        {
            processState.idct = idct_neon;
            m_idct_name = "iDCT: NEON";
        }
#endif

#if defined(MANGO_ENABLE_SSE2)
        if (flags & INTEL_SSE2)
        {
            processState.idct = idct_sse2;
            m_idct_name = "iDCT: SSE2";
        }
#endif

        if (precision == 12)
        {
            // Force 12 bit idct
            // This will round down to 8 bit precision until we have a 12 bit capable color conversion
            processState.idct = idct12;
            m_idct_name = "iDCT: 12 bit";
        }

        // configure block processing

        switch (sample)
        {
            case JPEG_U8_Y:
                processState.process_y           = process_y_8bit;
                processState.process_ycbcr       = process_ycbcr_8bit;
                processState.process_ycbcr_8x8   = nullptr;
                processState.process_ycbcr_8x16  = nullptr;
                processState.process_ycbcr_16x8  = nullptr;
                processState.process_ycbcr_16x16 = nullptr;
                break;
            case JPEG_U8_BGR:
                processState.process_y           = process_y_24bit;
                processState.process_ycbcr       = process_ycbcr_bgr;
                processState.process_ycbcr_8x8   = process_ycbcr_bgr_8x8;
                processState.process_ycbcr_8x16  = process_ycbcr_bgr_8x16;
                processState.process_ycbcr_16x8  = process_ycbcr_bgr_16x8;
                processState.process_ycbcr_16x16 = process_ycbcr_bgr_16x16;
                break;
            case JPEG_U8_RGB:
                processState.process_y           = process_y_24bit;
                processState.process_ycbcr       = process_ycbcr_rgb;
                processState.process_ycbcr_8x8   = process_ycbcr_rgb_8x8;
                processState.process_ycbcr_8x16  = process_ycbcr_rgb_8x16;
                processState.process_ycbcr_16x8  = process_ycbcr_rgb_16x8;
                processState.process_ycbcr_16x16 = process_ycbcr_rgb_16x16;
                break;
            case JPEG_U8_BGRA:
                processState.process_y           = process_y_32bit;
                processState.process_ycbcr       = process_ycbcr_bgra;
                processState.process_ycbcr_8x8   = process_ycbcr_bgra_8x8;
                processState.process_ycbcr_8x16  = process_ycbcr_bgra_8x16;
                processState.process_ycbcr_16x8  = process_ycbcr_bgra_16x8;
                processState.process_ycbcr_16x16 = process_ycbcr_bgra_16x16;
                break;
            case JPEG_U8_RGBA:
                processState.process_y           = process_y_32bit;
                processState.process_ycbcr       = process_ycbcr_rgba;
                processState.process_ycbcr_8x8   = process_ycbcr_rgba_8x8;
                processState.process_ycbcr_8x16  = process_ycbcr_rgba_8x16;
                processState.process_ycbcr_16x8  = process_ycbcr_rgba_16x8;
                processState.process_ycbcr_16x16 = process_ycbcr_rgba_16x16;
                break;
        }

        // CMYK / YCCK
        processState.process_cmyk = process_cmyk_bgra;

#if defined(MANGO_ENABLE_NEON)

        if (flags & ARM_NEON)
        {
            switch (sample)
            {
                case JPEG_U8_Y:
                    break;
                case JPEG_U8_BGR:
                    processState.process_ycbcr_8x8   = process_ycbcr_bgr_8x8_neon;
                    processState.process_ycbcr_8x16  = process_ycbcr_bgr_8x16_neon;
                    processState.process_ycbcr_16x8  = process_ycbcr_bgr_16x8_neon;
                    processState.process_ycbcr_16x16 = process_ycbcr_bgr_16x16_neon;
                    simd = "NEON";
                    break;
                case JPEG_U8_RGB:
                    processState.process_ycbcr_8x8   = process_ycbcr_rgb_8x8_neon;
                    processState.process_ycbcr_8x16  = process_ycbcr_rgb_8x16_neon;
                    processState.process_ycbcr_16x8  = process_ycbcr_rgb_16x8_neon;
                    processState.process_ycbcr_16x16 = process_ycbcr_rgb_16x16_neon;
                    simd = "NEON";
                    break;
                case JPEG_U8_BGRA:
                    processState.process_ycbcr_8x8   = process_ycbcr_bgra_8x8_neon;
                    processState.process_ycbcr_8x16  = process_ycbcr_bgra_8x16_neon;
                    processState.process_ycbcr_16x8  = process_ycbcr_bgra_16x8_neon;
                    processState.process_ycbcr_16x16 = process_ycbcr_bgra_16x16_neon;
                    simd = "NEON";
                    break;
                case JPEG_U8_RGBA:
                    processState.process_ycbcr_8x8   = process_ycbcr_rgba_8x8_neon;
                    processState.process_ycbcr_8x16  = process_ycbcr_rgba_8x16_neon;
                    processState.process_ycbcr_16x8  = process_ycbcr_rgba_16x8_neon;
                    processState.process_ycbcr_16x16 = process_ycbcr_rgba_16x16_neon;
                    simd = "NEON";
                    break;
            }
        }

#endif

#if defined(MANGO_ENABLE_SSE2)

        if (flags & INTEL_SSE2)
        {
            switch (sample)
            {
                case JPEG_U8_Y:
                    break;
                case JPEG_U8_BGR:
                    break;
                case JPEG_U8_RGB:
                    break;
                case JPEG_U8_BGRA:
                    processState.process_ycbcr_8x8   = process_ycbcr_bgra_8x8_sse2;
                    processState.process_ycbcr_8x16  = process_ycbcr_bgra_8x16_sse2;
                    processState.process_ycbcr_16x8  = process_ycbcr_bgra_16x8_sse2;
                    processState.process_ycbcr_16x16 = process_ycbcr_bgra_16x16_sse2;
                    simd = "SSE2";
                    break;
                case JPEG_U8_RGBA:
                    processState.process_ycbcr_8x8   = process_ycbcr_rgba_8x8_sse2;
                    processState.process_ycbcr_8x16  = process_ycbcr_rgba_8x16_sse2;
                    processState.process_ycbcr_16x8  = process_ycbcr_rgba_16x8_sse2;
                    processState.process_ycbcr_16x16 = process_ycbcr_rgba_16x16_sse2;
                    simd = "SSE2";
                    break;
            }
        }

#endif // MANGO_ENABLE_SSE2

#if defined(MANGO_ENABLE_SSE4_1)

        if (flags & INTEL_SSSE3)
        {
            switch (sample)
            {
                case JPEG_U8_Y:
                    break;
                case JPEG_U8_BGR:
                    processState.process_ycbcr_8x8   = process_ycbcr_bgr_8x8_ssse3;
                    processState.process_ycbcr_8x16  = process_ycbcr_bgr_8x16_ssse3;
                    processState.process_ycbcr_16x8  = process_ycbcr_bgr_16x8_ssse3;
                    processState.process_ycbcr_16x16 = process_ycbcr_bgr_16x16_ssse3;
                    simd = "SSSE3";
                    break;
                case JPEG_U8_RGB:
                    processState.process_ycbcr_8x8   = process_ycbcr_rgb_8x8_ssse3;
                    processState.process_ycbcr_8x16  = process_ycbcr_rgb_8x16_ssse3;
                    processState.process_ycbcr_16x8  = process_ycbcr_rgb_16x8_ssse3;
                    processState.process_ycbcr_16x16 = process_ycbcr_rgb_16x16_ssse3;
                    simd = "SSSE3";
                    break;
                case JPEG_U8_BGRA:
                    break;
                case JPEG_U8_RGBA:
                    break;
            }
        }

#endif // MANGO_ENABLE_SSE4_1

        std::string id;

        // determine jpeg type -> select innerloops
        switch (components)
        {
            case 1:
                processState.process = processState.process_y;
                id = "Color: Y";
                break;

            case 3:
                processState.process = processState.process_ycbcr;
                id = "Color: YCbCr";

                // detect optimized cases
                if (blocks_in_mcu <= 6)
                {
                    if (xblock == 8 && yblock == 8)
                    {
                        if (processState.process_ycbcr_8x8)
                        {
                            processState.process = processState.process_ycbcr_8x8;
                            id = makeString("Color: %s YCbCr 8x8", simd);
                        }
                    }

                    if (xblock == 8 && yblock == 16)
                    {
                        if (processState.process_ycbcr_8x16)
                        {
                            processState.process = processState.process_ycbcr_8x16;
                            id = makeString("Color: %s YCbCr 8x16", simd);
                        }
                    }

                    if (xblock == 16 && yblock == 8)
                    {
                        if (processState.process_ycbcr_16x8)
                        {
                            processState.process = processState.process_ycbcr_16x8;
                            id = makeString("Color: %s YCbCr 16x8", simd);
                        }
                    }

                    if (xblock == 16 && yblock == 16)
                    {
                        if (processState.process_ycbcr_16x16)
                        {
                            processState.process = processState.process_ycbcr_16x16;
                            id = makeString("Color: %s YCbCr 16x16", simd);
                        }
                    }
                }
                break;

            case 4:
                processState.process = processState.process_cmyk;
                id = "Color: CMYK";
                break;
        }

        m_ycbcr_name = id;
        debugPrint("  Decoder: %s\n", id.c_str());
    }

    ImageDecodeStatus Parser::decode(const Surface& target, const ImageDecodeOptions& options)
    {
        ImageDecodeStatus status;

        if (!scan_memory.address || !header)
        {
            status.setError(header.info);
            return status;
        }

        // determine if we need a full-surface temporary storage
        if (is_progressive || is_multiscan)
        {
            // allocate blocks
            size_t num_blocks = size_t(mcus) * blocks_in_mcu;
            blockVector.resize(num_blocks * 64);
        }

        // find best matching format
        SampleFormat sf = getSampleFormat(target.format);

        // configure innerloops based on CPU caps
        configureCPU(sf.sample, options);

        // configure multithreading
        m_hardware_concurrency = options.multithread ? ThreadPool::getHardwareConcurrency() : 1;

        if (is_lossless)
        {
            // lossless only supports L8 and BGRA
            if (components == 1)
            {
                sf.sample = JPEG_U8_Y;
                sf.format = LuminanceFormat(8, Format::UNORM, 8, 0);
            }
            else
            {
                sf.sample = JPEG_U8_BGRA;
                sf.format = Format(32, Format::UNORM, Format::BGRA, 8, 8, 8, 8);
            }
        }
        else if (components == 4)
        {
            // CMYK / YCCK is in the slow-path anyway so force BGRA
            sf.sample = JPEG_U8_BGRA;
            sf.format = Format(32, Format::UNORM, Format::BGRA, 8, 8, 8, 8);
        }

        status.direct = true;

        if (target.width != xsize || target.height != ysize)
        {
            status.direct = false;
        }

        if (target.format != sf.format)
        {
            status.direct = false;
        }

        // set decoding target surface
        m_surface = &target;

        std::unique_ptr<Bitmap> temp;

        if (!status.direct)
        {
            // create a temporary decoding target
            temp = std::make_unique<Bitmap>(width, height, sf.format);
            m_surface = temp.get();
        }

        parse(scan_memory, true);

        if (!header)
        {
            status.setError(header.info);
            return status;
        }

        if (is_progressive || is_multiscan)
        {
            finishProgressive();
        }

        if (!status.direct)
        {
            target.blit(0, 0, *m_surface);
        }

        blockVector.resize(0);
        status.info = getInfo();

        return status;
    }

    std::string Parser::getInfo() const
    {
        std::string info = m_encoding;

        info += ", ";
        info += m_compression;

        if (!m_idct_name.empty())
        {
            info += ", ";
            info += m_idct_name;
        }

        if (!m_ycbcr_name.empty())
        {
            info += ", ";
            info += m_ycbcr_name;
        }

        if (restartInterval > 0)
        {
            info += " [RST]";
        }

        return info;
    }

    int Parser::getTaskSize(int tasks) const
    {
        constexpr int max_threads = 64;
        const int threads = std::min(m_hardware_concurrency, max_threads);
        const int tasks_per_thread = threads > 1 ? std::max(tasks / threads, 1) : 0;
        //printf("Scheduling %d tasks in %d threads (%d tasks/thread)\n", tasks, threads, tasks_per_thread);
        return tasks_per_thread;
    }

    void Parser::decodeLossless()
    {
        // NOTE: need more test files to make this more conformant
        // NOTE: color sub-sampling is not supported (need test files)

        int predictor = decodeState.spectralStart;
        int pointTransform = decodeState.successiveLow;

        auto decodeFunction = huff_decode_mcu_lossless;
        int* previousDC = decodeState.huffman.last_dc_value;

        if (is_arithmetic)
        {
            decodeFunction = arith_decode_mcu_lossless;
            previousDC = decodeState.arithmetic.last_dc_value;
        }

        const int width = m_surface->width;
        const int height = m_surface->height;
        const int xlast = width - 1;
        const int components = decodeState.comps_in_scan;

        int initPredictor = 1 << (precision - pointTransform - 1);

        std::vector<int> scanLineCache[JPEG_MAX_BLOCKS_IN_MCU];

        for (int i = 0; i < components; ++i)
        {
            scanLineCache[i] = std::vector<int>(width + 1, 0);
        }

        bool first = true;

        for (int y = 0; y < height; ++y)
        {
            u8* image = m_surface->address<u8>(0, y);

            for (int x = 0; x < width; ++x)
            {
                s16 data[JPEG_MAX_BLOCKS_IN_MCU];

                decodeFunction(data, &decodeState);
                bool restarted = handleRestart();
                bool init = restarted | first;
                first = false;

                for (int currentComponent = 0; currentComponent < components; ++currentComponent)
                {
                    // predictors
                    int* cache = scanLineCache[currentComponent].data();
                    int a = data[currentComponent];
                    int b = cache[x + 1];
                    int c = cache[x + 0];

                    int s;

                    if (init)
                        s = initPredictor;
                    else if (predictor == 0)
                        s = 0;
                    else if (x == xlast)
                        s = cache[0];
                    else if (predictor == 1 || y == 0 || restarted)
                        s = a;
                    else if (predictor == 2)
                        s = b;
                    else if (predictor == 3)
                        s = c;
                    else if (predictor == 4)
                        s = a + b - c;
                    else if (predictor == 5)
                        s = a + ((b - c) >> 1);
                    else if (predictor == 6)
                        s = b + ((a - c) >> 1);
                    else if (predictor == 7)
                        s = (a + b) >> 1;
                    else
                        s = 0;

                    previousDC[currentComponent] = s;

                    cache[x] = data[currentComponent];
                    data[currentComponent] = data[currentComponent] >> (precision - 8);
                }

                if (components == 1)
                {
                    image[0] = byteclamp(data[0] + 128);
                    image += 1;
                }
                else
                {
                    image[0] = byteclamp(data[2] + 128); // blue
                    image[1] = byteclamp(data[1] + 128); // green
                    image[2] = byteclamp(data[0] + 128); // red
                    image[3] = 0xff;
                    image += 4;
                }
            }
        }
    }

    void Parser::decodeSequential()
    {
        int n = getTaskSize(ymcu);
        if (n)
        {
            decodeSequentialMT(n);
        }
        else
        {
            decodeSequentialST();
        }
    }

    void Parser::decodeSequentialST()
    {
        const size_t stride = m_surface->stride;
        const int bytes_per_pixel = m_surface->format.bytes();
        const size_t xstride = bytes_per_pixel * xblock;
        const size_t ystride = stride * yblock;

        u8* image = m_surface->image;

        ProcessFunc process = processState.process;

        const int xmcu_last = xmcu - 1;
        const int ymcu_last = ymcu - 1;

        const int xclip = xsize % xblock;
        const int yclip = ysize % yblock;
        const int xblock_last = xclip ? xclip : xblock;
        const int yblock_last = yclip ? yclip : yblock;

        s16 data[JPEG_MAX_SAMPLES_IN_MCU];

        for (int y = 0; y < ymcu_last; ++y)
        {
            u8* dest = image;
            image += ystride;

            for (int x = 0; x < xmcu_last; ++x)
            {
                decodeState.decode(data, &decodeState);
                handleRestart();
                process(dest, stride, data, &processState, xblock, yblock);
                dest += xstride;
            }

            // last column
            decodeState.decode(data, &decodeState);
            handleRestart();
            process_and_clip(dest, stride, data, xblock_last, yblock);
            dest += xstride;
        }

        // last row
        for (int x = 0; x < xmcu_last; ++x)
        {
            decodeState.decode(data, &decodeState);
            handleRestart();
            process_and_clip(image, stride, data, xblock, yblock_last);
            image += xstride;
        }

        // last mcu
        decodeState.decode(data, &decodeState);
        handleRestart();
        process_and_clip(image, stride, data, xblock_last, yblock_last);
    }

    void Parser::decodeSequentialMT(int N)
    {
        ConcurrentQueue queue("jpeg.sequential", Priority::HIGH);

        if (!restartInterval)
        {
            const int mcu_data_size = blocks_in_mcu * 64;

            for (int y = 0; y < ymcu; y += N)
            {
                const int y0 = y;
                const int y1 = std::min(y + N, ymcu);
                const int count = (y1 - y0) * xmcu;
                debugPrint("  Process: [%d, %d] --> ThreadPool.\n", y0, y1 - 1);

                void* aligned_ptr = aligned_malloc(count * mcu_data_size * sizeof(s16));
                s16* data = reinterpret_cast<s16*>(aligned_ptr);

                for (int i = 0; i < count; ++i)
                {
                    decodeState.decode(data + i * mcu_data_size, &decodeState);
                }

                // enqueue task
                queue.enqueue([=]
                {
                    process_range(y0, y1, data);
                    aligned_free(data);
                });
            }
        }
        else
        {
            const u8* p = decodeState.buffer.ptr;

            const size_t stride = m_surface->stride;
            const int bytes_per_pixel = m_surface->format.bytes();
            const size_t xstride = bytes_per_pixel * xblock;
            const size_t ystride = stride * yblock;

            u8* image = m_surface->image;

            for (int i = 0; i < mcus; i += restartInterval)
            {
                // enqueue task
                queue.enqueue([=]
                {
                    AlignedStorage<s16> data(JPEG_MAX_SAMPLES_IN_MCU);

                    DecodeState state = decodeState;
                    state.buffer.ptr = p;

                    const int left = std::min(restartInterval, mcus - i);

                    const int xmcu_last = xmcu - 1;
                    const int ymcu_last = ymcu - 1;

                    const int xclip = xsize % xblock;
                    const int yclip = ysize % yblock;
                    const int xblock_last = xclip ? xclip : xblock;
                    const int yblock_last = yclip ? yclip : yblock;

                    for (int j = 0; j < left; ++j)
                    {
                        int n = i + j;

                        state.decode(data, &state);

                        int x = n % xmcu;
                        int y = n / xmcu;
                        u8* dest = image + y * ystride + x * xstride;

                        int width = x == xmcu_last ? xblock_last : xblock;
                        int height = y == ymcu_last ? yblock_last : yblock;

                        process_and_clip(dest, stride, data, width, height);
                    }
                });

                // seek next restart marker
                p = seekRestartMarker(p, decodeState.buffer.end);
                if (isRestartMarker(p))
                    p += 2;
            }

            decodeState.buffer.ptr = p;
        }
    }

    void Parser::decodeMultiScan()
    {
        s16* data = blockVector;
        data += decodeState.block[0].offset;

        for (int i = 0; i < mcus; ++i)
        {
            decodeState.decode(data, &decodeState);
            handleRestart();
            data += blocks_in_mcu * 64;
        }
    }

    void Parser::decodeProgressive()
    {
        if (decodeState.spectralStart == 0)
        {
            if (decodeState.comps_in_scan == 1 && decodeState.blocks > 1)
            {
                decodeState.block[0].offset = 0;
                decodeState.blocks = 1;

                decodeProgressiveAC();
            }
            else
            {
                decodeProgressiveDC();
            }
        }
        else
        {
            decodeProgressiveAC();
        }
    }

    void Parser::decodeProgressiveDC()
    {
        if (restartInterval)
        {
            s16* data = blockVector;

            const u8* p = decodeState.buffer.ptr;

            ConcurrentQueue queue("jpeg.progressive", Priority::HIGH);

            for (int i = 0; i < mcus; i += restartInterval)
            {
                // enqueue task
                queue.enqueue([=]
                {
                    DecodeState state = decodeState;
                    state.buffer.ptr = p;

                    s16* dest = data + i * blocks_in_mcu * 64;

                    const int left = std::min(restartInterval, mcus - i);
                    for (int j = 0; j < left; ++j)
                    {
                        state.decode(dest, &state);
                        dest += blocks_in_mcu * 64;
                    }
                });

                // seek next restart marker
                p = seekRestartMarker(p, decodeState.buffer.end);
                if (isRestartMarker(p))
                    p += 2;
            }

            decodeState.buffer.ptr = p;
        }
        else
        {
            s16* data = blockVector;

            for (int i = 0; i < mcus; ++i)
            {
                decodeState.decode(data, &decodeState);
                data += blocks_in_mcu * 64;
            }
        }
    }

    void Parser::decodeProgressiveAC()
    {
        if (restartInterval)
        {
            s16* data = blockVector;

            const int hsf = u32_log2(scanFrame->Hsf);
            const int vsf = u32_log2(scanFrame->Vsf);
            const int hsize = (Hmax >> hsf) * 8;
            const int vsize = (Vmax >> vsf) * 8;

            debugPrint("    hf: %i x %i, log2: %i x %i\n", 1 << hsf, 1 << vsf, hsf, vsf);
            debugPrint("    bs: %i x %i  scanSize: %d\n", hsize, vsize, decodeState.blocks);

            const int scan_offset = scanFrame->offset;

            const int xs = ((xsize + hsize - 1) / hsize);
            const int ys = ((ysize + vsize - 1) / vsize);
            const int cnt = xs * ys;

            debugPrint("    blocks: %d x %d (%d x %d)\n", xs, ys, xs * hsize, ys * vsize);

            MANGO_UNREFERENCED(xs);
            MANGO_UNREFERENCED(ys);
            MANGO_UNREFERENCED(hsize);
            MANGO_UNREFERENCED(vsize);

            const int HMask = (1 << hsf) - 1;
            const int VMask = (1 << vsf) - 1;

            ConcurrentQueue queue("jpeg.progressive", Priority::HIGH);

            const u8* p = decodeState.buffer.ptr;

            for (int i = 0; i < cnt; i += restartInterval)
            {
                // enqueue task
                queue.enqueue([=]
                {
                    DecodeState state = decodeState;
                    state.buffer.ptr = p;

                    const int left = std::min(restartInterval, mcus - i);
                    for (int j = 0; j < left; ++j)
                    {
                        int n = i + j;
                        int x = n % xmcu;
                        int y = n / xmcu;

                        int mcu_yoffset = (y >> vsf) * xmcu;
                        int block_yoffset = ((y & VMask) << hsf) + scan_offset;

                        int mcu_offset = (mcu_yoffset + (x >> hsf)) * blocks_in_mcu;
                        int block_offset = (x & HMask) + block_yoffset;
                        s16* dest = data + (block_offset + mcu_offset) * 64;

                        state.decode(dest, &state);
                    }
                });

                // seek next restart marker
                p = seekRestartMarker(p, decodeState.buffer.end);
                if (isRestartMarker(p))
                    p += 2;
            }

            decodeState.buffer.ptr = p;
        }
        else
        {
            s16* data = blockVector;

            const int hsf = u32_log2(scanFrame->Hsf);
            const int vsf = u32_log2(scanFrame->Vsf);
            const int hsize = (Hmax >> hsf) * 8;
            const int vsize = (Vmax >> vsf) * 8;

            debugPrint("    hf: %i x %i, log2: %i x %i\n", 1 << hsf, 1 << vsf, hsf, vsf);
            debugPrint("    bs: %i x %i  scanSize: %d\n", hsize, vsize, decodeState.blocks);

            const int scan_offset = scanFrame->offset;

            const int xs = ((xsize + hsize - 1) / hsize);
            const int ys = ((ysize + vsize - 1) / vsize);

            debugPrint("    blocks: %d x %d (%d x %d)\n", xs, ys, xs * hsize, ys * vsize);

            const int HMask = (1 << hsf) - 1;
            const int VMask = (1 << vsf) - 1;

            for (int y = 0; y < ys; ++y)
            {
                int mcu_yoffset = (y >> vsf) * xmcu;
                int block_yoffset = ((y & VMask) << hsf) + scan_offset;

                for (int x = 0; x < xs; ++x)
                {
                    int mcu_offset = (mcu_yoffset + (x >> hsf)) * blocks_in_mcu;
                    int block_offset = (x & HMask) + block_yoffset;
                    s16* mcudata = data + (block_offset + mcu_offset) * 64;

                    decodeState.decode(mcudata, &decodeState);
                }
            }
        }
    }

    void Parser::finishProgressive()
    {
        int n = getTaskSize(ymcu);
        if (n)
        {
            ConcurrentQueue queue("jpeg.progressive", Priority::HIGH);

            size_t mcu_stride = size_t(xmcu) * blocks_in_mcu * 64;

            for (int y = 0; y < ymcu; y += n)
            {
                const int y0 = y;
                const int y1 = std::min(y + n, ymcu);

                s16* data = blockVector + y0 * mcu_stride;

                debugPrint("  Process: [%d, %d] --> ThreadPool.\n", y0, y1 - 1);

                // enqueue task
                queue.enqueue([=]
                {
                    process_range(y0, y1, data);
                });
            }
        }
        else
        {
            s16* data = blockVector;
            process_range(0, ymcu, data);
        }
    }

    void Parser::process_range(int y0, int y1, const s16* data)
    {
        const size_t stride = m_surface->stride;
        const size_t bytes_per_pixel = m_surface->format.bytes();
        const size_t xstride = bytes_per_pixel * xblock;
        const size_t ystride = stride * yblock;

        u8* image = m_surface->image;

        const int mcu_data_size = blocks_in_mcu * 64;

        const int xmcu_last = xmcu - 1;
        const int ymcu_last = ymcu - 1;

        const int xclip = xsize % xblock;
        const int yclip = ysize % yblock;
        const int xblock_last = xclip ? xclip : xblock;
        const int yblock_last = yclip ? yclip : yblock;

        for (int y = y0; y < y1; ++y)
        {
            u8* dest = image + y * ystride;
            int height = y == ymcu_last ? yblock_last : yblock;

            for (int x = 0; x < xmcu_last; ++x)
            {
                process_and_clip(dest, stride, data, xblock, height);
                data += mcu_data_size;
                dest += xstride;
            }

            // last column
            process_and_clip(dest, stride, data, xblock_last, height);
            data += mcu_data_size;
            dest += xstride;
        }
    }

    void Parser::process_and_clip(u8* dest, size_t stride, const s16* data, int width, int height)
    {
        if (xblock != width || yblock != height)
        {
            u8 temp[JPEG_MAX_SAMPLES_IN_MCU * 4];

            const int bytes_per_scan = width * m_surface->format.bytes();
            const int block_stride = xblock * 4;
            u8* src = temp;

            processState.process(temp, block_stride, data, &processState, width, height);

            // clipping
            for (int y = 0; y < height; ++y)
            {
                std::memcpy(dest, src, bytes_per_scan);
                src += block_stride;
                dest += stride;
            }
        }
        else
        {
            // fast-path (no clipping required)
            processState.process(dest, stride, data, &processState, width, height);
        }
    }

} // namespace mango::jpeg

// ----------------------------------------------------------------------------
// encode
// ----------------------------------------------------------------------------

namespace
{
    using namespace mango;
    using namespace mango::math;
    using namespace jpeg;

    constexpr int BLOCK_SIZE = 64;

    // Table K.1 – Luminance quantization table

    const u8 g_luminance_quant_table [] =
    {
        16, 11, 10, 16,  24,  40,  51,  61,
        12, 12, 14, 19,  26,  58,  60,  55,
        14, 13, 16, 24,  40,  57,  69,  56,
        14, 17, 22, 29,  51,  87,  80,  62,
        18, 22, 37, 56,  68, 109, 103,  77,
        24, 35, 55, 64,  81, 104, 113,  92,
        49, 64, 78, 87, 103, 121, 120, 101,
        72, 92, 95, 98, 112, 100, 103,  99
    };

    // Table K.2 – Chrominance quantization table

    const u8 g_chrominance_quant_table [] =
    {
        17, 18, 24, 47, 99, 99, 99, 99,
        18, 21, 26, 66, 99, 99, 99, 99,
        24, 26, 56, 99, 99, 99, 99, 99,
        47, 66, 99, 99, 99, 99, 99, 99,
        99, 99, 99, 99, 99, 99, 99, 99,
        99, 99, 99, 99, 99, 99, 99, 99,
        99, 99, 99, 99, 99, 99, 99, 99,
        99, 99, 99, 99, 99, 99, 99, 99
    };

    // Table K.3 – Table for luminance DC coefficient differences

    const u32 g_luminance_dc_code_table [] =
    {
        0x00000000, 0x00000004, 0x0000000c, 0x00000020, 0x00000050, 0x000000c0, 0x00000380, 0x00000f00, 0x00003e00, 0x0000fc00, 0x0003f800, 0x000ff000,
    };

    const u16 g_luminance_dc_size_table [] =
    {
        0x0002, 0x0004, 0x0005, 0x0006, 0x0007, 0x0008, 0x000a, 0x000c, 0x000e, 0x0010, 0x0012, 0x0014,
    };

    // Table K.4 – Table for chrominance DC coefficient differences

    const u32 g_chrominance_dc_code_table [] =
    {
        0x00000000, 0x00000002, 0x00000008, 0x00000030, 0x000000e0, 0x000003c0, 0x00000f80, 0x00003f00, 0x0000fe00, 0x0003fc00, 0x000ff800, 0x003ff000,
    };

    const u16 g_chrominance_dc_size_table [] =
    {
        0x0002, 0x0003, 0x0004, 0x0006, 0x0008, 0x000a, 0x000c, 0x000e, 0x0010, 0x0012, 0x0014, 0x0016,
    };

    // Table K.5 – Table for luminance AC coefficients

    alignas(64)
    const u32 g_luminance_ac_code_table [] =
    {
        0x0000000a, 0x000007f9, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000000, 0x00000018, 0x00000038, 0x00000074, 0x00000076, 0x000000f4, 0x000000f6, 0x000001f4, 0x000003f0, 0x000003f2, 0x000003f4, 0x000007f2, 0x000007f4, 0x00000ff0, 0x0001ffd6, 0x0001ffea,
        0x00000004, 0x0000006c, 0x000003e4, 0x000007dc, 0x00000fe0, 0x00001fdc, 0x00003fd8, 0x00003fdc, 0x0001ff00, 0x0003fef8, 0x0003ff1c, 0x0003ff40, 0x0003ff64, 0x0003ff88, 0x0003ffb0, 0x0003ffd8,
        0x00000020, 0x000003c8, 0x00001fb8, 0x00007fa8, 0x0007fcb0, 0x0007fcf0, 0x0007fd30, 0x0007fd70, 0x0007fdb0, 0x0007fdf8, 0x0007fe40, 0x0007fe88, 0x0007fed0, 0x0007ff18, 0x0007ff68, 0x0007ffb8,
        0x000000b0, 0x00001f60, 0x0000ff40, 0x000ff8f0, 0x000ff970, 0x000ff9f0, 0x000ffa70, 0x000ffaf0, 0x000ffb70, 0x000ffc00, 0x000ffc90, 0x000ffd20, 0x000ffdb0, 0x000ffe40, 0x000ffee0, 0x000fff80,
        0x00000340, 0x0000fec0, 0x001ff120, 0x001ff200, 0x001ff300, 0x001ff400, 0x001ff500, 0x001ff600, 0x001ff700, 0x001ff820, 0x001ff940, 0x001ffa60, 0x001ffb80, 0x001ffca0, 0x001ffde0, 0x001fff20,
        0x00001e00, 0x003fe100, 0x003fe280, 0x003fe440, 0x003fe640, 0x003fe840, 0x003fea40, 0x003fec40, 0x003fee40, 0x003ff080, 0x003ff2c0, 0x003ff500, 0x003ff740, 0x003ff980, 0x003ffc00, 0x003ffe80,
        0x00007c00, 0x007fc280, 0x007fc580, 0x007fc900, 0x007fcd00, 0x007fd100, 0x007fd500, 0x007fd900, 0x007fdd00, 0x007fe180, 0x007fe600, 0x007fea80, 0x007fef00, 0x007ff380, 0x007ff880, 0x007ffd80,
        0x0003f600, 0x00ff8600, 0x00ff8c00, 0x00ff9300, 0x00ff9b00, 0x00ffa300, 0x00ffab00, 0x00ffb300, 0x00ffbb00, 0x00ffc400, 0x00ffcd00, 0x00ffd600, 0x00ffdf00, 0x00ffe800, 0x00fff200, 0x00fffc00,
        0x01ff0400, 0x01ff0e00, 0x01ff1a00, 0x01ff2800, 0x01ff3800, 0x01ff4800, 0x01ff5800, 0x01ff6800, 0x01ff7800, 0x01ff8a00, 0x01ff9c00, 0x01ffae00, 0x01ffc000, 0x01ffd200, 0x01ffe600, 0x01fffa00,
        0x03fe0c00, 0x03fe2000, 0x03fe3800, 0x03fe5400, 0x03fe7400, 0x03fe9400, 0x03feb400, 0x03fed400, 0x03fef400, 0x03ff1800, 0x03ff3c00, 0x03ff6000, 0x03ff8400, 0x03ffa800, 0x03ffd000, 0x03fff800,
    };

    alignas(64)
    const u16 g_luminance_ac_size_table [] =
    {
        0x0004, 0x000b, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
        0x0003, 0x0005, 0x0006, 0x0007, 0x0007, 0x0008, 0x0008, 0x0009, 0x000a, 0x000a, 0x000a, 0x000b, 0x000b, 0x000c, 0x0011, 0x0011,
        0x0004, 0x0007, 0x000a, 0x000b, 0x000c, 0x000d, 0x000e, 0x000e, 0x0011, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012,
        0x0006, 0x000a, 0x000d, 0x000f, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013,
        0x0008, 0x000d, 0x0010, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014,
        0x000a, 0x0010, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015,
        0x000d, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016,
        0x000f, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017,
        0x0012, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018,
        0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019,
        0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a,
    };

    // Table K.6 – Table for chrominance AC coefficients

    alignas(64)
    const u32 g_chrominance_ac_code_table [] =
    {
        0x00000000, 0x000003fa, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
        0x00000002, 0x00000016, 0x00000034, 0x00000036, 0x00000074, 0x00000076, 0x000000f2, 0x000000f4, 0x000001f2, 0x000003ee, 0x000003f0, 0x000003f2, 0x000003f4, 0x00000ff2, 0x00007fc0, 0x0000ff86,
        0x00000010, 0x000000e4, 0x000003dc, 0x000003e0, 0x000007d8, 0x00000fe4, 0x00001fdc, 0x00001fe0, 0x0003fedc, 0x0003ff00, 0x0003ff24, 0x0003ff48, 0x0003ff6c, 0x0003ff90, 0x0003ffb4, 0x0003ffd8,
        0x00000050, 0x000007b0, 0x00001fb8, 0x00001fc0, 0x0007fcb8, 0x0007fcf8, 0x0007fd38, 0x0007fd78, 0x0007fdc0, 0x0007fe08, 0x0007fe50, 0x0007fe98, 0x0007fee0, 0x0007ff28, 0x0007ff70, 0x0007ffb8,
        0x00000180, 0x00001f50, 0x0000ff60, 0x0000ff70, 0x000ff980, 0x000ffa00, 0x000ffa80, 0x000ffb00, 0x000ffb90, 0x000ffc20, 0x000ffcb0, 0x000ffd40, 0x000ffdd0, 0x000ffe60, 0x000ffef0, 0x000fff80,
        0x00000320, 0x0000fec0, 0x000ff840, 0x001ff220, 0x001ff320, 0x001ff420, 0x001ff520, 0x001ff620, 0x001ff740, 0x001ff860, 0x001ff980, 0x001ffaa0, 0x001ffbc0, 0x001ffce0, 0x001ffe00, 0x001fff20,
        0x00000e00, 0x0003fd40, 0x003fe300, 0x003fe480, 0x003fe680, 0x003fe880, 0x003fea80, 0x003fec80, 0x003feec0, 0x003ff100, 0x003ff340, 0x003ff580, 0x003ff7c0, 0x003ffa00, 0x003ffc40, 0x003ffe80,
        0x00003c00, 0x007fc400, 0x007fc680, 0x007fc980, 0x007fcd80, 0x007fd180, 0x007fd580, 0x007fd980, 0x007fde00, 0x007fe280, 0x007fe700, 0x007feb80, 0x007ff000, 0x007ff480, 0x007ff900, 0x007ffd80,
        0x0001f400, 0x00ff8900, 0x00ff8e00, 0x00ff9400, 0x00ff9c00, 0x00ffa400, 0x00ffac00, 0x00ffb400, 0x00ffbd00, 0x00ffc600, 0x00ffcf00, 0x00ffd800, 0x00ffe100, 0x00ffea00, 0x00fff300, 0x00fffc00,
        0x0007ec00, 0x01ff1400, 0x01ff1e00, 0x01ff2a00, 0x01ff3a00, 0x01ff4a00, 0x01ff5a00, 0x01ff6a00, 0x01ff7c00, 0x01ff8e00, 0x01ffa000, 0x01ffb200, 0x01ffc400, 0x01ffd600, 0x01ffe800, 0x01fffa00,
        0x003fd000, 0x03fe2c00, 0x03fe4000, 0x03fe5800, 0x03fe7800, 0x03fe9800, 0x03feb800, 0x03fed800, 0x03fefc00, 0x03ff2000, 0x03ff4400, 0x03ff6800, 0x03ff8c00, 0x03ffb000, 0x03ffd400, 0x03fff800,
    };

    alignas(64)
    const u16 g_chrominance_ac_size_table [] =
    {
        0x0002, 0x000a, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
        0x0003, 0x0005, 0x0006, 0x0006, 0x0007, 0x0007, 0x0008, 0x0008, 0x0009, 0x000a, 0x000a, 0x000a, 0x000a, 0x000c, 0x000f, 0x0010,
        0x0005, 0x0008, 0x000a, 0x000a, 0x000b, 0x000c, 0x000d, 0x000d, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012, 0x0012,
        0x0007, 0x000b, 0x000d, 0x000d, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013, 0x0013,
        0x0009, 0x000d, 0x0010, 0x0010, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014, 0x0014,
        0x000a, 0x0010, 0x0014, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015, 0x0015,
        0x000c, 0x0012, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016, 0x0016,
        0x000e, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017, 0x0017,
        0x0011, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018,
        0x0013, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019, 0x0019,
        0x0016, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a, 0x001a,
    };

    const u8 g_marker_data [] =
    {
        0xFF, 0xC4, 0x00, 0x1F, 0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
        0xFF, 0xC4, 0x00, 0xB5, 0x10, 0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7D, 0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06,
        0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xA1, 0x08, 0x23, 0x42, 0xB1, 0xC1, 0x15, 0x52, 0xD1, 0xF0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0A, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x25,
        0x26, 0x27, 0x28, 0x29, 0x2A, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67,
        0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6,
        0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE1, 0xE2,
        0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA,
        0xFF, 0xC4, 0x00, 0x1F, 0x01, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B,
        0xFF, 0xC4, 0x00, 0xB5, 0x11, 0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04, 0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77, 0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41,
        0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33, 0x52, 0xF0, 0x15, 0x62, 0x72, 0xD1, 0x0A, 0x16, 0x24, 0x34, 0xE1, 0x25, 0xF1, 0x17, 0x18,
        0x19, 0x1A, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x63, 0x64, 0x65, 0x66,
        0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4,
        0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA,
        0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA,
    };

    // ----------------------------------------------------------------------------
    // HuffmanEncoder
    // ----------------------------------------------------------------------------

    static inline
    u8* writeStuffedBytes(u8* output, DataType code, int count)
    {
        code = byteswap(code);
        for (int i = 0; i < count; ++i)
        {
            u8 value = u8(code);
            code >>= 8;
            *output++ = value;

            // always write the stuff byte
            *output = 0;

            // .. but advance ptr only when it actually was one
            output += (value == 0xff);
        }
        return output;
    }

#ifdef MANGO_CPU_64BIT

    static inline
    u8* flushStuffedBytes(u8* output, DataType code)
    {
        if (code & 0x8080808080808080ull & ~(code + 0x0101010101010101ull))
        {
            output = writeStuffedBytes(output, code, 8);
        }
        else
        {
            ustore64be(output, code);
            output += 8;
        }
        return output;
    }

#else

    static inline
    u8* flushStuffedBytes(u8* output, DataType code)
    {
        if (code & 0x80808080 & ~(code + 0x01010101))
        {
            output = writeStuffedBytes(output, code, 4);
        }
        else
        {
            ustore32be(output, code);
            output += 4;
        }
        return output;
    }

#endif

    struct EncodeBuffer : Buffer
    {
        std::atomic<bool> ready { false };
    };

    struct HuffmanEncoder
    {
        DataType code;
        int space;

        int last_dc_value[3] = { 0, 0, 0 };
        void (*fdct)(s16* dest, const s16* data, const s16* qtable);

        HuffmanEncoder()
        {
            code = 0;
            space = JPEG_REGISTER_BITS;
        }

        ~HuffmanEncoder()
        {
        }

        u8* putBits(u8* output, DataType data, int numbits)
        {
            space -= numbits;
            if (space < 0)
            {
                output = flushStuffedBytes(output, code | (data >> -space));
                space += JPEG_REGISTER_BITS;
                code = data << space;
            }
            else
            {
                code |= (data << space);
            }
            return output;
        }

        u8* flush(u8* output)
        {
            int count = ((JPEG_REGISTER_BITS - space) + 7) >> 3;
            output = writeStuffedBytes(output, code, count);
            return output;
        }
    };

    struct jpegEncoder
    {
        Surface m_surface;
        SampleType m_sample;
        const ImageEncodeOptions& m_options;

        int mcu_width;
        int mcu_height;
        int mcu_stride;
        int horizontal_mcus;
        int vertical_mcus;
        int cols_in_right_mcus;
        int rows_in_bottom_mcus;

        u8 luminance_qtable[BLOCK_SIZE];
        u8 chrominance_qtable[BLOCK_SIZE];
        AlignedStorage<s16> inverse_luminance_qtable;
        AlignedStorage<s16> inverse_chrominance_qtable;

        // MCU configuration
        struct Channel
        {
            int component;
            const s16* qtable;
            const u32* dc_code;
            const u16* dc_size;
            const u32* ac_code;
            const u16* ac_size;
        };
        Channel channel[3];
        int components;

        std::string info;

        using ReadFunc = void (*)(s16*, const u8*, size_t, int, int);

        void (*read_8x8) (s16* block, const u8* input, size_t stride, int rows, int cols);
        void (*read)     (s16* block, const u8* input, size_t stride, int rows, int cols);
        void (*fdct)     (s16* dest, const s16* data, const s16* qtable);
        u8*  (*encode)   (HuffmanEncoder& encoder, u8* p, const s16* input, const Channel& channel);

        jpegEncoder(const Surface& surface, SampleType sample, const ImageEncodeOptions& options);
        ~jpegEncoder();

        void encodeInterval(EncodeBuffer& buffer, const u8* src, size_t stride, ReadFunc read_func, int rows);
        void writeMarkers(BigEndianStream& p);
        ImageEncodeStatus encodeImage(Stream& stream);
    };

    // ----------------------------------------------------------------------------
    // fdct_scalar
    // ----------------------------------------------------------------------------

    static
    void fdct_scalar(s16* dest, const s16* data, const s16* qtable)
    {
        constexpr s16 c1 = 1420; // cos 1PI/16 * root(2)
        constexpr s16 c2 = 1338; // cos 2PI/16 * root(2)
        constexpr s16 c3 = 1204; // cos 3PI/16 * root(2)
        constexpr s16 c5 = 805;  // cos 5PI/16 * root(2)
        constexpr s16 c6 = 554;  // cos 6PI/16 * root(2)
        constexpr s16 c7 = 283;  // cos 7PI/16 * root(2)

        s16 temp[64];

        for (int i = 0; i < 8; ++i)
        {
            s16 x8 = data [0] + data [7];
            s16 x0 = data [0] - data [7];
            s16 x7 = data [1] + data [6];
            s16 x1 = data [1] - data [6];
            s16 x6 = data [2] + data [5];
            s16 x2 = data [2] - data [5];
            s16 x5 = data [3] + data [4];
            s16 x3 = data [3] - data [4];
            s16 x4 = x8 + x5;
            x8 = x8 - x5;
            x5 = x7 + x6;
            x7 = x7 - x6;
            temp[i * 8 + 0] = x4 + x5;
            temp[i * 8 + 4] = x4 - x5;
            temp[i * 8 + 2] = (x8 * c2 + x7 * c6) >> 10;
            temp[i * 8 + 6] = (x8 * c6 - x7 * c2) >> 10;
            temp[i * 8 + 7] = (x0 * c7 - x1 * c5 + x2 * c3 - x3 * c1) >> 10;
            temp[i * 8 + 5] = (x0 * c5 - x1 * c1 + x2 * c7 + x3 * c3) >> 10;
            temp[i * 8 + 3] = (x0 * c3 - x1 * c7 - x2 * c1 - x3 * c5) >> 10;
            temp[i * 8 + 1] = (x0 * c1 + x1 * c3 + x2 * c5 + x3 * c7) >> 10;
            data += 8;
        }

        for (int i = 0; i < 8; ++i)
        {
            s16 x8 = temp [i +  0] + temp [i + 56];
            s16 x0 = temp [i +  0] - temp [i + 56];
            s16 x7 = temp [i +  8] + temp [i + 48];
            s16 x1 = temp [i +  8] - temp [i + 48];
            s16 x6 = temp [i + 16] + temp [i + 40];
            s16 x2 = temp [i + 16] - temp [i + 40];
            s16 x5 = temp [i + 24] + temp [i + 32];
            s16 x3 = temp [i + 24] - temp [i + 32];
            s16 x4 = x8 + x5;
            x8 = x8 - x5;
            x5 = x7 + x6;
            x7 = x7 - x6;
            s16 v0 = (x4 + x5) >> 3;
            s16 v4 = (x4 - x5) >> 3;
            s16 v2 = (x8 * c2 + x7 * c6) >> 13;
            s16 v6 = (x8 * c6 - x7 * c2) >> 13;
            s16 v7 = (x0 * c7 - x1 * c5 + x2 * c3 - x3 * c1) >> 13;
            s16 v5 = (x0 * c5 - x1 * c1 + x2 * c7 + x3 * c3) >> 13;
            s16 v3 = (x0 * c3 - x1 * c7 - x2 * c1 - x3 * c5) >> 13;
            s16 v1 = (x0 * c1 + x1 * c3 + x2 * c5 + x3 * c7) >> 13;
            dest[i + 8 * 0] = (v0 * qtable[i + 8 * 0] + 0x4000) >> 15;
            dest[i + 8 * 1] = (v1 * qtable[i + 8 * 1] + 0x4000) >> 15;
            dest[i + 8 * 2] = (v2 * qtable[i + 8 * 2] + 0x4000) >> 15;
            dest[i + 8 * 3] = (v3 * qtable[i + 8 * 3] + 0x4000) >> 15;
            dest[i + 8 * 4] = (v4 * qtable[i + 8 * 4] + 0x4000) >> 15;
            dest[i + 8 * 5] = (v5 * qtable[i + 8 * 5] + 0x4000) >> 15;
            dest[i + 8 * 6] = (v6 * qtable[i + 8 * 6] + 0x4000) >> 15;
            dest[i + 8 * 7] = (v7 * qtable[i + 8 * 7] + 0x4000) >> 15;
        }
    }

#if defined(MANGO_ENABLE_SSE2)

    // ----------------------------------------------------------------------------
    // fdct sse2
    // ----------------------------------------------------------------------------

    static inline void interleave16(__m128i& a, __m128i& b)
    {
        __m128i c = a;
        a = _mm_unpacklo_epi16(a, b);
        b = _mm_unpackhi_epi16(c, b);
    }

    #define JPEG_TRANSPOSE16() \
        interleave16(v0, v4); \
        interleave16(v2, v6); \
        interleave16(v1, v5); \
        interleave16(v3, v7); \
        interleave16(v0, v2); \
        interleave16(v1, v3); \
        interleave16(v4, v6); \
        interleave16(v5, v7); \
        interleave16(v0, v1); \
        interleave16(v2, v3); \
        interleave16(v4, v5); \
        interleave16(v6, v7)

    #define JPEG_CONST16_SSE2(x, y) \
        _mm_setr_epi16(x, y, x, y, x, y, x, y)

    #define JPEG_TRANSFORM_SSE2(n) { \
        __m128i a_lo; \
        __m128i a_hi; \
        __m128i b_lo; \
        __m128i b_hi; \
        \
        a_lo = _mm_madd_epi16(x87_lo, c26p); \
        a_hi = _mm_madd_epi16(x87_hi, c26p); \
        a_lo = _mm_srai_epi32(a_lo, n); \
        a_hi = _mm_srai_epi32(a_hi, n); \
        v2 = _mm_packs_epi32(a_lo, a_hi); \
        \
        a_lo = _mm_madd_epi16(x87_lo, c62n); \
        a_hi = _mm_madd_epi16(x87_hi, c62n); \
        a_lo = _mm_srai_epi32(a_lo, n); \
        a_hi = _mm_srai_epi32(a_hi, n); \
        v6 = _mm_packs_epi32(a_lo, a_hi); \
        \
        a_lo = _mm_madd_epi16(x01_lo, c75n); \
        a_hi = _mm_madd_epi16(x01_hi, c75n); \
        b_lo = _mm_madd_epi16(x23_lo, c31n); \
        b_hi = _mm_madd_epi16(x23_hi, c31n); \
        a_lo = _mm_add_epi32(a_lo, b_lo); \
        a_hi = _mm_add_epi32(a_hi, b_hi); \
        a_lo = _mm_srai_epi32(a_lo, n); \
        a_hi = _mm_srai_epi32(a_hi, n); \
        v7 = _mm_packs_epi32(a_lo, a_hi); \
        \
        a_lo = _mm_madd_epi16(x01_lo, c51n); \
        a_hi = _mm_madd_epi16(x01_hi, c51n); \
        b_lo = _mm_madd_epi16(x23_lo, c73p); \
        b_hi = _mm_madd_epi16(x23_hi, c73p); \
        a_lo = _mm_add_epi32(a_lo, b_lo); \
        a_hi = _mm_add_epi32(a_hi, b_hi); \
        a_lo = _mm_srai_epi32(a_lo, n); \
        a_hi = _mm_srai_epi32(a_hi, n); \
        v5 = _mm_packs_epi32(a_lo, a_hi); \
        \
        a_lo = _mm_madd_epi16(x01_lo, c37n); \
        a_hi = _mm_madd_epi16(x01_hi, c37n); \
        b_lo = _mm_madd_epi16(x23_lo, c15p); \
        b_hi = _mm_madd_epi16(x23_hi, c15p); \
        a_lo = _mm_sub_epi32(a_lo, b_lo); \
        a_hi = _mm_sub_epi32(a_hi, b_hi); \
        a_lo = _mm_srai_epi32(a_lo, n); \
        a_hi = _mm_srai_epi32(a_hi, n); \
        v3 = _mm_packs_epi32(a_lo, a_hi); \
        \
        a_lo = _mm_madd_epi16(x01_lo, c13p); \
        a_hi = _mm_madd_epi16(x01_hi, c13p); \
        b_lo = _mm_madd_epi16(x23_lo, c57p); \
        b_hi = _mm_madd_epi16(x23_hi, c57p); \
        a_lo = _mm_add_epi32(a_lo, b_lo); \
        a_hi = _mm_add_epi32(a_hi, b_hi); \
        a_lo = _mm_srai_epi32(a_lo, n); \
        a_hi = _mm_srai_epi32(a_hi, n); \
        v1 = _mm_packs_epi32(a_lo, a_hi); }

    static inline
    __m128i quantize(__m128i v, __m128i q, __m128i one, __m128i bias)
    {
        __m128i lo = _mm_madd_epi16(_mm_unpacklo_epi16(v, one), _mm_unpacklo_epi16(q, bias));
        __m128i hi = _mm_madd_epi16(_mm_unpackhi_epi16(v, one), _mm_unpackhi_epi16(q, bias));
        lo = _mm_srai_epi32(lo, 15);
        hi = _mm_srai_epi32(hi, 15);
        v = _mm_packs_epi32(lo, hi);
        return v;
    }

    static
    void fdct_sse2(s16* dest, const s16* data, const s16* qtable)
    {
        constexpr s16 c1 = 1420; // cos 1PI/16 * root(2)
        constexpr s16 c2 = 1338; // cos 2PI/16 * root(2)
        constexpr s16 c3 = 1204; // cos 3PI/16 * root(2)
        constexpr s16 c5 = 805;  // cos 5PI/16 * root(2)
        constexpr s16 c6 = 554;  // cos 6PI/16 * root(2)
        constexpr s16 c7 = 283;  // cos 7PI/16 * root(2)

        const __m128i c26p = JPEG_CONST16_SSE2(c2, c6);
        const __m128i c62n = JPEG_CONST16_SSE2(c6,-c2);
        const __m128i c75n = JPEG_CONST16_SSE2(c7,-c5);
        const __m128i c31n = JPEG_CONST16_SSE2(c3,-c1);
        const __m128i c51n = JPEG_CONST16_SSE2(c5,-c1);
        const __m128i c73p = JPEG_CONST16_SSE2(c7, c3);
        const __m128i c37n = JPEG_CONST16_SSE2(c3,-c7);
        const __m128i c15p = JPEG_CONST16_SSE2(c1, c5);
        const __m128i c13p = JPEG_CONST16_SSE2(c1, c3);
        const __m128i c57p = JPEG_CONST16_SSE2(c5, c7);

        // load

        const __m128i* s = reinterpret_cast<const __m128i *>(data);
        __m128i v0 = _mm_loadu_si128(s + 0);
        __m128i v1 = _mm_loadu_si128(s + 1);
        __m128i v2 = _mm_loadu_si128(s + 2);
        __m128i v3 = _mm_loadu_si128(s + 3);
        __m128i v4 = _mm_loadu_si128(s + 4);
        __m128i v5 = _mm_loadu_si128(s + 5);
        __m128i v6 = _mm_loadu_si128(s + 6);
        __m128i v7 = _mm_loadu_si128(s + 7);

        // pass 1

        JPEG_TRANSPOSE16();

        __m128i x8 = _mm_add_epi16(v0, v7);
        __m128i x7 = _mm_add_epi16(v1, v6);
        __m128i x6 = _mm_add_epi16(v2, v5);
        __m128i x5 = _mm_add_epi16(v3, v4);
        __m128i x0 = _mm_sub_epi16(v0, v7);
        __m128i x1 = _mm_sub_epi16(v1, v6);
        __m128i x2 = _mm_sub_epi16(v2, v5);
        __m128i x3 = _mm_sub_epi16(v3, v4);

        __m128i x4;
        x4 = _mm_add_epi16(x8, x5);
        x8 = _mm_sub_epi16(x8, x5);
        x5 = _mm_add_epi16(x7, x6);
        x7 = _mm_sub_epi16(x7, x6);

        __m128i x87_lo = _mm_unpacklo_epi16(x8, x7);
        __m128i x87_hi = _mm_unpackhi_epi16(x8, x7);
        __m128i x01_lo = _mm_unpacklo_epi16(x0, x1);
        __m128i x01_hi = _mm_unpackhi_epi16(x0, x1);
        __m128i x23_lo = _mm_unpacklo_epi16(x2, x3);
        __m128i x23_hi = _mm_unpackhi_epi16(x2, x3);

        v0 = _mm_add_epi16(x4, x5);
        v4 = _mm_sub_epi16(x4, x5);

        JPEG_TRANSFORM_SSE2(10);

        // pass 2

        JPEG_TRANSPOSE16();

        x8 = _mm_add_epi16(v0, v7);
        x7 = _mm_add_epi16(v1, v6);
        x6 = _mm_add_epi16(v2, v5);
        x5 = _mm_add_epi16(v3, v4);
        x0 = _mm_sub_epi16(v0, v7);
        x1 = _mm_sub_epi16(v1, v6);
        x2 = _mm_sub_epi16(v2, v5);
        x3 = _mm_sub_epi16(v3, v4);

        x4 = _mm_add_epi16(x8, x5);
        x8 = _mm_sub_epi16(x8, x5);
        x5 = _mm_add_epi16(x7, x6);
        x7 = _mm_sub_epi16(x7, x6);

        x87_lo = _mm_unpacklo_epi16(x8, x7);
        x87_hi = _mm_unpackhi_epi16(x8, x7);
        x01_lo = _mm_unpacklo_epi16(x0, x1);
        x01_hi = _mm_unpackhi_epi16(x0, x1);
        x23_lo = _mm_unpacklo_epi16(x2, x3);
        x23_hi = _mm_unpackhi_epi16(x2, x3);

        v0 = _mm_srai_epi16(_mm_add_epi16(x4, x5), 3);
        v4 = _mm_srai_epi16(_mm_sub_epi16(x4, x5), 3);

        JPEG_TRANSFORM_SSE2(13);

        // quantize

        const __m128i one = _mm_set1_epi16(1);
        const __m128i bias = _mm_set1_epi16(0x4000);
        const __m128i* q = reinterpret_cast<const __m128i*>(qtable);

        v0 = quantize(v0, q[0], one, bias);
        v1 = quantize(v1, q[1], one, bias);
        v2 = quantize(v2, q[2], one, bias);
        v3 = quantize(v3, q[3], one, bias);
        v4 = quantize(v4, q[4], one, bias);
        v5 = quantize(v5, q[5], one, bias);
        v6 = quantize(v6, q[6], one, bias);
        v7 = quantize(v7, q[7], one, bias);

        // store

        __m128i* d = reinterpret_cast<__m128i *>(dest);
        _mm_storeu_si128(d + 0, v0);
        _mm_storeu_si128(d + 1, v1);
        _mm_storeu_si128(d + 2, v2);
        _mm_storeu_si128(d + 3, v3);
        _mm_storeu_si128(d + 4, v4);
        _mm_storeu_si128(d + 5, v5);
        _mm_storeu_si128(d + 6, v6);
        _mm_storeu_si128(d + 7, v7);
    }

#endif // defined(MANGO_ENABLE_SSE2)

#if defined(MANGO_ENABLE_AVX2__disabled)

    // ----------------------------------------------------------------------------
    // fdct_avx2
    // ----------------------------------------------------------------------------

    static inline
    void transpose_8x8_avx2(__m256i& v0, __m256i& v1, __m256i& v2, __m256i& v3)
    {
        __m256i x0 = _mm256_permute2x128_si256(v0, v2, 0x20);
        __m256i x1 = _mm256_permute2x128_si256(v0, v2, 0x31);
        __m256i x2 = _mm256_permute2x128_si256(v1, v3, 0x20);
        __m256i x3 = _mm256_permute2x128_si256(v1, v3, 0x31);
        __m256i v4 = _mm256_unpacklo_epi16(x0, x1);
        __m256i v5 = _mm256_unpackhi_epi16(x0, x1);
        __m256i v6 = _mm256_unpacklo_epi16(x2, x3);
        __m256i v7 = _mm256_unpackhi_epi16(x2, x3);
        x0 = _mm256_unpacklo_epi32(v4, v6);
        x1 = _mm256_unpackhi_epi32(v4, v6);
        x2 = _mm256_unpacklo_epi32(v5, v7);
        x3 = _mm256_unpackhi_epi32(v5, v7);
        v0 = _mm256_permute4x64_epi64(x0, 0xd8);
        v1 = _mm256_permute4x64_epi64(x1, 0xd8);
        v2 = _mm256_permute4x64_epi64(x2, 0xd8);
        v3 = _mm256_permute4x64_epi64(x3, 0xd8);
    }

    static inline
    __m256i quantize(__m256i v, __m256i q, __m256i one, __m256i bias)
    {
        __m256i lo = _mm256_madd_epi16(_mm256_unpacklo_epi16(v, one), _mm256_unpacklo_epi16(q, bias));
        __m256i hi = _mm256_madd_epi16(_mm256_unpackhi_epi16(v, one), _mm256_unpackhi_epi16(q, bias));
        lo = _mm256_srai_epi32(lo, 15);
        hi = _mm256_srai_epi32(hi, 15);
        v = _mm256_packs_epi32(lo, hi);
        return v;
    }

    #define JPEG_CONST16_AVX2(x, y) \
        _mm256_setr_epi16(x, y, x, y, x, y, x, y, x, y, x, y, x, y, x, y)

    template <int nbits>
    __m256i transform_sub_term(__m256i c0, __m256i c1, __m256i c2, __m256i c3, __m256i c4, __m256i c5)
    {
        __m256i a;
        __m256i b;

        a = _mm256_madd_epi16(c0, c1);
        a = _mm256_srai_epi32(a, nbits);
        __m128i v2 = _mm_packs_epi32(_mm256_extracti128_si256(a, 0), _mm256_extracti128_si256(a, 1));

        a = _mm256_madd_epi16(c2, c3);
        b = _mm256_madd_epi16(c4, c5);
        a = _mm256_sub_epi32(a, b); // <-- sub
        a = _mm256_srai_epi32(a, nbits);
        __m128i v3 = _mm_packs_epi32(_mm256_extracti128_si256(a, 0), _mm256_extracti128_si256(a, 1));

        return _mm256_setr_m128i(v2, v3);
    }

    template <int nbits>
    __m256i transform_add_term(__m256i c0, __m256i c1, __m256i c2, __m256i c3, __m256i c4, __m256i c5)
    {
        __m256i a;
        __m256i b;

        a = _mm256_madd_epi16(c0, c1);
        a = _mm256_srai_epi32(a, nbits);
        __m128i v2 = _mm_packs_epi32(_mm256_extracti128_si256(a, 0), _mm256_extracti128_si256(a, 1));

        a = _mm256_madd_epi16(c2, c3);
        b = _mm256_madd_epi16(c4, c5);
        a = _mm256_add_epi32(a, b); // <-- add
        a = _mm256_srai_epi32(a, nbits);
        __m128i v3 = _mm_packs_epi32(_mm256_extracti128_si256(a, 0), _mm256_extracti128_si256(a, 1));

        return _mm256_setr_m128i(v2, v3);
    }

    template <int nbits>
    __m256i transform_term(__m128i v, __m256i c0, __m256i c1, __m256i c2, __m256i c3)
    {
        __m256i a;
        __m256i b;

        a = _mm256_madd_epi16(c0, c1);
        b = _mm256_madd_epi16(c2, c3);
        a = _mm256_add_epi32(a, b);
        a = _mm256_srai_epi32(a, nbits);
        __m128i v5 = _mm_packs_epi32(_mm256_extracti128_si256(a, 0), _mm256_extracti128_si256(a, 1));
        return _mm256_setr_m128i(v, v5);
    }

    static inline
    __m256i unpack(__m256i  x)
    {
        __m128i x0 = _mm256_extracti128_si256(x, 0);
        __m128i x1 = _mm256_extracti128_si256(x, 1);
        __m128i lo = _mm_unpacklo_epi16(x0, x1);
        __m128i hi = _mm_unpackhi_epi16(x0, x1);
        return _mm256_setr_m128i(lo, hi);
    }

    static
    void fdct_avx2(s16* dest, const s16* data, const s16* qtable)
    {
        constexpr s16 c1 = 1420; // cos 1PI/16 * root(2)
        constexpr s16 c2 = 1338; // cos 2PI/16 * root(2)
        constexpr s16 c3 = 1204; // cos 3PI/16 * root(2)
        constexpr s16 c5 = 805;  // cos 5PI/16 * root(2)
        constexpr s16 c6 = 554;  // cos 6PI/16 * root(2)
        constexpr s16 c7 = 283;  // cos 7PI/16 * root(2)

        const __m256i c26p = JPEG_CONST16_AVX2(c2, c6);
        const __m256i c62n = JPEG_CONST16_AVX2(c6,-c2);
        const __m256i c75n = JPEG_CONST16_AVX2(c7,-c5);
        const __m256i c31n = JPEG_CONST16_AVX2(c3,-c1);
        const __m256i c51n = JPEG_CONST16_AVX2(c5,-c1);
        const __m256i c73p = JPEG_CONST16_AVX2(c7, c3);
        const __m256i c37n = JPEG_CONST16_AVX2(c3,-c7);
        const __m256i c15p = JPEG_CONST16_AVX2(c1, c5);
        const __m256i c13p = JPEG_CONST16_AVX2(c1, c3);
        const __m256i c57p = JPEG_CONST16_AVX2(c5, c7);

        // load

        const __m256i* s = reinterpret_cast<const __m256i *>(data);
        __m256i s0 = _mm256_loadu_si256(s + 0);
        __m256i s1 = _mm256_loadu_si256(s + 1);
        __m256i s2 = _mm256_loadu_si256(s + 2);
        __m256i s3 = _mm256_loadu_si256(s + 3);

        transpose_8x8_avx2(s0, s1, s2, s3);

        // pass 1

        __m256i v76 = _mm256_permute4x64_epi64(s3, 0x4e);
        __m256i v54 = _mm256_permute4x64_epi64(s2, 0x4e);

        __m256i x87 = _mm256_add_epi16(s0, v76);
        __m256i x65 = _mm256_add_epi16(s1, v54);
        __m256i x01 = _mm256_sub_epi16(s0, v76);
        __m256i x23 = _mm256_sub_epi16(s1, v54);
        __m256i x56 = _mm256_permute4x64_epi64(x65, 0x4e);

        __m256i x45;
        x45 = _mm256_add_epi16(x87, x56);
        x87 = _mm256_sub_epi16(x87, x56);

        __m128i x4 = _mm256_extracti128_si256(x45, 0);
        __m128i x5 = _mm256_extracti128_si256(x45, 1);
        __m128i v0 = _mm_add_epi16(x4, x5);
        __m128i v4 = _mm_sub_epi16(x4, x5);

        // transform

        x87 = unpack(x87);
        x01 = unpack(x01);
        x23 = unpack(x23);

        s0 = transform_term<10>(v0, x01, c13p, x23, c57p);
        s2 = transform_term<10>(v4, x01, c51n, x23, c73p);
        s1 = transform_sub_term<10>(x87, c26p, x01, c37n, x23, c15p);
        s3 = transform_add_term<10>(x87, c62n, x01, c75n, x23, c31n);

        // pass 2

        transpose_8x8_avx2(s0, s1, s2, s3);

        v76 = _mm256_permute4x64_epi64(s3, 0x4e);
        v54 = _mm256_permute4x64_epi64(s2, 0x4e);

        x87 = _mm256_add_epi16(s0, v76);
        x65 = _mm256_add_epi16(s1, v54);
        x01 = _mm256_sub_epi16(s0, v76);
        x23 = _mm256_sub_epi16(s1, v54);
        x56 = _mm256_permute4x64_epi64(x65, 0x4e);

        x45 = _mm256_add_epi16(x87, x56);
        x87 = _mm256_sub_epi16(x87, x56);

        x4 = _mm256_extracti128_si256(x45, 0);
        x5 = _mm256_extracti128_si256(x45, 1);
        v0 = _mm_add_epi16(x4, x5);
        v4 = _mm_sub_epi16(x4, x5);
        v0 = _mm_srai_epi16(v0, 3);
        v4 = _mm_srai_epi16(v4, 3);

        // transform

        x87 = unpack(x87);
        x01 = unpack(x01);
        x23 = unpack(x23);

        s0 = transform_term<13>(v0, x01, c13p, x23, c57p);
        s2 = transform_term<13>(v4, x01, c51n, x23, c73p);
        s1 = transform_sub_term<13>(x87, c26p, x01, c37n, x23, c15p);
        s3 = transform_add_term<13>(x87, c62n, x01, c75n, x23, c31n);

        // quantize

        const __m256i one = _mm256_set1_epi16(1);
        const __m256i bias = _mm256_set1_epi16(0x4000);
        const __m256i* q = reinterpret_cast<const __m256i*>(qtable);

        s0 = quantize(s0, q[0], one, bias);
        s1 = quantize(s1, q[1], one, bias);
        s2 = quantize(s2, q[2], one, bias);
        s3 = quantize(s3, q[3], one, bias);

        // store

        __m256i* d = reinterpret_cast<__m256i *>(dest);
        _mm256_storeu_si256(d + 0, s0);
        _mm256_storeu_si256(d + 1, s1);
        _mm256_storeu_si256(d + 2, s2);
        _mm256_storeu_si256(d + 3, s3);
    }

#endif // defined(MANGO_ENABLE_AVX2)

#if defined(MANGO_ENABLE_NEON)

    // ----------------------------------------------------------------------------
    // fdct_neon
    // ----------------------------------------------------------------------------

    static inline
    void dct_trn16(int16x8_t& x, int16x8_t& y)
    {
        int16x8x2_t t = vtrnq_s16(x, y);
        x = t.val[0];
        y = t.val[1];
    }

    static inline
    void dct_trn32(int16x8_t& x, int16x8_t& y)
    {
        int32x4x2_t t = vtrnq_s32(vreinterpretq_s32_s16(x), vreinterpretq_s32_s16(y));
        x = vreinterpretq_s16_s32(t.val[0]);
        y = vreinterpretq_s16_s32(t.val[1]);
    }

    static inline
    void dct_trn64(int16x8_t& x, int16x8_t& y)
    {
        int16x8_t x0 = x;
        int16x8_t y0 = y;
        x = vcombine_s16(vget_low_s16(x0), vget_low_s16(y0));
        y = vcombine_s16(vget_high_s16(x0), vget_high_s16(y0));
    }

    static inline
    int16x8_t packs_s32(int32x4_t a, int32x4_t b)
    {
        return vcombine_s16(vqmovn_s32(a), vqmovn_s32(b));
    }

    static inline
    int16x8_t quantize(int16x8_t v, int16x8_t q, int32x4_t bias)
    {
        int32x4_t lo = vmlal_s16(bias, vget_low_s16(v), vget_low_s16(q));
        int32x4_t hi = vmlal_s16(bias, vget_high_s16(v), vget_high_s16(q));
        lo = vshrq_n_s32(lo, 15);
        hi = vshrq_n_s32(hi, 15);
        v = packs_s32(lo, hi);
        return v;
    }

    #define JPEG_TRANSPOSE16() \
        dct_trn16(v0, v1); \
        dct_trn16(v2, v3); \
        dct_trn16(v4, v5); \
        dct_trn16(v6, v7); \
        dct_trn32(v0, v2); \
        dct_trn32(v1, v3); \
        dct_trn32(v4, v6); \
        dct_trn32(v5, v7); \
        dct_trn64(v0, v4); \
        dct_trn64(v1, v5); \
        dct_trn64(v2, v6); \
        dct_trn64(v3, v7)

    #define JPEG_MUL2(v, x0, c0, x1, c1, f1, n) { \
        int32x4_t lo; \
        int32x4_t hi; \
        lo = vmull_s16(vget_low_s16(x0), c0); \
        lo = vml##f1##l_s16(lo, vget_low_s16(x1), c1); \
        lo = vshrq_n_s32(lo, n); \
        hi = vmull_s16(vget_high_s16(x0), c0); \
        hi = vml##f1##l_s16(hi, vget_high_s16(x1), c1); \
        hi = vshrq_n_s32(hi, n); \
        v = packs_s32(lo, hi); }

    #define JPEG_MUL4(v, x0, c0, x1, c1, x2, c2, x3, c3, f1, f2, f3, n) { \
        int32x4_t lo; \
        int32x4_t hi; \
        lo = vmull_s16(vget_low_s16(x0), c0); \
        lo = vml##f1##l_s16(lo, vget_low_s16(x1), c1); \
        lo = vml##f2##l_s16(lo, vget_low_s16(x2), c2); \
        lo = vml##f3##l_s16(lo, vget_low_s16(x3), c3); \
        lo = vshrq_n_s32(lo, n); \
        hi = vmull_s16(vget_high_s16(x0), c0); \
        hi = vml##f1##l_s16(hi, vget_high_s16(x1), c1); \
        hi = vml##f2##l_s16(hi, vget_high_s16(x2), c2); \
        hi = vml##f3##l_s16(hi, vget_high_s16(x3), c3); \
        hi = vshrq_n_s32(hi, n); \
        v = packs_s32(lo, hi); }

    #define JPEG_TRANSFORM(n) \
        JPEG_MUL2(v2, x8, c2, x7, c6, a, n); \
        JPEG_MUL2(v6, x8, c6, x7, c2, s, n); \
        JPEG_MUL4(v7, x0, c7, x1, c5, x2, c3, x3, c1, s, a, s, n); \
        JPEG_MUL4(v5, x0, c5, x1, c1, x2, c7, x3, c3, s, a, a, n); \
        JPEG_MUL4(v3, x0, c3, x1, c7, x2, c1, x3, c5, s, s, s, n); \
        JPEG_MUL4(v1, x0, c1, x1, c3, x2, c5, x3, c7, a, a, a, n);

    static
    void fdct_neon(s16* dest, const s16* data, const s16* qtable)
    {
        const int16x4_t c1 = vdup_n_s16(1420); // cos 1PI/16 * root(2)
        const int16x4_t c2 = vdup_n_s16(1338); // cos 2PI/16 * root(2)
        const int16x4_t c3 = vdup_n_s16(1204); // cos 3PI/16 * root(2)
        const int16x4_t c5 = vdup_n_s16(805);  // cos 5PI/16 * root(2)
        const int16x4_t c6 = vdup_n_s16(554);  // cos 6PI/16 * root(2)
        const int16x4_t c7 = vdup_n_s16(283);  // cos 7PI/16 * root(2)

        // load

        int16x8_t v0 = vld1q_s16(data + 0 * 8);
        int16x8_t v1 = vld1q_s16(data + 1 * 8);
        int16x8_t v2 = vld1q_s16(data + 2 * 8);
        int16x8_t v3 = vld1q_s16(data + 3 * 8);
        int16x8_t v4 = vld1q_s16(data + 4 * 8);
        int16x8_t v5 = vld1q_s16(data + 5 * 8);
        int16x8_t v6 = vld1q_s16(data + 6 * 8);
        int16x8_t v7 = vld1q_s16(data + 7 * 8);

        // pass 1

        JPEG_TRANSPOSE16();

        int16x8_t x8 = vaddq_s16(v0, v7);
        int16x8_t x0 = vsubq_s16(v0, v7);
        int16x8_t x7 = vaddq_s16(v1, v6);
        int16x8_t x1 = vsubq_s16(v1, v6);
        int16x8_t x6 = vaddq_s16(v2, v5);
        int16x8_t x2 = vsubq_s16(v2, v5);
        int16x8_t x5 = vaddq_s16(v3, v4);
        int16x8_t x3 = vsubq_s16(v3, v4);
        int16x8_t x4 = vaddq_s16(x8, x5);
        x8 = vsubq_s16(x8, x5);
        x5 = vaddq_s16(x7, x6);
        x7 = vsubq_s16(x7, x6);

        v0 = vaddq_s16(x4, x5);
        v4 = vsubq_s16(x4, x5);

        JPEG_TRANSFORM(10);

        // pass 2

        JPEG_TRANSPOSE16();

        x8 = vaddq_s16(v0, v7);
        x0 = vsubq_s16(v0, v7);
        x7 = vaddq_s16(v1, v6);
        x1 = vsubq_s16(v1, v6);
        x6 = vaddq_s16(v2, v5);
        x2 = vsubq_s16(v2, v5);
        x5 = vaddq_s16(v3, v4);
        x3 = vsubq_s16(v3, v4);
        x4 = vaddq_s16(x8, x5);
        x8 = vsubq_s16(x8, x5);
        x5 = vaddq_s16(x7, x6);
        x7 = vsubq_s16(x7, x6);

        v0 = vshrq_n_s16(vaddq_s16(x4, x5), 3);
        v4 = vshrq_n_s16(vsubq_s16(x4, x5), 3);

        JPEG_TRANSFORM(13);

        // quantize

        const int32x4_t bias = vdupq_n_s32(0x4000);
        const int16x8_t* q = reinterpret_cast<const int16x8_t *>(qtable);

        v0 = quantize(v0, q[0], bias);
        v1 = quantize(v1, q[1], bias);
        v2 = quantize(v2, q[2], bias);
        v3 = quantize(v3, q[3], bias);
        v4 = quantize(v4, q[4], bias);
        v5 = quantize(v5, q[5], bias);
        v6 = quantize(v6, q[6], bias);
        v7 = quantize(v7, q[7], bias);

        // store

        vst1q_s16(dest + 0 * 8, v0);
        vst1q_s16(dest + 1 * 8, v1);
        vst1q_s16(dest + 2 * 8, v2);
        vst1q_s16(dest + 3 * 8, v3);
        vst1q_s16(dest + 4 * 8, v4);
        vst1q_s16(dest + 5 * 8, v5);
        vst1q_s16(dest + 6 * 8, v6);
        vst1q_s16(dest + 7 * 8, v7);
    }

#endif // defined(MANGO_ENABLE_NEON)

    static inline
    u8* encode_dc(HuffmanEncoder& encoder, u8* p, s16 dc, const jpegEncoder::Channel& channel)
    {
        int coeff = dc - encoder.last_dc_value[channel.component];
        encoder.last_dc_value[channel.component] = dc;

        int absCoeff = std::abs(coeff);
        coeff -= (absCoeff != coeff);

        u32 size = absCoeff ? u32_log2(absCoeff) + 1 : 0;
        u32 mask = (1 << size) - 1;

        p = encoder.putBits(p, channel.dc_code[size] | (coeff & mask), channel.dc_size[size]);

        return p;
    }

    // ----------------------------------------------------------------------------
    // encode_block_scalar
    // ----------------------------------------------------------------------------

    static
    u8* encode_block_scalar(HuffmanEncoder& encoder, u8* p, const s16* input, const jpegEncoder::Channel& channel)
    {
        s16 block[64];
        encoder.fdct(block, input, channel.qtable);

        p = encode_dc(encoder, p, block[0], channel);

        const u8 zigzag_table_inverse [] =
        {
             0,  1,  8, 16,  9,  2,  3, 10,
            17, 24, 32, 25, 18, 11,  4,  5,
            12, 19, 26, 33, 40, 48, 41, 34,
            27, 20, 13,  6,  7, 14, 21, 28,
            35, 42, 49, 56, 57, 50, 43, 36,
            29, 22, 15, 23, 30, 37, 44, 51,
            58, 59, 52, 45, 38, 31, 39, 46,
            53, 60, 61, 54, 47, 55, 62, 63,
        };

        const u32* ac_code = channel.ac_code;
        const u16* ac_size = channel.ac_size;
        const u32 zero16_code = ac_code[1];
        const u32 zero16_size = ac_size[1];

        int counter = 0;

        for (int i = 1; i < 64; ++i)
        {
            int coeff = block[zigzag_table_inverse[i]];
            if (coeff)
            {
                while (counter > 15)
                {
                    counter -= 16;
                    p = encoder.putBits(p, zero16_code, zero16_size);
                }

                int absCoeff = std::abs(coeff);
                coeff -= (absCoeff != coeff);

                u32 size = u32_log2(absCoeff) + 1;
                u32 mask = (1 << size) - 1;

                int index = counter + size * 16;
                p = encoder.putBits(p, ac_code[index] | (coeff & mask), ac_size[index]);

                counter = 0;
            }
            else
            {
                ++counter;
            }
        }

        if (counter)
        {
            p = encoder.putBits(p, ac_code[0], ac_size[0]);
        }

        return p;
    }

#if defined(MANGO_ENABLE_SSE4_1)

    // ----------------------------------------------------------------------------
    // encode_block_ssse3
    // ----------------------------------------------------------------------------

    constexpr s8 lane(s8 v, s8 offset)
    {
        return v == -1 ? -1 : (v & 7) * 2 + offset;
    }

    static inline
    int16x8 shuffle(__m128i v, s8 c0, s8 c1, s8 c2, s8 c3, s8 c4, s8 c5, s8 c6, s8 c7)
    {
        const __m128i s = _mm_setr_epi8(
            lane(c0, 0), lane(c0, 1), lane(c1, 0), lane(c1, 1),
            lane(c2, 0), lane(c2, 1), lane(c3, 0), lane(c3, 1),
            lane(c4, 0), lane(c4, 1), lane(c5, 0), lane(c5, 1),
            lane(c6, 0), lane(c6, 1), lane(c7, 0), lane(c7, 1));
        return int16x8(_mm_shuffle_epi8(v, s));
    }

    u64 zigzag_ssse3(s16* out, const s16* in)
    {
        const __m128i* src = reinterpret_cast<const __m128i *>(in);

        const __m128i A = _mm_loadu_si128(src + 0); //  0 .. 7
        const __m128i B = _mm_loadu_si128(src + 1); //  8 .. 15
        const __m128i C = _mm_loadu_si128(src + 2); // 16 .. 23
        const __m128i D = _mm_loadu_si128(src + 3); // 24 .. 31
        const __m128i E = _mm_loadu_si128(src + 4); // 32 .. 39
        const __m128i F = _mm_loadu_si128(src + 5); // 40 .. 47
        const __m128i G = _mm_loadu_si128(src + 6); // 48 .. 55
        const __m128i H = _mm_loadu_si128(src + 7); // 56 .. 63

        constexpr s8 z = -1;

        // ------------------------------------------------------------------------
        //     0,  1,  8, 16,  9,  2,  3, 10,
        // ------------------------------------------------------------------------
        // A:  x   x   -   -   -   x   x   -
        // B:  -   -   x   -   x   -   -   x
        // C:  -   -   -   x   -   -   -   -
        // D:  -   -   -   -   -   -   -   -
        // E:  -   -   -   -   -   -   -   -
        // F:  -   -   -   -   -   -   -   -
        // G:  -   -   -   -   -   -   -   -
        // H:  -   -   -   -   -   -   -   -

        __m128i row0 = shuffle(A, 0, 1, z,  z, z, 2, 3,  z) |
                       shuffle(B, z, z, 8,  z, 9, z, z, 10) |
                       shuffle(C, z, z, z, 16, z, z, z,  z);

        // ------------------------------------------------------------------------
        //    17, 24, 32, 25, 18, 11,  4,  5,
        // ------------------------------------------------------------------------
        // A:  -   -   -   -   -   -   x   x
        // B:  -   -   -   -   -   x   -   -
        // C:  x   -   -   -   x   -   -   -
        // D:  -   x   -   x   -   -   -   -
        // E:  -   -   x   -   -   -   -   -
        // F:  -   -   -   -   -   -   -   -
        // G:  -   -   -   -   -   -   -   -
        // H:  -   -   -   -   -   -   -   -

        __m128i row1 = shuffle(A,  z,  z, z,  z,  z,  z, 4, 5) |
                       shuffle(B,  z,  z, z,  z,  z, 11, z, z) |
                       shuffle(C, 17,  z, z,  z, 18,  z, z, z) |
                       shuffle(D,  z, 24, z, 25,  z,  z, z, z) |
                       shuffle(E,  z, z, 32,  z,  z,  z, z, z);

        // ------------------------------------------------------------------------
        //    12, 19, 26, 33, 40, 48, 41, 34,
        // ------------------------------------------------------------------------
        // A:  -   -   -   -   -   -   -   -
        // B:  x   -   -   -   -   -   -   -
        // C:  -   x   -   -   -   -   -   -
        // D:  -   -   x   -   -   -   -   -
        // E:  -   -   -   x   -   -   -   x
        // F:  -   -   -   -   x   -   x   -
        // G:  -   -   -   -   -   x   -   -
        // H:  -   -   -   -   -   -   -   -

        __m128i row2 = shuffle(B, 12,  z,  z,  z,  z,  z,  z,  z) |
                       shuffle(C,  z, 19,  z,  z,  z,  z,  z,  z) |
                       shuffle(D,  z,  z, 26,  z,  z,  z,  z,  z) |
                       shuffle(E,  z,  z,  z, 33,  z,  z,  z, 34) |
                       shuffle(F,  z,  z,  z,  z, 40,  z, 41,  z) |
                       shuffle(G,  z,  z,  z,  z,  z, 48,  z,  z);

        // ------------------------------------------------------------------------
        //    27, 20, 13,  6,  7, 14, 21, 28,
        // ------------------------------------------------------------------------
        // A:  -   -   -   x   x   -   -   -
        // B:  -   -   x   -   -   x   -   -
        // C:  -   x   -   -   -   -   x   -
        // D:  x   -   -   -   -   -   -   x
        // E:  -   -   -   -   -   -   -   -
        // F:  -   -   -   -   -   -   -   -
        // G:  -   -   -   -   -   -   -   -
        // H:  -   -   -   -   -   -   -   -

        __m128i row3 = shuffle(A,  z,  z,  z, 6, 7,  z,  z,  z) |
                       shuffle(B,  z,  z, 13, z, z, 14,  z,  z) |
                       shuffle(C,  z, 20,  z, z, z,  z, 21,  z) |
                       shuffle(D, 27,  z,  z, z, z,  z,  z, 28);

        // ------------------------------------------------------------------------
        //    35, 42, 49, 56, 57, 50, 43, 36,
        // ------------------------------------------------------------------------
        // A:  -   -   -   -   -   -   -   -
        // B:  -   -   -   -   -   -   -   -
        // C:  -   -   -   -   -   -   -   -
        // D:  -   -   -   -   -   -   -   -
        // E:  x   -   -   -   -   -   -   x
        // F:  -   x   -   -   -   -   x   -
        // G:  -   -   x   -   -   x   -   -
        // H:  -   -   -   x   x   -   -   -

        __m128i row4 = shuffle(E, 35,  z,  z,  z,  z,  z,  z, 36) |
                       shuffle(F,  z, 42,  z,  z,  z,  z, 43,  z) |
                       shuffle(G,  z,  z, 49,  z,  z, 50,  z,  z) |
                       shuffle(H,  z,  z,  z, 56, 57,  z,  z,  z);

        // ------------------------------------------------------------------------
        //    29, 22, 15, 23, 30, 37, 44, 51,
        // ------------------------------------------------------------------------
        // A:  -   -   -   -   -   -   -   -
        // B:  -   -   x   -   -   -   -   -
        // C:  -   x   -   x   -   -   -   -
        // D:  x   -   -   -   x   -   -   -
        // E:  -   -   -   -   -   x   -   -
        // F:  -   -   -   -   -   -   x   -
        // G:  -   -   -   -   -   -   -   x
        // H:  -   -   -   -   -   -   -   -

        __m128i row5 = shuffle(B,  z,  z, 15,  z,  z,  z,  z,  z) |
                       shuffle(C,  z, 22,  z, 23,  z,  z,  z,  z) |
                       shuffle(D, 29,  z,  z,  z, 30,  z,  z,  z) |
                       shuffle(E,  z,  z,  z,  z,  z, 37,  z,  z) |
                       shuffle(F,  z,  z,  z,  z,  z,  z, 44,  z) |
                       shuffle(G,  z,  z,  z,  z,  z,  z,  z, 51);

        // ------------------------------------------------------------------------
        //    58, 59, 52, 45, 38, 31, 39, 46,
        // ------------------------------------------------------------------------
        // A:  -   -   -   -   -   -   -   -
        // B:  -   -   -   -   -   -   -   -
        // C:  -   -   -   -   -   -   -   -
        // D:  -   -   -   -   -   x   -   -
        // E:  -   -   -   -   x   -   x   -
        // F:  -   -   -   x   -   -   -   x
        // G:  -   -   x   -   -   -   -   -
        // H:  x   x   -   -   -   -   -   -

        __m128i row6 = shuffle(D,  z,  z,  z,  z,  z, 31,  z,  z) |
                       shuffle(E,  z,  z,  z,  z, 38,  z, 39,  z) |
                       shuffle(F,  z,  z,  z, 45,  z,  z,  z, 46) |
                       shuffle(G,  z,  z, 52,  z,  z,  z,  z,  z) |
                       shuffle(H, 58, 59,  z,  z,  z,  z,  z,  z);

        // ------------------------------------------------------------------------
        //    53, 60, 61, 54, 47, 55, 62, 63,
        // ------------------------------------------------------------------------
        // A:  -   -   -   -   -   -   -   -
        // B:  -   -   -   -   -   -   -   -
        // C:  -   -   -   -   -   -   -   -
        // D:  -   -   -   -   -   -   -   -
        // E:  -   -   -   -   -   -   -   -
        // F:  -   -   -   -   x   -   -   -
        // G:  x   -   -   x   -   x   -   -
        // H:  -   x   x   -   -   -   x   x

        __m128i row7 = shuffle(F,  z,  z,  z,  z, 47,  z,  z,  z) |
                       shuffle(G, 53,  z,  z, 54,  z, 55,  z,  z) |
                       shuffle(H,  z, 60, 61,  z,  z,  z, 62, 63);

        // compute zeromask
        const __m128i zero = _mm_setzero_si128();
        __m128i zero0 = _mm_packs_epi16(_mm_cmpeq_epi16(row0, zero), _mm_cmpeq_epi16(row1, zero));
        __m128i zero1 = _mm_packs_epi16(_mm_cmpeq_epi16(row2, zero), _mm_cmpeq_epi16(row3, zero));
        __m128i zero2 = _mm_packs_epi16(_mm_cmpeq_epi16(row4, zero), _mm_cmpeq_epi16(row5, zero));
        __m128i zero3 = _mm_packs_epi16(_mm_cmpeq_epi16(row6, zero), _mm_cmpeq_epi16(row7, zero));
        u64 mask_lo = u64(_mm_movemask_epi8(zero0)) | ((u64(_mm_movemask_epi8(zero1))) << 16);
        u64 mask_hi = u64(_mm_movemask_epi8(zero2)) | ((u64(_mm_movemask_epi8(zero3))) << 16);
        u64 zeromask = ~((mask_hi << 32) | mask_lo);

        __m128i* dest = reinterpret_cast<__m128i *>(out);
        _mm_storeu_si128(dest + 0, row0);
        _mm_storeu_si128(dest + 1, row1);
        _mm_storeu_si128(dest + 2, row2);
        _mm_storeu_si128(dest + 3, row3);
        _mm_storeu_si128(dest + 4, row4);
        _mm_storeu_si128(dest + 5, row5);
        _mm_storeu_si128(dest + 6, row6);
        _mm_storeu_si128(dest + 7, row7);

        return zeromask;
    }

    static
    u8* encode_block_ssse3(HuffmanEncoder& encoder, u8* p, const s16* input, const jpegEncoder::Channel& channel)
    {
        s16 block[64];
        encoder.fdct(block, input, channel.qtable);

        s16 temp[64];
        u64 zeromask = zigzag_ssse3(temp, block);

        p = encode_dc(encoder, p, temp[0], channel);
        zeromask >>= 1;

        const u32* ac_code = channel.ac_code;
        const u16* ac_size = channel.ac_size;
        const u32 zero16_code = ac_code[1];
        const u32 zero16_size = ac_size[1];

        for (int i = 1; i < 64; ++i)
        {
            if (!zeromask)
            {
                // only zeros left
                p = encoder.putBits(p, ac_code[0], ac_size[0]);
                break;
            }

            int counter = u64_tzcnt(zeromask); // BMI
            zeromask >>= (counter + 1);
            i += counter;

            while (counter > 15)
            {
                counter -= 16;
                p = encoder.putBits(p, zero16_code, zero16_size);
            }

            int coeff = temp[i];
            int absCoeff = std::abs(coeff);
            coeff -= (absCoeff != coeff);

            u32 size = u32_log2(absCoeff) + 1;
            u32 mask = (1 << size) - 1;

            int index = counter + size * 16;
            p = encoder.putBits(p, ac_code[index] | (coeff & mask), ac_size[index]);
        }

        return p;
    }

#endif // defined(MANGO_ENABLE_SSE4_1)

#if defined(MANGO_ENABLE_AVX512)

    // ----------------------------------------------------------------------------
    // encode_block_avx512
    // ----------------------------------------------------------------------------

//#define PROTOTYPE_PARALLEL_COEFFICIENTS
#ifdef PROTOTYPE_PARALLEL_COEFFICIENTS

    static
    inline __m512i getSymbolSize(__m512i absCoeff)
    {
        int16x32 value(absCoeff);
        int16x32 base(0);
        int16x32 temp;
        mask16x32 mask;

        temp = value & 0xff00;
        mask = temp != 0;
        base = select(mask, base | 8, base);
        value = select(mask, temp, value);

        temp = value & 0xf0f0;
        mask = temp != 0;
        base = select(mask, base | 4, base);
        value = select(mask, temp, value);

        temp = value & 0xcccc;
        mask = temp != 0;
        base = select(mask, base | 2, base);
        value = select(mask, temp, value);

        temp = value & 0xaaaa;
        mask = temp != 0;
        base = select(mask, base | 1, base);
        value = select(mask, temp, value);

        base += 1;

        return base;
    }

    static
    inline __m512i absSymbolSize(__m512i* ptr_sz, __m512i coeff)
    {
        __m512i absCoeff = _mm512_abs_epi16(coeff);

        __m512i one = _mm512_set1_epi16(1);
        __mmask32 mask = ~_mm512_cmpeq_epi16_mask(absCoeff, coeff);
        coeff = _mm512_mask_sub_epi16(coeff, mask, coeff, one);

        __m512i sz = getSymbolSize(absCoeff);
        _mm512_storeu_si512(ptr_sz, sz);

        return coeff;
    }

#endif // PROTOTYPE_PARALLEL_COEFFICIENTS

    u64 zigzag_avx512bw(s16* out, const s16* in)
    {
        static const u16 zigzag_shuffle [] =
        {
             0,  1,  8, 16,  9,  2,  3, 10,
            17, 24, 32, 25, 18, 11,  4,  5,
            12, 19, 26, 33, 40, 48, 41, 34,
            27, 20, 13,  6,  7, 14, 21, 28,
            35, 42, 49, 56, 57, 50, 43, 36,
            29, 22, 15, 23, 30, 37, 44, 51,
            58, 59, 52, 45, 38, 31, 39, 46,
            53, 60, 61, 54, 47, 55, 62, 63
        };

        const __m512i* src = reinterpret_cast<const __m512i *>(in);
        const __m512i* table = reinterpret_cast<const __m512i *>(zigzag_shuffle);

        const __m512i src0 = _mm512_loadu_si512(src + 0);
        const __m512i src1 = _mm512_loadu_si512(src + 1);
        const __m512i table0 = _mm512_loadu_si512(table + 0);
        const __m512i table1 = _mm512_loadu_si512(table + 1);
        const __m512i v0 = _mm512_permutex2var_epi16(src0, table0, src1);
        const __m512i v1 = _mm512_permutex2var_epi16(src0, table1, src1);

        // compute zeromask
        const __m512i zero = _mm512_setzero_si512();
        __mmask32 mask_lo = _mm512_cmpneq_epu16_mask(v0, zero);
        __mmask32 mask_hi = _mm512_cmpneq_epu16_mask(v1, zero);
        u64 zeromask = (u64(mask_hi) << 32) | mask_lo;

        __m512i* dest = reinterpret_cast<__m512i *>(out);

        _mm512_storeu_si512(dest + 0, v0);
        _mm512_storeu_si512(dest + 1, v1);

        return zeromask;
    }

    static
    u8* encode_block_avx512bw(HuffmanEncoder& encoder, u8* p, const s16* input, const jpegEncoder::Channel& channel)
    {
        s16 block[64];
        encoder.fdct(block, input, channel.qtable);

        s16 temp[64];
        u64 zeromask = zigzag_avx512bw(temp, block);

        p = encode_dc(encoder, p, temp[0], channel);
        zeromask >>= 1;

        const u32* ac_code = channel.ac_code;
        const u16* ac_size = channel.ac_size;
        const u32 zero16_code = ac_code[1];
        const u32 zero16_size = ac_size[1];

#ifdef PROTOTYPE_PARALLEL_COEFFICIENTS
        s16 s_coeff[64];
        s16 s_size[64];
        __m512i c0 = *(__m512i*)(temp + 0);
        __m512i c1 = *(__m512i*)(temp + 32);
        c0 = absSymbolSize( (__m512i*)(s_size + 0), c0);
        c1 = absSymbolSize( (__m512i*)(s_size + 32), c1);
        *(__m512i*)(s_coeff + 0) = c0;
        *(__m512i*)(s_coeff + 32) = c1;
#endif

        for (int i = 1; i < 64; ++i)
        {
            if (!zeromask)
            {
                // only zeros left
                p = encoder.putBits(p, ac_code[0], ac_size[0]);
                break;
            }

            int counter = u64_tzcnt(zeromask); // BMI
            zeromask >>= (counter + 1);
            i += counter;

            while (counter > 15)
            {
                counter -= 16;
                p = encoder.putBits(p, zero16_code, zero16_size);
            }

#ifdef PROTOTYPE_PARALLEL_COEFFICIENTS
            int coeff = s_coeff[i];
            u32 size = s_size[i];
            u32 mask = (1 << size) - 1;
#else
            int coeff = temp[i];
            int absCoeff = std::abs(coeff);
            coeff -= (absCoeff != coeff);

            u32 size = u32_log2(absCoeff) + 1;
            u32 mask = (1 << size) - 1;
#endif

            int index = counter + size * 16;
            p = encoder.putBits(p, ac_code[index] | (coeff & mask), ac_size[index]);
        }

        return p;
    }

#endif // defined(MANGO_ENABLE_AVX512)

#if defined(MANGO_ENABLE_NEON64)

    // ----------------------------------------------------------------------------
    // encode_block_neon
    // ----------------------------------------------------------------------------

#if defined(MANGO_COMPILER_GCC)

    static inline
    uint8x16x4_t jpeg_vld1q_u8_x4(const u8* p)
    {
        uint8x16x4_t result;
        result.val[0] = vld1q_u8(p +  0);
        result.val[1] = vld1q_u8(p + 16);
        result.val[2] = vld1q_u8(p + 32);
        result.val[3] = vld1q_u8(p + 48);
        return result;
    }

    static inline
    int8x16x4_t jpeg_vld1q_s8_x4(const s8* p)
    {
        int8x16x4_t result;
        result.val[0] = vld1q_s8(p +  0);
        result.val[1] = vld1q_s8(p + 16);
        result.val[2] = vld1q_s8(p + 32);
        result.val[3] = vld1q_s8(p + 48);
        return result;
    }

#else

    static inline
    uint8x16x4_t jpeg_vld1q_u8_x4(const u8 *p)
    {
        return vld1q_u8_x4(p);
    }

    static inline
    int8x16x4_t jpeg_vld1q_s8_x4(const s8 *p)
    {
        return vld1q_s8_x4(p);
    }

#endif // MANGO_COMPILER_GCC

    u64 zigzag_neon64(s16* out, const s16* in)
    {

#define ID(x) x * 2 + 0, x * 2 + 1
#define ID_255 255, 255

        const u8 zigzag_shuffle [] =
        {
            ID( 0), ID( 1), ID( 8), ID(16), ID(9),  ID( 2), ID( 3), ID(10),
            ID(17), ID(24), ID_255, ID(25), ID(18), ID(11), ID( 4), ID( 5),
            ID_255, ID( 3), ID(10), ID(17), ID(24), ID_255, ID(25), ID(18),
            ID(27), ID(20), ID(13), ID( 6), ID( 7), ID(14), ID(21), ID(28),
            ID( 3), ID(10), ID(17), ID(24), ID(25), ID(18), ID(11), ID( 4),
            ID(13), ID( 6), ID_255, ID( 7), ID(14), ID(21), ID(28), ID_255,
            ID(26), ID(27), ID(20), ID(13), ID( 6), ID_255, ID(7),  ID(14),
            ID(13), ID(20), ID(21), ID(14), ID( 7), ID(15), ID(22), ID(23),
        };

#undef ID
#undef ID_255

        const uint8x16x4_t idx_rows_0123 = jpeg_vld1q_u8_x4(zigzag_shuffle + 0 * 8);
        const uint8x16x4_t idx_rows_4567 = jpeg_vld1q_u8_x4(zigzag_shuffle + 8 * 8);

        const int8x16x4_t tbl_rows_0123 = jpeg_vld1q_s8_x4((s8 *)(in + 0 * 8));
        const int8x16x4_t tbl_rows_4567 = jpeg_vld1q_s8_x4((s8 *)(in + 4 * 8));

        const int8x16x4_t tbl_rows_2345 =
        {{
            tbl_rows_0123.val[2], tbl_rows_0123.val[3],
            tbl_rows_4567.val[0], tbl_rows_4567.val[1]
        }};
        const int8x16x3_t tbl_rows_567 = {{ tbl_rows_4567.val[1], tbl_rows_4567.val[2], tbl_rows_4567.val[3] }};

        // shuffle coefficients
        int16x8_t row0 = vreinterpretq_s16_s8(vqtbl4q_s8(tbl_rows_0123, idx_rows_0123.val[0]));
        int16x8_t row1 = vreinterpretq_s16_s8(vqtbl4q_s8(tbl_rows_0123, idx_rows_0123.val[1]));
        int16x8_t row2 = vreinterpretq_s16_s8(vqtbl4q_s8(tbl_rows_2345, idx_rows_0123.val[2]));
        int16x8_t row3 = vreinterpretq_s16_s8(vqtbl4q_s8(tbl_rows_0123, idx_rows_0123.val[3]));
        int16x8_t row4 = vreinterpretq_s16_s8(vqtbl4q_s8(tbl_rows_4567, idx_rows_4567.val[0]));
        int16x8_t row5 = vreinterpretq_s16_s8(vqtbl4q_s8(tbl_rows_2345, idx_rows_4567.val[1]));
        int16x8_t row6 = vreinterpretq_s16_s8(vqtbl4q_s8(tbl_rows_4567, idx_rows_4567.val[2]));
        int16x8_t row7 = vreinterpretq_s16_s8(vqtbl3q_s8(tbl_rows_567, idx_rows_4567.val[3]));

        // patch "holes" left in the shuffle table (ID_255)
        row1 = vsetq_lane_s16(vgetq_lane_s16(vreinterpretq_s16_s8(tbl_rows_4567.val[0]), 0), row1, 2);
        row2 = vsetq_lane_s16(vgetq_lane_s16(vreinterpretq_s16_s8(tbl_rows_0123.val[1]), 4), row2, 0);
        row2 = vsetq_lane_s16(vgetq_lane_s16(vreinterpretq_s16_s8(tbl_rows_4567.val[2]), 0), row2, 5);
        row5 = vsetq_lane_s16(vgetq_lane_s16(vreinterpretq_s16_s8(tbl_rows_0123.val[1]), 7), row5, 2);
        row5 = vsetq_lane_s16(vgetq_lane_s16(vreinterpretq_s16_s8(tbl_rows_4567.val[2]), 3), row5, 7);
        row6 = vsetq_lane_s16(vgetq_lane_s16(vreinterpretq_s16_s8(tbl_rows_0123.val[3]), 7), row6, 5);

        // zeromask
        const uint16x8_t zero = vdupq_n_u16(0);
        uint8x8_t gt0 = vmovn_u16(vcgtq_u16(vreinterpretq_u16_s16(row0), zero));
        uint8x8_t gt1 = vmovn_u16(vcgtq_u16(vreinterpretq_u16_s16(row1), zero));
        uint8x8_t gt2 = vmovn_u16(vcgtq_u16(vreinterpretq_u16_s16(row2), zero));
        uint8x8_t gt3 = vmovn_u16(vcgtq_u16(vreinterpretq_u16_s16(row3), zero));
        uint8x8_t gt4 = vmovn_u16(vcgtq_u16(vreinterpretq_u16_s16(row4), zero));
        uint8x8_t gt5 = vmovn_u16(vcgtq_u16(vreinterpretq_u16_s16(row5), zero));
        uint8x8_t gt6 = vmovn_u16(vcgtq_u16(vreinterpretq_u16_s16(row6), zero));
        uint8x8_t gt7 = vmovn_u16(vcgtq_u16(vreinterpretq_u16_s16(row7), zero));

        const uint8x16_t mask = { 1, 2, 4, 8, 16, 32, 64, 128, 1, 2, 4, 8, 16, 32, 64, 128 };
        uint8x16_t gt01 = vandq_u8(vcombine_u8(gt0, gt1), mask);
        uint8x16_t gt23 = vandq_u8(vcombine_u8(gt2, gt3), mask);
        uint8x16_t gt45 = vandq_u8(vcombine_u8(gt4, gt5), mask);
        uint8x16_t gt67 = vandq_u8(vcombine_u8(gt6, gt7), mask);
        uint8x16_t gt0123 = vpaddq_u8(gt01, gt23);
        uint8x16_t gt4567 = vpaddq_u8(gt45, gt67);
        uint8x16_t gt01234567 = vpaddq_u8(gt0123, gt4567);
        uint8x16_t x = vpaddq_u8(gt01234567, gt01234567);
        u64 zeromask = vgetq_lane_u64(vreinterpretq_u64_u8(x), 0);

        vst1q_s16(out + 0 * 8, row0);
        vst1q_s16(out + 1 * 8, row1);
        vst1q_s16(out + 2 * 8, row2);
        vst1q_s16(out + 3 * 8, row3);
        vst1q_s16(out + 4 * 8, row4);
        vst1q_s16(out + 5 * 8, row5);
        vst1q_s16(out + 6 * 8, row6);
        vst1q_s16(out + 7 * 8, row7);

        return zeromask;
    }

    static
    u8* encode_block_neon64(HuffmanEncoder& encoder, u8* p, const s16* input, const jpegEncoder::Channel& channel)
    {
        s16 block[64];
        encoder.fdct(block, input, channel.qtable);

        s16 temp[64];
        u64 zeromask = zigzag_neon64(temp, block);

        p = encode_dc(encoder, p, temp[0], channel);
        zeromask >>= 1;

        const u32* ac_code = channel.ac_code;
        const u16* ac_size = channel.ac_size;
        const u32 zero16_code = ac_code[1];
        const u32 zero16_size = ac_size[1];

        for (int i = 1; i < 64; ++i)
        {
            if (!zeromask)
            {
                // only zeros left
                p = encoder.putBits(p, ac_code[0], ac_size[0]);
                break;
            }

            int counter = u64_tzcnt(zeromask);
            zeromask >>= (counter + 1);
            i += counter;

            while (counter > 15)
            {
                counter -= 16;
                p = encoder.putBits(p, zero16_code, zero16_size);
            }

            int coeff = temp[i];
            int absCoeff = std::abs(coeff);
            coeff -= (absCoeff != coeff);

            u32 size = u32_log2(absCoeff) + 1;
            u32 mask = (1 << size) - 1;

            int index = counter + size * 16;
            p = encoder.putBits(p, ac_code[index] | (coeff & mask), ac_size[index]);
        }

        return p;
    }

#endif // defined(MANGO_ENABLE_NEON64)

    // ----------------------------------------------------------------------------
    // read_xxx_format
    // ----------------------------------------------------------------------------

    static
    void compute_ycbcr(s16* dest, int r, int g, int b)
    {
        int y = (76 * r + 151 * g + 29 * b) >> 8;
        int cr = ((r - y) * 182) >> 8;
        int cb = ((b - y) * 144) >> 8;
        dest[0 * 64] = s16(y - 128);
        dest[1 * 64] = s16(cb);
        dest[2 * 64] = s16(cr);
    }

    static
    void read_y_format(s16* block, const u8* input, size_t stride, int rows, int cols)
    {
        for (int y = 0; y < rows; ++y)
        {
            for (int x = 0; x < cols; ++x)
            {
                *block++ = input[x] - 128;
            }

            // replicate last column
            if (cols < 8)
            {
                const int count = 8 - cols;
                std::fill_n(block, count, block[-1]);
                block += count;
            }

            input += stride;
        }

        // replicate last row
        for (int y = rows; y < 8; ++y)
        {
            std::memcpy(block, block - 8, 16);
            block += 8;
        }
    }

    static
    void read_bgr_format(s16* block, const u8* input, size_t stride, int rows, int cols)
    {
        for (int y = 0; y < rows; ++y)
        {
            const u8* scan = input;

            for (int x = 0; x < cols; ++x)
            {
                int r = scan[2];
                int g = scan[1];
                int b = scan[0];
                compute_ycbcr(block, r, g, b);
                ++block;
                scan += 3;
            }

            // replicate last column
            if (cols < 8)
            {
                const int count = 8 - cols;
                std::fill_n(block + 0 * 64, count, block[0 * 64 - 1]);
                std::fill_n(block + 1 * 64, count, block[1 * 64 - 1]);
                std::fill_n(block + 2 * 64, count, block[2 * 64 - 1]);
                block += count;
            }

            input += stride;
        }

        // replicate last row
        for (int y = rows; y < 8; ++y)
        {
            std::memcpy(block + 0 * 64, block + 0 * 64 - 8, 16);
            std::memcpy(block + 1 * 64, block + 1 * 64 - 8, 16);
            std::memcpy(block + 2 * 64, block + 2 * 64 - 8, 16);
            block += 8;
        }
    }

    static
    void read_rgb_format(s16* block, const u8* input, size_t stride, int rows, int cols)
    {
        for (int y = 0; y < rows; ++y)
        {
            const u8* scan = input;
            for (int x = 0; x < cols; ++x)
            {
                int r = scan[0];
                int g = scan[1];
                int b = scan[2];
                compute_ycbcr(block, r, g, b);
                ++block;
                scan += 3;
            }

            // replicate last column
            if (cols < 8)
            {
                const int count = 8 - cols;
                std::fill_n(block + 0 * 64, count, block[0 * 64 - 1]);
                std::fill_n(block + 1 * 64, count, block[1 * 64 - 1]);
                std::fill_n(block + 2 * 64, count, block[2 * 64 - 1]);
                block += count;
            }

            input += stride;
        }

        // replicate last row
        for (int y = rows; y < 8; ++y)
        {
            std::memcpy(block + 0 * 64, block + 0 * 64 - 8, 16);
            std::memcpy(block + 1 * 64, block + 1 * 64 - 8, 16);
            std::memcpy(block + 2 * 64, block + 2 * 64 - 8, 16);
            block += 8;
        }
    }

    static
    void read_bgra_format(s16* block, const u8* input, size_t stride, int rows, int cols)
    {
        for (int y = 0; y < rows; ++y)
        {
            const u8* scan = input;
            for (int x = 0; x < cols; ++x)
            {
                s16 r = scan[2];
                s16 g = scan[1];
                s16 b = scan[0];
                compute_ycbcr(block, r, g, b);
                ++block;
                scan += 4;
            }

            // replicate last column
            if (cols < 8)
            {
                const int count = 8 - cols;
                std::fill_n(block + 0 * 64, count, block[0 * 64 - 1]);
                std::fill_n(block + 1 * 64, count, block[1 * 64 - 1]);
                std::fill_n(block + 2 * 64, count, block[2 * 64 - 1]);
                block += count;
            }

            input += stride;
        }

        // replicate last row
        for (int y = rows; y < 8; ++y)
        {
            std::memcpy(block + 0 * 64, block + 0 * 64 - 8, 16);
            std::memcpy(block + 1 * 64, block + 1 * 64 - 8, 16);
            std::memcpy(block + 2 * 64, block + 2 * 64 - 8, 16);
            block += 8;
        }
    }

    static
    void read_rgba_format(s16* block, const u8* input, size_t stride, int rows, int cols)
    {
        for (int y = 0; y < rows; ++y)
        {
            const u8* scan = input;
            for (int x = 0; x < cols; ++x)
            {
                int r = scan[0];
                int g = scan[1];
                int b = scan[2];
                compute_ycbcr(block, r, g, b);
                ++block;
                scan += 4;
            }

            // replicate last column
            if (cols < 8)
            {
                const int count = 8 - cols;
                std::fill_n(block + 0 * 64, count, block[0 * 64 - 1]);
                std::fill_n(block + 1 * 64, count, block[1 * 64 - 1]);
                std::fill_n(block + 2 * 64, count, block[2 * 64 - 1]);
                block += count;
            }

            input += stride;
        }

        // replicate last row
        for (int y = rows; y < 8; ++y)
        {
            std::memcpy(block + 0 * 64, block + 0 * 64 - 8, 16);
            std::memcpy(block + 1 * 64, block + 1 * 64 - 8, 16);
            std::memcpy(block + 2 * 64, block + 2 * 64 - 8, 16);
            block += 8;
        }
    }

#if defined(MANGO_ENABLE_SSE2) || defined(MANGO_ENABLE_SSE4_1)

    static
    void compute_ycbcr(s16* dest, __m128i r, __m128i g, __m128i b)
    {
        const __m128i c076 = _mm_set1_epi16(76);
        const __m128i c151 = _mm_set1_epi16(151);
        const __m128i c029 = _mm_set1_epi16(29);
        const __m128i c128 = _mm_set1_epi16(128);
        const __m128i c182 = _mm_set1_epi16(182);
        const __m128i c144 = _mm_set1_epi16(144);

        // compute luminance
        __m128i s0 = _mm_mullo_epi16(r, c076);
        __m128i s1 = _mm_mullo_epi16(g, c151);
        __m128i s2 = _mm_mullo_epi16(b, c029);
        __m128i s = _mm_add_epi16(s0, _mm_add_epi16(s1, s2));
        s = _mm_srli_epi16(s, 8);

        // compute chroma
        __m128i cr = _mm_sub_epi16(r, s);
        __m128i cb = _mm_sub_epi16(b, s);
        cr = _mm_mullo_epi16(cr, c182);
        cb = _mm_mullo_epi16(cb, c144);
        cr = _mm_srai_epi16(cr, 8);
        cb = _mm_srai_epi16(cb, 8);

        // adjust bias
        s = _mm_sub_epi16(s, c128);

        // store
        __m128i* ptr = reinterpret_cast<__m128i*>(dest);
        _mm_storeu_si128(ptr +  0, s);
        _mm_storeu_si128(ptr +  8, cb);
        _mm_storeu_si128(ptr + 16, cr);
    }

#endif // defined(MANGO_ENABLE_SSE2) || defined(MANGO_ENABLE_SSE4_1)

#if defined(MANGO_ENABLE_SSE2)

    static
    void read_y_format_sse2(s16* block, const u8* input, size_t stride, int rows, int cols)
    {
        MANGO_UNREFERENCED(rows);
        MANGO_UNREFERENCED(cols);

        // load
        __m128i v0 = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(input)); input += stride;
        __m128i v1 = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(input)); input += stride;
        __m128i v2 = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(input)); input += stride;
        __m128i v3 = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(input)); input += stride;
        __m128i v4 = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(input)); input += stride;
        __m128i v5 = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(input)); input += stride;
        __m128i v6 = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(input)); input += stride;
        __m128i v7 = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(input));

        // adjust bias
        __m128i bias = _mm_set1_epi8(-128);
        v0 = _mm_sub_epi8(v0, bias);
        v1 = _mm_sub_epi8(v1, bias);
        v2 = _mm_sub_epi8(v2, bias);
        v3 = _mm_sub_epi8(v3, bias);
        v4 = _mm_sub_epi8(v4, bias);
        v5 = _mm_sub_epi8(v5, bias);
        v6 = _mm_sub_epi8(v6, bias);
        v7 = _mm_sub_epi8(v7, bias);

        // sign-extend
#if defined(MANGO_ENABLE_SSE4_1)
        v0 = _mm_cvtepi8_epi16(v0);
        v1 = _mm_cvtepi8_epi16(v1);
        v2 = _mm_cvtepi8_epi16(v2);
        v3 = _mm_cvtepi8_epi16(v3);
        v4 = _mm_cvtepi8_epi16(v4);
        v5 = _mm_cvtepi8_epi16(v5);
        v6 = _mm_cvtepi8_epi16(v6);
        v7 = _mm_cvtepi8_epi16(v7);
#else
        __m128i zero = _mm_setzero_si128();
        v0 = _mm_unpacklo_epi8(v0, _mm_cmpgt_epi8(zero, v0));
        v1 = _mm_unpacklo_epi8(v1, _mm_cmpgt_epi8(zero, v1));
        v2 = _mm_unpacklo_epi8(v2, _mm_cmpgt_epi8(zero, v2));
        v3 = _mm_unpacklo_epi8(v3, _mm_cmpgt_epi8(zero, v3));
        v4 = _mm_unpacklo_epi8(v4, _mm_cmpgt_epi8(zero, v4));
        v5 = _mm_unpacklo_epi8(v5, _mm_cmpgt_epi8(zero, v5));
        v6 = _mm_unpacklo_epi8(v6, _mm_cmpgt_epi8(zero, v6));
        v7 = _mm_unpacklo_epi8(v7, _mm_cmpgt_epi8(zero, v7));
#endif

        // store
        __m128i* dest = reinterpret_cast<__m128i*>(block);
        _mm_storeu_si128(dest + 0, v0);
        _mm_storeu_si128(dest + 1, v1);
        _mm_storeu_si128(dest + 2, v2);
        _mm_storeu_si128(dest + 3, v3);
        _mm_storeu_si128(dest + 4, v4);
        _mm_storeu_si128(dest + 5, v5);
        _mm_storeu_si128(dest + 6, v6);
        _mm_storeu_si128(dest + 7, v7);
    }

    static
    void read_bgra_format_sse2(s16* block, const u8* input, size_t stride, int rows, int cols)
    {
        MANGO_UNREFERENCED(rows);
        MANGO_UNREFERENCED(cols);

        const __m128i mask = _mm_set1_epi32(0xff);

        for (int y = 0; y < 8; ++y)
        {
            const __m128i* ptr = reinterpret_cast<const __m128i*>(input);

            __m128i b0 = _mm_loadu_si128(ptr + 0);
            __m128i b1 = _mm_loadu_si128(ptr + 1);
            __m128i g0 = _mm_srli_epi32(b0, 8);
            __m128i r0 = _mm_srli_epi32(b0, 16);
            __m128i g1 = _mm_srli_epi32(b1, 8);
            __m128i r1 = _mm_srli_epi32(b1, 16);
            b0 = _mm_and_si128(b0, mask);
            b1 = _mm_and_si128(b1, mask);
            g0 = _mm_and_si128(g0, mask);
            g1 = _mm_and_si128(g1, mask);
            r0 = _mm_and_si128(r0, mask);
            r1 = _mm_and_si128(r1, mask);
            __m128i b = _mm_packs_epi32(b0, b1);
            __m128i g = _mm_packs_epi32(g0, g1);
            __m128i r = _mm_packs_epi32(r0, r1);

            compute_ycbcr(block, r, g, b);

            block += 8;
            input += stride;
        }
    }

    static
    void read_rgba_format_sse2(s16* block, const u8* input, size_t stride, int rows, int cols)
    {
        MANGO_UNREFERENCED(rows);
        MANGO_UNREFERENCED(cols);

        const __m128i mask = _mm_set1_epi32(0xff);

        for (int y = 0; y < 8; ++y)
        {
            const __m128i* ptr = reinterpret_cast<const __m128i*>(input);

            __m128i r0 = _mm_loadu_si128(ptr + 0);
            __m128i r1 = _mm_loadu_si128(ptr + 1);
            __m128i g0 = _mm_srli_epi32(r0, 8);
            __m128i b0 = _mm_srli_epi32(r0, 16);
            __m128i g1 = _mm_srli_epi32(r1, 8);
            __m128i b1 = _mm_srli_epi32(r1, 16);
            b0 = _mm_and_si128(b0, mask);
            b1 = _mm_and_si128(b1, mask);
            g0 = _mm_and_si128(g0, mask);
            g1 = _mm_and_si128(g1, mask);
            r0 = _mm_and_si128(r0, mask);
            r1 = _mm_and_si128(r1, mask);
            __m128i b = _mm_packs_epi32(b0, b1);
            __m128i g = _mm_packs_epi32(g0, g1);
            __m128i r = _mm_packs_epi32(r0, r1);

            compute_ycbcr(block, r, g, b);

            block += 8;
            input += stride;
        }
    }

#endif // MANGO_ENABLE_SSE2

#if defined(MANGO_ENABLE_SSE4_1)

    static
    void read_bgr_format_ssse3(s16* block, const u8* input, size_t stride, int rows, int cols)
    {
        MANGO_UNREFERENCED(rows);
        MANGO_UNREFERENCED(cols);

        for (int y = 0; y < 8; ++y)
        {
            const __m128i* ptr = reinterpret_cast<const __m128i*>(input);

            // load
            __m128i v0 = _mm_loadu_si128(ptr + 0);
            __m128i v1 = _mm_loadl_epi64(ptr + 1);

            // unpack
            constexpr u8 n = 0x80;
            __m128i b0 = _mm_shuffle_epi8(v0, _mm_setr_epi8(0, n, 3, n, 6, n, 9, n, 12, n, 15, n, n, n, n, n));
            __m128i b1 = _mm_shuffle_epi8(v1, _mm_setr_epi8(n, n, n, n, n, n, n, n, n, n, n, n, 2, n, 5, n));
            __m128i g0 = _mm_shuffle_epi8(v0, _mm_setr_epi8(1, n, 4, n, 7, n, 10, n, 13, n, n, n, n, n, n, n));
            __m128i g1 = _mm_shuffle_epi8(v1, _mm_setr_epi8(n, n, n, n, n, n, n, n, n, n, 0, n, 3, n, 6, n));
            __m128i r0 = _mm_shuffle_epi8(v0, _mm_setr_epi8(2, n, 5, n, 8, n, 11, n, 14, n, n, n, n, n, n, n));
            __m128i r1 = _mm_shuffle_epi8(v1, _mm_setr_epi8(n, n, n, n, n, n, n, n, n, n, 1, n, 4, n, 7, n));
           __m128i b = _mm_or_si128(b0, b1);
           __m128i g = _mm_or_si128(g0, g1);
           __m128i r = _mm_or_si128(r0, r1);

            compute_ycbcr(block, r, g, b);

            block += 8;
            input += stride;
        }
    }

    static
    void read_rgb_format_ssse3(s16* block, const u8* input, size_t stride, int rows, int cols)
    {
        MANGO_UNREFERENCED(rows);
        MANGO_UNREFERENCED(cols);

        for (int y = 0; y < 8; ++y)
        {
            const __m128i* ptr = reinterpret_cast<const __m128i*>(input);

            // load
            __m128i v0 = _mm_loadu_si128(ptr + 0);
            __m128i v1 = _mm_loadl_epi64(ptr + 1);

            // unpack
            constexpr u8 n = 0x80;
            __m128i r0 = _mm_shuffle_epi8(v0, _mm_setr_epi8(0, n, 3, n, 6, n, 9, n, 12, n, 15, n, n, n, n, n));
            __m128i r1 = _mm_shuffle_epi8(v1, _mm_setr_epi8(n, n, n, n, n, n, n, n, n, n, n, n, 2, n, 5, n));
            __m128i g0 = _mm_shuffle_epi8(v0, _mm_setr_epi8(1, n, 4, n, 7, n, 10, n, 13, n, n, n, n, n, n, n));
            __m128i g1 = _mm_shuffle_epi8(v1, _mm_setr_epi8(n, n, n, n, n, n, n, n, n, n, 0, n, 3, n, 6, n));
            __m128i b0 = _mm_shuffle_epi8(v0, _mm_setr_epi8(2, n, 5, n, 8, n, 11, n, 14, n, n, n, n, n, n, n));
            __m128i b1 = _mm_shuffle_epi8(v1, _mm_setr_epi8(n, n, n, n, n, n, n, n, n, n, 1, n, 4, n, 7, n));
           __m128i b = _mm_or_si128(b0, b1);
           __m128i g = _mm_or_si128(g0, g1);
           __m128i r = _mm_or_si128(r0, r1);

            compute_ycbcr(block, r, g, b);

            block += 8;
            input += stride;
        }
    }

#endif // MANGO_ENABLE_SSE4_1

#if defined(MANGO_ENABLE_NEON)

    static
    void compute_ycbcr(s16* dest, int16x8_t r, int16x8_t g, int16x8_t b)
    {
        const int16x8_t c076 = vdupq_n_s16(76);
        const int16x8_t c151 = vdupq_n_s16(151);
        const int16x8_t c029 = vdupq_n_s16(29);
        const int16x8_t c182 = vdupq_n_s16(182);
        const int16x8_t c144 = vdupq_n_s16(144);
        const int16x8_t c128 = vdupq_n_s16(128);

        // compute luminance
        int16x8_t s = vmulq_s16(r, c076);
        s = vmlaq_s16(s, g, c151);
        s = vmlaq_s16(s, b, c029);
        s = vreinterpretq_s16_u16(vshrq_n_u16(vreinterpretq_u16_s16(s), 8));

        // compute chroma
        int16x8_t cr = vmulq_s16(vsubq_s16(r, s), c182);
        int16x8_t cb = vmulq_s16(vsubq_s16(b, s), c144);
        cr = vshrq_n_s16(cr, 8);
        cb = vshrq_n_s16(cb, 8);

        // adjust bias
        s = vsubq_s16(s, c128);

        // store
        vst1q_s16(dest +   0, s);
        vst1q_s16(dest +  64, cb);
        vst1q_s16(dest + 128, cr);
    }

    static
    void read_y_format_neon(s16* block, const u8* input, size_t stride, int rows, int cols)
    {
        MANGO_UNREFERENCED(rows);
        MANGO_UNREFERENCED(cols);

        const int16x8_t c128 = vdupq_n_s16(128);

        for (int y = 0; y < 8; ++y)
        {
            uint8x8_t v = vld1_u8(input);
            int16x8_t s = vreinterpretq_s16_u16(vmovl_u8(v));
            s = vsubq_s16(s, c128);
            vst1q_s16(block, s);

            input += stride;
            block += 8;
        }
    }

    static
    void read_bgra_format_neon(s16* block, const u8* input, size_t stride, int rows, int cols)
    {
        MANGO_UNREFERENCED(rows);
        MANGO_UNREFERENCED(cols);

        for (int y = 0; y < 8; ++y)
        {
            const uint8x8x4_t temp = vld4_u8(input);
            int16x8_t r = vreinterpretq_s16_u16(vmovl_u8(temp.val[2]));
            int16x8_t g = vreinterpretq_s16_u16(vmovl_u8(temp.val[1]));
            int16x8_t b = vreinterpretq_s16_u16(vmovl_u8(temp.val[0]));

            compute_ycbcr(block, r, g, b);

            input += stride;
            block += 8;
        }
    }

    static
    void read_rgba_format_neon(s16* block, const u8* input, size_t stride, int rows, int cols)
    {
        MANGO_UNREFERENCED(rows);
        MANGO_UNREFERENCED(cols);

        for (int y = 0; y < 8; ++y)
        {
            const uint8x8x4_t temp = vld4_u8(input);
            int16x8_t r = vreinterpretq_s16_u16(vmovl_u8(temp.val[0]));
            int16x8_t g = vreinterpretq_s16_u16(vmovl_u8(temp.val[1]));
            int16x8_t b = vreinterpretq_s16_u16(vmovl_u8(temp.val[2]));

            compute_ycbcr(block, r, g, b);

            input += stride;
            block += 8;
        }
    }

    static
    void read_bgr_format_neon(s16* block, const u8* input, size_t stride, int rows, int cols)
    {
        MANGO_UNREFERENCED(rows);
        MANGO_UNREFERENCED(cols);

        for (int y = 0; y < 8; ++y)
        {
            const uint8x8x3_t temp = vld3_u8(input);
            int16x8_t r = vreinterpretq_s16_u16(vmovl_u8(temp.val[2]));
            int16x8_t g = vreinterpretq_s16_u16(vmovl_u8(temp.val[1]));
            int16x8_t b = vreinterpretq_s16_u16(vmovl_u8(temp.val[0]));

            compute_ycbcr(block, r, g, b);

            input += stride;
            block += 8;
        }
    }

    static
    void read_rgb_format_neon(s16* block, const u8* input, size_t stride, int rows, int cols)
    {
        MANGO_UNREFERENCED(rows);
        MANGO_UNREFERENCED(cols);

        for (int y = 0; y < 8; ++y)
        {
            const uint8x8x3_t temp = vld3_u8(input);
            int16x8_t r = vreinterpretq_s16_u16(vmovl_u8(temp.val[0]));
            int16x8_t g = vreinterpretq_s16_u16(vmovl_u8(temp.val[1]));
            int16x8_t b = vreinterpretq_s16_u16(vmovl_u8(temp.val[2]));

            compute_ycbcr(block, r, g, b);

            input += stride;
            block += 8;
        }
    }

#endif // MANGO_ENABLE_NEON

    // ----------------------------------------------------------------------------
    // jpegEncoder
    // ----------------------------------------------------------------------------

    jpegEncoder::jpegEncoder(const Surface& surface, SampleType sample, const ImageEncodeOptions& options)
        : m_surface(surface)
        , m_sample(sample)
        , m_options(options)
        , inverse_luminance_qtable(64)
        , inverse_chrominance_qtable(64)
    {
        components = 0;

        channel[0].component = 0;
        channel[0].qtable = inverse_luminance_qtable;
        channel[0].dc_code = g_luminance_dc_code_table;
        channel[0].dc_size = g_luminance_dc_size_table;
        channel[0].ac_code = g_luminance_ac_code_table;
        channel[0].ac_size = g_luminance_ac_size_table;

        channel[1].component = 1;
        channel[1].qtable = inverse_chrominance_qtable;
        channel[1].dc_code = g_chrominance_dc_code_table;
        channel[1].dc_size = g_chrominance_dc_size_table;
        channel[1].ac_code = g_chrominance_ac_code_table;
        channel[1].ac_size = g_chrominance_ac_size_table;

        channel[2].component = 2;
        channel[2].qtable = inverse_chrominance_qtable;
        channel[2].dc_code = g_chrominance_dc_code_table;
        channel[2].dc_size = g_chrominance_dc_size_table;
        channel[2].ac_code = g_chrominance_ac_code_table;
        channel[2].ac_size = g_chrominance_ac_size_table;

        int bytes_per_pixel = 0;
        read_8x8 = nullptr;

        u64 flags = options.simd ? getCPUFlags() : 0;
        MANGO_UNREFERENCED(flags);

        // select sampler

        const char* sampler_name = "Scalar";

        switch (sample)
        {
            case JPEG_U8_Y:
#if defined(MANGO_ENABLE_SSE2)
                if (flags & INTEL_SSE2)
                {
                    read_8x8 = read_y_format_sse2;
                    sampler_name = "SSE2 Y 8x8";
                }
#endif
#if defined(MANGO_ENABLE_NEON)
                if (flags & ARM_NEON)
                {
                    read_8x8 = read_y_format_neon;
                    sampler_name = "NEON Y 8x8";
                }
#endif
                read = read_y_format;
                bytes_per_pixel = 1;
                components = 1;
                break;

            case JPEG_U8_BGR:
#if defined(MANGO_ENABLE_SSE4_1)
                if (flags & INTEL_SSSE3)
                {
                    read_8x8 = read_bgr_format_ssse3;
                    sampler_name = "SSSE3 BGR 8x8";
                }
#endif
#if defined(MANGO_ENABLE_NEON)
                if (flags & ARM_NEON)
                {
                    read_8x8 = read_bgr_format_neon;
                    sampler_name = "NEON BGR 8x8";
                }
#endif
                read = read_bgr_format;
                bytes_per_pixel = 3;
                components = 3;
                break;

            case JPEG_U8_RGB:
#if defined(MANGO_ENABLE_SSE4_1)
                if (flags & INTEL_SSSE3)
                {
                    read_8x8 = read_rgb_format_ssse3;
                    sampler_name = "SSSE3 RGB 8x8";
                }
#endif
#if defined(MANGO_ENABLE_NEON)
                if (flags & ARM_NEON)
                {
                    read_8x8 = read_rgb_format_neon;
                    sampler_name = "NEON RGB 8x8";
                }
#endif
                read = read_rgb_format;
                bytes_per_pixel = 3;
                components = 3;
                break;

            case JPEG_U8_BGRA:
#if defined(MANGO_ENABLE_SSE2)
                if (flags & INTEL_SSE2)
                {
                    read_8x8 = read_bgra_format_sse2;
                    sampler_name = "SSE2 BGRA 8x8";
                }
#endif
#if defined(MANGO_ENABLE_NEON)
                if (flags & ARM_NEON)
                {
                    read_8x8 = read_bgra_format_neon;
                    sampler_name = "NEON BGRA 8x8";
                }
#endif
                read = read_bgra_format;
                bytes_per_pixel = 4;
                components = 3;
                break;

            case JPEG_U8_RGBA:
#if defined(MANGO_ENABLE_SSE2)
                if (flags & INTEL_SSE2)
                {
                    read_8x8 = read_rgba_format_sse2;
                    sampler_name = "SSE2 RGBA 8x8";
                }
#endif
#if defined(MANGO_ENABLE_NEON)
                if (flags & ARM_NEON)
                {
                    read_8x8 = read_rgba_format_neon;
                    sampler_name = "NEON RGBA 8x8";
                }
#endif
                read = read_rgba_format;
                bytes_per_pixel = 4;
                components = 3;
                break;
        }

        if (!read_8x8)
        {
            // no accelerated 8x8 sampler found; use the default
            read_8x8 = read;
        }

        // select fdct

        fdct = fdct_scalar;
        const char* fdct_name = "Scalar";

#if defined(MANGO_ENABLE_SSE2)
        if (flags & INTEL_SSE2)
        {
            fdct = fdct_sse2;
            fdct_name = "SSE2";
        }
#endif

#if defined(MANGO_ENABLE_AVX2__disabled)
        if (flags & INTEL_AVX2)
        {
            fdct = fdct_avx2;
            fdct_name = "AVX2";
        }
#endif

#if defined(MANGO_ENABLE_NEON)
        {
            fdct = fdct_neon;
            fdct_name = "NEON";
        }
#endif

        // select block encoder

        encode = encode_block_scalar;
        const char* encode_name = "Scalar";

#if defined(MANGO_ENABLE_SSE4_1)
        if (flags & INTEL_SSSE3)
        {
            encode = encode_block_ssse3;
            encode_name = "SSSE3";
        }
#endif

#if defined(MANGO_ENABLE_AVX512)
        if (flags & INTEL_AVX512BW)
        {
            encode = encode_block_avx512bw;
            encode_name = "AVX512BW";
        }
#endif

#if defined(MANGO_ENABLE_NEON64)
        {
            encode = encode_block_neon64;
            encode_name = "NEON64";
        }
#endif

        // build encoder info string
        info = "fDCT: ";
        info += fdct_name;

        info += ", Color: ";
        info += sampler_name;

        info += ", Encoder: ";
        info += encode_name;

        mcu_width = 8;
        mcu_height = 8;

        horizontal_mcus = (m_surface.width + mcu_width - 1) >> 3;
        vertical_mcus   = (m_surface.height + mcu_height - 1) >> 3;

        rows_in_bottom_mcus = m_surface.height - (vertical_mcus - 1) * mcu_height;
        cols_in_right_mcus  = m_surface.width  - (horizontal_mcus - 1) * mcu_width;

        mcu_stride = mcu_width * bytes_per_pixel;

        // initialize quantization tables

        const u8 zigzag_table [] =
        {
             0,  1,  5,  6, 14, 15, 27, 28,
             2,  4,  7, 13, 16, 26, 29, 42,
             3,  8, 12, 17, 25, 30, 41, 43,
             9, 11, 18, 24, 31, 40, 44, 53,
            10, 19, 23, 32, 39, 45, 52, 54,
            20, 22, 33, 38, 46, 51, 55, 60,
            21, 34, 37, 47, 50, 56, 59, 61,
            35, 36, 48, 49, 57, 58, 62, 63
        };

        // configure quality
        u32 quality = u32(std::pow(1.0f + clamp(1.0f - options.quality, 0.0f, 1.0f), 11.0f) * 8.0f);

        for (int i = 0; i < 64; ++i)
        {
            u8 index = zigzag_table[i];

            // luminance
            u32 L = u32_clamp((g_luminance_quant_table[i] * quality + 0x200) >> 10, 2, 255);
            luminance_qtable[index] = u8(L);
            inverse_luminance_qtable[i] = u16(0x8000 / L);

            // chrominance
            u32 C = u32_clamp((g_chrominance_quant_table [i] * quality + 0x200) >> 10, 2, 255);
            chrominance_qtable[index] = u8(C);
            inverse_chrominance_qtable[i] = u16(0x8000 / C);
        }
    }

    jpegEncoder::~jpegEncoder()
    {
    }

    void jpegEncoder::writeMarkers(BigEndianStream& p)
    {
        // Start of image marker
        p.write16(MARKER_SOI);

        // ICC profile.  splitting to multiple markers if necessary
        if (m_options.icc.size)
        {
            const size_t magicICCLength = 12;
            const u8 magicICC[magicICCLength] = { 0x49, 0x43, 0x43, 0x5f, 0x50, 0x52, 0x4f, 0x46, 0x49, 0x4c, 0x45, 0 }; // 'ICC_PROFILE', 0

            const size_t iccMaxSegmentSize = 65000;  // marker size is 16bit minus icc stuff, if larger we need to split
            const size_t iccSegments = (m_options.icc.size + iccMaxSegmentSize - 1) / iccMaxSegmentSize;

            for(size_t i=0; i < iccSegments; i++)
            {
                const size_t size = i < (iccSegments - 1) ? iccMaxSegmentSize :  m_options.icc.size % iccMaxSegmentSize;
                p.write16(MARKER_APP2);
                p.write16(u16(size + magicICCLength + 4));
                p.write(magicICC, magicICCLength);
                p.write8(u8(i + 1)); // segment index, 1-based
                p.write8(u8(iccSegments));
                p.write(m_options.icc.slice(i * iccMaxSegmentSize), size);
            }
        }

        // Quantization table marker
        p.write16(MARKER_DQT);
        p.write16(0x43); // quantization table length
        p.write8(0x00); // Pq, Tq

        // Lqt table
        p.write(luminance_qtable, 64);

        // Quantization table marker
        p.write16(MARKER_DQT);
        p.write16(0x43); // quantization table length
        p.write8(0x01); // Pq, Tq

        // Cqt table
        p.write(chrominance_qtable, 64);

        // Start of frame marker
        p.write16(MARKER_SOF0);

        u8 number_of_components = 0;

        switch (m_sample)
        {
            case JPEG_U8_Y:
                number_of_components = 1;
                break;

            case JPEG_U8_RGB:
            case JPEG_U8_BGR:
            case JPEG_U8_BGRA:
            case JPEG_U8_RGBA:
                number_of_components = 3;
                break;
        }

        u16 header_length = 8 + 3 * number_of_components;

        p.write16(header_length); // frame header length
        p.write8(8); // precision
        p.write16(u16(m_surface.height)); // image height
        p.write16(u16(m_surface.width)); // image width
        p.write8(number_of_components); // Nf

        const u8 nfdata[] =
        {
            0x01, 0x11, 0x00, // component 1
            0x00, 0x00, 0x00, // padding
            0x01, 0x11, 0x00, // component 1
            0x02, 0x11, 0x01, // component 2
            0x03, 0x11, 0x01, // component 3
        };

        p.write(nfdata + (number_of_components - 1) * 3, number_of_components * 3);

        // huffman table(DHT)
        p.write(g_marker_data, sizeof(g_marker_data));

        // Define Restart Interval marker
        p.write16(MARKER_DRI);
        p.write16(4);
        p.write16(horizontal_mcus);

        // Start of scan marker
        p.write16(MARKER_SOS);
        p.write16(6 + number_of_components * 2); // header length
        p.write8(number_of_components); // Ns

        const u8 nsdata[] =
        {
            0x01, 0x00,
            0x02, 0x11,
            0x03, 0x11,
        };

        p.write(nsdata, number_of_components * 2);
        p.write8(0x00);
        p.write8(0x3f);
        p.write8(0x00);
    }

    void jpegEncoder::encodeInterval(EncodeBuffer& buffer, const u8* image, size_t stride, ReadFunc read_func, int rows)
    {
        HuffmanEncoder huffman;
        huffman.fdct = fdct;

        const int right_mcu = horizontal_mcus - 1;

        constexpr int buffer_size = 2048;
        constexpr int flush_threshold = buffer_size - 512;

        u8 huff_temp[buffer_size]; // encoding buffer
        u8* ptr = huff_temp;

        int cols = mcu_width;
        auto reader = read_func;

        for (int x = 0; x < horizontal_mcus; ++x)
        {
            if (x >= right_mcu)
            {
                // horizontal clipping
                cols = cols_in_right_mcus;
                reader = read; // clipping reader
            }

            // read MCU data
            s16 block[BLOCK_SIZE * 3];
            reader(block, image, stride, rows, cols);

            // encode the data in MCU
            for (int i = 0; i < components; ++i)
            {
                ptr = encode(huffman, ptr, block + i * BLOCK_SIZE, channel[i]);
            }

            // flush encoding buffer
            if (ptr - huff_temp > flush_threshold)
            {
                buffer.append(huff_temp, ptr - huff_temp);
                ptr = huff_temp;
            }

            image += mcu_stride;
        }

        // flush encoding buffer
        ptr = huffman.flush(ptr);
        buffer.append(huff_temp, ptr - huff_temp);

        // mark buffer ready for writing
        buffer.ready = true;
    }

    ImageEncodeStatus jpegEncoder::encodeImage(Stream& stream)
    {
        const u8* image = m_surface.image;
        size_t stride = m_surface.stride;

        BigEndianStream s(stream);

        // writing marker data
        writeMarkers(s);

        // encode MCUs

        if (m_options.multithread)
        {
            ConcurrentQueue queue;

            // bitstream for each MCU scan
            std::vector<EncodeBuffer> buffers(vertical_mcus);

            for (int y = 0; y < vertical_mcus; ++y)
            {
                int rows = mcu_height;
                auto read_func = read_8x8; // default: optimized 8x8 reader

                if (y >= vertical_mcus - 1)
                {
                    // vertical clipping
                    rows = rows_in_bottom_mcus;
                    read_func = read; // clipping reader
                }

                queue.enqueue([this, y, &buffers, image, stride, rows, read_func]
                {
                    EncodeBuffer& buffer = buffers[y];
                    encodeInterval(buffer, image, stride, read_func, rows);
                });

                image += m_surface.stride * mcu_height;
            }

            for (int y = 0; y < vertical_mcus; ++y)
            {
                EncodeBuffer& buffer = buffers[y];

                for ( ; !buffer.ready; )
                {
                    // buffer is not processed yet; help the thread pool while waiting
                    queue.steal();
                }

                // write huffman bitstream
                s.write(buffer, buffer.size());

                // write restart marker
                int index = y & 7;
                s.write16(MARKER_RST0 + index);
            }
        }
        else
        {
            for (int y = 0; y < vertical_mcus; ++y)
            {
                int rows = mcu_height;
                auto read_func = read_8x8; // default: optimized 8x8 reader

                if (y >= vertical_mcus - 1)
                {
                    // vertical clipping
                    rows = rows_in_bottom_mcus;
                    read_func = read; // clipping reader
                }

                EncodeBuffer buffer;
                encodeInterval(buffer, image, stride, read_func, rows);

                // write huffman bitstream
                s.write(buffer, buffer.size());

                // write restart marker
                int index = y & 7;
                s.write16(MARKER_RST0 + index);

                image += m_surface.stride * mcu_height;
            }
        }

        // EOI marker
        s.write16(MARKER_EOI);

        ImageEncodeStatus status;
        status.info = info;

        return status;
    }

} // namespace

namespace mango::jpeg
{

    ImageEncodeStatus encodeImage(Stream& stream, const Surface& surface, const ImageEncodeOptions& options)
    {
        SampleFormat sf = getSampleFormat(surface.format);

        ImageEncodeStatus status;

        // encode
        if (surface.format == sf.format)
        {
            jpegEncoder encoder(surface, sf.sample, options);
            status = encoder.encodeImage(stream);
            status.direct = true;
        }
        else
        {
            // convert source surface to format supported in the encoder
            Bitmap temp(surface, sf.format);
            jpegEncoder encoder(temp, sf.sample, options);
            status = encoder.encodeImage(stream);
        }

        return status;
    }

} // namespace mango::jpeg

// ----------------------------------------------------------------------------
// huffman
// ----------------------------------------------------------------------------

namespace mango::jpeg
{

    // ----------------------------------------------------------------------------
    // Huffman
    // ----------------------------------------------------------------------------

    void Huffman::restart()
    {
        for (int i = 0; i < JPEG_MAX_COMPS_IN_SCAN; ++i)
        {
            last_dc_value[i] = 0;
        }

        eob_run = 0;
    }

    // ----------------------------------------------------------------------------
    // HuffTable
    // ----------------------------------------------------------------------------

    bool HuffTable::configure()
    {
        u8 huffsize[257];
        u32 huffcode[257];

        // Figure C.1: make table of Huffman code length for each symbol
        int p = 0;
        for (int j = 1; j <= 16; ++j)
        {
            int count = int(size[j]);
            while (count-- > 0)
            {
                huffsize[p++] = u8(j);
            }
        }
        huffsize[p] = 0;

        // Figure C.2: generate the codes themselves
        u32 code = 0;
        int si = huffsize[0];
        p = 0;
        while (huffsize[p])
        {
            while ((int(huffsize[p])) == si)
            {
                huffcode[p++] = code;
                code++;
            }
            code <<= 1;
            si++;
        }

        // Figure F.15: generate decoding tables for bit-sequential decoding
        p = 0;
        for (int j = 1; j <= 16; j++)
        {
            if (size[j])
            {
                valueOffset[j] = p - int(huffcode[p]);
                p += size[j];
                maxcode[j] = huffcode[p - 1]; // maximum code of length j
                maxcode[j] <<= (JPEG_REGISTER_BITS - j); // left justify
                maxcode[j] |= (DataType(1) << (JPEG_REGISTER_BITS - j)) - 1;
            }
            else
            {
                maxcode[j] = 0; // TODO: should be -1 if no codes of this length
            }
        }
        valueOffset[18] = 0;
        maxcode[17] = ~DataType(0); //0xfffff; // ensures jpeg_huff_decode terminates

        // Compute lookahead tables to speed up decoding.
        // First we set all the table entries to 0, indicating "too long";
        // then we iterate through the Huffman codes that are short enough and
        // fill in all the entries that correspond to bit sequences starting
        // with that code.

        for (int i = 0; i < JPEG_HUFF_LOOKUP_SIZE; i++)
        {
            lookupSize[i] = JPEG_HUFF_LOOKUP_BITS + 1;
            lookupValue[i] = 0;
        }

        p = 0;

        for (int bits = 1; bits <= JPEG_HUFF_LOOKUP_BITS; ++bits)
        {
            int ishift = JPEG_HUFF_LOOKUP_BITS - bits;
            int isize = size[bits];

            u8 current_size = u8(bits);

            for (int i = 1; i <= isize; ++i)
            {
                u8 current_value = value[p];
                int lookbits = huffcode[p] << ishift;
                ++p;

                // Generate left-justified code followed by all possible bit sequences
                int count = 1 << ishift;
                for (int mask = 0; mask < count; ++mask)
                {
                    int x = lookbits | mask;
                    if (x >= JPEG_HUFF_LOOKUP_SIZE)
                    {
                        // overflow
                        return false;
                    }

                    lookupSize[x] = current_size;
                    lookupValue[x] = current_value;
                }
            }
        }

        return true;
    }

    int HuffTable::decode(BitBuffer& buffer) const
    {
        buffer.ensure();

        int index = buffer.peekBits(JPEG_HUFF_LOOKUP_BITS);
        int size = lookupSize[index];

        int symbol;

        if (size <= JPEG_HUFF_LOOKUP_BITS)
        {
            symbol = lookupValue[index];
        }
        else
        {
            DataType x = (buffer.data << (JPEG_REGISTER_BITS - buffer.remain));
            while (x > maxcode[size])
            {
                ++size;
            }

            DataType offset = (x >> (JPEG_REGISTER_BITS - size)) + valueOffset[size];
#if 0
            if (offset > 255)
                return 0; // decoding error
#endif
            symbol = value[offset];
        }

        buffer.remain -= size;
        return symbol;
    }

    // ----------------------------------------------------------------------------
    // huffman decoding functions
    // ----------------------------------------------------------------------------

    void huff_decode_mcu_lossless(s16* output, DecodeState* state)
    {
        Huffman& huffman = state->huffman;
        BitBuffer& buffer = state->buffer;

        for (int j = 0; j < state->blocks; ++j)
        {
            const DecodeBlock* block = state->block + j;
            const HuffTable* dc = &huffman.table[0][block->dc];

            int s = dc->decode(buffer);
            if (s)
            {
                s = buffer.receive(s);
            }

            s += huffman.last_dc_value[block->pred];
            output[j] = s;
        }
    }

    void huff_decode_mcu(s16* output, DecodeState* state)
    {
        const u8* zigzagTable = state->zigzagTable;
        Huffman& huffman = state->huffman;
        BitBuffer& buffer = state->buffer;

        std::memset(output, 0, state->blocks * 64 * sizeof(s16));

        for (int j = 0; j < state->blocks; ++j)
        {
            const DecodeBlock* block = state->block + j;

            const HuffTable* dc = &huffman.table[0][block->dc];
            const HuffTable* ac = &huffman.table[1][block->ac];

            // DC
            int s = dc->decode(buffer);
            if (s)
            {
                s = buffer.receive(s);
            }

            s += huffman.last_dc_value[block->pred];
            huffman.last_dc_value[block->pred] = s;

            output[0] = s16(s);

            // AC
            for (int i = 1; i < 64; )
            {
                int s = ac->decode(buffer);
                int x = s & 15;

                if (x)
                {
                    i += (s >> 4);
                    s = buffer.receive(x);
                    output[zigzagTable[i++]] = s16(s);
                }
                else
                {
                    if (s < 16) break;
                    i += 16;
                }
            }

            output += 64;
        }
    }

    void huff_decode_dc_first(s16* output, DecodeState* state)
    {
        Huffman& huffman = state->huffman;
        BitBuffer& buffer = state->buffer;

        for (int j = 0; j < state->blocks; ++j)
        {
            const DecodeBlock* block = state->block + j;

            s16* dest = output + block->offset;
            const HuffTable* dc = &huffman.table[0][block->dc];

            std::memset(dest, 0, 64 * sizeof(s16));

            int s = dc->decode(buffer);
            if (s)
            {
                s = buffer.receive(s);
            }

            s += huffman.last_dc_value[block->pred];
            huffman.last_dc_value[block->pred] = s;

            dest[0] = s16(s << state->successiveLow);
        }
    }

    void huff_decode_dc_refine(s16* output, DecodeState* state)
    {
        BitBuffer& buffer = state->buffer;

        for (int j = 0; j < state->blocks; ++j)
        {
            s16* dest = output + state->block[j].offset;
            dest[0] |= (buffer.getBits(1) << state->successiveLow);
        }
    }

    void huff_decode_ac_first(s16* output, DecodeState* state)
    {
        const u8* zigzagTable = state->zigzagTable;
        Huffman& huffman = state->huffman;
        BitBuffer& buffer = state->buffer;

        const HuffTable* ac = &huffman.table[1][state->block[0].ac];

        const int start = state->spectralStart;
        const int end = state->spectralEnd;

        if (huffman.eob_run)
        {
            --huffman.eob_run;
        }
        else
        {
            for (int i = start; i <= end; ++i)
            {
                int s = ac->decode(buffer);
                int r = s >> 4;
                s &= 15;

                i += r;

                if (s)
                {
                    s = buffer.receive(s);
                    output[zigzagTable[i]] = s16(s << state->successiveLow);
                }
                else
                {
                    if (r != 15)
                    {
                        huffman.eob_run = 1 << r;
                        if (r)
                        {
                            huffman.eob_run += buffer.getBits(r);
                        }

                        --huffman.eob_run;
                        break;
                    }
                }
            }
        }
    }

    void huff_decode_ac_refine(s16* output, DecodeState* state)
    {
        const u8* zigzagTable = state->zigzagTable;
        Huffman& huffman = state->huffman;
        BitBuffer& buffer = state->buffer;

        const HuffTable* ac = &huffman.table[1][state->block[0].ac];

        const int start = state->spectralStart;
        const int end = state->spectralEnd;

        const int p1 = 1 << state->successiveLow;
        const int m1 = (-1) << state->successiveLow;

        int k = start;

        if (!huffman.eob_run)
        {
            for (; k <= end; k++)
            {
                int s = ac->decode(buffer);
                int r = s >> 4;
                s &= 15;

                if (s)
                {
                    if (buffer.getBits(1))
                        s = p1;
                    else
                        s = m1;
                }
                else
                {
                    if (r != 15)
                    {
                        huffman.eob_run = 1 << r;

                        if (r)
                        {
                            huffman.eob_run += buffer.getBits(r);
                        }

                        break;
                    }
                }

                do
                {
                    s16* coef = output + zigzagTable[k];
                    if (*coef != 0)
                    {
                        if (buffer.getBits(1))
                        {
                            if ((*coef & p1) == 0)
                            {
                                const int d = *coef >= 0 ? p1 : m1;
                                *coef += s16(d);
                            }
                        }
                    }
                    else
                    {
                        if (--r < 0)
                            break;
                    }

                    k++;
                } while (k <= end);

                if ((s) && (k < 64))
                {
                    output[zigzagTable[k]] = s16(s);
                }
            }
        }

        if (huffman.eob_run > 0)
        {
            for ( ; k <= end; k++)
            {
                s16* coef = output + zigzagTable[k];

                if (*coef != 0)
                {
                    if (buffer.getBits(1))
                    {
                        if ((*coef & p1) == 0)
                        {
                            const int d = *coef >= 0 ? p1 : m1;
                            *coef += s16(d);
                        }
                    }
                }
            }

            --huffman.eob_run;
        }
    }

} // namespace mango::jpeg

// ----------------------------------------------------------------------------
// idct
// ----------------------------------------------------------------------------

namespace
{
    using namespace mango;
    using namespace jpeg;

    struct IDCT
    {
        int x0, x1, x2, x3;
        int y0, y1, y2, y3;

        void compute(int s0, int s1, int s2, int s3, int s4, int s5, int s6, int s7)
        {
            const int n0 = (s2 + s6) * 2217;
            const int t2 = n0 + s6 * -7567;
            const int t3 = n0 + s2 * 3135;
            const int t0 = (s0 + s4) << 12;
            const int t1 = (s0 - s4) << 12;
            x0 = t0 + t3;
            x3 = t0 - t3;
            x1 = t1 + t2;
            x2 = t1 - t2;

            int p1 = s7 + s1;
            int p2 = s5 + s3;
            int p3 = s7 + s3;
            int p4 = s5 + s1;
            int p5 = (p3 + p4) * 4816;
            p1 = p1 * -3685 + p5;
            p2 = p2 * -10497 + p5;
            p3 = p3 * -8034;
            p4 = p4 * -1597;
            y0 = p1 + p3 + s7 * 1223;
            y1 = p2 + p4 + s5 * 8410;
            y2 = p2 + p3 + s3 * 12586;
            y3 = p1 + p4 + s1 * 6149;
        }
    };

    template <int PRECISION>
    void idct(u8* dest, const s16* data, const s16* qt)
    {
        int temp[64];
        int* v;

        const s16* s = data;

        v = temp;

        for (int i = 0; i < 8; ++i)
        {
            if (s[i + 8 * 1] || s[i + 8 * 2] || s[i + 8 * 3] || s[i + 8 * 4] || s[i + 8 * 5] || s[i + 8 * 6] || s[i + 8 * 7])
            {
                // dequantize
                const int s0 = s[i + 8 * 0] * qt[i + 8 * 0];
                const int s1 = s[i + 8 * 1] * qt[i + 8 * 1];
                const int s2 = s[i + 8 * 2] * qt[i + 8 * 2];
                const int s3 = s[i + 8 * 3] * qt[i + 8 * 3];
                const int s4 = s[i + 8 * 4] * qt[i + 8 * 4];
                const int s5 = s[i + 8 * 5] * qt[i + 8 * 5];
                const int s6 = s[i + 8 * 6] * qt[i + 8 * 6];
                const int s7 = s[i + 8 * 7] * qt[i + 8 * 7];

                IDCT idct;
                idct.compute(s0, s1, s2, s3, s4, s5, s6, s7);
                const int bias = 0x200;
                idct.x0 += bias;
                idct.x1 += bias;
                idct.x2 += bias;
                idct.x3 += bias;
                v[0] = (idct.x0 + idct.y3) >> 10;
                v[1] = (idct.x1 + idct.y2) >> 10;
                v[2] = (idct.x2 + idct.y1) >> 10;
                v[3] = (idct.x3 + idct.y0) >> 10;
                v[4] = (idct.x3 - idct.y0) >> 10;
                v[5] = (idct.x2 - idct.y1) >> 10;
                v[6] = (idct.x1 - idct.y2) >> 10;
                v[7] = (idct.x0 - idct.y3) >> 10;
            }
            else
            {
                int dc = (s[i] * qt[i]) << 2;
                v[0] = dc;
                v[1] = dc;
                v[2] = dc;
                v[3] = dc;
                v[4] = dc;
                v[5] = dc;
                v[6] = dc;
                v[7] = dc;
            }

            v += 8;
        }

        v = temp;
        const int shift = PRECISION + 9;

        for (int i = 0; i < 8; ++i)
        {
            IDCT idct;
            idct.compute(v[0], v[8], v[16], v[24], v[32], v[40], v[48], v[56]);
            ++v;
            const int bias = 0x10000 + (128 << shift);
            idct.x0 += bias;
            idct.x1 += bias;
            idct.x2 += bias;
            idct.x3 += bias;
            dest[0] = byteclamp((idct.x0 + idct.y3) >> shift);
            dest[1] = byteclamp((idct.x1 + idct.y2) >> shift);
            dest[2] = byteclamp((idct.x2 + idct.y1) >> shift);
            dest[3] = byteclamp((idct.x3 + idct.y0) >> shift);
            dest[4] = byteclamp((idct.x3 - idct.y0) >> shift);
            dest[5] = byteclamp((idct.x2 - idct.y1) >> shift);
            dest[6] = byteclamp((idct.x1 - idct.y2) >> shift);
            dest[7] = byteclamp((idct.x0 - idct.y3) >> shift);
            dest += 8;
        }
    }

} // namespace

namespace mango::jpeg
{

    // ------------------------------------------------------------------------------------------------
    // Generic C++ implementation
    // ------------------------------------------------------------------------------------------------

    void idct8(u8* dest, const s16* data, const s16* qt)
    {
        idct<8>(dest, data, qt);
    }

    void idct12(u8* dest, const s16* data, const s16* qt)
    {
        idct<12>(dest, data, qt);
    }

#if defined(MANGO_ENABLE_SSE2)

    // ------------------------------------------------------------------------------------------------
    // SSE2 implementation
    // ------------------------------------------------------------------------------------------------

    // The original code is by Petr Kobalicek ; WE HAVE TAKEN LIBERTIES TO ADAPT IT TO OUR USE!!!
    // https://github.com/kobalicek/simdtests
    // [License]
    // Public Domain <unlicense.org>

    // Derived from jidctint's `jpeg_idct_islow`
    constexpr int JPEG_IDCT_PREC = 12;
    constexpr int JPEG_IDCT_HALF(int precision) { return (1 << ((precision) - 1)); }
    constexpr int JPEG_IDCT_FIXED(double x) { return int((x * double(1 << JPEG_IDCT_PREC) + 0.5)); }

    constexpr int JPEG_IDCT_M_2_562915447 = JPEG_IDCT_FIXED(-2.562915447);
    constexpr int JPEG_IDCT_M_1_961570560 = JPEG_IDCT_FIXED(-1.961570560);
    constexpr int JPEG_IDCT_M_1_847759065 = JPEG_IDCT_FIXED(-1.847759065);
    constexpr int JPEG_IDCT_M_0_899976223 = JPEG_IDCT_FIXED(-0.899976223);
    constexpr int JPEG_IDCT_M_0_390180644 = JPEG_IDCT_FIXED(-0.390180644);
    constexpr int JPEG_IDCT_P_0_298631336 = JPEG_IDCT_FIXED(0.298631336);
    constexpr int JPEG_IDCT_P_0_541196100 = JPEG_IDCT_FIXED(0.541196100);
    constexpr int JPEG_IDCT_P_0_765366865 = JPEG_IDCT_FIXED(0.765366865);
    constexpr int JPEG_IDCT_P_1_175875602 = JPEG_IDCT_FIXED(1.175875602);
    constexpr int JPEG_IDCT_P_1_501321110 = JPEG_IDCT_FIXED(1.501321110);
    constexpr int JPEG_IDCT_P_2_053119869 = JPEG_IDCT_FIXED(2.053119869);
    constexpr int JPEG_IDCT_P_3_072711026 = JPEG_IDCT_FIXED(3.072711026);

    // Keep 2 bits of extra precision for the intermediate results
    constexpr int JPEG_IDCT_COL_NORM = (JPEG_IDCT_PREC - 2);
    constexpr int JPEG_IDCT_COL_BIAS = JPEG_IDCT_HALF(JPEG_IDCT_COL_NORM);

    // Consume 2 bits of an intermediate results precision and 3 bits that were
    // produced by `2 * sqrt(8)`. Also normalize to from `-128..127` to `0..255`
    constexpr int JPEG_IDCT_ROW_NORM = (JPEG_IDCT_PREC + 2 + 3);
    constexpr int JPEG_IDCT_ROW_BIAS = (JPEG_IDCT_HALF(JPEG_IDCT_ROW_NORM) + (128 << JPEG_IDCT_ROW_NORM));

#define JPEG_CONST16_SSE2(x, y)  _mm_setr_epi16(x, y, x, y, x, y, x, y)
#define JPEG_CONST32_SSE2(x)     _mm_setr_epi32(x, x, x, x)

    static const __m128i rot0_0 = JPEG_CONST16_SSE2(JPEG_IDCT_P_0_541196100                          , JPEG_IDCT_P_0_541196100 + JPEG_IDCT_M_1_847759065);
    static const __m128i rot0_1 = JPEG_CONST16_SSE2(JPEG_IDCT_P_0_541196100 + JPEG_IDCT_P_0_765366865, JPEG_IDCT_P_0_541196100                          );
    static const __m128i rot1_0 = JPEG_CONST16_SSE2(JPEG_IDCT_P_1_175875602 + JPEG_IDCT_M_0_899976223, JPEG_IDCT_P_1_175875602                          );
    static const __m128i rot1_1 = JPEG_CONST16_SSE2(JPEG_IDCT_P_1_175875602                          , JPEG_IDCT_P_1_175875602 + JPEG_IDCT_M_2_562915447);
    static const __m128i rot2_0 = JPEG_CONST16_SSE2(JPEG_IDCT_M_1_961570560 + JPEG_IDCT_P_0_298631336, JPEG_IDCT_M_1_961570560                          );
    static const __m128i rot2_1 = JPEG_CONST16_SSE2(JPEG_IDCT_M_1_961570560                          , JPEG_IDCT_M_1_961570560 + JPEG_IDCT_P_3_072711026);
    static const __m128i rot3_0 = JPEG_CONST16_SSE2(JPEG_IDCT_M_0_390180644 + JPEG_IDCT_P_2_053119869, JPEG_IDCT_M_0_390180644                          );
    static const __m128i rot3_1 = JPEG_CONST16_SSE2(JPEG_IDCT_M_0_390180644                          , JPEG_IDCT_M_0_390180644 + JPEG_IDCT_P_1_501321110);
    static const __m128i colBias = JPEG_CONST32_SSE2(JPEG_IDCT_COL_BIAS);
    static const __m128i rowBias = JPEG_CONST32_SSE2(JPEG_IDCT_ROW_BIAS);

#define JPEG_IDCT_ROTATE_XMM(dst0, dst1, x, y, c0, c1) \
    __m128i c0##_l = _mm_unpacklo_epi16(x, y); \
    __m128i c0##_h = _mm_unpackhi_epi16(x, y); \
    __m128i dst0##_l = _mm_madd_epi16(c0##_l, c0); \
    __m128i dst0##_h = _mm_madd_epi16(c0##_h, c0); \
    __m128i dst1##_l = _mm_madd_epi16(c0##_l, c1); \
    __m128i dst1##_h = _mm_madd_epi16(c0##_h, c1);

    // out = in << 12  (in 16-bit, out 32-bit)
#define JPEG_IDCT_WIDEN_XMM(dst, in) \
    __m128i dst##_l = _mm_srai_epi32(_mm_unpacklo_epi16(_mm_setzero_si128(), (in)), 4); \
    __m128i dst##_h = _mm_srai_epi32(_mm_unpackhi_epi16(_mm_setzero_si128(), (in)), 4);

    // wide add
#define JPEG_IDCT_WADD_XMM(dst, a, b) \
    __m128i dst##_l = _mm_add_epi32(a##_l, b##_l); \
    __m128i dst##_h = _mm_add_epi32(a##_h, b##_h);

    // wide sub
#define JPEG_IDCT_WSUB_XMM(dst, a, b) \
    __m128i dst##_l = _mm_sub_epi32(a##_l, b##_l); \
    __m128i dst##_h = _mm_sub_epi32(a##_h, b##_h);

    // butterfly a/b, add bias, then shift by `norm` and pack to 16-bit
#define JPEG_IDCT_BFLY_XMM(dst0, dst1, a, b, bias, norm) { \
    __m128i abiased_l = _mm_add_epi32(a##_l, bias); \
    __m128i abiased_h = _mm_add_epi32(a##_h, bias); \
    JPEG_IDCT_WADD_XMM(sum, abiased, b) \
    JPEG_IDCT_WSUB_XMM(diff, abiased, b) \
    dst0 = _mm_packs_epi32(_mm_srai_epi32(sum_l, norm), _mm_srai_epi32(sum_h, norm)); \
    dst1 = _mm_packs_epi32(_mm_srai_epi32(diff_l, norm), _mm_srai_epi32(diff_h, norm)); \
    }

#define JPEG_IDCT_IDCT_PASS_XMM(bias, norm) { \
    JPEG_IDCT_ROTATE_XMM(t2e, t3e, v2, v6, rot0_0, rot0_1) \
    __m128i sum04 = _mm_add_epi16(v0, v4); \
    __m128i dif04 = _mm_sub_epi16(v0, v4); \
    JPEG_IDCT_WIDEN_XMM(t0e, sum04) \
    JPEG_IDCT_WIDEN_XMM(t1e, dif04) \
    JPEG_IDCT_WADD_XMM(x0, t0e, t3e) \
    JPEG_IDCT_WSUB_XMM(x3, t0e, t3e) \
    JPEG_IDCT_WADD_XMM(x1, t1e, t2e) \
    JPEG_IDCT_WSUB_XMM(x2, t1e, t2e) \
    JPEG_IDCT_ROTATE_XMM(y0o, y2o, v7, v3, rot2_0, rot2_1) \
    JPEG_IDCT_ROTATE_XMM(y1o, y3o, v5, v1, rot3_0, rot3_1) \
    __m128i sum17 = _mm_add_epi16(v1, v7); \
    __m128i sum35 = _mm_add_epi16(v3, v5); \
    JPEG_IDCT_ROTATE_XMM(y4o,y5o, sum17, sum35, rot1_0, rot1_1) \
    JPEG_IDCT_WADD_XMM(x4, y0o, y4o) \
    JPEG_IDCT_WADD_XMM(x5, y1o, y5o) \
    JPEG_IDCT_WADD_XMM(x6, y2o, y5o) \
    JPEG_IDCT_WADD_XMM(x7, y3o, y4o) \
    JPEG_IDCT_BFLY_XMM(v0, v7, x0, x7, bias, norm) \
    JPEG_IDCT_BFLY_XMM(v1, v6, x1, x6, bias, norm) \
    JPEG_IDCT_BFLY_XMM(v2, v5, x2, x5, bias, norm) \
    JPEG_IDCT_BFLY_XMM(v3, v4, x3, x4, bias, norm) \
    }

    static inline void interleave8(__m128i &a, __m128i &b)
    {
        __m128i c = a;
        a = _mm_unpacklo_epi8(a, b);
        b = _mm_unpackhi_epi8(c, b);
    }

    static inline void interleave16(__m128i &a, __m128i &b)
    {
        __m128i c = a;
        a = _mm_unpacklo_epi16(a, b);
        b = _mm_unpackhi_epi16(c, b);
    }

    void idct_sse2(u8* dest, const s16* src, const s16* qt)
    {
        const __m128i* data = reinterpret_cast<const __m128i *>(src);
        const __m128i* qtable = reinterpret_cast<const __m128i *>(qt);

        // Load and dequantize
        __m128i v0 = _mm_mullo_epi16(data[0], qtable[0]);
        __m128i v1 = _mm_mullo_epi16(data[1], qtable[1]);
        __m128i v2 = _mm_mullo_epi16(data[2], qtable[2]);
        __m128i v3 = _mm_mullo_epi16(data[3], qtable[3]);
        __m128i v4 = _mm_mullo_epi16(data[4], qtable[4]);
        __m128i v5 = _mm_mullo_epi16(data[5], qtable[5]);
        __m128i v6 = _mm_mullo_epi16(data[6], qtable[6]);
        __m128i v7 = _mm_mullo_epi16(data[7], qtable[7]);

        // IDCT columns
        JPEG_IDCT_IDCT_PASS_XMM(colBias, 10)

        // Transpose
        interleave16(v0, v4);
        interleave16(v2, v6);
        interleave16(v1, v5);
        interleave16(v3, v7);

        interleave16(v0, v2);
        interleave16(v1, v3);
        interleave16(v4, v6);
        interleave16(v5, v7);

        interleave16(v0, v1);
        interleave16(v2, v3);
        interleave16(v4, v5);
        interleave16(v6, v7);

        // IDCT rows
        JPEG_IDCT_IDCT_PASS_XMM(rowBias, 17)

        // Pack to 8-bit integers, also saturates the result to 0..255
        __m128i s0 = _mm_packus_epi16(v0, v1);
        __m128i s1 = _mm_packus_epi16(v2, v3);
        __m128i s2 = _mm_packus_epi16(v4, v5);
        __m128i s3 = _mm_packus_epi16(v6, v7);

        // Transpose
        interleave8(s0, s2);
        interleave8(s1, s3);
        interleave8(s0, s1);
        interleave8(s2, s3);
        interleave8(s0, s2);
        interleave8(s1, s3);

        // Store
        __m128i* d = reinterpret_cast<__m128i *>(dest);
        _mm_storeu_si128(d + 0, s0);
        _mm_storeu_si128(d + 1, s2);
        _mm_storeu_si128(d + 2, s1);
        _mm_storeu_si128(d + 3, s3);
    }

#endif // MANGO_ENABLE_SSE2

#if defined(MANGO_ENABLE_NEON)

    // ------------------------------------------------------------------------------------------------
    // NEON implementation
    // ------------------------------------------------------------------------------------------------

    // The original code is by Sean Barrett ; WE HAVE TAKEN LIBERTIES TO ADAPT IT TO OUR USE!!!
    // https://github.com/nothings/stb
    // [License]
    // Public Domain / MIT

    #define JPEG_FIXED(x)  ((int) (((x) * 4096 + 0.5)))

    static inline
    void dct_trn16(int16x8_t& x, int16x8_t& y)
    {
        int16x8x2_t t = vtrnq_s16(x, y);
        x = t.val[0];
        y = t.val[1];
    }

    static inline
    void dct_trn32(int16x8_t& x, int16x8_t& y)
    {
        int32x4x2_t t = vtrnq_s32(vreinterpretq_s32_s16(x), vreinterpretq_s32_s16(y));
        x = vreinterpretq_s16_s32(t.val[0]);
        y = vreinterpretq_s16_s32(t.val[1]);
    }

    static inline
    void dct_trn64(int16x8_t& x, int16x8_t& y)
    {
        int16x8_t x0 = x;
        int16x8_t y0 = y;
        x = vcombine_s16(vget_low_s16(x0), vget_low_s16(y0));
        y = vcombine_s16(vget_high_s16(x0), vget_high_s16(y0));
    }

    static inline
    void dct_trn8(uint8x8_t& x, uint8x8_t& y)
    {
        uint8x8x2_t t = vtrn_u8(x, y);
        x = t.val[0];
        y = t.val[1];
    }

    static inline
    void dct_trn16(uint8x8_t& x, uint8x8_t& y)
    {
        uint16x4x2_t t = vtrn_u16(vreinterpret_u16_u8(x), vreinterpret_u16_u8(y));
        x = vreinterpret_u8_u16(t.val[0]);
        y = vreinterpret_u8_u16(t.val[1]);
    }

    static inline
    void dct_trn32(uint8x8_t& x, uint8x8_t& y)
    {
        uint32x2x2_t t = vtrn_u32(vreinterpret_u32_u8(x), vreinterpret_u32_u8(y));
        x = vreinterpret_u8_u32(t.val[0]);
        y = vreinterpret_u8_u32(t.val[1]);
    }

#define dct_long_mul(out, inq, coeff) \
    int32x4_t out##_l = vmull_s16(vget_low_s16(inq), coeff); \
    int32x4_t out##_h = vmull_s16(vget_high_s16(inq), coeff)

#define dct_long_mac(out, acc, inq, coeff) \
    int32x4_t out##_l = vmlal_s16(acc##_l, vget_low_s16(inq), coeff); \
    int32x4_t out##_h = vmlal_s16(acc##_h, vget_high_s16(inq), coeff)

#define dct_widen(out, inq) \
    int32x4_t out##_l = vshll_n_s16(vget_low_s16(inq), 12); \
    int32x4_t out##_h = vshll_n_s16(vget_high_s16(inq), 12)

// wide add
#define dct_wadd(out, a, b) \
    int32x4_t out##_l = vaddq_s32(a##_l, b##_l); \
    int32x4_t out##_h = vaddq_s32(a##_h, b##_h)

// wide sub
#define dct_wsub(out, a, b) \
    int32x4_t out##_l = vsubq_s32(a##_l, b##_l); \
    int32x4_t out##_h = vsubq_s32(a##_h, b##_h)

// butterfly a/b, then shift using "shiftop" by "s" and pack
#define dct_bfly32o(out0, out1, a, b, shiftop, s) \
    { \
        dct_wadd(sum, a, b); \
        dct_wsub(dif, a, b); \
        out0 = vcombine_s16(shiftop(sum_l, s), shiftop(sum_h, s)); \
        out1 = vcombine_s16(shiftop(dif_l, s), shiftop(dif_h, s)); \
    }

#define dct_pass(shiftop, shift) \
    { \
        /* even part */ \
        int16x8_t sum26 = vaddq_s16(row2, row6); \
        dct_long_mul(p1e, sum26, rot0_0); \
        dct_long_mac(t2e, p1e, row6, rot0_1); \
        dct_long_mac(t3e, p1e, row2, rot0_2); \
        int16x8_t sum04 = vaddq_s16(row0, row4); \
        int16x8_t dif04 = vsubq_s16(row0, row4); \
        dct_widen(t0e, sum04); \
        dct_widen(t1e, dif04); \
        dct_wadd(x0, t0e, t3e); \
        dct_wsub(x3, t0e, t3e); \
        dct_wadd(x1, t1e, t2e); \
        dct_wsub(x2, t1e, t2e); \
        /* odd part */ \
        int16x8_t sum15 = vaddq_s16(row1, row5); \
        int16x8_t sum17 = vaddq_s16(row1, row7); \
        int16x8_t sum35 = vaddq_s16(row3, row5); \
        int16x8_t sum37 = vaddq_s16(row3, row7); \
        int16x8_t sumodd = vaddq_s16(sum17, sum35); \
        dct_long_mul(p5o, sumodd, rot1_0); \
        dct_long_mac(p1o, p5o, sum17, rot1_1); \
        dct_long_mac(p2o, p5o, sum35, rot1_2); \
        dct_long_mul(p3o, sum37, rot2_0); \
        dct_long_mul(p4o, sum15, rot2_1); \
        dct_wadd(sump13o, p1o, p3o); \
        dct_wadd(sump24o, p2o, p4o); \
        dct_wadd(sump23o, p2o, p3o); \
        dct_wadd(sump14o, p1o, p4o); \
        dct_long_mac(x4, sump13o, row7, rot3_0); \
        dct_long_mac(x5, sump24o, row5, rot3_1); \
        dct_long_mac(x6, sump23o, row3, rot3_2); \
        dct_long_mac(x7, sump14o, row1, rot3_3); \
        dct_bfly32o(row0,row7, x0,x7,shiftop,shift); \
        dct_bfly32o(row1,row6, x1,x6,shiftop,shift); \
        dct_bfly32o(row2,row5, x2,x5,shiftop,shift); \
        dct_bfly32o(row3,row4, x3,x4,shiftop,shift); \
    }

void idct_neon(u8* out, const s16* data, const s16* qt)
{
    const int16x4_t rot0_0 = vdup_n_s16(JPEG_FIXED(0.5411961f));
    const int16x4_t rot0_1 = vdup_n_s16(JPEG_FIXED(-1.847759065f));
    const int16x4_t rot0_2 = vdup_n_s16(JPEG_FIXED( 0.765366865f));
    const int16x4_t rot1_0 = vdup_n_s16(JPEG_FIXED( 1.175875602f));
    const int16x4_t rot1_1 = vdup_n_s16(JPEG_FIXED(-0.899976223f));
    const int16x4_t rot1_2 = vdup_n_s16(JPEG_FIXED(-2.562915447f));
    const int16x4_t rot2_0 = vdup_n_s16(JPEG_FIXED(-1.961570560f));
    const int16x4_t rot2_1 = vdup_n_s16(JPEG_FIXED(-0.390180644f));
    const int16x4_t rot3_0 = vdup_n_s16(JPEG_FIXED( 0.298631336f));
    const int16x4_t rot3_1 = vdup_n_s16(JPEG_FIXED( 2.053119869f));
    const int16x4_t rot3_2 = vdup_n_s16(JPEG_FIXED( 3.072711026f));
    const int16x4_t rot3_3 = vdup_n_s16(JPEG_FIXED( 1.501321110f));

    // load
    int16x8_t row0 = vld1q_s16(data + 0 * 8);
    int16x8_t row1 = vld1q_s16(data + 1 * 8);
    int16x8_t row2 = vld1q_s16(data + 2 * 8);
    int16x8_t row3 = vld1q_s16(data + 3 * 8);
    int16x8_t row4 = vld1q_s16(data + 4 * 8);
    int16x8_t row5 = vld1q_s16(data + 5 * 8);
    int16x8_t row6 = vld1q_s16(data + 6 * 8);
    int16x8_t row7 = vld1q_s16(data + 7 * 8);

    int16x8_t qt0 = vld1q_s16(qt + 0 * 8);
    int16x8_t qt1 = vld1q_s16(qt + 1 * 8);
    int16x8_t qt2 = vld1q_s16(qt + 2 * 8);
    int16x8_t qt3 = vld1q_s16(qt + 3 * 8);
    int16x8_t qt4 = vld1q_s16(qt + 4 * 8);
    int16x8_t qt5 = vld1q_s16(qt + 5 * 8);
    int16x8_t qt6 = vld1q_s16(qt + 6 * 8);
    int16x8_t qt7 = vld1q_s16(qt + 7 * 8);

    row0 = vmulq_s16(row0, qt0);
    row1 = vmulq_s16(row1, qt1);
    row2 = vmulq_s16(row2, qt2);
    row3 = vmulq_s16(row3, qt3);
    row4 = vmulq_s16(row4, qt4);
    row5 = vmulq_s16(row5, qt5);
    row6 = vmulq_s16(row6, qt6);
    row7 = vmulq_s16(row7, qt7);

    // add DC bias
    row0 = vaddq_s16(row0, vsetq_lane_s16(1024, vdupq_n_s16(0), 0));

    // column pass
    dct_pass(vrshrn_n_s32, 10);

    // 16bit 8x8 transpose
    dct_trn16(row0, row1);
    dct_trn16(row2, row3);
    dct_trn16(row4, row5);
    dct_trn16(row6, row7);
    dct_trn32(row0, row2);
    dct_trn32(row1, row3);
    dct_trn32(row4, row6);
    dct_trn32(row5, row7);
    dct_trn64(row0, row4);
    dct_trn64(row1, row5);
    dct_trn64(row2, row6);
    dct_trn64(row3, row7);

    // row pass
    // vrshrn_n_s32 only supports shifts up to 16, we need
    // 17. so do a non-rounding shift of 16 first then follow
    // up with a rounding shift by 1.
    dct_pass(vshrn_n_s32, 16);

    // pack and round
    uint8x8_t p0 = vqrshrun_n_s16(row0, 1);
    uint8x8_t p1 = vqrshrun_n_s16(row1, 1);
    uint8x8_t p2 = vqrshrun_n_s16(row2, 1);
    uint8x8_t p3 = vqrshrun_n_s16(row3, 1);
    uint8x8_t p4 = vqrshrun_n_s16(row4, 1);
    uint8x8_t p5 = vqrshrun_n_s16(row5, 1);
    uint8x8_t p6 = vqrshrun_n_s16(row6, 1);
    uint8x8_t p7 = vqrshrun_n_s16(row7, 1);

    // 8x8 8-bit transpose
    dct_trn8(p0, p1);
    dct_trn8(p2, p3);
    dct_trn8(p4, p5);
    dct_trn8(p6, p7);
    dct_trn16(p0, p2);
    dct_trn16(p1, p3);
    dct_trn16(p4, p6);
    dct_trn16(p5, p7);
    dct_trn32(p0, p4);
    dct_trn32(p1, p5);
    dct_trn32(p2, p6);
    dct_trn32(p3, p7);

    // store
    vst1_u8(out, p0); out += 8;
    vst1_u8(out, p1); out += 8;
    vst1_u8(out, p2); out += 8;
    vst1_u8(out, p3); out += 8;
    vst1_u8(out, p4); out += 8;
    vst1_u8(out, p5); out += 8;
    vst1_u8(out, p6); out += 8;
    vst1_u8(out, p7);
}

#undef dct_long_mul
#undef dct_long_mac
#undef dct_widen
#undef dct_wadd
#undef dct_wsub
#undef dct_bfly32o
#undef dct_pass

#endif // MANGO_ENABLE_NEON

} // namespace mango::jpeg

// ----------------------------------------------------------------------------
// process
// ----------------------------------------------------------------------------

namespace mango::jpeg
{

// ----------------------------------------------------------------------------
// color conversion
// ----------------------------------------------------------------------------

/*

    Full-range YCbCr color conversion:

    |R|     |1.000  0.000  1.400|     | Y      |
    |G|  =  |1.000 -0.343 -0.711|  *  |Cb - 128|
    |B|     |1.000  1.765  0.000|     |Cr - 128|

    Pseudo code:

    R = Y +               Cr * 1.400   - (128 * 1.4)
    G = Y + Cb * -0.343 + Cr * -0.711  - (128 * (-0.343 - 0.711))
    B = Y + Cb * 1.765                 - (128 * 1.765)

    NOTE: ITU BT.601 would be the correct method :)
*/

#define COMPUTE_CBCR(cb, cr) \
    r = (cr * 91750 - 11711232) >> 16; \
    g = (cb * -22479 + cr * -46596 + 8874368) >> 16; \
    b = (cb * 115671 - 14773120) >> 16;

// ----------------------------------------------------------------------------
// Generic C++ implementation
// ----------------------------------------------------------------------------

void process_y_8bit(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height)
{
    u8 result[64];
    state->idct(result, data, state->block[0].qt); // Y

    for (int y = 0; y < height; ++y)
    {
        std::memcpy(dest, result + y * 8, width);
        dest += stride;
    }
}

void process_y_24bit(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height)
{
    u8 result[64];
    state->idct(result, data, state->block[0].qt); // Y

    stride -= width * 3;

    for (int y = 0; y < height; ++y)
    {
        const u8* s = result + y * 8;
        for (int x = 0; x < width; ++x)
        {
            u8 v = s[x];
            dest[0] = v;
            dest[1] = v;
            dest[2] = v;
            dest += 3;
        }
        dest += stride;
    }
}

void process_y_32bit(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height)
{
    u8 result[64];
    state->idct(result, data, state->block[0].qt); // Y

    for (int y = 0; y < height; ++y)
    {
        const u8* s = result + y * 8;
        u32* d = reinterpret_cast<u32*>(dest);
        for (int x = 0; x < width; ++x)
        {
            u32 v = s[x];
            d[x] = 0xff000000 | (v << 16) | (v << 8) | v;
        }
        dest += stride;
    }
}

void process_cmyk_bgra(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height)
{
    u8 result[JPEG_MAX_SAMPLES_IN_MCU];

    for (int i = 0; i < state->blocks; ++i)
    {
        Block& block = state->block[i];
        state->idct(result + i * 64, data, block.qt);
        data += 64;
    }

    // MCU size in blocks
    int xsize = (width + 7) / 8;
    int ysize = (height + 7) / 8;

    int cb_offset = state->frame[1].offset * 64;
    int cb_xshift = state->frame[1].Hsf;
    int cb_yshift = state->frame[1].Vsf;

    int cr_offset = state->frame[2].offset * 64;
    int cr_xshift = state->frame[2].Hsf;
    int cr_yshift = state->frame[2].Vsf;

    int ck_offset = state->frame[3].offset * 64;
    int ck_xshift = state->frame[3].Hsf;
    int ck_yshift = state->frame[3].Vsf;

    u8* cb_data = result + cb_offset;
    u8* cr_data = result + cr_offset;
    u8* ck_data = result + ck_offset;

    const ColorSpace colorspace = state->colorspace;

    // process MCU
    for (int yb = 0; yb < ysize; ++yb)
    {
        // vertical clipping limit for current block
        const int ymax = std::min(8, height - yb * 8);

        for (int xb = 0; xb < xsize; ++xb)
        {
            u8* dest_block = dest + yb * 8 * stride + xb * 8 * sizeof(u32);
            u8* y_block = result + (yb * xsize + xb) * 64;
            u8* cb_block = cb_data + yb * (8 >> cb_yshift) * 8 + xb * (8 >> cb_xshift);
            u8* cr_block = cr_data + yb * (8 >> cr_yshift) * 8 + xb * (8 >> cr_xshift);
            u8* ck_block = ck_data + yb * (8 >> ck_yshift) * 8 + xb * (8 >> ck_xshift);

            // horizontal clipping limit for current block
            const int xmax = std::min(8, width - xb * 8);

            // process 8x8 block
            for (int y = 0; y < ymax; ++y)
            {
                u32* d = reinterpret_cast<u32*>(dest_block);

                u8* cb_scan = cb_block + (y >> cb_yshift) * 8;
                u8* cr_scan = cr_block + (y >> cr_yshift) * 8;
                u8* ck_scan = ck_block + (y >> ck_yshift) * 8;

                for (int x = 0; x < xmax; ++x)
                {
                    u8 y0 = y_block[x];
                    u8 cb = cb_scan[x >> cb_xshift];
                    u8 cr = cr_scan[x >> cr_xshift];
                    u8 ck = ck_scan[x >> ck_xshift];

                    int C;
                    int M;
                    int Y;
                    int K;

                    switch (colorspace)
                    {
                        case ColorSpace::CMYK:
                            C = y0;
                            M = cb;
                            Y = cr;
                            K = ck;
                            break;
                        case ColorSpace::YCCK:
                            // convert YCCK to CMYK
                            C = 255 - (y0 + ((5734 * cr - 735052) >> 12));
                            M = 255 - (y0 + ((-1410 * cb - 2925 * cr + 554844) >> 12));
                            Y = 255 - (y0 + ((7258 * cb - 929038) >> 12));
                            K = ck;
                            break;
                        default:
                        case ColorSpace::YCBCR:
                            C = 0;
                            M = 0;
                            Y = 0;
                            K = 0;
                            break;
                    }

                    // NOTE: We should output "raw" CMYK here so that it can be mapped into
                    //       RGB with correct ICC color profile. It's mot JPEG encoder/decoder's
                    //       responsibility to handle color management.
                    //
                    // We don't have API to expose the CMYK color data so we do the worst possible
                    // thing and approximate the RGB colors. THIS IS VERY BAD!!!!!
                    //
                    // TODO: Proposed API is to expose CMYK as "packed pixels" compressed image format,
                    //       we DO have a mechanism for that. Alternatively, we could add CMYK
                    //       color type in the mango::Format. We already expose sRGB-U8 this way.
                    int r = (C * K) / 255;
                    int g = (M * K) / 255;
                    int b = (Y * K) / 255;

                    r = byteclamp(r);
                    g = byteclamp(g);
                    b = byteclamp(b);
                    d[x] = image::makeBGRA(r, g, b, 0xff);
                }
                dest_block += stride;
                y_block += 8;
            }
        }
    }
}

void process_ycbcr_8bit(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height)
{
    u8 result[JPEG_MAX_SAMPLES_IN_MCU];

    const int luma_blocks = state->blocks - 2; // don't idct two last blocks (Cb, Cr)
    for (int i = 0; i < luma_blocks; ++i)
    {
        Block& block = state->block[i];
        state->idct(result + i * 64, data, block.qt);
        data += 64;
    }

    // MCU size in blocks
    int xsize = (width + 7) / 8;
    int ysize = (height + 7) / 8;

    // process MCU
    for (int yb = 0; yb < ysize; ++yb)
    {
        // vertical clipping limit for current block
        const int ymax = std::min(8, height - yb * 8);

        for (int xb = 0; xb < xsize; ++xb)
        {
            u8* dest_block = dest + yb * 8 * stride + xb * 8 * sizeof(u8);
            u8* y_block = result + (yb * xsize + xb) * 64;

            // horizontal clipping limit for current block
            const int xmax = std::min(8, width - xb * 8);

            // process 8x8 block
            for (int y = 0; y < ymax; ++y)
            {
                std::memcpy(dest_block, y_block, xmax);
                dest_block += stride;
                y_block += 8;
            }
        }
    }
}

static inline void write_color_bgra(u8* dest, int y, int r, int g, int b)
{
    dest[0] = byteclamp(b + y);
    dest[1] = byteclamp(g + y);
    dest[2] = byteclamp(r + y);
    dest[3] = 0xff;
}

static inline void write_color_rgba(u8* dest, int y, int r, int g, int b)
{
    dest[0] = byteclamp(r + y);
    dest[1] = byteclamp(g + y);
    dest[2] = byteclamp(b + y);
    dest[3] = 0xff;
}

static inline void write_color_bgr(u8* dest, int y, int r, int g, int b)
{
    dest[0] = byteclamp(b + y);
    dest[1] = byteclamp(g + y);
    dest[2] = byteclamp(r + y);
}

static inline void write_color_rgb(u8* dest, int y, int r, int g, int b)
{
    dest[0] = byteclamp(r + y);
    dest[1] = byteclamp(g + y);
    dest[2] = byteclamp(b + y);
}

namespace process::func
{
    typedef void WRITE_COLOR_FUNC(u8*, int, int, int, int);
    void FUNCTION_GENERIC(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height, const int XSTEP, WRITE_COLOR_FUNC * WRITE_COLOR)
    {
        u8 result[JPEG_MAX_SAMPLES_IN_MCU];

        for (int i = 0; i < state->blocks; ++i)
        {
            Block& block = state->block[i];
            state->idct(result + i * 64, data, block.qt);
            data += 64;
        }

        // MCU size in blocks
        int xsize = (width + 7) / 8;
        int ysize = (height + 7) / 8;

        int cb_offset = state->frame[1].offset * 64;
        int cb_xshift = state->frame[1].Hsf;
        int cb_yshift = state->frame[1].Vsf;

        int cr_offset = state->frame[2].offset * 64;
        int cr_xshift = state->frame[2].Hsf;
        int cr_yshift = state->frame[2].Vsf;

        u8* cb_data = result + cb_offset;
        u8* cr_data = result + cr_offset;

        // process MCU
        for (int yb = 0; yb < ysize; ++yb)
        {
            // vertical clipping limit for current block
            const int ymax = std::min(8, height - yb * 8);

            for (int xb = 0; xb < xsize; ++xb)
            {
                u8* dest_block = dest + yb * 8 * stride + xb * 8 * XSTEP;
                u8* y_block = result + (yb * xsize + xb) * 64;
                u8* cb_block = cb_data + yb * (8 >> cb_yshift) * 8 + xb * (8 >> cb_xshift);
                u8* cr_block = cr_data + yb * (8 >> cr_yshift) * 8 + xb * (8 >> cr_xshift);

                // horizontal clipping limit for current block
                const int xmax = std::min(8, width - xb * 8);

                // process 8x8 block
                for (int y = 0; y < ymax; ++y)
                {
                    u8* d = dest_block;
                    u8* cb_scan = cb_block + (y >> cb_yshift) * 8;
                    u8* cr_scan = cr_block + (y >> cr_yshift) * 8;

                    for (int x = 0; x < xmax; ++x)
                    {
                        u8 y0 = y_block[x];
                        u8 cb = cb_scan[x >> cb_xshift];
                        u8 cr = cr_scan[x >> cr_xshift];
                        int r, g, b;
                        COMPUTE_CBCR(cb, cr);
                        WRITE_COLOR(d, y0, r, g, b);
                        d += XSTEP;
                    }

                    dest_block += stride;
                    y_block += 8;
                }
            }
        }
    }
    void FUNCTION_YCBCR_8x8(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height, const int XSTEP, WRITE_COLOR_FUNC * WRITE_COLOR)
    {
        u8 result[64 * 3];

        state->idct(result + 64 * 0, data + 64 * 0, state->block[0].qt); // Y
        state->idct(result + 64 * 1, data + 64 * 1, state->block[1].qt); // Cb
        state->idct(result + 64 * 2, data + 64 * 2, state->block[2].qt); // Cr

        // color conversion
        const u8* src = result;

        for (int y = 0; y < 8; ++y)
        {
            const u8* s = src + y * 8;
            u8* d = dest;

            for (int x = 0; x < 8; ++x)
            {
                int y0 = s[x];
                int cb = s[x + 64];
                int cr = s[x + 128];
                int r, g, b;
                COMPUTE_CBCR(cb, cr);
                WRITE_COLOR(d, y0, r, g, b);
                d += XSTEP;
            }

            dest += stride;
        }

        MANGO_UNREFERENCED(width);
        MANGO_UNREFERENCED(height);
    }
    void FUNCTION_YCBCR_8x16(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height, const int XSTEP, WRITE_COLOR_FUNC * WRITE_COLOR)
    {
        u8 result[64 * 4];

        state->idct(result +   0, data +   0, state->block[0].qt); // Y0
        state->idct(result +  64, data +  64, state->block[1].qt); // Y1
        state->idct(result + 128, data + 128, state->block[2].qt); // Cb
        state->idct(result + 192, data + 192, state->block[3].qt); // Cr

        // color conversion
        for (int y = 0; y < 8; ++y)
        {
            u8* d0 = dest;
            u8* d1 = dest + stride;
            const u8* s = result + y * 16;
            const u8* c = result + y * 8 + 128;

            for (int x = 0; x < 8; ++x)
            {
                int y0 = s[x + 0];
                int y1 = s[x + 8];
                int cb = c[x + 0];
                int cr = c[x + 64];
                int r, g, b;
                COMPUTE_CBCR(cb, cr);
                WRITE_COLOR(d0, y0, r, g, b);
                WRITE_COLOR(d1, y1, r, g, b);
                d0 += XSTEP;
                d1 += XSTEP;
            }

            dest += stride * 2;
        }

        MANGO_UNREFERENCED(width);
        MANGO_UNREFERENCED(height);
    }
    void FUNCTION_YCBCR_16x8(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height, const int XSTEP, WRITE_COLOR_FUNC * WRITE_COLOR)
    {
        u8 result[64 * 4];

        state->idct(result +   0, data +   0, state->block[0].qt); // Y0
        state->idct(result +  64, data +  64, state->block[1].qt); // Y1
        state->idct(result + 128, data + 128, state->block[2].qt); // Cb
        state->idct(result + 192, data + 192, state->block[3].qt); // Cr

        // color conversion
        for (int y = 0; y < 8; ++y)
        {
            u8* d = dest;
            u8* s = result + y * 8;
            u8* c = result + y * 8 + 128;

            for (int x = 0; x < 4; ++x)
            {
                int y0 = s[x * 2 + 0];
                int y1 = s[x * 2 + 1];
                int cb = c[x + 0];
                int cr = c[x + 64];
                int r, g, b;
                COMPUTE_CBCR(cb, cr);
                WRITE_COLOR(d + 0 * XSTEP, y0, r, g, b);
                WRITE_COLOR(d + 1 * XSTEP, y1, r, g, b);
                d += 2 * XSTEP;
            }

            for (int x = 0; x < 4; ++x)
            {
                int y0 = s[x * 2 + 64];
                int y1 = s[x * 2 + 65];
                int cb = c[x + 4];
                int cr = c[x + 68];
                int r, g, b;
                COMPUTE_CBCR(cb, cr);
                WRITE_COLOR(d + 0 * XSTEP, y0, r, g, b);
                WRITE_COLOR(d + 1 * XSTEP, y1, r, g, b);
                d += 2 * XSTEP;
            }

            dest += stride;
        }

        MANGO_UNREFERENCED(width);
        MANGO_UNREFERENCED(height);
    }
    void FUNCTION_YCBCR_16x16(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height, const int XSTEP, WRITE_COLOR_FUNC * WRITE_COLOR)
    {
        u8 result[64 * 6];

        state->idct(result +   0, data +   0, state->block[0].qt); // Y0
        state->idct(result +  64, data +  64, state->block[1].qt); // Y1
        state->idct(result + 128, data + 128, state->block[2].qt); // Y2
        state->idct(result + 192, data + 192, state->block[3].qt); // Y3
        state->idct(result + 256, data + 256, state->block[4].qt); // Cb
        state->idct(result + 320, data + 320, state->block[5].qt); // Cr

        // color conversion
        for (int i = 0; i < 4; ++i)
        {
            int cbcr_offset = (i & 1) * 4 + (i >> 1) * 32;
            int y_offset = i * 64;
            size_t dest_offset = (i >> 1) * 8 * stride + (i & 1) * 8 * XSTEP;
            const u8* ptr_cbcr = result + 256 + cbcr_offset;
            const u8* ptr_y = result + y_offset;
            u8* ptr_dest = dest + dest_offset;

            for (int y = 0; y < 4; ++y)
            {
                u8* scan = ptr_dest;

                for (int x = 0; x < 4; ++x)
                {
                    u8 y0 = ptr_y[x * 2 + 0];
                    u8 y1 = ptr_y[x * 2 + 1];
                    u8 y2 = ptr_y[x * 2 + 8];
                    u8 y3 = ptr_y[x * 2 + 9];
                    u8 cb = ptr_cbcr[x + 0];
                    u8 cr = ptr_cbcr[x + 64];

                    int r, g, b;
                    COMPUTE_CBCR(cb, cr);
                    WRITE_COLOR(scan + 0 * XSTEP, y0, r, g, b);
                    WRITE_COLOR(scan + 1 * XSTEP, y1, r, g, b);

                    u8* next = scan + stride;
                    scan += 2 * XSTEP;
                    WRITE_COLOR(next + 0 * XSTEP, y2, r, g, b);
                    WRITE_COLOR(next + 1 * XSTEP, y3, r, g, b);
                }

                ptr_dest += stride * 2;
                ptr_y += 8 * 2;
                ptr_cbcr += 8;
            }
        }

        MANGO_UNREFERENCED(width);
        MANGO_UNREFERENCED(height);
    }
}

// Generate YCBCR to BGRA functions
void process_ycbcr_bgra(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_GENERIC(dest, stride, data, state, width, height,
        4, write_color_bgra);
}
void process_ycbcr_bgra_8x8(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_8x8(dest, stride, data, state, width, height,
        4, write_color_bgra);
}
void process_ycbcr_bgra_8x16(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_8x16(dest, stride, data, state, width, height,
        4, write_color_bgra);
}
void process_ycbcr_bgra_16x8(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_16x8(dest, stride, data, state, width, height,
        4, write_color_bgra);
}
void process_ycbcr_bgra_16x16(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_16x16(dest, stride, data, state, width, height,
        4, write_color_bgra);
}

// Generate YCBCR to RGBA functions
void process_ycbcr_rgba(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_GENERIC(dest, stride, data, state, width, height,
        4, write_color_rgba);
}
void process_ycbcr_rgba_8x8(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_8x8(dest, stride, data, state, width, height,
        4, write_color_rgba);
}
void process_ycbcr_rgba_8x16(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_8x16(dest, stride, data, state, width, height,
        4, write_color_rgba);
}
void process_ycbcr_rgba_16x8(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_16x8(dest, stride, data, state, width, height,
        4, write_color_rgba);
}
void process_ycbcr_rgba_16x16(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_16x16(dest, stride, data, state, width, height,
        4, write_color_rgba);
}

// Generate YCBCR to BGR functions
void process_ycbcr_bgr(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_GENERIC(dest, stride, data, state, width, height,
        3, write_color_bgr);
}
void process_ycbcr_bgr_8x8(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_8x8(dest, stride, data, state, width, height,
        3, write_color_bgr);
}
void process_ycbcr_bgr_8x16(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_8x16(dest, stride, data, state, width, height,
        3, write_color_bgr);
}
void process_ycbcr_bgr_16x8(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_16x8(dest, stride, data, state, width, height,
        3, write_color_bgr);
}
void process_ycbcr_bgr_16x16(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_16x16(dest, stride, data, state, width, height,
        3, write_color_bgr);
}

// Generate YCBCR to RGB functions
void process_ycbcr_rgb(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_GENERIC(dest, stride, data, state, width, height,
        3, write_color_rgb);
}
void process_ycbcr_rgb_8x8(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_8x8(dest, stride, data, state, width, height,
        3, write_color_rgb);
}
void process_ycbcr_rgb_8x16(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_8x16(dest, stride, data, state, width, height,
        3, write_color_rgb);
}
void process_ycbcr_rgb_16x8(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_16x8(dest, stride, data, state, width, height,
        3, write_color_rgb);
}
void process_ycbcr_rgb_16x16(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::func::FUNCTION_YCBCR_16x16(dest, stride, data, state, width, height,
        3, write_color_rgb);
}

#undef COMPUTE_CBCR

#if defined(MANGO_ENABLE_NEON)

// ------------------------------------------------------------------------------------------------
// NEON implementation
// ------------------------------------------------------------------------------------------------

constexpr s16 JPEG_PREC = 12;
constexpr s16 JPEG_FIXED(double x) { return s16((x * double(1 << JPEG_PREC) + 0.5)); }

static inline
void convert_ycbcr_bgra_8x1_neon(u8* dest, int16x8_t y, int16x8_t cb, int16x8_t cr, int16x8_t s0, int16x8_t s1, int16x8_t s2, int16x8_t s3)
{
    int16x8_t cb0 = vqdmulhq_s16(cb, s2);
    int16x8_t cr0 = vqdmulhq_s16(cr, s0);
    int16x8_t cb1 = vqdmulhq_s16(cb, s3);
    int16x8_t cr1 = vqdmulhq_s16(cr, s1);
    int16x8_t r = vaddq_s16(y, cr0);
    int16x8_t g = vaddq_s16(vaddq_s16(y, cb0), cr1);
    int16x8_t b = vaddq_s16(y, cb1);

    uint8x8x4_t packed;
    packed.val[0] = vqrshrun_n_s16(b, 4);
    packed.val[1] = vqrshrun_n_s16(g, 4);
    packed.val[2] = vqrshrun_n_s16(r, 4);
    packed.val[3] = vdup_n_u8(255);
    vst4_u8(dest, packed);
}

static inline
void convert_ycbcr_rgba_8x1_neon(u8* dest, int16x8_t y, int16x8_t cb, int16x8_t cr, int16x8_t s0, int16x8_t s1, int16x8_t s2, int16x8_t s3)
{
    int16x8_t cb0 = vqdmulhq_s16(cb, s2);
    int16x8_t cr0 = vqdmulhq_s16(cr, s0);
    int16x8_t cb1 = vqdmulhq_s16(cb, s3);
    int16x8_t cr1 = vqdmulhq_s16(cr, s1);
    int16x8_t r = vaddq_s16(y, cr0);
    int16x8_t g = vaddq_s16(vaddq_s16(y, cb0), cr1);
    int16x8_t b = vaddq_s16(y, cb1);

    uint8x8x4_t packed;
    packed.val[0] = vqrshrun_n_s16(r, 4);
    packed.val[1] = vqrshrun_n_s16(g, 4);
    packed.val[2] = vqrshrun_n_s16(b, 4);
    packed.val[3] = vdup_n_u8(255);
    vst4_u8(dest, packed);
}

static inline
void convert_ycbcr_bgr_8x1_neon(u8* dest, int16x8_t y, int16x8_t cb, int16x8_t cr, int16x8_t s0, int16x8_t s1, int16x8_t s2, int16x8_t s3)
{
    int16x8_t cb0 = vqdmulhq_s16(cb, s2);
    int16x8_t cr0 = vqdmulhq_s16(cr, s0);
    int16x8_t cb1 = vqdmulhq_s16(cb, s3);
    int16x8_t cr1 = vqdmulhq_s16(cr, s1);
    int16x8_t r = vaddq_s16(y, cr0);
    int16x8_t g = vaddq_s16(vaddq_s16(y, cb0), cr1);
    int16x8_t b = vaddq_s16(y, cb1);

    uint8x8x3_t packed;
    packed.val[0] = vqrshrun_n_s16(b, 4);
    packed.val[1] = vqrshrun_n_s16(g, 4);
    packed.val[2] = vqrshrun_n_s16(r, 4);
    vst3_u8(dest, packed);
}

static inline
void convert_ycbcr_rgb_8x1_neon(u8* dest, int16x8_t y, int16x8_t cb, int16x8_t cr, int16x8_t s0, int16x8_t s1, int16x8_t s2, int16x8_t s3)
{
    int16x8_t cb0 = vqdmulhq_s16(cb, s2);
    int16x8_t cr0 = vqdmulhq_s16(cr, s0);
    int16x8_t cb1 = vqdmulhq_s16(cb, s3);
    int16x8_t cr1 = vqdmulhq_s16(cr, s1);
    int16x8_t r = vaddq_s16(y, cr0);
    int16x8_t g = vaddq_s16(vaddq_s16(y, cb0), cr1);
    int16x8_t b = vaddq_s16(y, cb1);

    uint8x8x3_t packed;
    packed.val[0] = vqrshrun_n_s16(r, 4);
    packed.val[1] = vqrshrun_n_s16(g, 4);
    packed.val[2] = vqrshrun_n_s16(b, 4);
    vst3_u8(dest, packed);
}

namespace process::neon
{
    typedef void INNERLOOP_YCBCR_FUNC(u8*, int16x8_t, int16x8_t, int16x8_t, int16x8_t, int16x8_t, int16x8_t, int16x8_t);
    void FUNCTION_YCBCR_8x8(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height, const int XSTEP, INNERLOOP_YCBCR_FUNC * INNERLOOP_YCBCR)
    {
        u8 result[64 * 3];

        state->idct(result +   0, data +   0, state->block[0].qt); // Y
        state->idct(result +  64, data +  64, state->block[1].qt); // Cb
        state->idct(result + 128, data + 128, state->block[2].qt); // Cr

        const uint8x8_t tosigned = vdup_n_u8(0x80);
        const int16x8_t s0 = vdupq_n_s16(JPEG_FIXED( 1.40200));
        const int16x8_t s1 = vdupq_n_s16(JPEG_FIXED(-0.71414));
        const int16x8_t s2 = vdupq_n_s16(JPEG_FIXED(-0.34414));
        const int16x8_t s3 = vdupq_n_s16(JPEG_FIXED( 1.77200));

        for (int y = 0; y < 8; ++y)
        {
            uint8x8_t u_y  = vld1_u8(result + y * 8 + 0);
            uint8x8_t u_cb = vld1_u8(result + y * 8 + 64);
            uint8x8_t u_cr = vld1_u8(result + y * 8 + 128);

            int16x8_t s_y = vreinterpretq_s16_u16(vshll_n_u8(u_y, 4));
            int16x8_t s_cb = vshll_n_s8(vreinterpret_s8_u8(vsub_u8(u_cb, tosigned)), 7);
            int16x8_t s_cr = vshll_n_s8(vreinterpret_s8_u8(vsub_u8(u_cr, tosigned)), 7);

            INNERLOOP_YCBCR(dest, s_y, s_cb, s_cr, s0, s1, s2, s3);
            dest += stride;
        }

        MANGO_UNREFERENCED(width);
        MANGO_UNREFERENCED(height);
    }
    void FUNCTION_YCBCR_8x16(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height, const int XSTEP, INNERLOOP_YCBCR_FUNC * INNERLOOP_YCBCR)
    {
        u8 result[64 * 4];

        state->idct(result +   0, data +   0, state->block[0].qt); // Y0
        state->idct(result +  64, data +  64, state->block[1].qt); // Y1
        state->idct(result + 128, data + 128, state->block[2].qt); // Cb
        state->idct(result + 192, data + 192, state->block[3].qt); // Cr

        const uint8x8_t tosigned = vdup_n_u8(0x80);
        const int16x8_t s0 = vdupq_n_s16(JPEG_FIXED( 1.40200));
        const int16x8_t s1 = vdupq_n_s16(JPEG_FIXED(-0.71414));
        const int16x8_t s2 = vdupq_n_s16(JPEG_FIXED(-0.34414));
        const int16x8_t s3 = vdupq_n_s16(JPEG_FIXED( 1.77200));

        for (int y = 0; y < 8; ++y)
        {
            uint8x8_t u_y0 = vld1_u8(result + y * 16 + 0);
            uint8x8_t u_y1 = vld1_u8(result + y * 16 + 8);
            uint8x8_t u_cb = vld1_u8(result + y * 8 + 128);
            uint8x8_t u_cr = vld1_u8(result + y * 8 + 192);

            int16x8_t s_y0 = vreinterpretq_s16_u16(vshll_n_u8(u_y0, 4));
            int16x8_t s_y1 = vreinterpretq_s16_u16(vshll_n_u8(u_y1, 4));
            int16x8_t s_cb = vshll_n_s8(vreinterpret_s8_u8(vsub_u8(u_cb, tosigned)), 7);
            int16x8_t s_cr = vshll_n_s8(vreinterpret_s8_u8(vsub_u8(u_cr, tosigned)), 7);

            INNERLOOP_YCBCR(dest, s_y0, s_cb, s_cr, s0, s1, s2, s3);
            dest += stride;

            INNERLOOP_YCBCR(dest, s_y1, s_cb, s_cr, s0, s1, s2, s3);
            dest += stride;
        }

        MANGO_UNREFERENCED(width);
        MANGO_UNREFERENCED(height);
    }
    void FUNCTION_YCBCR_16x8(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height, const int XSTEP, INNERLOOP_YCBCR_FUNC * INNERLOOP_YCBCR)
    {
        u8 result[64 * 4];

        state->idct(result +   0, data +   0, state->block[0].qt); // Y0
        state->idct(result +  64, data +  64, state->block[1].qt); // Y1
        state->idct(result + 128, data + 128, state->block[2].qt); // Cb
        state->idct(result + 192, data + 192, state->block[3].qt); // Cr

        const uint8x8_t tosigned = vdup_n_u8(0x80);
        const int16x8_t s0 = vdupq_n_s16(JPEG_FIXED( 1.40200));
        const int16x8_t s1 = vdupq_n_s16(JPEG_FIXED(-0.71414));
        const int16x8_t s2 = vdupq_n_s16(JPEG_FIXED(-0.34414));
        const int16x8_t s3 = vdupq_n_s16(JPEG_FIXED( 1.77200));

        for (int y = 0; y < 8; ++y)
        {
            uint8x8_t u_y0 = vld1_u8(result + y * 8 + 0);
            uint8x8_t u_y1 = vld1_u8(result + y * 8 + 64);
            uint8x8_t u_cb = vld1_u8(result + y * 8 + 128);
            uint8x8_t u_cr = vld1_u8(result + y * 8 + 192);

            int16x8_t s_y0 = vreinterpretq_s16_u16(vshll_n_u8(u_y0, 4));
            int16x8_t s_y1 = vreinterpretq_s16_u16(vshll_n_u8(u_y1, 4));
            int16x8_t s_cb = vshll_n_s8(vreinterpret_s8_u8(vsub_u8(u_cb, tosigned)), 7);
            int16x8_t s_cr = vshll_n_s8(vreinterpret_s8_u8(vsub_u8(u_cr, tosigned)), 7);

            int16x8x2_t w_cb = vzipq_s16(s_cb, s_cb);
            int16x8x2_t w_cr = vzipq_s16(s_cr, s_cr);

            INNERLOOP_YCBCR(dest + 0 * XSTEP, s_y0, w_cb.val[0], w_cr.val[0], s0, s1, s2, s3);
            INNERLOOP_YCBCR(dest + 1 * XSTEP, s_y1, w_cb.val[1], w_cr.val[1], s0, s1, s2, s3);
            dest += stride;
        }

        MANGO_UNREFERENCED(width);
        MANGO_UNREFERENCED(height);
    }
    void FUNCTION_YCBCR_16x16(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height, const int XSTEP, INNERLOOP_YCBCR_FUNC * INNERLOOP_YCBCR)
    {
        u8 result[64 * 6];

        state->idct(result +   0, data +   0, state->block[0].qt); // Y0
        state->idct(result + 128, data +  64, state->block[1].qt); // Y1
        state->idct(result +  64, data + 128, state->block[2].qt); // Y2
        state->idct(result + 192, data + 192, state->block[3].qt); // Y3
        state->idct(result + 256, data + 256, state->block[4].qt); // Cb
        state->idct(result + 320, data + 320, state->block[5].qt); // Cr

        const uint8x8_t tosigned = vdup_n_u8(0x80);
        const int16x8_t s0 = vdupq_n_s16(JPEG_FIXED( 1.40200));
        const int16x8_t s1 = vdupq_n_s16(JPEG_FIXED(-0.71414));
        const int16x8_t s2 = vdupq_n_s16(JPEG_FIXED(-0.34414));
        const int16x8_t s3 = vdupq_n_s16(JPEG_FIXED( 1.77200));

        for (int y = 0; y < 8; ++y)
        {
            uint8x8_t u_y0 = vld1_u8(result + y * 16 + 0);
            uint8x8_t u_y1 = vld1_u8(result + y * 16 + 128);
            uint8x8_t u_y2 = vld1_u8(result + y * 16 + 8);
            uint8x8_t u_y3 = vld1_u8(result + y * 16 + 136);
            uint8x8_t u_cb = vld1_u8(result + y * 8 + 256);
            uint8x8_t u_cr = vld1_u8(result + y * 8 + 320);

            int16x8_t s_y0 = vreinterpretq_s16_u16(vshll_n_u8(u_y0, 4));
            int16x8_t s_y1 = vreinterpretq_s16_u16(vshll_n_u8(u_y1, 4));
            int16x8_t s_y2 = vreinterpretq_s16_u16(vshll_n_u8(u_y2, 4));
            int16x8_t s_y3 = vreinterpretq_s16_u16(vshll_n_u8(u_y3, 4));
            int16x8_t s_cb = vshll_n_s8(vreinterpret_s8_u8(vsub_u8(u_cb, tosigned)), 7);
            int16x8_t s_cr = vshll_n_s8(vreinterpret_s8_u8(vsub_u8(u_cr, tosigned)), 7);

            int16x8x2_t w_cb = vzipq_s16(s_cb, s_cb);
            int16x8x2_t w_cr = vzipq_s16(s_cr, s_cr);

            INNERLOOP_YCBCR(dest + 0 * XSTEP, s_y0, w_cb.val[0], w_cr.val[0], s0, s1, s2, s3);
            INNERLOOP_YCBCR(dest + 1 * XSTEP, s_y1, w_cb.val[1], w_cr.val[1], s0, s1, s2, s3);
            dest += stride;

            INNERLOOP_YCBCR(dest + 0 * XSTEP, s_y2, w_cb.val[0], w_cr.val[0], s0, s1, s2, s3);
            INNERLOOP_YCBCR(dest + 1 * XSTEP, s_y3, w_cb.val[1], w_cr.val[1], s0, s1, s2, s3);
            dest += stride;
        }

        MANGO_UNREFERENCED(width);
        MANGO_UNREFERENCED(height);
    }
}

// Generate YCBCR to BGRA functions
void process_ycbcr_bgra_8x8_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_8x8(dest, stride, data, state, width, height,
        32, convert_ycbcr_bgra_8x1_neon);
}
void process_ycbcr_bgra_8x16_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_8x16(dest, stride, data, state, width, height,
        32, convert_ycbcr_bgra_8x1_neon);
}
void process_ycbcr_bgra_16x8_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_16x8(dest, stride, data, state, width, height,
        32, convert_ycbcr_bgra_8x1_neon);
}
void process_ycbcr_bgra_16x16_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_16x16(dest, stride, data, state, width, height,
        32, convert_ycbcr_bgra_8x1_neon);
}

// Generate YCBCR to RGBA functions
void process_ycbcr_rgba_8x8_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_8x8(dest, stride, data, state, width, height,
        32, convert_ycbcr_rgba_8x1_neon);
}
void process_ycbcr_rgba_8x16_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_8x16(dest, stride, data, state, width, height,
        32, convert_ycbcr_rgba_8x1_neon);
}
void process_ycbcr_rgba_16x8_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_16x8(dest, stride, data, state, width, height,
        32, convert_ycbcr_rgba_8x1_neon);
}
void process_ycbcr_rgba_16x16_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_16x16(dest, stride, data, state, width, height,
        32, convert_ycbcr_rgba_8x1_neon);
}

// Generate YCBCR to BGR functions
void process_ycbcr_bgr_8x8_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_8x8(dest, stride, data, state, width, height,
        24, convert_ycbcr_bgr_8x1_neon);
}
void process_ycbcr_bgr_8x16_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_8x16(dest, stride, data, state, width, height,
        24, convert_ycbcr_bgr_8x1_neon);
}
void process_ycbcr_bgr_16x8_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_16x8(dest, stride, data, state, width, height,
        24, convert_ycbcr_bgr_8x1_neon);
}
void process_ycbcr_bgr_16x16_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_16x16(dest, stride, data, state, width, height,
        24, convert_ycbcr_bgr_8x1_neon);
}

// Generate YCBCR to RGB functions
void process_ycbcr_rgb_8x8_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_8x8(dest, stride, data, state, width, height,
        24, convert_ycbcr_rgb_8x1_neon);
}
void process_ycbcr_rgb_8x16_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_8x16(dest, stride, data, state, width, height,
        24, convert_ycbcr_rgb_8x1_neon);
}
void process_ycbcr_rgb_16x8_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_16x8(dest, stride, data, state, width, height,
        24, convert_ycbcr_rgb_8x1_neon);
}
void process_ycbcr_rgb_16x16_neon(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::neon::FUNCTION_YCBCR_16x16(dest, stride, data, state, width, height,
        24, convert_ycbcr_rgb_8x1_neon);
}

#endif // MANGO_ENABLE_NEON

#if defined(MANGO_ENABLE_SSE2)

// ------------------------------------------------------------------------------------------------
// SSE2 implementation
// ------------------------------------------------------------------------------------------------

// The original code is by Petr Kobalicek ; WE HAVE TAKEN LIBERTIES TO ADAPT IT TO OUR USE!!!
// https://github.com/kobalicek/simdtests
// [License]
// Public Domain <unlicense.org>

constexpr int JPEG_PREC = 12;
constexpr int JPEG_SCALE(int x) { return x << JPEG_PREC; }
constexpr int JPEG_FIXED(double x) { return int((x * double(1 << JPEG_PREC) + 0.5)); }

#define JPEG_CONST_SSE2(x, y)  _mm_setr_epi16(x, y, x, y, x, y, x, y)

static inline
void convert_ycbcr_bgra_8x1_sse2(u8* dest, __m128i y, __m128i cb, __m128i cr, __m128i s0, __m128i s1, __m128i s2, __m128i rounding)
{
    __m128i zero = _mm_setzero_si128();

    __m128i r_l = _mm_madd_epi16(_mm_unpacklo_epi16(y, cr), s0);
    __m128i r_h = _mm_madd_epi16(_mm_unpackhi_epi16(y, cr), s0);

    __m128i b_l = _mm_madd_epi16(_mm_unpacklo_epi16(y, cb), s1);
    __m128i b_h = _mm_madd_epi16(_mm_unpackhi_epi16(y, cb), s1);

    __m128i g_l = _mm_madd_epi16(_mm_unpacklo_epi16(cb, cr), s2);
    __m128i g_h = _mm_madd_epi16(_mm_unpackhi_epi16(cb, cr), s2);

    g_l = _mm_add_epi32(g_l, _mm_slli_epi32(_mm_unpacklo_epi16(y, zero), JPEG_PREC));
    g_h = _mm_add_epi32(g_h, _mm_slli_epi32(_mm_unpackhi_epi16(y, zero), JPEG_PREC));

    r_l = _mm_add_epi32(r_l, rounding);
    r_h = _mm_add_epi32(r_h, rounding);

    b_l = _mm_add_epi32(b_l, rounding);
    b_h = _mm_add_epi32(b_h, rounding);

    g_l = _mm_add_epi32(g_l, rounding);
    g_h = _mm_add_epi32(g_h, rounding);

    r_l = _mm_srai_epi32(r_l, JPEG_PREC);
    r_h = _mm_srai_epi32(r_h, JPEG_PREC);

    b_l = _mm_srai_epi32(b_l, JPEG_PREC);
    b_h = _mm_srai_epi32(b_h, JPEG_PREC);

    g_l = _mm_srai_epi32(g_l, JPEG_PREC);
    g_h = _mm_srai_epi32(g_h, JPEG_PREC);

    __m128i r = _mm_packs_epi32(r_l, r_h);
    __m128i g = _mm_packs_epi32(g_l, g_h);
    __m128i b = _mm_packs_epi32(b_l, b_h);

    r = _mm_packus_epi16(r, r);
    g = _mm_packus_epi16(g, g);
    b = _mm_packus_epi16(b, b);
    __m128i a = _mm_cmpeq_epi8(r, r);

    __m128i ra = _mm_unpacklo_epi8(r, a);
    __m128i bg = _mm_unpacklo_epi8(b, g);

    __m128i bgra0 = _mm_unpacklo_epi16(bg, ra);
    __m128i bgra1 = _mm_unpackhi_epi16(bg, ra);

    _mm_storeu_si128(reinterpret_cast<__m128i *>(dest +  0), bgra0);
    _mm_storeu_si128(reinterpret_cast<__m128i *>(dest + 16), bgra1);
}

static inline
void convert_ycbcr_rgba_8x1_sse2(u8* dest, __m128i y, __m128i cb, __m128i cr, __m128i s0, __m128i s1, __m128i s2, __m128i rounding)
{
    __m128i zero = _mm_setzero_si128();

    __m128i r_l = _mm_madd_epi16(_mm_unpacklo_epi16(y, cr), s0);
    __m128i r_h = _mm_madd_epi16(_mm_unpackhi_epi16(y, cr), s0);

    __m128i b_l = _mm_madd_epi16(_mm_unpacklo_epi16(y, cb), s1);
    __m128i b_h = _mm_madd_epi16(_mm_unpackhi_epi16(y, cb), s1);

    __m128i g_l = _mm_madd_epi16(_mm_unpacklo_epi16(cb, cr), s2);
    __m128i g_h = _mm_madd_epi16(_mm_unpackhi_epi16(cb, cr), s2);

    g_l = _mm_add_epi32(g_l, _mm_slli_epi32(_mm_unpacklo_epi16(y, zero), JPEG_PREC));
    g_h = _mm_add_epi32(g_h, _mm_slli_epi32(_mm_unpackhi_epi16(y, zero), JPEG_PREC));

    r_l = _mm_add_epi32(r_l, rounding);
    r_h = _mm_add_epi32(r_h, rounding);

    b_l = _mm_add_epi32(b_l, rounding);
    b_h = _mm_add_epi32(b_h, rounding);

    g_l = _mm_add_epi32(g_l, rounding);
    g_h = _mm_add_epi32(g_h, rounding);

    r_l = _mm_srai_epi32(r_l, JPEG_PREC);
    r_h = _mm_srai_epi32(r_h, JPEG_PREC);

    b_l = _mm_srai_epi32(b_l, JPEG_PREC);
    b_h = _mm_srai_epi32(b_h, JPEG_PREC);

    g_l = _mm_srai_epi32(g_l, JPEG_PREC);
    g_h = _mm_srai_epi32(g_h, JPEG_PREC);

    __m128i r = _mm_packs_epi32(r_l, r_h);
    __m128i g = _mm_packs_epi32(g_l, g_h);
    __m128i b = _mm_packs_epi32(b_l, b_h);

    r = _mm_packus_epi16(r, r);
    g = _mm_packus_epi16(g, g);
    b = _mm_packus_epi16(b, b);
    __m128i a = _mm_cmpeq_epi8(r, r);

    __m128i ba = _mm_unpacklo_epi8(b, a);
    __m128i rg = _mm_unpacklo_epi8(r, g);

    __m128i rgba0 = _mm_unpacklo_epi16(rg, ba);
    __m128i rgba1 = _mm_unpackhi_epi16(rg, ba);

    _mm_storeu_si128(reinterpret_cast<__m128i *>(dest +  0), rgba0);
    _mm_storeu_si128(reinterpret_cast<__m128i *>(dest + 16), rgba1);
}

namespace process::sse
{
    typedef void INNERLOOP_YCBCR_FUNC(u8*, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i, __m128i);
    void FUNCTION_YCBCR_8x8(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height, const int XSTEP, INNERLOOP_YCBCR_FUNC * INNERLOOP_YCBCR)
    {
        u8 result[64 * 3];

        state->idct(result +   0, data +   0, state->block[0].qt); // Y
        state->idct(result +  64, data +  64, state->block[1].qt); // Cb
        state->idct(result + 128, data + 128, state->block[2].qt); // Cr

        // color conversion
        const __m128i s0 = JPEG_CONST_SSE2(JPEG_FIXED( 1.00000), JPEG_FIXED( 1.40200));
        const __m128i s1 = JPEG_CONST_SSE2(JPEG_FIXED( 1.00000), JPEG_FIXED( 1.77200));
        const __m128i s2 = JPEG_CONST_SSE2(JPEG_FIXED(-0.34414), JPEG_FIXED(-0.71414));
        const __m128i rounding = _mm_set1_epi32(1 << (JPEG_PREC - 1));
        const __m128i tosigned = _mm_set1_epi16(-128);

        for (int y = 0; y < 4; ++y)
        {
            __m128i yy = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 16 + 0));
            __m128i cb = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 16 + 64));
            __m128i cr = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 16 + 128));

            __m128i zero = _mm_setzero_si128();

            __m128i cb0 = _mm_unpacklo_epi8(cb, zero);
            __m128i cr0 = _mm_unpacklo_epi8(cr, zero);
            __m128i cb1 = _mm_unpackhi_epi8(cb, zero);
            __m128i cr1 = _mm_unpackhi_epi8(cr, zero);

            cb0 = _mm_add_epi16(cb0, tosigned);
            cr0 = _mm_add_epi16(cr0, tosigned);
            cb1 = _mm_add_epi16(cb1, tosigned);
            cr1 = _mm_add_epi16(cr1, tosigned);

            INNERLOOP_YCBCR(dest, _mm_unpacklo_epi8(yy, zero), cb0, cr0, s0, s1, s2, rounding);
            dest += stride;

            INNERLOOP_YCBCR(dest, _mm_unpackhi_epi8(yy, zero), cb1, cr1, s0, s1, s2, rounding);
            dest += stride;
        }

        MANGO_UNREFERENCED(width);
        MANGO_UNREFERENCED(height);
    }
    void FUNCTION_YCBCR_8x16(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height, const int XSTEP, INNERLOOP_YCBCR_FUNC * INNERLOOP_YCBCR)
    {
        u8 result[64 * 4];

        state->idct(result +   0, data +   0, state->block[0].qt); // Y0
        state->idct(result +  64, data +  64, state->block[1].qt); // Y1
        state->idct(result + 128, data + 128, state->block[2].qt); // Cb
        state->idct(result + 192, data + 192, state->block[3].qt); // Cr

        // color conversion
        const __m128i s0 = JPEG_CONST_SSE2(JPEG_FIXED( 1.00000), JPEG_FIXED( 1.40200));
        const __m128i s1 = JPEG_CONST_SSE2(JPEG_FIXED( 1.00000), JPEG_FIXED( 1.77200));
        const __m128i s2 = JPEG_CONST_SSE2(JPEG_FIXED(-0.34414), JPEG_FIXED(-0.71414));
        const __m128i rounding = _mm_set1_epi32(1 << (JPEG_PREC - 1));
        const __m128i tosigned = _mm_set1_epi16(-128);

        for (int y = 0; y < 4; ++y)
        {
            __m128i y0 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 32 + 0));
            __m128i y1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 32 + 16));
            __m128i cb = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 16 + 128));
            __m128i cr = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 16 + 192));

            __m128i zero = _mm_setzero_si128();

            __m128i cb0 = _mm_unpacklo_epi8(cb, zero);
            __m128i cr0 = _mm_unpacklo_epi8(cr, zero);
            __m128i cb1 = _mm_unpackhi_epi8(cb, zero);
            __m128i cr1 = _mm_unpackhi_epi8(cr, zero);

            cb0 = _mm_add_epi16(cb0, tosigned);
            cr0 = _mm_add_epi16(cr0, tosigned);
            cb1 = _mm_add_epi16(cb1, tosigned);
            cr1 = _mm_add_epi16(cr1, tosigned);

            INNERLOOP_YCBCR(dest, _mm_unpacklo_epi8(y0, zero), cb0, cr0, s0, s1, s2, rounding);
            dest += stride;

            INNERLOOP_YCBCR(dest, _mm_unpackhi_epi8(y0, zero), cb0, cr0, s0, s1, s2, rounding);
            dest += stride;

            INNERLOOP_YCBCR(dest, _mm_unpacklo_epi8(y1, zero), cb1, cr1, s0, s1, s2, rounding);
            dest += stride;

            INNERLOOP_YCBCR(dest, _mm_unpackhi_epi8(y1, zero), cb1, cr1, s0, s1, s2, rounding);
            dest += stride;
        }

        MANGO_UNREFERENCED(width);
        MANGO_UNREFERENCED(height);
    }
    void FUNCTION_YCBCR_16x8(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height, const int XSTEP, INNERLOOP_YCBCR_FUNC * INNERLOOP_YCBCR)
    {
        u8 result[64 * 4];

        state->idct(result +   0, data +   0, state->block[0].qt); // Y0
        state->idct(result +  64, data +  64, state->block[1].qt); // Y1
        state->idct(result + 128, data + 128, state->block[2].qt); // Cb
        state->idct(result + 192, data + 192, state->block[3].qt); // Cr

        // color conversion
        const __m128i s0 = JPEG_CONST_SSE2(JPEG_FIXED( 1.00000), JPEG_FIXED( 1.40200));
        const __m128i s1 = JPEG_CONST_SSE2(JPEG_FIXED( 1.00000), JPEG_FIXED( 1.77200));
        const __m128i s2 = JPEG_CONST_SSE2(JPEG_FIXED(-0.34414), JPEG_FIXED(-0.71414));
        const __m128i rounding = _mm_set1_epi32(1 << (JPEG_PREC - 1));
        const __m128i tosigned = _mm_set1_epi16(-128);

        for (int y = 0; y < 4; ++y)
        {
            __m128i y0 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 16 + 0));
            __m128i y1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 16 + 64));
            __m128i cb = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 16 + 128));
            __m128i cr = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 16 + 192));

            __m128i zero = _mm_setzero_si128();
            __m128i cb0;
            __m128i cb1;
            __m128i cr0;
            __m128i cr1;

            cb0 = _mm_unpacklo_epi8(cb, cb);
            cr0 = _mm_unpacklo_epi8(cr, cr);

            cb1 = _mm_unpackhi_epi8(cb0, zero);
            cr1 = _mm_unpackhi_epi8(cr0, zero);
            cb0 = _mm_unpacklo_epi8(cb0, zero);
            cr0 = _mm_unpacklo_epi8(cr0, zero);

            cb0 = _mm_add_epi16(cb0, tosigned);
            cr0 = _mm_add_epi16(cr0, tosigned);
            cb1 = _mm_add_epi16(cb1, tosigned);
            cr1 = _mm_add_epi16(cr1, tosigned);

            INNERLOOP_YCBCR(dest + 0 * XSTEP, _mm_unpacklo_epi8(y0, zero), cb0, cr0, s0, s1, s2, rounding);
            INNERLOOP_YCBCR(dest + 1 * XSTEP, _mm_unpacklo_epi8(y1, zero), cb1, cr1, s0, s1, s2, rounding);
            dest += stride;

            cb0 = _mm_unpackhi_epi8(cb, cb);
            cr0 = _mm_unpackhi_epi8(cr, cr);

            cb1 = _mm_unpackhi_epi8(cb0, zero);
            cr1 = _mm_unpackhi_epi8(cr0, zero);
            cb0 = _mm_unpacklo_epi8(cb0, zero);
            cr0 = _mm_unpacklo_epi8(cr0, zero);

            cb0 = _mm_add_epi16(cb0, tosigned);
            cr0 = _mm_add_epi16(cr0, tosigned);
            cb1 = _mm_add_epi16(cb1, tosigned);
            cr1 = _mm_add_epi16(cr1, tosigned);

            INNERLOOP_YCBCR(dest + 0 * XSTEP, _mm_unpackhi_epi8(y0, zero), cb0, cr0, s0, s1, s2, rounding);
            INNERLOOP_YCBCR(dest + 1 * XSTEP, _mm_unpackhi_epi8(y1, zero), cb1, cr1, s0, s1, s2, rounding);
            dest += stride;
        }

        MANGO_UNREFERENCED(width);
        MANGO_UNREFERENCED(height);
    }
    void FUNCTION_YCBCR_16x16(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height, const int XSTEP, INNERLOOP_YCBCR_FUNC * INNERLOOP_YCBCR)
    {
        u8 result[64 * 6];

        state->idct(result +   0, data +   0, state->block[0].qt); // Y0
        state->idct(result + 128, data +  64, state->block[1].qt); // Y1
        state->idct(result +  64, data + 128, state->block[2].qt); // Y2
        state->idct(result + 192, data + 192, state->block[3].qt); // Y3
        state->idct(result + 256, data + 256, state->block[4].qt); // Cb
        state->idct(result + 320, data + 320, state->block[5].qt); // Cr

        // color conversion
        const __m128i s0 = JPEG_CONST_SSE2(JPEG_FIXED( 1.00000), JPEG_FIXED( 1.40200));
        const __m128i s1 = JPEG_CONST_SSE2(JPEG_FIXED( 1.00000), JPEG_FIXED( 1.77200));
        const __m128i s2 = JPEG_CONST_SSE2(JPEG_FIXED(-0.34414), JPEG_FIXED(-0.71414));
        const __m128i rounding = _mm_set1_epi32(1 << (JPEG_PREC - 1));
        const __m128i tosigned = _mm_set1_epi16(-128);

        for (int y = 0; y < 4; ++y)
        {
            __m128i y0 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 32 + 0));
            __m128i y1 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 32 + 128));
            __m128i y2 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 32 + 16));
            __m128i y3 = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 32 + 144));
            __m128i cb = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 16 + 256));
            __m128i cr = _mm_loadu_si128(reinterpret_cast<const __m128i *>(result + y * 16 + 320));

            __m128i zero = _mm_setzero_si128();
            __m128i cb0;
            __m128i cb1;
            __m128i cr0;
            __m128i cr1;

            cb0 = _mm_unpacklo_epi8(cb, cb);
            cr0 = _mm_unpacklo_epi8(cr, cr);

            cb1 = _mm_unpackhi_epi8(cb0, zero);
            cr1 = _mm_unpackhi_epi8(cr0, zero);
            cb0 = _mm_unpacklo_epi8(cb0, zero);
            cr0 = _mm_unpacklo_epi8(cr0, zero);

            cb0 = _mm_add_epi16(cb0, tosigned);
            cr0 = _mm_add_epi16(cr0, tosigned);
            cb1 = _mm_add_epi16(cb1, tosigned);
            cr1 = _mm_add_epi16(cr1, tosigned);

            INNERLOOP_YCBCR(dest + 0 * XSTEP, _mm_unpacklo_epi8(y0, zero), cb0, cr0, s0, s1, s2, rounding);
            INNERLOOP_YCBCR(dest + 1 * XSTEP, _mm_unpacklo_epi8(y1, zero), cb1, cr1, s0, s1, s2, rounding);
            dest += stride;

            INNERLOOP_YCBCR(dest + 0 * XSTEP, _mm_unpackhi_epi8(y0, zero), cb0, cr0, s0, s1, s2, rounding);
            INNERLOOP_YCBCR(dest + 1 * XSTEP, _mm_unpackhi_epi8(y1, zero), cb1, cr1, s0, s1, s2, rounding);
            dest += stride;

            cb0 = _mm_unpackhi_epi8(cb, cb);
            cr0 = _mm_unpackhi_epi8(cr, cr);

            cb1 = _mm_unpackhi_epi8(cb0, zero);
            cr1 = _mm_unpackhi_epi8(cr0, zero);
            cb0 = _mm_unpacklo_epi8(cb0, zero);
            cr0 = _mm_unpacklo_epi8(cr0, zero);

            cb0 = _mm_add_epi16(cb0, tosigned);
            cr0 = _mm_add_epi16(cr0, tosigned);
            cb1 = _mm_add_epi16(cb1, tosigned);
            cr1 = _mm_add_epi16(cr1, tosigned);

            INNERLOOP_YCBCR(dest + 0 * XSTEP, _mm_unpacklo_epi8(y2, zero), cb0, cr0, s0, s1, s2, rounding);
            INNERLOOP_YCBCR(dest + 1 * XSTEP, _mm_unpacklo_epi8(y3, zero), cb1, cr1, s0, s1, s2, rounding);
            dest += stride;

            INNERLOOP_YCBCR(dest + 0 * XSTEP, _mm_unpackhi_epi8(y2, zero), cb0, cr0, s0, s1, s2, rounding);
            INNERLOOP_YCBCR(dest + 1 * XSTEP, _mm_unpackhi_epi8(y3, zero), cb1, cr1, s0, s1, s2, rounding);
            dest += stride;
        }

        MANGO_UNREFERENCED(width);
        MANGO_UNREFERENCED(height);
    }
}

// Generate YCBCR to BGRA functions
void process_ycbcr_bgra_8x8_sse2(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_8x8(dest, stride, data, state, width, height,
        32, convert_ycbcr_bgra_8x1_sse2);
}
void process_ycbcr_bgra_8x16_sse2(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_8x16(dest, stride, data, state, width, height,
        32, convert_ycbcr_bgra_8x1_sse2);
}
void process_ycbcr_bgra_16x8_sse2(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_16x8(dest, stride, data, state, width, height,
        32, convert_ycbcr_bgra_8x1_sse2);
}
void process_ycbcr_bgra_16x16_sse2(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_16x16(dest, stride, data, state, width, height,
        32, convert_ycbcr_bgra_8x1_sse2);
}

// Generate YCBCR to RGBA functions
void process_ycbcr_rgba_8x8_sse2(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_8x8(dest, stride, data, state, width, height,
        32, convert_ycbcr_rgba_8x1_sse2);
}
void process_ycbcr_rgba_8x16_sse2(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_8x16(dest, stride, data, state, width, height,
        32, convert_ycbcr_rgba_8x1_sse2);
}
void process_ycbcr_rgba_16x8_sse2(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_16x8(dest, stride, data, state, width, height,
        32, convert_ycbcr_rgba_8x1_sse2);
}
void process_ycbcr_rgba_16x16_sse2(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_16x16(dest, stride, data, state, width, height,
        32, convert_ycbcr_rgba_8x1_sse2);
}

#endif // MANGO_ENABLE_SSE2

#if defined(MANGO_ENABLE_SSE4_1)

static inline
void convert_ycbcr_bgr_8x1_ssse3(u8* dest, __m128i y, __m128i cb, __m128i cr, __m128i s0, __m128i s1, __m128i s2, __m128i rounding)
{
    __m128i zero = _mm_setzero_si128();

    __m128i r_l = _mm_madd_epi16(_mm_unpacklo_epi16(y, cr), s0);
    __m128i r_h = _mm_madd_epi16(_mm_unpackhi_epi16(y, cr), s0);

    __m128i b_l = _mm_madd_epi16(_mm_unpacklo_epi16(y, cb), s1);
    __m128i b_h = _mm_madd_epi16(_mm_unpackhi_epi16(y, cb), s1);

    __m128i g_l = _mm_madd_epi16(_mm_unpacklo_epi16(cb, cr), s2);
    __m128i g_h = _mm_madd_epi16(_mm_unpackhi_epi16(cb, cr), s2);

    g_l = _mm_add_epi32(g_l, _mm_slli_epi32(_mm_unpacklo_epi16(y, zero), JPEG_PREC));
    g_h = _mm_add_epi32(g_h, _mm_slli_epi32(_mm_unpackhi_epi16(y, zero), JPEG_PREC));

    r_l = _mm_add_epi32(r_l, rounding);
    r_h = _mm_add_epi32(r_h, rounding);

    b_l = _mm_add_epi32(b_l, rounding);
    b_h = _mm_add_epi32(b_h, rounding);

    g_l = _mm_add_epi32(g_l, rounding);
    g_h = _mm_add_epi32(g_h, rounding);

    r_l = _mm_srai_epi32(r_l, JPEG_PREC);
    r_h = _mm_srai_epi32(r_h, JPEG_PREC);

    b_l = _mm_srai_epi32(b_l, JPEG_PREC);
    b_h = _mm_srai_epi32(b_h, JPEG_PREC);

    g_l = _mm_srai_epi32(g_l, JPEG_PREC);
    g_h = _mm_srai_epi32(g_h, JPEG_PREC);

    __m128i r = _mm_packs_epi32(r_l, r_h);
    __m128i g = _mm_packs_epi32(g_l, g_h);
    __m128i b = _mm_packs_epi32(b_l, b_h);

    r = _mm_packus_epi16(r, r);
    g = _mm_packus_epi16(g, g);
    b = _mm_packus_epi16(b, b);

    __m128i bg = _mm_unpacklo_epi64(b, g);

    constexpr u8 n = 0x80;

    __m128i bg0 = _mm_shuffle_epi8(bg, _mm_setr_epi8(0, 8, n, 1, 9, n, 2, 10, n, 3, 11, n, 4, 12, n, 5));
    __m128i bg1 = _mm_shuffle_epi8(bg, _mm_setr_epi8(13, n, 6, 14, n, 7, 15, n, n, n, n, n, n, n, n, n));
    __m128i r0 = _mm_shuffle_epi8(r, _mm_setr_epi8(n, n, 0, n, n, 1, n, n, 2, n, n, 3, n, n, 4, n));
    __m128i r1 = _mm_shuffle_epi8(r, _mm_setr_epi8(n, 5, n, n, 6, n, n, 7, n, n, n, n, n, n, n, n));
    __m128i bgr0 = _mm_or_si128(bg0, r0);
    __m128i bgr1 = _mm_or_si128(bg1, r1);

    _mm_storeu_si128(reinterpret_cast<__m128i *>(dest +  0), bgr0);
    _mm_storel_epi64(reinterpret_cast<__m128i *>(dest + 16), bgr1);
}

static inline
void convert_ycbcr_rgb_8x1_ssse3(u8* dest, __m128i y, __m128i cb, __m128i cr, __m128i s0, __m128i s1, __m128i s2, __m128i rounding)
{
    __m128i zero = _mm_setzero_si128();

    __m128i r_l = _mm_madd_epi16(_mm_unpacklo_epi16(y, cr), s0);
    __m128i r_h = _mm_madd_epi16(_mm_unpackhi_epi16(y, cr), s0);

    __m128i b_l = _mm_madd_epi16(_mm_unpacklo_epi16(y, cb), s1);
    __m128i b_h = _mm_madd_epi16(_mm_unpackhi_epi16(y, cb), s1);

    __m128i g_l = _mm_madd_epi16(_mm_unpacklo_epi16(cb, cr), s2);
    __m128i g_h = _mm_madd_epi16(_mm_unpackhi_epi16(cb, cr), s2);

    g_l = _mm_add_epi32(g_l, _mm_slli_epi32(_mm_unpacklo_epi16(y, zero), JPEG_PREC));
    g_h = _mm_add_epi32(g_h, _mm_slli_epi32(_mm_unpackhi_epi16(y, zero), JPEG_PREC));

    r_l = _mm_add_epi32(r_l, rounding);
    r_h = _mm_add_epi32(r_h, rounding);

    b_l = _mm_add_epi32(b_l, rounding);
    b_h = _mm_add_epi32(b_h, rounding);

    g_l = _mm_add_epi32(g_l, rounding);
    g_h = _mm_add_epi32(g_h, rounding);

    r_l = _mm_srai_epi32(r_l, JPEG_PREC);
    r_h = _mm_srai_epi32(r_h, JPEG_PREC);

    b_l = _mm_srai_epi32(b_l, JPEG_PREC);
    b_h = _mm_srai_epi32(b_h, JPEG_PREC);

    g_l = _mm_srai_epi32(g_l, JPEG_PREC);
    g_h = _mm_srai_epi32(g_h, JPEG_PREC);

    __m128i r = _mm_packs_epi32(r_l, r_h);
    __m128i g = _mm_packs_epi32(g_l, g_h);
    __m128i b = _mm_packs_epi32(b_l, b_h);

    r = _mm_packus_epi16(r, r);
    g = _mm_packus_epi16(g, g);
    b = _mm_packus_epi16(b, b);

    __m128i rg = _mm_unpacklo_epi64(r, g);

    constexpr u8 n = 0x80;

    __m128i rg0 = _mm_shuffle_epi8(rg, _mm_setr_epi8(0, 8, n, 1, 9, n, 2, 10, n, 3, 11, n, 4, 12, n, 5));
    __m128i rg1 = _mm_shuffle_epi8(rg, _mm_setr_epi8(13, n, 6, 14, n, 7, 15, n, n, n, n, n, n, n, n, n));
    __m128i b0 = _mm_shuffle_epi8(b, _mm_setr_epi8(n, n, 0, n, n, 1, n, n, 2, n, n, 3, n, n, 4, n));
    __m128i b1 = _mm_shuffle_epi8(b, _mm_setr_epi8(n, 5, n, n, 6, n, n, 7, n, n, n, n, n, n, n, n));
    __m128i rgb0 = _mm_or_si128(rg0, b0);
    __m128i rgb1 = _mm_or_si128(rg1, b1);

    _mm_storeu_si128(reinterpret_cast<__m128i *>(dest +  0), rgb0);
    _mm_storel_epi64(reinterpret_cast<__m128i *>(dest + 16), rgb1);
}

// Generate YCBCR to BGR functions
void process_ycbcr_bgr_8x8_ssse3(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_8x8(dest, stride, data, state, width, height,
        24, convert_ycbcr_bgr_8x1_ssse3);
}
void process_ycbcr_bgr_8x16_ssse3(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_8x16(dest, stride, data, state, width, height,
        24, convert_ycbcr_bgr_8x1_ssse3);
}
void process_ycbcr_bgr_16x8_ssse3(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_16x8(dest, stride, data, state, width, height,
        24, convert_ycbcr_bgr_8x1_ssse3);
}
void process_ycbcr_bgr_16x16_ssse3(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_16x16(dest, stride, data, state, width, height,
        24, convert_ycbcr_bgr_8x1_ssse3);
}

// Generate YCBCR to RGB functions
void process_ycbcr_rgb_8x8_ssse3(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_8x8(dest, stride, data, state, width, height,
        24, convert_ycbcr_rgb_8x1_ssse3);
}
void process_ycbcr_rgb_8x16_ssse3(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_8x16(dest, stride, data, state, width, height,
        24, convert_ycbcr_rgb_8x1_ssse3);
}
void process_ycbcr_rgb_16x8_ssse3(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_16x8(dest, stride, data, state, width, height,
        24, convert_ycbcr_rgb_8x1_ssse3);
}
void process_ycbcr_rgb_16x16_ssse3(u8* dest, size_t stride, const s16* data, ProcessState* state, int width, int height) {
    return process::sse::FUNCTION_YCBCR_16x16(dest, stride, data, state, width, height,
        24, convert_ycbcr_rgb_8x1_ssse3);
}
#endif // MANGO_ENABLE_SSE4_1

} // namespace mango::jpeg

