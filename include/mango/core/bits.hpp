/*
    MANGO Multimedia Development Platform
    Copyright (C) 2012-2021 Twilight Finland 3D Oy Ltd. All rights reserved.
*/
#pragma once

#include <cmath>
#include <cassert>
#include <algorithm>
#include <mango/core/configure.hpp>
#include <mango/core/half.hpp>

namespace mango
{
#if defined(MANGO_COMPILER_GCC) || defined(MANGO_COMPILER_CLANG) || defined(MANGO_COMPILER_INTEL)
    #define MANGO_BITS_GCC_BUILTINS
#endif

#if defined(MANGO_COMPILER_MICROSOFT)
    static inline u16 byteswap(u16 v) { return _byteswap_ushort(v); }
    static inline u32 byteswap(u32 v) { return _byteswap_ulong(v); }
    static inline u64 byteswap(u64 v) { return _byteswap_uint64(v); }
#elif defined(MANGO_COMPILER_GCC) || defined(MANGO_COMPILER_CLANG) || defined(MANGO_COMPILER_INTEL)
    // GCC / CLANG intrinsics
    static inline u16 byteswap(u16 v) { return __builtin_bswap32(v << 16); }
    static inline u32 byteswap(u32 v) { return __builtin_bswap32(v); }
    static inline u64 byteswap(u64 v) { return __builtin_bswap64(v); }
#else
    // Generic implementation
    // These idioms are often recognized by compilers and result in bswap instruction being generated
    // but cannot be guaranteed so compiler intrinsics above are preferred when available.
    static inline u16 byteswap(u16 v) { return u16((v << 8) | (v >> 8)); }
    static inline u32 byteswap(u32 v) { return (v >> 24) | ((v >> 8) & 0x0000ff00) | ((v << 8) & 0x00ff0000) | (v << 24); }
    static inline u64 byteswap(u64 v)
    {
        v = (v >> 32) | (v << 32);
        v = ((v & 0xffff0000ffff0000ull) >> 16) | ((v << 16) & 0xffff0000ffff0000ull);
        v = ((v & 0xff00ff00ff00ff00ull) >>  8) | ((v <<  8) & 0xff00ff00ff00ff00ull);
        return v;
    }
#endif

    constexpr u8 u8_mask(u8 c0, u8 c1, u8 c2, u8 c3) noexcept { return (c3 << 6) | (c2 << 4) | (c1 << 2) | c0; }
    constexpr u16 u16_mask(char c0, char c1) noexcept { return (c1 << 8) | c0; }
    constexpr u32 u32_mask(char c0, char c1, char c2, char c3) noexcept { return (c3 << 24) | (c2 << 16) | (c1 << 8) | c0; }

#if defined(MANGO_COMPILER_MICROSOFT) || defined(MANGO_COMPILER_CLANG) || defined(MANGO_COMPILER_INTEL)
    static inline u32 byteclamp(s32 value) { return std::max(0, std::min(255, value)); }
#elif defined(MANGO_COMPILER_GCC)
    static inline u32 byteclamp(s32 value) { return std::clamp(value, 0, 255); }
#else
    static inline u32 byteclamp(s32 value) { return u32(value & 0xffffff00 ? (((~value) >> 31) & 0xff) : value); }
#endif

    // ----------------------------------------------------------------------------
    // 32 bits
    // ----------------------------------------------------------------------------

#ifdef MANGO_ENABLE_BMI

    static inline u32 u32_extract_lsb(u32 value)
    {
        // value:  xxxxxx100000
        // result: 000000100000
        return _blsi_u32(value);
    }

    static inline u32 u32_clear_lsb(u32 value)
    {
        // value:  xxxxxx100000
        // result: xxxxxx000000
        return _blsr_u32(value);
    }

#else

    constexpr u32 u32_extract_lsb(u32 value)
    {
        // value:  xxxxxx100000
        // result: 000000100000
        return value & (0 - value);
    }

    constexpr u32 u32_clear_lsb(u32 value)
    {
        // value:  xxxxxx100000
        // result: xxxxxx000000
        return value & (value - 1);
    }

#endif

#ifdef MANGO_ENABLE_BMI

    static inline int u32_tzcnt(u32 value)
    {
        // NOTE: value 0 is undefined
        return _tzcnt_u32(value);
    }

#elif defined(__aarch64__)

    static inline int u32_tzcnt(u32 value)
    {
        // NOTE: value 0 is undefined
        value = __rbit(value);
        return __clz(value);
    }

#elif defined(MANGO_BITS_GCC_BUILTINS)

    static inline int u32_tzcnt(u32 value)
    {
        // NOTE: value 0 is undefined
        return __builtin_ctz(value);
    }

#else

    static inline int u32_tzcnt(u32 value)
    {
        // NOTE: value 0 is undefined
        const u32 lsb = u32_extract_lsb(value);
        static const u8 table [] =
        {
            0, 1, 28, 2, 29, 14, 24, 3, 30, 22, 20, 15, 25, 17, 4, 8,
            31, 27, 13, 23, 21, 19, 16, 7, 26, 12, 18, 6, 11, 5, 10, 9
        };
        return table[(lsb * 0x077cb531) >> 27];
    }

#endif

    static inline int u32_index_of_lsb(u32 value)
    {
        // NOTE: value 0 is undefined
        return u32_tzcnt(value);
    }

#ifdef MANGO_ENABLE_LZCNT

    static inline u32 u32_mask_msb(u32 value)
    {
        // value:  0001xxxxxxxx
        // result: 000111111111
        u32 mask = 1u << (31 - _lzcnt_u32(value));
        return mask | (mask - 1);
    }

    static inline int u32_index_of_msb(u32 value)
    {
        // NOTE: value 0 is undefined
        return int(31 - _lzcnt_u32(value));
    }

#elif defined(__ARM_FEATURE_CLZ)

    static inline u32 u32_mask_msb(u32 value)
    {
        // value:  0001xxxxxxxx
        // result: 000111111111
        u32 mask = 1u << (31 - __clz(value));
        return mask | (mask - 1);
    }

    static inline int u32_index_of_msb(u32 value)
    {
        // NOTE: value 0 is undefined
        return int(31 - __clz(value));
    }


#elif defined(MANGO_BITS_GCC_BUILTINS)

    static inline u32 u32_mask_msb(u32 value)
    {
        // value:  0001xxxxxxxx
        // result: 000111111111
        u32 mask = 1u << (31 - __builtin_clz(value));
        return mask | (mask - 1);
    }

    static inline int u32_index_of_msb(u32 value)
    {
        // NOTE: value 0 is undefined
        return int(31 - __builtin_clz(value));
    }

#else

    static inline u32 u32_mask_msb(u32 value)
    {
        // value:  0001xxxxxxxx
        // result: 000111111111
        value |= value >> 1;
        value |= value >> 2;
        value |= value >> 4;
        value |= value >> 8;
        value |= value >> 16;
        return value;
    }

    static inline int u32_index_of_msb(u32 value)
    {
        // NOTE: value 0 is undefined
        /* NOTE: This generates branchless code
        int base = 0;
        u32 temp;
        temp = value & 0xffff0000; if (temp) { base |= 16; value = temp; }
        temp = value & 0xff00ff00; if (temp) { base |= 8;  value = temp; }
        temp = value & 0xf0f0f0f0; if (temp) { base |= 4;  value = temp; }
        temp = value & 0xcccccccc; if (temp) { base |= 2;  value = temp; }
        temp = value & 0xaaaaaaaa; if (temp) { base |= 1; }
        return base;
        */
        const u32 mask = u32_mask_msb(value);
        static const u8 table [] =
        {
            0, 9, 1, 10, 13, 21, 2, 29, 11, 14, 16, 18, 22, 25, 3, 30,
            8, 12, 20, 28, 15, 17, 24, 7, 19, 27, 23, 6, 26, 5, 4, 31
        };
        return table[(mask * 0x07c4acdd) >> 27];
    }


#endif

    static inline int u32_log2(u32 value)
    {
        // NOTE: value 0 is undefined
        return u32_index_of_msb(value);
    }

#if defined(MANGO_ENABLE_POPCNT)
    static inline int u32_count_bits(u32 value) { return _mm_popcnt_u32(value); }
#elif defined(MANGO_BITS_GCC_BUILTINS)
    static inline int u32_count_bits(u32 value) { return __builtin_popcountl(value); }
#elif defined(__aarch64__)
    static inline int u32_count_bits(u32 value)
    {
        uint8x8_t count8x8 = vcnt_u8(vcreate_u8(u64(value)));
        return int(vaddlv_u8(count8x8));
    }
#else
    static inline int u32_count_bits(u32 value)
    {
        value -= (value >> 1) & 0x55555555;
        value = (value & 0x33333333) + ((value >> 2) & 0x33333333);
        value = (value + (value >> 4)) & 0x0f0f0f0f;
        return int((value * 0x01010101) >> 24);
    }
#endif

#ifdef MANGO_ENABLE_BMI
    static inline u32 u32_extract_bits(u32 value, int offset, int size) { return _bextr_u32(value, offset, size); }
#else
    constexpr u32 u32_extract_bits(u32 value, int offset, int size) { return (value >> offset) & ((1 << size) - 1); }
#endif
    static inline bool u32_is_power_of_two(u32 value) { return u32_clear_lsb(value) == 0; }
    constexpr u32 u32_clamp(u32 value, u32 low, u32 high) { return (value < low) ? low : (high < value) ? high : value; }

    // ----------------------------------------------------------------------------
    // 64 bits
    // ----------------------------------------------------------------------------

#ifdef MANGO_ENABLE_BMI
    static inline int u64_tzcnt(u64 value)
    {
        // NOTE: value 0 is undefined
        return int(_tzcnt_u64(value));
    }
#elif defined(__aarch64__)
    static inline int u64_tzcnt(u64 value)
    {
        // NOTE: value 0 is undefined
        value = __rbitll(value);
        return __clzll(value);
    }
#elif defined(MANGO_BITS_GCC_BUILTINS)
    static inline int u64_tzcnt(u64 value)
    {
        // NOTE: value 0 is undefined
        return __builtin_ctzll(value);
    }
#else
    static inline int u64_tzcnt(u64 value)
    {
        // NOTE: value 0 is undefined
        const u64 lsb = value & (0 - value);
        static const u8 table [] =
        {
            0, 1, 2, 53, 3, 7, 54, 27, 4, 38, 41, 8, 34, 55, 48, 28,
            62, 5, 39, 46, 44, 42, 22, 9, 24, 35, 59, 56, 49, 18, 29, 11,
            63, 52, 6, 26, 37, 40, 33, 47, 61, 45, 43, 21, 23, 58, 17, 10,
            51, 25, 36, 32, 60, 20, 57, 16, 50, 31, 19, 15, 30, 14, 13, 12,
        };
        return table[(lsb * 0x022fdd63cc95386du) >> 58];
    }
#endif

#ifdef MANGO_ENABLE_BMI
    static inline u64 u64_extract_bits(u64 value, int offset, int size) { return _bextr_u64(value, offset, size); }
#else
    constexpr u64 u64_extract_bits(u64 value, int offset, int size) { return (value >> offset) & ((1ull << size) - 1); }
#endif


} // namespace mango
