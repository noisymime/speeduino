#pragma once

/** @file
 * @brief Optimized multi-byte bit shifting *for AVR-GCC only*. See @ref group-opt-shift
*/

#include <stdint.h>
#include "globals.h" // Required for CPU/architecture preprocessor symbols

/// @defgroup group-opt-shift Optimised bitwise shifts
///
/// @brief As of AVR-GCC 13.2.0, the code produced for 32-bit shifts with a compile time shift distance is very poor.
/// The templates here implement optimised shifting. Average 35% increase in shift performance on AtMega2560: see unit tests.
///
/// Usage:
/// @code
///      rpmDelta = lshift<10>(toothDeltaV) / (6 * toothDeltaT);
/// @endcode
///
/// If there is no overload for a certain shift, that's because GCC produced decent ASM
/// in that case.
/// 
/// @note Code is usable on all architectures, but the optimization only applies to AVR-GCC.
/// Other compilers will see a standard bitwise shift.
/// @{
    
// Flag if we should turn on optimized shifts
#if !defined(USE_OPTIMIZED_SHIFTS)
#if (defined(CORE_AVR) || defined(ARDUINO_ARCH_AVR)) && defined(__GNUC__)
#define USE_OPTIMIZED_SHIFTS 1
#else
#define USE_OPTIMIZED_SHIFTS 0
#endif
#endif

/**
 * @brief Bitwise left shift - generic, unoptimized, case 
 * 
 * @tparam b number of bits to shift by
 * @param a value to shift
 * @return uint32_t a<<b
 */
template <uint8_t b> 
static inline uint32_t lshift(uint32_t a) { 
#if USE_OPTIMIZED_SHIFTS==1
    // The shifts below have been validated to produce performant code in GCC. 
    // Other shift amounts are either in a specialized template below (good) or are unvalidated (bad).
    static_assert(b==1 || b==2 || b==3 || b==8 || b==16 || b==24,
                  "Unvalidated shift - confirm gcc produces performant code");
#endif
    return a << b; 
}

#if USE_OPTIMIZED_SHIFTS==1

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

/// @{
/// @brief uint32_t bitwise left shift optimised for the specified shift distance  
/// 
/// @note The assembler was generated using clang 17.0.1 cross compiling: -O3 --target=avr -mmcu=atmega2560. See https://godbolt.org/z/71cPnMYqs
/// 
/// @param a value to shift
/// @return uint32_t a<<b
template <>
uint32_t lshift<4U>(uint32_t a)
{
    asm(
        "swap    %D0\n"
        "andi    %D0, 240\n"
        "swap    %C0\n"
        "eor     %D0, %C0\n"
        "andi    %C0, 240\n"
        "eor     %D0, %C0\n"
        "swap    %B0\n"
        "eor     %C0, %B0\n"
        "andi    %B0, 240\n"
        "eor     %C0, %B0\n"
        "swap    %A0\n"
        "eor     %B0, %A0\n"
        "andi    %A0, 240\n"
        "eor     %B0, %A0\n"
        : "=d" (a) 
        : "0" (a) 
        :
    );

    return a;
}

template <>
uint32_t lshift<5U>(uint32_t a)
{
    asm(
        "swap    %D0\n"
        "andi    %D0, 240\n"
        "swap    %C0\n"
        "eor     %D0, %C0\n"
        "andi    %C0, 240\n"
        "eor     %D0, %C0\n"
        "swap    %B0\n"
        "eor     %C0, %B0\n"
        "andi    %B0, 240\n"
        "eor     %C0, %B0\n"
        "swap    %A0\n"
        "eor     %B0, %A0\n"
        "andi    %A0, 240\n"
        "eor     %B0, %A0\n"
        "lsl     %A0\n"
        "rol     %B0\n"
        "rol     %C0\n"
        "rol     %D0\n"
        : "=d" (a) 
        : "0" (a) 
        : 
    );

    return a;
}

template <>
uint32_t lshift<6U>(uint32_t a)
{
    asm(
        "lsr     %D0\n"
        "ror     %C0\n"
        "ror     %B0\n"
        "ror     %A0\n"
        "mov     r18, __zero_reg__\n"
        "ror     r18\n"
        "lsr     %D0\n"
        "ror     %C0\n"
        "ror     %B0\n"
        "ror     %A0\n"
        "ror     r18\n"
        "mov     %D0, %C0\n"
        "mov     %C0, %B0\n"
        "mov     r19, %A0\n"
        "movw    %A0, r18\n"
        : "=d" (a) 
        : "0" (a) 
        : "r18", "r19"
    );

    return a;
}

template <>
uint32_t lshift<7U>(uint32_t a)
{
    asm(
        "lsr     %D0\n"
        "ror     %C0\n"
        "ror     %B0\n"
        "ror     %A0\n"
        "mov     r18, __zero_reg__\n"
        "ror     r18\n"
        "mov     %D0, %C0\n"
        "mov     %C0, %B0\n"
        "mov     r19, %A0\n"
        "movw    %A0, r18\n"
        : "=d" (a) 
        : "0" (a) 
        : "r18", "r19"
    );

    return a;
}

template <>
uint32_t lshift<9U>(uint32_t a)
{
    asm(
        "lsl     %A0\n"
        "rol     %B0\n"
        "rol     %C0\n"
        "mov     %D0, %C0\n"
        "mov     %C0, %B0\n"
        "mov     %B0, %A0\n"
        "mov     %A0, __zero_reg__\n"
        : "=d" (a) 
        : "0" (a) 
        : 
    );

    return a;
}

template <>
uint32_t lshift<10U>(uint32_t a)
{
    asm(
        "lsl     %A0\n"
        "rol     %B0\n"
        "rol     %C0\n"
        "lsl     %A0\n"
        "rol     %B0\n"
        "rol     %C0\n"
        "mov     %D0, %C0\n"
        "mov     %C0, %B0\n"
        "mov     %B0, %A0\n"
        "mov     %A0, __zero_reg__\n"
        : "=d" (a) 
        : "0" (a) 
        : 
    );

    return a;
}

template <>
uint32_t lshift<11U>(uint32_t a)
{
    asm(
        "lsl     %A0\n"
        "rol     %B0\n"
        "rol     %C0\n"
        "lsl     %A0\n"
        "rol     %B0\n"
        "rol     %C0\n"
        "lsl     %A0\n"
        "rol     %B0\n"
        "rol     %C0\n"
        "mov     %D0, %C0\n"
        "mov     %C0, %B0\n"
        "mov     %B0, %A0\n"
        "mov     %A0, __zero_reg__\n"
        : "=d" (a) 
        : "0" (a) 
        : 
    );

    return a;
}

template <>
uint32_t lshift<12U>(uint32_t a)
{
    asm(
        "swap    %C0\n"
        "andi    %C0, 240\n"
        "swap    %B0\n"
        "eor     %C0, %B0\n"
        "andi    %B0, 240\n"
        "eor     %C0, %B0\n"
        "swap    %A0\n"
        "eor     %B0, %A0\n"
        "andi    %A0, 240\n"
        "eor     %B0, %A0\n"
        "mov     %D0, %C0\n"
        "mov     %C0, %B0\n"
        "mov     %B0, %A0\n"
        "mov     %A0, __zero_reg__\n"
        : "=d" (a) 
        : "0" (a) 
        : 
    );

    return a;
}

template <>
uint32_t lshift<13U>(uint32_t a)
{
    asm(
        "swap    %C0\n"
        "andi    %C0, 240\n"
        "swap    %B0\n"
        "eor     %C0, %B0\n"
        "andi    %B0, 240\n"
        "eor     %C0, %B0\n"
        "swap    %A0\n"
        "eor     %B0, %A0\n"
        "andi    %A0, 240\n"
        "eor     %B0, %A0\n"
        "lsl     %A0\n"
        "rol     %B0\n"
        "rol     %C0\n"
        "mov     %D0, %C0\n"
        "mov     %C0, %B0\n"
        "mov     %B0, %A0\n"
        "mov     %A0, __zero_reg__\n"
        : "=d" (a) 
        : "0" (a) 
        : 
    );

    return a;
}

template <>
uint32_t lshift<14U>(uint32_t a)
{
    asm(
        "movw    r18, %A0\n"
        "lsr     %C0\n"
        "ror     r19\n"
        "ror     r18\n"
        "mov     %B0, __zero_reg__\n"
        "ror     %B0\n"
        "lsr     %C0\n"
        "ror     r19\n"
        "ror     r18\n"
        "ror     %B0\n"
        "mov     %A0, __zero_reg__\n"
        "movw    %C0, r18\n"
        : "=d" (a) 
        : "0" (a) 
        : "r18", "r19"
    );

    return a;
}

template <>
uint32_t lshift<15U>(uint32_t a)
{
    asm(
        "movw    r18, %A0\n"
        "lsr     %C0\n"
        "ror     r19\n"
        "ror     r18\n"
        "mov     %B0, __zero_reg__\n"
        "ror     %B0\n"
        "mov     %A0, __zero_reg__\n"
        "movw    %C0, r18\n"
        : "=d" (a) 
        : "0" (a) 
        : "r18", "r19"
    );

    return a;
}
///@}

#pragma GCC diagnostic pop

#endif

/**
 * @brief Bitwise right shift - generic, unoptimized, case 
 * 
 * @tparam b number of bits to shift by
 * @param a value to shift
 * @return uint32_t 
 */
template <uint8_t b> 
static inline uint32_t rshift(uint32_t a) { 
#if USE_OPTIMIZED_SHIFTS==1    // The shifts below have been validated to produce performant code in GCC. 
    // Other shift amounts are either in a specialized template below (good) or are unvalidated (bad).
    static_assert(b==1 || b==2 || b==8 || b==16 || b==24,
                  "Unvalidated shift - confirm gcc produces performant code");
#endif
    return a >> b; 
}

#if USE_OPTIMIZED_SHIFTS==1

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"

/// @{
/// @brief uint32_t bitwise right shift optimised for the specified shift distance  
/// 
/// @note The assembler was generated using clang 17.0.1 cross compiling: -O3 --target=avr -mmcu=atmega2560. See https://godbolt.org/z/71cPnMYqs
/// 
/// @param a value to shift
/// @return uint32_t a>>b
template <>
uint32_t rshift<3U>(uint32_t a)
{
    asm(
        "lsr     %D0\n"
        "ror     %C0\n"
        "ror     %B0\n"
        "ror     %A0\n"
        "lsr     %D0\n"
        "ror     %C0\n"
        "ror     %B0\n"
        "ror     %A0\n"
        "lsr     %D0\n"
        "ror     %C0\n"
        "ror     %B0\n"
        "ror     %A0\n"
        : "=d" (a) 
        : "0" (a) 
        : 
    );

    return a;
}

template <>
uint32_t rshift<4U>(uint32_t a)
{
    asm(
        "swap    %A0\n"
        "andi    %A0, 15\n"
        "swap    %B0\n"
        "eor     %A0, %B0\n"
        "andi    %B0, 15\n"
        "eor     %A0, %B0\n"
        "swap    %C0\n"
        "eor     %B0, %C0\n"
        "andi    %C0, 15\n"
        "eor     %B0, %C0\n"
        "swap    %D0\n"
        "eor     %C0, %D0\n"
        "andi    %D0, 15\n"
        "eor     %C0, %D0\n"
        : "=d" (a) 
        : "0" (a) 
        : 
    );

    return a;
}

template <>
uint32_t rshift<5U>(uint32_t a)
{
    asm(
        "swap    %A0\n"
        "andi    %A0, 15\n"
        "swap    %B0\n"
        "eor     %A0, %B0\n"
        "andi    %B0, 15\n"
        "eor     %A0, %B0\n"
        "swap    %C0\n"
        "eor     %B0, %C0\n"
        "andi    %C0, 15\n"
        "eor     %B0, %C0\n"
        "swap    %D0\n"
        "eor     %C0, %D0\n"
        "andi    %D0, 15\n"
        "eor     %C0, %D0\n"
        "lsr     %D0\n"
        "ror     %C0\n"
        "ror     %B0\n"
        "ror     %A0\n"
        : "=d" (a) 
        : "0" (a) 
        : 
    );

    return a;
}

template <>
uint32_t rshift<6U>(uint32_t a)
{
    asm(
        "lsl     %A0\n"
        "rol     %B0\n"
        "rol     %C0\n"
        "rol     %D0\n"
        "mov     r19, __zero_reg__\n"
        "rol     r19\n"
        "lsl     %A0\n"
        "rol     %B0\n"
        "rol     %C0\n"
        "rol     %D0\n"
        "rol     r19\n"
        "mov     %A0, %B0\n"
        "mov     %B0, %C0\n"
        "mov     r18, %D0\n"
        "movw    %C0, r18\n"
        : "=d" (a) 
        : "0" (a) 
        : "r18", "r19"
    );

    return a;
}

template <>
uint32_t rshift<7U>(uint32_t a)
{
    asm(
        "lsl     %A0\n"
        "rol     %B0\n"
        "rol     %C0\n"
        "rol     %D0\n"
        "mov     r19, __zero_reg__\n"
        "rol     r19\n"
        "mov     %A0, %B0\n"
        "mov     %B0, %C0\n"
        "mov     r18, %D0\n"
        "movw    %C0, r18\n"
        : "=d" (a) 
        : "0" (a) 
        : "r18", "r19"
    );

    return a;
}

template <>
uint32_t rshift<9U>(uint32_t a)
{
    asm(
        "lsr     %D0\n"
        "ror     %C0\n"
        "ror     %B0\n"
        "mov     %A0, %B0\n"
        "mov     %B0, %C0\n"
        "mov     %C0, %D0\n"
        "mov     %D0, __zero_reg__\n"
        : "=d" (a) 
        : "0" (a) 
        : 
    );

    return a;
}

template <>
uint32_t rshift<10U>(uint32_t a)
{
    asm(
        "lsr     %D0\n"
        "ror     %C0\n"
        "ror     %B0\n"
        "lsr     %D0\n"
        "ror     %C0\n"
        "ror     %B0\n" 
        "mov     %A0, %B0\n"
        "mov     %B0, %C0\n"
        "mov     %C0, %D0\n"
        "mov     %D0, __zero_reg__\n"
        : "=d" (a) 
        : "0" (a) 
        : 
    );

    return a;
}

template <>
uint32_t rshift<11U>(uint32_t a)
{
    asm(
        "lsr     %D0\n"
        "ror     %C0\n"
        "ror     %B0\n"
        "lsr     %D0\n"
        "ror     %C0\n"
        "ror     %B0\n"
        "lsr     %D0\n"
        "ror     %C0\n"
        "ror     %B0\n"
        "mov     %A0, %B0\n"
        "mov     %B0, %C0\n"
        "mov     %C0, %D0\n"
        "mov     %D0, __zero_reg__\n"
        : "=d" (a) 
        : "0" (a) 
        : 
    );

    return a;
}

template <>
uint32_t rshift<12U>(uint32_t a)
{
    asm(
        "swap    %B0\n"
        "andi    %B0, 15\n"
        "swap    %C0\n"
        "eor     %B0, %C0\n"
        "andi    %C0, 15\n"
        "eor     %B0, %C0\n"
        "swap    %D0\n"
        "eor     %C0, %D0\n"
        "andi    %D0, 15\n"
        "eor     %C0, %D0\n"
        "mov     %A0, %B0\n"
        "mov     %B0, %C0\n"
        "mov     %C0, %D0\n"
        "mov     %D0, __zero_reg__\n"
        : "=d" (a) 
        : "0" (a) 
        : 
    );

    return a;
}

template <>
uint32_t rshift<13U>(uint32_t a)
{
    asm(
        "swap    %B0\n"
        "andi    %B0, 15\n"
        "swap    %C0\n"
        "eor     %B0, %C0\n"
        "andi    %C0, 15\n"
        "eor     %B0, %C0\n"
        "swap    %D0\n"
        "eor     %C0, %D0\n"
        "andi    %D0, 15\n"
        "eor     %C0, %D0\n"
        "lsr     %D0\n"
        "ror     %C0\n"
        "ror     %B0\n"
        "mov     %A0, %B0\n"
        "mov     %B0, %C0\n"
        "mov     %C0, %D0\n"
        "mov     %D0, __zero_reg__\n"
        : "=d" (a) 
        : "0" (a) 
        : 
    );

    return a;
}

template <>
uint32_t rshift<14U>(uint32_t a)
{
    asm(
        "movw    r18, %C0\n"
        "lsl     %B0\n"
        "rol     r18\n"
        "rol     r19\n"
        "mov     %C0, __zero_reg__\n"
        "rol     %C0\n"
        "lsl     %B0\n"
        "rol     r18\n"
        "rol     r19\n"
        "rol     %C0\n"
        "mov     %D0, __zero_reg__\n"
        "movw    %A0, r18\n"
        : "=d" (a) 
        : "0" (a) 
        : "r18", "r19"
    );

    return a;
}

template <>
uint32_t rshift<15U>(uint32_t a)
{
    asm(
        "movw    r18, %C0\n"
        "lsl     %B0\n"
        "rol     r18\n"
        "rol     r19\n"
        "mov     %C0, __zero_reg__\n"
        "rol     %C0\n"
        "mov     %D0, __zero_reg__\n"
        "movw    %A0, r18\n"
        : "=d" (a) 
        : "0" (a) 
        : "r18", "r19"
    );

    return a;
}

///@}

#pragma GCC diagnostic pop

#endif

/**
 * @brief Rounded arithmetic right shift
 * 
 * Right shifting throws away bits. When use for fixed point division, this
 * effecitvely rounds down (towards zero). To round-to-the-nearest-integer
 * when right-shifting by S, just add in 2 power b−1 (which is the 
 * fixed-point equivalent of 0.5) first
 *  
 * @tparam b number of bits to shift by
 * @param a value to shift
 * @return uint32_t 
 */
template <uint8_t b> 
static inline uint32_t rshift_round(uint32_t a) { 
    return rshift<b>(a+(1UL<<(b-1UL))); 
}

///@}
