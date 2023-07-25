#ifndef MATH_H
#define MATH_H

#include <limits.h>

unsigned long percentage(uint8_t x, unsigned long y);
unsigned long halfPercentage(uint8_t x, unsigned long y);
inline long powint(int factor, unsigned int exponent);
uint8_t random1to100();

#ifdef USE_LIBDIVIDE
#include "src/libdivide/libdivide.h"
#include "src/libdivide/constant_fast_div.h"
extern const struct libdivide::libdivide_u32_t libdiv_u32_100;
extern const struct libdivide::libdivide_s32_t libdiv_s32_100;
extern const struct libdivide::libdivide_u32_t libdiv_u32_360;
#endif

static inline uint8_t div100(uint8_t n) {
    return n / (uint8_t)100U;
}
static inline int8_t div100(int8_t n) {
    return n / (int8_t)100U;
}
static inline uint16_t div100(uint16_t n) {
    // As of avr-gcc 5.4.0, the compiler will optimize this to a multiply/shift
    // (unlike the signed integer overload, where __divmodhi4 is still called);
    return n / (uint16_t)100U;
}
static inline int16_t div100(int16_t n) {
#ifdef USE_LIBDIVIDE    
    return libdivide::libdivide_s16_do_raw(n, S16_MAGIC(100), S16_MORE(100));
#else
    return n / (int16_t)100;
#endif
}
inline uint32_t div100(uint32_t n) {
    if (n<UINT16_MAX) {
        return div100((uint16_t)n);
    }
#ifdef USE_LIBDIVIDE    
    return libdivide::libdivide_u32_do(n, &libdiv_u32_100);
#else
    return n / (uint32_t)100U;
#endif
}

#if defined(__arm__)
static inline int div100(int n) {
#ifdef USE_LIBDIVIDE    
    return libdivide::libdivide_s32_do(n, &libdiv_s32_100);
#else
    return n / (int)100;
#endif
}
#else
static inline int32_t div100(int32_t n) {
#ifdef USE_LIBDIVIDE    
    if (n<=INT16_MAX) {
        if (n>=0) {
            return div100((uint16_t)n);
        } else if (n>=INT16_MIN) {
            return div100((int16_t)n);            
        }
    }
    return libdivide::libdivide_s32_do(n, &libdiv_s32_100);
#else
    return n / (int32_t)100;
#endif
}
#endif

static inline uint32_t div360(uint32_t n) {
#ifdef USE_LIBDIVIDE
    return libdivide::libdivide_u32_do(n, &libdiv_u32_360);
#else
    return n / 360U;
#endif
}

#define DIV_ROUND_CLOSEST(n, d) ((((n) < 0) ^ ((d) < 0)) ? (((n) - (d)/2)/(d)) : (((n) + (d)/2)/(d)))
#define IS_INTEGER(d) (d == (int32_t)d)

//This is a dedicated function that specifically handles the case of mapping 0-1023 values into a 0 to X range
//This is a common case because it means converting from a standard 10-bit analog input to a byte or 10-bit analog into 0-511 (Eg the temperature readings)
#define fastMap1023toX(x, out_max) ( ((unsigned long)x * out_max) >> 10)
//This is a new version that allows for out_min
#define fastMap10Bit(x, out_min, out_max) ( ( ((unsigned long)x * (out_max-out_min)) >> 10 ) + out_min)

/**
 * @brief Optimised division: uint32_t/uint16_t => uint16_t
 * 
 * Optimised division of unsigned 32-bit by unsigned 16-bit when it is known
 * that the result fits into unsigned 16-bit.
 * 
 * ~60% quicker than raw 32/32 => 32 division on ATMega
 * 
 * @note Bad things will likely happen if the result doesn't fit into 16-bits.
 * @note Copied from https://stackoverflow.com/a/66593564
 * 
 * @param dividend The dividend (numerator)
 * @param divisor The divisor (denominator)
 * @return uint16_t 
 */
static inline uint16_t udiv_32_16 (uint32_t dividend, uint16_t divisor)
{
    // Without no-unroll-loops, this routine takes up a lot of flash memory (.text) AND it's
    // inlined in a number of places.
#if defined(CORE_AVR) || defined(ARDUINO_ARCH_AVR)
    if (dividend<=UINT16_MAX) { // Just in case  
        return (uint16_t)dividend/divisor;
    }

    #define INDEX_REG "r16"

    asm(
        "    ldi " INDEX_REG ", 16 ; bits = 16\n\t"
        "0:\n\t"
        "    lsl  %A0     ; shift\n\t"
        "    rol  %B0     ;  rem:quot\n\t"
        "    rol  %C0     ;   left\n\t"
        "    rol  %D0     ;    by 1\n\t"
        "    brcs 1f     ; if carry out, rem > divisor\n\t"
        "    cp   %C0, %A1 ; is rem less\n\t"
        "    cpc  %D0, %B1 ;  than divisor ?\n\t"
        "    brcs 2f     ; yes, when carry out\n\t"
        "1:\n\t"
        "    sub  %C0, %A1 ; compute\n\t"
        "    sbc  %D0, %B1 ;  rem -= divisor\n\t"
        "    ori  %A0, 1  ; record quotient bit as 1\n\t"
        "2:\n\t"
        "    dec  " INDEX_REG "     ; bits--\n\t"
        "    brne 0b     ; until bits == 0"
        : "=d" (dividend) 
        : "d" (divisor) , "0" (dividend) 
        : INDEX_REG
    );

    // Lower word contains the quotient, upper word contains the remainder.
    return dividend & 0xFFFF;
#else
    // The non-AVR platforms are all fast enough (or have built in hardware dividers)
    // so just fall back to regular 32-bit division.
    return dividend / divisor;
#endif
}


#endif