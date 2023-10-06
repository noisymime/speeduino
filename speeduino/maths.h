#ifndef MATH_H
#define MATH_H

#include "globals.h"
#include <limits.h>

unsigned long percentage(uint8_t x, unsigned long y);
unsigned long halfPercentage(uint8_t x, unsigned long y);
inline long powint(int factor, unsigned int exponent);
uint8_t random1to100();

#ifdef USE_LIBDIVIDE
#include "src/libdivide/libdivide.h"
extern const struct libdivide::libdivide_u16_t libdiv_u16_100;
extern const struct libdivide::libdivide_s16_t libdiv_s16_100;
extern const struct libdivide::libdivide_u32_t libdiv_u32_100;
extern const struct libdivide::libdivide_s32_t libdiv_s32_100;
extern const struct libdivide::libdivide_u32_t libdiv_u32_360;
#endif

inline uint8_t div100(uint8_t n) {
    return n / (uint8_t)100U;
}
inline int8_t div100(int8_t n) {
    return n / (int8_t)100U;
}
inline uint16_t div100(uint16_t n) {
#ifdef USE_LIBDIVIDE    
    return libdivide::libdivide_u16_do(n, &libdiv_u16_100);
#else
    return n / (uint16_t)100U;
#endif
}
inline int16_t div100(int16_t n) {
#ifdef USE_LIBDIVIDE    
    return libdivide::libdivide_s16_do(n, &libdiv_s16_100);
#else
    return n / (int16_t)100;
#endif
}
inline uint32_t div100(uint32_t n) {
#ifdef USE_LIBDIVIDE    
    return libdivide::libdivide_u32_do(n, &libdiv_u32_100);
#else
    return n / (uint32_t)100U;
#endif
}
#if defined(__arm__)
inline int div100(int n) {
#ifdef USE_LIBDIVIDE    
    return libdivide::libdivide_s32_do(n, &libdiv_s32_100);
#else
    return n / (int)100;
#endif
}
#else
inline int32_t div100(int32_t n) {
#ifdef USE_LIBDIVIDE    
    return libdivide::libdivide_s32_do(n, &libdiv_s32_100);
#else
    return n / (int32_t)100;
#endif
}
#endif

inline uint32_t div360(uint32_t n) {
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
 * ~50% quicker than raw 32/32 => 32 division on ATMega
 * 
 * @note Bad things will likely happen if the result doesn't fit into 16-bits.
 * @note Copied from https://stackoverflow.com/a/66593564
 * 
 * @param dividend The dividend (numerator)
 * @param divisor The divisor (denominator)
 * @return uint16_t 
 */
static inline uint16_t  __attribute__((optimize("no-unroll-loops"))) udiv_32_16 (uint32_t dividend, uint16_t divisor)
{
    // Without no-unroll-loops, this routine takes up a lot of flash memory (.text) AND it's
    // inlined in a number of places.
#if defined(CORE_AVR) || defined(ARDUINO_ARCH_AVR)
    if (dividend<UINT16_MAX) { // Just in case  
        return (uint16_t)dividend/divisor;
    }
    uint16_t quot = dividend;        
    uint16_t rem  = dividend >> 16;  

    uint8_t bits = sizeof(uint16_t) * CHAR_BIT;     
    do {
        // (rem:quot) << 1, with carry out
        bool carry = rem >> 15;
        rem  = (rem << 1) | (quot >> 15);
        quot = quot << 1;
        // if partial remainder greater or equal to divisor, subtract divisor
        if (carry || (rem >= divisor)) {
            rem = rem - divisor;
            quot = quot | 1;
        }
        bits--;
    } while (bits);
    return quot;
#else
    // The non-AVR platforms are all fast enough (or have built in dividers)
    // so just fall back to regular 32-bit division.
    return dividend / divisor;
#endif
}


#endif