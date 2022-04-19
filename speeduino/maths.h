#ifndef MATH_H
#define MATH_H

#define USE_LIBDIVIDE

int fastMap1023toX(int, int);
unsigned long percentage(uint8_t, unsigned long);
unsigned long halfPercentage(uint8_t, unsigned long);
inline long powint(int, unsigned int);

#ifdef USE_LIBDIVIDE
#include "src/libdivide/libdivide.h"
extern struct libdivide::libdivide_u16_t libdiv_u16_100;
extern struct libdivide::libdivide_s16_t libdiv_s16_100;
extern struct libdivide::libdivide_u32_t libdiv_u32_100;
extern struct libdivide::libdivide_s32_t libdiv_s32_100;
extern struct libdivide::libdivide_u32_t libdiv_u32_360;
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

//This is a dedicated function that specifically handles the case of mapping 0-1023 values into a 0 to X range
//This is a common case because it means converting from a standard 10-bit analog input to a byte or 10-bit analog into 0-511 (Eg the temperature readings)
#define fastMap1023toX(x, out_max) ( ((unsigned long)x * out_max) >> 10)
//This is a new version that allows for out_min
#define fastMap10Bit(x, out_min, out_max) ( ( ((unsigned long)x * (out_max-out_min)) >> 10 ) + out_min)

#endif