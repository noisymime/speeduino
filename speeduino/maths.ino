#include "maths.h"
#include <stdlib.h>

#ifdef USE_LIBDIVIDE
#include "src/libdivide/constant_fast_div.h"

// Constants used for libdivide. Using predefined constants saves flash and RAM (.bss)
// versus calling the libdivide generator functions (E.g. libdivide_s32_gen)
libdivide::libdivide_s16_t libdiv_s16_100 = { .magic = S16_MAGIC(100), .more = S16_MORE(100) };
libdivide::libdivide_u16_t libdiv_u16_100 = { .magic = U16_MAGIC(100), .more = U16_MORE(100) };
// 32-bit constants generated here: https://godbolt.org/z/vP8Kfejo9
libdivide::libdivide_u32_t libdiv_u32_100 = { .magic = 2748779070, .more = 6 };
libdivide::libdivide_s32_t libdiv_s32_100 = { .magic = 1374389535, .more = 5 };
libdivide::libdivide_u32_t libdiv_u32_200 = { .magic = 2748779070, .more = 7 };
libdivide::libdivide_u32_t libdiv_u32_360 = { .magic = 1813430637, .more = 72 };
#endif

//Replace the standard arduino map() function to use the div function instead
int fastMap(unsigned long x, int in_min, int in_max, int out_min, int out_max)
{
  unsigned long a = (x - (unsigned long)in_min);
  int b = (out_max - out_min);
  int c = (in_max - in_min);
  int d = (ldiv( (a * (long)b) , (long)c ).quot);
  return d + out_min;
  //return ldiv( ((x - in_min) * (out_max - out_min)) , (in_max - in_min) ).quot + out_min;
  //return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Return x percent of y
//This is a relatively fast approximation of a percentage value.
unsigned long percentage(uint8_t x, unsigned long y)
{
  return div100(y * x);
}

//Same as above, but 0.5% accuracy
unsigned long halfPercentage(uint8_t x, unsigned long y)
{
#ifdef USE_LIBDIVIDE    
    return libdivide::libdivide_u32_do(y * x, &libdiv_u32_200);
#else
    return (y * x) / 200;
#endif  
}

/*
 * Calculates integer power values. Same as pow() but with ints
 */
inline long powint(int factor, unsigned int exponent)
{
  long product = 1;
  unsigned int counter = exponent;
  while ( (counter--) > 0) { product *= factor; }
  return product;
}
