#include "maths.h"
#include <stdlib.h>

#ifdef USE_LIBDIVIDE
// Constants used for libdivide. Using predefined constants saves flash and RAM (.bss)
// versus calling the libdivide generator functions (E.g. libdivide_s32_gen)
// 32-bit constants generated here: https://godbolt.org/z/vP8Kfejo9
const libdivide::libdivide_u32_t libdiv_u32_100 = { .magic = 2748779070, .more = 6 };
const libdivide::libdivide_s32_t libdiv_s32_100 = { .magic = 1374389535, .more = 5 };
const libdivide::libdivide_u32_t libdiv_u32_200 = { .magic = 2748779070, .more = 7 };
const libdivide::libdivide_u32_t libdiv_u32_360 = { .magic = 1813430637, .more = 72 };
#endif

//Same as above, but 0.5% accuracy
unsigned long halfPercentage(uint8_t x, unsigned long y)
{
#ifdef USE_LIBDIVIDE    
    return libdivide::libdivide_u32_do(y * x, &libdiv_u32_200);
#else
    return (y * x) / 200U;
#endif  
}

//Generates a random number from 1 to 100 (inclusive).
//The initial seed used is always based on micros(), though this is unlikely to cause an issue as the first run is nearly random itself
//Function requires 4 bytes to store state and seed, but operates very quickly (around 4uS per call)
static uint8_t a, x, y, z;
uint8_t random1to100()
{
  //Check if this is the first time being run. If so, seed the random number generator with micros()
  if( (a == 0U) && (x == 0U) && (y == 0U) && (z == 0U) )
  {
    x = micros() >> 24U;
    y = micros() >> 16U;
    z = micros() >> 8U;
    a = micros();
  }

  do
  {
    unsigned char t = x ^ (x << 4);
		x=y;
		y=z;
		z=a;
		a = z ^ t ^ ( z >> 1) ^ (t << 1);
  }
  while(a >= 100U);
  return (a+1U);
}
