#include "maths.h"
#include <stdlib.h>

#ifdef USE_LIBDIVIDE
#include "src/libdivide/constant_fast_div.h"

// Constants used for libdivide. Using predefined constants saves flash and RAM (.bss)
// versus calling the libdivide generator functions (E.g. libdivide_s32_gen)
const libdivide::libdivide_s16_t libdiv_s16_100 = { .magic = S16_MAGIC(100), .more = S16_MORE(100) };
const libdivide::libdivide_u16_t libdiv_u16_100 = { .magic = U16_MAGIC(100), .more = U16_MORE(100) };
// 32-bit constants generated here: https://godbolt.org/z/vP8Kfejo9
const libdivide::libdivide_u32_t libdiv_u32_100 = { .magic = 2748779070, .more = 6 };
const libdivide::libdivide_s32_t libdiv_s32_100 = { .magic = 1374389535, .more = 5 };
const libdivide::libdivide_u32_t libdiv_u32_200 = { .magic = 2748779070, .more = 7 };
const libdivide::libdivide_u32_t libdiv_u32_360 = { .magic = 1813430637, .more = 72 };
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

//Generates a random number from 1 to 100 (inclusive).
//The initial seed used is always based on micros(), though this is unlikely to cause an issue as the first run is nearly random itself
//Function requires 4 bytes to store state and seed, but operates very quickly (around 4uS per call)
uint8_t a, x, y, z;
uint8_t random1to100()
{
  //Check if this is the first time being run. If so, seed the random number generator with micros()
  if( (a == 0) && (x == 0) && (y == 0) && (z == 0) )
  {
    x = micros() >> 24;
    y = micros() >> 16;
    z = micros() >> 8;
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
  while(a >= 100);
  return (a+1);
}
