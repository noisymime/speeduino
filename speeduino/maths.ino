#include "maths.h"
#include "globals.h"
#include "src/libdivide/libdivide.h"

#ifdef USE_LIBDIVIDE
  //Constants used for libdivide
  struct libdivide::libdivide_u32_t libdiv_u32_10 = libdivide::libdivide_u32_gen(10);
  struct libdivide::libdivide_u32_t libdiv_u32_100 = libdivide::libdivide_u32_gen(100);
  struct libdivide::libdivide_s32_t libdiv_s32_100 = libdivide::libdivide_s32_gen(100);
  struct libdivide::libdivide_u32_t libdiv_u32_200 = libdivide::libdivide_u32_gen(200);
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

//Unsigned divide by 10
uint32_t divu10(uint32_t n)
{
#ifdef USE_LIBDIVIDE
  
  //Check whether 16 or 32 bit divide is required
  if( n <= UINT8_MAX) { return (uint8_t)n / 10; }
  else if( n <= UINT16_MAX ) { return FAST_DIV16U(n, 10); }
  else { return libdivide::libdivide_u32_do(n, &libdiv_u32_10); }
  //return libdivide::libdivide_u32_do(n, &libdiv_u32_10);

#else
  return (n / 10);
#endif
}

//Signed divide by 100
int32_t divs100(int32_t n)
{
#ifdef USE_LIBDIVIDE
  return libdivide::libdivide_s32_do(n, &libdiv_s32_100);
#else
  return (n / 100); // Amazingly, gcc is producing a better /divide by 100 function than this
#endif
}

//Unsigned divide by 100
unsigned long divu100(unsigned long n)
{
#ifdef USE_LIBDIVIDE

  //Check whether 8, 16 or 32 bit divide is required
  if( n <= UINT8_MAX) { return (uint8_t)n / 100; }
  else if( n <= UINT16_MAX ) { return FAST_DIV16U(n, 100); }
  else { return libdivide::libdivide_u32_do(n, &libdiv_u32_100); }
  //return libdivide::libdivide_u32_do(n, &libdiv_u32_100);

#else
  return (n / 100);
#endif
}


//Return x percent of y
//This is a relatively fast approximation of a percentage value.
unsigned long percentage(byte x, unsigned long y)
{
#ifdef USE_LIBDIVIDE
  return divu100((y * x));
#else
  return (y * x) / 100;
#endif
}

//Same as above, but 0.5% accuracy
unsigned long halfPercentage(byte x, unsigned long y)
{
#ifdef USE_LIBDIVIDE
  
  uint32_t numerator = (y * x);
  if( numerator <= UINT8_MAX) { return (uint8_t)numerator / 200; }
  else if( numerator <= UINT16_MAX ) { return FAST_DIV16U((uint16_t)numerator, 200); }
  else { return libdivide::libdivide_u32_do(numerator, &libdiv_u32_200); }
  
  //return libdivide::libdivide_u32_do((y * x), &libdiv_u32_200);
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
