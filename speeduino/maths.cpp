#include <Arduino.h>
#include "maths.h"

//Generates a random number from 1 to 100 (inclusive).
//The initial seed used is always based on micros(), though this is unlikely to cause an issue as the first run is nearly random itself
//Function requires 4 bytes to store state and seed, but operates very quickly (around 4uS per call)
static uint8_t a, x, y, z;
uint8_t random1to100(void)
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
