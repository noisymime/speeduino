#include <stdio.h>
#include <unity.h>
#include "test_fp_support.h"

#if defined(ARDUINO_ARCH_AVR)
#include <fp64lib.h>
using test_float_t = float64_t;
#else
using test_float_t = double;
#endif

test_float_t floatDivision(int32_t a, int32_t b) {
#if defined(ARDUINO_ARCH_AVR)
  return fp64_div(fp64_int32_to_float64(a), fp64_int32_to_float64(b));
#else
  return (double)a/(double)b;
#endif
}

int32_t round_float(test_float_t f) {
#if defined(ARDUINO_ARCH_AVR)
  return fp64_lround(f);
#else
  return round(f);
#endif
}


void assert_rounded_div(int32_t a, int32_t b, int32_t actual, uint8_t delta) {
  test_float_t fExpected = floatDivision(a, b);
  int32_t expected = round_float(fExpected);

  // char msg[64];
  // sprintf(msg, "a: %" PRIi32 ", b:  %" PRIi32 " fExpected: %s", a, b, fp64_to_string(fExpected, 17, 15));
  // TEST_ASSERT_EQUAL_MESSAGE(expected, actual, msg);
  TEST_ASSERT_INT32_WITHIN(delta, expected, actual);
}