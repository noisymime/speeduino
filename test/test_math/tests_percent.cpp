#include <unity.h>
#include "test_fp_support.h"
#include "maths.h"
#include "../timer.hpp"
#include "../test_utils.h"

static void test_percent(uint8_t percent, uint16_t value) {
  assert_rounded_div((uint32_t)percent*value, 100, percentage(percent, value));
}

void test_maths_percent_U8(void)
{
  uint8_t percentOf = 77;
  test_percent(0, percentOf);
  test_percent(33, percentOf);
  test_percent(50, percentOf);
  test_percent(66, percentOf);
  test_percent(75, percentOf);
  test_percent(100, percentOf);
  test_percent(125, percentOf);
}

void test_maths_percent_U16(void)
{
  uint16_t percentOf = 33333;
  test_percent(0, percentOf);
  test_percent(33, percentOf);
  test_percent(50, percentOf);
  test_percent(66, percentOf);
  test_percent(75, percentOf);
  test_percent(100, percentOf);
  test_percent(125, percentOf);
}


static void test_halfPercentage(uint8_t percent, uint16_t value) {
  assert_rounded_div((int32_t)percent*value, 200, halfPercentage(percent, value));
}

void test_maths_halfpercent_U8(void)
{
  uint8_t percentOf = 111;
  test_halfPercentage(0, percentOf);
  test_halfPercentage(33, percentOf);
  test_halfPercentage(50, percentOf);
  test_halfPercentage(66, percentOf);
  test_halfPercentage(75, percentOf);
  test_halfPercentage(100, percentOf);
  test_halfPercentage(125, percentOf);  
}

void test_maths_halfpercent_U16(void)
{
  uint16_t percentOf = 57357;
  test_halfPercentage(0, percentOf);
  test_halfPercentage(33, percentOf);
  test_halfPercentage(50, percentOf);
  test_halfPercentage(66, percentOf);
  test_halfPercentage(75, percentOf);
  test_halfPercentage(100, percentOf);
  test_halfPercentage(125, percentOf); 
}

static void test_percentApprox(uint8_t percent, uint16_t value) {
  uint8_t delta = ((uint32_t)percent*value)/100; // 1% allowable delta
  assert_rounded_div((uint32_t)percent*value, 100, percentageApprox(percent, value), delta);
}

static void test_maths_percentApprox(void)
{
  uint16_t percentOf = 33333;
  test_percentApprox(0, percentOf);
  test_percentApprox(33, percentOf);
  test_percentApprox(50, percentOf);
  test_percentApprox(66, percentOf);
  test_percentApprox(75, percentOf);
  test_percentApprox(100, percentOf);
  test_percentApprox(125, percentOf);
}

// These are shared by all percentage perf tests for consistency
static constexpr int16_t iters = 4;
static constexpr uint16_t start_percent = 3;
static constexpr uint16_t end_percent = 2047;
static constexpr uint16_t percent_step = 3;
static constexpr uint16_t percentOf = 57357;

void test_maths_halfPercentage_perf(void)
{
#if defined(ARDUINO_ARCH_AVR)
    auto nativeTest = [] (uint16_t index, uint32_t &checkSum) { checkSum += ((uint32_t)percentOf * index) / 200U; };
    auto optimizedTest = [] (uint16_t index, uint32_t &checkSum) { checkSum += halfPercentage(index, percentOf); };
    TEST_MESSAGE("halfPercentage ");
    auto comparison = compare_executiontime<uint16_t, uint32_t>(iters, start_percent, end_percent, percent_step, nativeTest, optimizedTest);
    
    // The checksums will be different due to rounding. This is only
    // here to force the compiler to run the loops above
    TEST_ASSERT_INT32_WITHIN(UINT32_MAX/2, comparison.timeA.result, comparison.timeB.result);

    TEST_ASSERT_LESS_THAN(comparison.timeA.durationMicros, comparison.timeB.durationMicros);
#endif
}


void test_maths_percentage_perf(void)
{
#if defined(ARDUINO_ARCH_AVR)
    auto nativeTest = [] (uint16_t index, uint32_t &checkSum) { checkSum += ((uint32_t)percentOf * index) / 100U; };
    auto optimizedTest = [] (uint16_t index, uint32_t &checkSum) { checkSum += percentage(index, percentOf); };
    TEST_MESSAGE("Percentage ");
    auto comparison = compare_executiontime<uint16_t, uint32_t>(iters, start_percent, end_percent, percent_step, nativeTest, optimizedTest);
    
    // The checksums will be different due to rounding. This is only
    // here to force the compiler to run the loops above
    TEST_ASSERT_INT32_WITHIN(UINT32_MAX/2, comparison.timeA.result, comparison.timeB.result);

    TEST_ASSERT_LESS_THAN(comparison.timeA.durationMicros, comparison.timeB.durationMicros);
#endif
}


void test_maths_percentageApprox_perf(void)
{
#if defined(ARDUINO_ARCH_AVR)
    auto nativeTest = [] (uint16_t index, uint32_t &checkSum) { checkSum += ((uint32_t)percentOf * index) / 100U; };
    auto optimizedTest = [] (uint16_t index, uint32_t &checkSum) { checkSum += percentageApprox(index, percentOf); };
    TEST_MESSAGE("percentageApprox ");
    auto comparison = compare_executiontime<uint16_t, uint32_t>(iters, start_percent, end_percent, percent_step, nativeTest, optimizedTest);
    
    // The checksums will be different due to rounding. This is only
    // here to force the compiler to run the loops above
    TEST_ASSERT_INT32_WITHIN(UINT32_MAX/2, comparison.timeA.result, comparison.timeB.result);

    TEST_ASSERT_LESS_THAN(comparison.timeA.durationMicros, comparison.timeB.durationMicros);
#endif
}

void testPercent()
{
  SET_UNITY_FILENAME() {

  RUN_TEST(test_maths_percent_U8);
  RUN_TEST(test_maths_percent_U16);
  RUN_TEST(test_maths_halfpercent_U8);
  RUN_TEST(test_maths_halfpercent_U16);
  RUN_TEST(test_maths_halfPercentage_perf);
  RUN_TEST(test_maths_percentage_perf);
  RUN_TEST(test_maths_percentageApprox_perf);
  RUN_TEST(test_maths_percentApprox);
  }
}
