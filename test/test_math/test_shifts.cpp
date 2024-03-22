#include <unity.h>
#include <Arduino.h>
#include "bit_shifts.h"
#include "maths.h"
#include "../timer.hpp"

template <uint8_t shiftDistance>
static void test_lshift(void) {
    uint32_t value = 33333;
    char szMsg[16];
    sprintf(szMsg, "%" PRIu8, shiftDistance);
    TEST_ASSERT_EQUAL_MESSAGE(value << shiftDistance, lshift<shiftDistance>(value), szMsg);
}

static void test_LShift()
{
    test_lshift<1U>();
    test_lshift<2U>();
    test_lshift<3U>();
    test_lshift<4U>();
    test_lshift<5U>();
    test_lshift<6U>();
    test_lshift<7U>();
    test_lshift<8U>();
    test_lshift<9U>();
    test_lshift<10U>();
    test_lshift<11U>();
    test_lshift<12U>();
    test_lshift<13U>();
    test_lshift<14U>();
    test_lshift<15U>();
}

template <uint8_t shiftDistance>
static void test_rshift(void) {
    uint32_t value = 33333;
    char szMsg[16];
    sprintf(szMsg, "%" PRIu8, shiftDistance);
    TEST_ASSERT_EQUAL_MESSAGE(value >> shiftDistance, rshift<shiftDistance>(value), szMsg);
}

void test_RShift()
{
    test_rshift<1U>();
    test_rshift<2U>();
    test_rshift<3U>();
    test_rshift<4U>();
    test_rshift<5U>();
    test_rshift<6U>();
    test_rshift<7U>();
    test_rshift<8U>();
    test_rshift<9U>();
    test_rshift<10U>();
    test_rshift<11U>();
    test_rshift<12U>();
    test_rshift<13U>();
    test_rshift<14U>();
    test_rshift<15U>();
}

static uint32_t seedValue;

// Force no inline, or compiler will optimize shifts away 
// (which it won't do in normal operaton when the left shift operand is unknown at compile time.)
static void __attribute__((noinline)) nativeTest(uint8_t index, uint32_t &checkSum) { 
    if (index==1U) { checkSum = seedValue; }
    if (index==4U) { checkSum += checkSum >> 4U; }
    if (index==5U) { checkSum += checkSum >> 5U; }
    if (index==6U) { checkSum += checkSum >> 6U; }
    if (index==7U) { checkSum += checkSum >> 7U; }
    if (index==9U) { checkSum += checkSum >> 9U; }
    if (index==10U) { checkSum += checkSum >> 10U; }
    if (index==11U) { checkSum += checkSum >> 11U; }
    if (index==12U) { checkSum += checkSum >> 12U; }
    if (index==13U) { checkSum += checkSum >> 13U; }
    if (index==14U) { checkSum += checkSum >> 14U; }
    if (index==15U) { checkSum += checkSum >> 15U; }
};
static void __attribute__((noinline)) optimizedTest(uint8_t index, uint32_t &checkSum) {
    if (index==1U) { checkSum = seedValue; }
    if (index==4U) { checkSum += rshift<4U>(checkSum); }
    if (index==5U) { checkSum += rshift<5U>(checkSum); }
    if (index==6U) { checkSum += rshift<6U>(checkSum); }
    if (index==7U) { checkSum += rshift<7U>(checkSum); }
    if (index==9U) { checkSum += rshift<9U>(checkSum); }
    if (index==10U) { checkSum += rshift<10U>(checkSum); }
    if (index==11U) { checkSum += rshift<11U>(checkSum); }
    if (index==12U) { checkSum += rshift<12U>(checkSum); }
    if (index==13U) { checkSum += rshift<13U>(checkSum); }
    if (index==14U) { checkSum += rshift<14U>(checkSum); }
    if (index==15U) { checkSum += rshift<15U>(checkSum); }
};      

static void test_rshift_perf(void) {
    constexpr uint16_t iters = 128;
    constexpr uint8_t start_index = 1;
    constexpr uint8_t end_index = 16;
    constexpr uint8_t step = 1;

    seedValue = rand();

    TEST_MESSAGE("rshift ");
    auto comparison = compare_executiontime<uint8_t, uint32_t>(iters, start_index, end_index, step, nativeTest, optimizedTest);
    
    // This must be here to force the compiler to run the loops above
    TEST_ASSERT_EQUAL(comparison.timeA.result, comparison.timeB.result);

    TEST_ASSERT_LESS_THAN(comparison.timeA.durationMicros, comparison.timeB.durationMicros);
}

void testBitShift() {
    RUN_TEST(test_LShift);
    RUN_TEST(test_RShift);
    RUN_TEST(test_rshift_perf);
}
