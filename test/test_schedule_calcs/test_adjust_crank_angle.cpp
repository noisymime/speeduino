#include <Arduino.h>
#include <unity.h>
#include "scheduler.h"
#include "schedule_calcs.h"
#include "../test_utils.h"

void test_adjust_crank_angle_pending_below_minrevolutions()
{
    auto counter = decltype(+IGN4_COUNTER){0};
    auto compare = decltype(+IGN4_COMPARE){0};
    IgnitionSchedule schedule(counter, compare);

    schedule.Status = PENDING;
    currentStatus.startRevolutions = 0;

    schedule._compare = 101;
    schedule._counter = 100;

    // Should do nothing.
    adjustCrankAngle(schedule, 0, 359, 180);

    TEST_ASSERT_EQUAL(101, schedule._compare);
    TEST_ASSERT_EQUAL(100, schedule._counter);
    TEST_ASSERT_FALSE(schedule.endScheduleSetByDecoder);
}


void test_adjust_crank_angle_pending_above_minrevolutions()
{
    auto counter = decltype(+IGN4_COUNTER){0};
    auto compare = decltype(+IGN4_COMPARE){0};
    IgnitionSchedule schedule(counter, compare);
    
    schedule.Status = PENDING;
    currentStatus.startRevolutions = 2000;
    // timePerDegreex16 = 666;

    schedule._compare = 101;
    schedule._counter = 100;
    constexpr uint16_t newCrankAngle = 180;
    constexpr uint16_t chargeAngle = 359;

    adjustCrankAngle(schedule, chargeAngle, 0, newCrankAngle);

    TEST_ASSERT_EQUAL(schedule._counter+uS_TO_TIMER_COMPARE(angleToTimeMicroSecPerDegree(chargeAngle-newCrankAngle)), schedule._compare);
    TEST_ASSERT_EQUAL(100, schedule._counter);
    TEST_ASSERT_TRUE(schedule.endScheduleSetByDecoder);
}

void test_adjust_crank_angle_running()
{
    auto counter = decltype(+IGN4_COUNTER){0};
    auto compare = decltype(+IGN4_COMPARE){0};
    IgnitionSchedule schedule(counter, compare);
    
    schedule.Status = RUNNING;
    currentStatus.startRevolutions = 2000;
    // timePerDegreex16 = 666;

    schedule._compare = 101;
    schedule._counter = 100;
    constexpr uint16_t newCrankAngle = 180;
    constexpr uint16_t chargeAngle = 359;

    adjustCrankAngle(schedule, 0, chargeAngle, newCrankAngle);

    TEST_ASSERT_EQUAL(schedule._counter+uS_TO_TIMER_COMPARE(angleToTimeMicroSecPerDegree(chargeAngle-newCrankAngle)), schedule._compare);
    TEST_ASSERT_EQUAL(100, schedule._counter);
    TEST_ASSERT_FALSE(schedule.endScheduleSetByDecoder);
}

void test_adjust_crank_angle()
{
  SET_UNITY_FILENAME() {

    RUN_TEST(test_adjust_crank_angle_pending_below_minrevolutions);
    RUN_TEST(test_adjust_crank_angle_pending_above_minrevolutions);
    RUN_TEST(test_adjust_crank_angle_running);
  }
}