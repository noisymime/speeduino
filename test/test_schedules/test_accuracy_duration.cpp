
#include <Arduino.h>
#include <unity.h>
#include "../test_utils.h"
#include "scheduler.h"

#define TIMEOUT 1000
#define DURATION 1000
#define DELTA 20

static uint32_t start_time, end_time;
static void startCallback(void) { start_time = micros(); }
static void endCallback(void) { end_time = micros(); }

void test_accuracy_duration_inj(FuelSchedule &schedule)
{
    initialiseSchedulers();
    setCallbacks(schedule, startCallback, endCallback);
    setFuelSchedule(schedule, TIMEOUT, DURATION);
    while(schedule.Status != OFF) /*Wait*/ ;
    TEST_ASSERT_UINT32_WITHIN(DELTA, DURATION, end_time - start_time);
}

void test_accuracy_duration_inj1(void)
{
    test_accuracy_duration_inj(fuelSchedule1);
}

void test_accuracy_duration_inj2(void)
{
    test_accuracy_duration_inj(fuelSchedule2);
}

void test_accuracy_duration_inj3(void)
{
    test_accuracy_duration_inj(fuelSchedule3);
}

void test_accuracy_duration_inj4(void)
{
    test_accuracy_duration_inj(fuelSchedule4);
}

#if INJ_CHANNELS >= 5
void test_accuracy_duration_inj5(void)
{
    test_accuracy_duration_inj(fuelSchedule5);
}
#endif

#if INJ_CHANNELS >= 6
void test_accuracy_duration_inj6(void)
{
    test_accuracy_duration_inj(fuelSchedule6);
}
#endif

#if INJ_CHANNELS >= 7
void test_accuracy_duration_inj7(void)
{
    test_accuracy_duration_inj(fuelSchedule7);
}
#endif

#if INJ_CHANNELS >= 8
void test_accuracy_duration_inj8(void)
{
    test_accuracy_duration_inj(fuelSchedule8);
}
#endif

void test_accuracy_duration_ign(IgnitionSchedule &schedule)
{
    initialiseSchedulers();
    setCallbacks(schedule, startCallback, endCallback);
    setIgnitionSchedule(schedule, TIMEOUT, DURATION);
    while(schedule.Status != OFF) /*Wait*/ ;
    TEST_ASSERT_UINT32_WITHIN(DELTA, DURATION, end_time - start_time);    

}
void test_accuracy_duration_ign1(void)
{
    test_accuracy_duration_ign(ignitionSchedule1);
}

void test_accuracy_duration_ign2(void)
{
    test_accuracy_duration_ign(ignitionSchedule2);
}

void test_accuracy_duration_ign3(void)
{
    test_accuracy_duration_ign(ignitionSchedule3);
}

void test_accuracy_duration_ign4(void)
{
    test_accuracy_duration_ign(ignitionSchedule4);
}

void test_accuracy_duration_ign5(void)
{
#if IGN_CHANNELS >= 5
    test_accuracy_duration_ign(ignitionSchedule5);
#endif
}

#if INJ_CHANNELS >= 6
void test_accuracy_duration_ign6(void)
{
    test_accuracy_duration_ign(ignitionSchedule6);
}
#endif

#if INJ_CHANNELS >= 7
void test_accuracy_duration_ign7(void)
{
    test_accuracy_duration_ign(ignitionSchedule7);
}
#endif

#if INJ_CHANNELS >= 8
void test_accuracy_duration_ign8(void)
{
    test_accuracy_duration_ign(ignitionSchedule8);
}
#endif

void test_accuracy_duration(void)
{
  SET_UNITY_FILENAME() {

    RUN_TEST(test_accuracy_duration_inj1);
    RUN_TEST(test_accuracy_duration_inj2);
    RUN_TEST(test_accuracy_duration_inj3);
    RUN_TEST(test_accuracy_duration_inj4);
#if INJ_CHANNELS >= 5
    RUN_TEST(test_accuracy_duration_inj5);
#endif
#if INJ_CHANNELS >= 6
    RUN_TEST(test_accuracy_duration_inj6);
#endif
#if INJ_CHANNELS >= 7
    RUN_TEST(test_accuracy_duration_inj7);
#endif
#if INJ_CHANNELS >= 8
    RUN_TEST(test_accuracy_duration_inj8);
#endif

    RUN_TEST(test_accuracy_duration_ign1);
    RUN_TEST(test_accuracy_duration_ign2);
    RUN_TEST(test_accuracy_duration_ign3);
    RUN_TEST(test_accuracy_duration_ign4);
#if INJ_CHANNELS >= 5
    RUN_TEST(test_accuracy_duration_ign5);
#endif
#if INJ_CHANNELS >= 6
    RUN_TEST(test_accuracy_duration_ign6);
#endif
#if INJ_CHANNELS >= 7
    RUN_TEST(test_accuracy_duration_ign7);
#endif
#if INJ_CHANNELS >= 8
    RUN_TEST(test_accuracy_duration_ign8);
#endif
  }
}
