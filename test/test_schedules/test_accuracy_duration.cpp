
#include <Arduino.h>
#include <unity.h>

#include "scheduler.h"

#define TIMEOUT 1000
#define DURATION 1000
#define DELTA 20

static uint32_t start_time, end_time;
static void startCallback(void) { start_time = micros(); }
static void endCallback(void) { end_time = micros(); }

void test_accuracy_duration_inj1(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule1, TIMEOUT, DURATION);
    while(fuelSchedule1.Status == PENDING) /*Wait*/ ;
    start_time = micros();
    while(fuelSchedule1.Status == RUNNING) /*Wait*/ ;
    end_time = micros();
    TEST_ASSERT_UINT32_WITHIN(DELTA, DURATION, end_time - start_time);
}

void test_accuracy_duration_inj2(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule2, TIMEOUT, DURATION);
    while(fuelSchedule2.Status == PENDING) /*Wait*/ ;
    start_time = micros();
    while(fuelSchedule2.Status == RUNNING) /*Wait*/ ;
    end_time = micros();
    TEST_ASSERT_UINT32_WITHIN(DELTA, DURATION, end_time - start_time);
}

void test_accuracy_duration_inj3(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule3, TIMEOUT, DURATION);
    while(fuelSchedule3.Status == PENDING) /*Wait*/ ;
    start_time = micros();
    while(fuelSchedule3.Status == RUNNING) /*Wait*/ ;
    end_time = micros();
    TEST_ASSERT_UINT32_WITHIN(DELTA, DURATION, end_time - start_time);
}

void test_accuracy_duration_inj4(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule4, TIMEOUT, DURATION);
    while(fuelSchedule4.Status == PENDING) /*Wait*/ ;
    start_time = micros();
    while(fuelSchedule4.Status == RUNNING) /*Wait*/ ;
    end_time = micros();
    TEST_ASSERT_UINT32_WITHIN(DELTA, DURATION, end_time - start_time);
}

#if INJ_CHANNELS >= 5
void test_accuracy_duration_inj5(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule5, TIMEOUT, DURATION);
    while(fuelSchedule5.Status == PENDING) /*Wait*/ ;
    start_time = micros();
    while(fuelSchedule5.Status == RUNNING) /*Wait*/ ;
    end_time = micros();
    TEST_ASSERT_UINT32_WITHIN(DELTA, DURATION, end_time - start_time);
}
#endif

#if INJ_CHANNELS >= 6
void test_accuracy_duration_inj6(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule6, TIMEOUT, DURATION);
    while(fuelSchedule6.Status == PENDING) /*Wait*/ ;
    start_time = micros();
    while(fuelSchedule6.Status == RUNNING) /*Wait*/ ;
    end_time = micros();
    TEST_ASSERT_UINT32_WITHIN(DELTA, DURATION, end_time - start_time);
}
#endif

#if INJ_CHANNELS >= 7
void test_accuracy_duration_inj7(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule7, TIMEOUT, DURATION);
    while(fuelSchedule7.Status == PENDING) /*Wait*/ ;
    start_time = micros();
    while(fuelSchedule7.Status == RUNNING) /*Wait*/ ;
    end_time = micros();
    TEST_ASSERT_UINT32_WITHIN(DELTA, DURATION, end_time - start_time);
}
#endif

#if INJ_CHANNELS >= 8
void test_accuracy_duration_inj8(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule8, TIMEOUT, DURATION);
    while(fuelSchedule8.Status == PENDING) /*Wait*/ ;
    start_time = micros();
    while(fuelSchedule8.Status == RUNNING) /*Wait*/ ;
    end_time = micros();
    TEST_ASSERT_UINT32_WITHIN(DELTA, DURATION, end_time - start_time);
}
#endif


void test_accuracy_duration_ign1(void)
{
    initialiseSchedulers();
    ignitionSchedule1.pStartCallback = startCallback;
    ignitionSchedule1.pEndCallback = endCallback;
    setIgnitionSchedule1(TIMEOUT, DURATION);
    while( (ignitionSchedule1.Status == PENDING) || (ignitionSchedule1.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_UINT32_WITHIN(DELTA, DURATION, end_time - start_time);
}

void test_accuracy_duration_ign2(void)
{
    initialiseSchedulers();
    ignitionSchedule2.pStartCallback = startCallback;
    ignitionSchedule2.pEndCallback = endCallback;
    setIgnitionSchedule2(TIMEOUT, DURATION);
    while( (ignitionSchedule2.Status == PENDING) || (ignitionSchedule2.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_UINT32_WITHIN(DELTA, TIMEOUT, end_time - start_time);
}

void test_accuracy_duration_ign3(void)
{
    initialiseSchedulers();
    ignitionSchedule3.pStartCallback = startCallback;
    ignitionSchedule3.pEndCallback = endCallback;
    setIgnitionSchedule3(TIMEOUT, DURATION);
    while( (ignitionSchedule3.Status == PENDING) || (ignitionSchedule3.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_UINT32_WITHIN(DELTA, TIMEOUT, end_time - start_time);
}

void test_accuracy_duration_ign4(void)
{
    initialiseSchedulers();
    ignitionSchedule4.pStartCallback = startCallback;
    ignitionSchedule4.pEndCallback = endCallback;
    setIgnitionSchedule4(TIMEOUT, DURATION);
    while( (ignitionSchedule4.Status == PENDING) || (ignitionSchedule4.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_UINT32_WITHIN(DELTA, TIMEOUT, end_time - start_time);
}

void test_accuracy_duration_ign5(void)
{
#if IGN_CHANNELS >= 5
    initialiseSchedulers();
    ignitionSchedule5.pStartCallback = startCallback;
    ignitionSchedule5.pEndCallback = endCallback;
    setIgnitionSchedule5(TIMEOUT, DURATION);
    while( (ignitionSchedule5.Status == PENDING) || (ignitionSchedule5.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_UINT32_WITHIN(DELTA, TIMEOUT, end_time - start_time);
#endif
}

#if INJ_CHANNELS >= 6
void test_accuracy_duration_ign6(void)
{
    initialiseSchedulers();
    ignitionSchedule6.pStartCallback = startCallback;
    ignitionSchedule6.pEndCallback = endCallback;
    setIgnitionSchedule6(TIMEOUT, DURATION);
    while( (ignitionSchedule6.Status == PENDING) || (ignitionSchedule6.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_UINT32_WITHIN(DELTA, TIMEOUT, end_time - start_time);
}
#endif

#if INJ_CHANNELS >= 7
void test_accuracy_duration_ign7(void)
{
    initialiseSchedulers();
    ignitionSchedule7.pStartCallback = startCallback;
    ignitionSchedule7.pEndCallback = endCallback;
    setIgnitionSchedule7(TIMEOUT, DURATION);
    while( (ignitionSchedule7.Status == PENDING) || (ignitionSchedule7.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_UINT32_WITHIN(DELTA, TIMEOUT, end_time - start_time);
}
#endif

#if INJ_CHANNELS >= 8
void test_accuracy_duration_ign8(void)
{
    initialiseSchedulers();
    ignitionSchedule8.pStartCallback = startCallback;
    ignitionSchedule8.pEndCallback = endCallback;
    setIgnitionSchedule8(TIMEOUT, DURATION);
    while( (ignitionSchedule8.Status == PENDING) || (ignitionSchedule8.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_UINT32_WITHIN(DELTA, TIMEOUT, end_time - start_time);
}
#endif

void test_accuracy_duration(void)
{
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
