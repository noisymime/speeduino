
#include <Arduino.h>
#include <unity.h>

#include "scheduler.h"

#define TIMEOUT 1000
#define DURATION 1000

static void emptyCallback(void) {  }

void test_status_running_to_off_inj1(void)
{
    initialiseSchedulers();
    setFuelSchedule1(TIMEOUT, DURATION);
    while( (fuelSchedule1.Status == PENDING) || (fuelSchedule1.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, fuelSchedule1.Status);
}

void test_status_running_to_off_inj2(void)
{
    initialiseSchedulers();
    setFuelSchedule2(TIMEOUT, DURATION);
    while( (fuelSchedule2.Status == PENDING) || (fuelSchedule2.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, fuelSchedule2.Status);
}

void test_status_running_to_off_inj3(void)
{
    initialiseSchedulers();
    setFuelSchedule3(TIMEOUT, DURATION);
    while( (fuelSchedule3.Status == PENDING) || (fuelSchedule3.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, fuelSchedule3.Status);
}

void test_status_running_to_off_inj4(void)
{
    initialiseSchedulers();
    setFuelSchedule4(TIMEOUT, DURATION);
    while( (fuelSchedule4.Status == PENDING) || (fuelSchedule4.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, fuelSchedule4.Status);
}

void test_status_running_to_off_inj5(void)
{
#if INJ_CHANNELS >= 5
    initialiseSchedulers();
    setFuelSchedule5(TIMEOUT, DURATION);
    while( (fuelSchedule5.Status == PENDING) || (fuelSchedule5.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, fuelSchedule5.Status);
#endif
}

void test_status_running_to_off_inj6(void)
{
#if INJ_CHANNELS >= 6
    initialiseSchedulers();
    setFuelSchedule6(TIMEOUT, DURATION);
    while( (fuelSchedule6.Status == PENDING) || (fuelSchedule6.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, fuelSchedule6.Status);
#endif
}

void test_status_running_to_off_inj7(void)
{
#if INJ_CHANNELS >= 7
    initialiseSchedulers();
    setFuelSchedule7(TIMEOUT, DURATION);
    while( (fuelSchedule7.Status == PENDING) || (fuelSchedule7.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, fuelSchedule7.Status);
#endif
}

void test_status_running_to_off_inj8(void)
{
#if INJ_CHANNELS >= 8
    initialiseSchedulers();
    setFuelSchedule8(TIMEOUT, DURATION);
    while( (fuelSchedule8.Status == PENDING) || (fuelSchedule8.Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, fuelSchedule8.Status);
#endif
}


void test_status_running_to_off_ign1(void)
{
    initialiseSchedulers();
    setIgnitionSchedule1(emptyCallback, TIMEOUT, DURATION, emptyCallback);
    while( (ignitionSchedule[0].Status == PENDING) || (ignitionSchedule[0].Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule[0].Status);
}

void test_status_running_to_off_ign2(void)
{
    initialiseSchedulers();
    setIgnitionSchedule2(emptyCallback, TIMEOUT, DURATION, emptyCallback);
    while( (ignitionSchedule[1].Status == PENDING) || (ignitionSchedule[1].Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule[1].Status);
}

void test_status_running_to_off_ign3(void)
{
    initialiseSchedulers();
    setIgnitionSchedule3(emptyCallback, TIMEOUT, DURATION, emptyCallback);
    while( (ignitionSchedule[2].Status == PENDING) || (ignitionSchedule[2].Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule[2].Status);
}

void test_status_running_to_off_ign4(void)
{
    initialiseSchedulers();
    setIgnitionSchedule4(emptyCallback, TIMEOUT, DURATION, emptyCallback);
    while( (ignitionSchedule[3].Status == PENDING) || (ignitionSchedule[3].Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule[3].Status);
}

void test_status_running_to_off_ign5(void)
{
#if IGN_CHANNELS >= 5
    initialiseSchedulers();
    setIgnitionSchedule5(emptyCallback, TIMEOUT, DURATION, emptyCallback);
    while( (ignitionSchedule[4].Status == PENDING) || (ignitionSchedule[4].Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule[4].Status);
#endif
}

void test_status_running_to_off_ign6(void)
{
#if IGN_CHANNELS >= 6
    initialiseSchedulers();
    setIgnitionSchedule6(emptyCallback, TIMEOUT, DURATION, emptyCallback);
    while( (ignitionSchedule[5].Status == PENDING) || (ignitionSchedule[5].Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule[5].Status);
#endif
}

void test_status_running_to_off_ign7(void)
{
#if IGN_CHANNELS >= 7
    initialiseSchedulers();
    setIgnitionSchedule7(emptyCallback, TIMEOUT, DURATION, emptyCallback);
    while( (ignitionSchedule[6].Status == PENDING) || (ignitionSchedule[6].Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule[6].Status);
#endif
}

void test_status_running_to_off_ign8(void)
{
#if IGN_CHANNELS >= 8
    initialiseSchedulers();
    setIgnitionSchedule8(emptyCallback, TIMEOUT, DURATION, emptyCallback);
    while( (ignitionSchedule[7].Status == PENDING) || (ignitionSchedule[7].Status == RUNNING) ) /*Wait*/ ;
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule[7].Status);
#endif
}

void test_status_running_to_off(void)
{
    RUN_TEST(test_status_running_to_off_inj1);
    RUN_TEST(test_status_running_to_off_inj2);
    RUN_TEST(test_status_running_to_off_inj3);
    RUN_TEST(test_status_running_to_off_inj4);
    RUN_TEST(test_status_running_to_off_inj5);
    RUN_TEST(test_status_running_to_off_inj6);
    RUN_TEST(test_status_running_to_off_inj7);
    RUN_TEST(test_status_running_to_off_inj8);

    RUN_TEST(test_status_running_to_off_ign1);
    RUN_TEST(test_status_running_to_off_ign2);
    RUN_TEST(test_status_running_to_off_ign3);
    RUN_TEST(test_status_running_to_off_ign4);
    RUN_TEST(test_status_running_to_off_ign5);
    RUN_TEST(test_status_running_to_off_ign6);
    RUN_TEST(test_status_running_to_off_ign7);
    RUN_TEST(test_status_running_to_off_ign8);
}
