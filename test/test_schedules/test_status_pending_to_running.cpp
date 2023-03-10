
#include <Arduino.h>
#include <unity.h>
#include "../test_utils.h"
#include "scheduler.h"

#define TIMEOUT 1000
#define DURATION 1000

static void emptyCallback(void) {  }

void test_status_pending_to_running_inj1(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule1, TIMEOUT, DURATION);
    while(fuelSchedule1.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, fuelSchedule1.Status);
}

void test_status_pending_to_running_inj2(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule2, TIMEOUT, DURATION);
    while(fuelSchedule2.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, fuelSchedule2.Status);
}

void test_status_pending_to_running_inj3(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule3, TIMEOUT, DURATION);
    while(fuelSchedule3.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, fuelSchedule3.Status);
}

void test_status_pending_to_running_inj4(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule4, TIMEOUT, DURATION);
    while(fuelSchedule4.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, fuelSchedule4.Status);
}

void test_status_pending_to_running_inj5(void)
{
#if INJ_CHANNELS >= 5
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule5, TIMEOUT, DURATION);
    while(fuelSchedule5.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, fuelSchedule5.Status);
#endif
}

void test_status_pending_to_running_inj6(void)
{
#if INJ_CHANNELS >= 6
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule6, TIMEOUT, DURATION);
    while(fuelSchedule6.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, fuelSchedule6.Status);
#endif
}

void test_status_pending_to_running_inj7(void)
{
#if INJ_CHANNELS >= 7
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule7, TIMEOUT, DURATION);
    while(fuelSchedule7.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, fuelSchedule7.Status);
#endif
}

void test_status_pending_to_running_inj8(void)
{
#if INJ_CHANNELS >= 8
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule8, TIMEOUT, DURATION);
    while(fuelSchedule8.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, fuelSchedule8.Status);
#endif
}


void test_status_pending_to_running_ign1(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule1, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule1, TIMEOUT, DURATION);
    while(ignitionSchedule1.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, ignitionSchedule1.Status);
}

void test_status_pending_to_running_ign2(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule2, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule2, TIMEOUT, DURATION);
    while(ignitionSchedule2.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, ignitionSchedule2.Status);
}

void test_status_pending_to_running_ign3(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule3, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule3, TIMEOUT, DURATION);
    while(ignitionSchedule3.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, ignitionSchedule3.Status);
}

void test_status_pending_to_running_ign4(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule4, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule4, TIMEOUT, DURATION);
    while(ignitionSchedule4.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, ignitionSchedule4.Status);
}

void test_status_pending_to_running_ign5(void)
{
#if IGN_CHANNELS >= 5
    initialiseSchedulers();
    setCallbacks(ignitionSchedule5, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule5, TIMEOUT, DURATION);
    while(ignitionSchedule5.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, ignitionSchedule5.Status);
#endif
}

void test_status_pending_to_running_ign6(void)
{
#if INJ_CHANNELS >= 6
    initialiseSchedulers();
    setCallbacks(ignitionSchedule6, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule6, TIMEOUT, DURATION);
    while(ignitionSchedule6.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, ignitionSchedule6.Status);
#endif
}

void test_status_pending_to_running_ign7(void)
{
#if INJ_CHANNELS >= 7
    initialiseSchedulers();
    setCallbacks(ignitionSchedule7, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule7, TIMEOUT, DURATION);
    while(ignitionSchedule7.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, ignitionSchedule7.Status);
#endif
}

void test_status_pending_to_running_ign8(void)
{
#if INJ_CHANNELS >= 8
    initialiseSchedulers();
    setCallbacks(ignitionSchedule8, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule8, TIMEOUT, DURATION);
    while(ignitionSchedule8.Status == PENDING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(RUNNING, ignitionSchedule8.Status);
#endif
}

void test_status_pending_to_running(void)
{
  SET_UNITY_FILENAME() {

    RUN_TEST(test_status_pending_to_running_inj1);
    RUN_TEST(test_status_pending_to_running_inj2);
    RUN_TEST(test_status_pending_to_running_inj3);
    RUN_TEST(test_status_pending_to_running_inj4);
    RUN_TEST(test_status_pending_to_running_inj5);
    RUN_TEST(test_status_pending_to_running_inj6);
    RUN_TEST(test_status_pending_to_running_inj7);
    RUN_TEST(test_status_pending_to_running_inj8);

    RUN_TEST(test_status_pending_to_running_ign1);
    RUN_TEST(test_status_pending_to_running_ign2);
    RUN_TEST(test_status_pending_to_running_ign3);
    RUN_TEST(test_status_pending_to_running_ign4);
    RUN_TEST(test_status_pending_to_running_ign5);
    RUN_TEST(test_status_pending_to_running_ign6);
    RUN_TEST(test_status_pending_to_running_ign7);
    RUN_TEST(test_status_pending_to_running_ign8);
  }
}
