
#include <Arduino.h>
#include <unity.h>
#include "../test_utils.h"
#include "scheduler.h"

#define TIMEOUT 1000
#define DURATION 1000

static void emptyCallback(void) {  }

void test_status_running_to_pending_inj1(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule1, TIMEOUT, DURATION);
    while(fuelSchedule1.Status == PENDING) /*Wait*/ ;
    setFuelSchedule(fuelSchedule1, 2*TIMEOUT, DURATION);
    while(fuelSchedule1.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule1.Status);
}

void test_status_running_to_pending_inj2(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule2, TIMEOUT, DURATION);
    while(fuelSchedule2.Status == PENDING) /*Wait*/ ;
    setFuelSchedule(fuelSchedule2, 2*TIMEOUT, DURATION);
    while(fuelSchedule2.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule2.Status);
}

void test_status_running_to_pending_inj3(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule3, TIMEOUT, DURATION);
    while(fuelSchedule3.Status == PENDING) /*Wait*/ ;
    setFuelSchedule(fuelSchedule3, 2*TIMEOUT, DURATION);
    while(fuelSchedule3.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule3.Status);
}

void test_status_running_to_pending_inj4(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule4, TIMEOUT, DURATION);
    while(fuelSchedule4.Status == PENDING) /*Wait*/ ;
    setFuelSchedule(fuelSchedule4, 2*TIMEOUT, DURATION);
    while(fuelSchedule4.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule4.Status);
}

void test_status_running_to_pending_inj5(void)
{
#if INJ_CHANNELS >= 5
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule5, TIMEOUT, DURATION);
    while(fuelSchedule5.Status == PENDING) /*Wait*/ ;
    setFuelSchedule(fuelSchedule5, 2*TIMEOUT, DURATION);
    while(fuelSchedule5.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule5.Status);
#endif
}

void test_status_running_to_pending_inj6(void)
{
#if INJ_CHANNELS >= 6
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule6, TIMEOUT, DURATION);
    while(fuelSchedule6.Status == PENDING) /*Wait*/ ;
    setFuelSchedule(fuelSchedule6, 2*TIMEOUT, DURATION);
    while(fuelSchedule6.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule6.Status);
#endif
}

void test_status_running_to_pending_inj7(void)
{
#if INJ_CHANNELS >= 7
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule7, TIMEOUT, DURATION);
    while(fuelSchedule7.Status == PENDING) /*Wait*/ ;
    setFuelSchedule(fuelSchedule7, 2*TIMEOUT, DURATION);
    while(fuelSchedule7.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule7.Status);
#endif
}

void test_status_running_to_pending_inj8(void)
{
#if INJ_CHANNELS >= 8
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule8, TIMEOUT, DURATION);
    while(fuelSchedule8.Status == PENDING) /*Wait*/ ;
    setFuelSchedule(fuelSchedule8, 2*TIMEOUT, DURATION);
    while(fuelSchedule8.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule8.Status);
#endif
}


void test_status_running_to_pending_ign1(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule1, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule1, TIMEOUT, DURATION);
    while(ignitionSchedule1.Status == PENDING) /*Wait*/ ;
    setIgnitionSchedule(ignitionSchedule1, 2*TIMEOUT, DURATION);
    while(ignitionSchedule1.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule1.Status);
}

void test_status_running_to_pending_ign2(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule2, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule2, TIMEOUT, DURATION);
    while(ignitionSchedule2.Status == PENDING) /*Wait*/ ;
    setIgnitionSchedule(ignitionSchedule2, 2*TIMEOUT, DURATION);
    while(ignitionSchedule2.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule2.Status);
}

void test_status_running_to_pending_ign3(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule3, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule3, TIMEOUT, DURATION);
    while(ignitionSchedule3.Status == PENDING) /*Wait*/ ;
    setIgnitionSchedule(ignitionSchedule3, 2*TIMEOUT, DURATION);
    while(ignitionSchedule3.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule3.Status);
}

void test_status_running_to_pending_ign4(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule4, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule4, TIMEOUT, DURATION);
    while(ignitionSchedule4.Status == PENDING) /*Wait*/ ;
    setIgnitionSchedule(ignitionSchedule4, 2*TIMEOUT, DURATION);
    while(ignitionSchedule4.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule4.Status);
}

void test_status_running_to_pending_ign5(void)
{
#if IGN_CHANNELS >= 5
    initialiseSchedulers();
    setCallbacks(ignitionSchedule5, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule5, TIMEOUT, DURATION);
    while(ignitionSchedule5.Status == PENDING) /*Wait*/ ;
    setIgnitionSchedule(ignitionSchedule5, 2*TIMEOUT, DURATION);
    while(ignitionSchedule5.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule5.Status);
#endif
}

void test_status_running_to_pending_ign6(void)
{
#if INJ_CHANNELS >= 6
    initialiseSchedulers();
    setCallbacks(ignitionSchedule6, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule6, TIMEOUT, DURATION);
    while(ignitionSchedule6.Status == PENDING) /*Wait*/ ;
    setIgnitionSchedule(ignitionSchedule6, 2*TIMEOUT, DURATION);
    while(ignitionSchedule6.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule6.Status);
#endif
}

void test_status_running_to_pending_ign7(void)
{
#if INJ_CHANNELS >= 7
    initialiseSchedulers();
    setCallbacks(ignitionSchedule7, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule7, TIMEOUT, DURATION);
    while(ignitionSchedule7.Status == PENDING) /*Wait*/ ;
    setIgnitionSchedule(ignitionSchedule7, 2*TIMEOUT, DURATION);
    while(ignitionSchedule7.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule7.Status);
#endif
}

void test_status_running_to_pending_ign8(void)
{
#if INJ_CHANNELS >= 8
    initialiseSchedulers();
    setCallbacks(ignitionSchedule8, emptyCallback, emptyCallback);
    
    setIgnitionSchedule(ignitionSchedule8, TIMEOUT, DURATION);
    while(ignitionSchedule8.Status == PENDING) /*Wait*/ ;
    setIgnitionSchedule(ignitionSchedule8, 2*TIMEOUT, DURATION);
    while(ignitionSchedule8.Status == RUNNING) /*Wait*/ ;
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule8.Status);
#endif
}

void test_status_running_to_pending(void)
{
  SET_UNITY_FILENAME() {

    RUN_TEST(test_status_running_to_pending_inj1);
    RUN_TEST(test_status_running_to_pending_inj2);
    RUN_TEST(test_status_running_to_pending_inj3);
    RUN_TEST(test_status_running_to_pending_inj4);
    RUN_TEST(test_status_running_to_pending_inj5);
    RUN_TEST(test_status_running_to_pending_inj6);
    RUN_TEST(test_status_running_to_pending_inj7);
    RUN_TEST(test_status_running_to_pending_inj8);

    RUN_TEST(test_status_running_to_pending_ign1);
    RUN_TEST(test_status_running_to_pending_ign2);
    RUN_TEST(test_status_running_to_pending_ign3);
    RUN_TEST(test_status_running_to_pending_ign4);
    RUN_TEST(test_status_running_to_pending_ign5);
    RUN_TEST(test_status_running_to_pending_ign6);
    RUN_TEST(test_status_running_to_pending_ign7);
    RUN_TEST(test_status_running_to_pending_ign8);
  }
}
