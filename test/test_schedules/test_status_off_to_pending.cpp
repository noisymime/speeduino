
#include <Arduino.h>
#include <unity.h>
#include "../test_utils.h"
#include "scheduler.h"

#define TIMEOUT 1000
#define DURATION 1000

static void emptyCallback(void) {  }

void test_status_off_to_pending_inj1(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule1, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule1.Status);
}

void test_status_off_to_pending_inj2(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule2, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule2.Status);
}

void test_status_off_to_pending_inj3(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule3, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule3.Status);
}

void test_status_off_to_pending_inj4(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule4, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule4.Status);
}

#if INJ_CHANNELS >= 5
void test_status_off_to_pending_inj5(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule5, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule5.Status);
}
#endif

#if INJ_CHANNELS >= 6
void test_status_off_to_pending_inj6(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule6, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule6.Status);
}
#endif

#if INJ_CHANNELS >= 7
void test_status_off_to_pending_inj7(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule7, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule7.Status);
}
#endif

#if INJ_CHANNELS >= 8
void test_status_off_to_pending_inj8(void)
{
    initialiseSchedulers();
    setFuelSchedule(fuelSchedule8, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, fuelSchedule8.Status);
}
#endif


void test_status_off_to_pending_ign1(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule1, emptyCallback, emptyCallback);
    setIgnitionSchedule(ignitionSchedule1, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule1.Status);
}

void test_status_off_to_pending_ign2(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule2, emptyCallback, emptyCallback);
    setIgnitionSchedule(ignitionSchedule2, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule2.Status);
}

void test_status_off_to_pending_ign3(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule3, emptyCallback, emptyCallback);
    setIgnitionSchedule(ignitionSchedule3, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule3.Status);
}

void test_status_off_to_pending_ign4(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule4, emptyCallback, emptyCallback);
    setIgnitionSchedule(ignitionSchedule4, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule4.Status);
}

#if IGN_CHANNELS >= 5
void test_status_off_to_pending_ign5(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule5, emptyCallback, emptyCallback);
    setIgnitionSchedule(ignitionSchedule5, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule5.Status);
}
#endif

#if IGN_CHANNELS >= 6
void test_status_off_to_pending_ign6(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule6, emptyCallback, emptyCallback);
    setIgnitionSchedule(ignitionSchedule6, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule6.Status);
}
#endif

#if IGN_CHANNELS >= 7
void test_status_off_to_pending_ign7(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule7, emptyCallback, emptyCallback);
    setIgnitionSchedule(ignitionSchedule7, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule7.Status);
}
#endif

#if IGN_CHANNELS >= 8
void test_status_off_to_pending_ign8(void)
{
    initialiseSchedulers();
    setCallbacks(ignitionSchedule8, emptyCallback, emptyCallback);
    setIgnitionSchedule(ignitionSchedule8, TIMEOUT, DURATION);
    TEST_ASSERT_EQUAL(PENDING, ignitionSchedule8.Status);
}
#endif

void test_status_off_to_pending(void)
{
  SET_UNITY_FILENAME() {

    RUN_TEST(test_status_off_to_pending_inj1);
    RUN_TEST(test_status_off_to_pending_inj2);
    RUN_TEST(test_status_off_to_pending_inj3);
    RUN_TEST(test_status_off_to_pending_inj4);
#if INJ_CHANNELS >= 5
    RUN_TEST(test_status_off_to_pending_inj5);
#endif
#if INJ_CHANNELS >= 6
    RUN_TEST(test_status_off_to_pending_inj6);
#endif
#if INJ_CHANNELS >= 7
    RUN_TEST(test_status_off_to_pending_inj7);
#endif
#if INJ_CHANNELS >= 8
    RUN_TEST(test_status_off_to_pending_inj8);
#endif

    RUN_TEST(test_status_off_to_pending_ign1);
    RUN_TEST(test_status_off_to_pending_ign2);
    RUN_TEST(test_status_off_to_pending_ign3);
    RUN_TEST(test_status_off_to_pending_ign4);
#if IGN_CHANNELS >= 5    
    RUN_TEST(test_status_off_to_pending_ign5);
#endif
#if IGN_CHANNELS >= 6
    RUN_TEST(test_status_off_to_pending_ign6);
#endif
#if IGN_CHANNELS >= 7
    RUN_TEST(test_status_off_to_pending_ign7);
#endif
#if IGN_CHANNELS >= 8
    RUN_TEST(test_status_off_to_pending_ign8);
#endif
  }
}
