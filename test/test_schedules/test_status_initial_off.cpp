
#include <Arduino.h>
#include <unity.h>

#include "scheduler.h"

void test_status_initial_off_inj1(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule1.Status);
}

void test_status_initial_off_inj2(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule2.Status);
}

void test_status_initial_off_inj3(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule3.Status);
}

void test_status_initial_off_inj4(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule4.Status);
}

#if INJ_CHANNELS >= 5
void test_status_initial_off_inj5(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule5.Status);
}
#endif 

#if INJ_CHANNELS >= 6
void test_status_initial_off_inj6(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule6.Status);
}
#endif 

#if INJ_CHANNELS >= 7
void test_status_initial_off_inj7(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule7.Status);
}
#endif 

#if INJ_CHANNELS >= 8
void test_status_initial_off_inj8(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, fuelSchedule8.Status);
}
#endif 


void test_status_initial_off_ign1(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule1.Status);
}

void test_status_initial_off_ign2(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule2.Status);
}

void test_status_initial_off_ign3(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule3.Status);
}

void test_status_initial_off_ign4(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule4.Status);
}

#if IGN_CHANNELS >= 5
void test_status_initial_off_ign5(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule5.Status);
}
#endif

#if IGN_CHANNELS >= 6
void test_status_initial_off_ign6(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule6.Status);
}
#endif

#if IGN_CHANNELS >= 7
void test_status_initial_off_ign7(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule7.Status);
}
#endif

#if IGN_CHANNELS >= 8
void test_status_initial_off_ign8(void)
{
    initialiseSchedulers();
    TEST_ASSERT_EQUAL(OFF, ignitionSchedule8.Status);
}
#endif

void test_status_initial_off(void)
{
    RUN_TEST(test_status_initial_off_inj1);
    RUN_TEST(test_status_initial_off_inj2);
    RUN_TEST(test_status_initial_off_inj3);
    RUN_TEST(test_status_initial_off_inj4);
#if INJ_CHANNELS >= 5
    RUN_TEST(test_status_initial_off_inj5);
#endif 
#if INJ_CHANNELS >= 6
    RUN_TEST(test_status_initial_off_inj6);
#endif 
#if INJ_CHANNELS >= 7
    RUN_TEST(test_status_initial_off_inj7);
#endif 
#if INJ_CHANNELS >= 8
    RUN_TEST(test_status_initial_off_inj8);
#endif 

    RUN_TEST(test_status_initial_off_ign1);
    RUN_TEST(test_status_initial_off_ign2);
    RUN_TEST(test_status_initial_off_ign3);
    RUN_TEST(test_status_initial_off_ign4);
#if IGN_CHANNELS >= 5
    RUN_TEST(test_status_initial_off_ign5);
#endif
#if IGN_CHANNELS >= 6
    RUN_TEST(test_status_initial_off_ign6);
#endif
#if IGN_CHANNELS >= 7
    RUN_TEST(test_status_initial_off_ign7);
#endif
#if IGN_CHANNELS >= 8
    RUN_TEST(test_status_initial_off_ign8);
#endif
}