#include <decoders.h>
#include <globals.h>
#include <unity.h>
#include "ThirtySixMinus21.h"
#include "schedule_calcs.h"

void test_setup_36_2_1()
{
    configPage4.triggerAngle = 7;

    triggerSetup_ThirtySixMinus21();
}

//************************************** Begin the new ignition setEndTooth tests **************************************

void test_ThirtySixMinus21_trigAng_adv0()
{
    test_setup_36_2_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 0; // Set 0 degrees advance

    triggerSetEndTeeth_ThirtySixMinus21();
    TEST_ASSERT_EQUAL(34, ignition1EndTooth);
    TEST_ASSERT_EQUAL(15, ignition2EndTooth);
}

void test_ThirtySixMinus21_trigAng_adv10()
{
    // Test the set end tooth function. Conditions:
    // Trigger pattern: 36-2-1
    // Advance: 10
    // Trigger angle: 90
    test_setup_36_2_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; // Set 10 degrees advance

    triggerSetEndTeeth_ThirtySixMinus21();
    test_setup_36_2_1();
    TEST_ASSERT_EQUAL(33, ignition1EndTooth);
    TEST_ASSERT_EQUAL(15, ignition2EndTooth);
}

void test_ThirtySixMinus21_trigAng_adv20()
{
    test_setup_36_2_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 20; // Set 20 degrees advance

    triggerSetEndTeeth_ThirtySixMinus21();
    test_setup_36_2_1();
    TEST_ASSERT_EQUAL(32, ignition1EndTooth);
    TEST_ASSERT_EQUAL(15, ignition2EndTooth);
}

void test_ThirtySixMinus21_trigAng_adv30()
{
    test_setup_36_2_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 30; // Set 30 degrees advance

    triggerSetEndTeeth_ThirtySixMinus21();
    test_setup_36_2_1();
    TEST_ASSERT_EQUAL(31, ignition1EndTooth);
    TEST_ASSERT_EQUAL(15, ignition2EndTooth);
}

void test_ThirtySixMinus21_trigAng_adv40()
{
    test_setup_36_2_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 40; // Set 40 degrees advance

    triggerSetEndTeeth_ThirtySixMinus21();
    test_setup_36_2_1();
    TEST_ASSERT_EQUAL(30, ignition1EndTooth);
    TEST_ASSERT_EQUAL(15, ignition2EndTooth);
}

void test_ThirtySixMinus21_trigAng_adv50()
{
    test_setup_36_2_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 50; // Set 50 degrees advance

    triggerSetEndTeeth_ThirtySixMinus21();
    test_setup_36_2_1();
    TEST_ASSERT_EQUAL(29, ignition1EndTooth);
    TEST_ASSERT_EQUAL(15, ignition2EndTooth);
}

void testThirtySixMinus21()
{
    // Add here the tests to be run
    // RUN_TEST(function_name);
    RUN_TEST(test_ThirtySixMinus21_trigAng_adv0);
    RUN_TEST(test_ThirtySixMinus21_trigAng_adv10);
    RUN_TEST(test_ThirtySixMinus21_trigAng_adv20);
    RUN_TEST(test_ThirtySixMinus21_trigAng_adv30);
    RUN_TEST(test_ThirtySixMinus21_trigAng_adv40);
    RUN_TEST(test_ThirtySixMinus21_trigAng_adv50);
}