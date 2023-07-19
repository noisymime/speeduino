#include <decoders.h>
#include <globals.h>
#include <unity.h>
#include "Nissan360.h"
#include "scheduler.h"
#include "schedule_calcs.h"

extern uint16_t ignition1EndTooth;
extern uint16_t ignition2EndTooth;
extern uint16_t ignition3EndTooth;
extern uint16_t ignition4EndTooth;

void test_nissan360_newIgn_12_trig0_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 12/1
    //Advance: 10
    //triggerAngle=0
    triggerSetup_Nissan360();
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = 0; //No trigger offset

    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle);    
    triggerSetEndTeeth_Nissan360();
    TEST_ASSERT_EQUAL(171, ignition1EndTooth);

    //Test again with 0 degrees advance
    calculateIgnitionAngle(5, 0, 0, &ignition1EndAngle, &ignition1StartAngle); 
    triggerSetEndTeeth_Nissan360();
    TEST_ASSERT_EQUAL(176, ignition1EndTooth);

    //Test again with 35 degrees advance
    calculateIgnitionAngle(5, 0, 35, &ignition1EndAngle, &ignition1StartAngle); 
    triggerSetEndTeeth_Nissan360();
    TEST_ASSERT_EQUAL(158, ignition1EndTooth);
}

void test_nissan360_newIgn_12_trig90_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 12/1
    //Advance: 10
    //triggerAngle=90
    triggerSetup_Nissan360();
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = 90; //No trigger offset

    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle);    
    triggerSetEndTeeth_Nissan360();
    TEST_ASSERT_EQUAL(126, ignition1EndTooth);
}

void test_nissan360_newIgn_12_trig180_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=180
    triggerSetup_Nissan360();
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = 180; //No trigger offset

    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle);    
    triggerSetEndTeeth_Nissan360();
    TEST_ASSERT_EQUAL(81, ignition1EndTooth);
}

void test_nissan360_newIgn_12_trig270_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=270
    triggerSetup_Nissan360();
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = 270; //No trigger offset

    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle);  
    triggerSetEndTeeth_Nissan360();
    TEST_ASSERT_EQUAL(36, ignition1EndTooth);
}

void test_nissan360_newIgn_12_trig360_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=360
    triggerSetup_Nissan360();
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = 360; //No trigger offset

    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle); 
    triggerSetEndTeeth_Nissan360();
    TEST_ASSERT_EQUAL(351, ignition1EndTooth);
}

void test_nissan360_newIgn_12_trigNeg90_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-90
    triggerSetup_Nissan360();
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = -90; //No trigger offset

    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle); 
    triggerSetEndTeeth_Nissan360();
    TEST_ASSERT_EQUAL(216, ignition1EndTooth);
}

void test_nissan360_newIgn_12_trigNeg180_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-180
    triggerSetup_Nissan360();
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = -180; //No trigger offset

    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle); 
    triggerSetEndTeeth_Nissan360();
    TEST_ASSERT_EQUAL(261, ignition1EndTooth);
}

void test_nissan360_newIgn_12_trigNeg270_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-270
    triggerSetup_Nissan360();
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = -270; //No trigger offset
    
    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle); 
    triggerSetEndTeeth_Nissan360();
    TEST_ASSERT_EQUAL(306, ignition1EndTooth);
}

void test_nissan360_newIgn_12_trigNeg360_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-360
    triggerSetup_Nissan360();
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = -360; //No trigger offset

    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle); 
    triggerSetEndTeeth_Nissan360();
    TEST_ASSERT_EQUAL(351, ignition1EndTooth);
}

void testNissan360()
{
    RUN_TEST(test_nissan360_newIgn_12_trig0_1);
    RUN_TEST(test_nissan360_newIgn_12_trig90_1);
    RUN_TEST(test_nissan360_newIgn_12_trig180_1);
    RUN_TEST(test_nissan360_newIgn_12_trig270_1);
    RUN_TEST(test_nissan360_newIgn_12_trig360_1);
    RUN_TEST(test_nissan360_newIgn_12_trigNeg90_1);
    RUN_TEST(test_nissan360_newIgn_12_trigNeg180_1);
    RUN_TEST(test_nissan360_newIgn_12_trigNeg270_1);
    RUN_TEST(test_nissan360_newIgn_12_trigNeg360_1);
}