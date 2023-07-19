#include <decoders.h>
#include <globals.h>
#include <unity.h>
#include "test_ngc.h"
#include "scheduler.h"
#include "schedule_calcs.h"

extern uint16_t ignition1EndTooth;
extern uint16_t ignition2EndTooth;
extern uint16_t ignition3EndTooth;
extern uint16_t ignition4EndTooth;

void test_ngc_newIgn_12_trig0_1()
{
    triggerSetup_NGC();
    CRANK_ANGLE_MAX_IGN = 360;
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = 0; //No trigger offset

    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle);    
    triggerSetEndTeeth_NGC();
    TEST_ASSERT_EQUAL(34, ignition1EndTooth);

    //Test again with 0 degrees advance
    calculateIgnitionAngle(5, 0, 0, &ignition1EndAngle, &ignition1StartAngle); 
    triggerSetEndTeeth_NGC();
    TEST_ASSERT_EQUAL(34, ignition1EndTooth);

    //Test again with 35 degrees advance
    calculateIgnitionAngle(5, 0, 35, &ignition1EndAngle, &ignition1StartAngle); 
    triggerSetEndTeeth_NGC();
    TEST_ASSERT_EQUAL(31, ignition1EndTooth);
}

void test_ngc_newIgn_12_trig90_1()
{
    triggerSetup_NGC();
    CRANK_ANGLE_MAX_IGN = 360;
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = 90;

    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle);    
    triggerSetEndTeeth_NGC();
    TEST_ASSERT_EQUAL(25, ignition1EndTooth);
}

void test_ngc_newIgn_12_trig180_1()
{
    triggerSetup_NGC();
    CRANK_ANGLE_MAX_IGN = 360;
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = 180;

    currentStatus.advance = 10; //Set 10deg advance
    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle); 

    
    triggerSetEndTeeth_NGC();
    TEST_ASSERT_EQUAL(16, ignition1EndTooth);
}

void test_ngc_newIgn_12_trig270_1()
{
    triggerSetup_NGC();
    CRANK_ANGLE_MAX_IGN = 360;
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = 270;

    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle);     
    triggerSetEndTeeth_NGC();
    TEST_ASSERT_EQUAL(7, ignition1EndTooth);
}

void test_ngc_newIgn_12_trig360_1()
{
    triggerSetup_NGC();
    CRANK_ANGLE_MAX_IGN = 360;
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = 360;

    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle);     
    triggerSetEndTeeth_NGC();
    TEST_ASSERT_EQUAL(34, ignition1EndTooth);
}

void test_ngc_newIgn_12_trigNeg90_1()
{
    triggerSetup_NGC();
    CRANK_ANGLE_MAX_IGN = 360;
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = -90;
    
    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle); 
    triggerSetEndTeeth_NGC();
    TEST_ASSERT_EQUAL(7, ignition1EndTooth);
}

void test_ngc_newIgn_12_trigNeg180_1()
{
    triggerSetup_NGC();
    CRANK_ANGLE_MAX_IGN = 360;
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = -180;

    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle);   
    triggerSetEndTeeth_NGC();
    TEST_ASSERT_EQUAL(16, ignition1EndTooth);
}

void test_ngc_newIgn_12_trigNeg270_1()
{
    triggerSetup_NGC();
    CRANK_ANGLE_MAX_IGN = 360;
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = -270;

    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle);   
    triggerSetEndTeeth_NGC();
    TEST_ASSERT_EQUAL(25, ignition1EndTooth);
}

void test_ngc_newIgn_12_trigNeg360_1()
{
    triggerSetup_NGC();
    CRANK_ANGLE_MAX_IGN = 360;
    configPage4.sparkMode = IGN_MODE_WASTED;
    configPage4.triggerAngle = -360;
    
    calculateIgnitionAngle(5, 0, 10, &ignition1EndAngle, &ignition1StartAngle);     
    triggerSetEndTeeth_NGC();
    TEST_ASSERT_EQUAL(34, ignition1EndTooth);
}

void testNGC()
{
    RUN_TEST(test_ngc_newIgn_12_trig0_1);
    RUN_TEST(test_ngc_newIgn_12_trig90_1);
    RUN_TEST(test_ngc_newIgn_12_trig180_1);
    RUN_TEST(test_ngc_newIgn_12_trig270_1);
    RUN_TEST(test_ngc_newIgn_12_trig360_1);
    RUN_TEST(test_ngc_newIgn_12_trigNeg90_1);
    RUN_TEST(test_ngc_newIgn_12_trigNeg180_1);
    RUN_TEST(test_ngc_newIgn_12_trigNeg270_1);
    RUN_TEST(test_ngc_newIgn_12_trigNeg360_1);
}