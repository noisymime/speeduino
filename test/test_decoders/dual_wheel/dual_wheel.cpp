#include <decoders.h>
#include <globals.h>
#include <unity.h>
#include "dual_wheel.h"
#include "schedule_calcs.h"


void test_setup_dualwheel_12_1()
{
    //Setup a 36-1 wheel
    configPage4.triggerTeeth = 12;
    //configPage4.triggerMissingTeeth = 1;
    configPage4.TrigSpeed = CRANK_SPEED;
    configPage4.trigPatternSec = SEC_TRIGGER_SINGLE;

    triggerSetup_missingTooth();
}

void test_setup_dualwheel_60_2()
{
    //Setup a 60-2 wheel
    configPage4.triggerTeeth = 60;
    configPage4.triggerMissingTeeth = 2;
    configPage4.TrigSpeed = CRANK_SPEED;
    configPage4.trigPatternSec = SEC_TRIGGER_SINGLE;

    triggerSetup_missingTooth();
}

//************************************** Begin the new ignition setEndTooth tests **************************************
void test_dualwheel_newIgn_12_1_trig0_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 12/1
    //Advance: 10
    //triggerAngle=0
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 0; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(11, ignition1EndTooth);

    //Test again with 0 degrees advance
    ignition1EndAngle = 360 - 0; //Set 0 degrees advance
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(12, ignition1EndTooth);

    //Test again with 35 degrees advance
    ignition1EndAngle = 360 - 35; //Set 35 degrees advance
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(10, ignition1EndTooth);
}

void test_dualwheel_newIgn_12_1_trig90_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 12/1
    //Advance: 10
    //triggerAngle=90
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 90; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(8, ignition1EndTooth);
}

void test_dualwheel_newIgn_12_1_trig180_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=180
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 180; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(5, ignition1EndTooth);
}

void test_dualwheel_newIgn_12_1_trig270_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=270
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 270; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(2, ignition1EndTooth);
}

void test_dualwheel_newIgn_12_1_trig360_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=360
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 360; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(12, ignition1EndTooth);
}

void test_dualwheel_newIgn_12_1_trigNeg90_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-90
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -90; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(2, ignition1EndTooth);
}

void test_dualwheel_newIgn_12_1_trigNeg180_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-180
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -180; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(5, ignition1EndTooth);
}

void test_dualwheel_newIgn_12_1_trigNeg270_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-270
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -270; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(8, ignition1EndTooth);
}

void test_dualwheel_newIgn_12_1_trigNeg360_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-360
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -360; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(11, ignition1EndTooth);
}

// ******* CHannel 2 *******
void test_dualwheel_newIgn_12_1_trig0_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=0
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 0; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(16, ignition2EndTooth);
}

void test_dualwheel_newIgn_12_1_trig90_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=90
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 90; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(7, ignition2EndTooth);
}

void test_dualwheel_newIgn_12_1_trig180_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=180
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 180; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(34, ignition2EndTooth);
}

void test_dualwheel_newIgn_12_1_trig270_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=270
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 270; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(25, ignition2EndTooth);
}

void test_dualwheel_newIgn_12_1_trig360_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=360
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 360; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(16, ignition2EndTooth);
}

void test_dualwheel_newIgn_12_1_trigNeg90_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-90
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -90; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(25, ignition2EndTooth);
}

void test_dualwheel_newIgn_12_1_trigNeg180_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-180
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -180; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(34, ignition2EndTooth);
}

void test_dualwheel_newIgn_12_1_trigNeg270_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-270
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -270; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(7, ignition2EndTooth);
}

void test_dualwheel_newIgn_12_1_trigNeg360_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-360
    test_setup_dualwheel_12_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -360; //No trigger offset
    
    triggerSetEndTeeth_DualWheel();
    TEST_ASSERT_EQUAL(16, ignition2EndTooth);
}

void test_dualwheel_newIgn_2()
{

}
void test_dualwheel_newIgn_3()
{

}
void test_dualwheel_newIgn_4()
{

}

void testDualWheel()
{
  RUN_TEST(test_dualwheel_newIgn_12_1_trig0_1);
  RUN_TEST(test_dualwheel_newIgn_12_1_trig90_1);
  RUN_TEST(test_dualwheel_newIgn_12_1_trig180_1);
  RUN_TEST(test_dualwheel_newIgn_12_1_trig270_1);
  RUN_TEST(test_dualwheel_newIgn_12_1_trig360_1);
  RUN_TEST(test_dualwheel_newIgn_12_1_trigNeg90_1);
  RUN_TEST(test_dualwheel_newIgn_12_1_trigNeg180_1);
  RUN_TEST(test_dualwheel_newIgn_12_1_trigNeg270_1);
  RUN_TEST(test_dualwheel_newIgn_12_1_trigNeg360_1);

/*
  RUN_TEST(test_dualwheel_newIgn_12_1_trig0_2);
  RUN_TEST(test_dualwheel_newIgn_12_1_trig90_2);
  RUN_TEST(test_dualwheel_newIgn_12_1_trig180_2);
  RUN_TEST(test_dualwheel_newIgn_12_1_trig270_2);
  RUN_TEST(test_dualwheel_newIgn_12_1_trig360_2);
  RUN_TEST(test_dualwheel_newIgn_12_1_trigNeg90_2);
  RUN_TEST(test_dualwheel_newIgn_12_1_trigNeg180_2);
  RUN_TEST(test_dualwheel_newIgn_12_1_trigNeg270_2);
  RUN_TEST(test_dualwheel_newIgn_12_1_trigNeg360_2);
*/
  //RUN_TEST(test_dualwheel_newIgn_60_2_trig181_2);
  //RUN_TEST(test_dualwheel_newIgn_60_2_trig182_2);
}