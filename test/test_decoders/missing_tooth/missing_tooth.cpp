#include <decoders.h>
#include <globals.h>
#include <unity.h>
#include "missing_tooth.h"
#include "schedule_calcs.h"

void test_setup_36_1()
{
    //Setup a 36-1 wheel
    configPage4.triggerTeeth = 36;
    configPage4.triggerMissingTeeth = 1;
    configPage4.TrigSpeed = CRANK_SPEED;
    configPage4.trigPatternSec = SEC_TRIGGER_SINGLE;

    triggerSetup_missingTooth();
}

void test_setup_60_2()
{
    //Setup a 60-2 wheel
    configPage4.triggerTeeth = 60;
    configPage4.triggerMissingTeeth = 2;
    configPage4.TrigSpeed = CRANK_SPEED;
    configPage4.trigPatternSec = SEC_TRIGGER_SINGLE;

    triggerSetup_missingTooth();
}

//************************************** Begin the new ignition setEndTooth tests **************************************
void test_missingtooth_newIgn_36_1_trig0_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=0
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 0; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(34, ignition1EndTooth);
}

void test_missingtooth_newIgn_36_1_trig90_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=90
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 90; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(25, ignition1EndTooth);
}

void test_missingtooth_newIgn_36_1_trig180_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=180
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 180; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(16, ignition1EndTooth);
}

void test_missingtooth_newIgn_36_1_trig270_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=270
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 270; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(7, ignition1EndTooth);
}

void test_missingtooth_newIgn_36_1_trig360_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=360
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 360; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(34, ignition1EndTooth);
}

void test_missingtooth_newIgn_36_1_trigNeg90_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-90
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -90; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(7, ignition1EndTooth);
}

void test_missingtooth_newIgn_36_1_trigNeg180_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-180
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -180; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(16, ignition1EndTooth);
}

void test_missingtooth_newIgn_36_1_trigNeg270_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-270
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -270; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(25, ignition1EndTooth);
}

void test_missingtooth_newIgn_36_1_trigNeg360_1()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-360
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition1EndAngle = 360 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -360; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(34, ignition1EndTooth);
}

// ******* CHannel 2 *******
void test_missingtooth_newIgn_36_1_trig0_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=0
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 0; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(16, ignition2EndTooth);
}

void test_missingtooth_newIgn_36_1_trig90_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=90
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 90; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(7, ignition2EndTooth);
}

void test_missingtooth_newIgn_36_1_trig180_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=180
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 180; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(34, ignition2EndTooth);
}

void test_missingtooth_newIgn_36_1_trig270_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=270
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 270; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(25, ignition2EndTooth);
}

void test_missingtooth_newIgn_36_1_trig360_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=360
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 360; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(16, ignition2EndTooth);
}

void test_missingtooth_newIgn_36_1_trigNeg90_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-90
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -90; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(25, ignition2EndTooth);
}

void test_missingtooth_newIgn_36_1_trigNeg180_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-180
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -180; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(34, ignition2EndTooth);
}

void test_missingtooth_newIgn_36_1_trigNeg270_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-270
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -270; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(7, ignition2EndTooth);
}

void test_missingtooth_newIgn_36_1_trigNeg360_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 36-1
    //Advance: 10
    //triggerAngle=-360
    test_setup_36_1();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = -360; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(16, ignition2EndTooth);
}

void test_missingtooth_newIgn_60_2_trig0_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 60-2
    //Advance: 10
    //triggerAngle=300
    test_setup_60_2();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 0; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(57, ignition2EndTooth);
}

void test_missingtooth_newIgn_60_2_trig181_2()
{
    //Test the set end tooth function. Conditions:
    //Trigger: 60-2
    //Advance: 10
    //triggerAngle=300
    test_setup_60_2();
    configPage4.sparkMode = IGN_MODE_WASTED;
    ignition2EndAngle = 180 - 10; //Set 10 degrees advance
    configPage4.triggerAngle = 181; //No trigger offset
    
    triggerSetEndTeeth_missingTooth();
    TEST_ASSERT_EQUAL(58, ignition2EndTooth);
}


void test_missingtooth_newIgn_2()
{

}
void test_missingtooth_newIgn_3()
{

}
void test_missingtooth_newIgn_4()
{

}

void testMissingTooth()
{
  RUN_TEST(test_missingtooth_newIgn_36_1_trig0_1);
  RUN_TEST(test_missingtooth_newIgn_36_1_trig90_1);
  RUN_TEST(test_missingtooth_newIgn_36_1_trig180_1);
  RUN_TEST(test_missingtooth_newIgn_36_1_trig270_1);
  RUN_TEST(test_missingtooth_newIgn_36_1_trig360_1);
  RUN_TEST(test_missingtooth_newIgn_36_1_trigNeg90_1);
  RUN_TEST(test_missingtooth_newIgn_36_1_trigNeg180_1);
  RUN_TEST(test_missingtooth_newIgn_36_1_trigNeg270_1);
  RUN_TEST(test_missingtooth_newIgn_36_1_trigNeg360_1);

  RUN_TEST(test_missingtooth_newIgn_36_1_trig0_2);
  RUN_TEST(test_missingtooth_newIgn_36_1_trig90_2);
  RUN_TEST(test_missingtooth_newIgn_36_1_trig180_2);
  RUN_TEST(test_missingtooth_newIgn_36_1_trig270_2);
  RUN_TEST(test_missingtooth_newIgn_36_1_trig360_2);
  RUN_TEST(test_missingtooth_newIgn_36_1_trigNeg90_2);
  RUN_TEST(test_missingtooth_newIgn_36_1_trigNeg180_2);
  RUN_TEST(test_missingtooth_newIgn_36_1_trigNeg270_2);
  RUN_TEST(test_missingtooth_newIgn_36_1_trigNeg360_2);

  //RUN_TEST(test_missingtooth_newIgn_60_2_trig181_2);
  //RUN_TEST(test_missingtooth_newIgn_60_2_trig182_2);
}