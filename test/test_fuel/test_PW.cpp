#include <globals.h>
#include <speeduino.h>
#include <unity.h>
#include "test_PW.h"
// #include "init.h"
#include "../test_utils.h"

#define PW_ALLOWED_ERROR  30

void testPW(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(test_PW_No_Multiply);
    RUN_TEST_P(test_PW_MAP_Multiply);
    RUN_TEST_P(test_PW_MAP_Multiply_Compatibility);
    RUN_TEST_P(test_PW_AFR_Multiply);
    RUN_TEST_P(test_PW_Large_Correction);
    RUN_TEST_P(test_PW_Very_Large_Correction);
    RUN_TEST_P(test_PW_4Cyl_PW0);
    RUN_TEST_P(test_PW_Limit_Long_Revolution);
    RUN_TEST_P(test_PW_Limit_90pct);
  }
}

int16_t REQ_FUEL;
byte VE;
long MAP;
uint16_t corrections;
int injOpen;

void test_PW_setCommon()
{
  // initialiseAll();
  REQ_FUEL = 1060;
  VE = 130;
  MAP = 94;
  corrections = 113;
  injOpen = 1000;
}

void test_PW_No_Multiply()
{
  test_PW_setCommon();

  configPage2.multiplyMAP = 0;
  configPage2.includeAFR = 0;
  configPage2.incorporateAFR = 0;
  configPage2.aeApplyMode = 0;

  uint16_t result = PW(REQ_FUEL, VE, MAP, corrections, injOpen);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR, 2557, result);
}

void test_PW_MAP_Multiply()
{
  test_PW_setCommon();

  configPage2.multiplyMAP = 1;
  currentStatus.baro = 103;
  configPage2.includeAFR = 0;
  configPage2.incorporateAFR = 0;
  configPage2.aeApplyMode = 0;

  uint16_t result = PW(REQ_FUEL, VE, MAP, corrections, injOpen);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR, 2400, result);
}

void test_PW_MAP_Multiply_Compatibility()
{
  test_PW_setCommon();

  configPage2.multiplyMAP = 2; //Divide MAP reading by 100 rather than by Baro reading
  currentStatus.baro = 103;
  configPage2.includeAFR = 0;
  configPage2.incorporateAFR = 0;
  configPage2.aeApplyMode = 0;

  uint16_t result = PW(REQ_FUEL, VE, MAP, corrections, injOpen);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR, 2449, result);
}

void test_PW_AFR_Multiply()
{
  test_PW_setCommon();

  configPage2.multiplyMAP = 0;
  currentStatus.baro = 100;
  configPage2.includeAFR = 1;
  configPage2.incorporateAFR = 0;
  configPage2.aeApplyMode = 0;
  configPage6.egoType = 2; //Set O2 sensor type to wideband
  currentStatus.runSecs = 20; configPage6.ego_sdelay = 10; //Ensure that the run time is longer than the O2 warmup time
  currentStatus.O2 = 150;
  currentStatus.afrTarget = 147;


  uint16_t result = PW(REQ_FUEL, VE, MAP, corrections, injOpen);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR, 2588, result);
}

/*
  To avoid overflow errors, the PW() function reduces accuracy slightly when the corrections figure becomes large.
  There are 3 levels of this:
  1) Corrections below 511 - No change in accuracy
  2) Corrections between 512 and 1023 - Minor reduction to accuracy
  3) Corrections above 1023 - Further reduction to accuracy
*/
void test_PW_Large_Correction()
{
  //This is the same as the test_PW_No_Multiply, but with correction changed to 600
  test_PW_setCommon();
  corrections = 600;

  configPage2.multiplyMAP = 0;
  configPage2.includeAFR = 0;
  configPage2.incorporateAFR = 0;
  configPage2.aeApplyMode = 0;

  uint16_t result = PW(REQ_FUEL, VE, MAP, corrections, injOpen);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR, 9268, result);
}

void test_PW_Very_Large_Correction()
{
  //This is the same as the test_PW_No_Multiply, but with correction changed to 1500
  test_PW_setCommon();
  corrections = 1500;

  configPage2.multiplyMAP = 0;
  configPage2.includeAFR = 0;
  configPage2.incorporateAFR = 0;
  configPage2.aeApplyMode = 0;

  uint16_t result = PW(REQ_FUEL, VE, MAP, corrections, injOpen);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR+30, 21670, result); //Additional allowed error here 
}

//Test that unused pulse width values are set to 0
//This test is for a 4 cylinder using paired injection where only INJ 1 and 2 should have PW > 0
void test_PW_4Cyl_PW0(void)
{
  test_PW_setCommon();

  configPage2.nCylinders = 4;
  configPage2.injLayout = INJ_PAIRED;
  configPage10.stagingEnabled = false; //Staging must be off or channels 3 and 4 will be used

  loop();
  TEST_ASSERT_EQUAL(0, currentStatus.PW3);
  TEST_ASSERT_EQUAL(0, currentStatus.PW4);
}

//Tests the PW Limit calculation for a normal scenario
void test_PW_Limit_90pct(void)
{
  test_PW_setCommon();

  revolutionTime = 10000UL; //6000 rpm
  configPage2.dutyLim = 90;

  //Duty limit of 90% for 10,000uS should give 9,000
  TEST_ASSERT_EQUAL(9000, calculatePWLimit());
}

//Tests the PW Limit calculation when the revolution time is greater than the max UINT16 value
//Occurs at approx. 915rpm
void test_PW_Limit_Long_Revolution(void)
{
  test_PW_setCommon();

  revolutionTime = 100000UL; //600 rpm, below 915rpm cutover point
  configPage2.dutyLim = 90;
  configPage2.strokes = TWO_STROKE;
  currentStatus.nSquirts = 1U;

  //Duty limit of 90% for 100,000uS should give 90,000, but as this would overflow the PW value, this should default to UINT16 Max
  TEST_ASSERT_EQUAL(UINT16_MAX, calculatePWLimit());
}