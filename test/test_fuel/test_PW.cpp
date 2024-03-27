#include <globals.h>
#include <speeduino.h>
#include <unity.h>
#include "test_PW.h"
// #include "init.h"
#include "../test_utils.h"
#include "pw_calcs.h"

#define PW_ALLOWED_ERROR  30

static int16_t REQ_FUEL;
static byte VE;
static long MAP;
static uint16_t corrections;

static void test_PW_setCommon_NoStage(void)
{
  // initialiseAll();
  REQ_FUEL = 1060;
  VE = 130;
  MAP = 94;
  corrections = 113;
  // Turns off pwLimit
  configPage2.dutyLim = 100;
  revolutionTime = UINT16_MAX;
  currentStatus.nSquirts = 1;
  // No staging
  configPage10.stagingEnabled = false;
  // Nitrous off
  currentStatus.nitrous_status = NITROUS_OFF;

  configPage2.injOpen = 10;
  configPage2.battVCorMode = BATTV_COR_MODE_WHOLE;
  currentStatus.batCorrection = 100;
  
  initialisePWCalcs();
}

static constexpr uint16_t NO_MULTIPLY_EXPECTED = 2557U;

static void test_setup_noMultiply(void) {
  configPage2.multiplyMAP = 0;
  configPage2.includeAFR = 0;
  configPage2.incorporateAFR = 0;
  configPage2.aeApplyMode = 0;
}

extern pulseWidths computePulseWidths(uint16_t REQ_FUEL, uint8_t VE, uint16_t MAP, uint16_t corrections);

static void test_PW_batt_correction(void) {
  // Same as test_PW_No_Multiply, but we apply battery correction to open time
  test_PW_setCommon_NoStage();
  test_setup_noMultiply();

  configPage2.battVCorMode = BATTV_COR_MODE_OPENTIME;
  currentStatus.batCorrection = 50U;
  uint16_t expectedOffset = (configPage2.injOpen*(100-currentStatus.batCorrection));

  pulseWidths result = computePulseWidths(REQ_FUEL, VE, MAP, corrections);
  TEST_ASSERT_GREATER_THAN(PW_ALLOWED_ERROR, expectedOffset);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR, NO_MULTIPLY_EXPECTED - expectedOffset, result.primary);
  TEST_ASSERT_EQUAL(0, result.secondary);
}

static void test_PW_ae_adder(void) {
  // Same as test_PW_No_Multiply, but we add in acceleration enrichment
  test_PW_setCommon_NoStage();
  test_setup_noMultiply();

  BIT_SET(currentStatus.engine, BIT_ENGINE_ACC);
  configPage2.aeApplyMode = AE_MODE_ADDER;
  currentStatus.AEamount = 105U;
  uint16_t expectedOffset = (REQ_FUEL*(currentStatus.AEamount-100U))/100;

  pulseWidths result = computePulseWidths(REQ_FUEL, VE, MAP, corrections);
  TEST_ASSERT_GREATER_THAN(PW_ALLOWED_ERROR, expectedOffset);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR, NO_MULTIPLY_EXPECTED + expectedOffset, result.primary);
  TEST_ASSERT_EQUAL(0, result.secondary);
}

static void test_setup_nitrous_stage1(void) {
  configPage10.n2o_stage1_minRPM = 20; // RPM/100
  configPage10.n2o_stage1_maxRPM = 30; // RPM/100
  configPage10.n2o_stage1_adderMin = 11; // milliseconds
  configPage10.n2o_stage1_adderMax = 3; // milliseconds
}

static void test_PW_nitrous_stage1(void) {
  // Same as test_PW_No_Multiply, but we add in nitrous
  test_PW_setCommon_NoStage();
  test_setup_noMultiply();

  currentStatus.nitrous_status = NITROUS_STAGE1;
  currentStatus.RPM = 2350;
  test_setup_nitrous_stage1();
  uint16_t expectedOffset = 820; // uS (3*100)+(1.0-(2350-(20*100))/((30-20)*100))*((11-3)*100)

  pulseWidths result = computePulseWidths(REQ_FUEL, VE, MAP, corrections);
  TEST_ASSERT_GREATER_THAN(PW_ALLOWED_ERROR, expectedOffset);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR, NO_MULTIPLY_EXPECTED + expectedOffset, result.primary);
  TEST_ASSERT_EQUAL(0, result.secondary);
}

static void test_setup_nitrous_stage2(void) {
  configPage10.n2o_stage2_minRPM = 25; // RPM/100
  configPage10.n2o_stage2_maxRPM = 30; // RPM/100
  configPage10.n2o_stage2_adderMin = 7; // milliseconds
  configPage10.n2o_stage2_adderMax = 1; // milliseconds
}

static void test_PW_nitrous_stage2(void) {
  // Same as test_PW_No_Multiply, but we add in nitrous
  test_PW_setCommon_NoStage();
  test_setup_noMultiply();

  currentStatus.nitrous_status = NITROUS_STAGE2;
  currentStatus.RPM = 2650;
  test_setup_nitrous_stage2();
  uint16_t expectedOffset = 520; // uS (1*100)+(1.0-(2650-(25*100))/((30-25)*100))*((7-1)*100)

  pulseWidths result = computePulseWidths(REQ_FUEL, VE, MAP, corrections);
  TEST_ASSERT_GREATER_THAN(PW_ALLOWED_ERROR, expectedOffset);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR, NO_MULTIPLY_EXPECTED + expectedOffset, result.primary);
  TEST_ASSERT_EQUAL(0, result.secondary);
}

static void test_PW_nitrous_stageboth(void) {
  // Same as test_PW_No_Multiply, but we add in nitrous
  test_PW_setCommon_NoStage();
  test_setup_noMultiply();

  currentStatus.nitrous_status = NITROUS_BOTH;
  currentStatus.RPM = 2650;
  test_setup_nitrous_stage2();
  uint16_t expectedOffset = 520+580; // uS

  pulseWidths result = computePulseWidths(REQ_FUEL, VE, MAP, corrections);
  TEST_ASSERT_GREATER_THAN(PW_ALLOWED_ERROR, expectedOffset);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR, NO_MULTIPLY_EXPECTED + expectedOffset, result.primary);
  TEST_ASSERT_EQUAL(0, result.secondary);
}

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
  RUN_TEST_P(test_PW_batt_correction);
  RUN_TEST_P(test_PW_ae_adder);
  RUN_TEST_P(test_PW_nitrous_stage1);
  RUN_TEST_P(test_PW_nitrous_stage2);
  RUN_TEST_P(test_PW_nitrous_stageboth);
  }
}

void test_PW_No_Multiply()
{
  test_PW_setCommon_NoStage();
  test_setup_noMultiply();

  pulseWidths result = computePulseWidths(REQ_FUEL, VE, MAP, corrections);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR, NO_MULTIPLY_EXPECTED, result.primary);
  TEST_ASSERT_EQUAL(0, result.secondary);
}

void test_PW_MAP_Multiply()
{
  test_PW_setCommon_NoStage();

  configPage2.multiplyMAP = 1;
  currentStatus.baro = 103;
  configPage2.includeAFR = 0;
  configPage2.incorporateAFR = 0;
  configPage2.aeApplyMode = 0;

  pulseWidths result = computePulseWidths(REQ_FUEL, VE, MAP, corrections);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR, 2400, result.primary);
  TEST_ASSERT_EQUAL(0, result.secondary);
}

void test_PW_MAP_Multiply_Compatibility()
{
  test_PW_setCommon_NoStage();

  configPage2.multiplyMAP = 2; //Divide MAP reading by 100 rather than by Baro reading
  currentStatus.baro = 103;
  configPage2.includeAFR = 0;
  configPage2.incorporateAFR = 0;
  configPage2.aeApplyMode = 0;

  pulseWidths result = computePulseWidths(REQ_FUEL, VE, MAP, corrections);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR, 2449, result.primary);
  TEST_ASSERT_EQUAL(0, result.secondary);
}

void test_PW_AFR_Multiply()
{
  test_PW_setCommon_NoStage();

  configPage2.multiplyMAP = 0;
  currentStatus.baro = 100;
  configPage2.includeAFR = 1;
  configPage2.incorporateAFR = 0;
  configPage2.aeApplyMode = 0;
  configPage6.egoType = 2; //Set O2 sensor type to wideband
  currentStatus.runSecs = 20; configPage6.ego_sdelay = 10; //Ensure that the run time is longer than the O2 warmup time
  currentStatus.O2 = 150;
  currentStatus.afrTarget = 147;

  pulseWidths result = computePulseWidths(REQ_FUEL, VE, MAP, corrections);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR, 2588, result.primary);
  TEST_ASSERT_EQUAL(0, result.secondary);
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
  test_PW_setCommon_NoStage();
  corrections = 600;

  configPage2.multiplyMAP = 0;
  configPage2.includeAFR = 0;
  configPage2.incorporateAFR = 0;
  configPage2.aeApplyMode = 0;

  pulseWidths result = computePulseWidths(REQ_FUEL, VE, MAP, corrections);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR, 9268, result.primary);
  TEST_ASSERT_EQUAL(0, result.secondary);
}

void test_PW_Very_Large_Correction()
{
  //This is the same as the test_PW_No_Multiply, but with correction changed to 1500
  test_PW_setCommon_NoStage();
  corrections = 1500;

  configPage2.multiplyMAP = 0;
  configPage2.includeAFR = 0;
  configPage2.incorporateAFR = 0;
  configPage2.aeApplyMode = 0;

  pulseWidths result = computePulseWidths(REQ_FUEL, VE, MAP, corrections);
  TEST_ASSERT_UINT16_WITHIN(PW_ALLOWED_ERROR+30, 21670, result.primary); //Additional allowed error here 
  TEST_ASSERT_EQUAL(0, result.secondary);
}


//Test that unused pulse width values are set to 0
//This test is for a 4 cylinder using paired injection where only INJ 1 and 2 should have PW > 0
void test_PW_4Cyl_PW0(void)
{
  test_PW_setCommon_NoStage();

  configPage2.nCylinders = 4;
  configPage2.injLayout = INJ_PAIRED;
  configPage10.stagingEnabled = false; //Staging must be off or channels 3 and 4 will be used

  applyPulseWidths(computePulseWidths(REQ_FUEL, VE, MAP, corrections));
  TEST_ASSERT_EQUAL(0, currentStatus.PW3);
  TEST_ASSERT_EQUAL(0, currentStatus.PW4);
}

extern uint16_t calculatePWLimit(void);

//Tests the PW Limit calculation for a normal scenario
void test_PW_Limit_90pct(void)
{
  test_PW_setCommon_NoStage();

  revolutionTime = 10000UL; //6000 rpm
  configPage2.dutyLim = 90;

  //Duty limit of 90% for 10,000uS should give 9,000
  TEST_ASSERT_INT32_WITHIN(4, 9000, calculatePWLimit());
}

//Tests the PW Limit calculation when the revolution time is greater than the max UINT16 value
//Occurs at approx. 915rpm
void test_PW_Limit_Long_Revolution(void)
{
  test_PW_setCommon_NoStage();

  revolutionTime = 100000UL; //600 rpm, below 915rpm cutover point
  configPage2.dutyLim = 90;
  configPage2.strokes = TWO_STROKE;
  currentStatus.nSquirts = 1U;

  //Duty limit of 90% for 100,000uS should give 90,000, but as this would overflow the PW value, this should default to UINT16 Max
  TEST_ASSERT_EQUAL(UINT16_MAX, calculatePWLimit());
}