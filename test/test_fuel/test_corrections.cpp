#include <unity.h>
#include "globals.h"
#include "corrections.h"
#include "test_corrections.h"
#include "../test_utils.h"
#include "init.h"
#include "sensors.h"
#include "speeduino.h"
#include "../test_utils.h"

void test_corrections_MAE(void);

void testCorrections()
{
  SET_UNITY_FILENAME() {

  test_corrections_WUE();
  test_corrections_dfco();
  test_corrections_TAE(); //TPS based accel enrichment corrections
  test_corrections_MAE(); //MAP based accel enrichment corrections
  test_corrections_cranking();
  test_corrections_ASE();
  test_corrections_floodclear();
  test_corrections_bat();
  test_corrections_launch();
  test_corrections_flex();
  /*
  RUN_TEST_P(test_corrections_closedloop); //Not written yet
  */
  }
}

void test_corrections_WUE_active(void)
{
  initialiseAll();

  //Check for WUE being active
  currentStatus.coolant = 0;
  configPage4.wueBins[9] = 120 + CALIBRATION_TEMPERATURE_OFFSET; //Set a WUE end value of 120
  correctionWUE();
  TEST_ASSERT_BIT_HIGH(BIT_ENGINE_WARMUP, currentStatus.engine);
}

void test_corrections_WUE_inactive(void)
{
  initialiseAll();

  //Check for WUE being inactive due to the temp being too high
  currentStatus.coolant = 200;
  configPage4.wueBins[9] = 120 + CALIBRATION_TEMPERATURE_OFFSET; //Set a WUE end value of 120
  correctionWUE();
  TEST_ASSERT_BIT_LOW(BIT_ENGINE_WARMUP, currentStatus.engine);
}

void test_corrections_WUE_inactive_value(void)
{
  initialiseAll();

  //Check for WUE being set to the final row of the WUE curve if the coolant is above the max WUE temp
  currentStatus.coolant = 200;
  configPage4.wueBins[9] = 100;
  configPage2.wueValues[9] = 123; //Use a value other than 100 here to ensure we are using the non-default value

  //Force invalidate the cache
  WUETable.cacheTime = currentStatus.secl - 1;
  
  TEST_ASSERT_EQUAL(123, correctionWUE() );
}

void test_corrections_WUE_active_value(void)
{
  initialiseAll();

  //Check for WUE being made active and returning a correct interpolated value
  currentStatus.coolant = 80;
  //Set some fake values in the table axis. Target value will fall between points 6 and 7
  configPage4.wueBins[0] = 0;
  configPage4.wueBins[1] = 0;
  configPage4.wueBins[2] = 0;
  configPage4.wueBins[3] = 0;
  configPage4.wueBins[4] = 0;
  configPage4.wueBins[5] = 0;
  configPage4.wueBins[6] = 70 + CALIBRATION_TEMPERATURE_OFFSET;
  configPage4.wueBins[7] = 90 + CALIBRATION_TEMPERATURE_OFFSET;
  configPage4.wueBins[8] = 100 + CALIBRATION_TEMPERATURE_OFFSET;
  configPage4.wueBins[9] = 120 + CALIBRATION_TEMPERATURE_OFFSET;

  configPage2.wueValues[6] = 120;
  configPage2.wueValues[7] = 130;

  //Force invalidate the cache
  WUETable.cacheTime = currentStatus.secl - 1;
  
  //Value should be midway between 120 and 130 = 125
  TEST_ASSERT_EQUAL(125, correctionWUE() );
}

void test_corrections_WUE(void)
{
  RUN_TEST_P(test_corrections_WUE_active);
  RUN_TEST_P(test_corrections_WUE_inactive);
  RUN_TEST_P(test_corrections_WUE_active_value);
  RUN_TEST_P(test_corrections_WUE_inactive_value);
}

extern uint16_t correctionCranking(void);

static void test_corrections_cranking_inactive(void) {
  initialiseAll();
  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK);
  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE);
  configPage10.crankingEnrichTaper = 0U;

  TEST_ASSERT_EQUAL(100, correctionCranking() );
} 

static void test_corrections_cranking_cranking(void) {
  initialiseAll();
  BIT_SET(currentStatus.engine, BIT_ENGINE_CRANK);
  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE);
  configPage10.crankingEnrichTaper = 0U;
  currentStatus.coolant = 150 - CALIBRATION_TEMPERATURE_OFFSET;

  configPage10.crankingEnrichValues[0] = 120U / 5U;
  configPage10.crankingEnrichBins[0] = currentStatus.coolant - 10U + CALIBRATION_TEMPERATURE_OFFSET;
  configPage10.crankingEnrichValues[1] = 130U / 5U;
  configPage10.crankingEnrichBins[1] = currentStatus.coolant + 10U + CALIBRATION_TEMPERATURE_OFFSET;
  configPage10.crankingEnrichValues[2] = 140U / 5U;
  configPage10.crankingEnrichBins[2] = currentStatus.coolant + 20U + CALIBRATION_TEMPERATURE_OFFSET;
  configPage10.crankingEnrichValues[3] = 150U / 5U;
  configPage10.crankingEnrichBins[3] = currentStatus.coolant + 30U + CALIBRATION_TEMPERATURE_OFFSET;

  // Should be half way between the 2 table values.
  TEST_ASSERT_EQUAL(125, correctionCranking() );
} 

static void test_corrections_cranking_taper_noase(void) {
  initialiseAll();
  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE);
  BIT_SET(LOOP_TIMER, BIT_TIMER_10HZ);
  configPage10.crankingEnrichTaper = 100U;
  currentStatus.ASEValue = 100U;
  
  currentStatus.coolant = 150 - CALIBRATION_TEMPERATURE_OFFSET;
  configPage10.crankingEnrichValues[0] = 120U / 5U;
  configPage10.crankingEnrichBins[0] = currentStatus.coolant - 10U + CALIBRATION_TEMPERATURE_OFFSET;
  configPage10.crankingEnrichValues[1] = 130U / 5U;
  configPage10.crankingEnrichBins[1] = currentStatus.coolant + 10U + CALIBRATION_TEMPERATURE_OFFSET;
  configPage10.crankingEnrichValues[2] = 140U / 5U;
  configPage10.crankingEnrichBins[2] = currentStatus.coolant + 20U + CALIBRATION_TEMPERATURE_OFFSET;
  configPage10.crankingEnrichValues[3] = 150U / 5U;
  configPage10.crankingEnrichBins[3] = currentStatus.coolant + 30U + CALIBRATION_TEMPERATURE_OFFSET;

  // Reset taper
  BIT_SET(currentStatus.engine, BIT_ENGINE_CRANK);
  (void)correctionCranking();

  // Advance taper to halfway
  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK);
  for (uint8_t index=0; index<configPage10.crankingEnrichTaper/2U; ++index) {
    (void)correctionCranking();
  }

  // Should be half way between the interpolated table value and 100%.
  TEST_ASSERT_EQUAL(113U, correctionCranking() );
  
  // Final taper step
  for (uint8_t index=configPage10.crankingEnrichTaper/2U; index<configPage10.crankingEnrichTaper-2U; ++index) {
    (void)correctionCranking();
  }
  TEST_ASSERT_EQUAL(101U, correctionCranking() );

  // Taper finished
  TEST_ASSERT_EQUAL(100U, correctionCranking());
  TEST_ASSERT_EQUAL(100U, correctionCranking());
} 


static void test_corrections_cranking_taper_withase(void) {
  initialiseAll();
  BIT_SET(LOOP_TIMER, BIT_TIMER_10HZ);
  configPage10.crankingEnrichTaper = 100U;
  
  currentStatus.coolant = 150 - CALIBRATION_TEMPERATURE_OFFSET;
  configPage10.crankingEnrichValues[0] = 120U / 5U;
  configPage10.crankingEnrichBins[0] = currentStatus.coolant - 10U + CALIBRATION_TEMPERATURE_OFFSET;
  configPage10.crankingEnrichValues[1] = 130U / 5U;
  configPage10.crankingEnrichBins[1] = currentStatus.coolant + 10U + CALIBRATION_TEMPERATURE_OFFSET;
  configPage10.crankingEnrichValues[2] = 140U / 5U;
  configPage10.crankingEnrichBins[2] = currentStatus.coolant + 20U + CALIBRATION_TEMPERATURE_OFFSET;
  configPage10.crankingEnrichValues[3] = 150U / 5U;
  configPage10.crankingEnrichBins[3] = currentStatus.coolant + 30U + CALIBRATION_TEMPERATURE_OFFSET;

  BIT_SET(currentStatus.engine, BIT_ENGINE_ASE);
  currentStatus.ASEValue = 50U;

  // Reset taper
  BIT_SET(currentStatus.engine, BIT_ENGINE_CRANK);
  (void)correctionCranking();

  // Advance taper to halfway
  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK);
  for (uint8_t index=0; index<configPage10.crankingEnrichTaper/2U; ++index) {
    (void)correctionCranking();
  }

  // Should be half way between the interpolated table value and 100%.
  TEST_ASSERT_EQUAL(175U, correctionCranking() );
  
  // Final taper step
  for (uint8_t index=configPage10.crankingEnrichTaper/2U; index<configPage10.crankingEnrichTaper-2U; ++index) {
    (void)correctionCranking();
  }
  TEST_ASSERT_EQUAL(102U, correctionCranking() );

  // Taper finished
  TEST_ASSERT_EQUAL(100U, correctionCranking());
  TEST_ASSERT_EQUAL(100U, correctionCranking());
} 

void test_corrections_cranking(void)
{
  RUN_TEST_P(test_corrections_cranking_inactive);
  RUN_TEST_P(test_corrections_cranking_cranking);
  RUN_TEST_P(test_corrections_cranking_taper_noase);
  RUN_TEST_P(test_corrections_cranking_taper_withase);
}

extern uint8_t correctionASE(void);

static void test_corrections_ASE_inactive_cranking(void)
{
  initialiseAll();
  BIT_SET(currentStatus.engine, BIT_ENGINE_CRANK);

  // Taper finished
  TEST_ASSERT_EQUAL(100U, correctionASE());
  TEST_ASSERT_FALSE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ASE));
}

static inline void setup_correctionASE(void) {
  initialiseAll();
  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK);
  BIT_SET(LOOP_TIMER, BIT_TIMER_10HZ) ;
  constexpr int16_t COOLANT_INITIAL = 150 - CALIBRATION_TEMPERATURE_OFFSET; 
  currentStatus.coolant = COOLANT_INITIAL;
  currentStatus.ASEValue = 0U;
  currentStatus.runSecs = 3;

  configPage2.aseCount[0] = 10;
  configPage2.aseBins[0] = COOLANT_INITIAL - 10U + CALIBRATION_TEMPERATURE_OFFSET;
  configPage2.aseCount[1] = 8;
  configPage2.aseBins[1] = COOLANT_INITIAL + 10U + CALIBRATION_TEMPERATURE_OFFSET;
  configPage2.aseCount[0] = 6;
  configPage2.aseBins[0] = COOLANT_INITIAL + 20U + CALIBRATION_TEMPERATURE_OFFSET;
  configPage2.aseCount[0] = 4;
  configPage2.aseBins[0] = COOLANT_INITIAL + 30U + CALIBRATION_TEMPERATURE_OFFSET;;

  configPage2.asePct[0] = 20U;
  configPage2.aseBins[0] = COOLANT_INITIAL - 10U + CALIBRATION_TEMPERATURE_OFFSET;
  configPage2.asePct[1] = 30U;
  configPage2.aseBins[1] = COOLANT_INITIAL + 10U + CALIBRATION_TEMPERATURE_OFFSET;
  configPage2.asePct[2] = 40U;
  configPage2.aseBins[2] = COOLANT_INITIAL + 20U + CALIBRATION_TEMPERATURE_OFFSET;
  configPage2.asePct[3] = 50U;
  configPage2.aseBins[3] = COOLANT_INITIAL + 30U + CALIBRATION_TEMPERATURE_OFFSET;  
}

static void test_corrections_ASE_initial(void)
{
  setup_correctionASE();

  // Should be half way between the 2 table values.
  TEST_ASSERT_EQUAL(125, correctionASE());
  TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ASE));
}

static void test_corrections_ASE_taper(void) {
  setup_correctionASE();
  // Switch to ASE taper
  configPage2.aseTaperTime = 12U;
  currentStatus.runSecs = 9;

  // Advance taper to halfway
  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK);
  for (uint8_t index=0; index<configPage2.aseTaperTime/2U; ++index) {
    (void)correctionASE();
  }

  // Should be half way between the interpolated table value and 100%.
  TEST_ASSERT_EQUAL(113, correctionASE());
  TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ASE));
  
  // Final taper step
  for (uint8_t index=configPage2.aseTaperTime/2U; index<configPage2.aseTaperTime-2U; ++index) {
    (void)correctionASE();
  }
  TEST_ASSERT_EQUAL(103U, correctionASE() );

  // Taper finished
  TEST_ASSERT_EQUAL(100U, correctionASE());  
  TEST_ASSERT_EQUAL(100U, correctionASE());  
}

void test_corrections_ASE(void)
{
  RUN_TEST_P(test_corrections_ASE_inactive_cranking);
  RUN_TEST_P(test_corrections_ASE_initial);
  RUN_TEST_P(test_corrections_ASE_taper);
}

uint8_t correctionFloodClear(void);

static void test_corrections_floodclear_no_crank_inactive(void) {
  initialiseAll();

  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK);
  configPage4.floodClear = 90;
  currentStatus.TPS = configPage4.floodClear + 10;

  TEST_ASSERT_EQUAL(100U, correctionFloodClear() );
}

static void test_corrections_floodclear_crank_below_threshold_inactive(void) {
  initialiseAll();

  BIT_SET(currentStatus.engine, BIT_ENGINE_CRANK);
  configPage4.floodClear = 90;
  currentStatus.TPS = configPage4.floodClear - 10;

  TEST_ASSERT_EQUAL(100U, correctionFloodClear() );
}

static void test_corrections_floodclear_crank_above_threshold_active(void) {
  initialiseAll();

  BIT_SET(currentStatus.engine, BIT_ENGINE_CRANK);
  configPage4.floodClear = 90;
  currentStatus.TPS = configPage4.floodClear + 10;

  TEST_ASSERT_EQUAL(0U, correctionFloodClear() );
}

void test_corrections_floodclear(void)
{
  RUN_TEST_P(test_corrections_floodclear_no_crank_inactive);
  RUN_TEST_P(test_corrections_floodclear_crank_below_threshold_inactive);
  RUN_TEST_P(test_corrections_floodclear_crank_above_threshold_active);
}

void test_corrections_closedloop(void)
{

}

uint8_t correctionFlex(void);

static void setupFlexFuelTable(void) {
  configPage10.flexFuelBins[0] = 0;
  configPage10.flexFuelAdj[0] = 0;
  configPage10.flexFuelBins[1] = 10;
  configPage10.flexFuelAdj[1] = 20;
  configPage10.flexFuelBins[2] = 30;
  configPage10.flexFuelAdj[2] = 40;
  configPage10.flexFuelBins[3] = 50;
  configPage10.flexFuelAdj[3] = 80;
  configPage10.flexFuelBins[4] = 60;
  configPage10.flexFuelAdj[4] = 120;
  configPage10.flexFuelBins[5] = 70;
  configPage10.flexFuelAdj[5] = 150;
}

static void test_corrections_flex_flex_off(void) {
  initialiseAll();
  setupFlexFuelTable();
  configPage2.flexEnabled = false;
  currentStatus.ethanolPct = 65;
  TEST_ASSERT_EQUAL(100U, correctionFlex() );
}

static void test_corrections_flex_flex_on(void) {
  initialiseAll();
  setupFlexFuelTable();
  configPage2.flexEnabled = true;
  currentStatus.ethanolPct = 65;
  TEST_ASSERT_EQUAL(135U, correctionFlex() );
}

uint8_t correctionFuelTemp(void);

static void setupFuelTempTable(void) {
  configPage10.fuelTempBins[0] = 0;
  configPage10.fuelTempValues[0] = 0;
  configPage10.fuelTempBins[1] = 10;
  configPage10.fuelTempValues[1] = 20;
  configPage10.fuelTempBins[2] = 30;
  configPage10.fuelTempValues[2] = 40;
  configPage10.fuelTempBins[3] = 50;
  configPage10.fuelTempValues[3] = 80;
  configPage10.fuelTempBins[4] = 60;
  configPage10.fuelTempValues[4] = 120;
  configPage10.fuelTempBins[5] = 70;
  configPage10.fuelTempValues[5] = 150;  
}

static void test_corrections_fueltemp_off(void) {
  initialiseAll();
  setupFuelTempTable();
  configPage2.flexEnabled = false;
  currentStatus.fuelTemp = 65 - CALIBRATION_TEMPERATURE_OFFSET;
  TEST_ASSERT_EQUAL(100U, correctionFuelTemp() );
}

static void test_corrections_fueltemp_on(void) {
  initialiseAll();
  setupFuelTempTable();
  configPage2.flexEnabled = true;
  currentStatus.fuelTemp = 65 - CALIBRATION_TEMPERATURE_OFFSET;
  TEST_ASSERT_EQUAL(135U, correctionFuelTemp() );
}

void test_corrections_flex(void)
{
  RUN_TEST_P(test_corrections_flex_flex_off);
  RUN_TEST_P(test_corrections_flex_flex_on);
  RUN_TEST_P(test_corrections_fueltemp_off);
  RUN_TEST_P(test_corrections_fueltemp_on);
}

uint8_t correctionBatVoltage(void);

static void setup_battery_correction(void) {
  initialiseAll();

  configPage6.voltageCorrectionBins[0]      = 60;
  configPage6.injVoltageCorrectionValues[0] = 115;
  configPage6.voltageCorrectionBins[1]      = 70;
  configPage6.injVoltageCorrectionValues[1] = 110;
  configPage6.voltageCorrectionBins[2]      = 80;
  configPage6.injVoltageCorrectionValues[2] = 105;
  configPage6.voltageCorrectionBins[3]      = 90;
  configPage6.injVoltageCorrectionValues[3] = 100;
  configPage6.voltageCorrectionBins[4]      = 100;
  configPage6.injVoltageCorrectionValues[4] = 95;
  configPage6.voltageCorrectionBins[5]      = 110;
  configPage6.injVoltageCorrectionValues[5] = 90;
}

static void test_corrections_bat_mode_wholePw(void) {
  setup_battery_correction();

  configPage2.battVCorMode = BATTV_COR_MODE_WHOLE;
  currentStatus.battery10 = 75;
  configPage2.injOpen = 10;
  inj_opentime_uS = configPage2.injOpen * 100U;

  TEST_ASSERT_EQUAL(108U, correctionBatVoltage() );
  TEST_ASSERT_EQUAL(configPage2.injOpen * 100U, inj_opentime_uS );
}

void test_corrections_bat(void)
{
  RUN_TEST_P(test_corrections_bat_mode_wholePw);
}

uint8_t correctionLaunch(void);

static void test_corrections_launch_inactive(void) {
  initialiseAll();

  currentStatus.launchingHard = false;
  currentStatus.launchingSoft = false;
  configPage6.lnchFuelAdd = 25;

  TEST_ASSERT_EQUAL(100U, correctionLaunch() );
}

static void test_corrections_launch_hard(void) {
  initialiseAll();

  currentStatus.launchingHard = true;
  currentStatus.launchingSoft = false;
  configPage6.lnchFuelAdd = 25;

  TEST_ASSERT_EQUAL(125U, correctionLaunch() );
}

static void test_corrections_launch_soft(void) {
  initialiseAll();

  currentStatus.launchingHard = false;
  currentStatus.launchingSoft = true;
  configPage6.lnchFuelAdd = 25;

  TEST_ASSERT_EQUAL(125U, correctionLaunch() );
}

static void test_corrections_launch_both(void) {
  initialiseAll();

  currentStatus.launchingHard = true;
  currentStatus.launchingSoft = true;
  configPage6.lnchFuelAdd = 25;

  TEST_ASSERT_EQUAL(125U, correctionLaunch() );
}

void test_corrections_launch(void)
{
  RUN_TEST_P(test_corrections_launch_inactive);
  RUN_TEST_P(test_corrections_launch_hard);
  RUN_TEST_P(test_corrections_launch_soft);
  RUN_TEST_P(test_corrections_launch_both);
}

void setup_DFCO_on()
{
  //Sets all the required conditions to have the DFCO be active
  configPage2.dfcoEnabled = 1; //Ensure DFCO option is turned on
  currentStatus.RPM = 4000; //Set the current simulated RPM to a level above the DFCO rpm threshold
  currentStatus.TPS = 0; //Set the simulated TPS to 0 
  currentStatus.coolant = 80;
  configPage4.dfcoRPM = 150; //DFCO enable RPM = 1500
  configPage4.dfcoTPSThresh = 1;
  configPage4.dfcoHyster = 25;
  configPage2.dfcoMinCLT = 40; //Actually 0 with offset
  configPage2.dfcoDelay = 10;

  dfcoDelay = 1;
  correctionDFCO();
  dfcoDelay = 20;
}
//**********************************************************************************************************************
void test_corrections_dfco_on(void)
{
  //Test under ideal conditions that DFCO goes active
  setup_DFCO_on();

  TEST_ASSERT_TRUE(correctionDFCO());
}
void test_corrections_dfco_off_RPM()
{
  //Test that DFCO comes on and then goes off when the RPM drops below threshold
  setup_DFCO_on();

  TEST_ASSERT_TRUE(correctionDFCO()); //Make sure DFCO is on initially
  currentStatus.RPM = 1000; //Set the current simulated RPM below the threshold + hyster
  TEST_ASSERT_FALSE(correctionDFCO()); //Test DFCO is now off
}
void test_corrections_dfco_off_TPS()
{
  //Test that DFCO comes on and then goes off when the TPS goes above the required threshold (ie not off throttle)
  setup_DFCO_on();

  TEST_ASSERT_TRUE(correctionDFCO()); //Make sure DFCO is on initially
  currentStatus.TPS = 10; //Set the current simulated TPS to be above the threshold
  TEST_ASSERT_FALSE(correctionDFCO()); //Test DFCO is now off
}
void test_corrections_dfco_off_delay()
{
  //Test that DFCO comes will not activate if there has not been a long enough delay
  //The steup function below simulates a 2 second delay
  setup_DFCO_on();

  //Set the threshold to be 2.5 seconds, above the simulated delay of 2s
  configPage2.dfcoDelay = 250;

  TEST_ASSERT_FALSE(correctionDFCO()); //Make sure DFCO does not come on
}
void setup_DFCO_taper_on()
{
  //Test that DFCO comes will not activate if there has not been a long enough delay
  //The steup function below simulates a 2 second delay
  setup_DFCO_on();

  configPage9.dfcoTaperEnable = 1; //Enable
  configPage9.dfcoTaperTime = 20; //2.0 second
  configPage9.dfcoTaperFuel = 0; //Scale fuel to 0%
  configPage9.dfcoTaperAdvance = 20; //Reduce 20deg until full fuel cut

  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_DFCO);
  //Set the threshold to be 2.5 seconds, above the simulated delay of 2s
  configPage2.dfcoDelay = 250;
}
void test_corrections_dfco_taper()
{
  setup_DFCO_taper_on();

  TEST_ASSERT_FALSE(correctionDFCO()); //Make sure DFCO does not come on
  correctionDFCOfuel();
  TEST_ASSERT_EQUAL(20, dfcoTaper); //Check if value was reset to setting
}
void test_corrections_dfco_taper_fuel()
{
  setup_DFCO_taper_on();

  correctionDFCOfuel();
  TEST_ASSERT_EQUAL(20, dfcoTaper); //Check if value was reset to setting

  BIT_SET(currentStatus.status1, BIT_STATUS1_DFCO);
  dfcoTaper = 10;
  TEST_ASSERT_EQUAL(50, correctionDFCOfuel());
  dfcoTaper = 5;
  TEST_ASSERT_EQUAL(25, correctionDFCOfuel());

  configPage9.dfcoTaperTime = 10; //1.0 second
  dfcoTaper = 15; //Check for overflow
  TEST_ASSERT_EQUAL(100, correctionDFCOfuel());
  configPage9.dfcoTaperEnable = 0; //Disable
  TEST_ASSERT_EQUAL(0, correctionDFCOfuel());
}
void test_corrections_dfco_taper_ign()
{
  setup_DFCO_taper_on();

  dfcoTaper = 20;
  BIT_SET(currentStatus.status1, BIT_STATUS1_DFCO);

  TEST_ASSERT_EQUAL(20, correctionDFCOignition(20));
  dfcoTaper = 15;
  TEST_ASSERT_EQUAL(15, correctionDFCOignition(20));
  dfcoTaper = 10;
  TEST_ASSERT_EQUAL(10, correctionDFCOignition(20));
  dfcoTaper = 5;
  TEST_ASSERT_EQUAL(5, correctionDFCOignition(20));
  configPage9.dfcoTaperEnable = 0; //Disable
  TEST_ASSERT_EQUAL(20, correctionDFCOignition(20));
}

void test_corrections_dfco()
{
  RUN_TEST_P(test_corrections_dfco_on);
  RUN_TEST_P(test_corrections_dfco_off_RPM);
  RUN_TEST_P(test_corrections_dfco_off_TPS);
  RUN_TEST_P(test_corrections_dfco_off_delay);
  RUN_TEST_P(test_corrections_dfco_taper);
  RUN_TEST_P(test_corrections_dfco_taper_fuel);
  RUN_TEST_P(test_corrections_dfco_taper_ign);
}

//**********************************************************************************************************************
//Setup a basic TAE enrichment curve, threshold etc that are common to all tests. Specifica values maybe updated in each individual test
void test_corrections_TAE_setup()
{
  configPage2.aeMode = AE_MODE_TPS; //Set AE to TPS

  configPage4.taeValues[0] = 70;
  configPage4.taeValues[1] = 103; 
  configPage4.taeValues[2] = 124;
  configPage4.taeValues[3] = 136; 

  //Note: These values are divided by 10
  configPage4.taeBins[0] = 0;
  configPage4.taeBins[1] = 8; 
  configPage4.taeBins[2] = 22;
  configPage4.taeBins[3] = 97; 
  
  configPage2.taeThresh = 0;
  configPage2.taeMinChange = 0;

  //Divided by 100
  configPage2.aeTaperMin = 10; //1000
  configPage2.aeTaperMax = 50; //5000
	
	//Set the coolant to be above the warmup AE taper
	configPage2.aeColdTaperMax = 60;
	configPage2.aeColdTaperMin = 0;
	currentStatus.coolant = (int)(configPage2.aeColdTaperMax - CALIBRATION_TEMPERATURE_OFFSET) + 1;

  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC); //Make sure AE is turned off
  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_DCC); //Make sure AE is turned off
}

void test_corrections_TAE_no_rpm_taper()
{
  test_corrections_TAE_setup();

  //Disable the taper
  currentStatus.RPM = 2000;
  configPage2.aeTaperMin = 50; //5000
  configPage2.aeTaperMax = 60; //6000

  currentStatus.TPSlast = 0;
  currentStatus.TPS = 50; //25% actual value

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(750, currentStatus.tpsDOT); //DOT is 750%/s (25 * 30)
  TEST_ASSERT_EQUAL((100+132), accelValue);
	TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged on
	TEST_ASSERT_FALSE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_DCC)); //Confirm AE is flagged on
}

void test_corrections_TAE_negative_tpsdot()
{
  test_corrections_TAE_setup();

  //Disable the taper
  currentStatus.RPM = 2000;
  configPage2.aeTaperMin = 50; //5000
  configPage2.aeTaperMax = 60; //6000
  configPage2.decelAmount = 50;

  currentStatus.TPSlast = 50;
  currentStatus.TPS = 0;

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(-750, currentStatus.tpsDOT); //DOT is 750%/s (25 * 30)
  TEST_ASSERT_EQUAL(configPage2.decelAmount, accelValue);
	TEST_ASSERT_FALSE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged on
	TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_DCC)); //Confirm AE is flagged on
}

void test_corrections_TAE_50pc_rpm_taper()
{
  test_corrections_TAE_setup();

  //RPM is 50% of the way through the taper range
  currentStatus.RPM = 3000;
  configPage2.aeTaperMin = 10; //1000
  configPage2.aeTaperMax = 50; //5000

  currentStatus.TPSlast = 0;
  currentStatus.TPS = 50; //25% actual value

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(750, currentStatus.tpsDOT); //DOT is 750%/s (25 * 30)
  TEST_ASSERT_EQUAL((100+66), accelValue);
	TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged on
	TEST_ASSERT_FALSE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_DCC)); //Confirm AE is flagged on
}

void test_corrections_TAE_110pc_rpm_taper()
{
  test_corrections_TAE_setup();

  //RPM is 110% of the way through the taper range, which should result in no additional AE
  currentStatus.RPM = 5400;
  configPage2.aeTaperMin = 10; //1000
  configPage2.aeTaperMax = 50; //5000

  currentStatus.TPSlast = 0;
  currentStatus.TPS = 50; //25% actual value

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(750, currentStatus.tpsDOT); //DOT is 750%/s (25 * 30)
  TEST_ASSERT_EQUAL(100, accelValue); //Should be no AE as we're above the RPM taper end point
	TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged on
	TEST_ASSERT_FALSE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_DCC)); //Confirm AE is flagged on
}

void test_corrections_TAE_under_threshold()
{
  test_corrections_TAE_setup();

  //RPM is 50% of the way through the taper range, but TPS value will be below threshold
  currentStatus.RPM = 3000;
  configPage2.aeTaperMin = 10; //1000
  configPage2.aeTaperMax = 50; //5000

  currentStatus.TPSlast = 0;
  currentStatus.TPS = 6; //3% actual value. TPSDot should be 90%/s
	configPage2.taeThresh = 100; //Above the reading of 90%/s

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(90, currentStatus.tpsDOT); //DOT is 90%/s (3% * 30)
  TEST_ASSERT_EQUAL(100, accelValue); //Should be no AE as we're above the RPM taper end point
	TEST_ASSERT_FALSE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged off
	TEST_ASSERT_FALSE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_DCC)); //Confirm AE is flagged on
}

void test_corrections_TAE_50pc_warmup_taper()
{
  test_corrections_TAE_setup();

  //Disable the RPM taper
  currentStatus.RPM = 2000;
  configPage2.aeTaperMin = 50; //5000
  configPage2.aeTaperMax = 60; //6000

  currentStatus.TPSlast = 0;
  currentStatus.TPS = 50; //25% actual value
	
	//Set a cold % of 50% increase
	configPage2.aeColdPct = 150;
	configPage2.aeColdTaperMax = 60 + CALIBRATION_TEMPERATURE_OFFSET;
	configPage2.aeColdTaperMin = 0 + CALIBRATION_TEMPERATURE_OFFSET;
	//Set the coolant to be 50% of the way through the warmup range
	currentStatus.coolant = 30;

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(750, currentStatus.tpsDOT); //DOT is 750%/s (25 * 30)
  TEST_ASSERT_EQUAL((100+165), accelValue); //Total AE should be 132 + (50% * 50%) = 132 * 1.25 = 165
	TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged on
	TEST_ASSERT_FALSE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_DCC)); //Confirm AE is flagged on
}

void test_corrections_TAE()
{
  RUN_TEST_P(test_corrections_TAE_negative_tpsdot);
  RUN_TEST_P(test_corrections_TAE_no_rpm_taper);
  RUN_TEST_P(test_corrections_TAE_50pc_rpm_taper);
  RUN_TEST_P(test_corrections_TAE_110pc_rpm_taper);
  RUN_TEST_P(test_corrections_TAE_under_threshold);
  RUN_TEST_P(test_corrections_TAE_50pc_warmup_taper);
}


//**********************************************************************************************************************
//Setup a basic MAE enrichment curve, threshold etc that are common to all tests. Specifica values maybe updated in each individual test
void test_corrections_MAE_setup()
{
  configPage2.aeMode = AE_MODE_MAP; //Set AE to TPS

  configPage4.maeRates[0] = 70;
  configPage4.maeRates[1] = 103; 
  configPage4.maeRates[2] = 124;
  configPage4.maeRates[3] = 136; 

  //Note: These values are divided by 10
  configPage4.maeBins[0] = 0;
  configPage4.maeBins[1] = 15; 
  configPage4.maeBins[2] = 19;
  configPage4.maeBins[3] = 50; 
  
  configPage2.maeThresh = 0;
  configPage2.maeMinChange = 0;

  //Divided by 100
  configPage2.aeTaperMin = 10; //1000
  configPage2.aeTaperMax = 50; //5000
	
	//Set the coolant to be above the warmup AE taper
	configPage2.aeColdTaperMax = 60;
	configPage2.aeColdTaperMin = 0;
	currentStatus.coolant = (int)(configPage2.aeColdTaperMax - CALIBRATION_TEMPERATURE_OFFSET) + 1;

  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC); //Make sure AE is turned off
  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_DCC); //Make sure AE is turned off
}

void test_corrections_MAE_negative_tpsdot()
{
  test_corrections_MAE_setup();

  //Disable the taper
  currentStatus.RPM = 2000;
  configPage2.aeTaperMin = 50; //5000
  configPage2.aeTaperMax = 60; //6000
  configPage2.decelAmount = 50;

  MAPlast_time = UINT16_MAX*2UL;
  MAP_time = MAPlast_time + 25000UL; 
  MAPlast = 50;
  currentStatus.MAP = 40;

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(-400, currentStatus.mapDOT);
  TEST_ASSERT_EQUAL(configPage2.decelAmount, accelValue);
	TEST_ASSERT_FALSE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged on
	TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_DCC)); //Confirm AE is flagged on
}

void test_corrections_MAE_no_rpm_taper()
{
  test_corrections_MAE_setup();

  //Disable the taper
  configPage2.aeTaperMin = 50; //5000
  configPage2.aeTaperMax = 60; //6000

  MAPlast_time = UINT16_MAX*2UL;
  MAP_time = MAPlast_time + 25000UL; 
  MAPlast = 40;
  currentStatus.MAP = 50;

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(400, currentStatus.mapDOT);
  TEST_ASSERT_EQUAL((100+132), accelValue);
	TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged on
}

void test_corrections_MAE_50pc_rpm_taper()
{
  test_corrections_MAE_setup();

  //RPM is 50% of the way through the taper range
  currentStatus.RPM = 3000;
  configPage2.aeTaperMin = 10; //1000
  configPage2.aeTaperMax = 50; //5000

  MAPlast_time = UINT16_MAX*2UL;
  MAP_time = MAPlast_time + 25000UL; 
  MAPlast = 40;
  currentStatus.MAP = 50;

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(400, currentStatus.mapDOT);
  TEST_ASSERT_EQUAL((100+66), accelValue);
	TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged on
}

void test_corrections_MAE_110pc_rpm_taper()
{
  test_corrections_MAE_setup();

  //RPM is 110% of the way through the taper range, which should result in no additional AE
  currentStatus.RPM = 5400;
  configPage2.aeTaperMin = 10; //1000
  configPage2.aeTaperMax = 50; //5000

  MAPlast_time = UINT16_MAX*2UL;
  MAP_time = MAPlast_time + 25000UL; 
  MAPlast = 40;
  currentStatus.MAP = 50;

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(400, currentStatus.mapDOT);
  TEST_ASSERT_EQUAL(100, accelValue); //Should be no AE as we're above the RPM taper end point
	TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged on
}

void test_corrections_MAE_under_threshold()
{
  test_corrections_MAE_setup();

  //RPM is 50% of the way through the taper range, but TPS value will be below threshold
  currentStatus.RPM = 3000;
  configPage2.aeTaperMin = 10; //1000
  configPage2.aeTaperMax = 50; //5000

  MAPlast_time = UINT16_MAX*2UL;
  MAP_time = MAPlast_time + 25000UL; 
  MAPlast = 0;
  currentStatus.MAP = 6; 
	configPage2.maeThresh = 241; //Above the reading of 240%/s

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(240, currentStatus.mapDOT);
  TEST_ASSERT_EQUAL(100, accelValue); //Should be no AE as we're above the RPM taper end point
	TEST_ASSERT_FALSE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged off
}

void test_corrections_MAE_50pc_warmup_taper()
{
  test_corrections_MAE_setup();

  //Disable the RPM taper
  configPage2.aeTaperMin = 50; //5000
  configPage2.aeTaperMax = 60; //6000

  MAPlast_time = UINT16_MAX*2UL;
  MAP_time = MAPlast_time + 25000UL; 
  MAPlast = 40;
  currentStatus.MAP = 50;

	//Set a cold % of 50% increase
	configPage2.aeColdPct = 150;
	configPage2.aeColdTaperMax = 60 + CALIBRATION_TEMPERATURE_OFFSET;
	configPage2.aeColdTaperMin = 0 + CALIBRATION_TEMPERATURE_OFFSET;
	//Set the coolant to be 50% of the way through the warmup range
	currentStatus.coolant = 30;

  uint16_t accelValue = correctionAccel(); //Run the AE calcs

  TEST_ASSERT_EQUAL(400, currentStatus.mapDOT);
  TEST_ASSERT_EQUAL((100+165), accelValue); 
	TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC)); //Confirm AE is flagged on
}

void test_corrections_MAE()
{
  RUN_TEST_P(test_corrections_MAE_negative_tpsdot);
  RUN_TEST_P(test_corrections_MAE_no_rpm_taper);
  RUN_TEST_P(test_corrections_MAE_50pc_rpm_taper);
  RUN_TEST_P(test_corrections_MAE_110pc_rpm_taper);
  RUN_TEST_P(test_corrections_MAE_under_threshold);
  RUN_TEST_P(test_corrections_MAE_50pc_warmup_taper);
}