#include <unity.h>
#include "test_staging.h"
#include "../test_utils.h"
#include "globals.h"
#include "pw_calcs.h"

void testStaging(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST(test_Staging_Off);
    RUN_TEST(test_Staging_4cyl_Auto_Inactive);
    RUN_TEST(test_Staging_4cyl_Table_Inactive);
    RUN_TEST(test_Staging_4cyl_Auto_50pct);
    RUN_TEST(test_Staging_4cyl_Auto_33pct);
    RUN_TEST(test_Staging_4cyl_Table_50pct);
  }
}

pulseWidths computePulseWidths(uint16_t REQ_FUEL, uint8_t VE, uint16_t MAP, uint16_t corrections);

void test_Staging_setCommon()
{
  // initialiseAll();
  
  configPage2.nCylinders = 4;
  maxInjOutputs = 2;
  currentStatus.RPM = 3000;
  currentStatus.fuelLoad = 50;
  configPage2.multiplyMAP = 0;
  configPage2.includeAFR = false;
  configPage2.incorporateAFR = false;
  configPage2.injOpen = 10;  //1ms inj open time
  configPage2.battVCorMode = BATTV_COR_MODE_WHOLE; 
  // Turns off pwLimit
  configPage2.dutyLim = 100;
  revolutionTime = 10000;
  currentStatus.nSquirts = 1;
  configPage2.strokes = FOUR_STROKE;
  BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC);

  // Nitrous off
  currentStatus.nitrous_status = NITROUS_OFF;
 
  /*
      These values are a percentage of the req_fuel value that would be required for each injector channel to deliver that much fuel.
      Eg:
      Pri injectors are 250cc
      Sec injectors are 500cc
      Total injector capacity = 750cc

      staged_req_fuel_mult_pri = 300% (The primary injectors would have to run 3x the overall PW in order to be the equivalent of the full 750cc capacity
      staged_req_fuel_mult_sec = 150% (The secondary injectors would have to run 1.5x the overall PW in order to be the equivalent of the full 750cc capacity
  */
  configPage10.stagedInjSizePri = 250;
  configPage10.stagedInjSizeSec = 500;

  initialisePWCalcs();
}

void test_Staging_Off(void)
{
  BIT_SET(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE);
  configPage10.stagingEnabled = false;

  test_Staging_setCommon();

  //90% duty cycle at 6000rpm
  configPage2.dutyLim = 90;
  revolutionTime = 5000;
  pulseWidths pw = computePulseWidths(1000, 100, 100, 200);
  TEST_ASSERT_FALSE(BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE));
  TEST_ASSERT_NOT_EQUAL(0, pw.primary);
  TEST_ASSERT_EQUAL(0, pw.secondary);  
}

void test_Staging_4cyl_Auto_Inactive(void)
{
  BIT_SET(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE);
  configPage2.injLayout = INJ_PAIRED;
  configPage10.stagingEnabled = true;
  configPage10.stagingMode = STAGING_MODE_AUTO;

  test_Staging_setCommon();

  //90% duty cycle at 6000rpm
  //90% duty cycle at 6000rpm
  configPage2.dutyLim = 90;
  revolutionTime = 5000;
  pulseWidths pw = computePulseWidths(1000, 100, 100, 200);

  //PW 1 and 2 should be normal, 3 and 4 should be 0 as that testPW is below the pwLimit
  //PW1/2 should be ((PW - openTime) * staged_req_fuel_mult_pri) + openTime = ((3000 - 1000) * 3.0) + 1000 = 7000
  TEST_ASSERT_EQUAL(7000, pw.primary);
  TEST_ASSERT_EQUAL(0, pw.secondary);
  TEST_ASSERT_FALSE(BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE));
}

void test_Staging_4cyl_Table_Inactive(void)
{
  BIT_SET(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE);
  configPage2.injLayout = INJ_PAIRED;
  configPage10.stagingEnabled = true;
  configPage10.stagingMode = STAGING_MODE_TABLE;

  test_Staging_setCommon();

  //Load the staging table with all 0
  //For this test it doesn't matter what the X and Y axis are, as the table is all 0 values
  for(byte x=0; x<64; x++) { stagingTable.values.values[x] = 0; }

  //90% duty cycle at 6000rpm
  configPage2.dutyLim = 90;
  revolutionTime = 5000;
  pulseWidths pw = computePulseWidths(1000, 100, 100, 200);
  //PW 1 and 2 should be normal, 3 and 4 should be 0 as that testPW is below the pwLimit
  //PW1/2 should be (PW - openTime) * staged_req_fuel_mult_pri = (3000 - 1000) * 3.0 = 6000
  TEST_ASSERT_EQUAL(7000, pw.primary);
  TEST_ASSERT_EQUAL(0, pw.secondary);
  TEST_ASSERT_FALSE(BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE));
}

void test_Staging_4cyl_Auto_50pct(void)
{
  BIT_CLEAR(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE);
  configPage2.injLayout = INJ_PAIRED;
  configPage10.stagingEnabled = true;
  configPage10.stagingMode = STAGING_MODE_AUTO;
  test_Staging_setCommon();

  configPage2.dutyLim = 90;
  revolutionTime = 5000;
  pulseWidths pw = computePulseWidths(4000, 100, 100, 200);
  TEST_ASSERT_EQUAL(9000, pw.primary);
  TEST_ASSERT_EQUAL(9000, pw.secondary);
  TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE));
}

void test_Staging_4cyl_Auto_33pct(void)
{
  BIT_CLEAR(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE);
  configPage2.injLayout = INJ_PAIRED;
  configPage10.stagingEnabled = true;
  configPage10.stagingMode = STAGING_MODE_AUTO;
  test_Staging_setCommon();

  configPage2.dutyLim = 90;
  revolutionTime = 5000;
  pulseWidths pw = computePulseWidths(3000, 100, 100, 200);
  //PW 1 and 2 should be maxed out at the pwLimit, 3 and 4 should be based on their relative size
  TEST_ASSERT_EQUAL(9000, pw.primary);
  TEST_ASSERT_EQUAL(6000, pw.secondary);
  TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE));
}

void test_Staging_4cyl_Table_50pct(void)
{
  BIT_CLEAR(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE);
  configPage2.injLayout = INJ_PAIRED;
  configPage10.stagingEnabled = true;
  configPage10.stagingMode = STAGING_MODE_TABLE;

  test_Staging_setCommon();

  //Load the staging table with all 0
  //For this test it doesn't matter what the X and Y axis are, as the table is all 50 values
  for(byte x=0; x<64; x++) { stagingTable.values.values[x] = 50; }

  //Need to change the lookup values so we don't get a cached value
  currentStatus.RPM += 1;
  currentStatus.fuelLoad += 1;

  configPage2.dutyLim = 90;
  revolutionTime = 5000;
  pulseWidths pw = computePulseWidths(1000, 100, 100, 200);
  TEST_ASSERT_EQUAL(4000, pw.primary);
  TEST_ASSERT_EQUAL(2500, pw.secondary);
  TEST_ASSERT_TRUE(BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE));  
}