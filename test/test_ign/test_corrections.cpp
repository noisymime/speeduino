#include <unity.h>
#include "globals.h"
#include "corrections.h"
// #include "init.h"
#include "idle.h"
#include "../test_utils.h"
#include "sensors.h"

extern void construct2dTables(void);

extern int8_t correctionFixedTiming(int8_t advance);

static void test_correctionFixedTiming_inactive(void) {
    configPage2.fixAngEnable = 0;
    configPage4.FixAng = 13;

    TEST_ASSERT_EQUAL(8, correctionFixedTiming(8));
}

static void test_correctionFixedTiming_active(void) {
    configPage2.fixAngEnable = 1;
    configPage4.FixAng = 13;

    TEST_ASSERT_EQUAL(configPage4.FixAng, correctionFixedTiming(8));
}

static void test_correctionFixedTiming(void) {
    RUN_TEST_P(test_correctionFixedTiming_inactive);
    RUN_TEST_P(test_correctionFixedTiming_active);
}

extern int8_t correctionCLTadvance(int8_t advance);

static void setup_clt_advance_table(void) {
  construct2dTables();
  initialiseCorrections();
  TEST_DATA_P uint8_t bins[] = { 60, 70, 80, 90, 100, 110 };
  TEST_DATA_P uint8_t values[] = { 30, 25, 20, 15, 10, 5 };
  populate_2dtable_P(&CLTAdvanceTable, values, bins);
}

static void test_correctionCLTadvance_lookup(void) {
    setup_clt_advance_table();

    currentStatus.coolant = 105 - CALIBRATION_TEMPERATURE_OFFSET;
    TEST_ASSERT_EQUAL(1, correctionCLTadvance(8));
}

static void test_correctionCLTadvance(void) {
    RUN_TEST_P(test_correctionCLTadvance_lookup);
}

static void test_correctionCrankingFixedTiming_nocrank_inactive(void) {
    setup_clt_advance_table();
    BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK);
    configPage2.crkngAddCLTAdv = 0;
    configPage4.CrankAng = 8;

    TEST_ASSERT_EQUAL(-7, correctionCrankingFixedTiming(-7));
}

static void test_correctionCrankingFixedTiming_crank_fixed(void) {
    setup_clt_advance_table();
    BIT_SET(currentStatus.engine, BIT_ENGINE_CRANK);
    configPage2.crkngAddCLTAdv = 0;
    configPage4.CrankAng = 8;

    TEST_ASSERT_EQUAL(configPage4.CrankAng, correctionCrankingFixedTiming(-7));
}

static void test_correctionCrankingFixedTiming_crank_coolant(void) {
    setup_clt_advance_table();
    BIT_SET(currentStatus.engine, BIT_ENGINE_CRANK);
    configPage2.crkngAddCLTAdv = 1;
    configPage4.CrankAng = 8;
    currentStatus.coolant = 105 - CALIBRATION_TEMPERATURE_OFFSET;

    TEST_ASSERT_EQUAL(1, correctionCrankingFixedTiming(8));
}

static void test_correctionCrankingFixedTiming(void) {
    RUN_TEST_P(test_correctionCrankingFixedTiming_nocrank_inactive);
    RUN_TEST_P(test_correctionCrankingFixedTiming_crank_fixed);
    RUN_TEST_P(test_correctionCrankingFixedTiming_crank_coolant);
}

extern int8_t correctionFlexTiming(int8_t advance);

static void setup_flexAdv(void) {
  construct2dTables();
  initialiseCorrections();
  TEST_DATA_P uint8_t bins[] = { 30, 40, 50, 60, 70, 80 };
  TEST_DATA_P uint8_t values[] = { 30, 25, 20, 15, 10, 5 };
  populate_2dtable_P(&flexAdvTable, values, bins);

  configPage2.flexEnabled = 1;
  currentStatus.ethanolPct = 55;
}

static void test_correctionFlexTiming_inactive(void) {
    setup_flexAdv();
    configPage2.flexEnabled = 0;

    TEST_ASSERT_EQUAL(-7, correctionFlexTiming(-7));
}

static void test_correctionFlexTiming_table_lookup(void) {
    setup_flexAdv();

    TEST_ASSERT_EQUAL(-14, correctionFlexTiming(8));
    TEST_ASSERT_EQUAL(18-OFFSET_IGNITION, currentStatus.flexIgnCorrection);    
}

static void test_correctionFlexTiming(void) {
    RUN_TEST_P(test_correctionFlexTiming_inactive);
    RUN_TEST_P(test_correctionFlexTiming_table_lookup);
}

extern int8_t correctionWMITiming(int8_t advance);

static void setup_WMIAdv(void) {
    construct2dTables();
    initialiseCorrections();

    configPage10.wmiEnabled= 1;
    configPage10.wmiAdvEnabled = 1;
    BIT_CLEAR(currentStatus.status4, BIT_STATUS4_WMI_EMPTY);
    configPage10.wmiTPS = 50;
    currentStatus.TPS = configPage10.wmiTPS + 1;
    configPage10.wmiRPM = 30;
    currentStatus.RPM = configPage10.wmiRPM + 1U;
    configPage10.wmiMAP = 75;
    currentStatus.MAP = (configPage10.wmiMAP*2L)+1L;
    configPage10.wmiIAT = 155;
    currentStatus.IAT = configPage10.wmiIAT - CALIBRATION_TEMPERATURE_OFFSET + 1;

    TEST_DATA_P uint8_t bins[] = { 30, 40, 50, 60, 70, 80 };
    TEST_DATA_P uint8_t values[] = { 30, 25, 20, 15, 10, 5 };
    populate_2dtable_P(&wmiAdvTable, values, bins);
}

static void test_correctionWMITiming_table_lookup(void) {
    setup_WMIAdv();

    TEST_ASSERT_EQUAL(-24, correctionWMITiming(8));
}

static void test_correctionWMITiming_wmidisabled_inactive(void) {
    setup_WMIAdv();
    configPage10.wmiEnabled= 0;

    TEST_ASSERT_EQUAL(8, correctionWMITiming(8));
}


static void test_correctionWMITiming_wmiadvdisabled_inactive(void) {
    setup_WMIAdv();
    configPage10.wmiAdvEnabled = 0;

    TEST_ASSERT_EQUAL(8, correctionWMITiming(8));
}

static void test_correctionWMITiming_empty_inactive(void) {
    setup_WMIAdv();
    BIT_SET(currentStatus.status4, BIT_STATUS4_WMI_EMPTY);

    TEST_ASSERT_EQUAL(8, correctionWMITiming(8));
}

static void test_correctionWMITiming_tpslow_inactive(void) {
    setup_WMIAdv();
    currentStatus.TPS = configPage10.wmiTPS - 1;

    TEST_ASSERT_EQUAL(8, correctionWMITiming(8));
}

static void test_correctionWMITiming_rpmlow_inactive(void) {
    setup_WMIAdv();
    currentStatus.RPM = configPage10.wmiRPM - 1U;

    TEST_ASSERT_EQUAL(8, correctionWMITiming(8));
}
   
static void test_correctionWMITiming_maplow_inactive(void) {
    setup_WMIAdv();
    currentStatus.MAP = (configPage10.wmiMAP*2)-1;

    TEST_ASSERT_EQUAL(8, correctionWMITiming(8));
}
    
static void test_correctionWMITiming_iatlow_inactive(void) {
    setup_WMIAdv();
    currentStatus.IAT = configPage10.wmiIAT - CALIBRATION_TEMPERATURE_OFFSET - 1;

    TEST_ASSERT_EQUAL(8, correctionWMITiming(8));
}   

static void test_correctionWMITiming(void) {
    RUN_TEST_P(test_correctionWMITiming_table_lookup);
    RUN_TEST_P(test_correctionWMITiming_tpslow_inactive);
    RUN_TEST_P(test_correctionWMITiming_wmidisabled_inactive);
    RUN_TEST_P(test_correctionWMITiming_wmiadvdisabled_inactive);
    RUN_TEST_P(test_correctionWMITiming_empty_inactive);
    RUN_TEST_P(test_correctionWMITiming_tpslow_inactive);
    RUN_TEST_P(test_correctionWMITiming_rpmlow_inactive);
    RUN_TEST_P(test_correctionWMITiming_maplow_inactive);
    RUN_TEST_P(test_correctionWMITiming_iatlow_inactive);
}

extern int8_t correctionIATretard(int8_t advance);

static void setup_IATRetard(void) {
  construct2dTables();
  initialiseCorrections();

  TEST_DATA_P uint8_t bins[] = { 30, 40, 50, 60, 70, 80 };
  TEST_DATA_P uint8_t values[] = { 30, 25, 20, 15, 10, 5 };
  populate_2dtable_P(&IATRetardTable, values, bins);

  currentStatus.IAT = 75;
}

static void test_correctionIATretard_table_lookup(void) {
    setup_IATRetard();

    TEST_ASSERT_EQUAL(-19, correctionIATretard(-11));
}

static void test_correctionIATretard(void) {
    RUN_TEST_P(test_correctionIATretard_table_lookup);
}

extern int8_t correctionIdleAdvance(int8_t advance);

static void setup_idleadv_tps(void) {
    configPage2.idleAdvAlgorithm = IDLEADVANCE_ALGO_TPS;
    configPage2.idleAdvTPS = 30;
    currentStatus.TPS = configPage2.idleAdvTPS - 1U;
}

static void setup_idleadv_ctps(void) {
    configPage2.idleAdvAlgorithm = IDLEADVANCE_ALGO_CTPS;
    currentStatus.CTPSActive = 1;
}

static void setup_correctionIdleAdvance(void) {
    construct2dTables();
    initialiseCorrections();

    TEST_DATA_P uint8_t bins[] = { 30, 40, 50, 60, 70, 80 };
    TEST_DATA_P uint8_t values[] = { 30, 25, 20, 15, 10, 5 };
    populate_2dtable_P(&idleAdvanceTable, values, bins);
  
    configPage2.idleAdvEnabled = IDLEADVANCE_MODE_ADDED;
    configPage2.idleAdvDelay = 5;
    configPage2.idleAdvRPM = 20;
    configPage2.vssMode = 0;
    configPage6.iacAlgorithm = IAC_ALGORITHM_NONE;
    configPage9.idleAdvStartDelay = 0U;

    runSecsX10 = configPage2.idleAdvDelay * 5;
    BIT_SET(currentStatus.engine, BIT_ENGINE_RUN);
    currentStatus.RPM = (configPage2.idleAdvRPM * 100) - 1U;
    currentStatus.CLIdleTarget = (currentStatus.RPM-100)/100;
    
    setup_idleadv_tps();
    // Run once to initialise internal state
    correctionIdleAdvance(8);
}

static void test_correctionIdleAdvance_tps_lookup_nodelay(void) {
    setup_correctionIdleAdvance();

    setup_idleadv_tps();
    configPage2.idleAdvEnabled = IDLEADVANCE_MODE_ADDED;
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));

    configPage2.idleAdvEnabled = IDLEADVANCE_MODE_SWITCHED;
    TEST_ASSERT_EQUAL(15, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_ctps_lookup_nodelay(void) {
    setup_correctionIdleAdvance();

    setup_idleadv_ctps();
    configPage2.idleAdvEnabled = IDLEADVANCE_MODE_ADDED;
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));

    configPage2.idleAdvEnabled = IDLEADVANCE_MODE_SWITCHED;
    TEST_ASSERT_EQUAL(15, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_inactive_notrunning(void) {
    setup_correctionIdleAdvance();
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
    BIT_CLEAR(currentStatus.engine, BIT_ENGINE_RUN);
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_noadvance_modeoff(void) {
    setup_correctionIdleAdvance();
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
    configPage2.idleAdvEnabled = IDLEADVANCE_MODE_OFF;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_noadvance_rpmtoohigh(void) {
    setup_correctionIdleAdvance();
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
    currentStatus.RPM = (configPage2.idleAdvRPM * 100)+1;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_noadvance_vsslimit(void) {
    setup_correctionIdleAdvance();
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
    configPage2.vssMode = 1;
    configPage2.idleAdvVss = 15;
    currentStatus.vss = configPage2.idleAdvVss + 1;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_noadvance_tpslimit(void) {
    setup_correctionIdleAdvance();
    setup_idleadv_tps();
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
    currentStatus.TPS = configPage2.idleAdvTPS + 1U;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_noadvance_ctpsinactive(void) {
    setup_correctionIdleAdvance();
    setup_idleadv_ctps();
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
    currentStatus.CTPSActive = 0;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_noadvance_rundelay(void) {
    setup_correctionIdleAdvance();
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
    runSecsX10 = (configPage2.idleAdvDelay * 5)-1;
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance_delay(void) {
    setup_correctionIdleAdvance();
    configPage9.idleAdvStartDelay = 3;
    BIT_SET(LOOP_TIMER, BIT_TIMER_10HZ);
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
    TEST_ASSERT_EQUAL(8, correctionIdleAdvance(8));
    TEST_ASSERT_EQUAL(23, correctionIdleAdvance(8));
}

static void test_correctionIdleAdvance(void) {
    RUN_TEST_P(test_correctionIdleAdvance_tps_lookup_nodelay);
    RUN_TEST_P(test_correctionIdleAdvance_ctps_lookup_nodelay);
    RUN_TEST_P(test_correctionIdleAdvance_inactive_notrunning);
    RUN_TEST_P(test_correctionIdleAdvance_noadvance_modeoff);
    RUN_TEST_P(test_correctionIdleAdvance_noadvance_rpmtoohigh);
    RUN_TEST_P(test_correctionIdleAdvance_noadvance_vsslimit);
    RUN_TEST_P(test_correctionIdleAdvance_noadvance_tpslimit);
    RUN_TEST_P(test_correctionIdleAdvance_noadvance_ctpsinactive);
    RUN_TEST_P(test_correctionIdleAdvance_noadvance_rundelay);
    RUN_TEST_P(test_correctionIdleAdvance_delay);
}

extern int8_t correctionSoftRevLimit(int8_t advance);

static void setup_correctionSoftRevLimit(void) {
    construct2dTables();
    initialiseCorrections();

    configPage6.engineProtectType = PROTECT_CUT_IGN;
    configPage4.SoftRevLim = 50;
    configPage4.SoftLimMax = 1;
    configPage4.SoftLimRetard = -5;
    configPage2.SoftLimitMode = SOFT_LIMIT_FIXED;

    currentStatus.RPMdiv100 = configPage4.SoftRevLim + 1;
    softLimitTime = 0;

    BIT_CLEAR(LOOP_TIMER, BIT_TIMER_10HZ);
}

static void test_correctionSoftRevLimit_modes(void) {
    setup_correctionSoftRevLimit();
    configPage4.SoftLimRetard = -5;

    configPage2.SoftLimitMode = SOFT_LIMIT_FIXED;
    TEST_ASSERT_EQUAL(-5, correctionSoftRevLimit(8));
    TEST_ASSERT_BIT_HIGH(BIT_SPARK_SFTLIM , currentStatus.spark);

    configPage2.SoftLimitMode = SOFT_LIMIT_RELATIVE;
    TEST_ASSERT_EQUAL(13, correctionSoftRevLimit(8));
    TEST_ASSERT_BIT_HIGH(BIT_SPARK_SFTLIM , currentStatus.spark);
}

static void test_correctionSoftRevLimit_inactive_protecttype(void) {
    setup_correctionSoftRevLimit();

    configPage6.engineProtectType = PROTECT_CUT_OFF;
    TEST_ASSERT_EQUAL(8, correctionSoftRevLimit(8));
    TEST_ASSERT_BIT_LOW(BIT_SPARK_SFTLIM , currentStatus.spark);

    configPage6.engineProtectType = PROTECT_CUT_FUEL;
    TEST_ASSERT_EQUAL(8, correctionSoftRevLimit(8));
    TEST_ASSERT_BIT_LOW(BIT_SPARK_SFTLIM , currentStatus.spark);
}

static void test_correctionSoftRevLimit_inactive_rpmtoohigh(void) {
    setup_correctionSoftRevLimit();
    TEST_ASSERT_EQUAL(-5, correctionSoftRevLimit(8));

    currentStatus.RPMdiv100 = configPage4.SoftRevLim-1;
    TEST_ASSERT_EQUAL(8, correctionSoftRevLimit(8));
    TEST_ASSERT_BIT_LOW(BIT_SPARK_SFTLIM , currentStatus.spark);
}

static void test_correctionSoftRevLimit_timeout(void) {
    setup_correctionSoftRevLimit();
    configPage4.SoftLimMax = 3;
    BIT_SET(LOOP_TIMER, BIT_TIMER_10HZ);
    TEST_ASSERT_EQUAL(-5, correctionSoftRevLimit(8));
    TEST_ASSERT_EQUAL(-5, correctionSoftRevLimit(8));
    TEST_ASSERT_EQUAL(-5, correctionSoftRevLimit(8));
    TEST_ASSERT_EQUAL(8, correctionSoftRevLimit(8));
}

static void test_correctionSoftRevLimit(void) {
    RUN_TEST_P(test_correctionSoftRevLimit_modes);
    RUN_TEST_P(test_correctionSoftRevLimit_inactive_protecttype);
    RUN_TEST_P(test_correctionSoftRevLimit_inactive_rpmtoohigh);
    RUN_TEST_P(test_correctionSoftRevLimit_timeout);
}

extern int8_t correctionNitrous(int8_t advance);

static void test_correctionNitrous_disabled(void) {
    configPage10.n2o_enable = 0;
    TEST_ASSERT_EQUAL(13, correctionNitrous(13));
    TEST_ASSERT_EQUAL(-13, correctionNitrous(-13));
}

static void test_correctionNitrous_stage1(void) {
    configPage10.n2o_enable = 1;
    configPage10.n2o_stage1_retard = 5;
    configPage10.n2o_stage2_retard = 0;
    
    currentStatus.nitrous_status = NITROUS_STAGE1;
    TEST_ASSERT_EQUAL(8, correctionNitrous(13));
    TEST_ASSERT_EQUAL(-18, correctionNitrous(-13));
    
    currentStatus.nitrous_status = NITROUS_BOTH;
    TEST_ASSERT_EQUAL(8, correctionNitrous(13));
    TEST_ASSERT_EQUAL(-18, correctionNitrous(-13));
}

static void test_correctionNitrous_stage2(void) {
    configPage10.n2o_enable = 1;
    configPage10.n2o_stage1_retard = 0;
    configPage10.n2o_stage2_retard = 5;
    
    currentStatus.nitrous_status = NITROUS_STAGE2;
    TEST_ASSERT_EQUAL(8, correctionNitrous(13));
    TEST_ASSERT_EQUAL(-18, correctionNitrous(-13));
    
    currentStatus.nitrous_status = NITROUS_BOTH;
    TEST_ASSERT_EQUAL(8, correctionNitrous(13));
    TEST_ASSERT_EQUAL(-18, correctionNitrous(-13));
}

static void test_correctionNitrous_stageboth(void) {
    configPage10.n2o_enable = 1;
    configPage10.n2o_stage1_retard = 3;
    configPage10.n2o_stage2_retard = 5;
      
    currentStatus.nitrous_status = NITROUS_BOTH;
    TEST_ASSERT_EQUAL(5, correctionNitrous(13));
    TEST_ASSERT_EQUAL(-21, correctionNitrous(-13));
}

static void test_correctionNitrous(void) {
    RUN_TEST_P(test_correctionNitrous_disabled);
    RUN_TEST_P(test_correctionNitrous_stage1);
    RUN_TEST_P(test_correctionNitrous_stage2);
    RUN_TEST_P(test_correctionNitrous_stageboth);
}

extern int8_t correctionSoftLaunch(int8_t advance);

static void setup_correctionSoftLaunch(void) {
    configPage6.launchEnabled = 1;
    configPage6.flatSArm = 20;
    configPage6.lnchSoftLim = 40;
    configPage10.lnchCtrlTPS = 80;
    
    currentStatus.clutchTrigger = 1;
    currentStatus.clutchEngagedRPM = ((configPage6.flatSArm) * 100) - 100;
    currentStatus.RPM = ((configPage6.lnchSoftLim) * 100) + 100;
    currentStatus.TPS = configPage10.lnchCtrlTPS + 1;
}

static void test_correctionSoftLaunch_on(void) {
    setup_correctionSoftLaunch();
    configPage6.lnchRetard = -3;

    TEST_ASSERT_EQUAL(configPage6.lnchRetard, correctionSoftLaunch(-8));
    TEST_ASSERT_TRUE(currentStatus.launchingSoft);
    TEST_ASSERT_BIT_HIGH(BIT_SPARK_SLAUNCH, currentStatus.spark);
}

static void test_correctionSoftLaunch_off_disabled(void) {
    setup_correctionSoftLaunch();
    configPage6.launchEnabled = 0;
    configPage6.lnchRetard = -3;

    TEST_ASSERT_EQUAL(-8, correctionSoftLaunch(-8));
    TEST_ASSERT_FALSE(currentStatus.launchingSoft);
    TEST_ASSERT_BIT_LOW(BIT_SPARK_SLAUNCH, currentStatus.spark);
}

static void test_correctionSoftLaunch_off_noclutchtrigger(void) {
    setup_correctionSoftLaunch();
    currentStatus.clutchTrigger = 0;
    configPage6.lnchRetard = -3;

    TEST_ASSERT_EQUAL(-8, correctionSoftLaunch(-8));
    TEST_ASSERT_FALSE(currentStatus.launchingSoft);
    TEST_ASSERT_BIT_LOW(BIT_SPARK_SLAUNCH, currentStatus.spark);
}

static void test_correctionSoftLaunch_off_clutchrpmlow(void) {
    setup_correctionSoftLaunch();
    currentStatus.clutchEngagedRPM = (configPage6.flatSArm * 100) + 1;
    configPage6.lnchRetard = -3;

    TEST_ASSERT_EQUAL(-8, correctionSoftLaunch(-8));
    TEST_ASSERT_FALSE(currentStatus.launchingSoft);
    TEST_ASSERT_BIT_LOW(BIT_SPARK_SLAUNCH, currentStatus.spark);
}

static void test_correctionSoftLaunch_off_rpmlimit(void) {
    setup_correctionSoftLaunch();
    currentStatus.RPM = (configPage6.lnchSoftLim * 100) - 1;
    configPage6.lnchRetard = -3;

    TEST_ASSERT_EQUAL(-8, correctionSoftLaunch(-8));
    TEST_ASSERT_FALSE(currentStatus.launchingSoft);
    TEST_ASSERT_BIT_LOW(BIT_SPARK_SLAUNCH, currentStatus.spark);
}

static void test_correctionSoftLaunch_off_tpslow(void) {
    setup_correctionSoftLaunch();
    currentStatus.TPS = configPage10.lnchCtrlTPS - 1;
    configPage6.lnchRetard = -3;

    TEST_ASSERT_EQUAL(-8, correctionSoftLaunch(-8));
    TEST_ASSERT_FALSE(currentStatus.launchingSoft);
    TEST_ASSERT_BIT_LOW(BIT_SPARK_SLAUNCH, currentStatus.spark);
}

static void test_correctionSoftLaunch(void) {
    RUN_TEST_P(test_correctionSoftLaunch_on);
    RUN_TEST_P(test_correctionSoftLaunch_off_disabled);
    RUN_TEST_P(test_correctionSoftLaunch_off_noclutchtrigger);
    RUN_TEST_P(test_correctionSoftLaunch_off_clutchrpmlow);
    RUN_TEST_P(test_correctionSoftLaunch_off_rpmlimit);
    RUN_TEST_P(test_correctionSoftLaunch_off_tpslow);
}

extern int8_t correctionSoftFlatShift(int8_t advance);

static void setup_correctionSoftFlatShift(void) {
    configPage6.flatSEnable = 1;
    configPage6.flatSArm = 10;
    configPage6.flatSSoftWin = 10;
    
    currentStatus.clutchTrigger = 1;
    currentStatus.clutchEngagedRPM = ((configPage6.flatSArm) * 100) + 500;
    currentStatus.RPM = currentStatus.clutchEngagedRPM + 600;
}

static void test_correctionSoftFlatShift_on(void) {
    setup_correctionSoftFlatShift();
    configPage6.flatSRetard = -3;

    TEST_ASSERT_EQUAL(configPage6.flatSRetard, correctionSoftFlatShift(-8));
    TEST_ASSERT_BIT_HIGH(BIT_SPARK2_FLATSS, currentStatus.spark2);
}

static void test_correctionSoftFlatShift_off_disabled(void) {
    setup_correctionSoftFlatShift();
    configPage6.flatSRetard = -3;
    configPage6.flatSEnable = 0;

    TEST_ASSERT_EQUAL(-8, correctionSoftFlatShift(-8));
    TEST_ASSERT_BIT_LOW(BIT_SPARK2_FLATSS, currentStatus.spark2);
}

static void test_correctionSoftFlatShift_off_noclutchtrigger(void) {
    setup_correctionSoftFlatShift();
    configPage6.flatSRetard = -3;
    currentStatus.clutchTrigger = 0;

    TEST_ASSERT_EQUAL(-8, correctionSoftFlatShift(-8));
    TEST_ASSERT_BIT_LOW(BIT_SPARK2_FLATSS, currentStatus.spark2);
}

static void test_correctionSoftFlatShift_off_clutchrpmtoolow(void) {
    setup_correctionSoftFlatShift();
    configPage6.flatSRetard = -3;
    currentStatus.clutchEngagedRPM = ((configPage6.flatSArm) * 100) - 500;

    TEST_ASSERT_EQUAL(-8, correctionSoftFlatShift(-8));
    TEST_ASSERT_BIT_LOW(BIT_SPARK2_FLATSS, currentStatus.spark2);
}

static void test_correctionSoftFlatShift_off_rpmnotinwindow(void) {
    setup_correctionSoftFlatShift();
    configPage6.flatSRetard = -3;
    currentStatus.RPM = (currentStatus.clutchEngagedRPM - (configPage6.flatSSoftWin * 100) ) - 100;

    TEST_ASSERT_EQUAL(-8, correctionSoftFlatShift(-8));
    TEST_ASSERT_BIT_LOW(BIT_SPARK2_FLATSS, currentStatus.spark2);
}

static void test_correctionSoftFlatShift(void) {
    RUN_TEST_P(test_correctionSoftFlatShift_on);
    RUN_TEST_P(test_correctionSoftFlatShift_off_disabled);
    RUN_TEST_P(test_correctionSoftFlatShift_off_noclutchtrigger);
    RUN_TEST_P(test_correctionSoftFlatShift_off_clutchrpmtoolow);
    RUN_TEST_P(test_correctionSoftFlatShift_off_rpmnotinwindow);
}

extern int8_t correctionKnock(int8_t advance);

static void setup_correctionKnock(void) {
    configPage10.knock_mode = KNOCK_MODE_DIGITAL;
    configPage10.knock_count = 5U;
    configPage10.knock_firstStep = 3U;
    knockCounter = configPage10.knock_count + 1;
//   TEST_DATA_P uint8_t startBins[] = { 30, 40, 50, 60, 70, 80 };
//   TEST_DATA_P uint8_t startValues[] = { 30, 25, 20, 15, 10, 5 };
//   populate_2dtable_P(&knockWindowStartTable, startValues, startBins);

//   TEST_DATA_P uint8_t durationBins[] = { 30, 40, 50, 60, 70, 80 };
//   TEST_DATA_P uint8_t durationValues[] = { 30, 25, 20, 15, 10, 5 };
//   populate_2dtable_P(&knockWindowDurationTable, durationValues, durationBins);
}

static void test_correctionKnock_firstStep(void) {
    setup_correctionKnock();

    TEST_ASSERT_EQUAL(-11, correctionKnock(-8));
}

static void test_correctionKnock_disabled_modeoff(void) {
    setup_correctionKnock();
    configPage10.knock_mode = KNOCK_MODE_OFF;
    TEST_ASSERT_EQUAL(-8, correctionKnock(-8));
}

static void test_correctionKnock_disabled_counttoolow(void) {
    setup_correctionKnock();
    knockCounter = configPage10.knock_count - 1;
    TEST_ASSERT_EQUAL(-8, correctionKnock(-8));
}

static void test_correctionKnock_disabled_knockactive(void) {
    setup_correctionKnock();
    currentStatus.knockActive = true;
    TEST_ASSERT_EQUAL(-8, correctionKnock(-8));
}

static void test_correctionKnock(void) {
    RUN_TEST_P(test_correctionKnock_firstStep);
    RUN_TEST_P(test_correctionKnock_disabled_modeoff);
    RUN_TEST_P(test_correctionKnock_disabled_counttoolow);
    RUN_TEST_P(test_correctionKnock_disabled_knockactive);
}

static void setup_correctionsDwell(void) {
    construct2dTables();
    initialiseCorrections();
    
    configPage4.sparkDur = 10;
    configPage2.perToothIgn = false;
    configPage4.dwellErrCorrect = 0;
    configPage4.sparkMode = IGN_MODE_WASTED;

    currentStatus.actualDwell = 770;
    currentStatus.battery10 = 95;

    revolutionTime = 666;

    TEST_DATA_P uint8_t bins[] = { 60,  70,  80,  90,  100, 110 };
    TEST_DATA_P uint8_t values[] = { 130, 125, 120, 115, 110, 90 };
    populate_2dtable_P(&dwellVCorrectionTable, values, bins);   
}

static void test_correctionsDwell_nopertooth(void) {
    setup_correctionsDwell();

    currentStatus.battery10 = 105;
    configPage2.nCylinders = 8;

    configPage4.sparkMode = IGN_MODE_WASTED;
    TEST_ASSERT_EQUAL(296, correctionsDwell(800));

    configPage4.sparkMode = IGN_MODE_SINGLE;
    TEST_ASSERT_EQUAL(74, correctionsDwell(800));

    configPage4.sparkMode = IGN_MODE_ROTARY;
    configPage10.rotaryType = ROTARY_IGN_RX8;
    TEST_ASSERT_EQUAL(296, correctionsDwell(800));

    configPage4.sparkMode = IGN_MODE_ROTARY;
    configPage10.rotaryType = ROTARY_IGN_FC;
    TEST_ASSERT_EQUAL(74, correctionsDwell(800));
}

static void test_correctionsDwell_pertooth(void) {
    setup_correctionsDwell();

    currentStatus.battery10 = 105;
    configPage2.perToothIgn = true;
    configPage4.dwellErrCorrect = 1;
    configPage4.sparkMode = IGN_MODE_WASTED;

    currentStatus.actualDwell = 200;
    TEST_ASSERT_EQUAL(444, correctionsDwell(800));

    currentStatus.actualDwell = 1400;
    TEST_ASSERT_EQUAL(296, correctionsDwell(800));
}

static void test_correctionsDwell_wasted_nopertooth_largerevolutiontime(void) {
    setup_correctionsDwell();

    currentStatus.dwellCorrection = 55;
    currentStatus.battery10 = 105;
    revolutionTime = 5000;
    TEST_ASSERT_EQUAL(800, correctionsDwell(800));
}

static void test_correctionsDwell_initialises_current_actualDwell(void) {
    setup_correctionsDwell();

    currentStatus.actualDwell = 0;
    correctionsDwell(777);
    TEST_ASSERT_EQUAL(777, currentStatus.actualDwell);
}

static void test_correctionsDwell_sets_dwellCorrection(void) {
    setup_correctionsDwell();

    currentStatus.dwellCorrection = UINT8_MAX;
    currentStatus.battery10 = 90;
    correctionsDwell(777);
    TEST_ASSERT_EQUAL(115, currentStatus.dwellCorrection);
}

static void test_correctionsDwell_uses_batvcorrection(void) {
    setup_correctionsDwell();
    configPage2.nCylinders = 8;
    configPage4.sparkMode = IGN_MODE_WASTED;

    currentStatus.battery10 = 105;
    TEST_ASSERT_EQUAL(296, correctionsDwell(800));

    currentStatus.battery10 = 65;
    TEST_ASSERT_EQUAL(337, correctionsDwell(800));
}

static void test_correctionsDwell(void) {
    RUN_TEST_P(test_correctionsDwell_nopertooth);
    RUN_TEST_P(test_correctionsDwell_pertooth);
    RUN_TEST_P(test_correctionsDwell_wasted_nopertooth_largerevolutiontime);
    RUN_TEST_P(test_correctionsDwell_initialises_current_actualDwell);
    RUN_TEST_P(test_correctionsDwell_sets_dwellCorrection);
    RUN_TEST_P(test_correctionsDwell_uses_batvcorrection);
}

void testIgnCorrections(void) {
    test_correctionFixedTiming();
    test_correctionCLTadvance();
    test_correctionCrankingFixedTiming();
    test_correctionFlexTiming();
    test_correctionWMITiming();
    test_correctionIATretard();
    test_correctionIdleAdvance();
    test_correctionSoftRevLimit();
    test_correctionNitrous();
    test_correctionSoftLaunch();
    test_correctionSoftFlatShift();
    test_correctionKnock();
    // correctionDFCOignition() is tested in the fueling unit tests, since it is tightly coupled to fuel DFCO
    test_correctionsDwell();
}