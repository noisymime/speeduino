#include <Unity.h>
#include <avr/pgmspace.h>
#include "secondaryTables.h"
#include "globals.h"
#include "../test_utils.h"
#include "storage.h"


#if defined(PROGMEM)
static const table3d_value_t values[] PROGMEM = {
#else
static const table3d_value_t values[] = {
#endif
 //0    1    2   3     4    5    6    7    8    9   10   11   12   13    14   15
34,  34,  34,  34,  34,  34,  34,  34,  34,  35,  35,  35,  35,  35,  35,  35, 
34,  35,  36,  37,  39,  41,  42,  43,  43,  44,  44,  44,  44,  44,  44,  44, 
35,  36,  38,  41,  44,  46,  47,  48,  48,  49,  49,  49,  49,  49,  49,  49, 
36,  39,  42,  46,  50,  51,  52,  53,  53,  53,  53,  53,  53,  53,  53,  53, 
38,  43,  48,  52,  55,  56,  57,  58,  58,  58,  58,  58,  58,  58,  58,  58, 
42,  49,  54,  58,  61,  62,  62,  63,  63,  63,  63,  63,  63,  63,  63,  63, 
48,  56,  60,  64,  66,  66,  68,  68,  68,  68,  68,  68,  68,  68,  68,  68, 
54,  62,  66,  69,  71,  71,  72,  72,  72,  72,  72,  72,  72,  72,  72,  72, 
61,  69,  72,  74,  76,  76,  77,  77,  77,  77,  77,  77,  77,  77,  77,  77, 
68,  75,  78,  79,  81,  81,  81,  82,  82,  82,  82,  82,  82,  82,  82,  82, 
74,  80,  83,  84,  85,  86,  86,  86,  87,  87,  87,  87,  87,  87,  87,  87, 
81,  86,  88,  89,  90,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91,  91, 
93,  96,  98,  99,  99,  107, 107, 101, 101, 101, 101, 101, 101, 101, 101, 101, 
98,  101, 103, 103, 104, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 105, 
104, 106, 107, 108, 109, 109, 110, 110, 110, 110, 110, 110, 110, 110, 110, 110, 
150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150,
  };
static const table3d_axis_t tempXAxis[] = {500, 700, 900, 1200, 1600, 2000, 2500, 3100, 3500, 4100, 4700, 5300, 5900, 6500, 6750, 7000};
static const table3d_axis_t tempYAxis[] = {16, 26, 30, 36, 40, 46, 50, 56, 60, 66, 70, 76, 86, 90, 96, 100};

static void __attribute__((noinline)) assert_2nd_spark_is_off(int8_t expectedAdvance) {
    TEST_ASSERT_BIT_LOW(BIT_SPARK2_SPARK2_ACTIVE, currentStatus.spark2);
    TEST_ASSERT_EQUAL(expectedAdvance, currentStatus.advance1);
    TEST_ASSERT_EQUAL(0, currentStatus.advance2);
    TEST_ASSERT_EQUAL(currentStatus.advance1, currentStatus.advance);
    TEST_ASSERT_EQUAL(0, currentStatus.ignLoad2);
} 

static void __attribute__((noinline)) assert_2nd_spark_is_on(int8_t expectedAdvance1, int8_t expectedAdvance2, int8_t expectedAdvance) {
    TEST_ASSERT_BIT_HIGH(BIT_SPARK2_SPARK2_ACTIVE, currentStatus.spark2);
    TEST_ASSERT_EQUAL(expectedAdvance1, currentStatus.advance1);
    TEST_ASSERT_EQUAL(expectedAdvance2, currentStatus.advance2);
    TEST_ASSERT_EQUAL(expectedAdvance, currentStatus.advance);
    // All tests in this file use MAP as the load source
    TEST_ASSERT_EQUAL(currentStatus.MAP, currentStatus.ignLoad2);
} 

static void test_setup(void) {
    resetConfigPages();
    memset(&currentStatus, 0, sizeof(currentStatus));
}

static void test_no_secondary_spark(void) {
    test_setup();
    configPage10.spark2Mode = SPARK2_MODE_OFF;
    configPage10.spark2Algorithm = LOAD_SOURCE_MAP;
    currentStatus.advance1 = 50;
    currentStatus.advance = currentStatus.advance1;

    calculateSecondarySpark();

    assert_2nd_spark_is_off(50);
}

static void __attribute__((noinline)) test_mode_cap_INT8_MAX(uint8_t mode) {
    test_setup();
    configPage10.spark2Mode = mode;    
    configPage10.spark2Algorithm = LOAD_SOURCE_MAP;
    currentStatus.advance1 = 120;
    currentStatus.advance = currentStatus.advance1;
    currentStatus.MAP = 100; //Load source value
    currentStatus.RPM = 7000;

    calculateSecondarySpark();

    assert_2nd_spark_is_on(120, (int16_t)150-INT16_C(OFFSET_IGNITION), INT8_MAX);
}

static constexpr int8_t SIMPLE_ADVANCE1 = 75;
static constexpr int16_t SIMPLE_LOAD_LOOKUP_VALUE = 68;
static constexpr int16_t SIMPLE_LOAD_VALUE = SIMPLE_LOAD_LOOKUP_VALUE-INT16_C(OFFSET_IGNITION);

static void __attribute__((noinline)) test_mode_simple(uint8_t mode, int8_t expectedAdvance) {
    test_setup();
    configPage10.spark2Mode = mode;    
    configPage10.spark2Algorithm = LOAD_SOURCE_MAP;
    currentStatus.advance1 = SIMPLE_ADVANCE1;
    currentStatus.advance = currentStatus.advance1;
    currentStatus.MAP = 50; //Load source value
    currentStatus.RPM = 3500;

    calculateSecondarySpark();

    assert_2nd_spark_is_on(SIMPLE_ADVANCE1, SIMPLE_LOAD_VALUE, expectedAdvance);
}

static void test_sparkmode_multiply_cap_UINT8_MAX(void) {
    test_mode_cap_INT8_MAX(SPARK2_MODE_MULTIPLY);
}

static void test_sparkmode_multiply(void) {
    test_mode_simple(SPARK2_MODE_MULTIPLY, (SIMPLE_ADVANCE1*(SIMPLE_LOAD_VALUE+OFFSET_IGNITION))/100);
}

static void test_sparkmode_add(void) {
    test_mode_simple(SPARK2_MODE_ADD, SIMPLE_ADVANCE1+SIMPLE_LOAD_VALUE);
}

static void test_sparkmode_add_cap_UINT8_MAX(void) {
    test_mode_cap_INT8_MAX(SPARK2_MODE_ADD);
}

static void __attribute__((noinline)) setup_test_mode_cond_switch(uint8_t cond, uint16_t trigger) {
    configPage10.spark2Mode = SPARK2_MODE_CONDITIONAL_SWITCH; 
    configPage10.spark2SwitchVariable = cond;
    configPage10.spark2SwitchValue = trigger;
    configPage10.spark2Algorithm = LOAD_SOURCE_MAP;    
    currentStatus.advance1 = SIMPLE_ADVANCE1;
    currentStatus.advance = currentStatus.advance1;
    currentStatus.MAP = 50; //Load source value
    currentStatus.RPM = 3500;
    currentStatus.TPS = 50;
    currentStatus.ethanolPct = 50;
}

static void __attribute__((noinline)) test_sparkmode_cond_switch_negative(uint8_t cond, uint16_t trigger) {
    setup_test_mode_cond_switch(cond, trigger);

    calculateSecondarySpark();

    assert_2nd_spark_is_off(SIMPLE_ADVANCE1);    
}

static void __attribute__((noinline)) test_sparkmode_cond_switch_positive(uint8_t cond, uint16_t trigger) {
    setup_test_mode_cond_switch(cond, trigger);

    calculateSecondarySpark();

    assert_2nd_spark_is_on(SIMPLE_ADVANCE1, SIMPLE_LOAD_VALUE, SIMPLE_LOAD_VALUE);
}

static void __attribute__((noinline)) test_sparkmode_cond_switch_rpm(void) {
    test_setup();
    test_sparkmode_cond_switch_positive(SPARK2_CONDITION_RPM, 3499);    
    test_sparkmode_cond_switch_negative(SPARK2_CONDITION_RPM, 3501);    
    test_sparkmode_cond_switch_positive(SPARK2_CONDITION_RPM, 3499);    
}

static void __attribute__((noinline)) test_sparkmode_cond_switch_tps(void) {
    test_setup();
    test_sparkmode_cond_switch_positive(SPARK2_CONDITION_TPS, 49);    
    test_sparkmode_cond_switch_negative(SPARK2_CONDITION_TPS, 51);    
    test_sparkmode_cond_switch_positive(SPARK2_CONDITION_TPS, 49);    
}

static void __attribute__((noinline)) test_sparkmode_cond_switch_map(void) {
    test_setup();
    test_sparkmode_cond_switch_positive(SPARK2_CONDITION_MAP, 49);    
    test_sparkmode_cond_switch_negative(SPARK2_CONDITION_MAP, 51);    
    test_sparkmode_cond_switch_positive(SPARK2_CONDITION_MAP, 49);    
}

static void __attribute__((noinline)) test_sparkmode_cond_switch_ethanol_pct(void) {
    test_setup();
    test_sparkmode_cond_switch_positive(SPARK2_CONDITION_ETH, 49);    
    test_sparkmode_cond_switch_negative(SPARK2_CONDITION_ETH, 51);    
    test_sparkmode_cond_switch_positive(SPARK2_CONDITION_ETH, 49);    
}

static void __attribute__((noinline)) test_sparkmode_input_switch(void) {
    test_setup();
    configPage10.spark2Mode = SPARK2_MODE_INPUT_SWITCH; 
    configPage10.spark2Algorithm = LOAD_SOURCE_MAP;
    currentStatus.advance1 = SIMPLE_ADVANCE1;
    currentStatus.advance = currentStatus.advance1;
    currentStatus.MAP = 50; //Load source value
    currentStatus.RPM = 3500;
    configPage10.spark2InputPolarity = HIGH;
    pinSpark2Input = 3;   
    pinMode(pinSpark2Input, OUTPUT);

    // On
    digitalWrite(pinSpark2Input, configPage10.spark2InputPolarity);
    calculateSecondarySpark();
    assert_2nd_spark_is_on(SIMPLE_ADVANCE1, SIMPLE_LOAD_VALUE, SIMPLE_LOAD_VALUE);

    // Off
    digitalWrite(pinSpark2Input, !configPage10.spark2InputPolarity);
    currentStatus.advance = currentStatus.advance1;
    calculateSecondarySpark();
    assert_2nd_spark_is_off(SIMPLE_ADVANCE1);

    // On again
    digitalWrite(pinSpark2Input, configPage10.spark2InputPolarity);
    calculateSecondarySpark();
    assert_2nd_spark_is_on(SIMPLE_ADVANCE1, SIMPLE_LOAD_VALUE, SIMPLE_LOAD_VALUE);
}

void test_calculateSecondarySpark(void)
{
    populateTable(ignitionTable2, values, tempXAxis, tempYAxis);

    RUN_TEST(test_no_secondary_spark);
    RUN_TEST(test_sparkmode_multiply);
    RUN_TEST(test_sparkmode_multiply_cap_UINT8_MAX);
    RUN_TEST(test_sparkmode_add);
    RUN_TEST(test_sparkmode_add_cap_UINT8_MAX);
    RUN_TEST(test_sparkmode_cond_switch_rpm);
    RUN_TEST(test_sparkmode_cond_switch_tps);
    RUN_TEST(test_sparkmode_cond_switch_map);
    RUN_TEST(test_sparkmode_cond_switch_ethanol_pct);
    RUN_TEST(test_sparkmode_input_switch);
}