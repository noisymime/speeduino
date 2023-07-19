#include <string.h> // memcpy
#include <unity.h>
#include <stdio.h>
typedef uint8_t byte;
#include "test_table2d.h"
#include "table2d.h"


static constexpr uint8_t TEST_TABLE2D_SIZE = 9;
static uint8_t table2d_data_u8[TEST_TABLE2D_SIZE] = {
    251, 211, 199, 167, 127, 101, 59, 23, 5
};
static int16_t table2d_data_s16[TEST_TABLE2D_SIZE] = {
    32029, 26357, 21323, 16363, 11329, 7537, 5531, 2539, 1237
};

static uint8_t table2d_axis_u8[TEST_TABLE2D_SIZE] {
    5, 23, 59, 101, 127, 167, 199, 211, 251,
};
static int16_t table2d_axis_s16[TEST_TABLE2D_SIZE] = {
    123, 2539, 5531, 7537, 11329, 16363, 21323, 26357, 32029,
};

static table2D table2d_u8_u8;
static table2D table2d_u8_s16;
static table2D table2d_s16_u8;
static table2D table2d_s16_s16;

template <typename dataT, typename axisT>
void setup_test_subject(table2D &table, dataT *data, axisT *axis)
{
    table.valueSize = sizeof(dataT)*CHAR_BIT;
    table.axisSize = sizeof(axisT)*CHAR_BIT;
    table.xSize = TEST_TABLE2D_SIZE;
    table.values = data;
    table.axisX = axis;
}

static void setup_test_subjects(void)
{
    setup_test_subject(table2d_u8_u8, table2d_data_u8, table2d_axis_u8);
    setup_test_subject(table2d_u8_s16, table2d_data_u8, table2d_axis_s16);
    setup_test_subject(table2d_s16_u8, table2d_data_s16, table2d_axis_u8);
    setup_test_subject(table2d_s16_s16, table2d_data_s16, table2d_axis_s16);
}


void test_table2dLookup_50pct(void)
{
    setup_test_subjects();

    uint8_t u8_u8_result = table2D_getValue(&table2d_u8_u8, table2d_axis_u8[3]+((table2d_axis_u8[4]-table2d_axis_u8[3])/2));
    TEST_ASSERT_EQUAL(147, u8_u8_result);
    TEST_ASSERT_EQUAL(4, table2d_u8_u8.lastXMax);

    uint8_t u8_s16_result = table2D_getValue(&table2d_u8_s16, table2d_axis_s16[6]+((table2d_axis_s16[7]-table2d_axis_s16[6])/2));
    TEST_ASSERT_EQUAL(41, u8_s16_result);
    TEST_ASSERT_EQUAL(7, table2d_u8_s16.lastXMax);

    int16_t s16_u8_result = table2D_getValue(&table2d_s16_u8, table2d_axis_u8[3]+((table2d_axis_u8[4]-table2d_axis_u8[3])/2));
    TEST_ASSERT_EQUAL(13846, s16_u8_result);
    TEST_ASSERT_EQUAL(4, table2d_s16_u8.lastXMax);

    int16_t s16_s16_result = table2D_getValue(&table2d_s16_s16, table2d_axis_s16[3]+((table2d_axis_s16[4]-table2d_axis_s16[3])/2));
    TEST_ASSERT_EQUAL(13846, s16_s16_result);
    TEST_ASSERT_EQUAL(4, table2d_s16_s16.lastXMax);
}


void test_table2dLookup_exactAxis(void)
{
    setup_test_subjects();

    uint8_t u8_u8_result = table2D_getValue(&table2d_u8_u8, table2d_axis_u8[7]);
    TEST_ASSERT_EQUAL(23, u8_u8_result);
    // TEST_ASSERT_EQUAL(7, table2d_u8_u8.lastXMax);

    uint8_t u8_s16_result = table2D_getValue(&table2d_u8_s16, table2d_axis_s16[1]);
    TEST_ASSERT_EQUAL(211, u8_s16_result);
    // TEST_ASSERT_EQUAL(7, table2d_u8_s16.lastXMax);

    int16_t s16_u8_result = table2D_getValue(&table2d_s16_u8, table2d_axis_u8[2]);
    TEST_ASSERT_EQUAL(21323, s16_u8_result);
    // TEST_ASSERT_EQUAL(4, table2d_s16_u8.lastXMax);

    int16_t s16_s16_result = table2D_getValue(&table2d_s16_s16, table2d_axis_s16[5]);
    TEST_ASSERT_EQUAL(7537, s16_s16_result);
    // TEST_ASSERT_EQUAL(4, table2d_s16_s16.lastXMax);    
}

void test_table2dLookup_overMax(void)
{
    setup_test_subjects();

    uint8_t u8_u8_result = table2D_getValue(&table2d_u8_u8, table2d_axis_u8[TEST_TABLE2D_SIZE-1]+1);
    TEST_ASSERT_EQUAL(5, u8_u8_result);
    // TEST_ASSERT_EQUAL(8, table2d_u8_u8.lastXMax);

    uint8_t u8_s16_result = table2D_getValue(&table2d_u8_s16, table2d_axis_s16[TEST_TABLE2D_SIZE-1]+1);
    TEST_ASSERT_EQUAL(5, u8_s16_result);
    // TEST_ASSERT_EQUAL(8, table2d_u8_s16.lastXMax);

    int16_t s16_u8_result = table2D_getValue(&table2d_s16_u8, table2d_axis_u8[TEST_TABLE2D_SIZE-1]+1);
    TEST_ASSERT_EQUAL(1237, s16_u8_result);
    // TEST_ASSERT_EQUAL(8, table2d_s16_u8.lastXMax);

    int16_t s16_s16_result = table2D_getValue(&table2d_s16_s16, table2d_axis_s16[TEST_TABLE2D_SIZE-1]+1);
    TEST_ASSERT_EQUAL(1237, s16_s16_result);
    // TEST_ASSERT_EQUAL(8, table2d_s16_s16.lastXMax);    
}

void test_table2dLookup_underMin(void)
{
    setup_test_subjects();

    uint8_t u8_u8_result = table2D_getValue(&table2d_u8_u8, table2d_axis_u8[0]-1);
    TEST_ASSERT_EQUAL(251, u8_u8_result);
    // TEST_ASSERT_EQUAL(0, table2d_u8_u8.lastXMax);

    uint8_t u8_s16_result = table2D_getValue(&table2d_u8_s16, table2d_axis_s16[0]-1);
    TEST_ASSERT_EQUAL(251, u8_s16_result);
    // TEST_ASSERT_EQUAL(0, table2d_u8_s16.lastXMax);

    int16_t s16_u8_result = table2D_getValue(&table2d_s16_u8, table2d_axis_u8[0]-1);
    TEST_ASSERT_EQUAL(32029, s16_u8_result);
    // TEST_ASSERT_EQUAL(0, table2d_s16_u8.lastXMax);

    int16_t s16_s16_result = table2D_getValue(&table2d_s16_s16, table2d_axis_s16[0]-1);
    TEST_ASSERT_EQUAL(32029, s16_s16_result);
    // TEST_ASSERT_EQUAL(0, table2d_s16_s16.lastXMax);       
}


void test_table2d_all_decrementing(void)
{
    setup_test_subjects();

    uint8_t u8_u8_result_last = UINT8_MAX;
    for (uint8_t loop=table2d_axis_u8[0]; loop<=table2d_axis_u8[TEST_TABLE2D_SIZE-1]; ++loop)
    {
        uint8_t u8_u8_result = table2D_getValue(&table2d_u8_u8, loop);
        TEST_ASSERT_LESS_OR_EQUAL(u8_u8_result_last, u8_u8_result);
        u8_u8_result_last = u8_u8_result;
    }
}


void testTable2d()
{
    RUN_TEST(test_table2dLookup_50pct);
    RUN_TEST(test_table2dLookup_exactAxis);
    RUN_TEST(test_table2dLookup_overMax);
    RUN_TEST(test_table2dLookup_underMin);
    RUN_TEST(test_table2d_all_decrementing); 
}