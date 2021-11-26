//#include <Arduino.h>
#include <string.h> // memcpy
#include <unity.h>
#include <stdio.h>
#include "tests_tables.h"
#include "table3d.h"

#if defined(PROGMEM)
const PROGMEM byte values[] = {
#else
const byte values[] = {
#endif
  109, 111, 112, 113, 114, 114, 114, 115, 115, 115, 114, 114, 113, 112, 111, 111,
  104, 106, 107, 108, 109, 109, 110, 110, 110, 110, 110, 109, 108, 107, 107, 106,
  98,  101, 103, 103, 104, 105, 105, 105, 105, 105, 105, 104, 104, 103, 102, 102,
  93,  96,  98,  99,  99,  100, 100, 101, 101, 101, 100, 100, 99,  98,  98,  97,
  81,  86,  88,  89,  90,  91,  91,  91,  91,  91,  91,  90,  90,  89,  89,  88,
  74,  80,  83,  84,  85,  86,  86,  86,  87,  86,  86,  86,  85,  84,  84,  84,
  68,  75,  78,  79,  81,  81,  81,  82,  82,  82,  82,  81,  81,  80,  79,  79,
  61,  69,  72,  74,  76,  76,  77,  77,  77,  77,  77,  76,  76,  75,  75,  74,
  54,  62,  66,  69,  71,  71,  72,  72,  72,  72,  72,  72,  71,  71,  70,  70,
  48,  56,  60,  64,  66,  66,  68,  68,  68,  68,  67,  67,  67,  66,  66,  65,
  42,  49,  54,  58,  61,  62,  62,  63,  63,  63,  63,  62,  62,  61,  61,  61,
  38,  43,  48,  52,  55,  56,  57,  58,  58,  58,  58,  58,  57,  57,  57,  56,
  36,  39,  42,  46,  50,  51,  52,  53,  53,  53,  53,  53,  53,  52,  52,  52,
  35,  36,  38,  41,  44,  46,  47,  48,  48,  49,  48,  48,  48,  48,  47,  47,
  34,  35,  36,  37,  39,  41,  42,  43,  43,  44,  44,  44,  43,  43,  43,  43,
  34,  34,  34,  34,  34,  34,  34,  34,  34,  35,  34,  34,  34,  34,  34,  34
  };

static table3d16RpmLoad testTable;

void setup_TestTable(void)
{
  //Setup the fuel table with some sane values for testing
  //Table is setup per the below
  /*
  100 |  109 |  111 |  112 |  113 |  114 |  114 |  114 |  115 |  115 |  115 |  114 |  114 |  113 |  112 |  111 |  111
  96  |  104 |  106 |  107 |  108 |  109 |  109 |  110 |  110 |  110 |  110 |  110 |  109 |  108 |  107 |  107 |  106
  90  |   98 |  101 |  103 |  103 |  104 |  105 |  105 |  105 |  105 |  105 |  105 |  104 |  104 |  103 |  102 |  102
  86  |   93 |   96 |   98 |   99 |   99 |  100 |  100 |  101 |  101 |  101 |  100 |  100 |   99 |   98 |   98 |   97
  76  |   81 |   86 |   88 |   89 |   90 |   91 |   91 |   91 |   91 |   91 |   91 |   90 |   90 |   89 |   89 |   88
  70  |   74 |   80 |   83 |   84 |   85 |   86 |   86 |   86 |   87 |   86 |   86 |   86 |   85 |   84 |   84 |   84
  65  |   68 |   75 |   78 |   79 |   81 |   81 |   81 |   82 |   82 |   82 |   82 |   81 |   81 |   80 |   79 |   79
  60  |   61 |   69 |   72 |   74 |   76 |   76 |   77 |   77 |   77 |   77 |   77 |   76 |   76 |   75 |   75 |   74
  56  |   54 |   62 |   66 |   69 |   71 |   71 |   72 |   72 |   72 |   72 |   72 |   72 |   71 |   71 |   70 |   70
  50  |   48 |   56 |   60 |   64 |   66 |   66 |   68 |   68 |   68 |   68 |   67 |   67 |   67 |   66 |   66 |   65
  46  |   42 |   49 |   54 |   58 |   61 |   62 |   62 |   63 |   63 |   63 |   63 |   62 |   62 |   61 |   61 |   61
  40  |   38 |   43 |   48 |   52 |   55 |   56 |   57 |   58 |   58 |   58 |   58 |   58 |   57 |   57 |   57 |   56
  36  |   36 |   39 |   42 |   46 |   50 |   51 |   52 |   53 |   53 |   53 |   53 |   53 |   53 |   52 |   52 |   52
  30  |   35 |   36 |   38 |   41 |   44 |   46 |   47 |   48 |   48 |   49 |   48 |   48 |   48 |   48 |   47 |   47
  26  |   34 |   35 |   36 |   37 |   39 |   41 |   42 |   43 |   43 |   44 |   44 |   44 |   43 |   43 |   43 |   43
  16  |   34 |   34 |   34 |   34 |   34 |   34 |   34 |   34 |   34 |   35 |   34 |   34 |   34 |   34 |   34 |   34
      ----------------------------------------------------------------------------------------------------------------
         500 |  700 |  900 | 1200 | 1600 | 2000 | 2500 | 3100 | 3500 | 4100 | 4700 | 5300 | 5900 | 6500 | 6750 | 7000
  */
  
  table3d_axis_t tempXAxis[] = {500,700, 900, 1200, 1600, 2000, 2500, 3100, 3500, 4100, 4700, 5300, 5900, 6500, 6750, 7000};
  memcpy(testTable.axisX.axis, tempXAxis, sizeof(testTable.axisX));
  table3d_axis_t tempYAxis[] = {100, 96, 90, 86, 76, 70, 66, 60, 56, 50, 46, 40, 36, 30, 26, 16};
  memcpy(testTable.axisY.axis, tempYAxis, sizeof(testTable.axisY));

#if defined(PROGMEM)
  for (int loop=0; loop<sizeof(testTable.values); ++loop) 
  { 
    testTable.values.values[loop] = pgm_read_byte_near(values + loop);
  }
#else
  memcpy(testTable.values.values, values, sizeof(testTable.values));
#endif
}

void testTables()
{
  RUN_TEST(test_tableLookup_50pct);
  RUN_TEST(test_tableLookup_exact1Axis);
  RUN_TEST(test_tableLookup_exact2Axis);
  RUN_TEST(test_tableLookup_overMaxX);
  RUN_TEST(test_tableLookup_overMaxY);
  RUN_TEST(test_tableLookup_underMinX);
  RUN_TEST(test_tableLookup_underMinY);
  RUN_TEST(test_tableLookup_roundUp);
  //RUN_TEST(test_all_incrementing);
  
}

void test_tableLookup_50pct(void)
{
  //Tests a lookup that is exactly 50% of the way between cells on both the X and Y axis
  setup_TestTable();

  uint16_t tempVE = get3DTableValue(&testTable, 53, 2250); //Perform lookup into fuel map for RPM vs MAP value
  TEST_ASSERT_EQUAL(tempVE, 69);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMax, (table3d_dim_t)6);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMin, (table3d_dim_t)5);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMax, (table3d_dim_t)9);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMin, (table3d_dim_t)8);
}

void test_tableLookup_exact1Axis(void)
{
  //Tests a lookup that exactly matches on the X axis and 50% of the way between cells on the Y axis
  setup_TestTable();

  uint16_t tempVE = get3DTableValue(&testTable, 48, 2500); //Perform lookup into fuel map for RPM vs MAP value
  TEST_ASSERT_EQUAL(tempVE, 65);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMax, (table3d_dim_t)6);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMin, (table3d_dim_t)5);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMax, (table3d_dim_t)10);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMin, (table3d_dim_t)9);
}

void test_tableLookup_exact2Axis(void)
{
  //Tests a lookup that exactly matches on both the X and Y axis
  setup_TestTable();

  uint16_t tempVE = get3DTableValue(&testTable, 70, 2500); //Perform lookup into fuel map for RPM vs MAP value
  TEST_ASSERT_EQUAL(tempVE, 86);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMax, (table3d_dim_t)6);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMin, (table3d_dim_t)5);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMax, (table3d_dim_t)5);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMin, (table3d_dim_t)5);
}

void test_tableLookup_overMaxX(void)
{
  //Tests a lookup where the RPM exceeds the highest value in the table. The Y value is a 50% match
  setup_TestTable();

  uint16_t tempVE = get3DTableValue(&testTable, 73, 10000); //Perform lookup into fuel map for RPM vs MAP value
  TEST_ASSERT_EQUAL(tempVE, 86);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMax, (table3d_dim_t)15);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMin, (table3d_dim_t)15);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMax, (table3d_dim_t)5);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMin, (table3d_dim_t)4);
}

void test_tableLookup_overMaxY(void)
{
  //Tests a lookup where the load value exceeds the highest value in the table. The X value is a 50% match
  setup_TestTable();

  uint16_t tempVE = get3DTableValue(&testTable, 110, 600); //Perform lookup into fuel map for RPM vs MAP value
  TEST_ASSERT_EQUAL(tempVE, 110);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMax, (table3d_dim_t)1);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMin, (table3d_dim_t)0);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMax, (table3d_dim_t)0);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMin, (table3d_dim_t)0);
}

void test_tableLookup_underMinX(void)
{
  //Tests a lookup where the RPM value is below the lowest value in the table. The Y value is a 50% match
  setup_TestTable();

  uint16_t tempVE = get3DTableValue(&testTable, 38, 300); //Perform lookup into fuel map for RPM vs MAP value
  TEST_ASSERT_EQUAL(tempVE, 37);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMax, (table3d_dim_t)0);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMin, (table3d_dim_t)0);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMax, (table3d_dim_t)12);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMin, (table3d_dim_t)11);
}

void test_tableLookup_underMinY(void)
{
  //Tests a lookup where the load value is below the lowest value in the table. The X value is a 50% match
  setup_TestTable();

  uint16_t tempVE = get3DTableValue(&testTable, 8, 600); //Perform lookup into fuel map for RPM vs MAP value
  TEST_ASSERT_EQUAL(tempVE, 34);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMax, (table3d_dim_t)1);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMin, (table3d_dim_t)0);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMax, (table3d_dim_t)15);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMin, (table3d_dim_t)15);
}

void test_tableLookup_roundUp(void)
{
  // Tests a lookup where the inputs result in a value that is outside the table range
  // due to fixed point rounding
  // Issue #726
  setup_TestTable();

  uint16_t tempVE = get3DTableValue(&testTable, 17, 600);
  // TEST_ASSERT_EQUAL(tempVE, 34);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMax, (table3d_dim_t)1);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastXMin, (table3d_dim_t)0);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMax, (table3d_dim_t)15);
  TEST_ASSERT_EQUAL(testTable.get_value_cache.lastYMin, (table3d_dim_t)14);
}

void test_all_incrementing(void)
{
  //Test the when going up both the load and RPM axis that the returned value is always equal or higher to the previous one
  //Tests all combinations of load/rpm from between 0-200 load and 0-9000 rpm
  //WARNING: This can take a LONG time to run. It is disabled by default for this reason
  uint16_t tempVE = 0;
  
  char buffer[64];
  for(uint16_t rpm = 0; rpm<8000; rpm+=100)
  {
    tempVE = 0;
    for(uint8_t load = 0; load<120; load++)
    {
      sprintf(buffer, "%d, %d", rpm, load);
      TEST_MESSAGE(buffer);
      uint16_t newVE = get3DTableValue(&testTable, load, rpm);
      TEST_ASSERT_GREATER_OR_EQUAL(tempVE, newVE);
      tempVE = newVE;
    }
  }
}