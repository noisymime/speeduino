#include "globals.h"
#include "crankMaths.h"
#include "unity.h"
#include "decoders.h"

struct crankmaths_rev_testdata {
  uint16_t rpm;
  unsigned long revolutionTime;
  uint16_t angle;
  unsigned long expected;
} *crankmaths_rev_testdata_current;

struct crankmaths_tooth_testdata {
  uint16_t rpm;
  uint16_t triggerToothAngle;
  unsigned long toothTime;
  uint16_t angle;
  unsigned long expected;
} *crankmaths_tooth_testdata_current;

void test_crankmaths_angletotime_revolution_execute() {
  crankmaths_rev_testdata *testdata = crankmaths_rev_testdata_current;
  revolutionTime = testdata->revolutionTime;
  TEST_ASSERT_EQUAL(testdata->expected, angleToTime(testdata->angle, CRANKMATH_METHOD_INTERVAL_REV));
}

void test_crankmaths_angletotime_tooth_execute() {
  crankmaths_tooth_testdata *testdata = crankmaths_tooth_testdata_current;
  triggerToothAngle = testdata->triggerToothAngle;
  toothLastToothTime = toothLastMinusOneToothTime + testdata->toothTime;
  TEST_ASSERT_EQUAL(testdata->expected, angleToTime(testdata->angle, CRANKMATH_METHOD_INTERVAL_TOOTH));
}

void testCrankMaths()
{
  const byte testNameLength = 200;
  char testName[testNameLength];

  const crankmaths_rev_testdata crankmaths_rev_testdatas[] = {
    { .rpm = 50,    .revolutionTime = 1200000, .angle = 0,   .expected = 0 },
    { .rpm = 50,    .revolutionTime = 1200000, .angle = 25,  .expected = 83333 }, // 83333,3333
    { .rpm = 50,    .revolutionTime = 1200000, .angle = 720, .expected = 2400000 },
    { .rpm = 2500,  .revolutionTime = 24000,   .angle = 0,   .expected = 0 },
    { .rpm = 2500,  .revolutionTime = 24000,   .angle = 25,  .expected = 1666 }, // 1666,6666
    { .rpm = 2500,  .revolutionTime = 24000,   .angle = 720, .expected = 48000 },
    { .rpm = 20000, .revolutionTime = 3000,    .angle = 0,   .expected = 0 },
    { .rpm = 20000, .revolutionTime = 3000,    .angle = 25,  .expected = 208 }, // 208,3333
    { .rpm = 20000, .revolutionTime = 3000,    .angle = 720, .expected = 6000 }
  };

  for (auto testdata : crankmaths_rev_testdatas) {
    crankmaths_rev_testdata_current = &testdata;
    snprintf(testName, testNameLength, "crankmaths/angletotime/revolution/%urpm/%uangle", testdata.rpm, testdata.angle);
    UnityDefaultTestRun(test_crankmaths_angletotime_revolution_execute, testName, __LINE__);
  }

  const crankmaths_tooth_testdata crankmaths_tooth_testdatas[] = {
    { .rpm = 50,    .triggerToothAngle = 3,   .toothTime = 10000,  .angle = 0,   .expected = 0 },
    { .rpm = 50,    .triggerToothAngle = 3,   .toothTime = 10000,  .angle = 25,  .expected = 83333 }, // 83333,3333
    { .rpm = 50,    .triggerToothAngle = 3,   .toothTime = 10000,  .angle = 720, .expected = 2400000 },
    { .rpm = 2500,  .triggerToothAngle = 3,   .toothTime = 200,    .angle = 0,   .expected = 0 },
    { .rpm = 2500,  .triggerToothAngle = 3,   .toothTime = 200,    .angle = 25,  .expected = 1666 }, // 1666,6666
    { .rpm = 2500,  .triggerToothAngle = 3,   .toothTime = 200,    .angle = 720, .expected = 48000 },
    { .rpm = 20000, .triggerToothAngle = 3,   .toothTime = 25,     .angle = 0,   .expected = 0 },
    { .rpm = 20000, .triggerToothAngle = 3,   .toothTime = 25,     .angle = 25,  .expected = 208 }, // 208,3333
    { .rpm = 20000, .triggerToothAngle = 3,   .toothTime = 25,     .angle = 720, .expected = 6000 },
    { .rpm = 50,    .triggerToothAngle = 180, .toothTime = 600000, .angle = 0,   .expected = 0 },
    { .rpm = 50,    .triggerToothAngle = 180, .toothTime = 600000, .angle = 25,  .expected = 83333 }, // 83333,3333
    { .rpm = 50,    .triggerToothAngle = 180, .toothTime = 600000, .angle = 720, .expected = 2400000 },
    { .rpm = 2500,  .triggerToothAngle = 180, .toothTime = 12000,  .angle = 0,   .expected = 0 },
    { .rpm = 2500,  .triggerToothAngle = 180, .toothTime = 12000,  .angle = 25,  .expected = 1666 }, // 1666,6666
    { .rpm = 2500,  .triggerToothAngle = 180, .toothTime = 12000,  .angle = 720, .expected = 48000 },
    { .rpm = 20000, .triggerToothAngle = 180, .toothTime = 1500,   .angle = 0,   .expected = 0 },
    { .rpm = 20000, .triggerToothAngle = 180, .toothTime = 1500,   .angle = 25,  .expected = 208 }, // 208,3333
    { .rpm = 20000, .triggerToothAngle = 180, .toothTime = 1500,   .angle = 720, .expected = 6000 },
  };
  // The same for all tests
  triggerToothAngleIsCorrect = true;
  toothLastMinusOneToothTime = 200000;

  for (auto testdata : crankmaths_tooth_testdatas) {
    crankmaths_tooth_testdata_current = &testdata;
    snprintf(testName, testNameLength, "crankmaths/angletotime/tooth/%urpm/%uangle/%utoothangle", testdata.rpm, testdata.angle, testdata.triggerToothAngle);
    UnityDefaultTestRun(test_crankmaths_angletotime_tooth_execute, testName, __LINE__);
  }

}