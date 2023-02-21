#include <Arduino.h>
#include <unity.h>
#include "test_calcs_common.h"
#include "schedule_calcs.h"
#include "crankMaths.h"

#define _countof(x) (sizeof(x) / sizeof (x[0]))

extern volatile uint16_t degreesPeruSx2048;

constexpr uint16_t DWELL_TIME_MS = 4;

uint16_t dwellAngle;
void setEngineSpeed(uint16_t rpm, int16_t max_crank) {
    timePerDegreex16 = ldiv( 2666656L, rpm).quot; //The use of a x16 value gives accuracy down to 0.1 of a degree and can provide noticeably better timing results on low resolution triggers
    timePerDegree = timePerDegreex16 / 16; 
    degreesPeruSx2048 = 2048 / timePerDegree;
    degreesPeruSx32768 = 524288L / timePerDegreex16;       
    revolutionTime =  (60L*1000000L) / rpm;
    CRANK_ANGLE_MAX_IGN = max_crank;
    CRANK_ANGLE_MAX_INJ = max_crank;
    dwellAngle = timeToAngle(DWELL_TIME_MS*1000UL, CRANKMATH_METHOD_INTERVAL_REV);
}

void test_calc_ign1_timeout(uint16_t crankAngle, uint16_t pending, uint16_t running, int16_t expectedEndAngle)
{
    memset(&ignitionSchedule1, 0, sizeof(ignitionSchedule1));

    ignitionSchedule1.Status = PENDING;
    calculateIgnitionAngle1(dwellAngle);
    TEST_ASSERT_EQUAL(expectedEndAngle, ignition1EndAngle);
    TEST_ASSERT_EQUAL(pending, calculateIgnition1Timeout(crankAngle));
    
    ignitionSchedule1.Status = RUNNING;
    calculateIgnitionAngle1(dwellAngle);
    TEST_ASSERT_EQUAL(running, calculateIgnition1Timeout(crankAngle));    
}

struct ign_test_parameters
{
    uint16_t channelAngle;  // deg
    int8_t advanceAngle;  // deg
    uint16_t crankAngle;    // deg
    uint32_t pending;       // Expected delay when channel status is PENDING
    uint32_t running;       // Expected delay when channel status is RUNNING
    int16_t endAngle;      // Expected end angle
};
void test_calc_ignN_timeout(Schedule &schedule, uint16_t channelDegrees, const int &startAngle, void (*pEndAngleCalc)(int), uint16_t crankAngle, uint32_t pending, uint32_t running, int16_t expectedEndAngle, const int16_t &endAngle)
{
    memset(&schedule, 0, sizeof(schedule));

    schedule.Status = PENDING;
    pEndAngleCalc(dwellAngle);
    TEST_ASSERT_EQUAL(expectedEndAngle, endAngle);
    
    TEST_ASSERT_EQUAL(pending, calculateIgnitionNTimeout(schedule, startAngle, channelDegrees, crankAngle));
    
    schedule.Status = RUNNING;
    pEndAngleCalc(dwellAngle);
    TEST_ASSERT_EQUAL(running, calculateIgnitionNTimeout(schedule, startAngle, channelDegrees, crankAngle));
}

void test_calc_ign_timeout(uint16_t channelDegrees, uint16_t crankAngle, uint32_t pending, uint32_t running, int16_t endAngle)
{
    channel2IgnDegrees = channelDegrees;
    test_calc_ignN_timeout(ignitionSchedule2, channelDegrees, ignition2StartAngle, &calculateIgnitionAngle2, crankAngle, pending, running, endAngle, ignition2EndAngle);
    channel3IgnDegrees = channelDegrees;
    test_calc_ignN_timeout(ignitionSchedule3, channelDegrees, ignition3StartAngle, &calculateIgnitionAngle3, crankAngle, pending, running, endAngle, ignition3EndAngle);
    channel4IgnDegrees = channelDegrees;
    test_calc_ignN_timeout(ignitionSchedule4, channelDegrees, ignition4StartAngle, &calculateIgnitionAngle4, crankAngle, pending, running, endAngle, ignition4EndAngle);
#if (IGN_CHANNELS >= 5)
    channel5IgnDegrees = channelDegrees;
    test_calc_ignN_timeout(ignitionSchedule5, channelDegrees, ignition5StartAngle, &calculateIgnitionAngle5, crankAngle, pending, running, endAngle, ignition5EndAngle);
#endif
#if (IGN_CHANNELS >= 6)
    channel6IgnDegrees = channelDegrees;
    test_calc_ignN_timeout(ignitionSchedule6, channelDegrees, ignition6StartAngle, &calculateIgnitionAngle6, crankAngle, pending, running, endAngle, ignition6EndAngle);
#endif
#if (IGN_CHANNELS >= 7)
    channel7IgnDegrees = channelDegrees;
    test_calc_ignN_timeout(ignitionSchedule7, channelDegrees, ignition7StartAngle, &calculateIgnitionAngle7, crankAngle, pending, running, endAngle, ignition7EndAngle);
#endif
#if (IGN_CHANNELS >= 8)
    channel8IgnDegrees = channelDegrees;
    test_calc_ignN_timeout(ignitionSchedule8, channelDegrees, ignition8StartAngle, &calculateIgnitionAngle8, crankAngle, pending, running, endAngle, ignition8EndAngle);
#endif
}

void test_calc_ign_timeout(const ign_test_parameters *pStart, const ign_test_parameters *pEnd)
{
    ign_test_parameters local;
    while (pStart!=pEnd)
    {
        memcpy_P(&local, pStart, sizeof(local));
        currentStatus.advance = local.advanceAngle;
        test_calc_ign_timeout(local.channelAngle, local.crankAngle, local.pending, local.running, local.endAngle);
        ++pStart;
    }
}

// Separate test for ign 1 - different code path, same results!
void test_calc_ign1_timeout()
{
    setEngineSpeed(4000, 360);

    static const ign_test_parameters test_data[] PROGMEM = {
         // ChannelAngle (deg), Advance, Crank, Expected Pending, Expected Running
        { 0, -40, 0, 12666, 12666, 40 },
        { 0, -40, 45, 10791, 10791, 40 },
        { 0, -40, 90, 8916, 8916, 40 },
        { 0, -40, 135, 7041, 7041, 40 },
        { 0, -40, 180, 5166, 5166, 40 },
        { 0, -40, 215, 3708, 3708, 40 },
        { 0, -40, 270, 1416, 1416, 40 },
        { 0, -40, 315, 0, 14541, 40 },
        { 0, -40, 360, 0, 12666, 40 },
        { 0, 0, 0, 11000, 11000, 360 },
        { 0, 0, 45, 9125, 9125, 360 },
        { 0, 0, 90, 7250, 7250, 360 },
        { 0, 0, 135, 5375, 5375, 360 },
        { 0, 0, 180, 3500, 3500, 360 },
        { 0, 0, 215, 2041, 2041, 360 },
        { 0, 0, 270, 0, 14750, 360 },
        { 0, 0, 315, 0, 12875, 360 },
        { 0, 0, 360, 0, 11000, 360 },
        { 0, 40, 0, 9333, 9333, 320 },
        { 0, 40, 45, 7458, 7458, 320 },
        { 0, 40, 90, 5583, 5583, 320 },
        { 0, 40, 135, 3708, 3708, 320 },
        { 0, 40, 180, 1833, 1833, 320 },
        { 0, 40, 215, 375, 375, 320 },
        { 0, 40, 270, 0, 13083, 320 },
        { 0, 40, 315, 0, 11208, 320 },
        { 0, 40, 360, 0, 9333, 320 },
    };

    const ign_test_parameters *pStart = &test_data[0];
    const ign_test_parameters *pEnd = pStart + +_countof(test_data);

    ign_test_parameters local;
    while (pStart!=pEnd)
    {
        memcpy_P(&local, pStart, sizeof(local));
        currentStatus.advance = local.advanceAngle;
        test_calc_ign1_timeout(local.crankAngle, local.pending, local.running, local.endAngle);
        ++pStart;
    }    
}

void test_calc_ign_timeout_360()
{
    setEngineSpeed(4000, 360);

    static const ign_test_parameters test_data[] PROGMEM = {
         // ChannelAngle (deg), Advance, Crank, Expected Pending, Expected Running
        { 0, -40, 0, 12666, 12666, 40 },
        { 0, -40, 45, 10791, 10791, 40 },
        { 0, -40, 90, 8916, 8916, 40 },
        { 0, -40, 135, 7041, 7041, 40 },
        { 0, -40, 180, 5166, 5166, 40 },
        { 0, -40, 215, 3708, 3708, 40 },
        { 0, -40, 270, 1416, 1416, 40 },
        { 0, -40, 315, 0, 14541, 40 },
        { 0, -40, 360, 0, 12666, 40 },
        { 0, 0, 0, 11000, 11000, 0 },
        { 0, 0, 45, 9125, 9125, 0 },
        { 0, 0, 90, 7250, 7250, 0 },
        { 0, 0, 135, 5375, 5375, 0 },
        { 0, 0, 180, 3500, 3500, 0 },
        { 0, 0, 215, 2041, 2041, 0 },
        { 0, 0, 270, 0, 14750, 0 },
        { 0, 0, 315, 0, 12875, 0 },
        { 0, 0, 360, 0, 11000, 0 },
        { 0, 40, 0, 9333, 9333, -40 },
        { 0, 40, 45, 7458, 7458, -40 },
        { 0, 40, 90, 5583, 5583, -40 },
        { 0, 40, 135, 3708, 3708, -40 },
        { 0, 40, 180, 1833, 1833, -40 },
        { 0, 40, 215, 375, 375, -40 },
        { 0, 40, 270, 0, 13083, -40 },
        { 0, 40, 315, 0, 11208, -40 },
        { 0, 40, 360, 0, 9333, -40 },
        { 72, -40, 0, 666, 666, 112 },
        { 72, -40, 45, 0, 13791, 112 },
        { 72, -40, 90, 11916, 11916, 112 },
        { 72, -40, 135, 10041, 10041, 112 },
        { 72, -40, 180, 8166, 8166, 112 },
        { 72, -40, 215, 6708, 6708, 112 },
        { 72, -40, 270, 4416, 4416, 112 },
        { 72, -40, 315, 2541, 2541, 112 },
        { 72, -40, 360, 666, 666, 112 },
        { 72, 0, 0, 0, 14000, 72 },
        { 72, 0, 45, 0, 12125, 72 },
        { 72, 0, 90, 10250, 10250, 72 },
        { 72, 0, 135, 8375, 8375, 72 },
        { 72, 0, 180, 6500, 6500, 72 },
        { 72, 0, 215, 5041, 5041, 72 },
        { 72, 0, 270, 2750, 2750, 72 },
        { 72, 0, 315, 875, 875, 72 },
        { 72, 0, 360, 0, 14000, 72 },
        { 72, 40, 0, 0, 12333, 32 },
        { 72, 40, 45, 0, 10458, 32 },
        { 72, 40, 90, 8583, 8583, 32 },
        { 72, 40, 135, 6708, 6708, 32 },
        { 72, 40, 180, 4833, 4833, 32 },
        { 72, 40, 215, 3375, 3375, 32 },
        { 72, 40, 270, 1083, 1083, 32 },
        { 72, 40, 315, 0, 14208, 32 },
        { 72, 40, 360, 0, 12333, 32 },
        { 90, -40, 0, 1416, 1416, 130 },
        { 90, -40, 45, 0, 14541, 130 },
        { 90, -40, 90, 12666, 12666, 130 },
        { 90, -40, 135, 10791, 10791, 130 },
        { 90, -40, 180, 8916, 8916, 130 },
        { 90, -40, 215, 7458, 7458, 130 },
        { 90, -40, 270, 5166, 5166, 130 },
        { 90, -40, 315, 3291, 3291, 130 },
        { 90, -40, 360, 1416, 1416, 130 },
        { 90, 0, 0, 0, 14750, 90 },
        { 90, 0, 45, 0, 12875, 90 },
        { 90, 0, 90, 11000, 11000, 90 },
        { 90, 0, 135, 9125, 9125, 90 },
        { 90, 0, 180, 7250, 7250, 90 },
        { 90, 0, 215, 5791, 5791, 90 },
        { 90, 0, 270, 3500, 3500, 90 },
        { 90, 0, 315, 1625, 1625, 90 },
        { 90, 0, 360, 0, 14750, 90 },
        { 90, 40, 0, 0, 13083, 50 },
        { 90, 40, 45, 0, 11208, 50 },
        { 90, 40, 90, 9333, 9333, 50 },
        { 90, 40, 135, 7458, 7458, 50 },
        { 90, 40, 180, 5583, 5583, 50 },
        { 90, 40, 215, 4125, 4125, 50 },
        { 90, 40, 270, 1833, 1833, 50 },
        { 90, 40, 315, 0, 14958, 50 },
        { 90, 40, 360, 0, 13083, 50 },
        { 144, -40, 0, 3666, 3666, 184 },
        { 144, -40, 45, 1791, 1791, 184 },
        { 144, -40, 90, 0, 14916, 184 },
        { 144, -40, 135, 0, 13041, 184 },
        { 144, -40, 180, 11166, 11166, 184 },
        { 144, -40, 215, 9708, 9708, 184 },
        { 144, -40, 270, 7416, 7416, 184 },
        { 144, -40, 315, 5541, 5541, 184 },
        { 144, -40, 360, 3666, 3666, 184 },
        { 144, 0, 0, 2000, 2000, 144 },
        { 144, 0, 45, 125, 125, 144 },
        { 144, 0, 90, 0, 13250, 144 },
        { 144, 0, 135, 0, 11375, 144 },
        { 144, 0, 180, 9500, 9500, 144 },
        { 144, 0, 215, 8041, 8041, 144 },
        { 144, 0, 270, 5750, 5750, 144 },
        { 144, 0, 315, 3875, 3875, 144 },
        { 144, 0, 360, 2000, 2000, 144 },
        { 144, 40, 0, 333, 333, 104 },
        { 144, 40, 45, 0, 13458, 104 },
        { 144, 40, 90, 0, 11583, 104 },
        { 144, 40, 135, 0, 9708, 104 },
        { 144, 40, 180, 7833, 7833, 104 },
        { 144, 40, 215, 6375, 6375, 104 },
        { 144, 40, 270, 4083, 4083, 104 },
        { 144, 40, 315, 2208, 2208, 104 },
        { 144, 40, 360, 333, 333, 104 },
        { 180, -40, 0, 5166, 5166, 220 },
        { 180, -40, 45, 3291, 3291, 220 },
        { 180, -40, 90, 1416, 1416, 220 },
        { 180, -40, 135, 0, 14541, 220 },
        { 180, -40, 180, 12666, 12666, 220 },
        { 180, -40, 215, 11208, 11208, 220 },
        { 180, -40, 270, 8916, 8916, 220 },
        { 180, -40, 315, 7041, 7041, 220 },
        { 180, -40, 360, 5166, 5166, 220 },
        { 180, 0, 0, 3500, 3500, 180 },
        { 180, 0, 45, 1625, 1625, 180 },
        { 180, 0, 90, 0, 14750, 180 },
        { 180, 0, 135, 0, 12875, 180 },
        { 180, 0, 180, 11000, 11000, 180 },
        { 180, 0, 215, 9541, 9541, 180 },
        { 180, 0, 270, 7250, 7250, 180 },
        { 180, 0, 315, 5375, 5375, 180 },
        { 180, 0, 360, 3500, 3500, 180 },
        { 180, 40, 0, 1833, 1833, 140 },
        { 180, 40, 45, 0, 14958, 140 },
        { 180, 40, 90, 0, 13083, 140 },
        { 180, 40, 135, 0, 11208, 140 },
        { 180, 40, 180, 9333, 9333, 140 },
        { 180, 40, 215, 7875, 7875, 140 },
        { 180, 40, 270, 5583, 5583, 140 },
        { 180, 40, 315, 3708, 3708, 140 },
        { 180, 40, 360, 1833, 1833, 140 },
        { 270, -40, 0, 8916, 8916, 310 },
        { 270, -40, 45, 7041, 7041, 310 },
        { 270, -40, 90, 5166, 5166, 310 },
        { 270, -40, 135, 3291, 3291, 310 },
        { 270, -40, 180, 1416, 1416, 310 },
        { 270, -40, 215, 0, 14958, 310 },
        { 270, -40, 270, 12666, 12666, 310 },
        { 270, -40, 315, 10791, 10791, 310 },
        { 270, -40, 360, 8916, 8916, 310 },
        { 270, 0, 0, 7250, 7250, 270 },
        { 270, 0, 45, 5375, 5375, 270 },
        { 270, 0, 90, 3500, 3500, 270 },
        { 270, 0, 135, 1625, 1625, 270 },
        { 270, 0, 180, 0, 14750, 270 },
        { 270, 0, 215, 0, 13291, 270 },
        { 270, 0, 270, 11000, 11000, 270 },
        { 270, 0, 315, 9125, 9125, 270 },
        { 270, 0, 360, 7250, 7250, 270 },
        { 270, 40, 0, 5583, 5583, 230 },
        { 270, 40, 45, 3708, 3708, 230 },
        { 270, 40, 90, 1833, 1833, 230 },
        { 270, 40, 135, 0, 14958, 230 },
        { 270, 40, 180, 0, 13083, 230 },
        { 270, 40, 215, 0, 11625, 230 },
        { 270, 40, 270, 9333, 9333, 230 },
        { 270, 40, 315, 7458, 7458, 230 },
        { 270, 40, 360, 5583, 5583, 230 },
        { 360, -40, 0, 12666, 12666, 40 },
        { 360, -40, 45, 10791, 10791, 40 },
        { 360, -40, 90, 8916, 8916, 40 },
        { 360, -40, 135, 7041, 7041, 40 },
        { 360, -40, 180, 5166, 5166, 40 },
        { 360, -40, 215, 3708, 3708, 40 },
        { 360, -40, 270, 1416, 1416, 40 },
        { 360, -40, 315, 0, 14541, 40 },
        { 360, -40, 360, 12666, 12666, 40 },
        { 360, 0, 0, 11000, 11000, 360 },
        { 360, 0, 45, 9125, 9125, 360 },
        { 360, 0, 90, 7250, 7250, 360 },
        { 360, 0, 135, 5375, 5375, 360 },
        { 360, 0, 180, 3500, 3500, 360 },
        { 360, 0, 215, 2041, 2041, 360 },
        { 360, 0, 270, 0, 14750, 360 },
        { 360, 0, 315, 0, 12875, 360 },
        { 360, 0, 360, 11000, 11000, 360 },
        { 360, 40, 0, 9333, 9333, 320 },
        { 360, 40, 45, 7458, 7458, 320 },
        { 360, 40, 90, 5583, 5583, 320 },
        { 360, 40, 135, 3708, 3708, 320 },
        { 360, 40, 180, 1833, 1833, 320 },
        { 360, 40, 215, 375, 375, 320 },
        { 360, 40, 270, 0, 13083, 320 },
        { 360, 40, 315, 0, 11208, 320 },
        { 360, 40, 360, 9333, 9333, 320 },
    };

    test_calc_ign_timeout(&test_data[0], &test_data[0]+_countof(test_data));
}



void test_calc_ign_timeout_720()
{
    setEngineSpeed(4000, 720);

    static const ign_test_parameters test_data[] PROGMEM = {
         // ChannelAngle (deg), Advance, Crank, Expected Pending, Expected Running
        { 0, -40, 0, 27666, 27666, 40 },
        { 0, -40, 45, 25791, 25791, 40 },
        { 0, -40, 90, 23916, 23916, 40 },
        { 0, -40, 135, 22041, 22041, 40 },
        { 0, -40, 180, 20166, 20166, 40 },
        { 0, -40, 215, 18708, 18708, 40 },
        { 0, -40, 270, 16416, 16416, 40 },
        { 0, -40, 315, 14541, 14541, 40 },
        { 0, -40, 360, 12666, 12666, 40 },
        { 0, 0, 0, 26000, 26000, 0 },
        { 0, 0, 45, 24125, 24125, 0 },
        { 0, 0, 90, 22250, 22250, 0 },
        { 0, 0, 135, 20375, 20375, 0 },
        { 0, 0, 180, 18500, 18500, 0 },
        { 0, 0, 215, 17041, 17041, 0 },
        { 0, 0, 270, 14750, 14750, 0 },
        { 0, 0, 315, 12875, 12875, 0 },
        { 0, 0, 360, 11000, 11000, 0 },
        { 0, 40, 0, 24333, 24333, -40 },
        { 0, 40, 45, 22458, 22458, -40 },
        { 0, 40, 90, 20583, 20583, -40 },
        { 0, 40, 135, 18708, 18708, -40 },
        { 0, 40, 180, 16833, 16833, -40 },
        { 0, 40, 215, 15375, 15375, -40 },
        { 0, 40, 270, 13083, 13083, -40 },
        { 0, 40, 315, 11208, 11208, -40 },
        { 0, 40, 360, 9333, 9333, -40 },
        { 72, -40, 0, 666, 666, 112 },
        { 72, -40, 45, 0, 28791, 112 },
        { 72, -40, 90, 26916, 26916, 112 },
        { 72, -40, 135, 25041, 25041, 112 },
        { 72, -40, 180, 23166, 23166, 112 },
        { 72, -40, 215, 21708, 21708, 112 },
        { 72, -40, 270, 19416, 19416, 112 },
        { 72, -40, 315, 17541, 17541, 112 },
        { 72, -40, 360, 15666, 15666, 112 },
        { 72, 0, 0, 0, 29000, 72 },
        { 72, 0, 45, 0, 27125, 72 },
        { 72, 0, 90, 25250, 25250, 72 },
        { 72, 0, 135, 23375, 23375, 72 },
        { 72, 0, 180, 21500, 21500, 72 },
        { 72, 0, 215, 20041, 20041, 72 },
        { 72, 0, 270, 17750, 17750, 72 },
        { 72, 0, 315, 15875, 15875, 72 },
        { 72, 0, 360, 14000, 14000, 72 },
        { 72, 40, 0, 0, 27333, 32 },
        { 72, 40, 45, 0, 25458, 32 },
        { 72, 40, 90, 23583, 23583, 32 },
        { 72, 40, 135, 21708, 21708, 32 },
        { 72, 40, 180, 19833, 19833, 32 },
        { 72, 40, 215, 18375, 18375, 32 },
        { 72, 40, 270, 16083, 16083, 32 },
        { 72, 40, 315, 14208, 14208, 32 },
        { 72, 40, 360, 12333, 12333, 32 },
        { 90, -40, 0, 1416, 1416, 130 },
        { 90, -40, 45, 0, 29541, 130 },
        { 90, -40, 90, 27666, 27666, 130 },
        { 90, -40, 135, 25791, 25791, 130 },
        { 90, -40, 180, 23916, 23916, 130 },
        { 90, -40, 215, 22458, 22458, 130 },
        { 90, -40, 270, 20166, 20166, 130 },
        { 90, -40, 315, 18291, 18291, 130 },
        { 90, -40, 360, 16416, 16416, 130 },
        { 90, 0, 0, 0, 29750, 90 },
        { 90, 0, 45, 0, 27875, 90 },
        { 90, 0, 90, 26000, 26000, 90 },
        { 90, 0, 135, 24125, 24125, 90 },
        { 90, 0, 180, 22250, 22250, 90 },
        { 90, 0, 215, 20791, 20791, 90 },
        { 90, 0, 270, 18500, 18500, 90 },
        { 90, 0, 315, 16625, 16625, 90 },
        { 90, 0, 360, 14750, 14750, 90 },
        { 90, 40, 0, 0, 28083, 50 },
        { 90, 40, 45, 0, 26208, 50 },
        { 90, 40, 90, 24333, 24333, 50 },
        { 90, 40, 135, 22458, 22458, 50 },
        { 90, 40, 180, 20583, 20583, 50 },
        { 90, 40, 215, 19125, 19125, 50 },
        { 90, 40, 270, 16833, 16833, 50 },
        { 90, 40, 315, 14958, 14958, 50 },
        { 90, 40, 360, 13083, 13083, 50 },
        { 144, -40, 0, 3666, 3666, 184 },
        { 144, -40, 45, 1791, 1791, 184 },
        { 144, -40, 90, 0, 29916, 184 },
        { 144, -40, 135, 0, 28041, 184 },
        { 144, -40, 180, 26166, 26166, 184 },
        { 144, -40, 215, 24708, 24708, 184 },
        { 144, -40, 270, 22416, 22416, 184 },
        { 144, -40, 315, 20541, 20541, 184 },
        { 144, -40, 360, 18666, 18666, 184 },
        { 144, 0, 0, 2000, 2000, 144 },
        { 144, 0, 45, 125, 125, 144 },
        { 144, 0, 90, 0, 28250, 144 },
        { 144, 0, 135, 0, 26375, 144 },
        { 144, 0, 180, 24500, 24500, 144 },
        { 144, 0, 215, 23041, 23041, 144 },
        { 144, 0, 270, 20750, 20750, 144 },
        { 144, 0, 315, 18875, 18875, 144 },
        { 144, 0, 360, 17000, 17000, 144 },
        { 144, 40, 0, 333, 333, 104 },
        { 144, 40, 45, 0, 28458, 104 },
        { 144, 40, 90, 0, 26583, 104 },
        { 144, 40, 135, 0, 24708, 104 },
        { 144, 40, 180, 22833, 22833, 104 },
        { 144, 40, 215, 21375, 21375, 104 },
        { 144, 40, 270, 19083, 19083, 104 },
        { 144, 40, 315, 17208, 17208, 104 },
        { 144, 40, 360, 15333, 15333, 104 },
        { 180, -40, 0, 5166, 5166, 220 },
        { 180, -40, 45, 3291, 3291, 220 },
        { 180, -40, 90, 1416, 1416, 220 },
        { 180, -40, 135, 0, 29541, 220 },
        { 180, -40, 180, 27666, 27666, 220 },
        { 180, -40, 215, 26208, 26208, 220 },
        { 180, -40, 270, 23916, 23916, 220 },
        { 180, -40, 315, 22041, 22041, 220 },
        { 180, -40, 360, 20166, 20166, 220 },
        { 180, 0, 0, 3500, 3500, 180 },
        { 180, 0, 45, 1625, 1625, 180 },
        { 180, 0, 90, 0, 29750, 180 },
        { 180, 0, 135, 0, 27875, 180 },
        { 180, 0, 180, 26000, 26000, 180 },
        { 180, 0, 215, 24541, 24541, 180 },
        { 180, 0, 270, 22250, 22250, 180 },
        { 180, 0, 315, 20375, 20375, 180 },
        { 180, 0, 360, 18500, 18500, 180 },
        { 180, 40, 0, 1833, 1833, 140 },
        { 180, 40, 45, 0, 29958, 140 },
        { 180, 40, 90, 0, 28083, 140 },
        { 180, 40, 135, 0, 26208, 140 },
        { 180, 40, 180, 24333, 24333, 140 },
        { 180, 40, 215, 22875, 22875, 140 },
        { 180, 40, 270, 20583, 20583, 140 },
        { 180, 40, 315, 18708, 18708, 140 },
        { 180, 40, 360, 16833, 16833, 140 },
        { 270, -40, 0, 8916, 8916, 310 },
        { 270, -40, 45, 7041, 7041, 310 },
        { 270, -40, 90, 5166, 5166, 310 },
        { 270, -40, 135, 3291, 3291, 310 },
        { 270, -40, 180, 1416, 1416, 310 },
        { 270, -40, 215, 0, 29958, 310 },
        { 270, -40, 270, 27666, 27666, 310 },
        { 270, -40, 315, 25791, 25791, 310 },
        { 270, -40, 360, 23916, 23916, 310 },
        { 270, 0, 0, 7250, 7250, 270 },
        { 270, 0, 45, 5375, 5375, 270 },
        { 270, 0, 90, 3500, 3500, 270 },
        { 270, 0, 135, 1625, 1625, 270 },
        { 270, 0, 180, 0, 29750, 270 },
        { 270, 0, 215, 0, 28291, 270 },
        { 270, 0, 270, 26000, 26000, 270 },
        { 270, 0, 315, 24125, 24125, 270 },
        { 270, 0, 360, 22250, 22250, 270 },
        { 270, 40, 0, 5583, 5583, 230 },
        { 270, 40, 45, 3708, 3708, 230 },
        { 270, 40, 90, 1833, 1833, 230 },
        { 270, 40, 135, 0, 29958, 230 },
        { 270, 40, 180, 0, 28083, 230 },
        { 270, 40, 215, 0, 26625, 230 },
        { 270, 40, 270, 24333, 24333, 230 },
        { 270, 40, 315, 22458, 22458, 230 },
        { 270, 40, 360, 20583, 20583, 230 },
        { 360, -40, 0, 12666, 12666, 400 },
        { 360, -40, 45, 10791, 10791, 400 },
        { 360, -40, 90, 8916, 8916, 400 },
        { 360, -40, 135, 7041, 7041, 400 },
        { 360, -40, 180, 5166, 5166, 400 },
        { 360, -40, 215, 3708, 3708, 400 },
        { 360, -40, 270, 1416, 1416, 400 },
        { 360, -40, 315, 0, 29541, 400 },
        { 360, -40, 360, 27666, 27666, 400 },
        { 360, 0, 0, 11000, 11000, 360 },
        { 360, 0, 45, 9125, 9125, 360 },
        { 360, 0, 90, 7250, 7250, 360 },
        { 360, 0, 135, 5375, 5375, 360 },
        { 360, 0, 180, 3500, 3500, 360 },
        { 360, 0, 215, 2041, 2041, 360 },
        { 360, 0, 270, 0, 29750, 360 },
        { 360, 0, 315, 0, 27875, 360 },
        { 360, 0, 360, 26000, 26000, 360 },
        { 360, 40, 0, 9333, 9333, 320 },
        { 360, 40, 45, 7458, 7458, 320 },
        { 360, 40, 90, 5583, 5583, 320 },
        { 360, 40, 135, 3708, 3708, 320 },
        { 360, 40, 180, 1833, 1833, 320 },
        { 360, 40, 215, 375, 375, 320 },
        { 360, 40, 270, 0, 28083, 320 },
        { 360, 40, 315, 0, 26208, 320 },
        { 360, 40, 360, 24333, 24333, 320 },
        { 432, -40, 0, 15666, 15666, 472 },
        { 432, -40, 45, 13791, 13791, 472 },
        { 432, -40, 90, 11916, 11916, 472 },
        { 432, -40, 135, 10041, 10041, 472 },
        { 432, -40, 180, 8166, 8166, 472 },
        { 432, -40, 215, 6708, 6708, 472 },
        { 432, -40, 270, 4416, 4416, 472 },
        { 432, -40, 315, 2541, 2541, 472 },
        { 432, -40, 360, 666, 666, 472 },
        { 432, 0, 0, 14000, 14000, 432 },
        { 432, 0, 45, 12125, 12125, 432 },
        { 432, 0, 90, 10250, 10250, 432 },
        { 432, 0, 135, 8375, 8375, 432 },
        { 432, 0, 180, 6500, 6500, 432 },
        { 432, 0, 215, 5041, 5041, 432 },
        { 432, 0, 270, 2750, 2750, 432 },
        { 432, 0, 315, 875, 875, 432 },
        { 432, 0, 360, 0, 29000, 432 },
        { 432, 40, 0, 12333, 12333, 392 },
        { 432, 40, 45, 10458, 10458, 392 },
        { 432, 40, 90, 8583, 8583, 392 },
        { 432, 40, 135, 6708, 6708, 392 },
        { 432, 40, 180, 4833, 4833, 392 },
        { 432, 40, 215, 3375, 3375, 392 },
        { 432, 40, 270, 1083, 1083, 392 },
        { 432, 40, 315, 0, 29208, 392 },
        { 432, 40, 360, 0, 27333, 392 },
        { 576, -40, 0, 21666, 21666, 616 },
        { 576, -40, 45, 19791, 19791, 616 },
        { 576, -40, 90, 17916, 17916, 616 },
        { 576, -40, 135, 16041, 16041, 616 },
        { 576, -40, 180, 14166, 14166, 616 },
        { 576, -40, 215, 12708, 12708, 616 },
        { 576, -40, 270, 10416, 10416, 616 },
        { 576, -40, 315, 8541, 8541, 616 },
        { 576, -40, 360, 6666, 6666, 616 },
        { 576, 0, 0, 20000, 20000, 576 },
        { 576, 0, 45, 18125, 18125, 576 },
        { 576, 0, 90, 16250, 16250, 576 },
        { 576, 0, 135, 14375, 14375, 576 },
        { 576, 0, 180, 12500, 12500, 576 },
        { 576, 0, 215, 11041, 11041, 576 },
        { 576, 0, 270, 8750, 8750, 576 },
        { 576, 0, 315, 6875, 6875, 576 },
        { 576, 0, 360, 5000, 5000, 576 },
        { 576, 40, 0, 18333, 18333, 536 },
        { 576, 40, 45, 16458, 16458, 536 },
        { 576, 40, 90, 14583, 14583, 536 },
        { 576, 40, 135, 12708, 12708, 536 },
        { 576, 40, 180, 10833, 10833, 536 },
        { 576, 40, 215, 9375, 9375, 536 },
        { 576, 40, 270, 7083, 7083, 536 },
        { 576, 40, 315, 5208, 5208, 536 },
        { 576, 40, 360, 3333, 3333, 536 },
        { 600, -40, 0, 22666, 22666, 640 },
        { 600, -40, 45, 20791, 20791, 640 },
        { 600, -40, 90, 18916, 18916, 640 },
        { 600, -40, 135, 17041, 17041, 640 },
        { 600, -40, 180, 15166, 15166, 640 },
        { 600, -40, 215, 13708, 13708, 640 },
        { 600, -40, 270, 11416, 11416, 640 },
        { 600, -40, 315, 9541, 9541, 640 },
        { 600, -40, 360, 7666, 7666, 640 },
        { 600, 0, 0, 21000, 21000, 600 },
        { 600, 0, 45, 19125, 19125, 600 },
        { 600, 0, 90, 17250, 17250, 600 },
        { 600, 0, 135, 15375, 15375, 600 },
        { 600, 0, 180, 13500, 13500, 600 },
        { 600, 0, 215, 12041, 12041, 600 },
        { 600, 0, 270, 9750, 9750, 600 },
        { 600, 0, 315, 7875, 7875, 600 },
        { 600, 0, 360, 6000, 6000, 600 },
        { 600, 40, 0, 19333, 19333, 560 },
        { 600, 40, 45, 17458, 17458, 560 },
        { 600, 40, 90, 15583, 15583, 560 },
        { 600, 40, 135, 13708, 13708, 560 },
        { 600, 40, 180, 11833, 11833, 560 },
        { 600, 40, 215, 10375, 10375, 560 },
        { 600, 40, 270, 8083, 8083, 560 },
        { 600, 40, 315, 6208, 6208, 560 },
        { 600, 40, 360, 4333, 4333, 560 },
    };

    test_calc_ign_timeout(&test_data[0], &test_data[0]+_countof(test_data));
}

void test_calc_ign_timeout(void)
{
    RUN_TEST(test_calc_ign1_timeout);
    RUN_TEST(test_calc_ign_timeout_360);
    RUN_TEST(test_calc_ign_timeout_720);
}