#include "globals.h"
#include "scheduler.h"
#include "crankMaths.h"

inline uint16_t calculateInjectorStartAngle(uint16_t PWdivTimerPerDegree, int16_t injChannelDegrees)
{
  uint16_t tempInjectorStartAngle = (currentStatus.injAngle + injChannelDegrees);
  if(tempInjectorStartAngle < PWdivTimerPerDegree) { tempInjectorStartAngle += CRANK_ANGLE_MAX_INJ; }
  tempInjectorStartAngle -= PWdivTimerPerDegree;
  while(tempInjectorStartAngle > (uint16_t)CRANK_ANGLE_MAX_INJ) { tempInjectorStartAngle -= CRANK_ANGLE_MAX_INJ; }

  return tempInjectorStartAngle;
}

inline uint32_t calculateInjector1Timeout(int injector1StartAngle, int crankAngle)
{
    if ( (injector1StartAngle <= crankAngle) && (fuelSchedule1.Status == RUNNING) ) { injector1StartAngle += CRANK_ANGLE_MAX_INJ; }
    if (injector1StartAngle > crankAngle)
    {
        return ((injector1StartAngle - crankAngle) * (unsigned long)timePerDegree);
    }
    return 0U;
}

inline uint32_t calculateInjectorNTimeout(const FuelSchedule &schedule, int channelInjDegrees, int injectorStartAngle, int crankAngle)
{
    int tempCrankAngle = crankAngle - channelInjDegrees;
    if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_INJ; }
    int tempStartAngle = injectorStartAngle - channelInjDegrees;
    if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
    if ( (tempStartAngle <= tempCrankAngle) && (schedule.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
    if ( tempStartAngle > tempCrankAngle )
    {
        return (((uint32_t)tempStartAngle - (uint32_t)tempCrankAngle) * (uint32_t)timePerDegree);
    }
    return 0U;
}


inline void calculateIgnitionAngle1(int dwellAngle)
{
  ignition1EndAngle = CRANK_ANGLE_MAX_IGN - currentStatus.advance;
  if(ignition1EndAngle > CRANK_ANGLE_MAX_IGN) {ignition1EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition1StartAngle = ignition1EndAngle - dwellAngle; // 360 - desired advance angle - number of degrees the dwell will take
  if(ignition1StartAngle < 0) {ignition1StartAngle += CRANK_ANGLE_MAX_IGN;}
}

inline void calculateIgnitionAngle2(int dwellAngle)
{
  ignition2EndAngle = channel2IgnDegrees - currentStatus.advance;
  if(ignition2EndAngle > CRANK_ANGLE_MAX_IGN) {ignition2EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition2StartAngle = ignition2EndAngle - dwellAngle;
  if(ignition2StartAngle < 0) {ignition2StartAngle += CRANK_ANGLE_MAX_IGN;}
}

inline void calculateIgnitionAngle3(int dwellAngle)
{
  ignition3EndAngle = channel3IgnDegrees - currentStatus.advance;
  if(ignition3EndAngle > CRANK_ANGLE_MAX_IGN) {ignition3EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition3StartAngle = ignition3EndAngle - dwellAngle;
  if(ignition3StartAngle < 0) {ignition3StartAngle += CRANK_ANGLE_MAX_IGN;}
}

// ignition 3 for rotary
inline void calculateIgnitionAngle3(int dwellAngle, int rotarySplitDegrees)
{
  ignition3EndAngle = ignition1EndAngle + rotarySplitDegrees;
  ignition3StartAngle = ignition3EndAngle - dwellAngle;
  if(ignition3StartAngle > CRANK_ANGLE_MAX_IGN) {ignition3StartAngle -= CRANK_ANGLE_MAX_IGN;}
  if(ignition3StartAngle < 0) {ignition3StartAngle += CRANK_ANGLE_MAX_IGN;}
}

inline void calculateIgnitionAngle4(int dwellAngle)
{
  ignition4EndAngle = channel4IgnDegrees - currentStatus.advance;
  if(ignition4EndAngle > CRANK_ANGLE_MAX_IGN) {ignition4EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition4StartAngle = ignition4EndAngle - dwellAngle;
  if(ignition4StartAngle < 0) {ignition4StartAngle += CRANK_ANGLE_MAX_IGN;}
}

// ignition 4 for rotary
inline void calculateIgnitionAngle4(int dwellAngle, int rotarySplitDegrees)
{
  ignition4EndAngle = ignition2EndAngle + rotarySplitDegrees;
  ignition4StartAngle = ignition4EndAngle - dwellAngle;
  if(ignition4StartAngle > CRANK_ANGLE_MAX_IGN) {ignition4StartAngle -= CRANK_ANGLE_MAX_IGN;}
  if(ignition4StartAngle < 0) {ignition4StartAngle += CRANK_ANGLE_MAX_IGN;}
}

inline void calculateIgnitionAngle5(int dwellAngle)
{
  ignition5EndAngle = channel5IgnDegrees - currentStatus.advance;
  if(ignition5EndAngle > CRANK_ANGLE_MAX_IGN) {ignition5EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition5StartAngle = ignition5EndAngle - dwellAngle;
  if(ignition5StartAngle < 0) {ignition5StartAngle += CRANK_ANGLE_MAX_IGN;}
}

inline void calculateIgnitionAngle6(int dwellAngle)
{
  ignition6EndAngle = channel6IgnDegrees - currentStatus.advance;
  if(ignition6EndAngle > CRANK_ANGLE_MAX_IGN) {ignition6EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition6StartAngle = ignition6EndAngle - dwellAngle;
  if(ignition6StartAngle < 0) {ignition6StartAngle += CRANK_ANGLE_MAX_IGN;}
}

inline void calculateIgnitionAngle7(int dwellAngle)
{
  ignition7EndAngle = channel7IgnDegrees - currentStatus.advance;
  if(ignition7EndAngle > CRANK_ANGLE_MAX_IGN) {ignition7EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition7StartAngle = ignition7EndAngle - dwellAngle;
  if(ignition7StartAngle < 0) {ignition7StartAngle += CRANK_ANGLE_MAX_IGN;}
}

inline void calculateIgnitionAngle8(int dwellAngle)
{
  ignition8EndAngle = channel8IgnDegrees - currentStatus.advance;
  if(ignition8EndAngle > CRANK_ANGLE_MAX_IGN) {ignition8EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition8StartAngle = ignition8EndAngle - dwellAngle;
  if(ignition8StartAngle < 0) {ignition8StartAngle += CRANK_ANGLE_MAX_IGN;}
}

inline uint32_t calculateIgnition1Timeout(int crankAngle)
{
    if ( (ignition1StartAngle <= crankAngle) && (ignitionSchedule1.Status == RUNNING) ) { ignition1StartAngle += CRANK_ANGLE_MAX_IGN; }
    if ( ignition1StartAngle > crankAngle)
    {
        return angleToTime((ignition1StartAngle - crankAngle), CRANKMATH_METHOD_INTERVAL_REV);
    }
    return 0;
}

inline uint32_t calculateIgnitionNTimeout(const Schedule &schedule, int startAngle, int channelIgnDegrees, int crankAngle)
{
    int tempCrankAngle = crankAngle - channelIgnDegrees;
    if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_IGN; }
    int tempStartAngle = startAngle - channelIgnDegrees;
    if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }

    if ( (tempStartAngle <= tempCrankAngle) && (schedule.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }
    if(tempStartAngle > tempCrankAngle)
    { 
        return angleToTime((tempStartAngle - tempCrankAngle), CRANKMATH_METHOD_INTERVAL_REV); 
    }
    
    return 0U;

}
