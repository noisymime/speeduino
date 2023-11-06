// Note that all functions with an underscore prefix are NOT part 
// of the public API. They are only here so we can inline them.

#include "scheduler.h"
#include "crankMaths.h"
#include "maths.h"
#include "timers.h"

static inline uint16_t calculateInjectorStartAngle(uint16_t pwDegrees, int16_t injChannelDegrees, uint16_t injAngle)
{
  // 0<=injAngle<=720°
  // 0<=injChannelDegrees<=720°
  // 0<pwDegrees<=??? (could be many crank rotations in the worst case!)
  // 45<=CRANK_ANGLE_MAX_INJ<=720
  // (CRANK_ANGLE_MAX_INJ can be as small as 360/nCylinders. E.g. 45° for 8 cylinder)

  uint16_t startAngle = (uint16_t)injAngle + (uint16_t)injChannelDegrees;
  // Avoid underflow
  while (startAngle<pwDegrees) { startAngle = startAngle + (uint16_t)CRANK_ANGLE_MAX_INJ; }
  // Guaranteed to be >=0.
  startAngle = startAngle - pwDegrees;
  // Clamp to 0<=startAngle<=CRANK_ANGLE_MAX_INJ
  while (startAngle>(uint16_t)CRANK_ANGLE_MAX_INJ) { startAngle = startAngle - (uint16_t)CRANK_ANGLE_MAX_INJ; }

  return startAngle;
}

static inline __attribute__((always_inline)) uint32_t _calculateAngularTime(const Schedule &schedule, uint16_t eventAngle, uint16_t crankAngle, uint16_t maxAngle) {
  int16_t delta = eventAngle - crankAngle;
  if (delta<0 && (schedule.Status == RUNNING)) {
    delta = delta + (int16_t)maxAngle; 
  } 

  return delta<0 ? 0U : angleToTimeMicroSecPerDegree((uint16_t)delta);
}

static inline __attribute__((always_inline)) uint16_t _adjustToTDC(int angle, uint16_t angleOffset, uint16_t maxAngle) {
  angle = angle - (int)angleOffset;
  if( angle < 0) { return angle + (int)maxAngle; }
  return angle;
}

static inline __attribute__((always_inline)) uint32_t _calculateAngularTime(const Schedule &schedule, uint16_t angleOffset, uint16_t eventAngle, uint16_t crankAngle, uint16_t maxAngle) {
  if (angleOffset==0U) {
    return _calculateAngularTime(schedule, eventAngle, crankAngle, maxAngle);
  }
  return _calculateAngularTime(schedule, 
            _adjustToTDC(eventAngle, angleOffset, maxAngle),
            _adjustToTDC(crankAngle, angleOffset, maxAngle),
            maxAngle);
}

static inline uint32_t calculateInjectorTimeout(const FuelSchedule &schedule, int channelInjDegrees, int openAngle, int crankAngle)
{
  return _calculateAngularTime(schedule, channelInjDegrees, openAngle, crankAngle, CRANK_ANGLE_MAX_INJ);
}

static inline void calculateIgnitionAngle(const uint16_t dwellAngle, const uint16_t channelIgnDegrees, int8_t advance, int *pEndAngle, int *pStartAngle)
{
  *pEndAngle = (int16_t)(channelIgnDegrees==0U ? (uint16_t)CRANK_ANGLE_MAX_IGN : channelIgnDegrees) - (int16_t)advance;
  if(*pEndAngle > CRANK_ANGLE_MAX_IGN) {*pEndAngle -= CRANK_ANGLE_MAX_IGN;}
  *pStartAngle = *pEndAngle - dwellAngle;
  if(*pStartAngle < 0) {*pStartAngle += CRANK_ANGLE_MAX_IGN;}
}

static inline void calculateIgnitionTrailingRotary(uint16_t dwellAngle, int rotarySplitDegrees, int leadIgnitionAngle, int *pEndAngle, int *pStartAngle)
{
  *pEndAngle = leadIgnitionAngle + rotarySplitDegrees;
  *pStartAngle = *pEndAngle - dwellAngle;
  if(*pStartAngle > CRANK_ANGLE_MAX_IGN) {*pStartAngle -= CRANK_ANGLE_MAX_IGN;}
  if(*pStartAngle < 0) {*pStartAngle += CRANK_ANGLE_MAX_IGN;}
}

static inline uint32_t calculateIgnitionTimeout(const IgnitionSchedule &schedule, int startAngle, int channelIgnDegrees, int crankAngle)
{
  return _calculateAngularTime(schedule, channelIgnDegrees, startAngle, crankAngle, CRANK_ANGLE_MAX_IGN);
}

// The concept here is that we have a more accurate crank angle.
// Ignition timing is driven by target spark angle relative to crank position.
// So the timing to begin & end charging the coil is based on crank angle.
// With a more accurate crank angle, we can increase the precision of the
// spark timing.
static inline void adjustCrankAngle(IgnitionSchedule &schedule, int startAngle, int endAngle, int crankAngle) {
  constexpr uint8_t MIN_CYCLES_FOR_CORRECTION = 6U;

  if( (schedule.Status == RUNNING) ) { 
    // Coil is charging so change the charge time so the spark fires at
    // the requested crank angle (this could reduce dwell time & potentially
    // result in a weaker spark).
    uint32_t timeToSpark = angleToTimeMicroSecPerDegree( ignitionLimits(endAngle-crankAngle) );
    COMPARE_TYPE ticksToSpark = (COMPARE_TYPE)uS_TO_TIMER_COMPARE( timeToSpark );
    schedule.Duration = ticksToSpark; 
    SET_COMPARE(schedule._compare, schedule._counter + ticksToSpark); 
  }
  else if((schedule.Status==PENDING) && (currentStatus.startRevolutions > MIN_CYCLES_FOR_CORRECTION) ) { 
    // We are waiting for the timer to fire & start charging the coil.
    // Keep dwell constant (for better spark) - instead adjust the waiting period so 
    // the spark fires at the requested crank angle.
    uint32_t timeToRun = angleToTimeMicroSecPerDegree( ignitionLimits(startAngle-crankAngle) );
    COMPARE_TYPE ticksToRun = (COMPARE_TYPE)uS_TO_TIMER_COMPARE( timeToRun );
    SET_COMPARE(schedule._compare, schedule._counter + ticksToRun); 
    schedule.endScheduleSetByDecoder = true;
  } else {
    // Schedule isn't on, so no adjustment possible
    // But keep the MISRA police happy.
  }
}