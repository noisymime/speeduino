/** @file
Injector and Ignition (on/off) scheduling (structs).

This scheduler is designed to maintain 2 schedules for use by the fuel and ignition systems.
It functions by waiting for the overflow vectors from each of the timers in use to overflow, which triggers an interrupt.

## Technical

Currently I am prescaling the 16-bit timers to 256 for injection and 64 for ignition.
This means that the counter increments every 16us (injection) / 4uS (ignition) and will overflow every 1048576uS.

    Max Period = (Prescale)*(1/Frequency)*(2^17)

For more details see https://playground.arduino.cc/Code/Timer1/ (OLD: http://playground.arduino.cc/code/timer1 ).
This means that the precision of the scheduler is:

- 16uS (+/- 8uS of target) for fuel
- 4uS (+/- 2uS) for ignition

## Features

This differs from most other schedulers in that its calls are non-recurring (ie when you schedule an event at a certain time and once it has occurred,
it will not reoccur unless you explicitly ask/re-register for it).
Each timer can have only 1 callback associated with it at any given time. If you call the setCallback function a 2nd time,
the original schedule will be overwritten and not occur.

## Timer identification

Arduino timers usage for injection and ignition schedules:
- timer3 is used for schedule 1(?) (fuel 1,2,3,4 ign 7,8)
- timer4 is used for schedule 2(?) (fuel 5,6 ign 4,5,6)
- timer5 is used ... (fuel 7,8, ign 1,2,3)

Timers 3,4 and 5 are 16-bit timers (ie count to 65536).
See page 136 of the processors datasheet: http://www.atmel.com/Images/doc2549.pdf .

256 prescale gives tick every 16uS.
256 prescale gives overflow every 1048576uS (This means maximum wait time is 1.0485 seconds).

*/
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "globals.h"

#define USE_IGN_REFRESH
#define IGNITION_REFRESH_THRESHOLD  30 //Time in uS that the refresh functions will check to ensure there is enough time before changing the end compare

#define DWELL_AVERAGE_ALPHA 30
#define DWELL_AVERAGE(input) LOW_PASS_FILTER((input), DWELL_AVERAGE_ALPHA, currentStatus.actualDwell)
//#define DWELL_AVERAGE(input) (currentStatus.dwell) //Can be use to disable the above for testing

void initialiseSchedulers(void);
void beginInjectorPriming(void);

void disablePendingFuelSchedule(byte channel);
void disablePendingIgnSchedule(byte channel);

void refreshIgnitionSchedule1(unsigned long timeToEnd);

//The ARM cores use separate functions for their ISRs
#if defined(ARDUINO_ARCH_STM32) || defined(CORE_TEENSY)
  void fuelSchedule1Interrupt(void);
  void fuelSchedule2Interrupt(void);
  void fuelSchedule3Interrupt(void);
  void fuelSchedule4Interrupt(void);
#if (INJ_CHANNELS >= 5)
  void fuelSchedule5Interrupt(void);
#endif
#if (INJ_CHANNELS >= 6)
  void fuelSchedule6Interrupt(void);
#endif
#if (INJ_CHANNELS >= 7)
  void fuelSchedule7Interrupt(void);
#endif
#if (INJ_CHANNELS >= 8)
  void fuelSchedule8Interrupt(void);
#endif
#if (IGN_CHANNELS >= 1)
  void ignitionSchedule1Interrupt(void);
#endif
#if (IGN_CHANNELS >= 2)
  void ignitionSchedule2Interrupt(void);
#endif
#if (IGN_CHANNELS >= 3)
  void ignitionSchedule3Interrupt(void);
#endif
#if (IGN_CHANNELS >= 4)
  void ignitionSchedule4Interrupt(void);
#endif
#if (IGN_CHANNELS >= 5)
  void ignitionSchedule5Interrupt(void);
#endif
#if (IGN_CHANNELS >= 6)
  void ignitionSchedule6Interrupt(void);
#endif
#if (IGN_CHANNELS >= 7)
  void ignitionSchedule7Interrupt(void);
#endif
#if (IGN_CHANNELS >= 8)
  void ignitionSchedule8Interrupt(void);
#endif
#endif
/** Schedule statuses.
 * - OFF - Schedule turned off and there is no scheduled plan
 * - PENDING - There's a scheduled plan, but is has not started to run yet
 * - STAGED - (???, Not used)
 * - RUNNING - Schedule is currently running
 */
enum ScheduleStatus {OFF, PENDING, STAGED, RUNNING}; //The statuses that a schedule can have

/** Ignition schedule.
 */
struct IgnitionSchedule {

  // Deduce the real types of the counter and compare registers.
  // COMPARE_TYPE is NOT the same - it's just an integer type wide enough to
  // store 16-bit counter/compare calculation results.
  using counter_t = decltype(IGN1_COUNTER);
  using compare_t = decltype(IGN1_COMPARE);

  IgnitionSchedule( counter_t &counter, compare_t &compare,
            void (&_pTimerDisable)(), void (&_pTimerEnable)())
  : counter(counter)
  , compare(compare)
  , pTimerDisable(_pTimerDisable)
  , pTimerEnable(_pTimerEnable)
  {
  }

  volatile unsigned long duration;///< Scheduled duration (uS ?)
  volatile ScheduleStatus Status; ///< Schedule status: OFF, PENDING, STAGED, RUNNING
  void (*pStartCallback)(void);        ///< Start Callback function for schedule
  void (*pEndCallback)(void);          ///< End Callback function for schedule
  volatile unsigned long startTime; /**< The system time (in uS) that the schedule started, used by the overdwell protection in timers.ino */
  volatile COMPARE_TYPE startCompare; ///< The counter value of the timer when this will start
  volatile COMPARE_TYPE endCompare;   ///< The counter value of the timer when this will end

  COMPARE_TYPE nextStartCompare;      ///< Planned start of next schedule (when current schedule is RUNNING)
  COMPARE_TYPE nextEndCompare;        ///< Planned end of next schedule (when current schedule is RUNNING)
  volatile bool hasNextSchedule = false; ///< Enable flag for planned next schedule (when current schedule is RUNNING)
  volatile bool endScheduleSetByDecoder = false;

  counter_t &counter;  // Reference to the counter register. E.g. TCNT3
  compare_t &compare;  // Reference to the compare register. E.g. OCR3A
  void (&pTimerDisable)();    // Reference to the timer disable function
  void (&pTimerEnable)();     // Reference to the timer enable function  
};

void _setIgnitionScheduleRunning(IgnitionSchedule &schedule, unsigned long timeout, unsigned long duration);
void _setIgnitionScheduleNext(IgnitionSchedule &schedule, unsigned long timeout, unsigned long duration);

inline __attribute__((always_inline)) void setIgnitionSchedule(IgnitionSchedule &schedule, unsigned long timeout, unsigned long duration) {
  if(schedule.Status != RUNNING) { //Check that we're not already part way through a schedule
    _setIgnitionScheduleRunning(schedule, timeout, duration);
  }
  // Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
  else if(timeout < MAX_TIMER_PERIOD){
    _setIgnitionScheduleNext(schedule, timeout, duration);
  }
}

/** Fuel injection schedule.
* Fuel schedules don't use the callback pointers, or the startTime/endScheduleSetByDecoder variables.
* They are removed in this struct to save RAM.
*/
struct FuelSchedule {

  // Deduce the real types of the counter and compare registers.
  // COMPARE_TYPE is NOT the same - it's just an integer type wide enough to
  // store 16-bit counter/compare calculation results.
  using counter_t = decltype(FUEL1_COUNTER);
  using compare_t = decltype(FUEL1_COMPARE);

  FuelSchedule( counter_t &counter, compare_t &compare,
            void (&_pTimerDisable)(), void (&_pTimerEnable)())
  : counter(counter)
  , compare(compare)
  , pTimerDisable(_pTimerDisable)
  , pTimerEnable(_pTimerEnable)
  {
  }

  volatile unsigned long duration;///< Scheduled duration (uS ?)
  volatile ScheduleStatus Status; ///< Schedule status: OFF, PENDING, STAGED, RUNNING
  volatile COMPARE_TYPE startCompare; ///< The counter value of the timer when this will start
  volatile COMPARE_TYPE endCompare;   ///< The counter value of the timer when this will end
  void (*pStartFunction)(void);
  void (*pEndFunction)(void);  
  COMPARE_TYPE nextStartCompare;
  COMPARE_TYPE nextEndCompare;
  volatile bool hasNextSchedule = false;

  counter_t &counter;  // Reference to the counter register. E.g. TCNT3
  compare_t &compare;  // Reference to the compare register. E.g. OCR3A
  void (&pTimerDisable)();    // Reference to the timer disable function
  void (&pTimerEnable)();     // Reference to the timer enable function  
};

void _setFuelScheduleRunning(FuelSchedule &schedule, unsigned long timeout, unsigned long duration);
void _setFuelScheduleNext(FuelSchedule &schedule, unsigned long timeout, unsigned long duration);

inline __attribute__((always_inline)) void setFuelSchedule(FuelSchedule &schedule, unsigned long timeout, unsigned long duration) 
{
  if(schedule.Status != RUNNING) 
  { //Check that we're not already part way through a schedule
    _setFuelScheduleRunning(schedule, timeout, duration);
  }
  else if(timeout < MAX_TIMER_PERIOD) 
  {
    _setFuelScheduleNext(schedule, timeout, duration);
  }
}

extern FuelSchedule fuelSchedule1;
extern FuelSchedule fuelSchedule2;
extern FuelSchedule fuelSchedule3;
extern FuelSchedule fuelSchedule4;
#if INJ_CHANNELS >= 5
extern FuelSchedule fuelSchedule5;
#endif
#if INJ_CHANNELS >= 6
extern FuelSchedule fuelSchedule6;
#endif
#if INJ_CHANNELS >= 7
extern FuelSchedule fuelSchedule7;
#endif
#if INJ_CHANNELS >= 8
extern FuelSchedule fuelSchedule8;
#endif

extern IgnitionSchedule ignitionSchedule1;
extern IgnitionSchedule ignitionSchedule2;
extern IgnitionSchedule ignitionSchedule3;
extern IgnitionSchedule ignitionSchedule4;
extern IgnitionSchedule ignitionSchedule5;
#if IGN_CHANNELS >= 6
extern IgnitionSchedule ignitionSchedule6;
#endif
#if IGN_CHANNELS >= 7
extern IgnitionSchedule ignitionSchedule7;
#endif
#if IGN_CHANNELS >= 8
extern IgnitionSchedule ignitionSchedule8;
#endif

#endif // SCHEDULER_H
