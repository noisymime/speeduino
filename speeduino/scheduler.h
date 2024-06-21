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

#include "board_definition.h"
#include "scheduledIO.h"

#define USE_IGN_REFRESH
#define IGNITION_REFRESH_THRESHOLD  30 //Time in uS that the refresh functions will check to ensure there is enough time before changing the end compare

void initialiseSchedulers(void);
void startSchedulers(void);
void beginInjectorPriming(void);

void disablePendingFuelSchedule(byte channel);
void disablePendingIgnSchedule(byte channel);

void refreshIgnitionSchedule1(unsigned long timeToEnd);

/** \enum ScheduleStatus
 * @brief The current state of a schedule
 * */
enum ScheduleStatus {
  /** Not running */
  OFF, 
  /** The delay phase of the schedule is active */
  PENDING,
  /** The schedule action is running */
  RUNNING,
  /** The schedule is running, with a next schedule queued up */
  RUNNING_WITHNEXT,
}; 


/**
 * @brief A schedule for a single output channel. 
 * 
 * @details
 * @par A schedule consists of 3 independent parts:
 * - an action that can be started and stopped. E.g. charge ignition coil or injection pulse
 * - a delay until the action is started
 * - a duration until the action is stopped
 * 
 * I.e.\n 
 * \code 
 *   <--------------- Delay ---------------><---- Duration ---->
 *                                          ^                  ^
 *                              Action starts                  Action ends
 * \endcode
 * 
 * @par Timers are modelled as registers
 * Once set, Schedule instances are usually driven externally by a timer
 * ISR calling moveToNextState()
 */
struct Schedule {
  // Deduce the real types of the counter and compare registers.
  // COMPARE_TYPE is NOT the same - it's just an integer type wide enough to
  // store 16-bit counter/compare calculation results.
  /** @brief The type of a timer counter register (this varies between platforms) */
  using counter_t = decltype(FUEL1_COUNTER /* <-- Arbitrary choice of macro, assumes all have the same type */);
  /** @brief The type of a timer compare register (this varies between platforms) */
  using compare_t = decltype(FUEL1_COMPARE /* <-- Arbitrary choice of macro, assumes all have the same type */);

  /**
   * @brief Construct a new Schedule object
   * 
   * @param counter A <b>reference</b> to the timer counter
   * @param compare A <b>reference</b> to the timer comparator
   */
  Schedule(counter_t &counter, compare_t &compare);

  /**
   * @brief Scheduled duration (timer ticks) 
   *
   * This captures the duration of the *next* interval to be scheduled. I.e.
   *  * Status==PENDING: this is the duration that will be used when the schedule moves to the RUNNING state 
   *  * Status==RUNNING_WITHNEXT: this is the duration that will be used after the current schedule finishes and the queued up scheduled starts 
   */
  volatile COMPARE_TYPE Duration;   ///< 
  volatile ScheduleStatus Status;   ///< Schedule status
  voidVoidCallback pStartCallback;  ///< Start Callback function for schedule
  voidVoidCallback pEndCallback;    ///< End Callback function for schedule

  volatile COUNTER_TYPE nextStartCompare;    ///< Planned start of next schedule (when current schedule is RUNNING_WITHNEXT)
  
  counter_t &_counter;       ///< **Reference** to the counter register. E.g. TCNT3
  compare_t &_compare;       ///< **Reference**to the compare register. E.g. OCR3A
};

static inline bool isRunning(const Schedule &schedule) {
  return schedule.Status==RUNNING || schedule.Status==RUNNING_WITHNEXT;
}

void _setScheduleNext(Schedule &schedule, uint32_t timeout, uint32_t duration);

void setCallbacks(Schedule &schedule, voidVoidCallback pStartCallback, voidVoidCallback pEndCallback);


/** Ignition schedule.
 */
struct IgnitionSchedule : public Schedule {

  using Schedule::Schedule;

  volatile unsigned long startTime; /**< The system time (in uS) that the schedule started, used by the overdwell protection in timers.ino */
  volatile bool endScheduleSetByDecoder = false;
};

void _setIgnitionScheduleRunning(IgnitionSchedule &schedule, unsigned long timeout, unsigned long duration);

static inline __attribute__((always_inline)) void setIgnitionSchedule(IgnitionSchedule &schedule, unsigned long timeout, unsigned long duration) {
  noInterrupts();
  if(!isRunning(schedule)) { //Check that we're not already part way through a schedule
    _setIgnitionScheduleRunning(schedule, timeout, duration);
  // Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
  } else if(timeout < MAX_TIMER_PERIOD){
    _setScheduleNext(schedule, timeout, duration);
  } else {
    // Keep MISRA checker happy
  }
  interrupts();
}

/**
 * @brief Shared ignition schedule timer ISR *implementation*. Should be called by the actual ignition timer ISRs
 * (as timed interrupts) when either the start time or the duration time are reached. See @ref schedule-state-machine
 * 
 * @param schedule The ignition schedule to move to the next state
 */
void moveToNextState(IgnitionSchedule &schedule);

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

/** Fuel injection schedule.
* Fuel schedules don't use the callback pointers, or the startTime/endScheduleSetByDecoder variables.
* They are removed in this struct to save RAM.
*/
struct FuelSchedule : public Schedule {

  using Schedule::Schedule;

};

void _setFuelScheduleRunning(FuelSchedule &schedule, unsigned long timeout, unsigned long duration);

static inline __attribute__((always_inline)) void setFuelSchedule(FuelSchedule &schedule, unsigned long timeout, unsigned long duration) {
  // Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
  if(timeout < MAX_TIMER_PERIOD) {
    noInterrupts();
    if(!isRunning(schedule)) { //Check that we're not already part way through a schedule
      _setFuelScheduleRunning(schedule, timeout, duration);
    }
    else {
      _setScheduleNext(schedule, timeout, duration);
    }
    interrupts();
  }
}

/**
 * @brief Shared fuel schedule timer ISR implementation. Should be called by the actual timer ISRs
 * (as timed interrupts) when either the start time or the duration time are reached. See @ref schedule-state-machine
 * 
 * @param schedule The fuel schedule to move to the next state
 */
void moveToNextState(FuelSchedule &schedule);

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

#endif // SCHEDULER_H
