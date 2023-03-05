/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
/** @file
 * Injector and Ignition (on/off) scheduling (functions).
 * There is usually 8 functions for cylinders 1-8 with same naming pattern.
 * 
 * ## Scheduling structures
 * 
 * Structures @ref FuelSchedule and @ref Schedule describe (from scheduler.h) describe the scheduling info for Fuel and Ignition respectively.
 * They contain duration, current activity status, start timing, end timing, callbacks to carry out action, etc.
 * 
 * ## Scheduling Functions
 * 
 * For Injection:
 * - setFuelSchedule*(tout,dur) - **Setup** schedule for (next) injection on the channel
 * - inj*StartFunction() - Execute **start** of injection (Interrupt handler)
 * - inj*EndFunction() - Execute **end** of injection (interrupt handler)
 * 
 * For Ignition (has more complex schedule setup):
 * - setIgnitionSchedule*(cb_st,tout,dur,cb_end) - **Setup** schedule for (next) ignition on the channel
 * - ign*StartFunction() - Execute **start** of ignition (Interrupt handler)
 * - ign*EndFunction() - Execute **end** of ignition (Interrupt handler)
 */
#include "globals.h"
#include "scheduler.h"
#include "scheduledIO.h"
#include "timers.h"
#include "schedule_calcs.h"

FuelSchedule fuelSchedule1(FUEL1_COUNTER, FUEL1_COMPARE, FUEL1_TIMER_DISABLE, FUEL1_TIMER_ENABLE);
FuelSchedule fuelSchedule2(FUEL2_COUNTER, FUEL2_COMPARE, FUEL2_TIMER_DISABLE, FUEL2_TIMER_ENABLE);
FuelSchedule fuelSchedule3(FUEL3_COUNTER, FUEL3_COMPARE, FUEL3_TIMER_DISABLE, FUEL3_TIMER_ENABLE);
FuelSchedule fuelSchedule4(FUEL4_COUNTER, FUEL4_COMPARE, FUEL4_TIMER_DISABLE, FUEL4_TIMER_ENABLE);

#if (INJ_CHANNELS >= 5)
FuelSchedule fuelSchedule5(FUEL5_COUNTER, FUEL5_COMPARE, FUEL5_TIMER_DISABLE, FUEL5_TIMER_ENABLE);
#endif
#if (INJ_CHANNELS >= 6)
FuelSchedule fuelSchedule6(FUEL6_COUNTER, FUEL6_COMPARE, FUEL6_TIMER_DISABLE, FUEL6_TIMER_ENABLE);
#endif
#if (INJ_CHANNELS >= 7)
FuelSchedule fuelSchedule7(FUEL7_COUNTER, FUEL7_COMPARE, FUEL7_TIMER_DISABLE, FUEL7_TIMER_ENABLE);
#endif
#if (INJ_CHANNELS >= 8)
FuelSchedule fuelSchedule8(FUEL8_COUNTER, FUEL8_COMPARE, FUEL8_TIMER_DISABLE, FUEL8_TIMER_ENABLE);
#endif

IgnitionSchedule ignitionSchedule1(IGN1_COUNTER, IGN1_COMPARE, IGN1_TIMER_DISABLE, IGN1_TIMER_ENABLE);
IgnitionSchedule ignitionSchedule2(IGN2_COUNTER, IGN2_COMPARE, IGN2_TIMER_DISABLE, IGN2_TIMER_ENABLE);
IgnitionSchedule ignitionSchedule3(IGN3_COUNTER, IGN3_COMPARE, IGN3_TIMER_DISABLE, IGN3_TIMER_ENABLE);
IgnitionSchedule ignitionSchedule4(IGN4_COUNTER, IGN4_COMPARE, IGN4_TIMER_DISABLE, IGN4_TIMER_ENABLE);
IgnitionSchedule ignitionSchedule5(IGN5_COUNTER, IGN5_COMPARE, IGN5_TIMER_DISABLE, IGN5_TIMER_ENABLE);

#if IGN_CHANNELS >= 6
IgnitionSchedule ignitionSchedule6(IGN6_COUNTER, IGN6_COMPARE, IGN6_TIMER_DISABLE, IGN6_TIMER_ENABLE);
#endif
#if IGN_CHANNELS >= 7
IgnitionSchedule ignitionSchedule7(IGN7_COUNTER, IGN7_COMPARE, IGN7_TIMER_DISABLE, IGN7_TIMER_ENABLE);
#endif
#if IGN_CHANNELS >= 8
IgnitionSchedule ignitionSchedule8(IGN8_COUNTER, IGN8_COMPARE, IGN8_TIMER_DISABLE, IGN8_TIMER_ENABLE);
#endif

static void reset(Schedule &schedule)
{
    schedule.Status = OFF;
    schedule.pStartCallback = nullCallback;
    schedule.pEndCallback = nullCallback;
    schedule.pTimerEnable();
}

static void reset(FuelSchedule &schedule) 
{
    reset((Schedule&)schedule);
}

static void reset(IgnitionSchedule &schedule) 
{
    reset((Schedule&)schedule);
}

void initialiseSchedulers()
{
    reset(fuelSchedule1);
    reset(fuelSchedule2);
    reset(fuelSchedule3);
    reset(fuelSchedule4);
#if INJ_CHANNELS >= 5
    reset(fuelSchedule5);
#endif
#if INJ_CHANNELS >= 6
    reset(fuelSchedule6);
#endif
#if INJ_CHANNELS >= 7
    reset(fuelSchedule7);
#endif
#if INJ_CHANNELS >= 8
    reset(fuelSchedule8);
#endif

    reset(ignitionSchedule1);
    reset(ignitionSchedule2);
    reset(ignitionSchedule3);
    reset(ignitionSchedule4);
    reset(ignitionSchedule5);
#if (IGN_CHANNELS >= 5)
    reset(ignitionSchedule5);
#endif
#if IGN_CHANNELS >= 6
    reset(ignitionSchedule6);
#endif
#if IGN_CHANNELS >= 7
    reset(ignitionSchedule7);
#endif
#if IGN_CHANNELS >= 8
    reset(ignitionSchedule8);
#endif

  ignition1StartAngle=0;
  ignition1EndAngle=0;
  channel1IgnDegrees=0; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */

  ignition2StartAngle=0;
  ignition2EndAngle=0;
  channel2IgnDegrees=0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */

  ignition3StartAngle=0;
  ignition3EndAngle=0;
  channel3IgnDegrees=0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */

  ignition4StartAngle=0;
  ignition4EndAngle=0;
  channel4IgnDegrees=0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */

#if (IGN_CHANNELS >= 5)
  ignition5StartAngle=0;
  ignition5EndAngle=0;
  channel5IgnDegrees=0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif
#if (IGN_CHANNELS >= 6)
  ignition6StartAngle=0;
  ignition6EndAngle=0;
  channel6IgnDegrees=0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif
#if (IGN_CHANNELS >= 7)
  ignition7StartAngle=0;
  ignition7EndAngle=0;
  channel7IgnDegrees=0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif
#if (IGN_CHANNELS >= 8)
  ignition8StartAngle=0;
  ignition8EndAngle=0;
  channel8IgnDegrees=0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif

	channel1InjDegrees = 0; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */
	channel2InjDegrees = 0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
	channel3InjDegrees = 0; /**< The number of crank degrees until cylinder 3 (and 5/6/7/8) is at TDC */
	channel4InjDegrees = 0; /**< The number of crank degrees until cylinder 4 (and 5/6/7/8) is at TDC */
#if (INJ_CHANNELS >= 5)
	channel5InjDegrees = 0; /**< The number of crank degrees until cylinder 5 is at TDC */
#endif
#if (INJ_CHANNELS >= 6)
	channel6InjDegrees = 0; /**< The number of crank degrees until cylinder 6 is at TDC */
#endif
#if (INJ_CHANNELS >= 7)
	channel7InjDegrees = 0; /**< The number of crank degrees until cylinder 7 is at TDC */
#endif
#if (INJ_CHANNELS >= 8)
	channel8InjDegrees = 0; /**< The number of crank degrees until cylinder 8 is at TDC */
#endif

}

void _setFuelScheduleRunning(FuelSchedule &schedule, unsigned long timeout, unsigned long duration)
{
  schedule.duration = duration;

  //Need to check that the timeout doesn't exceed the overflow
  timeout = min(timeout, (MAX_TIMER_PERIOD - 1));

  //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
  noInterrupts();
  schedule.startCompare = schedule._counter + uS_TO_TIMER_COMPARE(timeout);
  schedule.endCompare = schedule.startCompare + uS_TO_TIMER_COMPARE(duration);
  SET_COMPARE(schedule._compare, schedule.startCompare); //Use the B compare unit of timer 3
  schedule.Status = PENDING; //Turn this schedule on
  interrupts();
  schedule.pTimerEnable();
}

void _setFuelScheduleNext(FuelSchedule &schedule, unsigned long timeout, unsigned long duration)
{
  //If the schedule is already running, we can set the next schedule so it is ready to go
  //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
  schedule.nextStartCompare = schedule._counter + uS_TO_TIMER_COMPARE(timeout);
  schedule.nextEndCompare = schedule.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
  schedule.hasNextSchedule = true;
}

void _setIgnitionScheduleRunning(IgnitionSchedule &schedule, unsigned long timeout, unsigned long duration)
{
  schedule.duration = duration;

  //Need to check that the timeout doesn't exceed the overflow
  COMPARE_TYPE timeout_timer_compare;
  if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when applied causing erratic behaviour such as erroneous sparking.
  else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

  noInterrupts();
  schedule.startCompare = schedule._counter + timeout_timer_compare; //As there is a tick every 4uS, there are timeout/4 ticks until the interrupt should be triggered ( >>2 divides by 4)
  if(schedule.endScheduleSetByDecoder == false) { schedule.endCompare = schedule.startCompare + uS_TO_TIMER_COMPARE(duration); } //The .endCompare value is also set by the per tooth timing in decoders.ino. The check here is so that it's not getting overridden. 
  SET_COMPARE(schedule._compare, schedule.startCompare);
  schedule.Status = PENDING; //Turn this schedule on
  interrupts();
  schedule.pTimerEnable();
}

void _setIgnitionScheduleNext(IgnitionSchedule &schedule, unsigned long timeout, unsigned long duration)
{
  //If the schedule is already running, we can set the next schedule so it is ready to go
  //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
  schedule.nextStartCompare = schedule._counter + uS_TO_TIMER_COMPARE(timeout);
  schedule.nextEndCompare = schedule.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
  schedule.hasNextSchedule = true;
}


void refreshIgnitionSchedule1(unsigned long timeToEnd)
{
  if( (ignitionSchedule1.Status == RUNNING) && (timeToEnd < ignitionSchedule1.duration) )
  //Must have the threshold check here otherwise it can cause a condition where the compare fires twice, once after the other, both for the end
  //if( (timeToEnd < ignitionSchedule1.duration) && (timeToEnd > IGNITION_REFRESH_THRESHOLD) )
  {
    noInterrupts();
    ignitionSchedule1.endCompare = IGN1_COUNTER + uS_TO_TIMER_COMPARE(timeToEnd);
    SET_COMPARE(IGN1_COMPARE, ignitionSchedule1.endCompare);
    interrupts();
  }
}

/** Perform the injector priming pulses.
 * Set these to run at an arbitrary time in the future (100us).
 * The prime pulse value is in ms*10, so need to multiple by 100 to get to uS
 */
extern void beginInjectorPriming(void)
{
  unsigned long primingValue = table2D_getValue(&PrimingPulseTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
  if( (primingValue > 0) && (currentStatus.TPS < configPage4.floodClear) )
  {
    primingValue = primingValue * 100 * 5; //to achieve long enough priming pulses, the values in tuner studio are divided by 0.5 instead of 0.1, so multiplier of 5 is required.
    if ( maxInjOutputs >= 1 ) { setFuelSchedule(fuelSchedule1, 100, primingValue); }
#if (INJ_CHANNELS >= 2)
    if ( maxInjOutputs >= 2 ) { setFuelSchedule(fuelSchedule2, 100, primingValue); }
#endif
#if (INJ_CHANNELS >= 3)
    if ( maxInjOutputs >= 3 ) { setFuelSchedule(fuelSchedule3, 100, primingValue); }
#endif
#if (INJ_CHANNELS >= 4)
    if ( maxInjOutputs >= 4 ) { setFuelSchedule(fuelSchedule4, 100, primingValue); }
#endif
#if (INJ_CHANNELS >= 5)
    if ( maxInjOutputs >= 5 ) { setFuelSchedule(fuelSchedule5, 100, primingValue); }
#endif
#if (INJ_CHANNELS >= 6)
    if ( maxInjOutputs >= 6 ) { setFuelSchedule(fuelSchedule6, 100, primingValue); }
#endif
#if (INJ_CHANNELS >= 7)
    if ( maxInjOutputs >= 7) { setFuelSchedule(fuelSchedule7, 100, primingValue); }
#endif
#if (INJ_CHANNELS >= 8)
    if ( maxInjOutputs >= 8 ) { setFuelSchedule(fuelSchedule8, 100, primingValue); }
#endif
  }
}

// Shared ISR function for all fuel timers.
// This is completely inlined into the ISR - there is no function call
// overhead.
static inline __attribute__((always_inline)) void fuelScheduleISR(FuelSchedule &schedule)
{
  if (schedule.Status == PENDING) //Check to see if this schedule is turn on
  {
    schedule.pStartCallback();
    schedule.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
    SET_COMPARE(schedule._compare, schedule._counter + uS_TO_TIMER_COMPARE(schedule.duration) ); //Doing this here prevents a potential overflow on restarts
  }
  else if (schedule.Status == RUNNING)
  {
      schedule.pEndCallback();
      schedule.Status = OFF; //Turn off the schedule

      //If there is a next schedule queued up, activate it
      if(schedule.hasNextSchedule == true)
      {
        SET_COMPARE(schedule._compare, schedule.nextStartCompare);
        SET_COMPARE(schedule.endCompare, schedule.nextEndCompare);
        schedule.Status = PENDING;
        schedule.hasNextSchedule = false;
      }
      else 
      { 
        schedule.pTimerDisable(); 
      }
  }
  else if (schedule.Status == OFF) 
  { 
    schedule.pTimerDisable(); //Safety check. Turn off this output compare unit and return without performing any action
  } 
} 

/*******************************************************************************************************************************************************************************************************/
/** fuelSchedule*Interrupt (All 8 ISR functions below) get called (as timed interrupts) when either the start time or the duration time are reached.
* This calls the relevant callback function (startCallback or endCallback) depending on the status (PENDING => Needs to run, RUNNING => Needs to stop) of the schedule.
* The status of schedule is managed here based on startCallback /endCallback function called:
* - startCallback - change scheduler into RUNNING state
* - endCallback - change scheduler into OFF state (or PENDING if schedule.hasNextSchedule is set)
*/
//Timer3A (fuel schedule 1) Compare Vector
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
//fuelSchedules 1 and 5
ISR(TIMER3_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule1Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedule1);
  }


#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER3_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule2Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedule2);
  }


#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER3_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule3Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedule3);
  }


#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER4_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule4Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedule4);
  }

#if INJ_CHANNELS >= 5
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule5Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedule5);
  }
#endif

#if INJ_CHANNELS >= 6
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule6Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedule6);
  }
#endif

#if INJ_CHANNELS >= 7
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule7Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedule7);
  }
#endif

#if INJ_CHANNELS >= 8
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void fuelSchedule8Interrupt() //Most ARM chips can simply call a function
#endif
  {
    fuelScheduleISR(fuelSchedule8);
  }
#endif

// Shared ISR function for all ignition timers.
// This is completely inlined into the ISR - there is no function call
// overhead.
static inline __attribute__((always_inline)) void ignitionScheduleISR(IgnitionSchedule &schedule)
{
  if (schedule.Status == PENDING) //Check to see if this schedule is turn on
  {
    schedule.pStartCallback();
    schedule.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
    schedule.startTime = micros();
    if(schedule.endScheduleSetByDecoder == true) { SET_COMPARE(schedule._compare, schedule.endCompare); }
    else { SET_COMPARE(schedule._compare, schedule._counter + uS_TO_TIMER_COMPARE(schedule.duration) ); } //Doing this here prevents a potential overflow on restarts
  }
  else if (schedule.Status == RUNNING)
  {
    schedule.pEndCallback();
    schedule.Status = OFF; //Turn off the schedule
    schedule.endScheduleSetByDecoder = false;
    ignitionCount = ignitionCount + 1; //Increment the ignition counter
    currentStatus.actualDwell = DWELL_AVERAGE( (micros() - schedule.startTime) );

    //If there is a next schedule queued up, activate it
    if(schedule.hasNextSchedule == true)
    {
      SET_COMPARE(schedule._compare, schedule.nextStartCompare);
      schedule.Status = PENDING;
      schedule.hasNextSchedule = false;
    }
    else
    { 
      schedule.pTimerDisable(); 
    }
  }
  else if (schedule.Status == OFF)
  {
    //Catch any spurious interrupts. This really shouldn't ever be called, but there as a safety
    schedule.pTimerDisable(); 
  }
}

#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule1Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule1);
  }

#if IGN_CHANNELS >= 2
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule2Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule2);
  }
#endif

#if IGN_CHANNELS >= 3
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule3Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule3);
  }
#endif

#if IGN_CHANNELS >= 4
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule4Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule4);
  }
#endif

#if IGN_CHANNELS >= 5
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule5Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule5);
  }
#endif

#if IGN_CHANNELS >= 6
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule6Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule6);
  }
#endif

#if IGN_CHANNELS >= 7
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER3_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule7Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule7);
  }
#endif

#if IGN_CHANNELS >= 8
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER3_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule8Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedule8);
  }
#endif

void disablePendingFuelSchedule(byte channel)
{
  noInterrupts();
  switch(channel)
  {
    case 0:
      if(fuelSchedule1.Status == PENDING) { fuelSchedule1.Status = OFF; }
      break;
    case 1:
      if(fuelSchedule2.Status == PENDING) { fuelSchedule2.Status = OFF; }
      break;
    case 2: 
      if(fuelSchedule3.Status == PENDING) { fuelSchedule3.Status = OFF; }
      break;
    case 3:
      if(fuelSchedule4.Status == PENDING) { fuelSchedule4.Status = OFF; }
      break;
    case 4:
#if (INJ_CHANNELS >= 5)
      if(fuelSchedule5.Status == PENDING) { fuelSchedule5.Status = OFF; }
#endif
      break;
    case 5:
#if (INJ_CHANNELS >= 6)
      if(fuelSchedule6.Status == PENDING) { fuelSchedule6.Status = OFF; }
#endif
      break;
    case 6:
#if (INJ_CHANNELS >= 7)
      if(fuelSchedule7.Status == PENDING) { fuelSchedule7.Status = OFF; }
#endif
      break;
    case 7:
#if (INJ_CHANNELS >= 8)
      if(fuelSchedule8.Status == PENDING) { fuelSchedule8.Status = OFF; }
#endif
      break;
  }
  interrupts();
}
void disablePendingIgnSchedule(byte channel)
{
  noInterrupts();
  switch(channel)
  {
    case 0:
      if(ignitionSchedule1.Status == PENDING) { ignitionSchedule1.Status = OFF; }
      break;
    case 1:
      if(ignitionSchedule2.Status == PENDING) { ignitionSchedule2.Status = OFF; }
      break;
    case 2: 
      if(ignitionSchedule3.Status == PENDING) { ignitionSchedule3.Status = OFF; }
      break;
    case 3:
      if(ignitionSchedule4.Status == PENDING) { ignitionSchedule4.Status = OFF; }
      break;
    case 4:
      if(ignitionSchedule5.Status == PENDING) { ignitionSchedule5.Status = OFF; }
      break;
#if IGN_CHANNELS >= 6      
    case 6:
      if(ignitionSchedule6.Status == PENDING) { ignitionSchedule6.Status = OFF; }
      break;
#endif
#if IGN_CHANNELS >= 7      
    case 7:
      if(ignitionSchedule7.Status == PENDING) { ignitionSchedule7.Status = OFF; }
      break;
#endif
#if IGN_CHANNELS >= 8      
    case 8:
      if(ignitionSchedule8.Status == PENDING) { ignitionSchedule8.Status = OFF; }
      break;
#endif
  }
  interrupts();
}
