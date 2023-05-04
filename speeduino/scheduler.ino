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

FuelSchedule fuelSchedule1;
FuelSchedule fuelSchedule2;
FuelSchedule fuelSchedule3;
FuelSchedule fuelSchedule4;
FuelSchedule fuelSchedule5;
FuelSchedule fuelSchedule6;
FuelSchedule fuelSchedule7;
FuelSchedule fuelSchedule8;

Schedule ignitionSchedule1;
Schedule ignitionSchedule2;
Schedule ignitionSchedule3;
Schedule ignitionSchedule4;
Schedule ignitionSchedule5;
Schedule ignitionSchedule6;
Schedule ignitionSchedule7;
Schedule ignitionSchedule8;

void (*inj1StartFunction)(void);
void (*inj1EndFunction)(void);
void (*inj2StartFunction)(void);
void (*inj2EndFunction)(void);
void (*inj3StartFunction)(void);
void (*inj3EndFunction)(void);
void (*inj4StartFunction)(void);
void (*inj4EndFunction)(void);
void (*inj5StartFunction)(void);
void (*inj5EndFunction)(void);
void (*inj6StartFunction)(void);
void (*inj6EndFunction)(void);
void (*inj7StartFunction)(void);
void (*inj7EndFunction)(void);
void (*inj8StartFunction)(void);
void (*inj8EndFunction)(void);

void (*ign1StartFunction)(void);
void (*ign1EndFunction)(void);
void (*ign2StartFunction)(void);
void (*ign2EndFunction)(void);
void (*ign3StartFunction)(void);
void (*ign3EndFunction)(void);
void (*ign4StartFunction)(void);
void (*ign4EndFunction)(void);
void (*ign5StartFunction)(void);
void (*ign5EndFunction)(void);
void (*ign6StartFunction)(void);
void (*ign6EndFunction)(void);
void (*ign7StartFunction)(void);
void (*ign7EndFunction)(void);
void (*ign8StartFunction)(void);
void (*ign8EndFunction)(void);

void initialiseSchedulers(void)
{
    //nullSchedule.Status = OFF;

    fuelSchedule1.Status = OFF;
    fuelSchedule2.Status = OFF;
    fuelSchedule3.Status = OFF;
    fuelSchedule4.Status = OFF;
    fuelSchedule5.Status = OFF;
    fuelSchedule6.Status = OFF;
    fuelSchedule7.Status = OFF;
    fuelSchedule8.Status = OFF;

    fuelSchedule1.schedulesSet = 0;
    fuelSchedule2.schedulesSet = 0;
    fuelSchedule3.schedulesSet = 0;
    fuelSchedule4.schedulesSet = 0;
    fuelSchedule5.schedulesSet = 0;
    fuelSchedule6.schedulesSet = 0;
    fuelSchedule7.schedulesSet = 0;
    fuelSchedule8.schedulesSet = 0;

    ignitionSchedule1.Status = OFF;
    ignitionSchedule2.Status = OFF;
    ignitionSchedule3.Status = OFF;
    ignitionSchedule4.Status = OFF;
    ignitionSchedule5.Status = OFF;
    ignitionSchedule6.Status = OFF;
    ignitionSchedule7.Status = OFF;
    ignitionSchedule8.Status = OFF;

    IGN1_TIMER_ENABLE();
    IGN2_TIMER_ENABLE();
    IGN3_TIMER_ENABLE();
    IGN4_TIMER_ENABLE();
#if (IGN_CHANNELS >= 5)
    IGN5_TIMER_ENABLE();
    IGN6_TIMER_ENABLE();
    IGN7_TIMER_ENABLE();
    IGN8_TIMER_ENABLE();
#endif

    FUEL1_TIMER_ENABLE();
    FUEL2_TIMER_ENABLE();
    FUEL3_TIMER_ENABLE();
    FUEL4_TIMER_ENABLE();
#if (INJ_CHANNELS >= 5)
    FUEL5_TIMER_ENABLE();
    FUEL6_TIMER_ENABLE();
    FUEL7_TIMER_ENABLE();
    FUEL8_TIMER_ENABLE();
#endif

    ignitionSchedule1.schedulesSet = 0;
    ignitionSchedule2.schedulesSet = 0;
    ignitionSchedule3.schedulesSet = 0;
    ignitionSchedule4.schedulesSet = 0;
    ignitionSchedule5.schedulesSet = 0;
    ignitionSchedule6.schedulesSet = 0;
    ignitionSchedule7.schedulesSet = 0;
    ignitionSchedule8.schedulesSet = 0;

    ignition1StartAngle = 0;
    ignition2StartAngle = 0;
    ignition3StartAngle = 0;
    ignition4StartAngle = 0;
    ignition5StartAngle = 0;
    ignition6StartAngle = 0;
    ignition7StartAngle = 0;
    ignition8StartAngle = 0;

    channel1IgnDegrees = 0; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */
    channel2IgnDegrees = 0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
    channel3IgnDegrees = 0; /**< The number of crank degrees until cylinder 3 (and 5/6/7/8) is at TDC */
    channel4IgnDegrees = 0; /**< The number of crank degrees until cylinder 4 (and 5/6/7/8) is at TDC */
    channel5IgnDegrees = 0; /**< The number of crank degrees until cylinder 5 is at TDC */
    channel6IgnDegrees = 0; /**< The number of crank degrees until cylinder 6 is at TDC */
    channel7IgnDegrees = 0; /**< The number of crank degrees until cylinder 7 is at TDC */
    channel8IgnDegrees = 0; /**< The number of crank degrees until cylinder 8 is at TDC */
    
    channel1InjDegrees = 0; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */
    channel2InjDegrees = 0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
    channel3InjDegrees = 0; /**< The number of crank degrees until cylinder 3 (and 5/6/7/8) is at TDC */
    channel4InjDegrees = 0; /**< The number of crank degrees until cylinder 4 (and 5/6/7/8) is at TDC */
    channel5InjDegrees = 0; /**< The number of crank degrees until cylinder 5 is at TDC */
    channel6InjDegrees = 0; /**< The number of crank degrees until cylinder 6 is at TDC */
    channel7InjDegrees = 0; /**< The number of crank degrees until cylinder 7 is at TDC */
    channel8InjDegrees = 0; /**< The number of crank degrees until cylinder 8 is at TDC */    
}

/*
These 8 function turn a schedule on, provides the time to start and the duration and gives it callback functions.
All 8 functions operate the same, just on different schedules
Args:
startCallback: The function to be called once the timeout is reached
timeout: The number of uS in the future that the startCallback should be triggered
duration: The number of uS after startCallback is called before endCallback is called
endCallback: This function is called once the duration time has been reached
*/

// Helper functions for injection schedulers
static void SET_INJ1_CMP(COMPARE_TYPE compare) { FUEL1_COMPARE = compare; }
static void SET_INJ2_CMP(COMPARE_TYPE compare) { FUEL2_COMPARE = compare; }
static void SET_INJ3_CMP(COMPARE_TYPE compare) { FUEL3_COMPARE = compare; }
static void SET_INJ4_CMP(COMPARE_TYPE compare) { FUEL4_COMPARE = compare; }
static void SET_INJ5_CMP(COMPARE_TYPE compare) { FUEL5_COMPARE = compare; }
static void SET_INJ6_CMP(COMPARE_TYPE compare) { FUEL6_COMPARE = compare; }
static void SET_INJ7_CMP(COMPARE_TYPE compare) { FUEL7_COMPARE = compare; }
static void SET_INJ8_CMP(COMPARE_TYPE compare) { FUEL8_COMPARE = compare; }

static COUNTER_TYPE GET_INJ1_CNT() { return FUEL1_COUNTER; }
static COUNTER_TYPE GET_INJ2_CNT() { return FUEL2_COUNTER; }
static COUNTER_TYPE GET_INJ3_CNT() { return FUEL3_COUNTER; }
static COUNTER_TYPE GET_INJ4_CNT() { return FUEL4_COUNTER; }
static COUNTER_TYPE GET_INJ5_CNT() { return FUEL5_COUNTER; }
static COUNTER_TYPE GET_INJ6_CNT() { return FUEL6_COUNTER; }
static COUNTER_TYPE GET_INJ7_CNT() { return FUEL7_COUNTER; }
static COUNTER_TYPE GET_INJ8_CNT() { return FUEL8_COUNTER; }


static inline __attribute__((always_inline)) // <-- this is critical for performance
void setFuelSchedule(unsigned long timeout,
  unsigned long duration,
  FuelSchedule& schedule,
  void (*timer_enable)(),
  void (*set_compare)(COMPARE_TYPE),
  COMPARE_TYPE (*get_counter)(),
  const bool safety_check)
{
  if (safety_check) // why is FuelSchedule1 different from every other?
  {
    //Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
    //if(timeout < MAX_TIMER_PERIOD)
    {
      if(schedule.Status != RUNNING) //Check that we're not already part way through a schedule
      {
        //Need to check that the timeout doesn't exceed the overflow
        if ((timeout+duration) < MAX_TIMER_PERIOD)
        {
          schedule.duration = duration;
          //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
          noInterrupts();
          schedule.startCompare = get_counter() + uS_TO_TIMER_COMPARE(timeout);
          schedule.endCompare = schedule.startCompare + uS_TO_TIMER_COMPARE(duration);
          schedule.Status = PENDING; //Turn this schedule on
          schedule.schedulesSet++; //Increment the number of times this schedule has been set
          //Schedule 1 shares a timer with schedule 5
          //if(channel5InjEnabled) { SET_COMPARE(compare, setQueue(timer3Aqueue, &schedule, &fuelSchedule5, FUEL1_COUNTER) ); }
          //else { timer3Aqueue[0] = &schedule; timer3Aqueue[1] = &schedule; timer3Aqueue[2] = &schedule; timer3Aqueue[3] = &schedule; SET_COMPARE(compare, fuelSchedule1.startCompare); }
          //timer3Aqueue[0] = &schedule; timer3Aqueue[1] = &schedule; timer3Aqueue[2] = &schedule; timer3Aqueue[3] = &schedule;
          set_compare(schedule.startCompare);
          interrupts();
          timer_enable();
        }
      }
      else
      {
        //If the schedule is already running, we can set the next schedule so it is ready to go
        //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
        if( (timeout+duration) < MAX_TIMER_PERIOD )
        {
          noInterrupts();
          schedule.nextStartCompare = get_counter() + uS_TO_TIMER_COMPARE(timeout);
          schedule.nextEndCompare = schedule.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
          schedule.duration = duration;
          schedule.hasNextSchedule = true;
          interrupts();
        }
      } //Schedule is RUNNING
    } //Timeout less than threshold
  }
  else
  {
    //Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
    if(timeout < MAX_TIMER_PERIOD)
    {
      if(schedule.Status != RUNNING) //Check that we're not already part way through a schedule
      {
        //Callbacks no longer used, but retained for now:
        //schedule.StartCallback = startCallback;
        //schedule.EndCallback = endCallback;
        schedule.duration = duration;

        //Need to check that the timeout doesn't exceed the overflow
        COMPARE_TYPE timeout_timer_compare;
        if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when applied causing erratic behaviour such as erroneous sparking.
        else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

        //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
        noInterrupts();
        schedule.startCompare = get_counter() + timeout_timer_compare;
        schedule.endCompare = schedule.startCompare + uS_TO_TIMER_COMPARE(duration);
        set_compare(schedule.startCompare); //Use the B compare unit of timer 3
        schedule.Status = PENDING; //Turn this schedule on
        schedule.schedulesSet++; //Increment the number of times this schedule has been set
        interrupts();
        timer_enable();
      }
      else
      {
        //If the schedule is already running, we can set the next schedule so it is ready to go
        //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
        schedule.nextStartCompare = get_counter() + uS_TO_TIMER_COMPARE(timeout);
        schedule.nextEndCompare = schedule.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
        schedule.hasNextSchedule = true;
      }
    }
  }
}

//void setFuelSchedule1(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
void setFuelSchedule1(unsigned long timeout, unsigned long duration) //Uses timer 3 compare A
{
  setFuelSchedule(timeout, duration, fuelSchedule1, FUEL1_TIMER_ENABLE, SET_INJ1_CMP, GET_INJ1_CNT, true);
}
void setFuelSchedule2(unsigned long timeout, unsigned long duration) //Uses timer 3 compare B
{
  setFuelSchedule(timeout, duration, fuelSchedule2, FUEL2_TIMER_ENABLE, SET_INJ2_CMP, GET_INJ2_CNT, false);
}
void setFuelSchedule3(unsigned long timeout, unsigned long duration) //Uses timer 3 compare C
{
  setFuelSchedule(timeout, duration, fuelSchedule3, FUEL3_TIMER_ENABLE, SET_INJ3_CMP, GET_INJ3_CNT, false);
}
void setFuelSchedule4(unsigned long timeout, unsigned long duration) //Uses timer 4 compare B
{
  setFuelSchedule(timeout, duration, fuelSchedule4, FUEL4_TIMER_ENABLE, SET_INJ4_CMP, GET_INJ4_CNT, false);
}
void setFuelSchedule5(unsigned long timeout, unsigned long duration) //Uses timer 4 compare C
{
  setFuelSchedule(timeout, duration, fuelSchedule5, FUEL5_TIMER_ENABLE, SET_INJ5_CMP, GET_INJ5_CNT, false);
}
void setFuelSchedule6(unsigned long timeout, unsigned long duration) //Uses timer 4 compare A
{
  setFuelSchedule(timeout, duration, fuelSchedule6, FUEL6_TIMER_ENABLE, SET_INJ6_CMP, GET_INJ6_CNT, false);
}
void setFuelSchedule7(unsigned long timeout, unsigned long duration) //Uses timer 5 compare C
{
  setFuelSchedule(timeout, duration, fuelSchedule7, FUEL7_TIMER_ENABLE, SET_INJ7_CMP, GET_INJ7_CNT, false);
}
void setFuelSchedule8(unsigned long timeout, unsigned long duration) //Uses timer 5 compare B
{
  setFuelSchedule(timeout, duration, fuelSchedule8, FUEL8_TIMER_ENABLE, SET_INJ8_CMP, GET_INJ8_CNT, false);
}

// Helper functions for ignition schedulers
static void SET_IGN1_CMP(COMPARE_TYPE compare) { IGN1_COMPARE = compare; }
static void SET_IGN2_CMP(COMPARE_TYPE compare) { IGN2_COMPARE = compare; }
static void SET_IGN3_CMP(COMPARE_TYPE compare) { IGN3_COMPARE = compare; }
static void SET_IGN4_CMP(COMPARE_TYPE compare) { IGN4_COMPARE = compare; }
static void SET_IGN5_CMP(COMPARE_TYPE compare) { IGN5_COMPARE = compare; }
static void SET_IGN6_CMP(COMPARE_TYPE compare) { IGN6_COMPARE = compare; }
static void SET_IGN7_CMP(COMPARE_TYPE compare) { IGN7_COMPARE = compare; }
static void SET_IGN8_CMP(COMPARE_TYPE compare) { IGN8_COMPARE = compare; }

static COUNTER_TYPE GET_IGN1_CNT() { return IGN1_COUNTER; }
static COUNTER_TYPE GET_IGN2_CNT() { return IGN2_COUNTER; }
static COUNTER_TYPE GET_IGN3_CNT() { return IGN3_COUNTER; }
static COUNTER_TYPE GET_IGN4_CNT() { return IGN4_COUNTER; }
static COUNTER_TYPE GET_IGN5_CNT() { return IGN5_COUNTER; }
static COUNTER_TYPE GET_IGN6_CNT() { return IGN6_COUNTER; }
static COUNTER_TYPE GET_IGN7_CNT() { return IGN7_COUNTER; }
static COUNTER_TYPE GET_IGN8_CNT() { return IGN8_COUNTER; }

static inline __attribute__((always_inline)) // <-- this is critical for performance
void setIgnitionSchedule(
  void (*startCallback)(),
  unsigned long timeout,
  unsigned long duration,
  void(*endCallback)(),
  Schedule& schedule,
  void (*timer_enable)(),
  void (*set_compare)(COMPARE_TYPE),
  COMPARE_TYPE (*get_counter)())
{
  if(schedule.Status != RUNNING) //Check that we're not already part way through a schedule
  {
    schedule.StartCallback = startCallback; //Name the start callback function
    schedule.EndCallback = endCallback; //Name the start callback function
    schedule.duration = duration;

    //Need to check that the timeout doesn't exceed the overflow
    COMPARE_TYPE timeout_timer_compare;
    //timeout -= (micros() - lastCrankAngleCalc);
    if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when applied causing erratic behaviour such as erroneous sparking.
    else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

    noInterrupts();
    schedule.startCompare = get_counter() + timeout_timer_compare; //As there is a tick every 4uS, there are timeout/4 ticks until the interrupt should be triggered ( >>2 divides by 4)
    if(schedule.endScheduleSetByDecoder == false) { schedule.endCompare = schedule.startCompare + uS_TO_TIMER_COMPARE(duration); } //The .endCompare value is also set by the per tooth timing in decoders.ino. The check here is so that it's not getting overridden. 
    set_compare(schedule.startCompare);
    schedule.Status = PENDING; //Turn this schedule on
    schedule.schedulesSet++;
    interrupts();
    timer_enable();
  }
  else
  {
    //If the schedule is already running, we can set the next schedule so it is ready to go
    //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
    if (timeout < MAX_TIMER_PERIOD)
    {
      schedule.nextStartCompare = get_counter() + uS_TO_TIMER_COMPARE(timeout);
      schedule.nextEndCompare = schedule.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
      schedule.hasNextSchedule = true;
    }
  }
}

void setIgnitionSchedule1(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  setIgnitionSchedule(startCallback, timeout, duration, endCallback, ignitionSchedule1, IGN1_TIMER_ENABLE, SET_IGN1_CMP, GET_IGN1_CNT);
}

inline void refreshIgnitionSchedule1(unsigned long timeToEnd)
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

void setIgnitionSchedule2(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  setIgnitionSchedule(startCallback, timeout, duration, endCallback, ignitionSchedule2, IGN2_TIMER_ENABLE, SET_IGN2_CMP, GET_IGN2_CNT);
}
void setIgnitionSchedule3(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  setIgnitionSchedule(startCallback, timeout, duration, endCallback, ignitionSchedule3, IGN3_TIMER_ENABLE, SET_IGN3_CMP, GET_IGN3_CNT);
}
void setIgnitionSchedule4(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  setIgnitionSchedule(startCallback, timeout, duration, endCallback, ignitionSchedule4, IGN4_TIMER_ENABLE, SET_IGN4_CMP, GET_IGN4_CNT);
}
void setIgnitionSchedule5(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  setIgnitionSchedule(startCallback, timeout, duration, endCallback, ignitionSchedule5, IGN5_TIMER_ENABLE, SET_IGN5_CMP, GET_IGN5_CNT);
}
void setIgnitionSchedule6(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  setIgnitionSchedule(startCallback, timeout, duration, endCallback, ignitionSchedule6, IGN6_TIMER_ENABLE, SET_IGN6_CMP, GET_IGN6_CNT);
}
void setIgnitionSchedule7(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  setIgnitionSchedule(startCallback, timeout, duration, endCallback, ignitionSchedule7, IGN7_TIMER_ENABLE, SET_IGN7_CMP, GET_IGN7_CNT);
}
void setIgnitionSchedule8(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
{
  setIgnitionSchedule(startCallback, timeout, duration, endCallback, ignitionSchedule8, IGN8_TIMER_ENABLE, SET_IGN8_CMP, GET_IGN8_CNT);
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
    if ( BIT_CHECK(channelInjEnabled, INJ1_CMD_BIT) == true ) { setFuelSchedule1(100, primingValue); }
#if (INJ_CHANNELS >= 2)
    if ( BIT_CHECK(channelInjEnabled, INJ2_CMD_BIT) == true ) { setFuelSchedule2(100, primingValue); }
#endif
#if (INJ_CHANNELS >= 3)
    if ( BIT_CHECK(channelInjEnabled, INJ3_CMD_BIT) == true ) { setFuelSchedule3(100, primingValue); }
#endif
#if (INJ_CHANNELS >= 4)
    if ( BIT_CHECK(channelInjEnabled, INJ4_CMD_BIT) == true ) { setFuelSchedule4(100, primingValue); }
#endif
#if (INJ_CHANNELS >= 5)
    if ( BIT_CHECK(channelInjEnabled, INJ5_CMD_BIT) == true ) { setFuelSchedule5(100, primingValue); }
#endif
#if (INJ_CHANNELS >= 6)
    if ( BIT_CHECK(channelInjEnabled, INJ6_CMD_BIT) == true ) { setFuelSchedule6(100, primingValue); }
#endif
#if (INJ_CHANNELS >= 7)
    if ( BIT_CHECK(channelInjEnabled, INJ7_CMD_BIT) == true) { setFuelSchedule7(100, primingValue); }
#endif
#if (INJ_CHANNELS >= 8)
    if ( BIT_CHECK(channelInjEnabled, INJ8_CMD_BIT) == true ) { setFuelSchedule8(100, primingValue); }
#endif
  }
}

/*******************************************************************************************************************************************************************************************************/
/** fuelSchedule*Interrupt (All 8 ISR functions below) get called (as timed interrupts) when either the start time or the duration time are reached.
* This calls the relevant callback function (startCallback or endCallback) depending on the status (PENDING => Needs to run, RUNNING => Needs to stop) of the schedule.
* The status of schedule is managed here based on startCallback /endCallback function called:
* - startCallback - change scheduler into RUNNING state
* - endCallback - change scheduler into OFF state (or PENDING if schedule.hasNextSchedule is set)
*/

static inline __attribute__((always_inline)) // <-- this is critical for performance
void fuelScheduleInterrupt(
  FuelSchedule& schedule,
  void (*timer_disable)(),
  COUNTER_TYPE (*get_counter)(),
  void (*set_compare)(COMPARE_TYPE),
  void (*start_function)(),
  void (*end_function)(),
  const bool safety_check)
{
    if (schedule.Status == PENDING) //Check to see if this schedule is turn on
    {
      //To use timer queue, change fuelShedule1 to timer3Aqueue[0];
      start_function();
      schedule.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      set_compare(get_counter() + uS_TO_TIMER_COMPARE(schedule.duration) ); //Doing this here prevents a potential overflow on restarts
    }
    else if (schedule.Status == RUNNING)
    {
       //timer3Aqueue[0]->EndCallback();
       end_function();
       schedule.Status = OFF; //Turn off the schedule
       schedule.schedulesSet = 0;

       //If there is a next schedule queued up, activate it
       if(schedule.hasNextSchedule == true)
       {
         set_compare(schedule.nextStartCompare);
         schedule.endCompare = schedule.nextEndCompare;
         schedule.Status = PENDING;
         schedule.schedulesSet = 1;
         schedule.hasNextSchedule = false;
       }
       else { timer_disable(); }
    }
    else if (safety_check)
    {
      if (schedule.Status == OFF) { timer_disable(); } //Safety check. Turn off this output compare unit and return without performing any action
    }
}

//Timer3A (fuel schedule 1) Compare Vector
#if (INJ_CHANNELS >= 1)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
//fuelSchedules 1 and 5
ISR(TIMER3_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void fuelSchedule1Interrupt(void)
#endif
  {
    fuelScheduleInterrupt(fuelSchedule1, FUEL1_TIMER_DISABLE, GET_INJ1_CNT, SET_INJ1_CMP, inj1StartFunction, inj1EndFunction, true);
  }
#endif

#if (INJ_CHANNELS >= 2)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER3_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void fuelSchedule2Interrupt(void)
#endif
  {
    fuelScheduleInterrupt(fuelSchedule2, FUEL2_TIMER_DISABLE, GET_IGN2_CNT, SET_IGN2_CMP, inj2StartFunction, inj2EndFunction, false);
  }
#endif

#if (INJ_CHANNELS >= 3)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER3_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void fuelSchedule3Interrupt(void)
#endif
  {
    fuelScheduleInterrupt(fuelSchedule3, FUEL3_TIMER_DISABLE, GET_IGN3_CNT, SET_IGN3_CMP, inj3StartFunction, inj3EndFunction, false);
  }
#endif

#if (INJ_CHANNELS >= 4)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER4_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void fuelSchedule4Interrupt(void)
#endif
  {
    fuelScheduleInterrupt(fuelSchedule4, FUEL4_TIMER_DISABLE, GET_IGN4_CNT, SET_IGN4_CMP, inj4StartFunction, inj4EndFunction, false);
  }
#endif

#if (INJ_CHANNELS >= 5)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void fuelSchedule5Interrupt(void)
#endif
{
    fuelScheduleInterrupt(fuelSchedule5, FUEL5_TIMER_DISABLE, GET_IGN5_CNT, SET_IGN5_CMP, inj5StartFunction, inj5EndFunction, false);
}
#endif

#if (INJ_CHANNELS >= 6)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void fuelSchedule6Interrupt(void)
#endif
{
    fuelScheduleInterrupt(fuelSchedule6, FUEL6_TIMER_DISABLE, GET_IGN6_CNT, SET_IGN6_CMP, inj6StartFunction, inj6EndFunction, false);
}
#endif

#if (INJ_CHANNELS >= 7)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void fuelSchedule7Interrupt(void)
#endif
{
    fuelScheduleInterrupt(fuelSchedule7, FUEL7_TIMER_DISABLE, GET_IGN7_CNT, SET_IGN7_CMP, inj7StartFunction, inj7EndFunction, false);
}
#endif

#if (INJ_CHANNELS >= 8)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void fuelSchedule8Interrupt(void)
#endif
{
    fuelScheduleInterrupt(fuelSchedule8, FUEL8_TIMER_DISABLE, GET_IGN8_CNT, SET_IGN8_CMP, inj8StartFunction, inj8EndFunction, false);
}
#endif

static inline __attribute__((always_inline)) // <-- this is critical for performance
void ignitionScheduleInterrupt(
  Schedule& schedule,
  void (*timer_disable)(),
  COUNTER_TYPE (*get_counter)(),
  void (*set_compare)(COMPARE_TYPE))
{
    if (schedule.Status == PENDING) //Check to see if this schedule is turn on
    {
      schedule.StartCallback();
      schedule.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      schedule.startTime = micros();
      if(schedule.endScheduleSetByDecoder == true) { set_compare(schedule.endCompare); }
      else { set_compare(get_counter() + uS_TO_TIMER_COMPARE(schedule.duration) ); } //Doing this here prevents a potential overflow on restarts
    }
    else if (schedule.Status == RUNNING)
    {
      schedule.EndCallback();
      schedule.Status = OFF; //Turn off the schedule
      schedule.schedulesSet = 0;
      schedule.endScheduleSetByDecoder = false;
      ignitionCount += 1; //Increment the ignition counter

      //If there is a next schedule queued up, activate it
      if(schedule.hasNextSchedule == true)
      {
        set_compare(schedule.nextStartCompare);
        schedule.Status = PENDING;
        schedule.schedulesSet = 1;
        schedule.hasNextSchedule = false;
      }
      else{ timer_disable(); }
    }
    else if (schedule.Status == OFF)
    {
      //Catch any spurious interrupts. This really shouldn't ever be called, but there as a safety
      timer_disable();
    }
}
#if IGN_CHANNELS >= 1
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void ignitionSchedule1Interrupt(void)
#endif
  {
    ignitionScheduleInterrupt(ignitionSchedule1, IGN1_TIMER_DISABLE, GET_IGN1_CNT, SET_IGN1_CMP);
  }
#endif

#if IGN_CHANNELS >= 2
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void ignitionSchedule2Interrupt(void)
#endif
  {
    ignitionScheduleInterrupt(ignitionSchedule2, IGN2_TIMER_DISABLE, GET_IGN2_CNT, SET_IGN2_CMP);
  }
#endif

#if IGN_CHANNELS >= 3
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void ignitionSchedule3Interrupt(void)
#endif
  {
    ignitionScheduleInterrupt(ignitionSchedule3, IGN3_TIMER_DISABLE, GET_IGN3_CNT, SET_IGN3_CMP);
  }
#endif

#if IGN_CHANNELS >= 4
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void ignitionSchedule4Interrupt(void)
#endif
  {
    ignitionScheduleInterrupt(ignitionSchedule4, IGN4_TIMER_DISABLE, GET_IGN4_CNT, SET_IGN4_CMP);
  }
#endif

#if IGN_CHANNELS >= 5
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void ignitionSchedule5Interrupt(void)
#endif
  {
    ignitionScheduleInterrupt(ignitionSchedule5, IGN5_TIMER_DISABLE, GET_IGN5_CNT, SET_IGN5_CMP);
  }
#endif

#if IGN_CHANNELS >= 6
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void ignitionSchedule6Interrupt(void)
#endif
  {
    ignitionScheduleInterrupt(ignitionSchedule6, IGN6_TIMER_DISABLE, GET_IGN6_CNT, SET_IGN6_CMP);
  }
#endif

#if IGN_CHANNELS >= 7
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER3_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void ignitionSchedule7Interrupt(void)
#endif
  {
    ignitionScheduleInterrupt(ignitionSchedule7, IGN7_TIMER_DISABLE, GET_IGN7_CNT, SET_IGN7_CMP);
  }
#endif

#if IGN_CHANNELS >= 8
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER3_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
inline void ignitionSchedule8Interrupt(void)
#endif
  {
    ignitionScheduleInterrupt(ignitionSchedule8, IGN8_TIMER_DISABLE, GET_IGN8_CNT, SET_IGN8_CMP);
  }
#endif
