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
#include "crankMaths.h"

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

void (*inj1StartFunction)();
void (*inj1EndFunction)();
void (*inj2StartFunction)();
void (*inj2EndFunction)();
void (*inj3StartFunction)();
void (*inj3EndFunction)();
void (*inj4StartFunction)();
void (*inj4EndFunction)();
void (*inj5StartFunction)();
void (*inj5EndFunction)();
void (*inj6StartFunction)();
void (*inj6EndFunction)();
void (*inj7StartFunction)();
void (*inj7EndFunction)();
void (*inj8StartFunction)();
void (*inj8EndFunction)();

void (*ign1StartFunction)();
void (*ign1EndFunction)();
void (*ign2StartFunction)();
void (*ign2EndFunction)();
void (*ign3StartFunction)();
void (*ign3EndFunction)();
void (*ign4StartFunction)();
void (*ign4EndFunction)();
void (*ign5StartFunction)();
void (*ign5EndFunction)();
void (*ign6StartFunction)();
void (*ign6EndFunction)();
void (*ign7StartFunction)();
void (*ign7EndFunction)();
void (*ign8StartFunction)();
void (*ign8EndFunction)();

void initialiseSchedulers()
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
    IGN5_TIMER_ENABLE();
    IGN6_TIMER_ENABLE();
    IGN7_TIMER_ENABLE();
    IGN8_TIMER_ENABLE();

    ignitionSchedule1.schedulesSet = 0;
    ignitionSchedule2.schedulesSet = 0;
    ignitionSchedule3.schedulesSet = 0;
    ignitionSchedule4.schedulesSet = 0;
    ignitionSchedule5.schedulesSet = 0;
    ignitionSchedule6.schedulesSet = 0;
    ignitionSchedule7.schedulesSet = 0;
    ignitionSchedule8.schedulesSet = 0;

    //set counter related functions
    ignitionSchedule1.getIgnCounter = getIgn1Counter;
    ignitionSchedule2.getIgnCounter = getIgn2Counter;
    ignitionSchedule3.getIgnCounter = getIgn3Counter;
    ignitionSchedule4.getIgnCounter = getIgn4Counter;
    ignitionSchedule5.getIgnCounter = getIgn5Counter;
    ignitionSchedule6.getIgnCounter = getIgn6Counter;
    ignitionSchedule7.getIgnCounter = getIgn7Counter;
    ignitionSchedule8.getIgnCounter = getIgn8Counter;
 
    ignitionSchedule1.setIgnitionCompare = setIgnition1Compare;
    ignitionSchedule2.setIgnitionCompare = setIgnition2Compare;
    ignitionSchedule3.setIgnitionCompare = setIgnition3Compare;
    ignitionSchedule4.setIgnitionCompare = setIgnition4Compare;
    ignitionSchedule5.setIgnitionCompare = setIgnition5Compare;
    ignitionSchedule6.setIgnitionCompare = setIgnition6Compare;
    ignitionSchedule7.setIgnitionCompare = setIgnition7Compare;
    ignitionSchedule8.setIgnitionCompare = setIgnition8Compare;

    ignitionSchedule1.ignTimerEnable = ign1TimerEnable;
    ignitionSchedule2.ignTimerEnable = ign2TimerEnable;
    ignitionSchedule3.ignTimerEnable = ign3TimerEnable;
    ignitionSchedule4.ignTimerEnable = ign4TimerEnable;
    ignitionSchedule5.ignTimerEnable = ign5TimerEnable;
    ignitionSchedule6.ignTimerEnable = ign6TimerEnable;
    ignitionSchedule7.ignTimerEnable = ign7TimerEnable;
    ignitionSchedule8.ignTimerEnable = ign8TimerEnable;

    ignitionSchedule1.StartCallback = ign1StartFunction; //Name the start callback function
    ignitionSchedule2.StartCallback = ign2StartFunction; //Name the start callback function
    ignitionSchedule3.StartCallback = ign3StartFunction; //Name the start callback function
    ignitionSchedule4.StartCallback = ign4StartFunction; //Name the start callback function
    ignitionSchedule5.StartCallback = ign5StartFunction; //Name the start callback function
    ignitionSchedule6.StartCallback = ign6StartFunction; //Name the start callback function
    ignitionSchedule7.StartCallback = ign7StartFunction; //Name the start callback function
    ignitionSchedule8.StartCallback = ign8StartFunction; //Name the start callback function

    ignitionSchedule1.EndCallback = ign1EndFunction; //Name the end callback function
    ignitionSchedule2.EndCallback = ign2EndFunction; //Name the end callback function
    ignitionSchedule3.EndCallback = ign3EndFunction; //Name the end callback function
    ignitionSchedule4.EndCallback = ign4EndFunction; //Name the end callback function
    ignitionSchedule5.EndCallback = ign5EndFunction; //Name the end callback function
    ignitionSchedule6.EndCallback = ign6EndFunction; //Name the end callback function
    ignitionSchedule7.EndCallback = ign7EndFunction; //Name the end callback function
    ignitionSchedule8.EndCallback = ign8EndFunction; //Name the end callback function


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

//Experimental new generic function. This is NOT yet ready and functional
void setFuelSchedule(struct Schedule *targetSchedule, unsigned long timeout, unsigned long duration)
{
  if(targetSchedule->Status != RUNNING) //Check that we're not already part way through a schedule
  {
    targetSchedule->duration = duration;

    //Need to check that the timeout doesn't exceed the overflow
    uint16_t timeout_timer_compare;
    if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >16x (Each tick represents 16uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
    else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

    //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
    noInterrupts();
    //targetSchedule->startCompare = *targetSchedule->counter + timeout_timer_compare;
    targetSchedule->startCompare = FUEL1_COUNTER + timeout_timer_compare; //Insert correct counter HERE!
    targetSchedule->endCompare = targetSchedule->startCompare + uS_TO_TIMER_COMPARE(duration);
    targetSchedule->Status = PENDING; //Turn this schedule on
    targetSchedule->schedulesSet++; //Increment the number of times this schedule has been set

    //*targetSchedule->compare = targetSchedule->startCompare;
    FUEL1_COMPARE = (uint16_t)targetSchedule->startCompare; //Insert corrector compare HERE!
    interrupts();
    FUEL1_TIMER_ENABLE();
  }
  else
  {
    //If the schedule is already running, we can set the next schedule so it is ready to go
    //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
    //targetSchedule->nextStartCompare = *targetSchedule->counter + uS_TO_TIMER_COMPARE(timeout);
    targetSchedule->nextEndCompare = targetSchedule->nextStartCompare + uS_TO_TIMER_COMPARE(duration);
    targetSchedule->hasNextSchedule = true;
  }
}


//void setFuelSchedule1(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
void setFuelSchedule1(unsigned long timeout, unsigned long duration) //Uses timer 3 compare A
{
  //Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
  //if(timeout < MAX_TIMER_PERIOD)
  {
    if(fuelSchedule1.Status != RUNNING) //Check that we're not already part way through a schedule
    {
      //Need to check that the timeout doesn't exceed the overflow
      if ((timeout+duration) < MAX_TIMER_PERIOD)
      {
        fuelSchedule1.duration = duration;
        //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
        noInterrupts();
        fuelSchedule1.startCompare = FUEL1_COUNTER + uS_TO_TIMER_COMPARE(timeout);
        fuelSchedule1.endCompare = fuelSchedule1.startCompare + uS_TO_TIMER_COMPARE(duration);
        fuelSchedule1.Status = PENDING; //Turn this schedule on
        fuelSchedule1.schedulesSet++; //Increment the number of times this schedule has been set
        //Schedule 1 shares a timer with schedule 5
        //if(channel5InjEnabled) { FUEL1_COMPARE = (uint16_t)setQueue(timer3Aqueue, &fuelSchedule1, &fuelSchedule5, FUEL1_COUNTER); }
        //else { timer3Aqueue[0] = &fuelSchedule1; timer3Aqueue[1] = &fuelSchedule1; timer3Aqueue[2] = &fuelSchedule1; timer3Aqueue[3] = &fuelSchedule1; FUEL1_COMPARE = (uint16_t)fuelSchedule1.startCompare; }
        //timer3Aqueue[0] = &fuelSchedule1; timer3Aqueue[1] = &fuelSchedule1; timer3Aqueue[2] = &fuelSchedule1; timer3Aqueue[3] = &fuelSchedule1;
        FUEL1_COMPARE = (uint16_t)fuelSchedule1.startCompare;
        interrupts();
        FUEL1_TIMER_ENABLE();
      }
    }
    else
    {
      //If the schedule is already running, we can set the next schedule so it is ready to go
      //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
      if( (timeout+duration) < MAX_TIMER_PERIOD )
      {
        noInterrupts();
        fuelSchedule1.nextStartCompare = FUEL1_COUNTER + uS_TO_TIMER_COMPARE(timeout);
        fuelSchedule1.nextEndCompare = fuelSchedule1.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
        fuelSchedule1.duration = duration;
        fuelSchedule1.hasNextSchedule = true;
        interrupts();
      }
    } //Schedule is RUNNING
  } //Timeout less than threshold
}

void setFuelSchedule2(unsigned long timeout, unsigned long duration) //Uses timer 3 compare B
{
  //Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
  if(timeout < MAX_TIMER_PERIOD)
  {
    if(fuelSchedule2.Status != RUNNING) //Check that we're not already part way through a schedule
    {
      //Callbacks no longer used, but retained for now:
      //fuelSchedule2.StartCallback = startCallback;
      //fuelSchedule2.EndCallback = endCallback;
      fuelSchedule2.duration = duration;

      //Need to check that the timeout doesn't exceed the overflow
      uint16_t timeout_timer_compare;
      if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
      else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

      //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
      noInterrupts();
      fuelSchedule2.startCompare = FUEL2_COUNTER + timeout_timer_compare;
      fuelSchedule2.endCompare = fuelSchedule2.startCompare + uS_TO_TIMER_COMPARE(duration);
      FUEL2_COMPARE = (uint16_t)fuelSchedule2.startCompare; //Use the B compare unit of timer 3
      fuelSchedule2.Status = PENDING; //Turn this schedule on
      fuelSchedule2.schedulesSet++; //Increment the number of times this schedule has been set
      interrupts();
      FUEL2_TIMER_ENABLE();
    }
    else
    {
      //If the schedule is already running, we can set the next schedule so it is ready to go
      //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
      fuelSchedule2.nextStartCompare = FUEL2_COUNTER + uS_TO_TIMER_COMPARE(timeout);
      fuelSchedule2.nextEndCompare = fuelSchedule2.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
      fuelSchedule2.hasNextSchedule = true;
    }
  }
}
//void setFuelSchedule3(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
void setFuelSchedule3(unsigned long timeout, unsigned long duration) //Uses timer 3 compare C
{
  //Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
  if(timeout < MAX_TIMER_PERIOD)
  {
    if(fuelSchedule3.Status != RUNNING) //Check that we're not already part way through a schedule
    {
      //Callbacks no longer used, but retained for now:
      //fuelSchedule3.StartCallback = startCallback;
      //fuelSchedule3.EndCallback = endCallback;
      fuelSchedule3.duration = duration;

      //Need to check that the timeout doesn't exceed the overflow
      uint16_t timeout_timer_compare;
      if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
      else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

      //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
      noInterrupts();
      fuelSchedule3.startCompare = FUEL3_COUNTER + timeout_timer_compare;
      fuelSchedule3.endCompare = fuelSchedule3.startCompare + uS_TO_TIMER_COMPARE(duration);
      FUEL3_COMPARE = (uint16_t)fuelSchedule3.startCompare; //Use the C compare unit of timer 3
      fuelSchedule3.Status = PENDING; //Turn this schedule on
      fuelSchedule3.schedulesSet++; //Increment the number of times this schedule has been set
      interrupts();
      FUEL3_TIMER_ENABLE();
    }
    else
    {
      //If the schedule is already running, we can set the next schedule so it is ready to go
      //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
      fuelSchedule3.nextStartCompare = FUEL3_COUNTER + uS_TO_TIMER_COMPARE(timeout);
      fuelSchedule3.nextEndCompare = fuelSchedule3.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
      fuelSchedule3.hasNextSchedule = true;
    }
  }
}
//void setFuelSchedule4(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)())
void setFuelSchedule4(unsigned long timeout, unsigned long duration) //Uses timer 4 compare B
{
  //Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
  if(timeout < MAX_TIMER_PERIOD)
  {
    if(fuelSchedule4.Status != RUNNING) //Check that we're not already part way through a schedule
    {
      //Callbacks no longer used, but retained for now:
      //fuelSchedule4.StartCallback = startCallback;
      //fuelSchedule4.EndCallback = endCallback;
      fuelSchedule4.duration = duration;

      //Need to check that the timeout doesn't exceed the overflow
      uint16_t timeout_timer_compare;
      if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
      else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

      //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
      noInterrupts();
      fuelSchedule4.startCompare = FUEL4_COUNTER + timeout_timer_compare;
      fuelSchedule4.endCompare = fuelSchedule4.startCompare + uS_TO_TIMER_COMPARE(duration);
      FUEL4_COMPARE = (uint16_t)fuelSchedule4.startCompare; //Use the B compare unit of timer 4
      fuelSchedule4.Status = PENDING; //Turn this schedule on
      fuelSchedule4.schedulesSet++; //Increment the number of times this schedule has been set
      interrupts();
      FUEL4_TIMER_ENABLE();
    }
    else
    {
      //If the schedule is already running, we can set the next schedule so it is ready to go
      //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
      fuelSchedule4.nextStartCompare = FUEL4_COUNTER + uS_TO_TIMER_COMPARE(timeout);
      fuelSchedule4.nextEndCompare = fuelSchedule4.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
      fuelSchedule4.hasNextSchedule = true;
    }
  }
}

#if INJ_CHANNELS >= 5
void setFuelSchedule5(unsigned long timeout, unsigned long duration) //Uses timer 4 compare C
{
  //Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
  if(timeout < MAX_TIMER_PERIOD)
  {
    if(fuelSchedule5.Status != RUNNING) //Check that we're not already part way through a schedule
    {
      fuelSchedule5.duration = duration;

      //Need to check that the timeout doesn't exceed the overflow
      uint16_t timeout_timer_compare;
      if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
      else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

      //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
      noInterrupts();
      fuelSchedule5.startCompare = FUEL5_COUNTER + timeout_timer_compare;
      fuelSchedule5.endCompare = fuelSchedule5.startCompare + uS_TO_TIMER_COMPARE(duration);
      FUEL5_COMPARE = (uint16_t)fuelSchedule5.startCompare; //Use the C compare unit of timer 4
      fuelSchedule5.Status = PENDING; //Turn this schedule on
      fuelSchedule5.schedulesSet++; //Increment the number of times this schedule has been set
      interrupts();
      FUEL5_TIMER_ENABLE();
    }
    else
    {
      //If the schedule is already running, we can set the next schedule so it is ready to go
      //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
      fuelSchedule5.nextStartCompare = FUEL5_COUNTER + uS_TO_TIMER_COMPARE(timeout);
      fuelSchedule5.nextEndCompare = fuelSchedule5.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
      fuelSchedule5.hasNextSchedule = true;
    }
  }
}
#endif

#if INJ_CHANNELS >= 6
void setFuelSchedule6(unsigned long timeout, unsigned long duration) //Uses timer 4 compare A
{
  //Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
  if(timeout < MAX_TIMER_PERIOD)
  {
    if(fuelSchedule6.Status != RUNNING) //Check that we're not already part way through a schedule
    {
      fuelSchedule6.duration = duration;

      //Need to check that the timeout doesn't exceed the overflow
      uint16_t timeout_timer_compare;
      if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
      else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

      //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
      noInterrupts();
      fuelSchedule6.startCompare = FUEL6_COUNTER + timeout_timer_compare;
      fuelSchedule6.endCompare = fuelSchedule6.startCompare + uS_TO_TIMER_COMPARE(duration);
      FUEL6_COMPARE = (uint16_t)fuelSchedule6.startCompare; //Use the A compare unit of timer 4
      fuelSchedule6.Status = PENDING; //Turn this schedule on
      fuelSchedule6.schedulesSet++; //Increment the number of times this schedule has been set
      interrupts();
      FUEL6_TIMER_ENABLE();
    }
    else
    {
      //If the schedule is already running, we can set the next schedule so it is ready to go
      //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
      fuelSchedule6.nextStartCompare = FUEL6_COUNTER + uS_TO_TIMER_COMPARE(timeout);
      fuelSchedule6.nextEndCompare = fuelSchedule6.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
      fuelSchedule6.hasNextSchedule = true;
    }
  }
}
#endif

#if INJ_CHANNELS >= 7
void setFuelSchedule7(unsigned long timeout, unsigned long duration) //Uses timer 5 compare C
{
  //Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
  if(timeout < MAX_TIMER_PERIOD)
  {
    if(fuelSchedule7.Status != RUNNING) //Check that we're not already part way through a schedule
    {
      fuelSchedule7.duration = duration;

      //Need to check that the timeout doesn't exceed the overflow
      uint16_t timeout_timer_compare;
      if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
      else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

      //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
      noInterrupts();
      fuelSchedule7.startCompare = FUEL7_COUNTER + timeout_timer_compare;
      fuelSchedule7.endCompare = fuelSchedule7.startCompare + uS_TO_TIMER_COMPARE(duration);
      FUEL7_COMPARE = (uint16_t)fuelSchedule7.startCompare; //Use the C compare unit of timer 5
      fuelSchedule7.Status = PENDING; //Turn this schedule on
      fuelSchedule7.schedulesSet++; //Increment the number of times this schedule has been set
      interrupts();
      FUEL7_TIMER_ENABLE();
    }
    else
    {
      //If the schedule is already running, we can set the next schedule so it is ready to go
      //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
      fuelSchedule7.nextStartCompare = FUEL7_COUNTER + uS_TO_TIMER_COMPARE(timeout);
      fuelSchedule7.nextEndCompare = fuelSchedule7.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
      fuelSchedule7.hasNextSchedule = true;
    }
  }
}
#endif

#if INJ_CHANNELS >= 8
void setFuelSchedule8(unsigned long timeout, unsigned long duration) //Uses timer 5 compare B
{
  //Check whether timeout exceeds the maximum future time. This can potentially occur on sequential setups when below ~115rpm
  if(timeout < MAX_TIMER_PERIOD)
  {
    if(fuelSchedule8.Status != RUNNING) //Check that we're not already part way through a schedule
    {
      fuelSchedule8.duration = duration;

      //Need to check that the timeout doesn't exceed the overflow
      uint16_t timeout_timer_compare;
      if (timeout > MAX_TIMER_PERIOD) { timeout_timer_compare = uS_TO_TIMER_COMPARE( (MAX_TIMER_PERIOD - 1) ); } // If the timeout is >4x (Each tick represents 4uS) the maximum allowed value of unsigned int (65535), the timer compare value will overflow when appliedcausing erratic behaviour such as erroneous sparking.
      else { timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout); } //Normal case

      //The following must be enclosed in the noInterupts block to avoid contention caused if the relevant interrupt fires before the state is fully set
      noInterrupts();
      fuelSchedule8.startCompare = FUEL8_COUNTER + timeout_timer_compare;
      fuelSchedule8.endCompare = fuelSchedule8.startCompare + uS_TO_TIMER_COMPARE(duration);
      FUEL8_COMPARE = (uint16_t)fuelSchedule8.startCompare; //Use the B compare unit of timer 5
      fuelSchedule8.Status = PENDING; //Turn this schedule on
      fuelSchedule8.schedulesSet++; //Increment the number of times this schedule has been set
      interrupts();
      FUEL8_TIMER_ENABLE();
    }
    else
    {
      //If the schedule is already running, we can set the next schedule so it is ready to go
      //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
      fuelSchedule8.nextStartCompare = FUEL8_COUNTER + uS_TO_TIMER_COMPARE(timeout);
      fuelSchedule8.nextEndCompare = fuelSchedule8.nextStartCompare + uS_TO_TIMER_COMPARE(duration);
      fuelSchedule8.hasNextSchedule = true;
    }
  }
}
#endif

//ignition timer enable functions
void ign1TimerEnable(){IGN1_TIMER_ENABLE();}
void ign2TimerEnable(){IGN2_TIMER_ENABLE();}
void ign3TimerEnable(){IGN3_TIMER_ENABLE();}
void ign4TimerEnable(){IGN4_TIMER_ENABLE();}
void ign5TimerEnable(){IGN5_TIMER_ENABLE();}
void ign6TimerEnable(){IGN6_TIMER_ENABLE();}
void ign7TimerEnable(){IGN7_TIMER_ENABLE();}
void ign8TimerEnable(){IGN8_TIMER_ENABLE();}

uint32_t getIgn1Counter() {return IGN1_COUNTER;}
uint32_t getIgn2Counter() {return IGN2_COUNTER;}
uint32_t getIgn3Counter() {return IGN3_COUNTER;}
uint32_t getIgn4Counter() {return IGN4_COUNTER;}
uint32_t getIgn5Counter() {return IGN5_COUNTER;}
uint32_t getIgn6Counter() {return IGN6_COUNTER;}
uint32_t getIgn7Counter() {return IGN7_COUNTER;}
uint32_t getIgn8Counter() {return IGN8_COUNTER;}

void setIgnition1Compare(COMPARE_TYPE compareValue){IGN1_COMPARE =(uint16_t)compareValue;}
void setIgnition2Compare(COMPARE_TYPE compareValue){IGN2_COMPARE =(uint16_t)compareValue;}
void setIgnition3Compare(COMPARE_TYPE compareValue){IGN3_COMPARE =(uint16_t)compareValue;}
void setIgnition4Compare(COMPARE_TYPE compareValue){IGN4_COMPARE =(uint16_t)compareValue;}
void setIgnition5Compare(COMPARE_TYPE compareValue){IGN5_COMPARE =(uint16_t)compareValue;}
void setIgnition6Compare(COMPARE_TYPE compareValue){IGN6_COMPARE =(uint16_t)compareValue;}
void setIgnition7Compare(COMPARE_TYPE compareValue){IGN7_COMPARE =(uint16_t)compareValue;}
void setIgnition8Compare(COMPARE_TYPE compareValue){IGN8_COMPARE =(uint16_t)compareValue;}

//Ignition schedulers use Timer 5
void setIgnitionSchedule(struct Schedule *ignitionSchedule ,  int16_t crankAngle, int channelIgnDegrees, int ignitionEndAngle, unsigned long duration)
{  
  unsigned long timeout;
  uint16_t timeout_timer_compare; 
  int16_t tempCrankAngle;
  int16_t tempEndAngle;     

  tempCrankAngle = crankAngle - channelIgnDegrees;
  if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_IGN; }
  tempEndAngle = ignitionEndAngle - channelIgnDegrees;
  if ( tempEndAngle < 0) { tempEndAngle += CRANK_ANGLE_MAX_IGN; }
  while (tempEndAngle <= tempCrankAngle)   { tempEndAngle += CRANK_ANGLE_MAX_IGN; }//calculate into the next cycle
  
  if(ignitionSchedule->Status == PENDING || ignitionSchedule->Status == OFF) //Check that we're not already part way through a schedule
  {
    timeout= angleToTime((tempEndAngle - tempCrankAngle), CRANKMATH_METHOD_INTERVAL_REV);
   
    if (timeout > MAX_TIMER_PERIOD) {
      ignitionSchedule->Status = OFF; //Off for now, come back later...
     } 
    else if(timeout > duration + IGNITION_REFRESH_THRESHOLD+200) { //refresh or start schedule safely (extra 200us is just a reasonable time for the code to run ) 
      timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout);  //Normal case

      noInterrupts(); // make sure start and end values are updated simultaneously
      ignitionSchedule->endCompare = ignitionSchedule->getIgnCounter() + timeout_timer_compare; //As there is a tick every 4uS, there are timeout/4 ticks until the interrupt should be triggered ( >>2 divides by 4)
      ignitionSchedule->startCompare = ignitionSchedule->endCompare - (uint16_t)uS_TO_TIMER_COMPARE(duration);   
      ignitionSchedule->setIgnitionCompare(ignitionSchedule->startCompare - uS_TO_TIMER_COMPARE((IGNITION_REFRESH_THRESHOLD+60)));//set up time for staging (actual impulse starting and timing is done totally in interrupts)
      ignitionSchedule->Status = PENDING; //Turn this schedule on
      interrupts();      
     ignitionSchedule->schedulesSet++;
     ignitionSchedule->duration = duration;
     ignitionSchedule->ignTimerEnable(); //just in case. Actually can be omitted because timers are always on.
    }
  }
  else
  {
    //If the schedule is already running, we can set the next schedule so it is ready to go
    //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
    timeout= angleToTime((tempEndAngle - tempCrankAngle), CRANKMATH_METHOD_INTERVAL_REV);  
    if (timeout < MAX_TIMER_PERIOD)
    {
      ignitionSchedule->nextEndCompare = ignitionSchedule->getIgnCounter() + uS_TO_TIMER_COMPARE(timeout);
      ignitionSchedule->nextStartCompare = ignitionSchedule->nextEndCompare - uS_TO_TIMER_COMPARE(duration);
      ignitionSchedule->hasNextSchedule = true;
    }

  }
}

extern void beginInjectorPriming()
{
  unsigned long primingValue = table2D_getValue(&PrimingPulseTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
  if( (primingValue > 0) && (currentStatus.TPS < configPage4.floodClear) )
  {
    primingValue = primingValue * 100 * 5; //to acheive long enough priming pulses, the values in tuner studio are divided by 0.5 instead of 0.1, so multiplier of 5 is required.
    if ( channel1InjEnabled == true ) { setFuelSchedule1(100, primingValue); }
#if (INJ_CHANNELS >= 2)
    if ( channel2InjEnabled == true ) { setFuelSchedule2(100, primingValue); }
#endif
#if (INJ_CHANNELS >= 3)
    if ( channel3InjEnabled == true ) { setFuelSchedule3(100, primingValue); }
#endif
#if (INJ_CHANNELS >= 4)
    if ( channel4InjEnabled == true ) { setFuelSchedule4(100, primingValue); }
#endif
#if (INJ_CHANNELS >= 5)
    if ( channel5InjEnabled == true ) { setFuelSchedule5(100, primingValue); }
#endif
#if (INJ_CHANNELS >= 6)
    if ( channel6InjEnabled == true ) { setFuelSchedule6(100, primingValue); }
#endif
#if (INJ_CHANNELS >= 7)
    if ( channel7InjEnabled == true) { setFuelSchedule7(100, primingValue); }
#endif
#if (INJ_CHANNELS >= 8)
    if ( channel8InjEnabled == true ) { setFuelSchedule8(100, primingValue); }
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
//Timer3A (fuel schedule 1) Compare Vector
#if (INJ_CHANNELS >= 1)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER3_COMPA_vect) //fuelSchedules 1 and 5
#else
static inline void fuelSchedule1Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if (fuelSchedule1.Status == PENDING) //Check to see if this schedule is turn on
    {
      //To use timer queue, change fuelShedule1 to timer3Aqueue[0];
      inj1StartFunction();
      fuelSchedule1.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      FUEL1_COMPARE = (uint16_t)(FUEL1_COUNTER + uS_TO_TIMER_COMPARE(fuelSchedule1.duration)); //Doing this here prevents a potential overflow on restarts
    }
    else if (fuelSchedule1.Status == RUNNING)
    {
       //timer3Aqueue[0]->EndCallback();
       inj1EndFunction();
       fuelSchedule1.Status = OFF; //Turn off the schedule
       fuelSchedule1.schedulesSet = 0;
       //FUEL1_COMPARE = (uint16_t)fuelSchedule1.endCompare;

       //If there is a next schedule queued up, activate it
       if(fuelSchedule1.hasNextSchedule == true)
       {
         FUEL1_COMPARE = (uint16_t)fuelSchedule1.nextStartCompare;
         fuelSchedule1.endCompare = fuelSchedule1.nextEndCompare;
         fuelSchedule1.Status = PENDING;
         fuelSchedule1.schedulesSet = 1;
         fuelSchedule1.hasNextSchedule = false;
       }
       else { FUEL1_TIMER_DISABLE(); }
    }
    else if (fuelSchedule1.Status == OFF) { FUEL1_TIMER_DISABLE(); } //Safety check. Turn off this output compare unit and return without performing any action
  }
#endif

#if (INJ_CHANNELS >= 2)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER3_COMPB_vect) //fuelSchedule2
#else
static inline void fuelSchedule2Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if (fuelSchedule2.Status == PENDING) //Check to see if this schedule is turn on
    {
      //fuelSchedule2.StartCallback();
      inj2StartFunction();
      fuelSchedule2.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      FUEL2_COMPARE = (uint16_t)(FUEL2_COUNTER + uS_TO_TIMER_COMPARE(fuelSchedule2.duration)); //Doing this here prevents a potential overflow on restarts
    }
    else if (fuelSchedule2.Status == RUNNING)
    {
       //fuelSchedule2.EndCallback();
       inj2EndFunction();
       fuelSchedule2.Status = OFF; //Turn off the schedule
       fuelSchedule2.schedulesSet = 0;

       //If there is a next schedule queued up, activate it
       if(fuelSchedule2.hasNextSchedule == true)
       {
         FUEL2_COMPARE = (uint16_t)fuelSchedule2.nextStartCompare;
         fuelSchedule2.endCompare = fuelSchedule2.nextEndCompare;
         fuelSchedule2.Status = PENDING;
         fuelSchedule2.schedulesSet = 1;
         fuelSchedule2.hasNextSchedule = false;
       }
       else { FUEL2_TIMER_DISABLE(); }
    }
  }
#endif

#if (INJ_CHANNELS >= 3)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER3_COMPC_vect) //fuelSchedule3
#else
static inline void fuelSchedule3Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if (fuelSchedule3.Status == PENDING) //Check to see if this schedule is turn on
    {
      //fuelSchedule3.StartCallback();
      inj3StartFunction();
      fuelSchedule3.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      FUEL3_COMPARE = (uint16_t)(FUEL3_COUNTER + uS_TO_TIMER_COMPARE(fuelSchedule3.duration)); //Doing this here prevents a potential overflow on restarts
    }
    else if (fuelSchedule3.Status == RUNNING)
    {
       //fuelSchedule3.EndCallback();
       inj3EndFunction();
       fuelSchedule3.Status = OFF; //Turn off the schedule
       fuelSchedule3.schedulesSet = 0;

       //If there is a next schedule queued up, activate it
       if(fuelSchedule3.hasNextSchedule == true)
       {
         FUEL3_COMPARE = (uint16_t)fuelSchedule3.nextStartCompare;
         fuelSchedule3.endCompare = fuelSchedule3.nextEndCompare;
         fuelSchedule3.Status = PENDING;
         fuelSchedule3.schedulesSet = 1;
         fuelSchedule3.hasNextSchedule = false;
       }
       else { FUEL3_TIMER_DISABLE(); }
    }
  }
#endif

#if (INJ_CHANNELS >= 4)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER4_COMPB_vect) //fuelSchedule4
#else
static inline void fuelSchedule4Interrupt() //Most ARM chips can simply call a function
#endif
  {
    if (fuelSchedule4.Status == PENDING) //Check to see if this schedule is turn on
    {
      //fuelSchedule4.StartCallback();
      inj4StartFunction();
      fuelSchedule4.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      FUEL4_COMPARE = (uint16_t)(FUEL4_COUNTER + uS_TO_TIMER_COMPARE(fuelSchedule4.duration)); //Doing this here prevents a potential overflow on restarts
    }
    else if (fuelSchedule4.Status == RUNNING)
    {
       //fuelSchedule4.EndCallback();
       inj4EndFunction();
       fuelSchedule4.Status = OFF; //Turn off the schedule
       fuelSchedule4.schedulesSet = 0;

       //If there is a next schedule queued up, activate it
       if(fuelSchedule4.hasNextSchedule == true)
       {
         FUEL4_COMPARE = (uint16_t)fuelSchedule4.nextStartCompare;
         fuelSchedule4.endCompare = fuelSchedule4.nextEndCompare;
         fuelSchedule4.Status = PENDING;
         fuelSchedule4.schedulesSet = 1;
         fuelSchedule4.hasNextSchedule = false;
       }
       else { FUEL4_TIMER_DISABLE(); }
    }
  }
#endif

#if (INJ_CHANNELS >= 5)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPC_vect) //fuelSchedule5
#else
static inline void fuelSchedule5Interrupt() //Most ARM chips can simply call a function
#endif
{
  if (fuelSchedule5.Status == PENDING) //Check to see if this schedule is turn on
  {
    inj5StartFunction();
    fuelSchedule5.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
    FUEL5_COMPARE = (uint16_t)(FUEL5_COUNTER + uS_TO_TIMER_COMPARE(fuelSchedule5.duration)); //Doing this here prevents a potential overflow on restarts
  }
  else if (fuelSchedule5.Status == RUNNING)
  {
     inj5EndFunction();
     fuelSchedule5.Status = OFF; //Turn off the schedule
     fuelSchedule5.schedulesSet = 0;

     //If there is a next schedule queued up, activate it
     if(fuelSchedule5.hasNextSchedule == true)
     {
       FUEL5_COMPARE = (uint16_t)fuelSchedule5.nextStartCompare;
       fuelSchedule5.endCompare = fuelSchedule5.nextEndCompare;
       fuelSchedule5.Status = PENDING;
       fuelSchedule5.schedulesSet = 1;
       fuelSchedule5.hasNextSchedule = false;
     }
     else { FUEL5_TIMER_DISABLE(); }
  }
}
#endif

#if (INJ_CHANNELS >= 6)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPA_vect) //fuelSchedule6
#else
static inline void fuelSchedule6Interrupt() //Most ARM chips can simply call a function
#endif
{
  if (fuelSchedule6.Status == PENDING) //Check to see if this schedule is turn on
  {
    //fuelSchedule6.StartCallback();
    inj6StartFunction();
    fuelSchedule6.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
    FUEL6_COMPARE = (uint16_t)(FUEL6_COUNTER + uS_TO_TIMER_COMPARE(fuelSchedule6.duration)); //Doing this here prevents a potential overflow on restarts
  }
  else if (fuelSchedule6.Status == RUNNING)
  {
     //fuelSchedule6.EndCallback();
     inj6EndFunction();
     fuelSchedule6.Status = OFF; //Turn off the schedule
     fuelSchedule6.schedulesSet = 0;

     //If there is a next schedule queued up, activate it
     if(fuelSchedule6.hasNextSchedule == true)
     {
       FUEL6_COMPARE = (uint16_t)fuelSchedule6.nextStartCompare;
       fuelSchedule6.endCompare = fuelSchedule6.nextEndCompare;
       fuelSchedule6.Status = PENDING;
       fuelSchedule6.schedulesSet = 1;
       fuelSchedule6.hasNextSchedule = false;
     }
     else { FUEL6_TIMER_DISABLE(); }
  }
}
#endif

#if (INJ_CHANNELS >= 7)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPC_vect) //fuelSchedule7
#else
static inline void fuelSchedule7Interrupt() //Most ARM chips can simply call a function
#endif
{
  if (fuelSchedule7.Status == PENDING) //Check to see if this schedule is turn on
  {
    //fuelSchedule7.StartCallback();
    inj7StartFunction();
    fuelSchedule7.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
    FUEL7_COMPARE = (uint16_t)(FUEL7_COUNTER + uS_TO_TIMER_COMPARE(fuelSchedule7.duration)); //Doing this here prevents a potential overflow on restarts
  }
  else if (fuelSchedule7.Status == RUNNING)
  {
     //fuelSchedule7.EndCallback();
     inj7EndFunction();
     fuelSchedule7.Status = OFF; //Turn off the schedule
     fuelSchedule7.schedulesSet = 0;

     //If there is a next schedule queued up, activate it
     if(fuelSchedule7.hasNextSchedule == true)
     {
       FUEL7_COMPARE = (uint16_t)fuelSchedule7.nextStartCompare;
       fuelSchedule7.endCompare = fuelSchedule7.nextEndCompare;
       fuelSchedule7.Status = PENDING;
       fuelSchedule7.schedulesSet = 1;
       fuelSchedule7.hasNextSchedule = false;
     }
     else { FUEL7_TIMER_DISABLE(); }
  }
}
#endif

#if (INJ_CHANNELS >= 8)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPB_vect) //fuelSchedule8
#else
static inline void fuelSchedule8Interrupt() //Most ARM chips can simply call a function
#endif
{
  if (fuelSchedule8.Status == PENDING) //Check to see if this schedule is turn on
  {
    //fuelSchedule8.StartCallback();
    inj8StartFunction();
    fuelSchedule8.Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
    FUEL8_COMPARE = (uint16_t)(FUEL8_COUNTER + uS_TO_TIMER_COMPARE(fuelSchedule8.duration)); //Doing this here prevents a potential overflow on restarts
  }
  else if (fuelSchedule8.Status == RUNNING)
  {
     //fuelSchedule8.EndCallback();
     inj8EndFunction();
     fuelSchedule8.Status = OFF; //Turn off the schedule
     fuelSchedule8.schedulesSet = 0;

     //If there is a next schedule queued up, activate it
     if(fuelSchedule8.hasNextSchedule == true)
     {
       FUEL8_COMPARE = (uint16_t)fuelSchedule8.nextStartCompare;
       fuelSchedule8.endCompare = fuelSchedule8.nextEndCompare;
       fuelSchedule8.Status = PENDING;
       fuelSchedule8.schedulesSet = 1;
       fuelSchedule8.hasNextSchedule = false;
     }
     else { FUEL8_TIMER_DISABLE(); }
  }
}
#endif

#if IGN_CHANNELS >= 1
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPA_vect) //ignitionSchedule1
#else
static inline void ignitionSchedule1Interrupt() //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule1);
}
#endif

#if IGN_CHANNELS >= 2
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPB_vect) //ignitionSchedule2
#else
static inline void ignitionSchedule2Interrupt() //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule2);
}
#endif

#if IGN_CHANNELS >= 3
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPC_vect) //ignitionSchedule3
#else
static inline void ignitionSchedule3Interrupt() //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule3);
}
#endif

#if IGN_CHANNELS >= 4
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPA_vect) //ignitionSchedule4
#else
static inline void ignitionSchedule4Interrupt() //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule4);
}
#endif

#if IGN_CHANNELS >= 5
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPC_vect) //ignitionSchedule5
#else
static inline void ignitionSchedule5Interrupt() //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule5);
}
#endif

#if IGN_CHANNELS >= 6
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPB_vect) //ignitionSchedule6
#else
static inline void ignitionSchedule6Interrupt() //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule6);
}
#endif

#if IGN_CHANNELS >= 7
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER3_COMPC_vect) //ignitionSchedule6
#else
static inline void ignitionSchedule7Interrupt() //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule7);
}
#endif

#if IGN_CHANNELS >= 8
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER3_COMPB_vect) //ignitionSchedule8
#else
static inline void ignitionSchedule8Interrupt() //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule8);
}
#endif

void ignitionScheduleInterrupt(struct Schedule *ignitionSchedule) // common function that all ignition channel interrupts use
{
  { if (ignitionSchedule->Status == PENDING){ //Check to see if this schedule is ready to be locked for action
    ignitionSchedule->Status = STAGED;
    ignitionSchedule->setIgnitionCompare(ignitionSchedule->startCompare);//ignition pulse start timing now locked.     
    }
    else if (ignitionSchedule->Status == STAGED) //Check to see if this schedule is ready to turn on
    {
      ignitionSchedule->StartCallback();
      ignitionSchedule->Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      ignitionSchedule->startTime = micros();
      ignitionSchedule->setIgnitionCompare(ignitionSchedule->endCompare); // Set end callback interrupt time
    }
    else if (ignitionSchedule->Status == RUNNING) //Check to see if its time for spark
    {
      ignitionSchedule->EndCallback(); //Moment of spark      
      ignitionSchedule->schedulesSet = 0;
      ignitionSchedule->endScheduleSetByDecoder = false;
      ignitionCount += 1; //Increment the igintion counter
      ignitionSchedule->Status = OFF; //Spark done: turn off the schedule
      //If there is a next schedule queued up, activate it
      if(ignitionSchedule1.hasNextSchedule == true)
      {
        ignitionSchedule->setIgnitionCompare(ignitionSchedule->nextStartCompare - uS_TO_TIMER_COMPARE(IGNITION_REFRESH_THRESHOLD)); //this gets them values ready for staging
        ignitionSchedule->endCompare=ignitionSchedule->nextEndCompare;  //load in values from queue
        ignitionSchedule->startCompare=ignitionSchedule->nextStartCompare;
        ignitionSchedule->Status = PENDING;
        ignitionSchedule->schedulesSet = 1;
        ignitionSchedule->hasNextSchedule = false;
      }

    }
    else if (ignitionSchedule->Status == OFF)
    {      //Catch any spurious interrupts. This really shouldn't ever be called, but there as a safety
      
    }
  }
}
