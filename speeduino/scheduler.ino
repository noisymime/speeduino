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

Fuel1 fuelSchedule1;
Fuel2 fuelSchedule2;
Fuel3 fuelSchedule3;
Fuel4 fuelSchedule4;
#if (INJ_CHANNELS >= 5)
Fuel5 fuelSchedule5;
#endif
#if (INJ_CHANNELS >= 6)
Fuel6 fuelSchedule6;
#endif
#if (INJ_CHANNELS >= 7)
Fuel7 fuelSchedule7;
#endif
#if (INJ_CHANNELS >= 8)
Fuel8 fuelSchedule8;
#endif

Ign1 ignitionSchedule1;
Ign2 ignitionSchedule2;
Ign3 ignitionSchedule3;
Ign4 ignitionSchedule4;
Ign5 ignitionSchedule5;
#if IGN_CHANNELS >= 6
Ign6 ignitionSchedule6;
#endif
#if IGN_CHANNELS >= 7
Ign7 ignitionSchedule7;
#endif
#if IGN_CHANNELS >= 8
Ign8 ignitionSchedule8;
#endif

void initialiseSchedulers(void)
{
    //nullSchedule.Status = OFF;    
    fuelSchedule1.Status = OFF;    
    fuelSchedule2.Status = OFF;    
    fuelSchedule3.Status = OFF;
    fuelSchedule4.Status = OFF;  
    #if (INJ_CHANNELS >= 5)
    fuelSchedule5.Status = OFF;
    #endif 
    #if (INJ_CHANNELS >= 6)
    fuelSchedule6.Status = OFF;
    #endif
    #if (INJ_CHANNELS >= 7)
    fuelSchedule7.Status = OFF;
    #endif
    #if (INJ_CHANNELS >= 8)
    fuelSchedule8.Status = OFF;
    #endif      
    
    ignitionSchedule1.Status = OFF;  
    ignitionSchedule2.Status = OFF; 
    ignitionSchedule3.Status = OFF; 
    ignitionSchedule4.Status = OFF;
    ignitionSchedule5.Status = OFF;
    #if IGN_CHANNELS >= 6
    ignitionSchedule6.Status = OFF; 
    #endif
    #if IGN_CHANNELS >= 7
    ignitionSchedule7.Status = OFF;
    #endif
    #if IGN_CHANNELS >= 8
    ignitionSchedule8.Status = OFF; 
    #endif   

}

/*
New generic function.
*/
void setFuelSchedule (struct Schedule *targetSchedule, int16_t crankAngle, int16_t injectorEndAngle, unsigned long duration)
{
  unsigned long endTimeout;

  while (injectorEndAngle <= crankAngle)   
    { 
      injectorEndAngle += CRANK_ANGLE_MAX_INJ; //calculate into the next cycle
    } 
  endTimeout=(injectorEndAngle - crankAngle) * (unsigned long)timePerDegree;
  
  if(targetSchedule->Status != RUNNING) //Check that we're not already part way through a schedule
  {
    if((endTimeout < MAX_TIMER_PERIOD) && (endTimeout > duration + INJECTION_REFRESH_TRESHOLD)) //Need to check that the timeout doesn't exceed the overflow, also allow for fixed safety between setting the schedule and running it
    {      
      noInterrupts(); // make sure start and end values are updated simultaneously
      targetSchedule->endCompare = targetSchedule->getCounter() + (COMPARE_TYPE)(uS_TO_TIMER_COMPARE(endTimeout)); //As there is a tick every 4uS, there are timeout/4 ticks until the interrupt should be triggered ( >>2 divides by 4)   
      targetSchedule->setCompare(targetSchedule->endCompare - (COMPARE_TYPE)(uS_TO_TIMER_COMPARE(duration))); // set pulse start Compare value
      targetSchedule->Status = PENDING; //Turn this schedule on
      interrupts(); 
      targetSchedule->timerEnable();
    }
  }
  else 
  {
    //If the schedule is already running, we can set the next schedule so it is ready to go
    //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
    injectorEndAngle += CRANK_ANGLE_MAX_INJ;
    endTimeout=(injectorEndAngle - crankAngle) * (unsigned long)timePerDegree;
      if((endTimeout < MAX_TIMER_PERIOD) && (endTimeout > duration + INJECTION_REFRESH_TRESHOLD)&&((COMPARE_TYPE)(targetSchedule->endCompare-targetSchedule->getCounter())>uS_TO_TIMER_COMPARE(INJECTION_REFRESH_TRESHOLD)))
      {
      noInterrupts();
      targetSchedule->nextEndCompare = targetSchedule->getCounter() + (COMPARE_TYPE)(uS_TO_TIMER_COMPARE(endTimeout));
      targetSchedule->nextStartCompare = targetSchedule->nextEndCompare - (COMPARE_TYPE)(uS_TO_TIMER_COMPARE(duration));      
      targetSchedule->hasNextSchedule = true;
      interrupts();
      }
  }
}

//separate function for setting the fuel schedules at priming
void setFuelSchedule (struct Schedule *targetSchedule, unsigned long duration)
{
  if(targetSchedule->Status != RUNNING) //Check that we're not already part way through a schedule
  {      
      targetSchedule->StartFunction();
      noInterrupts(); // make sure start and end values are updated simultaneously
      targetSchedule->setCompare(targetSchedule->getCounter() + (COMPARE_TYPE)uS_TO_TIMER_COMPARE(duration));
      targetSchedule->Status = RUNNING; //RUN this schedule immediately
      targetSchedule->hasNextSchedule = false;
      interrupts(); 
      targetSchedule->timerEnable();
  }
}

//New generic function
void setIgnitionSchedule(struct Schedule *targetSchedule ,  int16_t crankAngle, int ignitionEndAngle, unsigned long duration)
{
  unsigned long endTimeout;

  while (ignitionEndAngle <= crankAngle)   { ignitionEndAngle += CRANK_ANGLE_MAX_IGN; } //calculate into the next cycle
  //endTimeout=(ignitionEndAngle - crankAngle) * (unsigned long)timePerDegree;
  endTimeout= angleToTime((ignitionEndAngle - crankAngle), CRANKMATH_METHOD_INTERVAL_REV);
  
  if (targetSchedule->Status != RUNNING) //Check that we're not already part way through a schedule
  {
    if((endTimeout < MAX_TIMER_PERIOD) && (endTimeout > duration + IGNITION_REFRESH_THRESHOLD)) //Need to check that the timeout doesn't exceed the overflow, also allow for fixed 230us safety between setting the schedule and running it
    {      
      noInterrupts(); // make sure start and end values are updated simultaneously
      targetSchedule->endCompare = targetSchedule->getCounter() + (COMPARE_TYPE)(uS_TO_TIMER_COMPARE(endTimeout)); //As there is a tick every 4uS, there are timeout/4 ticks until the interrupt should be triggered ( >>2 divides by 4)   
      targetSchedule->setCompare(targetSchedule->endCompare - (COMPARE_TYPE)(uS_TO_TIMER_COMPARE(duration))); // previously startCompare
      targetSchedule->Status = PENDING; //Turn this schedule on
      interrupts(); 
      targetSchedule->timerEnable();
    }
  }
  else 
  {
    //If the schedule is already running, we can set the next schedule so it is ready to go
    //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
    ignitionEndAngle += CRANK_ANGLE_MAX_IGN;
    //endTimeout=(ignitionEndAngle - crankAngle) * (unsigned long)timePerDegree;
    endTimeout= angleToTime((ignitionEndAngle - crankAngle), CRANKMATH_METHOD_INTERVAL_REV);
      if(endTimeout < MAX_TIMER_PERIOD)
      {
      noInterrupts();
      targetSchedule->nextEndCompare = targetSchedule->getCounter() + (COMPARE_TYPE)(uS_TO_TIMER_COMPARE(endTimeout));
      targetSchedule->nextStartCompare = targetSchedule->nextEndCompare - (COMPARE_TYPE)(uS_TO_TIMER_COMPARE(duration));      
      targetSchedule->hasNextSchedule = true;
      interrupts();
      }
  }
}

//overload function for starting schedule(dwell) immediately, this is used in the fixed cranking ignition
void setIgnitionSchedule(struct Schedule *ignitionSchedule)
{            
             ignitionSchedule->StartFunction(); //start coil charging
             ignitionSchedule->setCompare(ignitionSchedule->getCounter()+ (COMPARE_TYPE)(uS_TO_TIMER_COMPARE(currentStatus.dwell)));
             ignitionSchedule->Status=RUNNING;
             ignitionSchedule->timerEnable();
}

extern void beginInjectorPriming(void)
{
  unsigned long primingValue = table2D_getValue(&PrimingPulseTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
  if( (primingValue > 0) && (currentStatus.TPS < configPage4.floodClear) )
  {
    primingValue = primingValue * 100 * 5; //to acheive long enough priming pulses, the values in tuner studio are divided by 0.5 instead of 0.1, so multiplier of 5 is required.
    if ( channel1InjEnabled == true ) { setFuelSchedule(&fuelSchedule1, primingValue); }
#if (INJ_CHANNELS >= 2)
    if ( channel2InjEnabled == true ) { setFuelSchedule(&fuelSchedule2, primingValue); }
#endif
#if (INJ_CHANNELS >= 3)
    if ( channel3InjEnabled == true ) { setFuelSchedule(&fuelSchedule3, primingValue); }
#endif
#if (INJ_CHANNELS >= 4)
    if ( channel4InjEnabled == true ) { setFuelSchedule(&fuelSchedule4, primingValue); }
#endif
#if (INJ_CHANNELS >= 5)
    if ( channel5InjEnabled == true ) { setFuelSchedule(&fuelSchedule5, primingValue); }
#endif
#if (INJ_CHANNELS >= 6)
    if ( channel6InjEnabled == true ) { setFuelSchedule(&fuelSchedule6, primingValue); }
#endif
#if (INJ_CHANNELS >= 7)
    if ( channel7InjEnabled == true) { setFuelSchedule(&fuelSchedule7, primingValue); }
#endif
#if (INJ_CHANNELS >= 8)
    if ( channel8InjEnabled == true ) { setFuelSchedule(&fuelSchedule8, primingValue); }
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
static inline void fuelSchedule1Interrupt(void) //Most ARM chips can simply call a function
#endif
{
fuelScheduleInterrupt(&fuelSchedule1);
}
#endif

#if (INJ_CHANNELS >= 2)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER3_COMPB_vect) //fuelSchedule2
#else
static inline void fuelSchedule2Interrupt(void) //Most ARM chips can simply call a function
#endif
{
fuelScheduleInterrupt(&fuelSchedule2);
}
#endif

#if (INJ_CHANNELS >= 3)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER3_COMPC_vect) //fuelSchedule3
#else
static inline void fuelSchedule3Interrupt(void) //Most ARM chips can simply call a function
#endif
{
fuelScheduleInterrupt(&fuelSchedule3);
}
#endif

#if (INJ_CHANNELS >= 4)
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this
ISR(TIMER4_COMPB_vect) //fuelSchedule4
#else
static inline void fuelSchedule4Interrupt(void) //Most ARM chips can simply call a function
#endif
{
fuelScheduleInterrupt(&fuelSchedule4);
}
#endif

#if (INJ_CHANNELS >= 5)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPC_vect) //fuelSchedule5
#else
static inline void fuelSchedule5Interrupt(void) //Most ARM chips can simply call a function
#endif
{
fuelScheduleInterrupt(&fuelSchedule5);
}
#endif

#if (INJ_CHANNELS >= 6)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPA_vect) //fuelSchedule6
#else
static inline void fuelSchedule6Interrupt(void) //Most ARM chips can simply call a function
#endif
{
fuelScheduleInterrupt(&fuelSchedule6);
}
#endif

#if (INJ_CHANNELS >= 7)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPC_vect) //fuelSchedule7
#else
static inline void fuelSchedule7Interrupt(void) //Most ARM chips can simply call a function
#endif
{
fuelScheduleInterrupt(&fuelSchedule7);
}
#endif

#if (INJ_CHANNELS >= 8)
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPB_vect) //fuelSchedule8
#else
static inline void fuelSchedule8Interrupt(void) //Most ARM chips can simply call a function
#endif
{
fuelScheduleInterrupt(&fuelSchedule8);
}
#endif

void fuelScheduleInterrupt(struct Schedule *fuelSchedule)
{
  if (fuelSchedule->Status == PENDING) //Check to see if this schedule is turn on
  {
    fuelSchedule->setCompare(fuelSchedule->endCompare);
    fuelSchedule->StartFunction();
    fuelSchedule->Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)    
  }
  else if (fuelSchedule->Status == RUNNING)
  {
    //If there is a next schedule queued up, activate it
    if(fuelSchedule->hasNextSchedule == true)
    {
      //check for possible overlap
      if((fuelSchedule->nextEndCompare-fuelSchedule->nextStartCompare)+ uS_TO_TIMER_COMPARE(INJECTION_OVERLAP_TRESHOLD)>=(fuelSchedule->nextEndCompare-fuelSchedule->endCompare)) 
      {
        fuelSchedule->setCompare(fuelSchedule->nextEndCompare);
        fuelSchedule->endCompare = fuelSchedule->nextEndCompare;
        fuelSchedule->Status = RUNNING;
        fuelSchedule->hasNextSchedule = false;
      }
      else //no overlap
      {
        fuelSchedule->EndFunction();
        fuelSchedule->setCompare(fuelSchedule->nextStartCompare);
        fuelSchedule->endCompare = fuelSchedule->nextEndCompare;
        fuelSchedule->Status = PENDING;
        fuelSchedule->hasNextSchedule = false;
      }
    }
    else //no next schedule
    {
    fuelSchedule->EndFunction();
    fuelSchedule->Status = OFF; //Turn off the schedule        
    }
  }
  else //(fuelSchedule->Status == OFF)
  {
    fuelSchedule->EndFunction();
    fuelSchedule->timerDisable(); //Safety check. Turn off this output compare unit and return without performing any action
  } 
}

#if IGN_CHANNELS >= 1
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPA_vect) //ignitionSchedule1
#else
static inline void ignitionSchedule1Interrupt(void) //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule1);
}
#endif

#if IGN_CHANNELS >= 2
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPB_vect) //ignitionSchedule2
#else
static inline void ignitionSchedule2Interrupt(void) //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule2);
}
#endif

#if IGN_CHANNELS >= 3
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPC_vect) //ignitionSchedule3
#else
static inline void ignitionSchedule3Interrupt(void) //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule3);
}
#endif

#if IGN_CHANNELS >= 4
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPA_vect) //ignitionSchedule4
#else
static inline void ignitionSchedule4Interrupt(void) //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule4);
}
#endif

#if IGN_CHANNELS >= 5
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPC_vect) //ignitionSchedule5
#else
static inline void ignitionSchedule5Interrupt(void) //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule5);
}
#endif

#if IGN_CHANNELS >= 6
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPB_vect) //ignitionSchedule6
#else
static inline void ignitionSchedule6Interrupt(void) //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule6);
}
#endif

#if IGN_CHANNELS >= 7
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER3_COMPC_vect) //ignitionSchedule6
#else
static inline void ignitionSchedule7Interrupt(void) //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule7);
}
#endif

#if IGN_CHANNELS >= 8
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER3_COMPB_vect) //ignitionSchedule8
#else
static inline void ignitionSchedule8Interrupt(void) //Most ARM chips can simply call a function
#endif
{
ignitionScheduleInterrupt(&ignitionSchedule8);
}
#endif

void ignitionScheduleInterrupt(struct Schedule *targetSchedule) // common function that all ignition channel interrupts use
{
    if (targetSchedule->Status == PENDING) //Check to see if this schedule is turn on
    {
      targetSchedule->StartFunction();
      targetSchedule->Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
      targetSchedule->startTime=(uint8_t)millis(); //store start time for overdwell protection
      targetSchedule->setCompare(targetSchedule->endCompare);
    }
    else if (targetSchedule->Status == RUNNING)
    {
      targetSchedule->EndFunction(); //Moment of spark 
      targetSchedule->Status = OFF; //Turn off the schedule

       //If there is a next schedule queued up, activate it
      if(targetSchedule->hasNextSchedule == true)
      {
        targetSchedule->setCompare(targetSchedule->nextStartCompare);
        targetSchedule->endCompare = targetSchedule->nextEndCompare;
        targetSchedule->Status = PENDING;
        targetSchedule->hasNextSchedule = false;
      }
      else {targetSchedule->timerDisable(); }
    }
    else //Safety check. Turn off this output compare unit and return without performing any action
    {
      targetSchedule->EndFunction();
      targetSchedule->timerDisable(); 
      } 
}