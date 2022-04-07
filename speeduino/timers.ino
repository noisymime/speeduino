/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
Timers are used for having actions performed repeatedly at a fixed interval (Eg every 100ms)
They should not be confused with Schedulers, which are for performing an action once at a given point of time in the future

Timers are typically low resolution (Compared to Schedulers), with maximum frequency currently being approximately every 10ms
*/
#include "globals.h"
#include "timers.h"
#include "sensors.h"
#include "scheduler.h"
#include "scheduledIO.h"
#include "speeduino.h"
#include "scheduler.h"
#include "auxiliaries.h"
#include "comms.h"

#if defined(CORE_AVR)
  #include <avr/wdt.h>
#endif

void initialiseTimers()
{
  lastRPM_100ms = 0;
}


//Timer2 Overflow Interrupt Vector, called when the timer overflows.
//Executes every ~1ms.
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER2_OVF_vect, ISR_NOBLOCK) //This MUST be no block. Turning NO_BLOCK off messes with timing accuracy
#else
void oneMSInterval() //Most ARM chips can simply call a function
#endif
{}

char getTimerFlags()
{
  static uint8_t previousMillis33ms=1;
  static uint8_t previousMillis66ms=2;
  static uint8_t previousMillis100ms=3;
  static uint16_t previousMillis250ms=4;
  static uint16_t previousMillis1000ms=6;
  uint16_t currentMillis;
  uint8_t interval;
  byte timerflags=0;

  currentMillis =(uint16_t)millis(); // capture the latest value of millis()
  //30Hz loop
  interval=lowByte(currentMillis) - previousMillis33ms;
  if(interval >= (uint8_t)33U )
  {
    previousMillis33ms=lowByte(currentMillis); 
    BIT_SET(timerflags, BIT_TIMER_30HZ);
  }

  //15Hz loop
  interval=lowByte(currentMillis) - previousMillis66ms;
  if(interval >= (uint8_t)66U )
  {
    previousMillis66ms=lowByte(currentMillis);
    BIT_SET(timerflags, BIT_TIMER_15HZ);
  }

  //10Hz loop
  interval=lowByte(currentMillis) - previousMillis100ms;
  if(interval >= (uint8_t)100U )
  {
    previousMillis100ms=lowByte(currentMillis);
    BIT_SET(timerflags, BIT_TIMER_10HZ);

    currentStatus.rpmDOT = (currentStatus.RPM - lastRPM_100ms) * 10; //This is the RPM per second that the engine has accelerated/decelleratedin the last loop
    lastRPM_100ms = currentStatus.RPM; //Record the current RPM for next calc

    if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN) ) { runSecsX10++; }
    else { runSecsX10 = 0; }

    if ( (injPrimed == false) && (seclx10 == configPage2.primingDelay) && (currentStatus.RPM == 0) ) { beginInjectorPriming(); injPrimed = true; }
    seclx10++;
  }

  //4Hz loop
  if((uint16_t)(currentMillis - previousMillis250ms) >= (uint16_t)250U )
  {
    previousMillis250ms+=(uint16_t)250U;
    BIT_SET(timerflags, BIT_TIMER_4HZ);
    #if defined(CORE_STM32) //debug purpose, only visual for running code
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    #endif

    #if defined(CORE_AVR)
      //Reset watchdog timer (Not active currently)
      //wdt_reset();
      //DIY watchdog
      //This is a sign of a crash:
      //if( (initialisationComplete == true) && (last250msLoopCount == mainLoopCount) ) { setup(); }
      //else { last250msLoopCount = mainLoopCount; }
    #endif
  }

  //1Hz loop
  if((uint16_t)(currentMillis - previousMillis1000ms) >= (uint16_t)1000U )
  {
    previousMillis1000ms+=(uint16_t)1000U;
    BIT_SET(timerflags, BIT_TIMER_1HZ);

    dwellLimit_uS = (1000 * configPage4.dwellLimit); //Update uS value incase setting has changed
    currentStatus.crankRPM = ((unsigned int)configPage4.crankRPM * 10);

    //**************************************************************************************************************************************************
    //This updates the runSecs variable
    //If the engine is running or cranking, we need ot update the run time counter.
    if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN))
    { //NOTE - There is a potential for a ~1sec gap between engine crank starting and ths runSec number being incremented. This may delay ASE!
      if (currentStatus.runSecs <= 254) //Ensure we cap out at 255 and don't overflow. (which would reset ASE and cause problems with the closed loop fueling (Which has to wait for the O2 to warmup))
        { currentStatus.runSecs++; } //Increment our run counter by 1 second.
    }
    //**************************************************************************************************************************************************
    //This records the number of main loops the system has completed in the last second
    currentStatus.loopsPerSecond = mainLoopCount;
    mainLoopCount = 0;
    //**************************************************************************************************************************************************
    //increament secl (secl is simply a counter that increments every second and is used to track whether the system has unexpectedly reset
    currentStatus.secl++;
    //**************************************************************************************************************************************************
    //Check whether fuel pump priming is complete
    if(fpPrimed == false)
    {
      //fpPrimeTime is the time that the pump priming started. This is 0 on startup, but can be changed if the unit has been running on USB power and then had the ignition turned on (Which starts the priming again)
      if( (currentStatus.secl - fpPrimeTime) >= configPage2.fpPrime)
      {
        fpPrimed = true; //Mark the priming as being completed
        if(currentStatus.RPM == 0)
        {
          //If we reach here then the priming is complete, however only turn off the fuel pump if the engine isn't running
          digitalWrite(pinFuelPump, LOW);
          currentStatus.fuelPumpOn = false;
        }
      }
    }
  }
return timerflags;
}