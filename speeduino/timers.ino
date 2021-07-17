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
#include "timers.h"
#include "globals.h"
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
  loop33ms = 0;
  loop66ms = 0;
  loop100ms = 0;
  loop250ms = 0;
  loopSec = 0;
}


//Timer2 Overflow Interrupt Vector, called when the timer overflows.
//Executes every ~1ms.
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER2_OVF_vect, ISR_NOBLOCK) //This MUST be no block. Turning NO_BLOCK off messes with timing accuracy
#else
void oneMSInterval() //Most ARM chips can simply call a function
#endif
{
  ms_counter++;

  //Increment Loop Counters
  loop33ms++;
  loop66ms++;
  loop100ms++;
  loop250ms++;
  loopSec++;

  unsigned long targetOverdwellTime;

  //Overdwell check
  targetOverdwellTime = micros() - dwellLimit_uS; //Set a target time in the past that all coil charging must have begun after. If the coil charge began before this time, it's been running too long
  bool isCrankLocked = configPage4.ignCranklock && (currentStatus.RPM < currentStatus.crankRPM); //Dwell limiter is disabled during cranking on setups using the locked cranking timing. WE HAVE to do the RPM check here as relying on the engine cranking bit can be potentially too slow in updating
  //Check first whether each spark output is currently on. Only check it's dwell time if it is

  if(ignitionSchedule1.Status == RUNNING) { if( (ignitionSchedule1.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { ign1EndFunction(); ignitionSchedule1.Status = OFF; } }
  if(ignitionSchedule2.Status == RUNNING) { if( (ignitionSchedule2.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { ign2EndFunction(); ignitionSchedule2.Status = OFF; } }
  if(ignitionSchedule3.Status == RUNNING) { if( (ignitionSchedule3.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { ign3EndFunction(); ignitionSchedule3.Status = OFF; } }
  if(ignitionSchedule4.Status == RUNNING) { if( (ignitionSchedule4.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { ign4EndFunction(); ignitionSchedule4.Status = OFF; } }
  if(ignitionSchedule5.Status == RUNNING) { if( (ignitionSchedule5.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { ign5EndFunction(); ignitionSchedule5.Status = OFF; } }
  if(ignitionSchedule6.Status == RUNNING) { if( (ignitionSchedule6.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { ign6EndFunction(); ignitionSchedule6.Status = OFF; } }
  if(ignitionSchedule7.Status == RUNNING) { if( (ignitionSchedule7.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { ign7EndFunction(); ignitionSchedule7.Status = OFF; } }
  if(ignitionSchedule8.Status == RUNNING) { if( (ignitionSchedule8.startTime < targetOverdwellTime) && (configPage4.useDwellLim) && (isCrankLocked != true) ) { ign8EndFunction(); ignitionSchedule8.Status = OFF; } }

  //Tacho output check
  //Tacho is flagged as being ready for a pulse by the ignition outputs. 
  if(tachoOutputFlag == READY)
  {
    //Check for half speed tacho
    if( (configPage2.tachoDiv == 0) || (tachoAlt == true) ) 
    { 
      TACHO_PULSE_LOW();
      //ms_counter is cast down to a byte as the tacho duration can only be in the range of 1-6, so no extra resolution above that is required
      tachoEndTime = (uint8_t)ms_counter + configPage2.tachoDuration;
      tachoOutputFlag = ACTIVE;
    }
    else
    {
      //Don't run on this pulse (Half speed tacho)
      tachoOutputFlag = DEACTIVE;
    }
    tachoAlt = !tachoAlt; //Flip the alternating value incase half speed tacho is in use. 
  }
  else if(tachoOutputFlag == ACTIVE)
  {
    //If the tacho output is already active, check whether it's reached it's end time
    if((uint8_t)ms_counter == tachoEndTime)
    {
      TACHO_PULSE_HIGH();
      tachoOutputFlag = DEACTIVE;
    }
  }
  // Tacho sweep
  


  //30Hz loop
  if (loop33ms == 33)
  {
    loop33ms = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_30HZ);
  }

  //15Hz loop
  if (loop66ms == 66)
  {
    loop66ms = 0;
    BIT_SET(TIMER_mask, BIT_TIMER_15HZ);
  }

  //10Hz loop
  if (loop100ms == 100)
  {
    loop100ms = 0; //Reset counter
    BIT_SET(TIMER_mask, BIT_TIMER_10HZ);

    currentStatus.rpmDOT = (currentStatus.RPM - lastRPM_100ms) * 10; //This is the RPM per second that the engine has accelerated/decelleratedin the last loop
    lastRPM_100ms = currentStatus.RPM; //Record the current RPM for next calc

    if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN) ) { runSecsX10++; }
    else { runSecsX10 = 0; }

    if ( (injPrimed == false) && (seclx10 == configPage2.primingDelay) && (currentStatus.RPM == 0) ) { beginInjectorPriming(); injPrimed = true; }
    seclx10++;
  }

  //4Hz loop
  if (loop250ms == 250)
  {
    loop250ms = 0; //Reset Counter
    BIT_SET(TIMER_mask, BIT_TIMER_4HZ);
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
  if (loopSec == 1000)
  {
    loopSec = 0; //Reset counter.
    BIT_SET(TIMER_mask, BIT_TIMER_1HZ);

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
    //Check the fan output status
    if (configPage6.fanEnable == 1)
    {
       fanControl();            // Fucntion to turn the cooling fan on/off
    }

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
    //**************************************************************************************************************************************************
    //Set the flex reading (if enabled). The flexCounter is updated with every pulse from the sensor. If cleared once per second, we get a frequency reading
    if(configPage2.flexEnabled == true)
    {
      byte tempEthPct = 0; 
      if(flexCounter < 50)
      {
        tempEthPct = 0; //Standard GM Continental sensor reads from 50Hz (0 ethanol) to 150Hz (Pure ethanol). Subtracting 50 from the frequency therefore gives the ethanol percentage.
        flexCounter = 0;
      }
      else if (flexCounter > 151) //1 pulse buffer
      {

        if(flexCounter < 169)
        {
          tempEthPct = 100;
          flexCounter = 0;
        }
        else
        {
          //This indicates an error condition. Spec of the sensor is that errors are above 170Hz)
          tempEthPct = 0;
          flexCounter = 0;
        }
      }
      else
      {
        tempEthPct = flexCounter - 50; //Standard GM Continental sensor reads from 50Hz (0 ethanol) to 150Hz (Pure ethanol). Subtracting 50 from the frequency therefore gives the ethanol percentage.
        flexCounter = 0;
      }

      //Off by 1 error check
      if (tempEthPct == 1) { tempEthPct = 0; }

      currentStatus.ethanolPct = ADC_FILTER(tempEthPct, configPage4.FILTER_FLEX, currentStatus.ethanolPct);

      //Continental flex sensor fuel temperature can be read with following formula: (Temperature = (41.25 * pulse width(ms)) - 81.25). 1000μs = -40C and 5000μs = 125C
      if(flexPulseWidth > 5000) { flexPulseWidth = 5000; }
      else if(flexPulseWidth < 1000) { flexPulseWidth = 1000; }
      currentStatus.fuelTemp = (((4224 * (long)flexPulseWidth) >> 10) - 8125) / 100;
    }

    //**************************************************************************************************************************************************
    //Handle any of the hardware testing outputs
    if( BIT_CHECK(currentStatus.testOutputs, 1) )
    {
      //Check whether any of the fuel outputs is on

      //Check for injector outputs on 50%
      if(BIT_CHECK(HWTest_INJ_50pc, INJ1_CMD_BIT)) { injector1Toggle(); }
      if(BIT_CHECK(HWTest_INJ_50pc, INJ2_CMD_BIT)) { injector2Toggle(); }
      if(BIT_CHECK(HWTest_INJ_50pc, INJ3_CMD_BIT)) { injector3Toggle(); }
      if(BIT_CHECK(HWTest_INJ_50pc, INJ4_CMD_BIT)) { injector4Toggle(); }
      if(BIT_CHECK(HWTest_INJ_50pc, INJ5_CMD_BIT)) { injector5Toggle(); }
      if(BIT_CHECK(HWTest_INJ_50pc, INJ6_CMD_BIT)) { injector6Toggle(); }
      if(BIT_CHECK(HWTest_INJ_50pc, INJ7_CMD_BIT)) { injector7Toggle(); }
      if(BIT_CHECK(HWTest_INJ_50pc, INJ8_CMD_BIT)) { injector8Toggle(); }

      //Check for ignition outputs on 50%
      if(BIT_CHECK(HWTest_IGN_50pc, IGN1_CMD_BIT)) { coil1Toggle(); }
      if(BIT_CHECK(HWTest_IGN_50pc, IGN2_CMD_BIT)) { coil2Toggle(); }
      if(BIT_CHECK(HWTest_IGN_50pc, IGN3_CMD_BIT)) { coil3Toggle(); }
      if(BIT_CHECK(HWTest_IGN_50pc, IGN4_CMD_BIT)) { coil4Toggle(); }
      if(BIT_CHECK(HWTest_IGN_50pc, IGN5_CMD_BIT)) { coil5Toggle(); }
      if(BIT_CHECK(HWTest_IGN_50pc, IGN6_CMD_BIT)) { coil6Toggle(); }
      if(BIT_CHECK(HWTest_IGN_50pc, IGN7_CMD_BIT)) { coil7Toggle(); }
      if(BIT_CHECK(HWTest_IGN_50pc, IGN8_CMD_BIT)) { coil8Toggle(); }
    }

  }
#if defined(CORE_AVR) //AVR chips use the ISR for this
    //Reset Timer2 to trigger in another ~1ms
    TCNT2 = 131;            //Preload timer2 with 100 cycles, leaving 156 till overflow.
#endif
}

