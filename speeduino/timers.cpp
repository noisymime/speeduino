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
#include "maths.h"

#if defined(CORE_AVR)
  #include <avr/wdt.h>
#endif

volatile uint16_t lastRPM_100ms; //Need to record this for rpmDOT calculation
volatile byte loop5ms;
volatile byte loop33ms;
volatile byte loop66ms;
volatile byte loop100ms;
volatile byte loop250ms;
volatile int loopSec;

volatile unsigned int dwellLimit_uS;

volatile uint8_t tachoEndTime; //The time (in ms) that the tacho pulse needs to end at
volatile TachoOutputStatus tachoOutputFlag;
volatile uint16_t tachoSweepIncr;
volatile uint16_t tachoSweepAccum;
volatile uint8_t testInjectorPulseCount = 0;
volatile uint8_t testIgnitionPulseCount = 0;

#if defined (CORE_TEENSY)
  IntervalTimer lowResTimer;
#endif

void initialiseTimers(void)
{
  lastRPM_100ms = 0;
  loop5ms = 0;
  loop33ms = 0;
  loop66ms = 0;
  loop100ms = 0;
  loop250ms = 0;
  loopSec = 0;
  tachoOutputFlag = TACHO_INACTIVE;
}

static inline void applyOverDwellCheck(IgnitionSchedule &schedule, uint32_t targetOverdwellTime) {
  //Check first whether each spark output is currently on. Only check it's dwell time if it is
  if ((schedule.Status == RUNNING) && (schedule.startTime < targetOverdwellTime)) { 
    schedule.pEndCallback(); schedule.Status = OFF; 
  }
}

//Timer2 Overflow Interrupt Vector, called when the timer overflows.
//Executes every ~1ms.
#if defined(CORE_AVR) //AVR chips use the ISR for this
//This MUST be no block. Turning NO_BLOCK off messes with timing accuracy. 
ISR(TIMER2_OVF_vect, ISR_NOBLOCK) //cppcheck-suppress misra-c2012-8.2
#else
void oneMSInterval(void) //Most ARM chips can simply call a function
#endif
{
  BIT_SET(TIMER_mask, BIT_TIMER_1KHZ);
  ms_counter++;

  //Increment Loop Counters
  loop5ms++;
  loop33ms++;
  loop66ms++;
  loop100ms++;
  loop250ms++;
  loopSec++;

  //Overdwell check
  uint32_t targetOverdwellTime = micros() - dwellLimit_uS; //Set a target time in the past that all coil charging must have begun after. If the coil charge began before this time, it's been running too long
  bool isCrankLocked = configPage4.ignCranklock && (currentStatus.RPM < currentStatus.crankRPM); //Dwell limiter is disabled during cranking on setups using the locked cranking timing. WE HAVE to do the RPM check here as relying on the engine cranking bit can be potentially too slow in updating
  if ((configPage4.useDwellLim == 1) && (isCrankLocked != true)) 
  {
    applyOverDwellCheck(ignitionSchedule1, targetOverdwellTime);
#if IGN_CHANNELS >= 2
    applyOverDwellCheck(ignitionSchedule2, targetOverdwellTime);
#endif
#if IGN_CHANNELS >= 3
    applyOverDwellCheck(ignitionSchedule3, targetOverdwellTime);
#endif
#if IGN_CHANNELS >= 4
    applyOverDwellCheck(ignitionSchedule4, targetOverdwellTime);
#endif
#if IGN_CHANNELS >= 5
    applyOverDwellCheck(ignitionSchedule5, targetOverdwellTime);
#endif
#if IGN_CHANNELS >= 6
    applyOverDwellCheck(ignitionSchedule6, targetOverdwellTime);
#endif
#if IGN_CHANNELS >= 7
    applyOverDwellCheck(ignitionSchedule7, targetOverdwellTime);
#endif
#if IGN_CHANNELS >= 8
    applyOverDwellCheck(ignitionSchedule8, targetOverdwellTime);
#endif
  }

  //Tacho is flagged as being ready for a pulse by the ignition outputs, or the sweep interval upon startup

  // See if we're in power-on sweep mode
  if( currentStatus.tachoSweepEnabled )
  {
    if( (currentStatus.engine != 0) || (ms_counter >= TACHO_SWEEP_TIME_MS) )  { currentStatus.tachoSweepEnabled = false; }  // Stop the sweep after SWEEP_TIME, or if real tach signals have started
    else 
    {
      // Ramp the needle smoothly to the max over the SWEEP_RAMP time
      if( ms_counter < TACHO_SWEEP_RAMP_MS ) { tachoSweepAccum += map(ms_counter, 0, TACHO_SWEEP_RAMP_MS, 0, tachoSweepIncr); }
      else                                   { tachoSweepAccum += tachoSweepIncr;                                             }
             
      // Each time it rolls over, it's time to pulse the Tach
      if( tachoSweepAccum >= MS_PER_SEC ) 
      {  
        tachoOutputFlag = READY;
        tachoSweepAccum -= MS_PER_SEC;
      }
    }
  }

  //Tacho output check. This code will not do anything if tacho pulse duration is fixed to coil dwell.
  if(tachoOutputFlag == READY)
  {
    //Check for half speed tacho
    if( (configPage2.tachoDiv == 0) || (currentStatus.tachoAlt == true) ) 
    { 
      TACHO_PULSE_LOW();
      //ms_counter is cast down to a byte as the tacho duration can only be in the range of 1-6, so no extra resolution above that is required
      tachoEndTime = (uint8_t)ms_counter + configPage2.tachoDuration;
      tachoOutputFlag = ACTIVE;
    }
    else
    {
      //Don't run on this pulse (Half speed tacho)
      tachoOutputFlag = TACHO_INACTIVE;
    }
    currentStatus.tachoAlt = !currentStatus.tachoAlt; //Flip the alternating value in case half speed tacho is in use. 
  }
  else if(tachoOutputFlag == ACTIVE)
  {
    //If the tacho output is already active, check whether it's reached it's end time
    if((uint8_t)ms_counter == tachoEndTime)
    {
      TACHO_PULSE_HIGH();
      tachoOutputFlag = TACHO_INACTIVE;
    }
  }

  //200Hz loop
  if (loop5ms == 5)
  {
    loop5ms = 0; //Reset counter
    BIT_SET(TIMER_mask, BIT_TIMER_200HZ);
  }  

  //30Hz loop
  if (loop33ms == 33)
  {
    loop33ms = 0;

    //Pulse fuel and ignition test outputs are set at 30Hz
    if( BIT_CHECK(currentStatus.testOutputs, 1) && (currentStatus.RPM == 0) )
    {
      //Check for pulsed injector output test
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ1_CMD_BIT)) { openInjector1(); }
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ2_CMD_BIT)) { openInjector2(); }
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ3_CMD_BIT)) { openInjector3(); }
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ4_CMD_BIT)) { openInjector4(); }
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ5_CMD_BIT)) { openInjector5(); }
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ6_CMD_BIT)) { openInjector6(); }
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ7_CMD_BIT)) { openInjector7(); }
      if(BIT_CHECK(HWTest_INJ_Pulsed, INJ8_CMD_BIT)) { openInjector8(); }
      testInjectorPulseCount = 0;

      //Check for pulsed ignition output test
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN1_CMD_BIT)) { beginCoil1Charge(); }
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN2_CMD_BIT)) { beginCoil2Charge(); }
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN3_CMD_BIT)) { beginCoil3Charge(); }
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN4_CMD_BIT)) { beginCoil4Charge(); }
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN5_CMD_BIT)) { beginCoil5Charge(); }
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN6_CMD_BIT)) { beginCoil6Charge(); }
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN7_CMD_BIT)) { beginCoil7Charge(); }
      if(BIT_CHECK(HWTest_IGN_Pulsed, IGN8_CMD_BIT)) { beginCoil8Charge(); }
      testIgnitionPulseCount = 0;
    }

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

    currentStatus.rpmDOT = (currentStatus.RPM - lastRPM_100ms) * 10; //This is the RPM per second that the engine has accelerated/decelerated in the last loop
    lastRPM_100ms = currentStatus.RPM; //Record the current RPM for next calc

    if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN) ) { runSecsX10++; }
    else { runSecsX10 = 0; }

    if ( (currentStatus.injPrimed == false) && (seclx10 == configPage2.primingDelay) && (currentStatus.RPM == 0) ) { beginInjectorPriming(); currentStatus.injPrimed = true; }
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
  }

  //1Hz loop
  if (loopSec == 1000)
  {
    loopSec = 0; //Reset counter.
    BIT_SET(TIMER_mask, BIT_TIMER_1HZ);

    dwellLimit_uS = (1000 * configPage4.dwellLimit); //Update uS value in case setting has changed
    currentStatus.crankRPM = ((unsigned int)configPage4.crankRPM * 10);

    //**************************************************************************************************************************************************
    //This updates the runSecs variable
    //If the engine is running or cranking, we need to update the run time counter.
    if (BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN))
    { //NOTE - There is a potential for a ~1sec gap between engine crank starting and the runSec number being incremented. This may delay ASE!
      if (currentStatus.runSecs <= 254) //Ensure we cap out at 255 and don't overflow. (which would reset ASE and cause problems with the closed loop fuelling (Which has to wait for the O2 to warmup))
        { currentStatus.runSecs++; } //Increment our run counter by 1 second.
    }
    //**************************************************************************************************************************************************
    //This records the number of main loops the system has completed in the last second
    currentStatus.loopsPerSecond = mainLoopCount;
    mainLoopCount = 0;
    //**************************************************************************************************************************************************
    //increment secl (secl is simply a counter that increments every second and is used to track whether the system has unexpectedly reset
    currentStatus.secl++;
    //**************************************************************************************************************************************************
    //Check the fan output status
    if (configPage2.fanEnable >= 1)
    {
       fanControl();            // Function to turn the cooling fan on/off
    }

    //Check whether fuel pump priming is complete
    if(currentStatus.fpPrimed == false)
    {
      //fpPrimeTime is the time that the pump priming started. This is 0 on startup, but can be changed if the unit has been running on USB power and then had the ignition turned on (Which starts the priming again)
      if( (currentStatus.secl - fpPrimeTime) >= configPage2.fpPrime)
      {
        currentStatus.fpPrimed = true; //Mark the priming as being completed
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
      flexPulseWidth = constrain(flexPulseWidth, 1000UL, 5000UL);
      int32_t tempX100 = (int32_t)rshift<10>(4224UL * flexPulseWidth) - 8125L; //Split up for MISRA compliance
      currentStatus.fuelTemp = div100((int16_t)tempX100);     
    }
  }

  //Turn off any of the pulsed testing outputs if they are active and have been running for long enough
  if( BIT_CHECK(currentStatus.testOutputs, 1) )
  {
    //Check for pulsed injector output test
    if( (HWTest_INJ_Pulsed > 0)  )
    {
      if(testInjectorPulseCount >= configPage13.hwTestInjDuration)
      {
        if(BIT_CHECK(HWTest_INJ_Pulsed, INJ1_CMD_BIT)) { closeInjector1(); }
        if(BIT_CHECK(HWTest_INJ_Pulsed, INJ2_CMD_BIT)) { closeInjector2(); }
        if(BIT_CHECK(HWTest_INJ_Pulsed, INJ3_CMD_BIT)) { closeInjector3(); }
        if(BIT_CHECK(HWTest_INJ_Pulsed, INJ4_CMD_BIT)) { closeInjector4(); }
        if(BIT_CHECK(HWTest_INJ_Pulsed, INJ5_CMD_BIT)) { closeInjector5(); }
        if(BIT_CHECK(HWTest_INJ_Pulsed, INJ6_CMD_BIT)) { closeInjector6(); }
        if(BIT_CHECK(HWTest_INJ_Pulsed, INJ7_CMD_BIT)) { closeInjector7(); }
        if(BIT_CHECK(HWTest_INJ_Pulsed, INJ8_CMD_BIT)) { closeInjector8(); }
        
        testInjectorPulseCount = 0;
      }
      else { testInjectorPulseCount++; }
    }
    

    //Check for pulsed ignition output test
    if( (HWTest_IGN_Pulsed > 0) )
    {
      if(testIgnitionPulseCount >= configPage13.hwTestIgnDuration)
      {
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN1_CMD_BIT)) { endCoil1Charge(); }
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN2_CMD_BIT)) { endCoil2Charge(); }
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN3_CMD_BIT)) { endCoil3Charge(); }
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN4_CMD_BIT)) { endCoil4Charge(); }
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN5_CMD_BIT)) { endCoil5Charge(); }
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN6_CMD_BIT)) { endCoil6Charge(); }
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN7_CMD_BIT)) { endCoil7Charge(); }
        if(BIT_CHECK(HWTest_IGN_Pulsed, IGN8_CMD_BIT)) { endCoil8Charge(); }

        testIgnitionPulseCount = 0;
      }
      else { testIgnitionPulseCount++; }
    }
    
  }


#if defined(CORE_AVR) //AVR chips use the ISR for this
    //Reset Timer2 to trigger in another ~1ms
    TCNT2 = 131;            //Preload timer2 with 100 cycles, leaving 156 till overflow.
#endif
}
