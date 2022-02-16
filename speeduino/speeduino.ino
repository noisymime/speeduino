/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,la
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/
/** @file
 * Speeduino initialization and main loop.
 */
#include <stdint.h> //developer.mbed.org/handbook/C-Data-Types
//************************************************
#include "globals.h"
#include "speeduino.h"
#include "scheduler.h"
#include "comms.h"
#include "newComms.h"
#include "cancomms.h"
#include "maths.h"
#include "corrections.h"
#include "timers.h"
#include "decoders.h"
#include "idle.h"
#include "auxiliaries.h"
#include "sensors.h"
#include "storage.h"
#include "crankMaths.h"
#include "init.h"
#include "utilities.h"
#include "engineProtection.h"
#include "secondaryTables.h"
#include "SD_logger.h"
#include RTC_LIB_H //Defined in each boards .h file
#include BOARD_H //Note that this is not a real file, it is defined in globals.h. 

int ignition1StartAngle = 0;
int ignition2StartAngle = 0;
int ignition3StartAngle = 0;
int ignition4StartAngle = 0;
int ignition5StartAngle = 0;
int ignition6StartAngle = 0;
int ignition7StartAngle = 0;
int ignition8StartAngle = 0;

int channel1IgnDegrees = 0; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */
int channel2IgnDegrees = 0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
int channel3IgnDegrees = 0; /**< The number of crank degrees until cylinder 3 (and 5/6/7/8) is at TDC */
int channel4IgnDegrees = 0; /**< The number of crank degrees until cylinder 4 (and 5/6/7/8) is at TDC */
int channel5IgnDegrees = 0; /**< The number of crank degrees until cylinder 5 is at TDC */
int channel6IgnDegrees = 0; /**< The number of crank degrees until cylinder 6 is at TDC */
int channel7IgnDegrees = 0; /**< The number of crank degrees until cylinder 7 is at TDC */
int channel8IgnDegrees = 0; /**< The number of crank degrees until cylinder 8 is at TDC */
int channel1InjDegrees = 0; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */
int channel2InjDegrees = 0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
int channel3InjDegrees = 0; /**< The number of crank degrees until cylinder 3 (and 5/6/7/8) is at TDC */
int channel4InjDegrees = 0; /**< The number of crank degrees until cylinder 4 (and 5/6/7/8) is at TDC */
int channel5InjDegrees = 0; /**< The number of crank degrees until cylinder 5 is at TDC */
int channel6InjDegrees = 0; /**< The number of crank degrees until cylinder 6 is at TDC */
int channel7InjDegrees = 0; /**< The number of crank degrees until cylinder 7 is at TDC */
int channel8InjDegrees = 0; /**< The number of crank degrees until cylinder 8 is at TDC */

uint16_t req_fuel_uS = 0; /**< The required fuel variable (As calculated by TunerStudio) in uS */
uint16_t inj_opentime_uS = 0;

bool ignitionOn = false; /**< The current state of the ignition system (on or off) */
bool fuelOn = false; /**< The current state of the fuel system (on or off) */

byte maxIgnOutputs = 1; /**< Used for rolling rev limiter to indicate how many total ignition channels should currently be firing */
byte curRollingCut = 0; /**< Rolling rev limiter, current ignition channel being cut */
byte rollingCutCounter = 0; /**< how many times (revolutions) the ignition has been cut in a row */
uint32_t rollingCutLastRev = 0; /**< Tracks whether we're on the same or a different rev for the rolling cut */

uint16_t staged_req_fuel_mult_pri = 0;
uint16_t staged_req_fuel_mult_sec = 0;   
#ifndef UNIT_TEST // Scope guard for unit testing
void setup()
{
  initialisationComplete = false; //Tracks whether the initialiseAll() function has run completely
  initialiseAll();
}
/** Speeduino main loop.
 * 
 * Main loop chores (roughly in  order they are preformed):
 * - Check if serial comms or tooth logging are in progress (send or reveive, prioritize communication)
 * - Record loop timing vars
 * - Check tooth time, update @ref statuses (currentStatus) variables
 * - Read sensors
 * - get VE for fuel calcs and spark advance for ignition
 * - Check crank/cam/tooth/timing sync (skip remaining ops if out-of-sync)
 * - execute doCrankSpeedCalcs()
 * 
 * single byte variable @ref LOOP_TIMER plays a big part here as:
 * - it contains expire-bits for interval based frequency driven events (e.g. 15Hz, 4Hz, 1Hz)
 * - Can be tested for certain frequency interval being expired by (eg) BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ)
 * 
 */
void loop()
{
      mainLoopCount++;
      LOOP_TIMER = TIMER_mask;

      //SERIAL Comms
      //Initially check that the last serial send values request is not still outstanding
      if (serialInProgress == true) 
      { 
        if(Serial.availableForWrite() > 16) { sendValues(inProgressOffset, inProgressLength, 0x30, 0); }
      }
      //Perform the same check for the tooth and composite logs
      if( toothLogSendInProgress == true)
      {
        if(Serial.availableForWrite() > 16) { sendToothLog(inProgressOffset); }
      }
      if( compositeLogSendInProgress == true)
      {
        if(Serial.availableForWrite() > 16) { sendCompositeLog(inProgressOffset); }
      }
      if(serialWriteInProgress == true)
      {
        if(Serial.availableForWrite() > 16) { continueSerialTransmission(); }
      }

      //Check for any new requets from serial.
      //if ( (Serial.available()) > 0) { command(); }
      if ( (Serial.available()) > 0) { parseSerial(); }
      
      else if(cmdPending == true)
      {
        //This is a special case just for the tooth and composite loggers
        if (currentCommand == 'T') { command(); }
      }

      //Check for any CAN comms requiring action 
      #if defined(CANSerial_AVAILABLE)
        //if can or secondary serial interface is enabled then check for requests.
        if (configPage9.enable_secondarySerial == 1)  //secondary serial interface enabled
        {
          if ( ((mainLoopCount & 31) == 1) or (CANSerial.available() > SERIAL_BUFFER_THRESHOLD) )
          {
            if (CANSerial.available() > 0)  { secondserial_Command(); }
          }
        }
      #endif
      #if defined (NATIVE_CAN_AVAILABLE)
          //currentStatus.canin[12] = configPage9.enable_intcan;
          if (configPage9.enable_intcan == 1) // use internal can module
          {
            //check local can module
            // if ( BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ) or (CANbus0.available())
            while (Can0.read(inMsg) ) 
            {
              can_Command();
              //Can0.read(inMsg);
              //currentStatus.canin[12] = inMsg.buf[5];
              //currentStatus.canin[13] = inMsg.id;
            }
          }
      #endif
          
    //Displays currently disabled
    // if (configPage2.displayType && (mainLoopCount & 255) == 1) { updateDisplay();}

    previousLoopTime = currentLoopTime;
    currentLoopTime = micros_safe();
    unsigned long timeToLastTooth = (currentLoopTime - toothLastToothTime);
    if ( (timeToLastTooth < MAX_STALL_TIME) || (toothLastToothTime > currentLoopTime) ) //Check how long ago the last tooth was seen compared to now. If it was more than half a second ago then the engine is probably stopped. toothLastToothTime can be greater than currentLoopTime if a pulse occurs between getting the lastest time and doing the comparison
    {
      currentStatus.longRPM = getRPM(); //Long RPM is included here
      currentStatus.RPM = currentStatus.longRPM;
      currentStatus.RPMdiv100 = divu100(currentStatus.RPM);
      FUEL_PUMP_ON();
      currentStatus.fuelPumpOn = true; //Not sure if this is needed.
    }
    else
    {
      //We reach here if the time between teeth is too great. This VERY likely means the engine has stopped
      currentStatus.RPM = 0;
      currentStatus.PW1 = 0;
      currentStatus.VE = 0;
      currentStatus.VE2 = 0;
      toothLastToothTime = 0;
      toothLastSecToothTime = 0;
      //toothLastMinusOneToothTime = 0;
      currentStatus.hasSync = false;
      BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC);
      currentStatus.runSecs = 0; //Reset the counter for number of seconds running.
      currentStatus.startRevolutions = 0;
      toothSystemCount = 0;
      secondaryToothCount = 0;
      MAPcurRev = 0;
      MAPcount = 0;
      currentStatus.rpmDOT = 0;
      AFRnextCycle = 0;
      ignitionCount = 0;
      ignitionOn = false;
      fuelOn = false;
      if (fpPrimed == true) { FUEL_PUMP_OFF(); currentStatus.fuelPumpOn = false; } //Turn off the fuel pump, but only if the priming is complete
      if (configPage6.iacPWMrun == false) { disableIdle(); } //Turn off the idle PWM
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK); //Clear cranking bit (Can otherwise get stuck 'on' even with 0 rpm)
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_WARMUP); //Same as above except for WUE
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_RUN); //Same as above except for RUNNING status
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE); //Same as above except for ASE status
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC); //Same as above but the accel enrich (If using MAP accel enrich a stall will cause this to trigger)
      //This is a safety check. If for some reason the interrupts have got screwed up (Leading to 0rpm), this resets them.
      //It can possibly be run much less frequently.
      //This should only be run if the high speed logger are off because it will change the trigger interrupts back to defaults rather than the logger versions
      if( (currentStatus.toothLogEnabled == false) && (currentStatus.compositeLogEnabled == false) ) { initialiseTriggers(); }

      VVT1_PIN_LOW();
      VVT2_PIN_LOW();
      DISABLE_VVT_TIMER();
      boostDisable();
      if(configPage4.ignBypassEnabled > 0) { digitalWrite(pinIgnBypass, LOW); } //Reset the ignition bypass ready for next crank attempt
    }

    //***Perform sensor reads***
    //-----------------------------------------------------------------------------------------------------
    readMAP();
    
    if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ)) //Every 32 loops
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_15HZ);
      #if TPS_READ_FREQUENCY == 15
        readTPS(); //TPS reading to be performed every 32 loops (any faster and it can upset the TPSdot sampling time)
      #endif
      #if  defined(CORE_TEENSY35)       
          if (configPage9.enable_intcan == 1) // use internal can module
          {
           // this is just to test the interface is sending
           //sendCancommand(3,((configPage9.realtime_base_address & 0x3FF)+ 0x100),currentStatus.TPS,0,0x200);
          }
      #endif     

      //Check for launching/flat shift (clutch) can be done around here too
      previousClutchTrigger = clutchTrigger;
      //Only check for pinLaunch if any function using it is enabled. Else pins might break starting a board
      if(configPage6.flatSEnable || configPage6.launchEnabled){
        if(configPage6.launchHiLo > 0) { clutchTrigger = digitalRead(pinLaunch); }
        else { clutchTrigger = !digitalRead(pinLaunch); }
      }

      if(previousClutchTrigger != clutchTrigger) { currentStatus.clutchEngagedRPM = currentStatus.RPM; }

      if (configPage6.launchEnabled && clutchTrigger && (currentStatus.clutchEngagedRPM < ((unsigned int)(configPage6.flatSArm) * 100)) && (currentStatus.RPM > ((unsigned int)(configPage6.lnchHardLim) * 100)) && (currentStatus.TPS >= configPage10.lnchCtrlTPS) ) 
      { 
        //HardCut rev limit for 2-step launch control.
        currentStatus.launchingHard = true; 
        BIT_SET(currentStatus.spark, BIT_SPARK_HLAUNCH); 
      } 
      else 
      { 
        //FLag launch as being off
        currentStatus.launchingHard = false; 
        BIT_CLEAR(currentStatus.spark, BIT_SPARK_HLAUNCH); 

        //If launch is not active, check whether flat shift should be active
        if(configPage6.flatSEnable && clutchTrigger && (currentStatus.RPM > ((unsigned int)(configPage6.flatSArm) * 100)) && (currentStatus.RPM > currentStatus.clutchEngagedRPM) ) { currentStatus.flatShiftingHard = true; }
        else { currentStatus.flatShiftingHard = false; }
      }

      //And check whether the tooth log buffer is ready
      if(toothHistoryIndex > TOOTH_LOG_SIZE) { BIT_SET(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY); }

    }
    if(BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ)) //10 hertz
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_10HZ);
      //updateFullStatus();
      checkProgrammableIO();

      if( (isEepromWritePending() == true) && (serialReceivePending == false) && (deferEEPROMWrites == false)) { writeAllConfig(); } //Check for any outstanding EEPROM writes.

      currentStatus.vss = getSpeed();
      currentStatus.gear = getGear();

      #ifdef SD_LOGGING
        if(configPage13.onboard_log_file_rate == LOGGER_RATE_10HZ) { writeSDLogEntry(); }
      #endif
    }
    if(BIT_CHECK(LOOP_TIMER, BIT_TIMER_30HZ)) //30 hertz
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_30HZ);
      //Most boost tends to run at about 30Hz, so placing it here ensures a new target time is fetched frequently enough
      boostControl();
      //VVT may eventually need to be synced with the cam readings (ie run once per cam rev) but for now run at 30Hz
      vvtControl();
      //Water methanol injection
      wmiControl();
      //FOR TEST PURPOSES ONLY!!!
      //if(vvt2_pwm_value < vvt_pwm_max_count) { vvt2_pwm_value++; }
      //else { vvt2_pwm_value = 1; }
      #if TPS_READ_FREQUENCY == 30
        readTPS();
      #endif

      #ifdef SD_LOGGING
        if(configPage13.onboard_log_file_rate == LOGGER_RATE_30HZ) { writeSDLogEntry(); }
      #endif
    }
    if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_4HZ))
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_4HZ);
      //The IAT and CLT readings can be done less frequently (4 times per second)
      readCLT();
      readIAT();
      readO2();
      readO2_2();
      readBat();
      nitrousControl();
      idleControl(); //Perform any idle related actions. Even at higher frequencies, running 4x per second is sufficient.

      #ifdef SD_LOGGING
        if(configPage13.onboard_log_file_rate == LOGGER_RATE_4HZ) { writeSDLogEntry(); }
      #endif  
      
      currentStatus.fuelPressure = getFuelPressure();
      currentStatus.oilPressure = getOilPressure();

      if(auxIsEnabled == true)
      {
        //TODO dazq to clean this right up :)
        //check through the Aux input channels if enabed for Can or local use
        for (byte AuxinChan = 0; AuxinChan <16 ; AuxinChan++)
        {
          currentStatus.current_caninchannel = AuxinChan;          
          
          if (((configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 4) 
              && (((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 0)&&(configPage9.intcan_available == 1)))
              || ((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 1)&&(configPage9.intcan_available == 1))&& 
              ((configPage9.caninput_sel[currentStatus.current_caninchannel]&64) == 0))
              || ((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 1)&&(configPage9.intcan_available == 0)))))              
          { //if current input channel is enabled as external & secondary serial enabled & internal can disabled(but internal can is available)
            // or current input channel is enabled as external & secondary serial enabled & internal can enabled(and internal can is available)
            //currentStatus.canin[13] = 11;  Dev test use only!
            if (configPage9.enable_secondarySerial == 1)  // megas only support can via secondary serial
            {
              sendCancommand(2,0,currentStatus.current_caninchannel,0,((configPage9.caninput_source_can_address[currentStatus.current_caninchannel]&2047)+0x100));
              //send an R command for data from caninput_source_address[currentStatus.current_caninchannel] from CANSERIAL
            }
          }  
          else if (((configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 4) 
              && (((configPage9.enable_secondarySerial == 1) && ((configPage9.enable_intcan == 1)&&(configPage9.intcan_available == 1))&& 
              ((configPage9.caninput_sel[currentStatus.current_caninchannel]&64) == 64))
              || ((configPage9.enable_secondarySerial == 0) && ((configPage9.enable_intcan == 1)&&(configPage9.intcan_available == 1))&& 
              ((configPage9.caninput_sel[currentStatus.current_caninchannel]&128) == 128))))                             
          { //if current input channel is enabled as external for canbus & secondary serial enabled & internal can enabled(and internal can is available)
            // or current input channel is enabled as external for canbus & secondary serial disabled & internal can enabled(and internal can is available)
            //currentStatus.canin[13] = 12;  Dev test use only!  
          #if defined(CORE_STM32) || defined(CORE_TEENSY)
           if (configPage9.enable_intcan == 1) //  if internal can is enabled 
           {
              sendCancommand(3,configPage9.speeduino_tsCanId,currentStatus.current_caninchannel,0,((configPage9.caninput_source_can_address[currentStatus.current_caninchannel]&2047)+0x100));  
              //send an R command for data from caninput_source_address[currentStatus.current_caninchannel] from internal canbus
           }
          #endif
          }   
          else if ((((configPage9.enable_secondarySerial == 1) || ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1))) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 8)
                  || (((configPage9.enable_secondarySerial == 0) && ( (configPage9.enable_intcan == 1) && (configPage9.intcan_available == 0) )) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 2)  
                  || (((configPage9.enable_secondarySerial == 0) && (configPage9.enable_intcan == 0)) && ((configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 2)))  
          { //if current input channel is enabled as analog local pin
            //read analog channel specified
            //currentStatus.canin[13] = (configPage9.Auxinpina[currentStatus.current_caninchannel]&63);  Dev test use only!127
            currentStatus.canin[currentStatus.current_caninchannel] = readAuxanalog(configPage9.Auxinpina[currentStatus.current_caninchannel]&63);
          }
          else if ((((configPage9.enable_secondarySerial == 1) || ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1))) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 12)
                  || (((configPage9.enable_secondarySerial == 0) && ( (configPage9.enable_intcan == 1) && (configPage9.intcan_available == 0) )) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 3)
                  || (((configPage9.enable_secondarySerial == 0) && (configPage9.enable_intcan == 0)) && ((configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 3)))
          { //if current input channel is enabled as digital local pin
            //read digital channel specified
            //currentStatus.canin[14] = ((configPage9.Auxinpinb[currentStatus.current_caninchannel]&63)+1);  Dev test use only!127+1
            currentStatus.canin[currentStatus.current_caninchannel] = readAuxdigital((configPage9.Auxinpinb[currentStatus.current_caninchannel]&63)+1);
          } //Channel type
        } //For loop going through each channel
      } //aux channels are enabled
    } //4Hz timer
    if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_1HZ)) //Once per second)
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_1HZ);
      readBaro(); //Infrequent baro readings are not an issue.
      deferEEPROMWrites = false; //Reset the slow EEPROM writes flag so that EEPROM burns will return to normal speed. This is set true in NewComms whenever there is a large chunk write to prvent mega2560s halting due to excess EEPROM burn times. 

      if ( (configPage10.wmiEnabled > 0) && (configPage10.wmiIndicatorEnabled > 0) )
      {
        // water tank empty
        if (BIT_CHECK(currentStatus.status4, BIT_STATUS4_WMI_EMPTY) > 0)
        {
          // flash with 1sec inverval
          digitalWrite(pinWMIIndicator, !digitalRead(pinWMIIndicator));
        }
        else
        {
          digitalWrite(pinWMIIndicator, configPage10.wmiIndicatorPolarity ? HIGH : LOW);
        } 
      }

      #ifdef SD_LOGGING
        if(configPage13.onboard_log_file_rate == LOGGER_RATE_1HZ) { writeSDLogEntry(); }
      #endif

    } //1Hz timer

    if( (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OL)
    || (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_CL)
    || (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OLCL) )
    {
      idleControl(); //Run idlecontrol every loop for stepper idle.
    }

    
    //VE and advance calculation were moved outside the sync/RPM check so that the fuel and ignition load value will be accurately shown when RPM=0
    currentStatus.VE1 = getVE1();
    currentStatus.VE = currentStatus.VE1; //Set the final VE value to be VE 1 as a default. This may be changed in the section below

    currentStatus.advance1 = getAdvance1();
    currentStatus.advance = currentStatus.advance1; //Set the final advance value to be advance 1 as a default. This may be changed in the section below

    calculateSecondaryFuel();
    calculateSecondarySpark();

    //Always check for sync
    //Main loop runs within this clause
    if (currentStatus.hasSync && (currentStatus.RPM > 0))
    {
        if(currentStatus.startRevolutions >= configPage4.StgCycles)  { ignitionOn = true; fuelOn = true; } //Enable the fuel and ignition, assuming staging revolutions are complete
        //Check whether running or cranking
        if(currentStatus.RPM > currentStatus.crankRPM) //Crank RPM in the config is stored as a x10. currentStatus.crankRPM is set in timers.ino and represents the true value
        {
          BIT_SET(currentStatus.engine, BIT_ENGINE_RUN); //Sets the engine running bit
          //Only need to do anything if we're transitioning from cranking to running
          if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
          {
            BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK);
            if(configPage4.ignBypassEnabled > 0) { digitalWrite(pinIgnBypass, HIGH); }
          }
        }
        else
        {  
          //Sets the engine cranking bit, clears the engine running bit
          BIT_SET(currentStatus.engine, BIT_ENGINE_CRANK);
          BIT_CLEAR(currentStatus.engine, BIT_ENGINE_RUN);
          currentStatus.runSecs = 0; //We're cranking (hopefully), so reset the engine run time to prompt ASE.
          if(configPage4.ignBypassEnabled > 0) { digitalWrite(pinIgnBypass, LOW); }

          //Check whether the user has selected to disable to the fan during cranking
          if(configPage2.fanWhenCranking == 0) { FAN_OFF(); }
        }
      //END SETTING ENGINE STATUSES
      //-----------------------------------------------------------------------------------------------------

      //Begin the fuel calculation
      //Calculate an injector pulsewidth from the VE
      currentStatus.corrections = correctionsFuel();

      currentStatus.PW1 = PW(req_fuel_uS, currentStatus.VE, currentStatus.MAP, currentStatus.corrections, inj_opentime_uS);

      //Manual adder for nitrous. These are not in correctionsFuel() because they are direct adders to the ms value, not % based
      if( (currentStatus.nitrous_status == NITROUS_STAGE1) || (currentStatus.nitrous_status == NITROUS_BOTH) )
      { 
        int16_t adderRange = (configPage10.n2o_stage1_maxRPM - configPage10.n2o_stage1_minRPM) * 100;
        int16_t adderPercent = ((currentStatus.RPM - (configPage10.n2o_stage1_minRPM * 100)) * 100) / adderRange; //The percentage of the way through the RPM range
        adderPercent = 100 - adderPercent; //Flip the percentage as we go from a higher adder to a lower adder as the RPMs rise
        currentStatus.PW1 = currentStatus.PW1 + (configPage10.n2o_stage1_adderMax + percentage(adderPercent, (configPage10.n2o_stage1_adderMin - configPage10.n2o_stage1_adderMax))) * 100; //Calculate the above percentage of the calculated ms value.
      }
      if( (currentStatus.nitrous_status == NITROUS_STAGE2) || (currentStatus.nitrous_status == NITROUS_BOTH) )
      {
        int16_t adderRange = (configPage10.n2o_stage2_maxRPM - configPage10.n2o_stage2_minRPM) * 100;
        int16_t adderPercent = ((currentStatus.RPM - (configPage10.n2o_stage2_minRPM * 100)) * 100) / adderRange; //The percentage of the way through the RPM range
        adderPercent = 100 - adderPercent; //Flip the percentage as we go from a higher adder to a lower adder as the RPMs rise
        currentStatus.PW1 = currentStatus.PW1 + (configPage10.n2o_stage2_adderMax + percentage(adderPercent, (configPage10.n2o_stage2_adderMin - configPage10.n2o_stage2_adderMax))) * 100; //Calculate the above percentage of the calculated ms value.
      }

      int injector1StartAngle = 0;
      uint16_t injector2StartAngle = 0;
      uint16_t injector3StartAngle = 0;
      uint16_t injector4StartAngle = 0;

      #if INJ_CHANNELS >= 5
      uint16_t injector5StartAngle = 0;
      #endif
      #if INJ_CHANNELS >= 6
      uint16_t injector6StartAngle = 0;
      #endif
      #if INJ_CHANNELS >= 7
      uint16_t injector7StartAngle = 0;
      #endif
      #if INJ_CHANNELS >= 8
      uint16_t injector8StartAngle = 0;
      #endif
      //These are used for comparisons on channels above 1 where the starting angle (for injectors or ignition) can be less than a single loop time
      //(Don't ask why this is needed, it's just there)
      int tempCrankAngle;
      int tempStartAngle;

      doCrankSpeedCalcs(); //In crankMaths.ino

      //Check that the duty cycle of the chosen pulsewidth isn't too high.
      unsigned long pwLimit = percentage(configPage2.dutyLim, revolutionTime); //The pulsewidth limit is determined to be the duty cycle limit (Eg 85%) by the total time it takes to perform 1 revolution
      //Handle multiple squirts per rev
      if (configPage2.strokes == FOUR_STROKE) { pwLimit = pwLimit * 2 / currentStatus.nSquirts; } 
      else { pwLimit = pwLimit / currentStatus.nSquirts; }
      //Apply the pwLimit if staging is dsiabled and engine is not cranking
      if( (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) && (configPage10.stagingEnabled == false) ) { if (currentStatus.PW1 > pwLimit) { currentStatus.PW1 = pwLimit; } }

      //Calculate staging pulsewidths if used
      //To run staged injection, the number of cylinders must be less than or equal to the injector channels (ie Assuming you're running paired injection, you need at least as many injector channels as you have cylinders, half for the primaries and half for the secondaries)
      if( (configPage10.stagingEnabled == true) && (configPage2.nCylinders <= INJ_CHANNELS || configPage2.injType == INJ_TYPE_TBODY) && (currentStatus.PW1 > inj_opentime_uS) ) //Final check is to ensure that DFCO isn't active, which would cause an overflow below (See #267)
      {
        //Scale the 'full' pulsewidth by each of the injector capacities
        currentStatus.PW1 -= inj_opentime_uS; //Subtract the opening time from PW1 as it needs to be multiplied out again by the pri/sec req_fuel values below. It is added on again after that calculation. 
        uint32_t tempPW1 = (((unsigned long)currentStatus.PW1 * staged_req_fuel_mult_pri) / 100);

        if(configPage10.stagingMode == STAGING_MODE_TABLE)
        {
          uint32_t tempPW3 = (((unsigned long)currentStatus.PW1 * staged_req_fuel_mult_sec) / 100); //This is ONLY needed in in table mode. Auto mode only calculates the difference.

          byte stagingSplit = get3DTableValue(&stagingTable, currentStatus.MAP, currentStatus.RPM);
          currentStatus.PW1 = ((100 - stagingSplit) * tempPW1) / 100;
          currentStatus.PW1 += inj_opentime_uS; 

          if(stagingSplit > 0) 
          { 
            currentStatus.PW3 = (stagingSplit * tempPW3) / 100; 
            currentStatus.PW3 += inj_opentime_uS;
          }
          else { currentStatus.PW3 = 0; }
        }
        else if(configPage10.stagingMode == STAGING_MODE_AUTO)
        {
          currentStatus.PW1 = tempPW1;
          //If automatic mode, the primary injectors are used all the way up to their limit (Configured by the pulsewidth limit setting)
          //If they exceed their limit, the extra duty is passed to the secondaries
          if(tempPW1 > pwLimit)
          {
            uint32_t extraPW = tempPW1 - pwLimit + inj_opentime_uS; //The open time must be added here AND below because tempPW1 does not include an open time. The addition of it here takes into account the fact that pwLlimit does not contain an allowance for an open time. 
            currentStatus.PW1 = pwLimit;
            currentStatus.PW3 = ((extraPW * staged_req_fuel_mult_sec) / staged_req_fuel_mult_pri); //Convert the 'left over' fuel amount from primary injector scaling to secondary
            currentStatus.PW3 += inj_opentime_uS;
          }
          else { currentStatus.PW3 = 0; } //If tempPW1 < pwLImit it means that the entire fuel load can be handled by the primaries. Simply set the secondaries to 0
        }

        //Set the 2nd channel of each stage with the same pulseWidth
        currentStatus.PW2 = currentStatus.PW1;
        currentStatus.PW4 = currentStatus.PW3;
      }
      else 
      { 
        //If staging is off, all the pulse widths are set the same (Sequential and other adjustments may be made below)
        currentStatus.PW2 = currentStatus.PW1;
        currentStatus.PW3 = currentStatus.PW1;
        currentStatus.PW4 = currentStatus.PW1;
        currentStatus.PW5 = currentStatus.PW1;
        currentStatus.PW6 = currentStatus.PW1;
        currentStatus.PW7 = currentStatus.PW1;
        currentStatus.PW8 = currentStatus.PW1;
      }

      //***********************************************************************************************
      //BEGIN INJECTION TIMING
      currentStatus.injAngle = table2D_getValue(&injectorAngleTable, currentStatus.RPMdiv100);
      unsigned int PWdivTimerPerDegree = div(currentStatus.PW1, timePerDegree).quot; //How many crank degrees the calculated PW will take at the current speed

      injector1StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees);

      //Repeat the above for each cylinder
      switch (configPage2.nCylinders)
      {
        //Single cylinder
        case 1:
          //The only thing that needs to be done for single cylinder is to check for staging. 
          if( (configPage10.stagingEnabled == true) && (currentStatus.PW3 > 0) )
          {
            PWdivTimerPerDegree = div(currentStatus.PW3, timePerDegree).quot; //Need to redo this for PW3 as it will be dramatically different to PW1 when staging
            //injector3StartAngle = calculateInjector3StartAngle(PWdivTimerPerDegree);
            injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees);
          }
          break;
        //2 cylinders
        case 2:
          //injector2StartAngle = calculateInjector2StartAngle(PWdivTimerPerDegree);
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees);
          
          if ( (configPage2.injLayout == INJ_SEQUENTIAL) && (configPage6.fuelTrimEnabled > 0) )
            {
              unsigned long pw1percent = 100 + (byte)get3DTableValue(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
              unsigned long pw2percent = 100 + (byte)get3DTableValue(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
              
              if (pw1percent != 100) { currentStatus.PW1 = (pw1percent * currentStatus.PW1) / 100; }
              if (pw2percent != 100) { currentStatus.PW2 = (pw2percent * currentStatus.PW2) / 100; }
            }
          else if( (configPage10.stagingEnabled == true) && (currentStatus.PW3 > 0) )
          {
            PWdivTimerPerDegree = div(currentStatus.PW3, timePerDegree).quot; //Need to redo this for PW3 as it will be dramatically different to PW1 when staging
            //injector3StartAngle = calculateInjector3StartAngle(PWdivTimerPerDegree);
            injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees);

            injector4StartAngle = injector3StartAngle + (CRANK_ANGLE_MAX_INJ / 2); //Phase this either 180 or 360 degrees out from inj3 (In reality this will always be 180 as you can't have sequential and staged currently)
            if(injector4StartAngle > (uint16_t)CRANK_ANGLE_MAX_INJ) { injector4StartAngle -= CRANK_ANGLE_MAX_INJ; }
          }
          break;
        //3 cylinders
        case 3:
          //injector2StartAngle = calculateInjector2StartAngle(PWdivTimerPerDegree);
          //injector3StartAngle = calculateInjector3StartAngle(PWdivTimerPerDegree);
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees);
          injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees);
          
          if ( (configPage2.injLayout == INJ_SEQUENTIAL) && (configPage6.fuelTrimEnabled > 0) )
            {
              unsigned long pw1percent = 100 + (byte)get3DTableValue(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
              unsigned long pw2percent = 100 + (byte)get3DTableValue(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
              unsigned long pw3percent = 100 + (byte)get3DTableValue(&trim3Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;

              if (pw1percent != 100) { currentStatus.PW1 = (pw1percent * currentStatus.PW1) / 100; }
              if (pw2percent != 100) { currentStatus.PW2 = (pw2percent * currentStatus.PW2) / 100; }
              if (pw3percent != 100) { currentStatus.PW3 = (pw3percent * currentStatus.PW3) / 100; }
            }
          break;
        //4 cylinders
        case 4:
          //injector2StartAngle = calculateInjector2StartAngle(PWdivTimerPerDegree);
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees);

          if(configPage2.injLayout == INJ_SEQUENTIAL)
          {
            //injector3StartAngle = calculateInjector3StartAngle(PWdivTimerPerDegree);
            //injector4StartAngle = calculateInjector4StartAngle(PWdivTimerPerDegree);
            injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees);
            injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees);

            if(configPage6.fuelTrimEnabled > 0)
            {
              unsigned long pw1percent = 100 + (byte)get3DTableValue(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
              unsigned long pw2percent = 100 + (byte)get3DTableValue(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
              unsigned long pw3percent = 100 + (byte)get3DTableValue(&trim3Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
              unsigned long pw4percent = 100 + (byte)get3DTableValue(&trim4Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;

              if (pw1percent != 100) { currentStatus.PW1 = (pw1percent * currentStatus.PW1) / 100; }
              if (pw2percent != 100) { currentStatus.PW2 = (pw2percent * currentStatus.PW2) / 100; }
              if (pw3percent != 100) { currentStatus.PW3 = (pw3percent * currentStatus.PW3) / 100; }
              if (pw4percent != 100) { currentStatus.PW4 = (pw4percent * currentStatus.PW4) / 100; }
            }
          }
          else if( (configPage10.stagingEnabled == true) && (currentStatus.PW3 > 0) )
          {
            PWdivTimerPerDegree = div(currentStatus.PW3, timePerDegree).quot; //Need to redo this for PW3 as it will be dramatically different to PW1 when staging
            //injector3StartAngle = calculateInjector3StartAngle(PWdivTimerPerDegree);
            injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees);

            injector4StartAngle = injector3StartAngle + (CRANK_ANGLE_MAX_INJ / 2); //Phase this either 180 or 360 degrees out from inj3 (In reality this will always be 180 as you can't have sequential and staged currently)
            if(injector4StartAngle > (uint16_t)CRANK_ANGLE_MAX_INJ) { injector4StartAngle -= CRANK_ANGLE_MAX_INJ; }
          }
          break;
        //5 cylinders
        case 5:
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees);
          injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees);
          injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees);
          #if INJ_CHANNELS >= 5
            injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel5InjDegrees);
          #endif
          break;
        //6 cylinders
        case 6:
          //injector2StartAngle = calculateInjector2StartAngle(PWdivTimerPerDegree);
          //injector3StartAngle = calculateInjector3StartAngle(PWdivTimerPerDegree);

          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees);
          injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees);
          
          #if INJ_CHANNELS >= 6
            if(configPage2.injLayout == INJ_SEQUENTIAL)
            {
              injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees);
              injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel5InjDegrees);
              injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel6InjDegrees);

              if(configPage6.fuelTrimEnabled > 0)
              {
                unsigned long pw1percent = 100 + (byte)get3DTableValue(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
                unsigned long pw2percent = 100 + (byte)get3DTableValue(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
                unsigned long pw3percent = 100 + (byte)get3DTableValue(&trim3Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
                unsigned long pw4percent = 100 + (byte)get3DTableValue(&trim4Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
                unsigned long pw5percent = 100 + (byte)get3DTableValue(&trim5Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
                unsigned long pw6percent = 100 + (byte)get3DTableValue(&trim6Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;

                if (pw1percent != 100) { currentStatus.PW1 = (pw1percent * currentStatus.PW1) / 100; }
                if (pw2percent != 100) { currentStatus.PW2 = (pw2percent * currentStatus.PW2) / 100; }
                if (pw3percent != 100) { currentStatus.PW3 = (pw3percent * currentStatus.PW3) / 100; }
                if (pw4percent != 100) { currentStatus.PW4 = (pw4percent * currentStatus.PW4) / 100; }
                if (pw5percent != 100) { currentStatus.PW5 = (pw5percent * currentStatus.PW5) / 100; }
                if (pw6percent != 100) { currentStatus.PW6 = (pw6percent * currentStatus.PW6) / 100; }
              }
            }
          #endif
          break;
        //8 cylinders
        case 8:
        /*
          injector2StartAngle = calculateInjector2StartAngle(PWdivTimerPerDegree);
          injector3StartAngle = calculateInjector3StartAngle(PWdivTimerPerDegree);
          injector4StartAngle = calculateInjector4StartAngle(PWdivTimerPerDegree);
          */

          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees);
          injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees);
          injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees);

          #if INJ_CHANNELS >= 8
            if(configPage2.injLayout == INJ_SEQUENTIAL)
            {
              injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel5InjDegrees);
              injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel6InjDegrees);
              injector7StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel7InjDegrees);
              injector8StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel8InjDegrees);

              if(configPage6.fuelTrimEnabled > 0)
              {
                unsigned long pw1percent = 100 + (byte)get3DTableValue(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
                unsigned long pw2percent = 100 + (byte)get3DTableValue(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
                unsigned long pw3percent = 100 + (byte)get3DTableValue(&trim3Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
                unsigned long pw4percent = 100 + (byte)get3DTableValue(&trim4Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
                unsigned long pw5percent = 100 + (byte)get3DTableValue(&trim5Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
                unsigned long pw6percent = 100 + (byte)get3DTableValue(&trim6Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
                unsigned long pw7percent = 100 + (byte)get3DTableValue(&trim7Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;
                unsigned long pw8percent = 100 + (byte)get3DTableValue(&trim8Table, currentStatus.fuelLoad, currentStatus.RPM) - OFFSET_FUELTRIM;

                if (pw1percent != 100) { currentStatus.PW1 = (pw1percent * currentStatus.PW1) / 100; }
                if (pw2percent != 100) { currentStatus.PW2 = (pw2percent * currentStatus.PW2) / 100; }
                if (pw3percent != 100) { currentStatus.PW3 = (pw3percent * currentStatus.PW3) / 100; }
                if (pw4percent != 100) { currentStatus.PW4 = (pw4percent * currentStatus.PW4) / 100; }
                if (pw5percent != 100) { currentStatus.PW5 = (pw5percent * currentStatus.PW5) / 100; }
                if (pw6percent != 100) { currentStatus.PW6 = (pw6percent * currentStatus.PW6) / 100; }
                if (pw7percent != 100) { currentStatus.PW7 = (pw7percent * currentStatus.PW7) / 100; }
                if (pw8percent != 100) { currentStatus.PW8 = (pw8percent * currentStatus.PW8) / 100; }
              }
            }
          #endif
          break;

        //Will hit the default case on 1 cylinder or >8 cylinders. Do nothing in these cases
        default:
          break;
      }

      //***********************************************************************************************
      //| BEGIN IGNITION CALCULATIONS

      //Set dwell
      //Dwell is stored as ms * 10. ie Dwell of 4.3ms would be 43 in configPage4. This number therefore needs to be multiplied by 100 to get dwell in uS
      if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) ) {
        currentStatus.dwell =  (configPage4.dwellCrank * 100); //use cranking dwell
      }
      else 
      {
        if ( configPage2.useDwellMap == true )
        {
          currentStatus.dwell = (get3DTableValue(&dwellTable, currentStatus.ignLoad, currentStatus.RPM) * 100); //use running dwell from map
        }
        else
        {
          currentStatus.dwell =  (configPage4.dwellRun * 100); //use fixed running dwell
        }
      }
      currentStatus.dwell = correctionsDwell(currentStatus.dwell);

      int dwellAngle = timeToAngle(currentStatus.dwell, CRANKMATH_METHOD_INTERVAL_REV); //Convert the dwell time to dwell angle based on the current engine speed

      calculateIgnitionAngles(dwellAngle);

      //If ignition timing is being tracked per tooth, perform the calcs to get the end teeth
      //This only needs to be run if the advance figure has changed, otherwise the end teeth will still be the same
      //if( (configPage2.perToothIgn == true) && (lastToothCalcAdvance != currentStatus.advance) ) { triggerSetEndTeeth(); }
      if( (configPage2.perToothIgn == true) ) { triggerSetEndTeeth(); }

      //***********************************************************************************************
      //| BEGIN FUEL SCHEDULES
      //Finally calculate the time (uS) until we reach the firing angles and set the schedules
      //We only need to set the shcedule if we're BEFORE the open angle
      //This may potentially be called a number of times as we get closer and closer to the opening time

      //Determine the current crank angle
      int crankAngle = getCrankAngle();
      while(crankAngle > CRANK_ANGLE_MAX_INJ ) { crankAngle = crankAngle - CRANK_ANGLE_MAX_INJ; } //Continue reducing the crank angle by the max injection amount until it's below the required limit. This will usually only run (at most) once, but in cases where there is sequential ignition and more than 2 squirts per cycle, it may run up to 4 times. 

      // if(Serial && false)
      // {
      //   if(ignition1StartAngle > crankAngle)
      //   {
      //     noInterrupts();
      //     Serial.print("Time2LastTooth:"); Serial.println(micros()-toothLastToothTime);
      //     Serial.print("elapsedTime:"); Serial.println(elapsedTime);
      //     Serial.print("CurAngle:"); Serial.println(crankAngle);
      //     Serial.print("RPM:"); Serial.println(currentStatus.RPM);
      //     Serial.print("Tooth:"); Serial.println(toothCurrentCount);
      //     Serial.print("timePerDegree:"); Serial.println(timePerDegree);
      //     Serial.print("IGN1Angle:"); Serial.println(ignition1StartAngle);
      //     Serial.print("TimeToIGN1:"); Serial.println(angleToTime((ignition1StartAngle - crankAngle), CRANKMATH_METHOD_INTERVAL_REV));
      //     interrupts();
      //   }
      // }
      
      //Check for any of the engine protections or rev limiters being turned on
      if(checkEngineProtect() || currentStatus.launchingHard || currentStatus.flatShiftingHard)
      {
        if( (currentStatus.RPMdiv100 > configPage4.engineProtectMaxRPM) || currentStatus.launchingHard || currentStatus.flatShiftingHard) //Ugly, but the launch/flat shift check needs to be here as well to prevent these limiters not happening when under the the Engine Protection min rpm
        {
          if( (configPage2.hardCutType == HARD_CUT_FULL) || (configPage6.engineProtectType == PROTECT_CUT_FUEL) ) 
          { 
            //Full hard cut turns outputs off completely. Note that hard cut is ALWAYS used on fuel cut only. 
            switch(configPage6.engineProtectType)
            {
              case PROTECT_CUT_OFF:
                ignitionOn = true;
                fuelOn = true;
                currentStatus.engineProtectStatus = 0;
                break;
              case PROTECT_CUT_IGN:
                ignitionOn = false;
                break;
              case PROTECT_CUT_FUEL:
                fuelOn = false;
                break;
              case PROTECT_CUT_BOTH:
                ignitionOn = false;
                fuelOn = false;
                break;
              default:
                ignitionOn = false;
                fuelOn = false;
                break;
            }
          }
          else 
          { 
            //if(rollingCutCounter >= 2) //Vary this number to change the intensity of the roll. The higher the number, the closer is it to full cut
            if(rollingCutLastRev == 0) { rollingCutLastRev = currentStatus.startRevolutions; } //
            if (currentStatus.startRevolutions > (rollingCutLastRev+1) )
            { 
              //Rolls through each of the active ignition channels based on how many revolutions have taken place
              //curRollingCut = ( (currentStatus.startRevolutions / 2) % maxIgnOutputs) + 1;
              curRollingCut = 0;
              rollingCutCounter += 1;
              if(rollingCutCounter > (maxIgnOutputs-1)) { rollingCutCounter = 0; }
              BIT_SET(curRollingCut, rollingCutCounter);

              ignitionOn = true;
              rollingCutLastRev = currentStatus.startRevolutions;
              //curRollingCut = 0;
            }
          } //Hard/Rolling cut check
        } //RPM Check
        else { currentStatus.engineProtectStatus = 0; } //Force all engine protection flags to be off as we're below the minimum RPM
      } //Protection active check
      else { curRollingCut = 0; } //Disables the rolling hard cut

#if INJ_CHANNELS >= 1
      if (fuelOn && !BIT_CHECK(currentStatus.status1, BIT_STATUS1_BOOSTCUT))
      {
        if(currentStatus.PW1 >= inj_opentime_uS)
        {
          if ( (injector1StartAngle <= crankAngle) && (fuelSchedule1.Status == RUNNING) ) { injector1StartAngle += CRANK_ANGLE_MAX_INJ; }
          if (injector1StartAngle > crankAngle)
          {
            setFuelSchedule1(
                      ((injector1StartAngle - crankAngle) * (unsigned long)timePerDegree),
                      (unsigned long)currentStatus.PW1
                      );
          }
        }
#endif

        /*-----------------------------------------------------------------------------------------
        | A Note on tempCrankAngle and tempStartAngle:
        |   The use of tempCrankAngle/tempStartAngle is described below. It is then used in the same way for channels 2, 3 and 4+ on both injectors and ignition
        |   Essentially, these 2 variables are used to realign the current crank angle and the desired start angle around 0 degrees for the given cylinder/output
        |   Eg: If cylinder 2 TDC is 180 degrees after cylinder 1 (Eg a standard 4 cylidner engine), then tempCrankAngle is 180* less than the current crank angle and
        |       tempStartAngle is the desired open time less 180*. Thus the cylinder is being treated relative to its own TDC, regardless of its offset
        |
        |   This is done to avoid problems with very short of very long times until tempStartAngle.
        |------------------------------------------------------------------------------------------
        */
#if INJ_CHANNELS >= 2
        if( (channel2InjEnabled) && (currentStatus.PW2 >= inj_opentime_uS) )
        {
          tempCrankAngle = crankAngle - channel2InjDegrees;
          if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_INJ; }
          tempStartAngle = injector2StartAngle - channel2InjDegrees;
          if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
          if ( (tempStartAngle <= tempCrankAngle) && (fuelSchedule2.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
          if ( tempStartAngle > tempCrankAngle )
          {
            setFuelSchedule2(
                      ((tempStartAngle - tempCrankAngle) * (unsigned long)timePerDegree),
                      (unsigned long)currentStatus.PW2
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 3
        if( (channel3InjEnabled) && (currentStatus.PW3 >= inj_opentime_uS) )
        {
          tempCrankAngle = crankAngle - channel3InjDegrees;
          if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_INJ; }
          tempStartAngle = injector3StartAngle - channel3InjDegrees;
          if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
          if ( (tempStartAngle <= tempCrankAngle) && (fuelSchedule3.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
          if ( tempStartAngle > tempCrankAngle )
          {
            setFuelSchedule3(
                      ((tempStartAngle - tempCrankAngle) * (unsigned long)timePerDegree),
                      (unsigned long)currentStatus.PW3
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 4
        if( (channel4InjEnabled) && (currentStatus.PW4 >= inj_opentime_uS) )
        {
          tempCrankAngle = crankAngle - channel4InjDegrees;
          if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_INJ; }
          tempStartAngle = injector4StartAngle - channel4InjDegrees;
          if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
          if ( (tempStartAngle <= tempCrankAngle) && (fuelSchedule4.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
          if ( tempStartAngle > tempCrankAngle )
          {
            setFuelSchedule4(
                      ((tempStartAngle - tempCrankAngle) * (unsigned long)timePerDegree),
                      (unsigned long)currentStatus.PW4
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 5
        if( (channel5InjEnabled) && (currentStatus.PW5 >= inj_opentime_uS) )
        {
          tempCrankAngle = crankAngle - channel5InjDegrees;
          if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_INJ; }
          tempStartAngle = injector5StartAngle - channel5InjDegrees;
          if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
          if ( (tempStartAngle <= tempCrankAngle) && (fuelSchedule5.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
          if ( tempStartAngle > tempCrankAngle )
          {
            //Note the hacky use of fuel schedule 3 below
            /*
            setFuelSchedule3(openInjector3and5,
                      ((unsigned long)(tempStartAngle - tempCrankAngle) * (unsigned long)timePerDegree),
                      (unsigned long)currentStatus.PW1,
                      closeInjector3and5
                    );*/
            
            setFuelSchedule5(
                      ((tempStartAngle - tempCrankAngle) * (unsigned long)timePerDegree),
                      (unsigned long)currentStatus.PW5
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 6
        if( (channel6InjEnabled) && (currentStatus.PW6 >= inj_opentime_uS) )
        {
          tempCrankAngle = crankAngle - channel6InjDegrees;
          if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_INJ; }
          tempStartAngle = injector6StartAngle - channel6InjDegrees;
          if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
          if ( (tempStartAngle <= tempCrankAngle) && (fuelSchedule6.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
          if ( tempStartAngle > tempCrankAngle )
          {
            setFuelSchedule6(
                      ((tempStartAngle - tempCrankAngle) * (unsigned long)timePerDegree),
                      (unsigned long)currentStatus.PW6
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 7
        if( (channel7InjEnabled) && (currentStatus.PW7 >= inj_opentime_uS) )
        {
          tempCrankAngle = crankAngle - channel7InjDegrees;
          if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_INJ; }
          tempStartAngle = injector7StartAngle - channel7InjDegrees;
          if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
          if ( (tempStartAngle <= tempCrankAngle) && (fuelSchedule7.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
          if ( tempStartAngle > tempCrankAngle )
          {
            setFuelSchedule7(
                      ((tempStartAngle - tempCrankAngle) * (unsigned long)timePerDegree),
                      (unsigned long)currentStatus.PW7
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 8
        if( (channel8InjEnabled) && (currentStatus.PW8 >= inj_opentime_uS) )
        {
          tempCrankAngle = crankAngle - channel8InjDegrees;
          if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_INJ; }
          tempStartAngle = injector8StartAngle - channel8InjDegrees;
          if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
          if ( (tempStartAngle <= tempCrankAngle) && (fuelSchedule8.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
          if ( tempStartAngle > tempCrankAngle )
          {
            setFuelSchedule8(
                      ((tempStartAngle - tempCrankAngle) * (unsigned long)timePerDegree),
                      (unsigned long)currentStatus.PW8
                      );
          }
        }
#endif
      }
      //***********************************************************************************************
      //| BEGIN IGNITION SCHEDULES
      //Same as above, except for ignition

      //fixedCrankingOverride is used to extend the dwell during cranking so that the decoder can trigger the spark upon seeing a certain tooth. Currently only available on the basic distributor and 4g63 decoders.
      if ( configPage4.ignCranklock && BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && (decoderHasFixedCrankingTiming == true) )
      {
        fixedCrankingOverride = currentStatus.dwell * 3;
        //This is a safety step to prevent the ignition start time occuring AFTER the target tooth pulse has already occcured. It simply moves the start time forward a little, which is compensated for by the increase in the dwell time
        if(currentStatus.RPM < 250)
        {
          ignition1StartAngle -= 5;
          ignition2StartAngle -= 5;
          ignition3StartAngle -= 5;
          ignition4StartAngle -= 5;
          ignition5StartAngle -= 5;
          ignition6StartAngle -= 5;
          ignition7StartAngle -= 5;
          ignition8StartAngle -= 5;
        }
      }
      else { fixedCrankingOverride = 0; }

      if(ignitionOn)
      {
        //Refresh the current crank angle info
        //ignition1StartAngle = 335;
        crankAngle = getCrankAngle(); //Refresh with the latest crank angle
        while (crankAngle > CRANK_ANGLE_MAX_IGN ) { crankAngle -= CRANK_ANGLE_MAX_IGN; }

#if IGN_CHANNELS >= 1
        if ( (ignition1StartAngle <= crankAngle) && (ignitionSchedule1.Status == RUNNING) ) { ignition1StartAngle += CRANK_ANGLE_MAX_IGN; }
        //if ( (ignition1StartAngle > crankAngle) && (curRollingCut != 1) )
        if ( (ignition1StartAngle > crankAngle) && (!BIT_CHECK(curRollingCut, IGN1_CMD_BIT)) )
        {
          
          setIgnitionSchedule1(ign1StartFunction,
                    //((unsigned long)(ignition1StartAngle - crankAngle) * (unsigned long)timePerDegree),
                    angleToTime((ignition1StartAngle - crankAngle), CRANKMATH_METHOD_INTERVAL_REV),
                    currentStatus.dwell + fixedCrankingOverride, //((unsigned long)((unsigned long)currentStatus.dwell* currentStatus.RPM) / newRPM) + fixedCrankingOverride,
                    ign1EndFunction
                    );
        }
#endif

#if defined(USE_IGN_REFRESH)
        if( (ignitionSchedule1.Status == RUNNING) && (ignition1EndAngle > crankAngle) && (configPage4.StgCycles == 0) && (configPage2.perToothIgn != true) )
        {
          unsigned long uSToEnd = 0;

          crankAngle = getCrankAngle(); //Refresh with the latest crank angle
          if (crankAngle > CRANK_ANGLE_MAX_IGN ) { crankAngle -= 360; }
          
          //ONLY ONE OF THE BELOW SHOULD BE USED (PROBABLY THE FIRST):
          //*********
          if(ignition1EndAngle > crankAngle) { uSToEnd = fastDegreesToUS( (ignition1EndAngle - crankAngle) ); }
          else { uSToEnd = fastDegreesToUS( (360 + ignition1EndAngle - crankAngle) ); }
          //*********
          //uSToEnd = ((ignition1EndAngle - crankAngle) * (toothLastToothTime - toothLastMinusOneToothTime)) / triggerToothAngle;
          //*********

          refreshIgnitionSchedule1( uSToEnd + fixedCrankingOverride );
        }
  #endif
        
#if IGN_CHANNELS >= 2
        if (maxIgnOutputs >= 2)
        {
            tempCrankAngle = crankAngle - channel2IgnDegrees;
            if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_IGN; }
            tempStartAngle = ignition2StartAngle - channel2IgnDegrees;
            if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }

            unsigned long ignition2StartTime = 0;
            if ( (tempStartAngle <= tempCrankAngle) && (ignitionSchedule2.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }
            if(tempStartAngle > tempCrankAngle) { ignition2StartTime = angleToTime((tempStartAngle - tempCrankAngle), CRANKMATH_METHOD_INTERVAL_REV); }
            //else if (tempStartAngle < tempCrankAngle) { ignition2StartTime = ((long)(360 - tempCrankAngle + tempStartAngle) * (long)timePerDegree); }
            else { ignition2StartTime = 0; }

            if ( (ignition2StartTime > 0) && (!BIT_CHECK(curRollingCut, IGN2_CMD_BIT)) )
            {
              setIgnitionSchedule2(ign2StartFunction,
                        ignition2StartTime,
                        currentStatus.dwell + fixedCrankingOverride,
                        ign2EndFunction
                        );
            }
        }
#endif

#if IGN_CHANNELS >= 3
        if (maxIgnOutputs >= 3)
        {
            tempCrankAngle = crankAngle - channel3IgnDegrees;
            if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_IGN; }
            tempStartAngle = ignition3StartAngle - channel3IgnDegrees;
            if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }

            unsigned long ignition3StartTime = 0;
            if ( (tempStartAngle <= tempCrankAngle) && (ignitionSchedule3.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }
            if(tempStartAngle > tempCrankAngle) { ignition3StartTime = angleToTime((tempStartAngle - tempCrankAngle), CRANKMATH_METHOD_INTERVAL_REV); }
            //else if (tempStartAngle < tempCrankAngle) { ignition3StartTime = ((long)(360 - tempCrankAngle + tempStartAngle) * (long)timePerDegree); }
            else { ignition3StartTime = 0; }

            if ( (ignition3StartTime > 0) && (!BIT_CHECK(curRollingCut, IGN3_CMD_BIT)) )
            {
              setIgnitionSchedule3(ign3StartFunction,
                        ignition3StartTime,
                        currentStatus.dwell + fixedCrankingOverride,
                        ign3EndFunction
                        );
            }
        }
#endif

#if IGN_CHANNELS >= 4
        if (maxIgnOutputs >= 4)
        {
            tempCrankAngle = crankAngle - channel4IgnDegrees;
            if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_IGN; }
            tempStartAngle = ignition4StartAngle - channel4IgnDegrees;
            if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }

            unsigned long ignition4StartTime = 0;
            if ( (tempStartAngle <= tempCrankAngle) && (ignitionSchedule4.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }
            if(tempStartAngle > tempCrankAngle) { ignition4StartTime = angleToTime((tempStartAngle - tempCrankAngle), CRANKMATH_METHOD_INTERVAL_REV); }
            //else if (tempStartAngle < tempCrankAngle) { ignition4StartTime = ((long)(360 - tempCrankAngle + tempStartAngle) * (long)timePerDegree); }
            else { ignition4StartTime = 0; }

            if ( (ignition4StartTime > 0) && (!BIT_CHECK(curRollingCut, IGN4_CMD_BIT)) )
            {
              setIgnitionSchedule4(ign4StartFunction,
                        ignition4StartTime,
                        currentStatus.dwell + fixedCrankingOverride,
                        ign4EndFunction
                        );
            }
        }
#endif

#if IGN_CHANNELS >= 5
        if (maxIgnOutputs >= 5)
        {
            tempCrankAngle = crankAngle - channel5IgnDegrees;
            if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_IGN; }
            tempStartAngle = ignition5StartAngle - channel5IgnDegrees;
            if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }

            unsigned long ignition5StartTime = 0;
            if ( (tempStartAngle <= tempCrankAngle) && (ignitionSchedule5.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }
            if(tempStartAngle > tempCrankAngle) { ignition5StartTime = angleToTime((tempStartAngle - tempCrankAngle), CRANKMATH_METHOD_INTERVAL_REV); }
            //else if (tempStartAngle < tempCrankAngle) { ignition5StartTime = ((long)(360 - tempCrankAngle + tempStartAngle) * (long)timePerDegree); }
            else { ignition5StartTime = 0; }

            if ( (ignition5StartTime > 0) && (!BIT_CHECK(curRollingCut, IGN5_CMD_BIT)) )
            {
              setIgnitionSchedule5(ign5StartFunction,
                        ignition5StartTime,
                        currentStatus.dwell + fixedCrankingOverride,
                        ign5EndFunction
                        );
            }
        }
#endif

#if IGN_CHANNELS >= 6
        if (maxIgnOutputs >= 6)
        {
            tempCrankAngle = crankAngle - channel6IgnDegrees;
            if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_IGN; }
            tempStartAngle = ignition6StartAngle - channel6IgnDegrees;
            if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }

            unsigned long ignition6StartTime = 0;
            if ( (tempStartAngle <= tempCrankAngle) && (ignitionSchedule6.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }
            if(tempStartAngle > tempCrankAngle) { ignition6StartTime = angleToTime((tempStartAngle - tempCrankAngle), CRANKMATH_METHOD_INTERVAL_REV); }
            //else if (tempStartAngle < tempCrankAngle) { ignition6StartTime = ((long)(360 - tempCrankAngle + tempStartAngle) * (long)timePerDegree); }
            else { ignition6StartTime = 0; }

            if ( (ignition6StartTime > 0) && (!BIT_CHECK(curRollingCut, IGN6_CMD_BIT)) )
            {
              setIgnitionSchedule6(ign6StartFunction,
                        ignition6StartTime,
                        currentStatus.dwell + fixedCrankingOverride,
                        ign6EndFunction
                        );
            }
        }
#endif

#if IGN_CHANNELS >= 7
        if (maxIgnOutputs >= 7)
        {
            tempCrankAngle = crankAngle - channel7IgnDegrees;
            if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_IGN; }
            tempStartAngle = ignition7StartAngle - channel7IgnDegrees;
            if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }

            unsigned long ignition7StartTime = 0;
            if ( (tempStartAngle <= tempCrankAngle) && (ignitionSchedule7.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }
            if(tempStartAngle > tempCrankAngle) { ignition7StartTime = angleToTime((tempStartAngle - tempCrankAngle), CRANKMATH_METHOD_INTERVAL_REV); }
            //else if (tempStartAngle < tempCrankAngle) { ignition7StartTime = ((long)(360 - tempCrankAngle + tempStartAngle) * (long)timePerDegree); }
            else { ignition7StartTime = 0; }

            if ( (ignition7StartTime > 0) && (!BIT_CHECK(curRollingCut, IGN7_CMD_BIT)) )
            {
              setIgnitionSchedule7(ign7StartFunction,
                        ignition7StartTime,
                        currentStatus.dwell + fixedCrankingOverride,
                        ign7EndFunction
                        );
            }
        }
#endif

#if IGN_CHANNELS >= 8
        if (maxIgnOutputs >= 8)
        {
            tempCrankAngle = crankAngle - channel8IgnDegrees;
            if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_IGN; }
            tempStartAngle = ignition8StartAngle - channel8IgnDegrees;
            if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }

            unsigned long ignition8StartTime = 0;
            if ( (tempStartAngle <= tempCrankAngle) && (ignitionSchedule8.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }
            if(tempStartAngle > tempCrankAngle) { ignition8StartTime = angleToTime((tempStartAngle - tempCrankAngle), CRANKMATH_METHOD_INTERVAL_REV); }
            //else if (tempStartAngle < tempCrankAngle) { ignition8StartTime = ((long)(360 - tempCrankAngle + tempStartAngle) * (long)timePerDegree); }
            else { ignition8StartTime = 0; }

            if ( (ignition8StartTime > 0) && (!BIT_CHECK(curRollingCut, IGN8_CMD_BIT)) )
            {
              setIgnitionSchedule8(ign8StartFunction,
                        ignition8StartTime,
                        currentStatus.dwell + fixedCrankingOverride,
                        ign8EndFunction
                        );
            }
        }
#endif

      } //Ignition schedules on

      if ( (!BIT_CHECK(currentStatus.status3, BIT_STATUS3_RESET_PREVENT)) && (resetControl == RESET_CONTROL_PREVENT_WHEN_RUNNING) ) 
      {
        //Reset prevention is supposed to be on while the engine is running but isn't. Fix that.
        digitalWrite(pinResetControl, HIGH);
        BIT_SET(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);
      }
    } //Has sync and RPM
    else if ( (BIT_CHECK(currentStatus.status3, BIT_STATUS3_RESET_PREVENT) > 0) && (resetControl == RESET_CONTROL_PREVENT_WHEN_RUNNING) )
    {
      digitalWrite(pinResetControl, LOW);
      BIT_CLEAR(currentStatus.status3, BIT_STATUS3_RESET_PREVENT);
    }
} //loop()
#endif //Unit test guard

/**
 * @brief This function calculates the required pulsewidth time (in us) given the current system state
 * 
 * @param REQ_FUEL The required fuel value in uS, as calculated by TunerStudio
 * @param VE Lookup from the main fuel table. This can either have been MAP or TPS based, depending on the algorithm used
 * @param MAP In KPa, read from the sensor (This is used when performing a multiply of the map only. It is applicable in both Speed density and Alpha-N)
 * @param corrections Sum of Enrichment factors (Cold start, acceleration). This is a multiplication factor (Eg to add 10%, this should be 110)
 * @param injOpen Injector opening time. The time the injector take to open minus the time it takes to close (Both in uS)
 * @return uint16_t The injector pulse width in uS
 */
uint16_t PW(int REQ_FUEL, byte VE, long MAP, uint16_t corrections, int injOpen)
{
  //Standard float version of the calculation
  //return (REQ_FUEL * (float)(VE/100.0) * (float)(MAP/100.0) * (float)(TPS/100.0) * (float)(corrections/100.0) + injOpen);
  //Note: The MAP and TPS portions are currently disabled, we use VE and corrections only
  uint16_t iVE, iCorrections;
  uint16_t iMAP = 100;
  uint16_t iAFR = 147;

  //100% float free version, does sacrifice a little bit of accuracy, but not much.

  //If corrections are huge, use less bitshift to avoid overflow. Sacrifices a bit more accuracy (basically only during very cold temp cranking)
  byte bitShift = 7;
  if (corrections > 511 ) { bitShift = 6; }
  if (corrections > 1023) { bitShift = 5; }
  
  iVE = ((unsigned int)VE << 7) / 100;
  //iVE = divu100(((unsigned int)VE << 7));

  //Check whether either of the multiply MAP modes is turned on
  if ( configPage2.multiplyMAP == MULTIPLY_MAP_MODE_100) { iMAP = ((unsigned int)MAP << 7) / 100; }
  else if( configPage2.multiplyMAP == MULTIPLY_MAP_MODE_BARO) { iMAP = ((unsigned int)MAP << 7) / currentStatus.baro; }
  
  if ( (configPage2.includeAFR == true) && (configPage6.egoType == 2) && (currentStatus.runSecs > configPage6.ego_sdelay) ) {
    iAFR = ((unsigned int)currentStatus.O2 << 7) / currentStatus.afrTarget;  //Include AFR (vs target) if enabled
  }
  if ( (configPage2.incorporateAFR == true) && (configPage2.includeAFR == false) ) {
    iAFR = ((unsigned int)configPage2.stoich << 7) / currentStatus.afrTarget;  //Incorporate stoich vs target AFR, if enabled.
  }
  iCorrections = (corrections << bitShift) / 100;
  //iCorrections = divu100((corrections << bitShift));


  unsigned long intermediate = ((uint32_t)REQ_FUEL * (uint32_t)iVE) >> 7; //Need to use an intermediate value to avoid overflowing the long
  if ( configPage2.multiplyMAP > 0 ) { intermediate = (intermediate * (unsigned long)iMAP) >> 7; }
  
  if ( (configPage2.includeAFR == true) && (configPage6.egoType == 2) && (currentStatus.runSecs > configPage6.ego_sdelay) ) {
    //EGO type must be set to wideband and the AFR warmup time must've elapsed for this to be used
    intermediate = (intermediate * (unsigned long)iAFR) >> 7;  
  }
  if ( (configPage2.incorporateAFR == true) && (configPage2.includeAFR == false) ) {
    intermediate = (intermediate * (unsigned long)iAFR) >> 7;
  }
  
  intermediate = (intermediate * (unsigned long)iCorrections) >> bitShift;
  if (intermediate != 0)
  {
    //If intermeditate is not 0, we need to add the opening time (0 typically indicates that one of the full fuel cuts is active)
    intermediate += injOpen; //Add the injector opening time
    //AE Adds % of req_fuel
    if ( configPage2.aeApplyMode == AE_MODE_ADDER )
    {
      intermediate += ( ((unsigned long)REQ_FUEL) * (currentStatus.AEamount - 100) ) / 100;
    }
    if ( intermediate > 65535)
    {
      intermediate = 65535;  //Make sure this won't overflow when we convert to uInt. This means the maximum pulsewidth possible is 65.535mS
    }
  }
  return (unsigned int)(intermediate);
}

/** Lookup the current VE value from the primary 3D fuel map.
 * The Y axis value used for this lookup varies based on the fuel algorithm selected (speed density, alpha-n etc).
 * 
 * @return byte The current VE value
 */
byte getVE1()
{
  byte tempVE = 100;
  if (configPage2.fuelAlgorithm == LOAD_SOURCE_MAP) //Check which fuelling algorithm is being used
  {
    //Speed Density
    currentStatus.fuelLoad = currentStatus.MAP;
  }
  else if (configPage2.fuelAlgorithm == LOAD_SOURCE_TPS)
  {
    //Alpha-N
    currentStatus.fuelLoad = currentStatus.TPS * 2;
  }
  else if (configPage2.fuelAlgorithm == LOAD_SOURCE_IMAPEMAP)
  {
    //IMAP / EMAP
    currentStatus.fuelLoad = (currentStatus.MAP * 100) / currentStatus.EMAP;
  }
  else { currentStatus.fuelLoad = currentStatus.MAP; } //Fallback position
  tempVE = get3DTableValue(&fuelTable, currentStatus.fuelLoad, currentStatus.RPM); //Perform lookup into fuel map for RPM vs MAP value

  return tempVE;
}

/** Lookup the ignition advance from 3D ignition table.
 * The values used to look this up will be RPM and whatever load source the user has configured.
 * 
 * @return byte The current target advance value in degrees
 */
byte getAdvance1()
{
  byte tempAdvance = 0;
  if (configPage2.ignAlgorithm == LOAD_SOURCE_MAP) //Check which fuelling algorithm is being used
  {
    //Speed Density
    currentStatus.ignLoad = currentStatus.MAP;
  }
  else if(configPage2.ignAlgorithm == LOAD_SOURCE_TPS)
  {
    //Alpha-N
    currentStatus.ignLoad = currentStatus.TPS * 2;

  }
  else if (configPage2.fuelAlgorithm == LOAD_SOURCE_IMAPEMAP)
  {
    //IMAP / EMAP
    currentStatus.ignLoad = (currentStatus.MAP * 100) / currentStatus.EMAP;
  }
  tempAdvance = get3DTableValue(&ignitionTable, currentStatus.ignLoad, currentStatus.RPM) - OFFSET_IGNITION; //As above, but for ignition advance
  tempAdvance = correctionsIgn(tempAdvance);

  return tempAdvance;
}

uint16_t calculateInjectorStartAngle(uint16_t PWdivTimerPerDegree, int16_t injChannelDegrees)
{
  uint16_t tempInjectorStartAngle = (currentStatus.injAngle + injChannelDegrees);
  if(tempInjectorStartAngle < PWdivTimerPerDegree) { tempInjectorStartAngle += CRANK_ANGLE_MAX_INJ; }
  tempInjectorStartAngle -= PWdivTimerPerDegree;
  while(tempInjectorStartAngle > (uint16_t)CRANK_ANGLE_MAX_INJ) { tempInjectorStartAngle -= CRANK_ANGLE_MAX_INJ; }

  return tempInjectorStartAngle;
}

void calculateIgnitionAngle1(int dwellAngle)
{
  ignition1EndAngle = CRANK_ANGLE_MAX_IGN - currentStatus.advance;
  if(ignition1EndAngle > CRANK_ANGLE_MAX_IGN) {ignition1EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition1StartAngle = ignition1EndAngle - dwellAngle; // 360 - desired advance angle - number of degrees the dwell will take
  if(ignition1StartAngle < 0) {ignition1StartAngle += CRANK_ANGLE_MAX_IGN;}
}

void calculateIgnitionAngle2(int dwellAngle)
{
  ignition2EndAngle = channel2IgnDegrees - currentStatus.advance;
  if(ignition2EndAngle > CRANK_ANGLE_MAX_IGN) {ignition2EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition2StartAngle = ignition2EndAngle - dwellAngle;
  if(ignition2StartAngle < 0) {ignition2StartAngle += CRANK_ANGLE_MAX_IGN;}
}

void calculateIgnitionAngle3(int dwellAngle)
{
  ignition3EndAngle = channel3IgnDegrees - currentStatus.advance;
  if(ignition3EndAngle > CRANK_ANGLE_MAX_IGN) {ignition3EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition3StartAngle = ignition3EndAngle - dwellAngle;
  if(ignition3StartAngle < 0) {ignition3StartAngle += CRANK_ANGLE_MAX_IGN;}
}

// ignition 3 for rotary
void calculateIgnitionAngle3(int dwellAngle, int rotarySplitDegrees)
{
  ignition3EndAngle = ignition1EndAngle + rotarySplitDegrees;
  ignition3StartAngle = ignition3EndAngle - dwellAngle;
  if(ignition3StartAngle > CRANK_ANGLE_MAX_IGN) {ignition3StartAngle -= CRANK_ANGLE_MAX_IGN;}
  if(ignition3StartAngle < 0) {ignition3StartAngle += CRANK_ANGLE_MAX_IGN;}
}

void calculateIgnitionAngle4(int dwellAngle)
{
  ignition4EndAngle = channel4IgnDegrees - currentStatus.advance;
  if(ignition4EndAngle > CRANK_ANGLE_MAX_IGN) {ignition4EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition4StartAngle = ignition4EndAngle - dwellAngle;
  if(ignition4StartAngle < 0) {ignition4StartAngle += CRANK_ANGLE_MAX_IGN;}
}

// ignition 4 for rotary
void calculateIgnitionAngle4(int dwellAngle, int rotarySplitDegrees)
{
  ignition4EndAngle = ignition2EndAngle + rotarySplitDegrees;
  ignition4StartAngle = ignition4EndAngle - dwellAngle;
  if(ignition4StartAngle > CRANK_ANGLE_MAX_IGN) {ignition4StartAngle -= CRANK_ANGLE_MAX_IGN;}
  if(ignition4StartAngle < 0) {ignition4StartAngle += CRANK_ANGLE_MAX_IGN;}
}

void calculateIgnitionAngle5(int dwellAngle)
{
  ignition5EndAngle = channel5IgnDegrees - currentStatus.advance;
  if(ignition5EndAngle > CRANK_ANGLE_MAX_IGN) {ignition5EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition5StartAngle = ignition5EndAngle - dwellAngle;
  if(ignition5StartAngle < 0) {ignition5StartAngle += CRANK_ANGLE_MAX_IGN;}
}

void calculateIgnitionAngle6(int dwellAngle)
{
  ignition6EndAngle = channel6IgnDegrees - currentStatus.advance;
  if(ignition6EndAngle > CRANK_ANGLE_MAX_IGN) {ignition6EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition6StartAngle = ignition6EndAngle - dwellAngle;
  if(ignition6StartAngle < 0) {ignition6StartAngle += CRANK_ANGLE_MAX_IGN;}
}

void calculateIgnitionAngle7(int dwellAngle)
{
  ignition7EndAngle = channel7IgnDegrees - currentStatus.advance;
  if(ignition7EndAngle > CRANK_ANGLE_MAX_IGN) {ignition7EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition7StartAngle = ignition7EndAngle - dwellAngle;
  if(ignition7StartAngle < 0) {ignition7StartAngle += CRANK_ANGLE_MAX_IGN;}
}

void calculateIgnitionAngle8(int dwellAngle)
{
  ignition8EndAngle = channel8IgnDegrees - currentStatus.advance;
  if(ignition8EndAngle > CRANK_ANGLE_MAX_IGN) {ignition8EndAngle -= CRANK_ANGLE_MAX_IGN;}
  ignition8StartAngle = ignition8EndAngle - dwellAngle;
  if(ignition8StartAngle < 0) {ignition8StartAngle += CRANK_ANGLE_MAX_IGN;}
}
/** Calculate the Ignition angles for all cylinders (based on @ref config2.nCylinders).
 * both start and end angles are calculated for each channel.
 * Also the mode of ignition firing - wasted spark vs. dedicated spark per cyl. - is considered here.
 */
void calculateIgnitionAngles(int dwellAngle)
{
  

  //This test for more cylinders and do the same thing
  switch (configPage2.nCylinders)
  {
    //1 cylinder
    case 1:
      calculateIgnitionAngle1(dwellAngle);
      break;
    //2 cylinders
    case 2:
      calculateIgnitionAngle1(dwellAngle);
      calculateIgnitionAngle2(dwellAngle);
      break;
    //3 cylinders
    case 3:
      calculateIgnitionAngle1(dwellAngle);
      calculateIgnitionAngle2(dwellAngle);
      calculateIgnitionAngle3(dwellAngle);
      break;
    //4 cylinders
    case 4:
      calculateIgnitionAngle1(dwellAngle);
      calculateIgnitionAngle2(dwellAngle);

      #if IGN_CHANNELS >= 4
      if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
      {
        calculateIgnitionAngle3(dwellAngle);
        calculateIgnitionAngle4(dwellAngle);
      }
      else if(configPage4.sparkMode == IGN_MODE_ROTARY)
      {
        byte splitDegrees = 0;
        splitDegrees = table2D_getValue(&rotarySplitTable, currentStatus.ignLoad);

        //The trailing angles are set relative to the leading ones
        calculateIgnitionAngle3(dwellAngle, splitDegrees);
        calculateIgnitionAngle4(dwellAngle, splitDegrees);
      }
      #endif
      break;
    //5 cylinders
    case 5:
      calculateIgnitionAngle1(dwellAngle);
      calculateIgnitionAngle2(dwellAngle);
      calculateIgnitionAngle3(dwellAngle);
      calculateIgnitionAngle4(dwellAngle);
      calculateIgnitionAngle5(dwellAngle);
      break;
    //6 cylinders
    case 6:
      calculateIgnitionAngle1(dwellAngle);
      calculateIgnitionAngle2(dwellAngle);
      calculateIgnitionAngle3(dwellAngle);

      #if IGN_CHANNELS >= 6
      if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
      {
        calculateIgnitionAngle4(dwellAngle);
        calculateIgnitionAngle5(dwellAngle);
        calculateIgnitionAngle6(dwellAngle);
      }
      #endif
      break;
    //8 cylinders
    case 8:
      calculateIgnitionAngle1(dwellAngle);
      calculateIgnitionAngle2(dwellAngle);
      calculateIgnitionAngle3(dwellAngle);
      calculateIgnitionAngle4(dwellAngle);

      #if IGN_CHANNELS >= 8
      if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
      {
        calculateIgnitionAngle5(dwellAngle);
        calculateIgnitionAngle6(dwellAngle);
        calculateIgnitionAngle7(dwellAngle);
        calculateIgnitionAngle8(dwellAngle);
      }
      #endif
      break;

    //Will hit the default case on >8 cylinders. Do nothing in these cases
    default:
      break;
  }
}
