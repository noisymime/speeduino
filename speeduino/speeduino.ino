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
 * Speeduino initialisation and main loop.
 */
#include <stdint.h> //developer.mbed.org/handbook/C-Data-Types
//************************************************
#include "globals.h"
#include "speeduino.h"
#include "scheduler.h"
#include "comms.h"
#include "comms_legacy.h"
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
#include "scheduledIO.h"
#include "secondaryTables.h"
#include "canBroadcast.h"
#include "SD_logger.h"
#include "schedule_calcs.h"
#include "auxiliaries.h"
#include RTC_LIB_H //Defined in each boards .h file
#include BOARD_H //Note that this is not a real file, it is defined in globals.h. 


uint16_t req_fuel_uS = 0; /**< The required fuel variable (As calculated by TunerStudio) in uS */
uint16_t inj_opentime_uS = 0;

uint8_t ignitionChannelsOn; /**< The current state of the ignition system (on or off) */
uint8_t ignitionChannelsPending = 0; /**< Any ignition channels that are pending injections before they are resumed */
uint8_t fuelChannelsOn; /**< The current state of the fuel system (on or off) */
uint32_t rollingCutLastRev = 0; /**< Tracks whether we're on the same or a different rev for the rolling cut */

uint16_t staged_req_fuel_mult_pri = 0;
uint16_t staged_req_fuel_mult_sec = 0;   
#ifndef UNIT_TEST // Scope guard for unit testing
void setup(void)
{
  initialisationComplete = false; //Tracks whether the initialiseAll() function has run completely
  initialiseAll();
}

inline uint16_t applyFuelTrimToPW(trimTable3d *pTrimTable, int16_t fuelLoad, int16_t RPM, uint16_t currentPW)
{
    unsigned long pw1percent = 100 + get3DTableValue(pTrimTable, fuelLoad, RPM) - OFFSET_FUELTRIM;
    if (pw1percent != 100) { return div100(uint32_t(pw1percent * currentPW)); }
    return currentPW;
}

/** Speeduino main loop.
 * 
 * Main loop chores (roughly in the order that they are performed):
 * - Check if serial comms or tooth logging are in progress (send or receive, prioritise communication)
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
void loop(void)
{
      mainLoopCount++;
      LOOP_TIMER = TIMER_mask;

      //SERIAL Comms
      //Initially check that the last serial send values request is not still outstanding
      if (serialTransmitInProgress())
      {
        serialTransmit();
      }

      //Check for any new or in-progress requests from serial.
      if (Serial.available()>0 || serialRecieveInProgress())
      {
        serialReceive();
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
              readAuxCanBus();
              //Can0.read(inMsg);
              //currentStatus.canin[12] = inMsg.buf[5];
              //currentStatus.canin[13] = inMsg.id;
            }
          }
      #endif
          
    if(currentLoopTime > micros_safe())
    {
      //Occurs when micros() has overflowed
      deferEEPROMWritesUntil = 0; //Required to ensure that EEPROM writes are not deferred indefinitely
    }

    currentLoopTime = micros_safe();
    unsigned long timeToLastTooth = (currentLoopTime - toothLastToothTime);
    if ( (timeToLastTooth < MAX_STALL_TIME) || (toothLastToothTime > currentLoopTime) ) //Check how long ago the last tooth was seen compared to now. If it was more than half a second ago then the engine is probably stopped. toothLastToothTime can be greater than currentLoopTime if a pulse occurs between getting the latest time and doing the comparison
    {
      currentStatus.longRPM = getRPM(); //Long RPM is included here
      currentStatus.RPM = currentStatus.longRPM;
      currentStatus.RPMdiv100 = div100(currentStatus.RPM);
      FUEL_PUMP_ON();
      currentStatus.fuelPumpOn = true; //Not sure if this is needed.
    }
    else
    {
      //We reach here if the time between teeth is too great. This VERY likely means the engine has stopped
      currentStatus.RPM = 0;
      currentStatus.RPMdiv100 = 0;
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
      ignitionChannelsOn = 0;
      fuelChannelsOn = 0;
      if (fpPrimed == true) { FUEL_PUMP_OFF(); currentStatus.fuelPumpOn = false; } //Turn off the fuel pump, but only if the priming is complete
      if (configPage6.iacPWMrun == false) { disableIdle(); } //Turn off the idle PWM
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK); //Clear cranking bit (Can otherwise get stuck 'on' even with 0 rpm)
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_WARMUP); //Same as above except for WUE
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_RUN); //Same as above except for RUNNING status
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE); //Same as above except for ASE status
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC); //Same as above but the accel enrich (If using MAP accel enrich a stall will cause this to trigger)
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_DCC); //Same as above but the decel enleanment
      //This is a safety check. If for some reason the interrupts have got screwed up (Leading to 0rpm), this resets them.
      //It can possibly be run much less frequently.
      //This should only be run if the high speed logger are off because it will change the trigger interrupts back to defaults rather than the logger versions
      if( (currentStatus.toothLogEnabled == false) && (currentStatus.compositeTriggerUsed == 0) ) { initialiseTriggers(); }

      VVT1_PIN_LOW();
      VVT2_PIN_LOW();
      DISABLE_VVT_TIMER();
      boostDisable();
      if(configPage4.ignBypassEnabled > 0) { digitalWrite(pinIgnBypass, LOW); } //Reset the ignition bypass ready for next crank attempt
    }

    //***Perform sensor reads***
    //-----------------------------------------------------------------------------------------------------
    if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_1KHZ)) //Every 1ms. NOTE: This is NOT guaranteed to run at 1kHz on AVR systems. It will run at 1kHz if possible or as fast as loops/s allows if not. 
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_1KHZ);
      readMAP();
    }
    
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

      checkLaunchAndFlatShift(); //Check for launch control and flat shift being active

      //And check whether the tooth log buffer is ready
      if(toothHistoryIndex > TOOTH_LOG_SIZE) { BIT_SET(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY); }

      

    }
    if(BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ)) //10 hertz
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_10HZ);
      //updateFullStatus();
      checkProgrammableIO();
      idleControl(); //Perform any idle related actions. This needs to be run at 10Hz to align with the idle taper resolution of 0.1s
      
      // Air conditioning control
      airConControl();

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
      #if defined(NATIVE_CAN_AVAILABLE)
      if (configPage2.canBMWCluster == true) { sendBMWCluster(); }
      if (configPage2.canVAGCluster == true) { sendVAGCluster(); }
      #endif
      #if TPS_READ_FREQUENCY == 30
        readTPS();
      #endif
      readO2();
      readO2_2();

      #ifdef SD_LOGGING
        if(configPage13.onboard_log_file_rate == LOGGER_RATE_30HZ) { writeSDLogEntry(); }
      #endif

      //Check for any outstanding EEPROM writes.
      if( (isEepromWritePending() == true) && (serialStatusFlag == SERIAL_INACTIVE) && (micros() > deferEEPROMWritesUntil)) { writeAllConfig(); } 
    }
    if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_4HZ))
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_4HZ);
      //The IAT and CLT readings can be done less frequently (4 times per second)
      readCLT();
      readIAT();
      readBat();
      nitrousControl();

      //Lookup the current target idle RPM. This is aligned with coolant and so needs to be calculated at the same rate CLT is read
      if( (configPage2.idleAdvEnabled >= 1) || (configPage6.iacAlgorithm != IAC_ALGORITHM_NONE) )
      {
        currentStatus.CLIdleTarget = (byte)table2D_getValue(&idleTargetTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //All temps are offset by 40 degrees
        if(BIT_CHECK(currentStatus.airConStatus, BIT_AIRCON_TURNING_ON)) { currentStatus.CLIdleTarget += configPage15.airConIdleUpRPMAdder;  } //Adds Idle Up RPM amount if active
      }

      #ifdef SD_LOGGING
        if(configPage13.onboard_log_file_rate == LOGGER_RATE_4HZ) { writeSDLogEntry(); }
        syncSDLog(); //Sync the SD log file to the card 4 times per second. 
      #endif  
      
      currentStatus.fuelPressure = getFuelPressure();
      currentStatus.oilPressure = getOilPressure();
      
      if(auxIsEnabled == true)
      {
        //TODO dazq to clean this right up :)
        //check through the Aux input channels if enabled for Can or local use
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
            currentStatus.canin[currentStatus.current_caninchannel] = readAuxanalog(pinTranslateAnalog(configPage9.Auxinpina[currentStatus.current_caninchannel]&63));
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

      if ( (configPage10.wmiEnabled > 0) && (configPage10.wmiIndicatorEnabled > 0) )
      {
        // water tank empty
        if (BIT_CHECK(currentStatus.status4, BIT_STATUS4_WMI_EMPTY) > 0)
        {
          // flash with 1sec interval
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
    if ((currentStatus.hasSync || BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC)) && (currentStatus.RPM > 0))
    {
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
          if( !BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN) || (currentStatus.RPM < (currentStatus.crankRPM - CRANK_RUN_HYSTER)) )
          {
            //Sets the engine cranking bit, clears the engine running bit
            BIT_SET(currentStatus.engine, BIT_ENGINE_CRANK);
            BIT_CLEAR(currentStatus.engine, BIT_ENGINE_RUN);
            currentStatus.runSecs = 0; //We're cranking (hopefully), so reset the engine run time to prompt ASE.
            if(configPage4.ignBypassEnabled > 0) { digitalWrite(pinIgnBypass, LOW); }

            //Check whether the user has selected to disable to the fan during cranking
            if(configPage2.fanWhenCranking == 0) { FAN_OFF(); }
          }
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

      doCrankSpeedCalcs(); //In crankMaths.ino

      //Check that the duty cycle of the chosen pulsewidth isn't too high.
      uint32_t pwLimit = percentage(configPage2.dutyLim, revolutionTime); //The pulsewidth limit is determined to be the duty cycle limit (Eg 85%) by the total time it takes to perform 1 revolution
      //Handle multiple squirts per rev
      if (configPage2.strokes == FOUR_STROKE) { pwLimit = pwLimit * 2 / currentStatus.nSquirts; } 
      else { pwLimit = pwLimit / currentStatus.nSquirts; }
      //Apply the pwLimit if staging is disabled and engine is not cranking
      if( (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) && (configPage10.stagingEnabled == false) ) { if (currentStatus.PW1 > pwLimit) { currentStatus.PW1 = pwLimit; } }

      calculateStaging(pwLimit);

      //***********************************************************************************************
      //BEGIN INJECTION TIMING
      currentStatus.injAngle = table2D_getValue(&injectorAngleTable, currentStatus.RPMdiv100);
      unsigned int PWdivTimerPerDegree = div(currentStatus.PW1, timePerDegree).quot; //How many crank degrees the calculated PW will take at the current speed

      injector1StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);

      //Repeat the above for each cylinder
      switch (configPage2.nCylinders)
      {
        //Single cylinder
        case 1:
          //The only thing that needs to be done for single cylinder is to check for staging. 
          if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
          {
            PWdivTimerPerDegree = div(currentStatus.PW2, timePerDegree).quot; //Need to redo this for PW2 as it will be dramatically different to PW1 when staging
            //injector3StartAngle = calculateInjector3StartAngle(PWdivTimerPerDegree);
            injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
          }
          break;
        //2 cylinders
        case 2:
          //injector2StartAngle = calculateInjector2StartAngle(PWdivTimerPerDegree);
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
          
          if ( (configPage2.injLayout == INJ_SEQUENTIAL) && (configPage6.fuelTrimEnabled > 0) )
          {
            currentStatus.PW1 = applyFuelTrimToPW(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW1);
            currentStatus.PW2 = applyFuelTrimToPW(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW2);
          }
          else if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
          {
            PWdivTimerPerDegree = div(currentStatus.PW3, timePerDegree).quot; //Need to redo this for PW3 as it will be dramatically different to PW1 when staging
            injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
            injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);

            injector4StartAngle = injector3StartAngle + (CRANK_ANGLE_MAX_INJ / 2); //Phase this either 180 or 360 degrees out from inj3 (In reality this will always be 180 as you can't have sequential and staged currently)
            if(injector4StartAngle > (uint16_t)CRANK_ANGLE_MAX_INJ) { injector4StartAngle -= CRANK_ANGLE_MAX_INJ; }
          }
          break;
        //3 cylinders
        case 3:
          //injector2StartAngle = calculateInjector2StartAngle(PWdivTimerPerDegree);
          //injector3StartAngle = calculateInjector3StartAngle(PWdivTimerPerDegree);
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
          injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
          
          if ( (configPage2.injLayout == INJ_SEQUENTIAL) && (configPage6.fuelTrimEnabled > 0) )
          {
            currentStatus.PW1 = applyFuelTrimToPW(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW1);
            currentStatus.PW2 = applyFuelTrimToPW(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW2);
            currentStatus.PW3 = applyFuelTrimToPW(&trim3Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW3);

            #if INJ_CHANNELS >= 6
              if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
              {
                PWdivTimerPerDegree = div(currentStatus.PW4, timePerDegree).quot; //Need to redo this for PW4 as it will be dramatically different to PW1 when staging
                injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
                injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
                injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
              }
            #endif
          }
          else if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
          {
            PWdivTimerPerDegree = div(currentStatus.PW4, timePerDegree).quot; //Need to redo this for PW3 as it will be dramatically different to PW1 when staging
            injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
            #if INJ_CHANNELS >= 6
              injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
              injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
            #endif
          }
          break;
        //4 cylinders
        case 4:
          //injector2StartAngle = calculateInjector2StartAngle(PWdivTimerPerDegree);
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);

          if((configPage2.injLayout == INJ_SEQUENTIAL) && currentStatus.hasSync)
          {
            if( CRANK_ANGLE_MAX_INJ != 720 ) { changeHalfToFullSync(); }

            injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
            injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees, currentStatus.injAngle);
            #if INJ_CHANNELS >= 8
              if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
              {
                PWdivTimerPerDegree = div(currentStatus.PW5, timePerDegree).quot; //Need to redo this for PW5 as it will be dramatically different to PW1 when staging
                injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
                injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
                injector7StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
                injector8StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees, currentStatus.injAngle);
              }
            #endif

            if(configPage6.fuelTrimEnabled > 0)
            {
              currentStatus.PW1 = applyFuelTrimToPW(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW1);
              currentStatus.PW2 = applyFuelTrimToPW(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW2);
              currentStatus.PW3 = applyFuelTrimToPW(&trim3Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW3);
              currentStatus.PW4 = applyFuelTrimToPW(&trim4Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW4);
            }
          }
          else if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
          {
            PWdivTimerPerDegree = div(currentStatus.PW3, timePerDegree).quot; //Need to redo this for PW3 as it will be dramatically different to PW1 when staging
            injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
            injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
          }
          else
          {
            if( BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && (CRANK_ANGLE_MAX_INJ != 360) ) { changeFullToHalfSync(); }
          }
          break;
        //5 cylinders
        case 5:
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
          injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
          injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees, currentStatus.injAngle);
          #if INJ_CHANNELS >= 5
            injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel5InjDegrees, currentStatus.injAngle);
          #endif

          //Staging is possible by using the 6th channel if available
          #if INJ_CHANNELS >= 6
            if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
            {
              PWdivTimerPerDegree = div(currentStatus.PW6, timePerDegree).quot;
              injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel6InjDegrees, currentStatus.injAngle);
            }
          #endif

          break;
        //6 cylinders
        case 6:
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
          injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
          
          #if INJ_CHANNELS >= 6
            if((configPage2.injLayout == INJ_SEQUENTIAL) && currentStatus.hasSync)
            {
              if( CRANK_ANGLE_MAX_INJ != 720 ) { changeHalfToFullSync(); }

              injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees, currentStatus.injAngle);
              injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel5InjDegrees, currentStatus.injAngle);
              injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel6InjDegrees, currentStatus.injAngle);

              if(configPage6.fuelTrimEnabled > 0)
              {
                currentStatus.PW1 = applyFuelTrimToPW(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW1);
                currentStatus.PW2 = applyFuelTrimToPW(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW2);
                currentStatus.PW3 = applyFuelTrimToPW(&trim3Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW3);
                currentStatus.PW4 = applyFuelTrimToPW(&trim4Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW4);
                currentStatus.PW5 = applyFuelTrimToPW(&trim5Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW5);
                currentStatus.PW6 = applyFuelTrimToPW(&trim6Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW6);
              }

              //Staging is possible with sequential on 8 channel boards by using outputs 7 + 8 for the staged injectors
              #if INJ_CHANNELS >= 8
                if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
                {
                  PWdivTimerPerDegree = div(currentStatus.PW4, timePerDegree).quot; //Need to redo this for staging PW as it will be dramatically different to PW1 when staging
                  injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
                  injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
                  injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
                }
              #endif
            }
            else
            {
              if( BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && (CRANK_ANGLE_MAX_INJ != 360) ) { changeFullToHalfSync(); }

              if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
              {
                PWdivTimerPerDegree = div(currentStatus.PW4, timePerDegree).quot; //Need to redo this for staging PW as it will be dramatically different to PW1 when staging
                injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
                injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
                injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle); 
              }
            }
          #endif
          break;
        //8 cylinders
        case 8:
          injector2StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
          injector3StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
          injector4StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees, currentStatus.injAngle);

          #if INJ_CHANNELS >= 8
            if((configPage2.injLayout == INJ_SEQUENTIAL) && currentStatus.hasSync)
            {
              if( CRANK_ANGLE_MAX_INJ != 720 ) { changeHalfToFullSync(); }

              injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel5InjDegrees, currentStatus.injAngle);
              injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel6InjDegrees, currentStatus.injAngle);
              injector7StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel7InjDegrees, currentStatus.injAngle);
              injector8StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel8InjDegrees, currentStatus.injAngle);

              if(configPage6.fuelTrimEnabled > 0)
              {
                currentStatus.PW1 = applyFuelTrimToPW(&trim1Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW1);
                currentStatus.PW2 = applyFuelTrimToPW(&trim2Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW2);
                currentStatus.PW3 = applyFuelTrimToPW(&trim3Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW3);
                currentStatus.PW4 = applyFuelTrimToPW(&trim4Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW4);
                currentStatus.PW5 = applyFuelTrimToPW(&trim5Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW5);
                currentStatus.PW6 = applyFuelTrimToPW(&trim6Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW6);
                currentStatus.PW7 = applyFuelTrimToPW(&trim7Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW7);
                currentStatus.PW8 = applyFuelTrimToPW(&trim8Table, currentStatus.fuelLoad, currentStatus.RPM, currentStatus.PW8);
              }
            }
            else
            {
              if( BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && (CRANK_ANGLE_MAX_INJ != 360) ) { changeFullToHalfSync(); }

              if( (configPage10.stagingEnabled == true) && (BIT_CHECK(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE) == true) )
              {
                PWdivTimerPerDegree = div(currentStatus.PW5, timePerDegree).quot; //Need to redo this for PW3 as it will be dramatically different to PW1 when staging
                injector5StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel1InjDegrees, currentStatus.injAngle);
                injector6StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel2InjDegrees, currentStatus.injAngle);
                injector7StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel3InjDegrees, currentStatus.injAngle);
                injector8StartAngle = calculateInjectorStartAngle(PWdivTimerPerDegree, channel4InjDegrees, currentStatus.injAngle);
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
      uint16_t maxAllowedRPM = checkRevLimit(); //The maximum RPM allowed by all the potential limiters (Engine protection, 2-step, flat shift etc). Divided by 100. `checkRevLimit()` returns the current maximum RPM allow (divided by 100) based on either the fixed hard limit or the current coolant temp
      //Check each of the functions that has an RPM limit. Update the max allowed RPM if the function is active and has a lower RPM than already set
      if( checkEngineProtect() && (configPage4.engineProtectMaxRPM < maxAllowedRPM)) { maxAllowedRPM = configPage4.engineProtectMaxRPM; }
      if ( (currentStatus.launchingHard == true) && (configPage6.lnchHardLim < maxAllowedRPM) ) { maxAllowedRPM = configPage6.lnchHardLim; }
      maxAllowedRPM = maxAllowedRPM * 100; //All of the above limits are divided by 100, convert back to RPM
      if ( (currentStatus.flatShiftingHard == true) && (currentStatus.clutchEngagedRPM < maxAllowedRPM) ) { maxAllowedRPM = currentStatus.clutchEngagedRPM; } //Flat shifting is a special case as the RPM limit is based on when the clutch was engaged. It is not divided by 100 as it is set with the actual RPM
    
      if( (configPage2.hardCutType == HARD_CUT_FULL) && (currentStatus.RPM > maxAllowedRPM) )
      {
        //Full hard cut turns outputs off completely. 
        switch(configPage6.engineProtectType)
        {
          case PROTECT_CUT_OFF:
            //Make sure all channels are turned on
            ignitionChannelsOn = 0xFF;
            fuelChannelsOn = 0xFF;
            currentStatus.engineProtectStatus = 0;
            break;
          case PROTECT_CUT_IGN:
            ignitionChannelsOn = 0;
            break;
          case PROTECT_CUT_FUEL:
            fuelChannelsOn = 0;
            break;
          case PROTECT_CUT_BOTH:
            ignitionChannelsOn = 0;
            fuelChannelsOn = 0;
            break;
          default:
            ignitionChannelsOn = 0;
            fuelChannelsOn = 0;
            break;
        }
      } //Hard cut check
      else if( (configPage2.hardCutType == HARD_CUT_ROLLING) && (currentStatus.RPM > (maxAllowedRPM + (configPage15.rollingProtRPMDelta[0] * 10))) ) //Limit for rolling is the max allowed RPM minus the lowest value in the delta table (Delta values are negative!)
      { 
        uint8_t revolutionsToCut = 1;
        if(configPage2.strokes == FOUR_STROKE) { revolutionsToCut *= 2; } //4 stroke needs to cut for at least 2 revolutions
        if( (configPage4.sparkMode != IGN_MODE_SEQUENTIAL) || (configPage2.injLayout != INJ_SEQUENTIAL) ) { revolutionsToCut *= 2; } //4 stroke and non-sequential will cut for 4 revolutions minimum. This is to ensure no half fuel ignition cycles take place

        if(rollingCutLastRev == 0) { rollingCutLastRev = currentStatus.startRevolutions; } //First time check
        if ( (currentStatus.startRevolutions >= (rollingCutLastRev + revolutionsToCut)) || (currentStatus.RPM > maxAllowedRPM) ) //If current RPM is over the max allowed RPM always cut, otherwise check if the required number of revolutions have passed since the last cut
        { 
          uint8_t cutPercent = 0;
          int16_t rpmDelta = currentStatus.RPM - maxAllowedRPM;
          if(rpmDelta >= 0) { cutPercent = 100; } //If the current RPM is over the max allowed RPM then cut is full (100%)
          else { cutPercent = table2D_getValue(&rollingCutTable, (rpmDelta / 10) ); } //
          

          for(uint8_t x=0; x<max(maxIgnOutputs, maxInjOutputs); x++)
          {  
            if( (cutPercent == 100) || (random1to100() < cutPercent) )
            {
              switch(configPage6.engineProtectType)
              {
                case PROTECT_CUT_OFF:
                  //Make sure all channels are turned on
                  ignitionChannelsOn = 0xFF;
                  fuelChannelsOn = 0xFF;
                  break;
                case PROTECT_CUT_IGN:
                  BIT_CLEAR(ignitionChannelsOn, x); //Turn off this ignition channel
                  disablePendingIgnSchedule(x);
                  break;
                case PROTECT_CUT_FUEL:
                  BIT_CLEAR(fuelChannelsOn, x); //Turn off this fuel channel
                  disablePendingFuelSchedule(x);
                  break;
                case PROTECT_CUT_BOTH:
                  BIT_CLEAR(ignitionChannelsOn, x); //Turn off this ignition channel
                  BIT_CLEAR(fuelChannelsOn, x); //Turn off this fuel channel
                  disablePendingFuelSchedule(x);
                  disablePendingIgnSchedule(x);
                  break;
                default:
                  BIT_CLEAR(ignitionChannelsOn, x); //Turn off this ignition channel
                  BIT_CLEAR(fuelChannelsOn, x); //Turn off this fuel channel
                  break;
              }
            }
            else
            {
              //Turn fuel and ignition channels on

              //Special case for non-sequential, 4-stroke where both fuel and ignition are cut. The ignition pulses should wait 1 cycle after the fuel channels are turned back on before firing again
              if( (revolutionsToCut == 4) &&                          //4 stroke and non-sequential
                  (BIT_CHECK(fuelChannelsOn, x) == false) &&          //Fuel on this channel is currently off, meaning it is the first revolution after a cut
                  (configPage6.engineProtectType == PROTECT_CUT_BOTH) //Both fuel and ignition are cut
                )
              { BIT_SET(ignitionChannelsPending, x); } //Set this ignition channel as pending
              else { BIT_SET(ignitionChannelsOn, x); } //Turn on this ignition channel
                
              
              BIT_SET(fuelChannelsOn, x); //Turn on this fuel channel
            }
          }
          rollingCutLastRev = currentStatus.startRevolutions;
        }

        //Check whether there are any ignition channels that are waiting for injection pulses to occur before being turned back on. This can only occur when at least 2 revolutions have taken place since the fuel was turned back on
        //Note that ignitionChannelsPending can only be >0 on 4 stroke, non-sequential fuel when protect type is Both
        if( (ignitionChannelsPending > 0) && (currentStatus.startRevolutions >= (rollingCutLastRev + 2)) )
        {
          ignitionChannelsOn = fuelChannelsOn;
          ignitionChannelsPending = 0;
        }
      } //Rolling cut check
      else
      {
        currentStatus.engineProtectStatus = 0;
        //No engine protection active, so turn all the channels on
        if(currentStatus.startRevolutions >= configPage4.StgCycles)
        { 
          //Enable the fuel and ignition, assuming staging revolutions are complete 
          ignitionChannelsOn = 0xff; 
          fuelChannelsOn = 0xff; 
        } 
      }


#if INJ_CHANNELS >= 1
      if( (maxInjOutputs >= 1) && (currentStatus.PW1 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ1_CMD_BIT)) )
      {
        uint32_t timeOut = calculateInjectorTimeout(fuelSchedule1, channel1InjDegrees, injector1StartAngle, crankAngle);
        if (timeOut>0U)
        {
          setFuelSchedule1(
                    timeOut,
                    (unsigned long)currentStatus.PW1
                    );
        }
      }
#endif

        /*-----------------------------------------------------------------------------------------
        | A Note on tempCrankAngle and tempStartAngle:
        |   The use of tempCrankAngle/tempStartAngle is described below. It is then used in the same way for channels 2, 3 and 4+ on both injectors and ignition
        |   Essentially, these 2 variables are used to realign the current crank angle and the desired start angle around 0 degrees for the given cylinder/output
        |   Eg: If cylinder 2 TDC is 180 degrees after cylinder 1 (Eg a standard 4 cylinder engine), then tempCrankAngle is 180* less than the current crank angle and
        |       tempStartAngle is the desired open time less 180*. Thus the cylinder is being treated relative to its own TDC, regardless of its offset
        |
        |   This is done to avoid problems with very short of very long times until tempStartAngle.
        |------------------------------------------------------------------------------------------
        */
#if INJ_CHANNELS >= 2
        if( (maxInjOutputs >= 2) && (currentStatus.PW2 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ2_CMD_BIT)) )
        {
          uint32_t timeOut = calculateInjectorTimeout(fuelSchedule2, channel2InjDegrees, injector2StartAngle, crankAngle);
          if ( timeOut>0U )
          {
            setFuelSchedule2(
                      timeOut,
                      (unsigned long)currentStatus.PW2
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 3
        if( (maxInjOutputs >= 3) && (currentStatus.PW3 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ3_CMD_BIT)) )
        {
          uint32_t timeOut = calculateInjectorTimeout(fuelSchedule3, channel3InjDegrees, injector3StartAngle, crankAngle);
          if ( timeOut>0U )
          {
            setFuelSchedule3(
                      timeOut,
                      (unsigned long)currentStatus.PW3
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 4
        if( (maxInjOutputs >= 4) && (currentStatus.PW4 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ4_CMD_BIT)) )
        {
          uint32_t timeOut = calculateInjectorTimeout(fuelSchedule4, channel4InjDegrees, injector4StartAngle, crankAngle);
          if ( timeOut>0U )
          {
            setFuelSchedule4(
                      timeOut,
                      (unsigned long)currentStatus.PW4
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 5
        if( (maxInjOutputs >= 5) && (currentStatus.PW5 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ5_CMD_BIT)) )
        {
          uint32_t timeOut = calculateInjectorTimeout(fuelSchedule5, channel5InjDegrees, injector5StartAngle, crankAngle);
          if ( timeOut>0U )
          {
            setFuelSchedule5(
                      timeOut,
                      (unsigned long)currentStatus.PW5
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 6
        if( (maxInjOutputs >= 6) && (currentStatus.PW6 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ6_CMD_BIT)) )
        {
          uint32_t timeOut = calculateInjectorTimeout(fuelSchedule6, channel6InjDegrees, injector6StartAngle, crankAngle);
          if ( timeOut>0U )
          {
            setFuelSchedule6(
                      timeOut,
                      (unsigned long)currentStatus.PW6
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 7
        if( (maxInjOutputs >= 7) && (currentStatus.PW7 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ7_CMD_BIT)) )
        {
          uint32_t timeOut = calculateInjectorTimeout(fuelSchedule7, channel7InjDegrees, injector7StartAngle, crankAngle);
          if ( timeOut>0U )
          {
            setFuelSchedule7(
                      timeOut,
                      (unsigned long)currentStatus.PW7
                      );
          }
        }
#endif

#if INJ_CHANNELS >= 8
        if( (maxInjOutputs >= 8) && (currentStatus.PW8 >= inj_opentime_uS) && (BIT_CHECK(fuelChannelsOn, INJ8_CMD_BIT)) )
        {
          uint32_t timeOut = calculateInjectorTimeout(fuelSchedule8, channel8InjDegrees, injector8StartAngle, crankAngle);
          if ( timeOut>0U )
          {
            setFuelSchedule8(
                      timeOut,
                      (unsigned long)currentStatus.PW8
                      );
          }
        }
#endif

      //***********************************************************************************************
      //| BEGIN IGNITION SCHEDULES
      //Same as above, except for ignition

      //fixedCrankingOverride is used to extend the dwell during cranking so that the decoder can trigger the spark upon seeing a certain tooth. Currently only available on the basic distributor and 4g63 decoders.
      if ( configPage4.ignCranklock && BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && (BIT_CHECK(decoderState, BIT_DECODER_HAS_FIXED_CRANKING)) )
      {
        fixedCrankingOverride = currentStatus.dwell * 3;
        //This is a safety step to prevent the ignition start time occurring AFTER the target tooth pulse has already occurred. It simply moves the start time forward a little, which is compensated for by the increase in the dwell time
        if(currentStatus.RPM < 250)
        {
          ignition1StartAngle -= 5;
          ignition2StartAngle -= 5;
          ignition3StartAngle -= 5;
          ignition4StartAngle -= 5;
#if IGN_CHANNELS >= 5
          ignition5StartAngle -= 5;
#endif
#if IGN_CHANNELS >= 6          
          ignition6StartAngle -= 5;
#endif
#if IGN_CHANNELS >= 7
          ignition7StartAngle -= 5;
#endif
#if IGN_CHANNELS >= 8
          ignition8StartAngle -= 5;
#endif
        }
      }
      else { fixedCrankingOverride = 0; }

      if(ignitionChannelsOn > 0)
      {
        //Refresh the current crank angle info
        //ignition1StartAngle = 335;
        crankAngle = getCrankAngle(); //Refresh with the latest crank angle
        while (crankAngle > CRANK_ANGLE_MAX_IGN ) { crankAngle -= CRANK_ANGLE_MAX_IGN; }

#if IGN_CHANNELS >= 1
        uint32_t timeOut = calculateIgnitionTimeout(ignitionSchedule1, ignition1StartAngle, channel1IgnDegrees, crankAngle);
        if ( (timeOut > 0U) && (BIT_CHECK(ignitionChannelsOn, IGN1_CMD_BIT)) )
        {
          
          setIgnitionSchedule1(ign1StartFunction,
                    //((unsigned long)(ignition1StartAngle - crankAngle) * (unsigned long)timePerDegree),
                    timeOut,
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
            unsigned long ignition2StartTime = calculateIgnitionTimeout(ignitionSchedule2, ignition2StartAngle, channel2IgnDegrees, crankAngle);

            if ( (ignition2StartTime > 0) && (BIT_CHECK(ignitionChannelsOn, IGN2_CMD_BIT)) )
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
            unsigned long ignition3StartTime = calculateIgnitionTimeout(ignitionSchedule3, ignition3StartAngle, channel3IgnDegrees, crankAngle);

            if ( (ignition3StartTime > 0) && (BIT_CHECK(ignitionChannelsOn, IGN3_CMD_BIT)) )
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
            unsigned long ignition4StartTime = calculateIgnitionTimeout(ignitionSchedule4, ignition4StartAngle, channel4IgnDegrees, crankAngle);

            if ( (ignition4StartTime > 0) && (BIT_CHECK(ignitionChannelsOn, IGN4_CMD_BIT)) )
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
            unsigned long ignition5StartTime = calculateIgnitionTimeout(ignitionSchedule5, ignition5StartAngle, channel5IgnDegrees, crankAngle);

            if ( (ignition5StartTime > 0) && (BIT_CHECK(ignitionChannelsOn, IGN5_CMD_BIT)) )
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
            unsigned long ignition6StartTime = calculateIgnitionTimeout(ignitionSchedule6, ignition6StartAngle, channel6IgnDegrees, crankAngle);

            if ( (ignition6StartTime > 0) && (BIT_CHECK(ignitionChannelsOn, IGN6_CMD_BIT)) )
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
            unsigned long ignition7StartTime = calculateIgnitionTimeout(ignitionSchedule7, ignition7StartAngle, channel7IgnDegrees, crankAngle);

            if ( (ignition7StartTime > 0) && (BIT_CHECK(ignitionChannelsOn, IGN7_CMD_BIT)) )
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
            unsigned long ignition8StartTime = calculateIgnitionTimeout(ignitionSchedule8, ignition8StartAngle, channel8IgnDegrees, crankAngle);

            if ( (ignition8StartTime > 0) && (BIT_CHECK(ignitionChannelsOn, IGN8_CMD_BIT)) )
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
  
  if ( (configPage2.includeAFR == true) && (configPage6.egoType == EGO_TYPE_WIDE) && (currentStatus.runSecs > configPage6.ego_sdelay) ) {
    iAFR = ((unsigned int)currentStatus.O2 << 7) / currentStatus.afrTarget;  //Include AFR (vs target) if enabled
  }
  if ( (configPage2.incorporateAFR == true) && (configPage2.includeAFR == false) ) {
    iAFR = ((unsigned int)configPage2.stoich << 7) / currentStatus.afrTarget;  //Incorporate stoich vs target AFR, if enabled.
  }
  iCorrections = (corrections << bitShift) / 100;
  //iCorrections = divu100((corrections << bitShift));


  unsigned long intermediate = ((uint32_t)REQ_FUEL * (uint32_t)iVE) >> 7; //Need to use an intermediate value to avoid overflowing the long
  if ( configPage2.multiplyMAP > 0 ) { intermediate = (intermediate * (unsigned long)iMAP) >> 7; }
  
  if ( (configPage2.includeAFR == true) && (configPage6.egoType == EGO_TYPE_WIDE) && (currentStatus.runSecs > configPage6.ego_sdelay) ) {
    //EGO type must be set to wideband and the AFR warmup time must've elapsed for this to be used
    intermediate = (intermediate * (unsigned long)iAFR) >> 7;  
  }
  if ( (configPage2.incorporateAFR == true) && (configPage2.includeAFR == false) ) {
    intermediate = (intermediate * (unsigned long)iAFR) >> 7;
  }
  
  intermediate = (intermediate * (unsigned long)iCorrections) >> bitShift;
  if (intermediate != 0)
  {
    //If intermediate is not 0, we need to add the opening time (0 typically indicates that one of the full fuel cuts is active)
    intermediate += injOpen; //Add the injector opening time
    //AE calculation only when ACC is active.
    if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC) )
    {
      //AE Adds % of req_fuel
      if ( configPage2.aeApplyMode == AE_MODE_ADDER )
        {
          intermediate += ( ((unsigned long)REQ_FUEL) * (currentStatus.AEamount - 100) ) / 100;
        }
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
byte getVE1(void)
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
byte getAdvance1(void)
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
      calculateIgnitionAngle(dwellAngle, channel1IgnDegrees, currentStatus.advance, &ignition1EndAngle, &ignition1StartAngle);
      break;
    //2 cylinders
    case 2:
      calculateIgnitionAngle(dwellAngle, channel1IgnDegrees, currentStatus.advance, &ignition1EndAngle, &ignition1StartAngle);
      calculateIgnitionAngle(dwellAngle, channel2IgnDegrees, currentStatus.advance, &ignition2EndAngle, &ignition2StartAngle);
      break;
    //3 cylinders
    case 3:
      calculateIgnitionAngle(dwellAngle, channel1IgnDegrees, currentStatus.advance, &ignition1EndAngle, &ignition1StartAngle);
      calculateIgnitionAngle(dwellAngle, channel2IgnDegrees, currentStatus.advance, &ignition2EndAngle, &ignition2StartAngle);
      calculateIgnitionAngle(dwellAngle, channel3IgnDegrees, currentStatus.advance, &ignition3EndAngle, &ignition3StartAngle);
      break;
    //4 cylinders
    case 4:
      calculateIgnitionAngle(dwellAngle, channel1IgnDegrees, currentStatus.advance, &ignition1EndAngle, &ignition1StartAngle);
      calculateIgnitionAngle(dwellAngle, channel2IgnDegrees, currentStatus.advance, &ignition2EndAngle, &ignition2StartAngle);

      #if IGN_CHANNELS >= 4
      if((configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && currentStatus.hasSync)
      {
        if( CRANK_ANGLE_MAX_IGN != 720 ) { changeHalfToFullSync(); }

        calculateIgnitionAngle(dwellAngle, channel3IgnDegrees, currentStatus.advance, &ignition3EndAngle, &ignition3StartAngle);
        calculateIgnitionAngle(dwellAngle, channel4IgnDegrees, currentStatus.advance, &ignition4EndAngle, &ignition4StartAngle);
      }
      else if(configPage4.sparkMode == IGN_MODE_ROTARY)
      {
        byte splitDegrees = 0;
        splitDegrees = table2D_getValue(&rotarySplitTable, currentStatus.ignLoad);

        //The trailing angles are set relative to the leading ones
        calculateIgnitionTrailingRotary(dwellAngle, splitDegrees, ignition1EndAngle, &ignition3EndAngle, &ignition3StartAngle);
        calculateIgnitionTrailingRotary(dwellAngle, splitDegrees, ignition2EndAngle, &ignition4EndAngle, &ignition4StartAngle);
      }
      else
      {
        if( BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && (CRANK_ANGLE_MAX_IGN != 360) ) { changeFullToHalfSync(); }
      }
      #endif
      break;
    //5 cylinders
    case 5:
      calculateIgnitionAngle(dwellAngle, channel1IgnDegrees, currentStatus.advance, &ignition1EndAngle, &ignition1StartAngle);
      calculateIgnitionAngle(dwellAngle, channel2IgnDegrees, currentStatus.advance, &ignition2EndAngle, &ignition2StartAngle);
      calculateIgnitionAngle(dwellAngle, channel3IgnDegrees, currentStatus.advance, &ignition3EndAngle, &ignition3StartAngle);
      calculateIgnitionAngle(dwellAngle, channel4IgnDegrees, currentStatus.advance, &ignition4EndAngle, &ignition4StartAngle);
      calculateIgnitionAngle(dwellAngle, channel5IgnDegrees, currentStatus.advance, &ignition5EndAngle, &ignition5StartAngle);
      break;
    //6 cylinders
    case 6:
      calculateIgnitionAngle(dwellAngle, channel1IgnDegrees, currentStatus.advance, &ignition1EndAngle, &ignition1StartAngle);
      calculateIgnitionAngle(dwellAngle, channel2IgnDegrees, currentStatus.advance, &ignition2EndAngle, &ignition2StartAngle);
      calculateIgnitionAngle(dwellAngle, channel3IgnDegrees, currentStatus.advance, &ignition3EndAngle, &ignition3StartAngle);

      #if IGN_CHANNELS >= 6
      if((configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && currentStatus.hasSync)
      {
        if( CRANK_ANGLE_MAX_IGN != 720 ) { changeHalfToFullSync(); }

        calculateIgnitionAngle(dwellAngle, channel4IgnDegrees, currentStatus.advance, &ignition4EndAngle, &ignition4StartAngle);
        calculateIgnitionAngle(dwellAngle, channel5IgnDegrees, currentStatus.advance, &ignition5EndAngle, &ignition5StartAngle);
        calculateIgnitionAngle(dwellAngle, channel6IgnDegrees, currentStatus.advance, &ignition6EndAngle, &ignition6StartAngle);
      }
      else
      {
        if( BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && (CRANK_ANGLE_MAX_IGN != 360) ) { changeFullToHalfSync(); }
      }
      #endif
      break;
    //8 cylinders
    case 8:
      calculateIgnitionAngle(dwellAngle, channel1IgnDegrees, currentStatus.advance, &ignition1EndAngle, &ignition1StartAngle);
      calculateIgnitionAngle(dwellAngle, channel2IgnDegrees, currentStatus.advance, &ignition2EndAngle, &ignition2StartAngle);
      calculateIgnitionAngle(dwellAngle, channel3IgnDegrees, currentStatus.advance, &ignition3EndAngle, &ignition3StartAngle);
      calculateIgnitionAngle(dwellAngle, channel4IgnDegrees, currentStatus.advance, &ignition4EndAngle, &ignition4StartAngle);

      #if IGN_CHANNELS >= 8
      if((configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && currentStatus.hasSync)
      {
        if( CRANK_ANGLE_MAX_IGN != 720 ) { changeHalfToFullSync(); }

        calculateIgnitionAngle(dwellAngle, channel5IgnDegrees, currentStatus.advance, &ignition5EndAngle, &ignition5StartAngle);
        calculateIgnitionAngle(dwellAngle, channel6IgnDegrees, currentStatus.advance, &ignition6EndAngle, &ignition6StartAngle);
        calculateIgnitionAngle(dwellAngle, channel7IgnDegrees, currentStatus.advance, &ignition7EndAngle, &ignition7StartAngle);
        calculateIgnitionAngle(dwellAngle, channel8IgnDegrees, currentStatus.advance, &ignition8EndAngle, &ignition8StartAngle);
      }
      else
      {
        if( BIT_CHECK(currentStatus.status3, BIT_STATUS3_HALFSYNC) && (CRANK_ANGLE_MAX_IGN != 360) ) { changeFullToHalfSync(); }
      }
      #endif
      break;

    //Will hit the default case on >8 cylinders. Do nothing in these cases
    default:
      break;
  }
}

void calculateStaging(uint32_t pwLimit)
{
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

      byte stagingSplit = get3DTableValue(&stagingTable, currentStatus.fuelLoad, currentStatus.RPM);
      currentStatus.PW1 = ((100 - stagingSplit) * tempPW1) / 100;
      currentStatus.PW1 += inj_opentime_uS; 

      //PW2 is used temporarily to hold the secondary injector pulsewidth. It will be assigned to the correct channel below
      if(stagingSplit > 0) 
      { 
        BIT_SET(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE); //Set the staging active flag
        currentStatus.PW2 = (stagingSplit * tempPW3) / 100; 
        currentStatus.PW2 += inj_opentime_uS;
      }
      else
      {
        BIT_CLEAR(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE); //Clear the staging active flag
        currentStatus.PW2 = 0; 
      }
    }
    else if(configPage10.stagingMode == STAGING_MODE_AUTO)
    {
      currentStatus.PW1 = tempPW1;
      //If automatic mode, the primary injectors are used all the way up to their limit (Configured by the pulsewidth limit setting)
      //If they exceed their limit, the extra duty is passed to the secondaries
      if(tempPW1 > pwLimit)
      {
        BIT_SET(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE); //Set the staging active flag
        uint32_t extraPW = tempPW1 - pwLimit + inj_opentime_uS; //The open time must be added here AND below because tempPW1 does not include an open time. The addition of it here takes into account the fact that pwLlimit does not contain an allowance for an open time. 
        currentStatus.PW1 = pwLimit;
        currentStatus.PW2 = ((extraPW * staged_req_fuel_mult_sec) / staged_req_fuel_mult_pri); //Convert the 'left over' fuel amount from primary injector scaling to secondary
        currentStatus.PW2 += inj_opentime_uS;
      }
      else 
      {
        //If tempPW1 < pwLImit it means that the entire fuel load can be handled by the primaries and staging is inactive. 
        //Secondary PW is simply set to 0 
        BIT_CLEAR(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE); //Clear the staging active flag 
        currentStatus.PW2 = 0; 
      } 
    }

    //Allocate the primary and secondary pulse widths based on the fuel configuration
    switch (configPage2.nCylinders) 
    {
      case 1:
        //Nothing required for 1 cylinder, channels are correct already
        break;
      case 2:
        //Primary pulsewidth on channels 1 and 2, secondary on channels 3 and 4
        currentStatus.PW3 = currentStatus.PW2;
        currentStatus.PW4 = currentStatus.PW2;
        currentStatus.PW2 = currentStatus.PW1;
        break;
      case 3:
        //6 channels required for 'normal' 3 cylinder staging support
        #if INJ_CHANNELS >= 6
          //Primary pulsewidth on channels 1, 2 and 3, secondary on channels 4, 5 and 6
          currentStatus.PW4 = currentStatus.PW2;
          currentStatus.PW5 = currentStatus.PW2;
          currentStatus.PW6 = currentStatus.PW2;
        #else
          //If there are not enough channels, then primary pulsewidth is on channels 1, 2 and 3, secondary on channel 4
          currentStatus.PW4 = currentStatus.PW2;
        #endif
        currentStatus.PW2 = currentStatus.PW1;
        currentStatus.PW3 = currentStatus.PW1;
        break;
      case 4:
        if( (configPage2.injLayout == INJ_SEQUENTIAL) || (configPage2.injLayout == INJ_SEMISEQUENTIAL) )
        {
          //Staging with 4 cylinders semi/sequential requires 8 total channels
          #if INJ_CHANNELS >= 8
            currentStatus.PW5 = currentStatus.PW2;
            currentStatus.PW6 = currentStatus.PW2;
            currentStatus.PW7 = currentStatus.PW2;
            currentStatus.PW8 = currentStatus.PW2;

            currentStatus.PW2 = currentStatus.PW1;
            currentStatus.PW3 = currentStatus.PW1;
            currentStatus.PW4 = currentStatus.PW1;
          #else
            //This is an invalid config as there are not enough outputs to support sequential + staging
            //Put the staging output to the non-existant channel 5
            currentStatus.PW5 = currentStatus.PW2;
          #endif
        }
        else
        {
          currentStatus.PW3 = currentStatus.PW2;
          currentStatus.PW4 = currentStatus.PW2;
          currentStatus.PW2 = currentStatus.PW1;
        }
        break;
        
      case 5:
        //No easily supportable 5 cylinder staging option unless there are at least 5 channels
        #if INJ_CHANNELS >= 5
          if (configPage2.injLayout != INJ_SEQUENTIAL)
          {
            currentStatus.PW5 = currentStatus.PW2;
          }
          #if INJ_CHANNELS >= 6
            currentStatus.PW6 = currentStatus.PW2;
          #endif
        #endif
        
          currentStatus.PW2 = currentStatus.PW1;
          currentStatus.PW3 = currentStatus.PW1;
          currentStatus.PW4 = currentStatus.PW1;
        break;

      case 6:
        #if INJ_CHANNELS >= 6
          //8 cylinder staging only if not sequential
          if (configPage2.injLayout != INJ_SEQUENTIAL)
          {
            currentStatus.PW4 = currentStatus.PW2;
            currentStatus.PW5 = currentStatus.PW2;
            currentStatus.PW6 = currentStatus.PW2;
          }
          #if INJ_CHANNELS >= 8
          else
            {
              //If there are 8 channels, then the 6 cylinder sequential option is available by using channels 7 + 8 for staging
              currentStatus.PW7 = currentStatus.PW2;
              currentStatus.PW8 = currentStatus.PW2;

              currentStatus.PW4 = currentStatus.PW1;
              currentStatus.PW5 = currentStatus.PW1;
              currentStatus.PW6 = currentStatus.PW1;
            }
          #endif
        #endif
        currentStatus.PW2 = currentStatus.PW1;
        currentStatus.PW3 = currentStatus.PW1;
        break;

      case 8:
        #if INJ_CHANNELS >= 8
          //8 cylinder staging only if not sequential
          if (configPage2.injLayout != INJ_SEQUENTIAL)
          {
            currentStatus.PW5 = currentStatus.PW2;
            currentStatus.PW6 = currentStatus.PW2;
            currentStatus.PW7 = currentStatus.PW2;
            currentStatus.PW8 = currentStatus.PW2;
          }
        #endif
        currentStatus.PW2 = currentStatus.PW1;
        currentStatus.PW3 = currentStatus.PW1;
        currentStatus.PW4 = currentStatus.PW1;
        break;

      default:
        //Assume 4 cylinder non-seq for default
        currentStatus.PW3 = currentStatus.PW2;
        currentStatus.PW4 = currentStatus.PW2;
        currentStatus.PW2 = currentStatus.PW1;
        break;
    }
  }
  else 
  { 
    if(maxInjOutputs >= 2) { currentStatus.PW2 = currentStatus.PW1; }
    else { currentStatus.PW2 = 0; }
    if(maxInjOutputs >= 3) { currentStatus.PW3 = currentStatus.PW1; }
    else { currentStatus.PW3 = 0; }
    if(maxInjOutputs >= 4) { currentStatus.PW4 = currentStatus.PW1; }
    else { currentStatus.PW4 = 0; }
    if(maxInjOutputs >= 5) { currentStatus.PW5 = currentStatus.PW1; }
    else { currentStatus.PW5 = 0; }
    if(maxInjOutputs >= 6) { currentStatus.PW6 = currentStatus.PW1; }
    else { currentStatus.PW6 = 0; }
    if(maxInjOutputs >= 7) { currentStatus.PW7 = currentStatus.PW1; }
    else { currentStatus.PW7 = 0; }
    if(maxInjOutputs >= 8) { currentStatus.PW8 = currentStatus.PW1; }
    else { currentStatus.PW8 = 0; }

    BIT_CLEAR(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE); //Clear the staging active flag
    
  } 

}

void checkLaunchAndFlatShift()
{
  //Check for launching/flat shift (clutch) based on the current and previous clutch states
  previousClutchTrigger = clutchTrigger;
  //Only check for pinLaunch if any function using it is enabled. Else pins might break starting a board
  if(configPage6.flatSEnable || configPage6.launchEnabled)
  {
    if(configPage6.launchHiLo > 0) { clutchTrigger = digitalRead(pinLaunch); }
    else { clutchTrigger = !digitalRead(pinLaunch); }
  }
  if(clutchTrigger && (previousClutchTrigger != clutchTrigger) ) { currentStatus.clutchEngagedRPM = currentStatus.RPM; } //Check whether the clutch has been engaged or disengaged and store the current RPM if so

  //Default flags to off
  currentStatus.launchingHard = false; 
  BIT_CLEAR(currentStatus.spark, BIT_SPARK_HLAUNCH); 
  currentStatus.flatShiftingHard = false;

  if (configPage6.launchEnabled && clutchTrigger && (currentStatus.clutchEngagedRPM < ((unsigned int)(configPage6.flatSArm) * 100)) && (currentStatus.TPS >= configPage10.lnchCtrlTPS) ) 
  { 
    //Check whether RPM is above the launch limit
    uint16_t launchRPMLimit = (configPage6.lnchHardLim * 100);
    if( (configPage2.hardCutType == HARD_CUT_ROLLING) ) { launchRPMLimit += (configPage15.rollingProtRPMDelta[0] * 10); } //Add the rolling cut delta if enabled (Delta is a negative value)

    if(currentStatus.RPM > launchRPMLimit)
    {
      //HardCut rev limit for 2-step launch control.
      currentStatus.launchingHard = true; 
      BIT_SET(currentStatus.spark, BIT_SPARK_HLAUNCH); 
    }
  } 
  else 
  { 
    //If launch is not active, check whether flat shift should be active
    if(configPage6.flatSEnable && clutchTrigger && (currentStatus.clutchEngagedRPM >= ((unsigned int)(configPage6.flatSArm * 100)) ) ) 
    { 
      uint16_t flatRPMLimit = currentStatus.clutchEngagedRPM;
      if( (configPage2.hardCutType == HARD_CUT_ROLLING) ) { flatRPMLimit += (configPage15.rollingProtRPMDelta[0] * 10); } //Add the rolling cut delta if enabled (Delta is a negative value)

      if(currentStatus.RPM > flatRPMLimit)
      {
        //Flat shift rev limit
        currentStatus.flatShiftingHard = true;
      }
    }
  }
}
