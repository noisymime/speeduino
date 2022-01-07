/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/** @file
Corrections to injection pulsewidth.
The corrections functions in this file affect the fuel pulsewidth (Either increasing or decreasing)
based on factors other than the VE lookup.

These factors include:
- Temperature (Warmup Enrichment and After Start Enrichment)
- Acceleration/Decelleration
- Flood clear mode
- etc.

Most correction functions return value 100 (like 100% == 1) for no need for correction.

There are 2 top level functions that call more detailed corrections for Fuel and Ignition respectively:
- @ref correctionsFuel() - All fuel related corrections
- @ref correctionsIgn() - All ignition related corrections
*/
//************************************************************************************************************

#include "globals.h"
#include "corrections.h"
#include "speeduino.h"
#include "timers.h"
#include "maths.h"
#include "sensors.h"
#include "src/PID_v1/PID_v1.h"

long PID_O2, PID_output, PID_AFRTarget;
/** Instance of the PID object in case that algorithm is used (Always instantiated).
* Needs to be global as it maintains state outside of each function call.
* Comes from Arduino (?) PID library.
*/
PID egoPID(&PID_O2, &PID_output, &PID_AFRTarget, configPage6.egoKP, configPage6.egoKI, configPage6.egoKD, REVERSE);

int MAP_rateOfChange;
int TPS_rateOfChange;
byte activateMAPDOT; //The mapDOT value seen when the MAE was activated. 
byte activateTPSDOT; //The tpsDOT value seen when the MAE was activated.

uint16_t AFRnextCycle;
unsigned long knockStartTime;
byte lastKnockCount;
int16_t knockWindowMin; //The current minimum crank angle for a knock pulse to be valid
int16_t knockWindowMax;//The current maximum crank angle for a knock pulse to be valid
uint16_t aseTaperStart;
uint16_t dfcoStart;
uint16_t idleAdvStart;

/** Initialize instances and vars related to corrections (at ECU boot-up).
 */
void initialiseCorrections()
{
  egoPID.SetMode(AUTOMATIC); //Turn O2 PID on
  currentStatus.flexIgnCorrection = 0;
  currentStatus.egoCorrection = 100; //Default value of no adjustment must be set to avoid randomness on first correction cycle after startup
  AFRnextCycle = 0;
  currentStatus.knockActive = false;
  currentStatus.battery10 = 125; //Set battery voltage to sensible value for dwell correction for "flying start" (else ignition gets suprious pulses after boot)  
}

/** Dispatch calculations for all fuel related corrections.
Calls all the other corrections functions and combines their results.
This is the only function that should be called from anywhere outside the file
*/
uint16_t correctionsFuel()
{
  #define MAX_CORRECTIONS 3 //The maximum number of corrections allowed before the sum is reprocessed
  uint32_t sumCorrections = 100;
  byte activeCorrections = 0;
  uint16_t result; //temporary variable to store the result of each corrections function

  //The values returned by each of the correction functions are multipled together and then divided back to give a single 0-255 value.
  currentStatus.wueCorrection = correctionWUE();
  if (currentStatus.wueCorrection != 100) { sumCorrections = (sumCorrections * currentStatus.wueCorrection); activeCorrections++; }

  result = correctionASE();
  if (result != 100) { sumCorrections = (sumCorrections * result); activeCorrections++; }

  result = correctionCranking();
  if (result != 100) { sumCorrections = (sumCorrections * result); activeCorrections++; }
  if (activeCorrections == MAX_CORRECTIONS) { sumCorrections = sumCorrections / powint(100,activeCorrections); activeCorrections = 0; } // Need to check this to ensure that sumCorrections doesn't overflow. Can occur when the number of corrections is greater than 3 (Which is 100^4) as 100^5 can overflow

  currentStatus.AEamount = correctionAccel();
  if (configPage2.aeApplyMode == AE_MODE_MULTIPLIER)
  {
  if (currentStatus.AEamount != 100) { sumCorrections = (sumCorrections * currentStatus.AEamount); activeCorrections++; }
  if (activeCorrections == MAX_CORRECTIONS) { sumCorrections = sumCorrections / powint(100,activeCorrections); activeCorrections = 0; }
  }

  result = correctionFloodClear();
  if (result != 100) { sumCorrections = (sumCorrections * result); activeCorrections++; }
  if (activeCorrections == MAX_CORRECTIONS) { sumCorrections = sumCorrections / powint(100,activeCorrections); activeCorrections = 0; }

  currentStatus.egoCorrection = correctionAFRClosedLoop();
  if (currentStatus.egoCorrection != 100) { sumCorrections = (sumCorrections * currentStatus.egoCorrection); activeCorrections++; }
  if (activeCorrections == MAX_CORRECTIONS) { sumCorrections = sumCorrections / powint(100,activeCorrections); activeCorrections = 0; }

  currentStatus.batCorrection = correctionBatVoltage();
  if (configPage2.battVCorMode == BATTV_COR_MODE_OPENTIME)
  {
    inj_opentime_uS = configPage2.injOpen * currentStatus.batCorrection; // Apply voltage correction to injector open time.
  }
  if (configPage2.battVCorMode == BATTV_COR_MODE_WHOLE)
  {
    if (currentStatus.batCorrection != 100) { sumCorrections = (sumCorrections * currentStatus.batCorrection); activeCorrections++; }
    if (activeCorrections == MAX_CORRECTIONS) { sumCorrections = sumCorrections / powint(100,activeCorrections); activeCorrections = 0; }    
  }

  currentStatus.iatCorrection = correctionIATDensity();
  if (currentStatus.iatCorrection != 100) { sumCorrections = (sumCorrections * currentStatus.iatCorrection); activeCorrections++; }
  if (activeCorrections == MAX_CORRECTIONS) { sumCorrections = sumCorrections / powint(100,activeCorrections); activeCorrections = 0; }

  currentStatus.baroCorrection = correctionBaro();
  if (currentStatus.baroCorrection != 100) { sumCorrections = (sumCorrections * currentStatus.baroCorrection); activeCorrections++; }
  if (activeCorrections == MAX_CORRECTIONS) { sumCorrections = sumCorrections / powint(100,activeCorrections); activeCorrections = 0; }

  currentStatus.flexCorrection = correctionFlex();
  if (currentStatus.flexCorrection != 100) { sumCorrections = (sumCorrections * currentStatus.flexCorrection); activeCorrections++; }
  if (activeCorrections == MAX_CORRECTIONS) { sumCorrections = sumCorrections / powint(100,activeCorrections); activeCorrections = 0; }

  currentStatus.fuelTempCorrection = correctionFuelTemp();
  if (currentStatus.fuelTempCorrection != 100) { sumCorrections = (sumCorrections * currentStatus.fuelTempCorrection); activeCorrections++; }
  if (activeCorrections == MAX_CORRECTIONS) { sumCorrections = sumCorrections / powint(100,activeCorrections); activeCorrections = 0; }

  currentStatus.launchCorrection = correctionLaunch();
  if (currentStatus.launchCorrection != 100) { sumCorrections = (sumCorrections * currentStatus.launchCorrection); activeCorrections++; }

  bitWrite(currentStatus.status1, BIT_STATUS1_DFCO, correctionDFCO());
  if ( BIT_CHECK(currentStatus.status1, BIT_STATUS1_DFCO) == 1 ) { sumCorrections = 0; }

  sumCorrections = sumCorrections / powint(100,activeCorrections);

  if(sumCorrections > 1500) { sumCorrections = 1500; } //This is the maximum allowable increase during cranking
  return (uint16_t)sumCorrections;
}

/*
correctionsTotal() calls all the other corrections functions and combines their results.
This is the only function that should be called from anywhere outside the file
*/
static inline byte correctionsFuel_new()
{
  uint32_t sumCorrections = 100;
  byte numCorrections = 0;

  //The values returned by each of the correction functions are multipled together and then divided back to give a single 0-255 value.
  currentStatus.wueCorrection = correctionWUE(); numCorrections++;
  uint16_t correctionASEvalue = correctionASE(); numCorrections++;
  uint16_t correctionCrankingValue = correctionCranking(); numCorrections++;
  currentStatus.AEamount = correctionAccel(); numCorrections++;
  uint8_t correctionFloodClearValue = correctionFloodClear(); numCorrections++;
  currentStatus.egoCorrection = correctionAFRClosedLoop(); numCorrections++;

  currentStatus.batCorrection = correctionBatVoltage(); numCorrections++;
  currentStatus.iatCorrection = correctionIATDensity(); numCorrections++;
  currentStatus.baroCorrection = correctionBaro(); numCorrections++; 
  currentStatus.flexCorrection = correctionFlex(); numCorrections++;
  currentStatus.launchCorrection = correctionLaunch(); numCorrections++;

  bitWrite(currentStatus.status1, BIT_STATUS1_DFCO, correctionDFCO());
  if ( BIT_CHECK(currentStatus.status1, BIT_STATUS1_DFCO) == 1 ) { sumCorrections = 0; }

  sumCorrections = currentStatus.wueCorrection \
                  + correctionASEvalue \
                  + correctionCrankingValue \
                  + currentStatus.AEamount \
                  + correctionFloodClearValue \
                  + currentStatus.batCorrection \
                  + currentStatus.iatCorrection \
                  + currentStatus.baroCorrection \
                  + currentStatus.flexCorrection \
                  + currentStatus.launchCorrection;
  return (sumCorrections);

}

/** Warm Up Enrichment (WUE) corrections.
Uses a 2D enrichment table (WUETable) where the X axis is engine temp and the Y axis is the amount of extra fuel to add
*/
byte correctionWUE()
{
  byte WUEValue;
  //Possibly reduce the frequency this runs at (Costs about 50 loops per second)
  //if (currentStatus.coolant > (WUETable.axisX[9] - CALIBRATION_TEMPERATURE_OFFSET))
  if (currentStatus.coolant > (table2D_getAxisValue(&WUETable, 9) - CALIBRATION_TEMPERATURE_OFFSET))
  {
    //This prevents us doing the 2D lookup if we're already up to temp
    BIT_CLEAR(currentStatus.engine, BIT_ENGINE_WARMUP);
    WUEValue = table2D_getRawValue(&WUETable, 9);
  }
  else
  {
    BIT_SET(currentStatus.engine, BIT_ENGINE_WARMUP);
    WUEValue = table2D_getValue(&WUETable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
  }

  return WUEValue;
}

/** Cranking Enrichment corrections.
Additional fuel % to be added when the engine is cranking
*/
uint16_t correctionCranking()
{
  uint16_t crankingValue = 100;
  //Check if we are actually cranking
  if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
  {
    crankingValue = table2D_getValue(&crankingEnrichTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
    crankingValue = (uint16_t) crankingValue * 5; //multiplied by 5 to get range from 0% to 1275%
  }
  
  //If we're not cranking, check if if cranking enrichment tapering to ASE should be done
  else if ( (uint32_t) runSecsX10 <= configPage10.crankingEnrichTaper)
  {
    crankingValue = table2D_getValue(&crankingEnrichTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
    crankingValue = (uint16_t) crankingValue * 5; //multiplied by 5 to get range from 0% to 1275%
    //Taper start value needs to account for ASE that is now running, so total correction does not increase when taper begins
    unsigned long taperStart = (unsigned long) crankingValue * 100 / currentStatus.ASEValue;
    crankingValue = (uint16_t) map(runSecsX10, 0, configPage10.crankingEnrichTaper, taperStart, 100); //Taper from start value to 100%
    if (crankingValue < 100) { crankingValue = 100; } //Sanity check
  }
  return crankingValue;
}

/** Afer Start Enrichment calculation.
 * This is a short period (Usually <20 seconds) immediately after the engine first fires (But not when cranking)
 * where an additional amount of fuel is added (Over and above the WUE amount).
 * 
 * @return uint8_t The After Start Enrichment modifier as a %. 100% = No modification. 
 */
byte correctionASE()
{
  int16_t ASEValue;
  //Two checks are requiredL:
  //1) Is the engine run time less than the configured ase time
  //2) Make sure we're not still cranking
  if ( BIT_CHECK(TIMER_mask, BIT_TIMER_10HZ) || (currentStatus.ASEValue == 0) )
  {
    if ( (currentStatus.runSecs < (table2D_getValue(&ASECountTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET))) && !(BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) )
    {
      BIT_SET(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as active.
      ASEValue = 100 + table2D_getValue(&ASETable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
      aseTaperStart = runSecsX10;
    }
    else
    {
      if ( (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) && ((runSecsX10 - aseTaperStart) < configPage2.aseTaperTime) ) //Cranking check needs to be here also, so cranking and afterstart enrichments won't run simultaneously
      {
        BIT_SET(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as active.
        ASEValue = 100 + map((runSecsX10 - aseTaperStart), 0, configPage2.aseTaperTime,\
          table2D_getValue(&ASETable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET), 0);
      }
      else
      {
        BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as inactive.
        ASEValue = 100;
      }
    }
    
    //Safety checks
    if(ASEValue > 255) { ASEValue = 255; }
    if(ASEValue < 0) { ASEValue = 0; }
    currentStatus.ASEValue = (byte)ASEValue;
  }
  return currentStatus.ASEValue;
}

/** Acceleration enrichment correction calculation.
 * 
 * Calculates the % change of the throttle over time (%/second) and performs a lookup based on this
 * Coolant-based modifier is applied on the top of this.
 * When the enrichment is turned on, it runs at that amount for a fixed period of time (taeTime)
 * 
 * @return uint16_t The Acceleration enrichment modifier as a %. 100% = No modification.
 * 
 * As the maximum enrichment amount is +255% and maximum cold adjustment for this is 255%, the overall return value
 * from this function can be 100+(255*255/100)=750. Hence this function returns a uint16_t rather than byte.
 */
uint16_t correctionAccel()
{
  int16_t accelValue = 100;
  int16_t MAP_change = 0;
  int8_t TPS_change = 0;

  if(configPage2.aeMode == AE_MODE_MAP)
  {
    //Get the MAP rate change
    MAP_change = (currentStatus.MAP - MAPlast);
    MAP_rateOfChange = ldiv(1000000, (MAP_time - MAPlast_time)).quot * MAP_change; //This is the % per second that the TPS has moved
    //MAP_rateOfChange = 15 * MAP_change; //This is the kpa per second that the MAP has moved
    if(MAP_rateOfChange >= 0) { currentStatus.mapDOT = MAP_rateOfChange / 10; } //The MAE bins are divided by 10 in order to allow them to be stored in a byte. Faster as this than divu10
    else { currentStatus.mapDOT = 0; } //Prevent overflow as mapDOT is signed
  }
  else if(configPage2.aeMode == AE_MODE_TPS)
  {
    //Get the TPS rate change
    TPS_change = (currentStatus.TPS - TPSlast);
    //TPS_rateOfChange = ldiv(1000000, (TPS_time - TPSlast_time)).quot * TPS_change; //This is the % per second that the TPS has moved
    TPS_rateOfChange = TPS_READ_FREQUENCY * TPS_change; //This is the % per second that the TPS has moved
    if(TPS_rateOfChange >= 0) { currentStatus.tpsDOT = TPS_rateOfChange / 20; } //The TAE bins are divided by 10 in order to allow them to be stored in a byte and then by 2 due to TPS being 0.5% resolution (0-200)
    else { currentStatus.tpsDOT = 0; } //Prevent overflow as tpsDOT is signed
  }
  

  //First, check whether the accel. enrichment is already running
  if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC) )
  {
    //If it is currently running, check whether it should still be running or whether it's reached it's end time
    if( micros_safe() >= currentStatus.AEEndTime )
    {
      //Time to turn enrichment off
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC);
      currentStatus.AEamount = 0;
      accelValue = 100;

      //Reset the relevant DOT value to 0
      if(configPage2.aeMode == AE_MODE_MAP) { currentStatus.mapDOT = 0; }
      else if(configPage2.aeMode == AE_MODE_TPS) { currentStatus.tpsDOT = 0; }
    }
    else
    {
      //Enrichment still needs to keep running. 
      //Simply return the total TAE amount
      accelValue = currentStatus.AEamount;

      //Need to check whether the accel amount has increased from when AE was turned on
      //If the accel amount HAS increased, we clear the current enrich phase and a new one will be started below
      if( (configPage2.aeMode == AE_MODE_MAP) && (currentStatus.mapDOT > activateMAPDOT) ) { BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC); }
      else if( (configPage2.aeMode == AE_MODE_TPS) && (currentStatus.tpsDOT > activateTPSDOT) ) { BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC); }
    }
  }

  //else
  if( !BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC) ) //Need to check this again as it may have been changed in the above section
  {
    if(configPage2.aeMode == AE_MODE_MAP)
    {
      if (MAP_change <= 2)
      {
        accelValue = 100;
        currentStatus.mapDOT = 0;
      }
      else
      {
        //If MAE isn't currently turned on, need to check whether it needs to be turned on
        if (MAP_rateOfChange > configPage2.maeThresh)
        {
          BIT_SET(currentStatus.engine, BIT_ENGINE_ACC); //Mark accleration enrichment as active.
          activateMAPDOT = currentStatus.mapDOT;
          currentStatus.AEEndTime = micros_safe() + ((unsigned long)configPage2.aeTime * 10000); //Set the time in the future where the enrichment will be turned off. taeTime is stored as mS / 10, so multiply it by 100 to get it in uS
          accelValue = table2D_getValue(&maeTable, currentStatus.mapDOT);

          //Apply the RPM taper to the above
          //The RPM settings are stored divided by 100:
          uint16_t trueTaperMin = configPage2.aeTaperMin * 100;
          uint16_t trueTaperMax = configPage2.aeTaperMax * 100;
          if (currentStatus.RPM > trueTaperMin)
          {
            if(currentStatus.RPM > trueTaperMax) { accelValue = 0; } //RPM is beyond taper max limit, so accel enrich is turned off
            else 
            {
              int16_t taperRange = trueTaperMax - trueTaperMin;
              int16_t taperPercent = ((currentStatus.RPM - trueTaperMin) * 100) / taperRange; //The percentage of the way through the RPM taper range
              accelValue = percentage((100-taperPercent), accelValue); //Calculate the above percentage of the calculated accel amount. 
            }
          }

          //Apply AE cold coolant modifier, if CLT is less than taper end temperature
          if ( currentStatus.coolant < (int)(configPage2.aeColdTaperMax - CALIBRATION_TEMPERATURE_OFFSET) )
          {
            //If CLT is less than taper min temp, apply full modifier on top of accelValue
            if ( currentStatus.coolant <= (int)(configPage2.aeColdTaperMin - CALIBRATION_TEMPERATURE_OFFSET) )
            {
              uint16_t accelValue_uint = (uint16_t) accelValue * configPage2.aeColdPct / 100;
              accelValue = (int16_t) accelValue_uint;
            }
            //If CLT is between taper min and max, taper the modifier value and apply it on top of accelValue
            else
            {
              int16_t taperRange = (int16_t) configPage2.aeColdTaperMax - configPage2.aeColdTaperMin;
              int16_t taperPercent = (int)((currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET - configPage2.aeColdTaperMin) * 100) / taperRange;
              int16_t coldPct = (int16_t) 100+ percentage((100-taperPercent), configPage2.aeColdPct-100);
              uint16_t accelValue_uint = (uint16_t) accelValue * coldPct / 100; //Potential overflow (if AE is large) without using uint16_t
              accelValue = (int16_t) accelValue_uint;
            }
          }
          
          accelValue = 100 + accelValue; //Add the 100 normalisation to the calculated amount
        } //MAE Threshold
      }
    }
    else if(configPage2.aeMode == AE_MODE_TPS)
    {
    
      //Check for deceleration (Deceleration adjustment not yet supported)
      //Also check for only very small movement (Movement less than or equal to 2% is ignored). This not only means we can skip the lookup, but helps reduce false triggering around 0-2% throttle openings
      if (TPS_change <= 2)
      {
        accelValue = 100;
        currentStatus.tpsDOT = 0;
      }
      else
      {
        //If TAE isn't currently turned on, need to check whether it needs to be turned on
        if (TPS_rateOfChange > configPage2.taeThresh)
        {
          BIT_SET(currentStatus.engine, BIT_ENGINE_ACC); //Mark accleration enrichment as active.
          activateTPSDOT = currentStatus.tpsDOT;
          currentStatus.AEEndTime = micros_safe() + ((unsigned long)configPage2.aeTime * 10000); //Set the time in the future where the enrichment will be turned off. taeTime is stored as mS / 10, so multiply it by 100 to get it in uS
          accelValue = table2D_getValue(&taeTable, currentStatus.tpsDOT);

          //Apply the RPM taper to the above
          //The RPM settings are stored divided by 100:
          uint16_t trueTaperMin = configPage2.aeTaperMin * 100;
          uint16_t trueTaperMax = configPage2.aeTaperMax * 100;
          if (currentStatus.RPM > trueTaperMin)
          {
            if(currentStatus.RPM > trueTaperMax) { accelValue = 0; } //RPM is beyond taper max limit, so accel enrich is turned off
            else 
            {
              int16_t taperRange = trueTaperMax - trueTaperMin;
              int16_t taperPercent = ((currentStatus.RPM - trueTaperMin) * 100) / taperRange; //The percentage of the way through the RPM taper range
              accelValue = percentage((100-taperPercent), accelValue); //Calculate the above percentage of the calculated accel amount. 
            }
          }

          //Apply AE cold coolant modifier, if CLT is less than taper end temperature
          if ( currentStatus.coolant < (int)(configPage2.aeColdTaperMax - CALIBRATION_TEMPERATURE_OFFSET) )
          {
            //If CLT is less than taper min temp, apply full modifier on top of accelValue
            if ( currentStatus.coolant <= (int)(configPage2.aeColdTaperMin - CALIBRATION_TEMPERATURE_OFFSET) )
            {
              uint16_t accelValue_uint = (uint16_t) accelValue * configPage2.aeColdPct / 100;
              accelValue = (int16_t) accelValue_uint;
            }
            //If CLT is between taper min and max, taper the modifier value and apply it on top of accelValue
            else
            {
              int16_t taperRange = (int16_t) configPage2.aeColdTaperMax - configPage2.aeColdTaperMin;
              int16_t taperPercent = (int)((currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET - configPage2.aeColdTaperMin) * 100) / taperRange;
              int16_t coldPct = (int16_t) 100+ percentage((100-taperPercent), configPage2.aeColdPct-100);
              uint16_t accelValue_uint = (uint16_t) accelValue * coldPct / 100; //Potential overflow (if AE is large) without using uint16_t
              accelValue = (int16_t) accelValue_uint;
            }
          }

          accelValue = 100 + accelValue; //Add the 100 normalisation to the calculated amount
        } //TAE Threshold
      } //TPS change > 2
    } //AE Mode
  } //AE active

  return accelValue;
}

/** Simple check to see whether we are cranking with the TPS above the flood clear threshold.
@return 100 (not cranking and thus no need for flood-clear) or 0 (Engine cranking and TPS above @ref config4.floodClear limit).
*/
byte correctionFloodClear()
{
  byte floodValue = 100;
  if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
  {
    //Engine is currently cranking, check what the TPS is
    if(currentStatus.TPS >= configPage4.floodClear)
    {
      //Engine is cranking and TPS is above threshold. Cut all fuel
      floodValue = 0;
    }
  }
  return floodValue;
}

/** Battery Voltage correction.
Uses a 2D enrichment table (WUETable) where the X axis is engine temp and the Y axis is the amount of extra fuel to add.
*/
byte correctionBatVoltage()
{
  byte batValue = 100;
  batValue = table2D_getValue(&injectorVCorrectionTable, currentStatus.battery10);
  return batValue;
}

/** Simple temperature based corrections lookup based on the inlet air temperature (IAT).
This corrects for changes in air density from movement of the temperature.
*/
byte correctionIATDensity()
{
  byte IATValue = 100;
  IATValue = table2D_getValue(&IATDensityCorrectionTable, currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET); //currentStatus.IAT is the actual temperature, values in IATDensityCorrectionTable.axisX are temp+offset

  return IATValue;
}

/** Correction for current baromtetric / ambient pressure.
 * @returns A percentage value indicating the amount the fueling should be changed based on the barometric reading. 100 = No change. 110 = 10% increase. 90 = 10% decrease
 */
byte correctionBaro()
{
  byte baroValue = 100;
  baroValue = table2D_getValue(&baroFuelTable, currentStatus.baro);

  return baroValue;
}

/** Launch control has a setting to increase the fuel load to assist in bringing up boost.
This simple check applies the extra fuel if we're currently launching
*/
byte correctionLaunch()
{
  byte launchValue = 100;
  if(currentStatus.launchingHard || currentStatus.launchingSoft) { launchValue = (100 + configPage6.lnchFuelAdd); }

  return launchValue;
}

/*
 * Returns true if decelleration fuel cutoff should be on, false if its off
 */
bool correctionDFCO()
{
  bool DFCOValue = false;
  if ( configPage2.dfcoEnabled == 1 )
  {
    if ( BIT_CHECK(currentStatus.status1, BIT_STATUS1_DFCO) == 1 ) 
    {
      DFCOValue = ( currentStatus.RPM > ( configPage4.dfcoRPM * 10) ) && ( currentStatus.TPS < configPage4.dfcoTPSThresh ); 
      if ( DFCOValue == false) { dfcoStart = 0; }
    }
    else 
    {
      if ( ( currentStatus.coolant >= (int)(configPage2.dfcoMinCLT - CALIBRATION_TEMPERATURE_OFFSET) ) && ( currentStatus.RPM > (unsigned int)( (configPage4.dfcoRPM * 10) + configPage4.dfcoHyster) ) && ( currentStatus.TPS < configPage4.dfcoTPSThresh ) )
      {
        if ( dfcoStart == 0 ) { dfcoStart = runSecsX10; }
        if( (runSecsX10 - dfcoStart) > configPage2.dfcoDelay ) { DFCOValue = true; }
      }
      else { dfcoStart = 0; } //Prevent future activation right away if previous time wasn't activated
    } // DFCO active check
  } // DFCO enabled check
  return DFCOValue;
}

/** Flex fuel adjustment to vary fuel based on ethanol content.
 * The amount of extra fuel required is a linear relationship based on the % of ethanol.
*/
byte correctionFlex()
{
  byte flexValue = 100;

  if (configPage2.flexEnabled == 1)
  {
    flexValue = table2D_getValue(&flexFuelTable, currentStatus.ethanolPct);
  }
  return flexValue;
}

/*
 * Fuel temperature adjustment to vary fuel based on fuel temperature reading
*/
byte correctionFuelTemp()
{
  byte fuelTempValue = 100;

  if (configPage2.flexEnabled == 1)
  {
    fuelTempValue = table2D_getValue(&fuelTempTable, currentStatus.fuelTemp + CALIBRATION_TEMPERATURE_OFFSET);
  }
  return fuelTempValue;
}

/** Lookup the AFR target table and perform either a simple or PID adjustment based on this.

Simple (Best suited to narrowband sensors):
If the O2 sensor reports that the mixture is lean/rich compared to the desired AFR target, it will make a 1% adjustment
It then waits egoDelta number of ignition events and compares O2 against the target table again. If it is still lean/rich then the adjustment is increased to 2%.

This continues until either:
- the O2 reading flips from lean to rich, at which point the adjustment cycle starts again at 1% or
- the adjustment amount increases to egoLimit at which point it stays at this level until the O2 state (rich/lean) changes

PID (Best suited to wideband sensors):

*/
byte correctionAFRClosedLoop()
{
  byte AFRValue = 100;
  
  if( (configPage6.egoType > 0) || (configPage2.incorporateAFR == true) ) //afrTarget value lookup must be done if O2 sensor is enabled, and always if incorporateAFR is enabled
  {
    currentStatus.afrTarget = currentStatus.O2; //Catch all incase the below doesn't run. This prevents the Include AFR option from doing crazy things if the AFR target conditions aren't met. This value is changed again below if all conditions are met.

    //Determine whether the Y axis of the AFR target table tshould be MAP (Speed-Density) or TPS (Alpha-N)
    //Note that this should only run after the sensor warmup delay when using Include AFR option, but on Incorporate AFR option it needs to be done at all times
    if( (currentStatus.runSecs > configPage6.ego_sdelay) || (configPage2.incorporateAFR == true) ) { currentStatus.afrTarget = get3DTableValue(&afrTable, currentStatus.fuelLoad, currentStatus.RPM); } //Perform the target lookup
  }
  
  if( configPage6.egoType > 0 ) //egoType of 0 means no O2 sensor
  {
    AFRValue = currentStatus.egoCorrection; //Need to record this here, just to make sure the correction stays 'on' even if the nextCycle count isn't ready
    
    if(ignitionCount >= AFRnextCycle)
    {
      AFRnextCycle = ignitionCount + configPage6.egoCount; //Set the target ignition event for the next calculation
        
      //Check all other requirements for closed loop adjustments
      if( (currentStatus.coolant > (int)(configPage6.egoTemp - CALIBRATION_TEMPERATURE_OFFSET)) && (currentStatus.RPM > (unsigned int)(configPage6.egoRPM * 100)) && (currentStatus.TPS < configPage6.egoTPSMax) && (currentStatus.O2 < configPage6.ego_max) && (currentStatus.O2 > configPage6.ego_min) && (currentStatus.runSecs > configPage6.ego_sdelay) &&  (BIT_CHECK(currentStatus.status1, BIT_STATUS1_DFCO) == 0) )
      {

        //Check which algorithm is used, simple or PID
        if (configPage6.egoAlgorithm == EGO_ALGORITHM_SIMPLE)
        {
          //*************************************************************************************************************************************
          //Simple algorithm
          if(currentStatus.O2 > currentStatus.afrTarget)
          {
            //Running lean
            if(currentStatus.egoCorrection < (100 + configPage6.egoLimit) ) //Fueling adjustment must be at most the egoLimit amount (up or down)
            {
              AFRValue = (currentStatus.egoCorrection + 1); //Increase the fueling by 1%
            }
            else { AFRValue = currentStatus.egoCorrection; } //Means we're at the maximum adjustment amount, so simply return that again
          }
          else if(currentStatus.O2 < currentStatus.afrTarget)
          {
            //Running Rich
            if(currentStatus.egoCorrection > (100 - configPage6.egoLimit) ) //Fueling adjustment must be at most the egoLimit amount (up or down)
            {
              AFRValue = (currentStatus.egoCorrection - 1); //Decrease the fueling by 1%
            }
            else { AFRValue = currentStatus.egoCorrection; } //Means we're at the maximum adjustment amount, so simply return that again
          }
          else { AFRValue = currentStatus.egoCorrection; } //Means we're already right on target

        }
        else if(configPage6.egoAlgorithm == EGO_ALGORITHM_PID)
        {
          //*************************************************************************************************************************************
          //PID algorithm
          egoPID.SetOutputLimits((long)(-configPage6.egoLimit), (long)(configPage6.egoLimit)); //Set the limits again, just incase the user has changed them since the last loop. Note that these are sent to the PID library as (Eg:) -15 and +15
          egoPID.SetTunings(configPage6.egoKP, configPage6.egoKI, configPage6.egoKD); //Set the PID values again, just incase the user has changed them since the last loop
          PID_O2 = (long)(currentStatus.O2);
          PID_AFRTarget = (long)(currentStatus.afrTarget);

          bool PID_compute = egoPID.Compute();
          //currentStatus.egoCorrection = 100 + PID_output;
          if(PID_compute == true) { AFRValue = 100 + PID_output; }
          
        }
        else { AFRValue = 100; } // Occurs if the egoAlgorithm is set to 0 (No Correction)
      } //Multi variable check 
      else { AFRValue = 100; } // If multivariable check fails disable correction
    } //Ignition count check
  } //egoType

  return AFRValue; //Catch all (Includes when AFR target = current AFR
}

//******************************** IGNITION ADVANCE CORRECTIONS ********************************
/** Dispatch calculations for all ignition related corrections.
 * @param base_advance - Base ignition advance (deg. ?)
 * @return Advance considering all (~12) individual corrections
 */
int8_t correctionsIgn(int8_t base_advance)
{
  int8_t advance;
  advance = correctionFlexTiming(base_advance);
  advance = correctionWMITiming(advance);
  advance = correctionIATretard(advance);
  advance = correctionCLTadvance(advance);
  advance = correctionIdleAdvance(advance);
  advance = correctionSoftRevLimit(advance);
  advance = correctionNitrous(advance);
  advance = correctionSoftLaunch(advance);
  advance = correctionSoftFlatShift(advance);
  advance = correctionKnock(advance);

  //Fixed timing check must go last
  advance = correctionFixedTiming(advance);
  advance = correctionCrankingFixedTiming(advance); //This overrides the regular fixed timing, must come last

  return advance;
}
/** Correct ignition timing to configured fixed value.
 * Must be called near end to override all other corrections.
 */
int8_t correctionFixedTiming(int8_t advance)
{
  int8_t ignFixValue = advance;
  if (configPage2.fixAngEnable == 1) { ignFixValue = configPage4.FixAng; } //Check whether the user has set a fixed timing angle
  return ignFixValue;
}
/** Correct ignition timing to configured fixed value to use during craning.
 * Must be called near end to override all other corrections.
 */
int8_t correctionCrankingFixedTiming(int8_t advance)
{
  int8_t ignCrankFixValue = advance;
  if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) ) { ignCrankFixValue = configPage4.CrankAng; } //Use the fixed cranking ignition angle
  return ignCrankFixValue;
}

int8_t correctionFlexTiming(int8_t advance)
{
  int16_t ignFlexValue = advance;
  if( configPage2.flexEnabled == 1 ) //Check for flex being enabled
  {
    ignFlexValue = (int16_t) table2D_getValue(&flexAdvTable, currentStatus.ethanolPct) - OFFSET_IGNITION; //Negative values are achieved with offset
    currentStatus.flexIgnCorrection = (int8_t) ignFlexValue; //This gets cast to a signed 8 bit value to allows for negative advance (ie retard) values here. 
    ignFlexValue = (int8_t) advance + currentStatus.flexIgnCorrection;
  }
  return (int8_t) ignFlexValue;
}

int8_t correctionWMITiming(int8_t advance)
{
  if( (configPage10.wmiEnabled >= 1) && (configPage10.wmiAdvEnabled == 1) && !BIT_CHECK(currentStatus.status4, BIT_STATUS4_WMI_EMPTY) ) //Check for wmi being enabled
  {
    if( (currentStatus.TPS >= configPage10.wmiTPS) && (currentStatus.RPM >= configPage10.wmiRPM) && (currentStatus.MAP/2 >= configPage10.wmiMAP) && ((currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET) >= configPage10.wmiIAT) )
    {
      return (int16_t) advance + table2D_getValue(&wmiAdvTable, currentStatus.MAP/2) - OFFSET_IGNITION; //Negative values are achieved with offset
    }
  }
  return advance;
}
/** Ignition correction for inlet air temperature (IAT).
 */
int8_t correctionIATretard(int8_t advance)
{
  int8_t advanceIATadjust = table2D_getValue(&IATRetardTable, currentStatus.IAT);

  return advance - advanceIATadjust;
}
/** Ignition correction for coolant temperature (CLT).
 */
int8_t correctionCLTadvance(int8_t advance)
{
  int8_t ignCLTValue = advance;
  //Adjust the advance based on CLT.
  int8_t advanceCLTadjust = (int16_t)(table2D_getValue(&CLTAdvanceTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET)) - 15;
  ignCLTValue = (advance + advanceCLTadjust);
  
  return ignCLTValue;
}
/** Ignition Idle advance correction.
 */
int8_t correctionIdleAdvance(int8_t advance)
{

  int8_t ignIdleValue = advance;
  //Adjust the advance based on idle target rpm.
  if( (configPage2.idleAdvEnabled >= 1) && (runSecsX10 >= (configPage2.idleAdvDelay * 5)) )
  {
    currentStatus.CLIdleTarget = (byte)table2D_getValue(&idleTargetTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //All temps are offset by 40 degrees
    int idleRPMdelta = (currentStatus.CLIdleTarget - (currentStatus.RPM / 10) ) + 50;
    // Limit idle rpm delta between -500rpm - 500rpm
    if(idleRPMdelta > 100) { idleRPMdelta = 100; }
    if(idleRPMdelta < 0) { idleRPMdelta = 0; }
    if( (currentStatus.RPMdiv100 < configPage2.idleAdvRPM) && ((configPage2.vssMode == 0) || (currentStatus.vss < configPage2.idleAdvVss))
    && (((configPage2.idleAdvAlgorithm == 0) && (currentStatus.TPS < configPage2.idleAdvTPS)) || ((configPage2.idleAdvAlgorithm == 1) && (currentStatus.CTPSActive == 1))) ) // closed throttle position sensor (CTPS) based idle state
    {
      if( (runSecsX10 - idleAdvStart) >= configPage9.idleAdvStartDelay )
      {
        int8_t advanceIdleAdjust = (int16_t)(table2D_getValue(&idleAdvanceTable, idleRPMdelta)) - 15;
        if(configPage2.idleAdvEnabled == 1) { ignIdleValue = (advance + advanceIdleAdjust); }
        else if(configPage2.idleAdvEnabled == 2) { ignIdleValue = advanceIdleAdjust; }
      }
    }
    else if( BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ) ) { idleAdvStart = runSecsX10; } //Only copy time at runSecsX10 update rate
  }
  return ignIdleValue;
}
/** Ignition soft revlimit correction.
 */
int8_t correctionSoftRevLimit(int8_t advance)
{
  byte ignSoftRevValue = advance;
  BIT_CLEAR(currentStatus.spark, BIT_SPARK_SFTLIM);

  if (configPage6.engineProtectType == PROTECT_CUT_IGN || configPage6.engineProtectType == PROTECT_CUT_BOTH) 
  {
    if (currentStatus.RPM > ((unsigned int)(configPage4.SoftRevLim) * 100) ) //Softcut RPM limit
    {
      if( (runSecsX10 - softStartTime) < configPage4.SoftLimMax )
      {
        BIT_SET(currentStatus.spark, BIT_SPARK_SFTLIM);
        if (configPage2.SoftLimitMode == SOFT_LIMIT_RELATIVE) { ignSoftRevValue = ignSoftRevValue - configPage4.SoftLimRetard; } //delay timing by configured number of degrees in relative mode
        else if (configPage2.SoftLimitMode == SOFT_LIMIT_FIXED) { ignSoftRevValue = configPage4.SoftLimRetard; } //delay timing to configured number of degrees in fixed mode
      }
    }  
    else if( BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ) ) { softStartTime = runSecsX10; } //Only copy time at runSecsX10 update rate
  }

  return ignSoftRevValue;
}
/** Ignition Nitrous oxide correction.
 */
int8_t correctionNitrous(int8_t advance)
{
  byte ignNitrous = advance;
  //Check if nitrous is currently active
  if(configPage10.n2o_enable > 0)
  {
    //Check which stage is running (if any)
    if( (currentStatus.nitrous_status == NITROUS_STAGE1) || (currentStatus.nitrous_status == NITROUS_BOTH) )
    {
      ignNitrous -= configPage10.n2o_stage1_retard;
    }
    if( (currentStatus.nitrous_status == NITROUS_STAGE2) || (currentStatus.nitrous_status == NITROUS_BOTH) )
    {
      ignNitrous -= configPage10.n2o_stage2_retard;
    }
  }

  return ignNitrous;
}
/** Ignition soft launch correction.
 */
int8_t correctionSoftLaunch(int8_t advance)
{
  byte ignSoftLaunchValue = advance;
  //SoftCut rev limit for 2-step launch control.
  if (configPage6.launchEnabled && clutchTrigger && (currentStatus.clutchEngagedRPM < ((unsigned int)(configPage6.flatSArm) * 100)) && (currentStatus.RPM > ((unsigned int)(configPage6.lnchSoftLim) * 100)) && (currentStatus.TPS >= configPage10.lnchCtrlTPS) )
  {
    currentStatus.launchingSoft = true;
    BIT_SET(currentStatus.spark, BIT_SPARK_SLAUNCH);
    ignSoftLaunchValue = configPage6.lnchRetard;
  }
  else
  {
    currentStatus.launchingSoft = false;
    BIT_CLEAR(currentStatus.spark, BIT_SPARK_SLAUNCH);
  }

  return ignSoftLaunchValue;
}
/** Ignition correction for soft flat shift.
 */
int8_t correctionSoftFlatShift(int8_t advance)
{
  int8_t ignSoftFlatValue = advance;

  if(configPage6.flatSEnable && clutchTrigger && (currentStatus.clutchEngagedRPM > ((unsigned int)(configPage6.flatSArm) * 100)) && (currentStatus.RPM > (currentStatus.clutchEngagedRPM - (configPage6.flatSSoftWin * 100) ) ) )
  {
    BIT_SET(currentStatus.spark2, BIT_SPARK2_FLATSS);
    ignSoftFlatValue = configPage6.flatSRetard;
  }
  else { BIT_CLEAR(currentStatus.spark2, BIT_SPARK2_FLATSS); }

  return ignSoftFlatValue;
}
/** Ignition knock (retard) correction.
 */
int8_t correctionKnock(int8_t advance)
{
  byte knockRetard = 0;

  //First check is to do the window calculations (ASsuming knock is enabled)
  if( configPage10.knock_mode != KNOCK_MODE_OFF )
  {
    knockWindowMin = table2D_getValue(&knockWindowStartTable, currentStatus.RPMdiv100);
    knockWindowMax = knockWindowMin + table2D_getValue(&knockWindowDurationTable, currentStatus.RPMdiv100);
  }


  if( (configPage10.knock_mode == KNOCK_MODE_DIGITAL)  )
  {
    //
    if(knockCounter > configPage10.knock_count)
    {
      if(currentStatus.knockActive == true)
      {
        //Knock retard is currently 
      }
      else
      {
        //Knock needs to be activated
        lastKnockCount = knockCounter;
        knockStartTime = micros();
        knockRetard = configPage10.knock_firstStep;
      }
    }

  }

  return advance - knockRetard;
}

/** Ignition Dwell Correction.
 */
uint16_t correctionsDwell(uint16_t dwell)
{
  uint16_t tempDwell = dwell;
  //Pull battery voltage based dwell correction and apply if needed
  currentStatus.dwellCorrection = table2D_getValue(&dwellVCorrectionTable, currentStatus.battery10);
  if (currentStatus.dwellCorrection != 100) { tempDwell = divs100(dwell) * currentStatus.dwellCorrection; }

  //Dwell limiter
  uint16_t dwellPerRevolution = tempDwell + (uint16_t)(configPage4.sparkDur * 100); //Spark duration is in mS*10. Multiple it by 100 to get spark duration in uS
  int8_t pulsesPerRevolution = 1;
  //Single channel spark mode is the only time there will be more than 1 pulse per revolution on any given output
  if( ( (configPage4.sparkMode == IGN_MODE_SINGLE) || (configPage4.sparkMode == IGN_MODE_ROTARY) ) && (configPage2.nCylinders > 1) ) //No point in running this for 1 cylinder engines
  {
    pulsesPerRevolution = (configPage2.nCylinders >> 1);
    dwellPerRevolution = dwellPerRevolution * pulsesPerRevolution;
  }

  if(dwellPerRevolution > revolutionTime)
  {
    //Possibly need some method of reducing spark duration here as well, but this is a start
    tempDwell = (revolutionTime / pulsesPerRevolution) - (configPage4.sparkDur * 100);
  }
  return tempDwell;
}
