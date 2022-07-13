
#include "globals.h"
#include "engineProtection.h"

byte checkEngineProtect()
{
  byte protectActive = 0;
  if(checkRevLimit() || checkBoostLimit() || checkOilPressureLimit() || checkAFRLimit() )
  {
    if( currentStatus.RPMdiv100 > configPage4.engineProtectMaxRPM ) { protectActive = 1; }
  }

  return protectActive;
}

byte checkRevLimit()
{
  //Hardcut RPM limit
  byte revLimiterActive = 0;
  BIT_CLEAR(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_RPM);
  BIT_CLEAR(currentStatus.spark, BIT_SPARK_HRDLIM);
  BIT_CLEAR(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_COOLANT);

  if (configPage6.engineProtectType != PROTECT_CUT_OFF) 
  {
    if(configPage9.hardRevMode == HARD_REV_FIXED)
    {
      if ( (currentStatus.RPMdiv100 >= configPage4.HardRevLim) || ((softLimitTime > configPage4.SoftLimMax) && (currentStatus.RPMdiv100 >= configPage4.SoftRevLim)) )
      { 
        BIT_SET(currentStatus.spark, BIT_SPARK_HRDLIM); //Legacy and likely to be removed at some point
        BIT_SET(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_RPM);
        revLimiterActive = 1; 
      } 
      else { BIT_CLEAR(currentStatus.spark, BIT_SPARK_HRDLIM); }
    }
    else if(configPage9.hardRevMode == HARD_REV_COOLANT )
    {
      int8_t coolantLimit = (int16_t)(table2D_getValue(&coolantProtectTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET));
      if(currentStatus.RPMdiv100 > coolantLimit)
      {
        BIT_SET(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_COOLANT);
        BIT_SET(currentStatus.spark, BIT_SPARK_HRDLIM); //Legacy and likely to be removed at some point
        BIT_SET(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_RPM);
        revLimiterActive = 1; 
      } 
    }
  }

  return revLimiterActive;
}

byte checkBoostLimit()
{
  byte boostLimitActive = 0;
  BIT_CLEAR(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_MAP);
  BIT_CLEAR(currentStatus.spark, BIT_SPARK_BOOSTCUT);
  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_BOOSTCUT);

  if (configPage6.engineProtectType != PROTECT_CUT_OFF) {
    //Boost cutoff is very similar to launchControl, but with a check against MAP rather than a switch
    if( (configPage6.boostCutEnabled > 0) && (currentStatus.MAP > (configPage6.boostLimit * 2)) ) //The boost limit is divided by 2 to allow a limit up to 511kPa
    {
      boostLimitActive = 1;
      BIT_SET(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_MAP);
      /*
      switch(configPage6.boostCutType)
      {
        case 1:
          BIT_SET(currentStatus.spark, BIT_SPARK_BOOSTCUT);
          BIT_CLEAR(currentStatus.status1, BIT_STATUS1_BOOSTCUT);
          BIT_SET(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_MAP);
          break;
        case 2:
          BIT_SET(currentStatus.status1, BIT_STATUS1_BOOSTCUT);
          BIT_CLEAR(currentStatus.spark, BIT_SPARK_BOOSTCUT);
          break;
        case 3:
          BIT_SET(currentStatus.spark, BIT_SPARK_BOOSTCUT);
          BIT_SET(currentStatus.status1, BIT_STATUS1_BOOSTCUT);
          break;
        default:
          //Shouldn't ever happen, but just in case, disable all cuts
          BIT_CLEAR(currentStatus.status1, BIT_STATUS1_BOOSTCUT);
          BIT_CLEAR(currentStatus.spark, BIT_SPARK_BOOSTCUT);
      }
      */
    }
  }

  return boostLimitActive;
}

byte checkOilPressureLimit()
{
  byte oilProtectActive = 0;
  BIT_CLEAR(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_OIL); //Will be set true below if required

  if (configPage6.engineProtectType != PROTECT_CUT_OFF) {
    if( (configPage10.oilPressureProtEnbl == true) && (configPage10.oilPressureEnable == true) )
    {
      byte oilLimit = table2D_getValue(&oilPressureProtectTable, currentStatus.RPMdiv100);
      if(currentStatus.oilPressure < oilLimit)
      {
        BIT_SET(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_OIL);
        oilProtectActive = 1;
      }
    }
  }

  return oilProtectActive;
}

byte checkAFRLimit()
{
  static bool checkAFRLimitActive = false;
  static bool afrProtectCountEnabled = false;
  static unsigned long afrProtectCount = 0;
  static constexpr char X2_MULTIPLIER = 2;
  static constexpr char X100_MULTIPLIER = 100;

  /*
    To use this function, a wideband sensor is required.

    First of all, check whether engine protection is enabled,
    thereafter check whether AFR protection is enabled and at last
    if wideband sensor is used.
    
    After confirmation, the following conditions has to be met:
    - MAP above x kPa
    - RPM above x
    - TPS above x %
    - AFR threshold (AFR target + defined maximum deviation)
    - Time before cut

    See afrProtect variables in globals.h for more information.

    If all conditions above are true, a specified time delay is starting
    to count down in which leads to the engine protection function
    to be activated using selected protection cut method (e.g. ignition,
    fuel or both).

    For reactivation, the following condition has to be met:
    - TPS below x %
  */

  /*
    Do 3 checks here;
    - whether engine protection is enabled
    - whether AFR protection is enabled
    - whether wideband sensor is used
  */
  if(configPage6.engineProtectType != PROTECT_CUT_OFF && configPage9.afrProtectEnabled && configPage6.egoType == EGO_TYPE_WIDE) {
    /* Conditions */
    bool mapCondition = (currentStatus.MAP >= (configPage9.afrProtectMinMAP * X2_MULTIPLIER)) ? true : false;
    bool rpmCondition = (currentStatus.RPMdiv100 >= configPage9.afrProtectMinRPM) ? true : false;
    bool tpsCondition = (currentStatus.TPS >= configPage9.afrProtectMinTPS) ? true : false;

    /*
      Depending on selected mode, this could either be fixed AFR value or a
      value set to be the maximum deviation from AFR target table.

      1 = fixed value mode, 2 = target table mode
    */
    bool afrCondition;
    switch(configPage9.afrProtectEnabled)
    {
      case 1: afrCondition = (currentStatus.O2 >= configPage9.afrProtectDeviation) ? true : false; break; /* Fixed value */
      case 2: afrCondition = (currentStatus.O2 >= (currentStatus.afrTarget + configPage9.afrProtectDeviation)) ? true : false; break; /* Deviation from target table */
      default: afrCondition = false; /* Unknown mode. Shouldn't even get here */
    }

    /* Check if conditions above are fulfilled */
    if(mapCondition && rpmCondition && tpsCondition && afrCondition) 
    {
      /* All conditions fulfilled - start counter for 'protection delay' */
      if(!afrProtectCountEnabled) 
      {
        afrProtectCountEnabled = true;
        afrProtectCount = millis();
      }

      /* Check if countdown has reached its target, if so then instruct to cut */
      if(millis() >= (afrProtectCount + (configPage9.afrProtectCutTime * X100_MULTIPLIER))) 
      {
        checkAFRLimitActive = true;
        BIT_SET(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_AFR);
      }
    } 
    else 
    {
      /* Conditions have presumably changed - deactivate and reset counter */
      if(afrProtectCountEnabled) 
      {
        afrProtectCountEnabled = false;
        afrProtectCount = 0;
      }
    }

    /* Check if condition for reactivation is fulfilled */
    if(checkAFRLimitActive && (currentStatus.TPS <= configPage9.afrProtectReactivationTPS)) 
    {
      checkAFRLimitActive = false;
      afrProtectCountEnabled = false;
      BIT_CLEAR(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_AFR);
    }
  }

  return checkAFRLimitActive;
}

