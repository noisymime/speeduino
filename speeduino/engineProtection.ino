
#include "globals.h"
#include "engineProtection.h"

byte checkEngineProtect()
{
  byte protectActive = 0;
  if(checkRevLimit() || checkBoostLimit() || checkOilPressureLimit() || checkAFRLimit() || checkCltPressureLimit() )
  {
    if(currentStatus.RPM > (configPage4.engineProtectMaxRPM*100U)) { protectActive = 1; }
  }

  return protectActive;
}

byte checkRevLimit()
{
  //Hardcut RPM limit
  byte revLimiterActive = 0;
  BIT_CLEAR(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_RPM);

  if (currentStatus.RPM > ((unsigned int)(configPage4.HardRevLim) * 100) )
  { 
    BIT_SET(currentStatus.spark, BIT_SPARK_HRDLIM); //Legacy and likely to be removed at some point
    BIT_SET(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_RPM);
    revLimiterActive = 1; 
  } 
  else { BIT_CLEAR(currentStatus.spark, BIT_SPARK_HRDLIM); }

  return revLimiterActive;
}

byte checkBoostLimit()
{
  byte boostLimitActive = 0;
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
  else
  {
    BIT_CLEAR(currentStatus.spark, BIT_SPARK_BOOSTCUT);
    BIT_CLEAR(currentStatus.status1, BIT_STATUS1_BOOSTCUT);
  }

  return boostLimitActive;
}

byte checkOilPressureLimit()
{
  byte oilProtectActive = 0;
  BIT_CLEAR(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_OIL); //Will be set true below if required

  if( (configPage10.oilPressureProtEnbl == true) && (configPage10.oilPressureEnable == true) )
  {
    byte oilLimit = table2D_getValue(&oilPressureProtectTable, currentStatus.RPMdiv100);
    if(currentStatus.oilPressure < oilLimit)
    {
      BIT_SET(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_OIL);
      oilProtectActive = 1;
    }
  }

  return oilProtectActive;
}

byte checkCltPressureLimit()
{
  byte cltProtectActive = 0;
  BIT_CLEAR(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_CLT_PRESSURE); //Will be set true below if required

  if( (configPage10.cltPressureProtEnbl == true) && (configPage10.cltPressureEnable == true) )
  {
    byte cltLimit = table2D_getValue(&cltPressureProtectTable, currentStatus.RPMdiv100);
    if(currentStatus.cltPressure > cltLimit)
    {
      BIT_SET(currentStatus.engineProtectStatus, ENGINE_PROTECT_BIT_CLT_PRESSURE);
      cltProtectActive = 1;
    }
  }

  return cltProtectActive;
}

byte checkAFRLimit()
{
  byte checkAFRLimitActive = 0;

  return checkAFRLimitActive;
}

