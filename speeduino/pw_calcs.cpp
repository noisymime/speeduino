#include "pw_calcs.h"

#if !defined(UNIT_TEST)
static 
#endif
uint16_t req_fuel_uS = 0U; /**< The required fuel variable (As calculated by TunerStudio) in uS */

/** @name Staging
 * These values are a percentage of the total (Combined) req_fuel value that would be required for each injector channel to deliver that much fuel.   
 * 
 * Eg:
 *  - Pri injectors are 250cc
 *  - Sec injectors are 500cc
 *  - Total injector capacity = 750cc
 * 
 *  - staged_req_fuel_mult_pri = 300% (The primary injectors would have to run 3x the overall PW in order to be the equivalent of the full 750cc capacity
 *  - staged_req_fuel_mult_sec = 150% (The secondary injectors would have to run 1.5x the overall PW in order to be the equivalent of the full 750cc capacity
*/
static uint16_t staged_req_fuel_mult_pri = 0;
static uint16_t staged_req_fuel_mult_sec = 0; 

void initialisePWCalcs(void)
{
  if(configPage10.stagingEnabled == true)
  {
    uint32_t totalInjector = configPage10.stagedInjSizePri + configPage10.stagedInjSizeSec;
    /*
        These values are a percentage of the req_fuel value that would be required for each injector channel to deliver that much fuel.
        Eg:
        Pri injectors are 250cc
        Sec injectors are 500cc
        Total injector capacity = 750cc

        staged_req_fuel_mult_pri = 300% (The primary injectors would have to run 3x the overall PW in order to be the equivalent of the full 750cc capacity
        staged_req_fuel_mult_sec = 150% (The secondary injectors would have to run 1.5x the overall PW in order to be the equivalent of the full 750cc capacity
    */
    staged_req_fuel_mult_pri = (100U * totalInjector) / configPage10.stagedInjSizePri;
    staged_req_fuel_mult_sec = (100U * totalInjector) / configPage10.stagedInjSizeSec;
  }
  else
  {
    staged_req_fuel_mult_pri = 0;
    staged_req_fuel_mult_sec = 0;
  }

  calculateRequiredFuel(configPage2.injLayout);
}

void calculateRequiredFuel(InjectorLayout injLayout) {
  req_fuel_uS = configPage2.reqFuel * 100U; //Convert to uS and an int. This is the only variable to be used in calculations
  if ((configPage2.strokes == FOUR_STROKE) && ((injLayout!= INJ_SEQUENTIAL) || (configPage2.nCylinders > INJ_CHANNELS)))
  {
    //Default is 1 squirt per revolution, so we halve the given req-fuel figure (Which would be over 2 revolutions)
    //The req_fuel calculation above gives the total required fuel (At VE 100%) in the full cycle.
    //If we're doing more than 1 squirt per cycle then we need to split the amount accordingly.
    //(Note that in a non-sequential 4-stroke setup you cannot have less than 2 squirts as you cannot determine the
    //stroke to make the single squirt on)
    req_fuel_uS = req_fuel_uS / 2U; 
  }
}

static inline uint16_t calcNitrousStagePulseWidth(uint8_t minRPM, uint8_t maxRPM, uint8_t adderMin, uint8_t adderMax)
{
  return (uint16_t)map(currentStatus.RPMdiv100, minRPM, maxRPM, adderMin, adderMax)*UINT16_C(100);
}

//Manual adder for nitrous. These are not in correctionsFuel() because they are direct adders to the ms value, not % based
static inline uint16_t pwApplyNitrous(uint16_t pw)
{
  if (currentStatus.nitrous_status!=NITROUS_OFF && pw!=0U)
  {
    if( (currentStatus.nitrous_status == NITROUS_STAGE1) || (currentStatus.nitrous_status == NITROUS_BOTH) )
    {
      pw = pw + calcNitrousStagePulseWidth(configPage10.n2o_stage1_minRPM, configPage10.n2o_stage1_maxRPM, configPage10.n2o_stage1_adderMin, configPage10.n2o_stage1_adderMax);
    }
    if( (currentStatus.nitrous_status == NITROUS_STAGE2) || (currentStatus.nitrous_status == NITROUS_BOTH) )
    {
      pw = pw + calcNitrousStagePulseWidth(configPage10.n2o_stage2_minRPM, configPage10.n2o_stage2_maxRPM, configPage10.n2o_stage2_adderMin, configPage10.n2o_stage2_adderMax);
    }
  }

  return pw;
}

static inline uint32_t pwApplyMapMode(uint32_t intermediate, uint16_t MAP) {
  if ( configPage2.multiplyMAP == MULTIPLY_MAP_MODE_100) { 
    uint16_t mutiplier = div100((uint16_t)(MAP << 7U));
    return rshift<7U>(intermediate * (uint32_t)mutiplier); 
  }
  if( configPage2.multiplyMAP == MULTIPLY_MAP_MODE_BARO) { 
     uint16_t mutiplier = (MAP << 7U) / currentStatus.baro; 
    return rshift<7U>(intermediate * (uint32_t)mutiplier); 
  }
  return intermediate;
}

static inline uint32_t pwApplyAFRMultiplier(uint32_t intermediate) {
  if ( (configPage2.includeAFR == true) && (configPage6.egoType == EGO_TYPE_WIDE) && (currentStatus.runSecs > configPage6.ego_sdelay) ) {
    uint16_t mutiplier = ((uint16_t)currentStatus.O2 << 7U) / currentStatus.afrTarget;  //Include AFR (vs target) if enabled
    return rshift<7U>(intermediate * (uint32_t)mutiplier); 
  }
  if ( (configPage2.incorporateAFR == true) && (configPage2.includeAFR == false) ) {
    uint16_t mutiplier = ((uint16_t)configPage2.stoich << 7U) / currentStatus.afrTarget;  //Incorporate stoich vs target AFR, if enabled.
    return rshift<7U>(intermediate * (uint32_t)mutiplier); 
  }
  return intermediate;
}

static inline uint32_t pwApplyCorrections(uint32_t intermediate, uint16_t corrections) {
  return percentageApprox(corrections, intermediate); 
}

static inline uint32_t pwComputeInitial(uint16_t REQ_FUEL, uint8_t VE) {
  return percentageApprox(VE, REQ_FUEL); 
}


static inline uint16_t computePrimaryPulseWidth(uint16_t REQ_FUEL, uint8_t VE, uint16_t MAP, uint16_t corrections, uint16_t injOpenTime) {
  //Standard float version of the calculation
  //return (REQ_FUEL * (float)(VE/100.0) * (float)(MAP/100.0) * (float)(TPS/100.0) * (float)(corrections/100.0) + injOpenTime);
  //Note: The MAP and TPS portions are currently disabled, we use VE and corrections only

  uint32_t intermediate = pwApplyCorrections(pwApplyAFRMultiplier(pwApplyMapMode(pwComputeInitial(REQ_FUEL, VE), MAP)), corrections);
  if (intermediate != 0U)
  {
    //If intermediate is not 0, we need to add the opening time (0 typically indicates that one of the full fuel cuts is active)
    intermediate += injOpenTime; //Add the injector opening time
    
    //AE calculation only when ACC is active.
    if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC) )
    {
      //AE Adds % of req_fuel
      if ( configPage2.aeApplyMode == AE_MODE_ADDER )
      {
        intermediate += percentage(currentStatus.AEamount - 100U, REQ_FUEL);
      }
    }
  }
  // Make sure this won't overflow when we convert to uInt. This means the maximum pulsewidth possible is 65.535mS
  return pwApplyNitrous((uint16_t)(intermediate>UINT16_MAX ? UINT16_MAX : intermediate));
}  

#if !defined(UNIT_TEST)
static inline 
#endif
uint16_t calculatePWLimit(void)
{
  uint32_t tempLimit = percentageApprox(configPage2.dutyLim, revolutionTime); //The pulsewidth limit is determined to be the duty cycle limit (Eg 85%) by the total time it takes to perform 1 revolution
  //Handle multiple squirts per rev
  if (configPage2.strokes == FOUR_STROKE) { tempLimit = tempLimit * 2U; }

  //Optimise for power of two divisions where possible
  switch(currentStatus.nSquirts)
  {
    case 1:
      //No action needed
      break;
    case 2:
      tempLimit = tempLimit / 2U;
      break;
    case 4:
      tempLimit = tempLimit / 4U;
      break;
    case 8:
      tempLimit = tempLimit / 8U;
      break;
    default:
      //Non-PoT squirts value. Perform (slow) uint32_t division
      tempLimit = tempLimit / currentStatus.nSquirts;
      break;
  }
  // Make sure this won't overflow when we convert to uInt. This means the maximum pulsewidth possible is 65.535mS
  return (uint16_t)min(tempLimit, (uint32_t)UINT16_MAX);
}

static inline pulseWidths applyStagingModeTable(uint16_t primaryPW, uint16_t injOpenTime) {
  //Subtract the opening time from PW1 as it needs to be multiplied out again by the pri/sec req_fuel values below. It is added on again after that calculation. 
  uint32_t pwPrimaryStaged = percentage(staged_req_fuel_mult_pri, primaryPW - injOpenTime);

  uint8_t stagingSplit = get3DTableValue(&stagingTable, currentStatus.fuelLoad, currentStatus.RPM);
  if(stagingSplit > 0U) 
  { 
    uint32_t pwSecondaryStaged = percentage(staged_req_fuel_mult_sec, primaryPW - injOpenTime); //This is ONLY needed in in table mode. Auto mode only calculates the difference.
    uint32_t primary = percentage(100U - stagingSplit, pwPrimaryStaged) + injOpenTime;
    uint32_t secondary = percentage(stagingSplit, pwSecondaryStaged) + injOpenTime;
    return { 
      (uint16_t)min(primary, (uint32_t)UINT16_MAX),
      (uint16_t)min(secondary, (uint32_t)UINT16_MAX),
    };
  }

  return { (uint16_t)min(pwPrimaryStaged + injOpenTime, (uint32_t)UINT16_MAX), 0U};
}

static inline pulseWidths applyStagingModeAuto(uint16_t primaryPW, uint16_t pwLimit, uint16_t injOpenTime) {
  //Subtract the opening time from PW1 as it needs to be multiplied out again by the pri/sec req_fuel values below. It is added on again after that calculation. 
  uint32_t pwPrimaryStaged = percentage(staged_req_fuel_mult_pri, primaryPW - injOpenTime);

  //If automatic mode, the primary injectors are used all the way up to their limit (Configured by the pulsewidth limit setting)
  //If they exceed their limit, the extra duty is passed to the secondaries
  if(pwPrimaryStaged > pwLimit)
  {
    uint32_t extraPW = pwPrimaryStaged - pwLimit + injOpenTime; //The open time must be added here AND below because pwPrimaryStaged does not include an open time. The addition of it here takes into account the fact that pwLlimit does not contain an allowance for an open time. 
    uint32_t secondary = udiv_32_16(extraPW * staged_req_fuel_mult_sec, staged_req_fuel_mult_pri) + injOpenTime;
    return { 
      pwLimit,
      (uint16_t)min(secondary, (uint32_t)UINT16_MAX),
    };
  }

  return { (uint16_t)min(pwPrimaryStaged + injOpenTime, (uint32_t)UINT16_MAX), 0U};
}

static inline pulseWidths applyStagingToPw(uint16_t primaryPW, uint16_t pwLimit, uint16_t injOpenTime) {
  pulseWidths widths = { primaryPW, 0U };

  if (primaryPW!=0U) {
    //To run staged injection, the number of cylinders must be less than or equal to the injector channels (ie Assuming you're running paired injection, you need at least as many injector channels as you have cylinders, half for the primaries and half for the secondaries)
    if ( (configPage10.stagingEnabled == true) && (configPage2.nCylinders <= INJ_CHANNELS || configPage2.injType == INJ_TYPE_TBODY)) //Final check is to ensure that DFCO isn't active, which would cause an overflow below (See #267)
    {
      //Scale the 'full' pulsewidth by each of the injector capacities
      if(configPage10.stagingMode == STAGING_MODE_TABLE) {
        widths = applyStagingModeTable(primaryPW, injOpenTime);
      } else if(configPage10.stagingMode == STAGING_MODE_AUTO) {
        widths = applyStagingModeAuto(primaryPW, pwLimit, injOpenTime);
      } else {
        // Unknown staging mode - accept default & keep MISRA checker happy.
      }
    }
    //Apply the pwLimit if staging is disabled and engine is not cranking
    else if( (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK))) { 
      widths = { min(primaryPW, pwLimit), 0U };
    } else {
      // No staging needed - accept default & keep MISRA checker happy.
    }
  }

  BIT_WRITE(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE, widths.secondary!=0U);

  return widths;
}

#if !defined(UNIT_TEST)
static inline 
#endif
pulseWidths computePulseWidths(uint16_t REQ_FUEL, uint8_t VE, uint16_t MAP, uint16_t corrections) {
  // Convert injector open time from tune to microseconds & apply voltage correction if required
  uint16_t injOpenTime = configPage2.injOpen * (configPage2.battVCorMode == BATTV_COR_MODE_OPENTIME ? currentStatus.batCorrection : 100U); 
  return applyStagingToPw(computePrimaryPulseWidth(REQ_FUEL, VE, MAP, corrections, injOpenTime), calculatePWLimit(), injOpenTime);
}

// Suppress "always_inline function might not be inlinable" warning
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"

pulseWidths __attribute__((flatten, always_inline)) computePulseWidths(uint8_t VE, uint16_t MAP, uint16_t corrections) {
  return computePulseWidths(req_fuel_uS, VE, MAP, corrections);
}

#pragma GCC diagnostic pop
