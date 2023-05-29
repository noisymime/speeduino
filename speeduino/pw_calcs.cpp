#include "pw_calcs.h"

uint16_t req_fuel_uS = 0; /**< The required fuel variable (As calculated by TunerStudio) in uS */
uint16_t inj_opentime_uS = 0;

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

#ifdef USE_LIBDIVIDE
#include "src/libdivide/libdivide.h"
static struct libdivide::libdivide_u32_t libdiv_u32_nsquirts;
#endif

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
    staged_req_fuel_mult_pri = (100 * totalInjector) / configPage10.stagedInjSizePri;
    staged_req_fuel_mult_sec = (100 * totalInjector) / configPage10.stagedInjSizeSec;
  }
  else
  {
    staged_req_fuel_mult_pri = 0;
    staged_req_fuel_mult_sec = 0;
  }

  req_fuel_uS = configPage2.reqFuel * 100; //Convert to uS and an int. This is the only variable to be used in calculations
  if ((configPage2.strokes == FOUR_STROKE) && ((configPage2.injLayout != INJ_SEQUENTIAL) || (configPage2.nCylinders > INJ_CHANNELS)))
  {
    //Default is 1 squirt per revolution, so we halve the given req-fuel figure (Which would be over 2 revolutions)
    req_fuel_uS = req_fuel_uS / 2; //The req_fuel calculation above gives the total required fuel (At VE 100%) in the full cycle. If we're doing more than 1 squirt per cycle then we need to split the amount accordingly. (Note that in a non-sequential 4-stroke setup you cannot have less than 2 squirts as you cannot determine the stroke to make the single squirt on)
  }

  inj_opentime_uS = configPage2.injOpen * 100; //Injector open time. Comes through as ms*10 (Eg 15.5ms = 155).

#ifdef USE_LIBDIVIDE
  libdiv_u32_nsquirts = libdivide::libdivide_u32_gen(currentStatus.nSquirts);
#endif    
}

static inline uint16_t pwApplyNitrousStage(uint16_t pw, uint8_t minRPM, uint8_t maxRPM, uint8_t adderMin, uint8_t adderMax)
{
  int16_t adderRange = ((int16_t)maxRPM - (int16_t)minRPM) * INT16_C(100);
  int16_t adderPercent = ((currentStatus.RPM - ((int16_t)minRPM * INT16_C(100))) * INT16_C(100)) / adderRange; //The percentage of the way through the RPM range
  adderPercent = INT16_C(100) - adderPercent; //Flip the percentage as we go from a higher adder to a lower adder as the RPMs rise
  return pw + (uint16_t)(adderMax + (uint16_t)percentage(adderPercent, (adderMin - adderMax))) * UINT16_C(100); //Calculate the above percentage of the calculated ms value.
}

//Manual adder for nitrous. These are not in correctionsFuel() because they are direct adders to the ms value, not % based
static inline uint16_t pwApplyNitrous(uint16_t pw)
{
  if (currentStatus.nitrous_status!=NITROUS_OFF )
  {
    if( (currentStatus.nitrous_status == NITROUS_STAGE1) || (currentStatus.nitrous_status == NITROUS_BOTH) )
    {
      pw = pwApplyNitrousStage(pw, configPage10.n2o_stage1_minRPM, configPage10.n2o_stage1_maxRPM, configPage10.n2o_stage1_adderMin, configPage10.n2o_stage1_adderMax);
    }
    if( (currentStatus.nitrous_status == NITROUS_STAGE2) || (currentStatus.nitrous_status == NITROUS_BOTH) )
    {
      pw = pwApplyNitrousStage(pw, configPage10.n2o_stage2_minRPM, configPage10.n2o_stage2_maxRPM, configPage10.n2o_stage2_adderMin, configPage10.n2o_stage2_adderMax);
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
  if (corrections < 512 ) { 
    intermediate = rshift<7U>(intermediate * div100(lshift<7U>(corrections))); 
  } else if (corrections < 1024 ) { 
    intermediate = rshift<6U>(intermediate * div100(lshift<6U>(corrections)));
  } else {
    intermediate = rshift<5U>(intermediate * div100(lshift<5U>(corrections)));
  }  
  return intermediate;
}

static inline uint32_t pwComputeInitial(uint16_t REQ_FUEL, uint8_t VE) {
  uint16_t iVE = div100((uint16_t)((uint16_t)VE << UINT16_C(7)));
  return ((uint32_t)REQ_FUEL * (uint32_t)iVE) >> 7UL; //Need to use an intermediate value to avoid overflowing the long
}

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
static inline uint16_t computePrimaryPulseWidth(uint16_t REQ_FUEL, uint8_t VE, uint16_t MAP, uint16_t corrections, uint16_t injOpen) {
  //Standard float version of the calculation
  //return (REQ_FUEL * (float)(VE/100.0) * (float)(MAP/100.0) * (float)(TPS/100.0) * (float)(corrections/100.0) + injOpen);
  //Note: The MAP and TPS portions are currently disabled, we use VE and corrections only

  uint32_t intermediate = pwApplyCorrections(pwApplyAFRMultiplier(pwApplyMapMode(pwComputeInitial(REQ_FUEL, VE), MAP)), corrections);
  if (intermediate != 0U)
  {
    //If intermediate is not 0, we need to add the opening time (0 typically indicates that one of the full fuel cuts is active)
    intermediate += injOpen; //Add the injector opening time
    
    //AE calculation only when ACC is active.
    if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC) )
    {
      //AE Adds % of req_fuel
      if ( configPage2.aeApplyMode == AE_MODE_ADDER )
      {
        intermediate += div100( (uint32_t)REQ_FUEL * (currentStatus.AEamount - 100U) );
      }
    }
  }
  // Make sure this won't overflow when we convert to uInt. This means the maximum pulsewidth possible is 65.535mS
  return pwApplyNitrous((uint16_t)(intermediate>UINT16_MAX ? UINT16_MAX : intermediate));
}  

#if !defined(UNIT_TEST)
static inline 
#endif
uint16_t calculatePWLimit(void) {
  //The pulsewidth limit is determined to be the duty cycle limit (Eg 85%) by the total time it takes to perform 1 revolution
  uint32_t limit = percentage(configPage2.dutyLim, revolutionTime); 
  if (configPage2.strokes == FOUR_STROKE) { limit = limit * 2U; }
      
  //Handle multiple squirts per rev
  if (currentStatus.nSquirts!=1U) {
#ifdef USE_LIBDIVIDE
    return libdivide::libdivide_u32_do(limit, &libdiv_u32_nsquirts);
#else
    return limit / currentStatus.nSquirts; 
#endif
  }
 
  // Make sure this won't overflow when we convert to uInt. This means the maximum pulsewidth possible is 65.535mS
  return (uint16_t)(limit>UINT16_MAX ? UINT16_MAX : limit);
}

static inline pulseWidths applyStagingToPw(uint16_t primaryPW, uint16_t pwLimit) {
  uint16_t secondaryPW = 0;

  //To run staged injection, the number of cylinders must be less than or equal to the injector channels (ie Assuming you're running paired injection, you need at least as many injector channels as you have cylinders, half for the primaries and half for the secondaries)
  if ( (configPage10.stagingEnabled == true) && (configPage2.nCylinders <= INJ_CHANNELS || configPage2.injType == INJ_TYPE_TBODY) && (primaryPW > inj_opentime_uS) ) //Final check is to ensure that DFCO isn't active, which would cause an overflow below (See #267)
  {
    //Scale the 'full' pulsewidth by each of the injector capacities
    primaryPW -= inj_opentime_uS; //Subtract the opening time from PW1 as it needs to be multiplied out again by the pri/sec req_fuel values below. It is added on again after that calculation. 
    uint32_t tempPW1 = div100((uint32_t)primaryPW * staged_req_fuel_mult_pri);

    if(configPage10.stagingMode == STAGING_MODE_TABLE)
    {
      uint32_t tempPW3 = div100((uint32_t)primaryPW * staged_req_fuel_mult_sec); //This is ONLY needed in in table mode. Auto mode only calculates the difference.

      uint8_t stagingSplit = get3DTableValue(&stagingTable, currentStatus.fuelLoad, currentStatus.RPM);
      primaryPW = div100((100U - stagingSplit) * tempPW1);
      primaryPW += inj_opentime_uS; 

      if(stagingSplit > 0) 
      { 
        secondaryPW = div100(stagingSplit * tempPW3); 
        secondaryPW += inj_opentime_uS;
      }
    }
    else if(configPage10.stagingMode == STAGING_MODE_AUTO)
    {
      primaryPW = tempPW1;
      //If automatic mode, the primary injectors are used all the way up to their limit (Configured by the pulsewidth limit setting)
      //If they exceed their limit, the extra duty is passed to the secondaries
      if(tempPW1 > pwLimit)
      {
        uint32_t extraPW = tempPW1 - pwLimit + inj_opentime_uS; //The open time must be added here AND below because tempPW1 does not include an open time. The addition of it here takes into account the fact that pwLlimit does not contain an allowance for an open time. 
        primaryPW = pwLimit;
        secondaryPW = udiv_32_16(extraPW * staged_req_fuel_mult_sec, staged_req_fuel_mult_pri); //Convert the 'left over' fuel amount from primary injector scaling to secondary
        secondaryPW += inj_opentime_uS;
      }
      else 
      {
        //If tempPW1 < pwLImit it means that the entire fuel load can be handled by the primaries and staging is inactive. 
        primaryPW += inj_opentime_uS; //Add the open time back in
      } 
    }
  }
  //Apply the pwLimit if staging is disabled and engine is not cranking
  else if( (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) && primaryPW>pwLimit) { 
    primaryPW = pwLimit; 
  }    

  BIT_WRITE(currentStatus.status4, BIT_STATUS4_STAGING_ACTIVE, secondaryPW!=0);

  return { primaryPW, secondaryPW };
}

pulseWidths computePulseWidths(uint16_t REQ_FUEL, uint8_t VE, uint16_t MAP, uint16_t corrections, uint16_t injOpen) {
  return applyStagingToPw(computePrimaryPulseWidth(REQ_FUEL, VE, MAP, corrections, injOpen), calculatePWLimit());
}
