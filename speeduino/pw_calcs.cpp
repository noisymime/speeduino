#include "pw_calcs.h"

#ifdef USE_LIBDIVIDE
#include "src/libdivide/libdivide.h"
struct libdivide::libdivide_u32_t libdiv_u32_nsquirts;
#endif

void initialisePWCalcs(void)
{
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
uint16_t PW(uint16_t REQ_FUEL, uint8_t VE, uint16_t MAP, uint16_t corrections, uint16_t injOpen)
{
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
