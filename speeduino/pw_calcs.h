#pragma once

#include <stdint.h>
#include "globals.h"
#include "maths.h"

#ifdef USE_LIBDIVIDE
#include "src/libdivide/libdivide.h"
extern struct libdivide::libdivide_u32_t libdiv_u32_nsquirts;
#endif

// Runs any initialization required by this module
void initialisePWCalcs(void);

static inline uint16_t calculatePWLimit()
{
  uint32_t tempLimit = percentage(configPage2.dutyLim, revolutionTime); //The pulsewidth limit is determined to be the duty cycle limit (Eg 85%) by the total time it takes to perform 1 revolution
  //Handle multiple squirts per rev
  if (configPage2.strokes == FOUR_STROKE) { tempLimit = tempLimit * 2U; }

  if (currentStatus.nSquirts>1U) {
#ifdef USE_LIBDIVIDE
    return libdivide::libdivide_u32_do(tempLimit, &libdiv_u32_nsquirts);
#else
    return tempLimit / currentStatus.nSquirts; 
#endif
  }

  return min(tempLimit, (uint32_t)UINT16_MAX);
}

static inline uint16_t applyFuelTrimToPW(trimTable3d *pTrimTable, int16_t fuelLoad, int16_t RPM, uint16_t currentPW)
{
  uint8_t pw1percent = 100U + get3DTableValue(pTrimTable, fuelLoad, RPM) - OFFSET_FUELTRIM;
  return percentage(pw1percent, currentPW);
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
uint16_t PW(uint16_t REQ_FUEL, uint8_t VE, uint16_t MAP, uint16_t corrections, uint16_t injOpen);