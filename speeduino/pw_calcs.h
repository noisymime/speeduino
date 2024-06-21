#pragma once

#include <stdint.h>
#include "maths.h"
#include "globals.h"

/**
 * @file
 * @brief Pulse width calculations
 */

/** @brief Runs any initialization required by this module */
void initialisePWCalcs(void);

/**
 * @brief Recalculates the required fuel based on the parameter
 * 
 * Should be called whenever the injector layout changes.
 * 
 * @param injLayout The injector layout, as per the enum
 */
void calculateRequiredFuel(InjectorLayout injLayout);

/** @brief Result of pulse width calculation */
struct pulseWidths {
  /** @brief Primary pulse width in µS */
  uint16_t primary;

  /** @brief Secondary pulse width in µS. 
   * 
   * Will be zero if no secondary pulse width is required. 
   * E.g. staged injection is not turned on or the required
   * fuel can be applied using the primary injectors.
   */
  uint16_t secondary;
};

/**
 * @brief This function calculates the required pulsewidth time (in us) given the current system state
 * 
 * @param VE Lookup from the main fuel table. This can either have been MAP or TPS based, depending on the algorithm used
 * @param MAP In KPa, read from the sensor (This is used when performing a multiply of the map only. It is applicable in both Speed density and Alpha-N)
 * @param corrections Sum of Enrichment factors (Cold start, acceleration). This is a multiplication factor (Eg to add 10%, this should be 110)
 * @return pulseWidths The primary and secondary injector pulse width in uS
 */
pulseWidths computePulseWidths(uint8_t VE, uint16_t MAP, uint16_t corrections);