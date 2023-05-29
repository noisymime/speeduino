#pragma once

#include <stdint.h>
#include "maths.h"
#include "globals.h"

extern uint16_t req_fuel_uS; /**< The required fuel variable (As calculated by TunerStudio) in uS */
extern uint16_t inj_opentime_uS; /**< The injector opening time. This is set within Tuner Studio, but stored here in uS rather than mS */

// Runs any initialization required by this module
void initialisePWCalcs(void);

static inline uint16_t applyFuelTrimToPW(trimTable3d *pTrimTable, int16_t fuelLoad, int16_t RPM, uint16_t currentPW)
{
  uint8_t pw1percent = 100U + get3DTableValue(pTrimTable, fuelLoad, RPM) - OFFSET_FUELTRIM;
  return percentage(pw1percent, currentPW);
}

struct pulseWidths {
  uint16_t primary;
  uint16_t secondary;
};

pulseWidths computePulseWidths(uint16_t REQ_FUEL, uint8_t VE, uint16_t MAP, uint16_t corrections, uint16_t injOpen);