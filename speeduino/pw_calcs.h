#pragma once

#include <stdint.h>
#include "maths.h"
#include "globals.h"

// Runs any initialization required by this module
void initialisePWCalcs(void);

//
void calculateRequiredFuel(uint8_t injLayout);

static inline uint16_t applyFuelTrimToPW(trimTable3d *pTrimTable, int16_t fuelLoad, int16_t RPM, uint16_t currentPW)
{
  uint8_t pw1percent = 100U + get3DTableValue(pTrimTable, fuelLoad, RPM) - OFFSET_FUELTRIM;
  return percentage(pw1percent, currentPW);
}

struct pulseWidths {
  uint16_t primary;
  uint16_t secondary;
};

pulseWidths computePulseWidths(uint8_t VE, uint16_t MAP, uint16_t corrections);