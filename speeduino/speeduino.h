/** \file speeduino.h
 * @brief Speeduino main file containing initial setup and system loop functions 
 * @author Josh Stewart
 * 
 * This file contains the main system loop of the Speeduino core and thus much of the logic of the fuel and ignition algorithms is contained within this
 * It is where calls to all the auxiliary control systems, sensor reads, comms etc are made
 * 
 * It also contains the setup() function that is called by the bootloader on system startup
 * 
 */

#ifndef SPEEDUINO_H
#define SPEEDUINO_H

#include "pw_calcs.h"

#define CRANK_RUN_HYSTER    15

void setup(void);
void loop(void);

void calculateStaging(uint32_t);
void calculateIgnitionAngles(uint16_t dwellAngle);
void checkLaunchAndFlatShift(void);

void applyPulseWidths(const pulseWidths &pulseWidths);

#endif
