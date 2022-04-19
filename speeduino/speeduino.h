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
//#include "globals.h"

void setup();
void loop();
uint16_t PW(int REQ_FUEL, byte VE, long MAP, uint16_t corrections, int injOpen);
byte getVE1();
byte getAdvance1();

uint16_t calculateInjectorStartAngle(uint16_t, int16_t);
void calculateIgnitionAngle1(int);
void calculateIgnitionAngle2(int);
void calculateIgnitionAngle3(int);
void calculateIgnitionAngle3(int, int);
void calculateIgnitionAngle4(int);
void calculateIgnitionAngle4(int, int);
void calculateIgnitionAngle5(int);
void calculateIgnitionAngle6(int);
void calculateIgnitionAngle7(int);
void calculateIgnitionAngle8(int);
void calculateIgnitionAngles(int);

extern uint16_t req_fuel_uS; /**< The required fuel variable (As calculated by TunerStudio) in uS */
extern uint16_t inj_opentime_uS; /**< The injector opening time. This is set within Tuner Studio, but stored here in uS rather than mS */

extern bool ignitionOn; /**< The current state of the ignition system (on or off) */
extern bool fuelOn; /**< The current state of the fuel system (on or off) */

extern byte maxIgnOutputs; /**< Used for rolling rev limiter to indicate how many total ignition channels should currently be firing */
extern byte curRollingCut; /**< Rolling rev limiter, current ignition channel being cut */
extern byte rollingCutCounter; /**< how many times (revolutions) the ignition has been cut in a row */
extern uint32_t rollingCutLastRev; /**< Tracks whether we're on the same or a different rev for the rolling cut */

extern int channel1IgnDegrees; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */
extern int channel2IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
extern int channel3IgnDegrees; /**< The number of crank degrees until cylinder 3 (and 5/6/7/8) is at TDC */
extern int channel4IgnDegrees; /**< The number of crank degrees until cylinder 4 (and 5/6/7/8) is at TDC */
extern int channel5IgnDegrees; /**< The number of crank degrees until cylinder 5 is at TDC */
extern int channel6IgnDegrees; /**< The number of crank degrees until cylinder 6 is at TDC */
extern int channel7IgnDegrees; /**< The number of crank degrees until cylinder 7 is at TDC */
extern int channel8IgnDegrees; /**< The number of crank degrees until cylinder 8 is at TDC */
extern int channel1InjDegrees; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */
extern int channel2InjDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
extern int channel3InjDegrees; /**< The number of crank degrees until cylinder 3 (and 5/6/7/8) is at TDC */
extern int channel4InjDegrees; /**< The number of crank degrees until cylinder 4 (and 5/6/7/8) is at TDC */
extern int channel5InjDegrees; /**< The number of crank degrees until cylinder 5 is at TDC */
extern int channel6InjDegrees; /**< The number of crank degrees until cylinder 6 is at TDC */
extern int channel7InjDegrees; /**< The number of crank degrees until cylinder 7 is at TDC */
extern int channel8InjDegrees; /**< The number of crank degrees until cylinder 8 is at TDC */

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
///@{
extern uint16_t staged_req_fuel_mult_pri;
extern uint16_t staged_req_fuel_mult_sec;
///@}



#endif
