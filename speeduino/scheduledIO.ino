#include "scheduledIO.h"
#include "scheduler.h"
#include "globals.h"
#include "timers.h"
#include "acc_mc33810.h"
/** @file
 * Injector and Coil (toggle/open/close) control (under various situations, eg with particular cylinder count, rotary engine type or wasted spark ign, etc.).
 * Also accounts for presence of MC33810 injector/ignition (dwell, etc.) control circuit.
 * Functions here are typically assigned (at initialisation) to callback function variables (e.g. inj1StartFunction or inj1EndFunction) 
 * form where they are called (by scheduler.ino).
 */
inline void openInjector1(void)   { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { openInjector1_DIRECT(); }   else { openInjector1_MC33810(); } }
inline void closeInjector1(void)  { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { closeInjector1_DIRECT(); }  else { closeInjector1_MC33810(); } }
inline void openInjector2(void)   { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { openInjector2_DIRECT(); }   else { openInjector2_MC33810(); } }
inline void closeInjector2(void)  { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { closeInjector2_DIRECT(); }  else { closeInjector2_MC33810(); } }
inline void openInjector3(void)   { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { openInjector3_DIRECT(); }   else { openInjector3_MC33810(); } }
inline void closeInjector3(void)  { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { closeInjector3_DIRECT(); }  else { closeInjector3_MC33810(); } }
inline void openInjector4(void)   { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { openInjector4_DIRECT(); }   else { openInjector4_MC33810(); } }
inline void closeInjector4(void)  { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { closeInjector4_DIRECT(); }  else { closeInjector4_MC33810(); } }
inline void openInjector5(void)   { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { openInjector5_DIRECT(); }   else { openInjector5_MC33810(); } }
inline void closeInjector5(void)  { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { closeInjector5_DIRECT(); }  else { closeInjector5_MC33810(); } }
inline void openInjector6(void)   { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { openInjector6_DIRECT(); }   else { openInjector6_MC33810(); } }
inline void closeInjector6(void)  { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { closeInjector6_DIRECT(); }  else { closeInjector6_MC33810(); } }
inline void openInjector7(void)   { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { openInjector7_DIRECT(); }   else { openInjector7_MC33810(); } }
inline void closeInjector7(void)  { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { closeInjector7_DIRECT(); }  else { closeInjector7_MC33810(); } }
inline void openInjector8(void)   { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { openInjector8_DIRECT(); }   else { openInjector8_MC33810(); } }
inline void closeInjector8(void)  { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { closeInjector8_DIRECT(); }  else { closeInjector8_MC33810(); } }

inline void injector1Toggle(void) { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { injector1Toggle_DIRECT(); } else { injector1Toggle_MC33810(); } }
inline void injector2Toggle(void) { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { injector2Toggle_DIRECT(); } else { injector2Toggle_MC33810(); } }
inline void injector3Toggle(void) { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { injector3Toggle_DIRECT(); } else { injector3Toggle_MC33810(); } }
inline void injector4Toggle(void) { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { injector4Toggle_DIRECT(); } else { injector4Toggle_MC33810(); } }
inline void injector5Toggle(void) { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { injector5Toggle_DIRECT(); } else { injector5Toggle_MC33810(); } }
inline void injector6Toggle(void) { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { injector6Toggle_DIRECT(); } else { injector6Toggle_MC33810(); } }
inline void injector7Toggle(void) { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { injector7Toggle_DIRECT(); } else { injector7Toggle_MC33810(); } }
inline void injector8Toggle(void) { if(injectorOutputControl != OUTPUT_CONTROL_MC33810) { injector8Toggle_DIRECT(); } else { injector8Toggle_MC33810(); } }

inline void coil1Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil1Toggle_DIRECT(); } else { coil1Toggle_MC33810(); } }
inline void coil2Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil2Toggle_DIRECT(); } else { coil2Toggle_MC33810(); } }
inline void coil3Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil3Toggle_DIRECT(); } else { coil3Toggle_MC33810(); } }
inline void coil4Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil4Toggle_DIRECT(); } else { coil4Toggle_MC33810(); } }
inline void coil5Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil5Toggle_DIRECT(); } else { coil5Toggle_MC33810(); } }
inline void coil6Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil6Toggle_DIRECT(); } else { coil6Toggle_MC33810(); } }
inline void coil7Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil7Toggle_DIRECT(); } else { coil7Toggle_MC33810(); } }
inline void coil8Toggle(void)     { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil8Toggle_DIRECT(); } else { coil8Toggle_MC33810(); } }

// These are for Semi-Sequential and 5 Cylinder injection
//Standard 4 cylinder pairings
void openInjector1and3(void) { openInjector1(); openInjector3(); }
void closeInjector1and3(void) { closeInjector1(); closeInjector3(); }
void openInjector2and4(void) { openInjector2(); openInjector4(); }
void closeInjector2and4(void) { closeInjector2(); closeInjector4(); }
//Alternative output pairings
void openInjector1and4(void) { openInjector1(); openInjector4(); }
void closeInjector1and4(void) { closeInjector1(); closeInjector4(); }
void openInjector2and3(void) { openInjector2(); openInjector3(); }
void closeInjector2and3(void) { closeInjector2(); closeInjector3(); }

void openInjector3and5(void) { openInjector3(); openInjector5(); }
void closeInjector3and5(void) { closeInjector3(); closeInjector5(); }

void openInjector2and5(void) { openInjector2(); openInjector5(); }
void closeInjector2and5(void) { closeInjector2(); closeInjector5(); }
void openInjector3and6(void) { openInjector3(); openInjector6(); }
void closeInjector3and6(void) { closeInjector3(); closeInjector6(); }

void openInjector1and5(void) { openInjector1(); openInjector5(); }
void closeInjector1and5(void) { closeInjector1(); closeInjector5(); }
void openInjector2and6(void) { openInjector2(); openInjector6(); }
void closeInjector2and6(void) { closeInjector2(); closeInjector6(); }
void openInjector3and7(void) { openInjector3(); openInjector7(); }
void closeInjector3and7(void) { closeInjector3(); closeInjector7(); }
void openInjector4and8(void) { openInjector4(); openInjector8(); }
void closeInjector4and8(void) { closeInjector4(); closeInjector8(); }

inline void beginCoil1Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil1Charging_DIRECT(); } else { coil1Charging_MC33810(); } tachoOutputOn(); }
inline void endCoil1Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil1StopCharging_DIRECT(); } else { coil1StopCharging_MC33810(); } tachoOutputOff(); }

inline void beginCoil2Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil2Charging_DIRECT(); } else { coil2Charging_MC33810(); } tachoOutputOn(); }
inline void endCoil2Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil2StopCharging_DIRECT(); } else { coil2StopCharging_MC33810(); } tachoOutputOff(); }

inline void beginCoil3Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil3Charging_DIRECT(); } else { coil3Charging_MC33810(); } tachoOutputOn(); }
inline void endCoil3Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil3StopCharging_DIRECT(); } else { coil3StopCharging_MC33810(); } tachoOutputOff(); }

inline void beginCoil4Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil4Charging_DIRECT(); } else { coil4Charging_MC33810(); } tachoOutputOn(); }
inline void endCoil4Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil4StopCharging_DIRECT(); } else { coil4StopCharging_MC33810(); } tachoOutputOff(); }

inline void beginCoil5Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil5Charging_DIRECT(); } else { coil5Charging_MC33810(); } tachoOutputOn(); }
inline void endCoil5Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil5StopCharging_DIRECT(); } else { coil5StopCharging_MC33810(); } tachoOutputOff(); }

inline void beginCoil6Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil6Charging_DIRECT(); } else { coil6Charging_MC33810(); } tachoOutputOn(); }
inline void endCoil6Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil6StopCharging_DIRECT(); } else { coil6StopCharging_MC33810(); } tachoOutputOff(); }

inline void beginCoil7Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil7Charging_DIRECT(); } else { coil7Charging_MC33810(); } tachoOutputOn(); }
inline void endCoil7Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil7StopCharging_DIRECT(); } else { coil7StopCharging_MC33810(); } tachoOutputOff(); }

inline void beginCoil8Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil8Charging_DIRECT(); } else { coil8Charging_MC33810(); } tachoOutputOn(); }
inline void endCoil8Charge(void) { if(ignitionOutputControl != OUTPUT_CONTROL_MC33810) { coil8StopCharging_DIRECT(); } else { coil8StopCharging_MC33810(); } tachoOutputOff(); }

//The below 3 calls are all part of the rotary ignition mode
inline void beginTrailingCoilCharge(void) { beginCoil2Charge(); }
inline void endTrailingCoilCharge1(void) { endCoil2Charge(); beginCoil3Charge(); } //Sets ign3 (Trailing select) high
inline void endTrailingCoilCharge2(void) { endCoil2Charge(); endCoil3Charge(); } //sets ign3 (Trailing select) low

//As above but for ignition (Wasted COP mode)
void beginCoil1and3Charge(void) { beginCoil1Charge(); beginCoil3Charge(); }
void endCoil1and3Charge(void)   { endCoil1Charge();  endCoil3Charge(); }
void beginCoil2and4Charge(void) { beginCoil2Charge(); beginCoil4Charge(); }
void endCoil2and4Charge(void)   { endCoil2Charge();  endCoil4Charge(); }

//For 6cyl wasted COP mode)
void beginCoil1and4Charge(void) { beginCoil1Charge(); beginCoil4Charge(); }
void endCoil1and4Charge(void)   { endCoil1Charge();  endCoil4Charge(); }
void beginCoil2and5Charge(void) { beginCoil2Charge(); beginCoil5Charge(); }
void endCoil2and5Charge(void)   { endCoil2Charge();  endCoil5Charge(); }
void beginCoil3and6Charge(void) { beginCoil3Charge(); beginCoil6Charge(); }
void endCoil3and6Charge(void)   { endCoil3Charge(); endCoil6Charge(); }

//For 8cyl wasted COP mode)
void beginCoil1and5Charge(void) { beginCoil1Charge(); beginCoil5Charge(); }
void endCoil1and5Charge(void)   { endCoil1Charge();  endCoil5Charge(); }
void beginCoil2and6Charge(void) { beginCoil2Charge(); beginCoil6Charge(); }
void endCoil2and6Charge(void)   { endCoil2Charge();  endCoil6Charge(); }
void beginCoil3and7Charge(void) { beginCoil3Charge(); beginCoil7Charge();  }
void endCoil3and7Charge(void)   { endCoil3Charge(); endCoil7Charge(); }
void beginCoil4and8Charge(void) { beginCoil4Charge(); beginCoil8Charge(); }
void endCoil4and8Charge(void)   { endCoil4Charge();  endCoil8Charge(); }

void tachoOutputOn(void) { if(configPage6.tachoMode) { TACHO_PULSE_LOW(); } else { tachoOutputFlag = READY; } }
void tachoOutputOff(void) { if(configPage6.tachoMode) { TACHO_PULSE_HIGH(); } }

void nullCallback(void) { return; }
