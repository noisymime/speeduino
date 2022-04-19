#ifndef SCHEDULEDIO_H
#define SCHEDULEDIO_H

#include <Arduino.h>

inline void openInjector1();
inline void closeInjector1();

inline void openInjector2();
inline void closeInjector2();

inline void openInjector3();
inline void closeInjector3();

inline void openInjector4();
inline void closeInjector4();

inline void openInjector5();
inline void closeInjector5();

inline void openInjector6();
inline void closeInjector6();

inline void openInjector7();
inline void closeInjector7();

inline void openInjector8();
inline void closeInjector8();

// These are for Semi-Sequential and 5 Cylinder injection
void openInjector1and3();
void closeInjector1and3();
void openInjector2and4();
void closeInjector2and4();
void openInjector1and4();
void closeInjector1and4();
void openInjector2and3();
void closeInjector2and3();

void openInjector3and5();
void closeInjector3and5();

void openInjector2and5();
void closeInjector2and5();
void openInjector3and6();
void closeInjector3and6();

void openInjector1and5();
void closeInjector1and5();
void openInjector2and6();
void closeInjector2and6();
void openInjector3and7();
void closeInjector3and7();
void openInjector4and8();
void closeInjector4and8();

void injector1Toggle();
void injector2Toggle();
void injector3Toggle();
void injector4Toggle();
void injector5Toggle();
void injector6Toggle();
void injector7Toggle();
void injector8Toggle();

inline void beginCoil1Charge();
inline void endCoil1Charge();

inline void beginCoil2Charge();
inline void endCoil2Charge();

inline void beginCoil3Charge();
inline void endCoil3Charge();

inline void beginCoil4Charge();
inline void endCoil4Charge();

inline void beginCoil5Charge();
inline void endCoil5Charge();

inline void beginCoil6Charge();
inline void endCoil6Charge();

inline void beginCoil7Charge();
inline void endCoil7Charge();

inline void beginCoil8Charge();
inline void endCoil8Charge();

//The following functions are used specifically for the trailing coil on rotary engines. They are separate as they also control the switching of the trailing select pin
inline void beginTrailingCoilCharge();
inline void endTrailingCoilCharge1();
inline void endTrailingCoilCharge2();

//And the combined versions of the above for simplicity
void beginCoil1and3Charge();
void endCoil1and3Charge();
void beginCoil2and4Charge();
void endCoil2and4Charge();

//For 6-cyl cop
void beginCoil1and4Charge();
void endCoil1and4Charge();
void beginCoil2and5Charge();
void endCoil2and5Charge();
void beginCoil3and6Charge();
void endCoil3and6Charge();

//For 8-cyl cop
void beginCoil1and5Charge();
void endCoil1and5Charge();
void beginCoil2and6Charge();
void endCoil2and6Charge();
void beginCoil3and7Charge();
void endCoil3and7Charge();
void beginCoil4and8Charge();
void endCoil4and8Charge();

void coil1Toggle();
void coil2Toggle();
void coil3Toggle();
void coil4Toggle();
void coil5Toggle();
void coil6Toggle();
void coil7Toggle();
void coil8Toggle();

/*
#ifndef USE_MC33810
#define openInjector1() *inj1_pin_port |= (inj1_pin_mask); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ1)
#define closeInjector1() *inj1_pin_port &= ~(inj1_pin_mask);  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ1)
#define openInjector2() *inj2_pin_port |= (inj2_pin_mask); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ2)
#define closeInjector2() *inj2_pin_port &= ~(inj2_pin_mask);  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ2)
#define openInjector3() *inj3_pin_port |= (inj3_pin_mask); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ3)
#define closeInjector3() *inj3_pin_port &= ~(inj3_pin_mask);  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ3)
#define openInjector4() *inj4_pin_port |= (inj4_pin_mask); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ4)
#define closeInjector4() *inj4_pin_port &= ~(inj4_pin_mask);  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ4)
#define openInjector5() *inj5_pin_port |= (inj5_pin_mask);
#define closeInjector5() *inj5_pin_port &= ~(inj5_pin_mask);
#define openInjector6() *inj6_pin_port |= (inj6_pin_mask);
#define closeInjector6() *inj6_pin_port &= ~(inj6_pin_mask);
#define openInjector7() *inj7_pin_port |= (inj7_pin_mask);
#define closeInjector7() *inj7_pin_port &= ~(inj7_pin_mask);
#define openInjector8() *inj8_pin_port |= (inj8_pin_mask);
#define closeInjector8() *inj8_pin_port &= ~(inj8_pin_mask);

#else
#include "acc_mc33810.h"
#define openInjector1() openInjector1_MC33810()
#define closeInjector1() closeInjector1_MC33810()
#define openInjector2() openInjector2_MC33810()
#define closeInjector2() closeInjector2_MC33810()
#define openInjector3() openInjector3_MC33810()
#define closeInjector3() closeInjector3_MC33810()
#define openInjector4() openInjector4_MC33810()
#define closeInjector4() closeInjector4_MC33810()
#define openInjector5() openInjector5_MC33810()
#define closeInjector5() closeInjector5_MC33810()
#define openInjector6() openInjector6_MC33810()
#define closeInjector6() closeInjector6_MC33810()
#define openInjector7() openInjector7_MC33810()
#define closeInjector7() closeInjector7_MC33810()
#define openInjector8() openInjector8_MC33810()
#define closeInjector8() closeInjector8_MC33810()

#endif

#define openInjector1and4() openInjector1(); openInjector4()
#define closeInjector1and4() closeInjector1(); closeInjector4()
#define openInjector2and3() openInjector2(); openInjector3()
#define closeInjector2and3() closeInjector2(); closeInjector3()

//5 cylinder support doubles up injector 3 as being closese to inj 5 (Crank angle)
#define openInjector3and5() openInjector3(); openInjector5()
#define closeInjector3and5() closeInjector3(); closeInjector5()
*/

//Macros are used to define how each injector control system functions. These are then called by the master openInjectx() function.
//The DIRECT macros (ie individual pins) are defined below. Others should be defined in their relevant acc_x.h file
#define openInjector1_DIRECT()  { *inj1_pin_port |= (inj1_pin_mask); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ1); }
#define closeInjector1_DIRECT() { *inj1_pin_port &= ~(inj1_pin_mask);  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ1); }
#define openInjector2_DIRECT()  { *inj2_pin_port |= (inj2_pin_mask); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ2); }
#define closeInjector2_DIRECT() { *inj2_pin_port &= ~(inj2_pin_mask);  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ2); }
#define openInjector3_DIRECT()  { *inj3_pin_port |= (inj3_pin_mask); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ3); }
#define closeInjector3_DIRECT() { *inj3_pin_port &= ~(inj3_pin_mask);  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ3); }
#define openInjector4_DIRECT()  { *inj4_pin_port |= (inj4_pin_mask); BIT_SET(currentStatus.status1, BIT_STATUS1_INJ4); }
#define closeInjector4_DIRECT() { *inj4_pin_port &= ~(inj4_pin_mask);  BIT_CLEAR(currentStatus.status1, BIT_STATUS1_INJ4); }
#define openInjector5_DIRECT()  { *inj5_pin_port |= (inj5_pin_mask); }
#define closeInjector5_DIRECT() { *inj5_pin_port &= ~(inj5_pin_mask); }
#define openInjector6_DIRECT()  { *inj6_pin_port |= (inj6_pin_mask); }
#define closeInjector6_DIRECT() { *inj6_pin_port &= ~(inj6_pin_mask); }
#define openInjector7_DIRECT()  { *inj7_pin_port |= (inj7_pin_mask); }
#define closeInjector7_DIRECT() { *inj7_pin_port &= ~(inj7_pin_mask); }
#define openInjector8_DIRECT()  { *inj8_pin_port |= (inj8_pin_mask); }
#define closeInjector8_DIRECT() { *inj8_pin_port &= ~(inj8_pin_mask); }

#define coil1Low_DIRECT()       (*ign1_pin_port &= ~(ign1_pin_mask))
#define coil1High_DIRECT()      (*ign1_pin_port |= (ign1_pin_mask))
#define coil2Low_DIRECT()       (*ign2_pin_port &= ~(ign2_pin_mask))
#define coil2High_DIRECT()      (*ign2_pin_port |= (ign2_pin_mask))
#define coil3Low_DIRECT()       (*ign3_pin_port &= ~(ign3_pin_mask))
#define coil3High_DIRECT()      (*ign3_pin_port |= (ign3_pin_mask))
#define coil4Low_DIRECT()       (*ign4_pin_port &= ~(ign4_pin_mask))
#define coil4High_DIRECT()      (*ign4_pin_port |= (ign4_pin_mask))
#define coil5Low_DIRECT()       (*ign5_pin_port &= ~(ign5_pin_mask))
#define coil5High_DIRECT()      (*ign5_pin_port |= (ign5_pin_mask))
#define coil6Low_DIRECT()       (*ign6_pin_port &= ~(ign6_pin_mask))
#define coil6High_DIRECT()      (*ign6_pin_port |= (ign6_pin_mask))
#define coil7Low_DIRECT()       (*ign7_pin_port &= ~(ign7_pin_mask))
#define coil7High_DIRECT()      (*ign7_pin_port |= (ign7_pin_mask))
#define coil8Low_DIRECT()       (*ign8_pin_port &= ~(ign8_pin_mask))
#define coil8High_DIRECT()      (*ign8_pin_port |= (ign8_pin_mask))

//Set the value of the coil pins to the coilHIGH or coilLOW state
#define coil1Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil1Low_DIRECT() : coil1High_DIRECT())
#define coil1StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil1High_DIRECT() : coil1Low_DIRECT())
#define coil2Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil2Low_DIRECT() : coil2High_DIRECT())
#define coil2StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil2High_DIRECT() : coil2Low_DIRECT())
#define coil3Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil3Low_DIRECT() : coil3High_DIRECT())
#define coil3StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil3High_DIRECT() : coil3Low_DIRECT())
#define coil4Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil4Low_DIRECT() : coil4High_DIRECT())
#define coil4StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil4High_DIRECT() : coil4Low_DIRECT())
#define coil5Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil5Low_DIRECT() : coil5High_DIRECT())
#define coil5StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil5High_DIRECT() : coil5Low_DIRECT())
#define coil6Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil6Low_DIRECT() : coil6High_DIRECT())
#define coil6StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil6High_DIRECT() : coil6Low_DIRECT())
#define coil7Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil7Low_DIRECT() : coil7High_DIRECT())
#define coil7StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil7High_DIRECT() : coil7Low_DIRECT())
#define coil8Charging_DIRECT()      (configPage4.IgInv == GOING_HIGH ? coil8Low_DIRECT() : coil8High_DIRECT())
#define coil8StopCharging_DIRECT()  (configPage4.IgInv == GOING_HIGH ? coil8High_DIRECT() : coil8Low_DIRECT())

#define coil1Charging_MC33810()      coil1High_MC33810()
#define coil1StopCharging_MC33810()  coil1Low_MC33810()
#define coil2Charging_MC33810()      coil2High_MC33810()
#define coil2StopCharging_MC33810()  coil2Low_MC33810()
#define coil3Charging_MC33810()      coil3High_MC33810()
#define coil3StopCharging_MC33810()  coil3Low_MC33810()
#define coil4Charging_MC33810()      coil4High_MC33810()
#define coil4StopCharging_MC33810()  coil4Low_MC33810()
#define coil5Charging_MC33810()      coil5High_MC33810()
#define coil5StopCharging_MC33810()  coil5Low_MC33810()
#define coil6Charging_MC33810()      coil6High_MC33810()
#define coil6StopCharging_MC33810()  coil6Low_MC33810()
#define coil7Charging_MC33810()      coil7High_MC33810()
#define coil7StopCharging_MC33810()  coil7Low_MC33810()
#define coil8Charging_MC33810()      coil8High_MC33810()
#define coil8StopCharging_MC33810()  coil8Low_MC33810()

#define coil1Toggle_DIRECT() (*ign1_pin_port ^= ign1_pin_mask )
#define coil2Toggle_DIRECT() (*ign2_pin_port ^= ign2_pin_mask )
#define coil3Toggle_DIRECT() (*ign3_pin_port ^= ign3_pin_mask )
#define coil4Toggle_DIRECT() (*ign4_pin_port ^= ign4_pin_mask )
#define coil5Toggle_DIRECT() (*ign5_pin_port ^= ign5_pin_mask )
#define coil6Toggle_DIRECT() (*ign6_pin_port ^= ign6_pin_mask )
#define coil7Toggle_DIRECT() (*ign7_pin_port ^= ign7_pin_mask )
#define coil8Toggle_DIRECT() (*ign8_pin_port ^= ign8_pin_mask )

#define injector1Toggle_DIRECT() (*inj1_pin_port ^= inj1_pin_mask )
#define injector2Toggle_DIRECT() (*inj2_pin_port ^= inj2_pin_mask )
#define injector3Toggle_DIRECT() (*inj3_pin_port ^= inj3_pin_mask )
#define injector4Toggle_DIRECT() (*inj4_pin_port ^= inj4_pin_mask )
#define injector5Toggle_DIRECT() (*inj5_pin_port ^= inj5_pin_mask )
#define injector6Toggle_DIRECT() (*inj6_pin_port ^= inj6_pin_mask )
#define injector7Toggle_DIRECT() (*inj7_pin_port ^= inj7_pin_mask )
#define injector8Toggle_DIRECT() (*inj8_pin_port ^= inj8_pin_mask )

void nullCallback();

#endif
