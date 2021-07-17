/*
All functions in the gamma file return

*/
#ifndef CORRECTIONS_H
#define CORRECTIONS_H

void initialiseCorrections();

uint16_t correctionsFuel();
byte correctionWUE(); //Warmup enrichment
uint16_t correctionCranking(); //Cranking enrichment
byte correctionASE(); //After Start Enrichment
uint16_t correctionAccel(); //Acceleration Enrichment
byte correctionFloodClear(); //Check for flood clear on cranking
byte correctionAFRClosedLoop(); //Closed loop AFR adjustment
byte correctionFlex(); //Flex fuel adjustment
byte correctionFuelTemp(); //Fuel temp correction
byte correctionBatVoltage(); //Battery voltage correction
byte correctionIATDensity(); //Inlet temp density correction
byte correctionBaro(); //Barometric pressure correction
byte correctionLaunch(); //Launch control correction
bool correctionDFCO(); //Decelleration fuel cutoff

smallAngle_t correctionsIgn(smallAngle_t advance);
smallAngle_t correctionFixedTiming(smallAngle_t);
smallAngle_t correctionCrankingFixedTiming(smallAngle_t);
smallAngle_t correctionFlexTiming(smallAngle_t);
smallAngle_t correctionWMITiming(smallAngle_t);
smallAngle_t correctionIATretard(smallAngle_t);
smallAngle_t correctionCLTadvance(smallAngle_t);
smallAngle_t correctionIdleAdvance(smallAngle_t);
smallAngle_t correctionSoftRevLimit(smallAngle_t);
smallAngle_t correctionNitrous(smallAngle_t);
smallAngle_t correctionSoftLaunch(smallAngle_t);
smallAngle_t correctionSoftFlatShift(smallAngle_t);
smallAngle_t correctionKnock(smallAngle_t);

uint16_t correctionsDwell(uint16_t dwell);

extern int MAP_rateOfChange;
extern int TPS_rateOfChange;
extern byte activateMAPDOT; //The mapDOT value seen when the MAE was activated. 
extern byte activateTPSDOT; //The tpsDOT value seen when the MAE was activated. 

extern uint16_t AFRnextCycle;
extern unsigned long knockStartTime;
extern byte lastKnockCount;
extern bigAngle_t knockWindowMin; //The current minimum crank angle for a knock pulse to be valid
extern bigAngle_t knockWindowMax;//The current maximum crank angle for a knock pulse to be valid
extern uint16_t aseTaperStart;
extern uint16_t dfcoStart;

#endif // CORRECTIONS_H
