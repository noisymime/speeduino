/*
All functions in the gamma file return

*/
#ifndef CORRECTIONS_H
#define CORRECTIONS_H

#include <stdint.h>

void initialiseCorrections(void);
uint16_t correctionsFuel(void);
uint8_t calculateAfrTarget(table3d16RpmLoad &afrLookUpTable, const statuses &current, const config2 &page2, const config6 &page6);
byte correctionWUE(void); //Warmup enrichment
uint16_t correctionCranking(void); //Cranking enrichment
byte correctionASE(void); //After Start Enrichment
uint16_t correctionAccel(void); //Acceleration Enrichment
byte correctionFloodClear(void); //Check for flood clear on cranking
byte correctionAFRClosedLoop(void); //Closed loop AFR adjustment
byte correctionFlex(void); //Flex fuel adjustment
byte correctionFuelTemp(void); //Fuel temp correction
byte correctionBatVoltage(void); //Battery voltage correction
byte correctionIATDensity(void); //Inlet temp density correction
byte correctionBaro(void); //Barometric pressure correction
byte correctionLaunch(void); //Launch control correction
byte correctionDFCOfuel(void); //DFCO taper correction
bool correctionDFCO(void); //Decelleration fuel cutoff

int8_t correctionsIgn(int8_t advance);
int8_t correctionFixedTiming(int8_t advance);
int8_t correctionCrankingFixedTiming(int8_t advance);
int8_t correctionFlexTiming(int8_t advance);
int8_t correctionWMITiming(int8_t advance);
int8_t correctionKnockTiming(int8_t advance);
int8_t correctionIATretard(int8_t advance);
int8_t correctionCLTadvance(int8_t advance);
int8_t correctionIdleAdvance(int8_t advance);
int8_t correctionSoftRevLimit(int8_t advance);
int8_t correctionNitrous(int8_t advance);
int8_t correctionSoftLaunch(int8_t advance);
int8_t correctionSoftFlatShift(int8_t advance);
int8_t correctionKnock(int8_t advance);
int8_t correctionDFCOignition(int8_t advance);
uint16_t correctionsDwell(uint16_t dwell);

bool isFixedTimingOn(void);

extern uint16_t AFRnextCycle;
extern uint8_t aseTaper;

#endif // CORRECTIONS_H
