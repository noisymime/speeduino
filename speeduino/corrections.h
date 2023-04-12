/*
All functions in the gamma file return

*/
#ifndef CORRECTIONS_H
#define CORRECTIONS_H

#define IGN_IDLE_THRESHOLD 200 //RPM threshold (below CL idle target) for when ign based idle control will engage

void initialiseCorrections(void);
uint16_t correctionsFuel(void);
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
bool correctionDFCO(void); //Decelleration fuel cutoff


int8_t correctionsIgn(int8_t advance);
int8_t correctionFixedTiming(int8_t advance);
int8_t correctionCrankingFixedTiming(int8_t advance);
int8_t correctionFlexTiming(int8_t advance);
int8_t correctionWMITiming(int8_t advance);
int8_t correctionIATretard(int8_t advance);
int8_t correctionCLTadvance(int8_t advance);
int8_t correctionIdleAdvance(int8_t advance);
int8_t correctionSoftRevLimit(int8_t advance);
int8_t correctionNitrous(int8_t advance);
int8_t correctionSoftLaunch(int8_t advance);
int8_t correctionSoftFlatShift(int8_t advance);
int8_t correctionKnock(int8_t advance);

uint16_t correctionsDwell(uint16_t dwell);

extern byte activateMAPDOT; //The mapDOT value seen when the MAE was activated. 
extern byte activateTPSDOT; //The tpsDOT value seen when the MAE was activated. 

extern uint16_t AFRnextCycle;
extern unsigned long knockStartTime;
extern byte lastKnockCount;
extern int16_t knockWindowMin; //The current minimum crank angle for a knock pulse to be valid
extern int16_t knockWindowMax;//The current maximum crank angle for a knock pulse to be valid
extern uint8_t aseTaper;
extern uint8_t dfcoTaper;
extern uint8_t idleAdvTaper;
extern uint8_t crankingEnrichTaper;

#endif // CORRECTIONS_H
