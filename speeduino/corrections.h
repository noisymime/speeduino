/*
All functions in the gamma file return

*/
#ifndef CORRECTIONS_H
#define CORRECTIONS_H

#define IGN_IDLE_THRESHOLD 200 //RPM threshold (below CL idle target) for when ign based idle control will engage

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

static inline bool isFixedTimingOn(void) {
            // Fixed timing is in effect
    return  configPage2.fixAngEnable == 1U
            // Cranking, so the cranking advance angle is in effect
            || BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK);
}

uint16_t correctionsDwell(uint16_t dwell);

extern byte activateMAPDOT; //The mapDOT value seen when the MAE was activated. 
extern byte activateTPSDOT; //The tpsDOT value seen when the MAE was activated. 

extern uint16_t AFRnextCycle;
extern uint8_t aseTaper;
extern uint8_t dfcoDelay;
extern uint8_t idleAdvTaper;
extern uint8_t crankingEnrichTaper;
extern uint8_t dfcoTaper;

#endif // CORRECTIONS_H
