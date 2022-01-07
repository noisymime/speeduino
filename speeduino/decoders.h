#ifndef DECODERS_H
#define DECODERS_H

#include "globals.h"

#if defined(CORE_AVR)
  #define READ_PRI_TRIGGER() ((*triggerPri_pin_port & triggerPri_pin_mask) ? true : false)
  #define READ_SEC_TRIGGER() ((*triggerSec_pin_port & triggerSec_pin_mask) ? true : false)
#else
  #define READ_PRI_TRIGGER() digitalRead(pinTrigger)
  #define READ_SEC_TRIGGER() digitalRead(pinTrigger2)
#endif

#define DECODER_MISSING_TOOTH     0
#define DECODER_BASIC_DISTRIBUTOR 1
#define DECODER_DUAL_WHEEL        2
#define DECODER_GM7X              3
#define DECODER_4G63              4
#define DECODER_24X               5
#define DECODER_JEEP2000          6
#define DECODER_AUDI135           7
#define DECODER_HONDA_D17         8
#define DECODER_MIATA_9905        9
#define DECODER_MAZDA_AU          10
#define DECODER_NON360            11
#define DECODER_NISSAN_360        12
#define DECODER_SUBARU_67         13
#define DECODER_DAIHATSU_PLUS1    14
#define DECODER_HARLEY            15
#define DECODER_36_2_2_2          16
#define DECODER_36_2_1            17
#define DECODER_420A              18
#define DECODER_WEBER             19
#define DECODER_ST170             20
#define DECODER_DRZ400            21
#define DECODER_NGC               22

//This isn't to to filter out wrong pulses on triggers, but just to smooth out the cam angle reading for better closed loop VVT control.
#define ANGLE_FILTER(input, alpha, prior) (((long)input * (256 - alpha) + ((long)prior * alpha))) >> 8

void loggerPrimaryISR();
void loggerSecondaryISR();

//All of the below are the 6 required functions for each decoder / pattern
void triggerSetup_missingTooth();
void triggerPri_missingTooth();
void triggerSec_missingTooth();
void triggerThird_missingTooth();
uint16_t getRPM_missingTooth();
int getCrankAngle_missingTooth();
extern void triggerSetEndTeeth_missingTooth();

void triggerSetup_DualWheel();
void triggerPri_DualWheel();
void triggerSec_DualWheel();
uint16_t getRPM_DualWheel();
int getCrankAngle_DualWheel();
void triggerSetEndTeeth_DualWheel();

void triggerSetup_BasicDistributor();
void triggerPri_BasicDistributor();
void triggerSec_BasicDistributor();
uint16_t getRPM_BasicDistributor();
int getCrankAngle_BasicDistributor();
void triggerSetEndTeeth_BasicDistributor();

void triggerSetup_GM7X();
void triggerPri_GM7X();
void triggerSec_GM7X();
uint16_t getRPM_GM7X();
int getCrankAngle_GM7X();
void triggerSetEndTeeth_GM7X();

void triggerSetup_4G63();
void triggerPri_4G63();
void triggerSec_4G63();
uint16_t getRPM_4G63();
int getCrankAngle_4G63();
void triggerSetEndTeeth_4G63();

void triggerSetup_24X();
void triggerPri_24X();
void triggerSec_24X();
uint16_t getRPM_24X();
int getCrankAngle_24X();
void triggerSetEndTeeth_24X();

void triggerSetup_Jeep2000();
void triggerPri_Jeep2000();
void triggerSec_Jeep2000();
uint16_t getRPM_Jeep2000();
int getCrankAngle_Jeep2000();
void triggerSetEndTeeth_Jeep2000();

void triggerSetup_Audi135();
void triggerPri_Audi135();
void triggerSec_Audi135();
uint16_t getRPM_Audi135();
int getCrankAngle_Audi135();
void triggerSetEndTeeth_Audi135();

void triggerSetup_HondaD17();
void triggerPri_HondaD17();
void triggerSec_HondaD17();
uint16_t getRPM_HondaD17();
int getCrankAngle_HondaD17();
void triggerSetEndTeeth_HondaD17();

void triggerSetup_Miata9905();
void triggerPri_Miata9905();
void triggerSec_Miata9905();
uint16_t getRPM_Miata9905();
int getCrankAngle_Miata9905();
void triggerSetEndTeeth_Miata9905();
int getCamAngle_Miata9905();

void triggerSetup_MazdaAU();
void triggerPri_MazdaAU();
void triggerSec_MazdaAU();
uint16_t getRPM_MazdaAU();
int getCrankAngle_MazdaAU();
void triggerSetEndTeeth_MazdaAU();

void triggerSetup_non360();
void triggerPri_non360();
void triggerSec_non360();
uint16_t getRPM_non360();
int getCrankAngle_non360();
void triggerSetEndTeeth_non360();

void triggerSetup_Nissan360();
void triggerPri_Nissan360();
void triggerSec_Nissan360();
uint16_t getRPM_Nissan360();
int getCrankAngle_Nissan360();
void triggerSetEndTeeth_Nissan360();

void triggerSetup_Subaru67();
void triggerPri_Subaru67();
void triggerSec_Subaru67();
uint16_t getRPM_Subaru67();
int getCrankAngle_Subaru67();
void triggerSetEndTeeth_Subaru67();

void triggerSetup_Daihatsu();
void triggerPri_Daihatsu();
void triggerSec_Daihatsu();
uint16_t getRPM_Daihatsu();
int getCrankAngle_Daihatsu();
void triggerSetEndTeeth_Daihatsu();

void triggerSetup_Harley();
void triggerPri_Harley();
void triggerSec_Harley();
uint16_t getRPM_Harley();
int getCrankAngle_Harley();
void triggerSetEndTeeth_Harley();

void triggerSetup_ThirtySixMinus222();
void triggerPri_ThirtySixMinus222();
void triggerSec_ThirtySixMinus222();
uint16_t getRPM_ThirtySixMinus222();
int getCrankAngle_ThirtySixMinus222();
void triggerSetEndTeeth_ThirtySixMinus222();

void triggerSetup_ThirtySixMinus21();
void triggerPri_ThirtySixMinus21();
void triggerSec_ThirtySixMinus21();
uint16_t getRPM_ThirtySixMinus21();
int getCrankAngle_ThirtySixMinus21();
void triggerSetEndTeeth_ThirtySixMinus21();

void triggerSetup_420a();
void triggerPri_420a();
void triggerSec_420a();
uint16_t getRPM_420a();
int getCrankAngle_420a();
void triggerSetEndTeeth_420a();

void triggerPri_Webber();
void triggerSec_Webber();

void triggerSetup_FordST170();
void triggerSec_FordST170();
uint16_t getRPM_FordST170();
int getCrankAngle_FordST170();
void triggerSetEndTeeth_FordST170();

void triggerSetup_DRZ400();
void triggerSec_DRZ400();

void triggerSetup_NGC();
void triggerPri_NGC();
void triggerSec_NGC4();
void triggerSec_NGC68();
uint16_t getRPM_NGC();
void triggerSetEndTeeth_NGC();

extern void (*triggerHandler)(); //Pointer for the trigger function (Gets pointed to the relevant decoder)
extern void (*triggerSecondaryHandler)(); //Pointer for the secondary trigger function (Gets pointed to the relevant decoder)
extern void (*triggerTertiaryHandler)(); //Pointer for the tertiary trigger function (Gets pointed to the relevant decoder)

extern uint16_t (*getRPM)(); //Pointer to the getRPM function (Gets pointed to the relevant decoder)
extern int (*getCrankAngle)(); //Pointer to the getCrank Angle function (Gets pointed to the relevant decoder)
extern void (*triggerSetEndTeeth)(); //Pointer to the triggerSetEndTeeth function of each decoder

extern volatile unsigned long curTime;
extern volatile unsigned long curGap;
extern volatile unsigned long curTime2;
extern volatile unsigned long curGap2;
extern volatile unsigned long curTime3;
extern volatile unsigned long curGap3;
extern volatile unsigned long lastGap;
extern volatile unsigned long targetGap;

extern unsigned long MAX_STALL_TIME; //The maximum time (in uS) that the system will continue to function before the engine is considered stalled/stopped. This is unique to each decoder, depending on the number of teeth etc. 500000 (half a second) is used as the default value, most decoders will be much less.
extern volatile uint16_t toothCurrentCount; //The current number of teeth (Onec sync has been achieved, this can never actually be 0
extern volatile byte toothSystemCount; //Used for decoders such as Audi 135 where not every tooth is used for calculating crank angle. This variable stores the actual number of teeth, not the number being used to calculate crank angle
extern volatile unsigned long toothSystemLastToothTime; //As below, but used for decoders where not every tooth count is used for calculation
extern volatile unsigned long toothLastToothTime; //The time (micros()) that the last tooth was registered
extern volatile unsigned long toothLastSecToothTime; //The time (micros()) that the last tooth was registered on the secondary input
extern volatile unsigned long toothLastThirdToothTime; //The time (micros()) that the last tooth was registered on the second cam input
extern volatile unsigned long toothLastMinusOneToothTime; //The time (micros()) that the tooth before the last tooth was registered
extern volatile unsigned long toothLastMinusOneSecToothTime; //The time (micros()) that the tooth before the last tooth was registered on secondary input
extern volatile unsigned long targetGap2;

extern volatile unsigned long toothOneTime; //The time (micros()) that tooth 1 last triggered
extern volatile unsigned long toothOneMinusOneTime; //The 2nd to last time (micros()) that tooth 1 last triggered
extern volatile bool revolutionOne; // For sequential operation, this tracks whether the current revolution is 1 or 2 (not 1)

extern volatile unsigned int secondaryToothCount; //Used for identifying the current secondary (Usually cam) tooth for patterns with multiple secondary teeth
extern volatile unsigned long secondaryLastToothTime; //The time (micros()) that the last tooth was registered (Cam input)
extern volatile unsigned long secondaryLastToothTime1; //The time (micros()) that the last tooth was registered (Cam input)

extern volatile uint16_t triggerActualTeeth;
extern volatile unsigned long triggerFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering)
extern volatile unsigned long triggerSecFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering) for the secondary input
extern volatile bool validTrigger; //Is set true when the last trigger (Primary or secondary) was valid (ie passed filters)
extern unsigned int triggerSecFilterTime_duration; // The shortest valid time (in uS) pulse DURATION
extern volatile uint16_t triggerToothAngle; //The number of crank degrees that elapse per tooth
extern volatile bool triggerToothAngleIsCorrect; //Whether or not the triggerToothAngle variable is currently accurate. Some patterns have times when the triggerToothAngle variable cannot be accurately set.
extern bool secondDerivEnabled; //The use of the 2nd derivative calculation is limited to certain decoders. This is set to either true or false in each decoders setup routine
extern bool decoderIsSequential; //Whether or not the decoder supports sequential operation
extern bool decoderIsLowRes; //Is set true, certain extra calculations are performed for better timing accuracy
extern bool decoderHasSecondary; //Whether or not the pattern uses a secondary input
extern bool decoderHasFixedCrankingTiming; //Whether or not the decoder supports fixed cranking timing
extern byte checkSyncToothCount; //How many teeth must've been seen on this revolution before we try to confirm sync (Useful for missing tooth type decoders)
extern unsigned long elapsedTime;
extern unsigned long lastCrankAngleCalc;
extern int16_t lastToothCalcAdvance; //Invalid value here forces calculation of this on first main loop
extern unsigned long lastVVTtime; //The time between the vvt reference pulse and the last crank pulse

extern uint16_t ignition1EndTooth;
extern uint16_t ignition2EndTooth;
extern uint16_t ignition3EndTooth;
extern uint16_t ignition4EndTooth;
extern uint16_t ignition5EndTooth;
extern uint16_t ignition6EndTooth;
extern uint16_t ignition7EndTooth;
extern uint16_t ignition8EndTooth;

extern int16_t toothAngles[24]; //An array for storing fixed tooth angles. Currently sized at 24 for the GM 24X decoder, but may grow later if there are other decoders that use this style

//Used for identifying long and short pulses on the 4G63 (And possibly other) trigger patterns
#define LONG 0;
#define SHORT 1;

#define CRANK_SPEED 0
#define CAM_SPEED   1

#define TOOTH_CRANK 0
#define TOOTH_CAM   1

#endif
