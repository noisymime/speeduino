/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/** @file
 * 
 * Crank and Cam decoders
 * 
 * This file contains the various crank and cam wheel decoder functions.
 * Each decoder must have the following 4 functions (Where xxxx is the decoder name):
 * 
 * - **triggerSetup_xxxx** - Called once from within setup() and configures any required variables
 * - **triggerPri_xxxx** - Called each time the primary (No. 1) crank/cam signal is triggered (Called as an interrupt, so variables must be declared volatile)
 * - **triggerSec_xxxx** - Called each time the secondary (No. 2) crank/cam signal is triggered (Called as an interrupt, so variables must be declared volatile)
 * - **getRPM_xxxx** - Returns the current RPM, as calculated by the decoder
 * - **getCrankAngle_xxxx** - Returns the current crank angle, as calculated by the decoder
 * - **getCamAngle_xxxx** - Returns the current CAM angle, as calculated by the decoder
 *
 * Each decoder must utilise at least the following variables:
 * 
 * - toothLastToothTime - The time (In uS) that the last primary tooth was 'seen'
 */

/* Notes on Doxygen Groups/Modules documentation style:
 * - Installing doxygen (e.g. Ubuntu) via pkg mgr: sudo apt-get install doxygen graphviz
 * - @defgroup tag name/description becomes the short name on (Doxygen) "Modules" page
 * - Relying on JAVADOC_AUTOBRIEF (in Doxyfile, essentially automatic @brief), the first sentence (ending with period) becomes
 *   the longer description (second column following name) on (Doxygen) "Modules" page (old Desc: ... could be this sentence)
 * - All the content after first sentence (like old Note:...) is visible on the page linked from the name (1st col) on "Modules" page
 * - To group all decoders together add 1) @defgroup dec Decoders (on top) and 2) "@ingroup dec" to each decoder (under @defgroup)
 * - To compare Speeduino Doxyfile to default config, do: `doxygen -g Doxyfile.default ; diff Doxyfile.default Doxyfile`
 */
#include <limits.h>
#include "globals.h"
#include "decoders.h"
#include "scheduledIO.h"
#include "scheduler.h"
#include "crankMaths.h"

void (*triggerHandler)(); ///Pointer for the trigger function (Gets pointed to the relevant decoder)
void (*triggerSecondaryHandler)(); ///Pointer for the secondary trigger function (Gets pointed to the relevant decoder)
void (*triggerTertiaryHandler)(); ///Pointer for the tertiary trigger function (Gets pointed to the relevant decoder)
uint16_t (*getRPM)(); ///Pointer to the getRPM function (Gets pointed to the relevant decoder)
bigAngle_t (*getCrankAngle)(); ///Pointer to the getCrank Angle function (Gets pointed to the relevant decoder)
void (*triggerSetEndTeeth)(); ///Pointer to the triggerSetEndTeeth function of each decoder

volatile unsigned long curTime;
volatile unsigned long curGap;
volatile unsigned long curTime2;
volatile unsigned long curGap2;
volatile unsigned long curTime3;
volatile unsigned long curGap3;
volatile unsigned long lastGap;
volatile unsigned long targetGap;
volatile unsigned long compositeLastToothTime;

unsigned long MAX_STALL_TIME = 500000UL; //The maximum time (in uS) that the system will continue to function before the engine is considered stalled/stopped. This is unique to each decoder, depending on the number of teeth etc. 500000 (half a second) is used as the default value, most decoders will be much less.
volatile uint16_t toothCurrentCount = 0; //The current number of teeth (Onec sync has been achieved, this can never actually be 0
volatile byte toothSystemCount = 0; //Used for decoders such as Audi 135 where not every tooth is used for calculating crank angle. This variable stores the actual number of teeth, not the number being used to calculate crank angle
volatile unsigned long toothSystemLastToothTime = 0; //As below, but used for decoders where not every tooth count is used for calculation
volatile unsigned long toothLastToothTime = 0; //The time (micros()) that the last tooth was registered
volatile unsigned long toothLastSecToothTime = 0; //The time (micros()) that the last tooth was registered on the secondary input
volatile unsigned long toothLastThirdToothTime = 0; //The time (micros()) that the last tooth was registered on the second cam input
volatile unsigned long toothLastMinusOneToothTime = 0; //The time (micros()) that the tooth before the last tooth was registered
volatile unsigned long toothLastMinusOneSecToothTime = 0; //The time (micros()) that the tooth before the last tooth was registered on secondary input
volatile unsigned long targetGap2;
volatile unsigned long toothOneTime = 0; //The time (micros()) that tooth 1 last triggered
volatile unsigned long toothOneMinusOneTime = 0; //The 2nd to last time (micros()) that tooth 1 last triggered
volatile bool revolutionOne = 0; // For sequential operation, this tracks whether the current revolution is 1 or 2 (not 1)

volatile unsigned int secondaryToothCount; //Used for identifying the current secondary (Usually cam) tooth for patterns with multiple secondary teeth
volatile unsigned long secondaryLastToothTime = 0; //The time (micros()) that the last tooth was registered (Cam input)
volatile unsigned long secondaryLastToothTime1 = 0; //The time (micros()) that the last tooth was registered (Cam input)

volatile uint16_t triggerActualTeeth;
volatile unsigned long triggerFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering)
volatile unsigned long triggerSecFilterTime; // The shortest time (in uS) that pulses will be accepted (Used for debounce filtering) for the secondary input
volatile bool validTrigger; //Is set true when the last trigger (Primary or secondary) was valid (ie passed filters)
unsigned int triggerSecFilterTime_duration; // The shortest valid time (in uS) pulse DURATION
volatile bigAngle_t triggerToothAngle; //The number of crank degrees that elapse per tooth
volatile bool triggerToothAngleIsCorrect = false; //Whether or not the triggerToothAngle variable is currently accurate. Some patterns have times when the triggerToothAngle variable cannot be accurately set.
bool secondDerivEnabled = false; //The use of the 2nd derivative calculation is limited to certain decoders. This is set to either true or false in each decoders setup routine
bool decoderIsSequential; //Whether or not the decoder supports sequential operation
bool decoderIsLowRes = false; //Is set true, certain extra calculations are performed for better timing accuracy
bool decoderHasSecondary = false; //Whether or not the pattern uses a secondary input
bool decoderHasFixedCrankingTiming = false; //Whether or not the decoder supports fixed cranking timing
byte checkSyncToothCount; //How many teeth must've been seen on this revolution before we try to confirm sync (Useful for missing tooth type decoders)
unsigned long elapsedTime;
unsigned long lastCrankAngleCalc;
int16_t lastToothCalcAdvance = 99; //Invalid value here forces calculation of this on first main loop
unsigned long lastVVTtime; //The time between the vvt reference pulse and the last crank pulse

uint16_t ignition1EndTooth = 0;
uint16_t ignition2EndTooth = 0;
uint16_t ignition3EndTooth = 0;
uint16_t ignition4EndTooth = 0;
uint16_t ignition5EndTooth = 0;
uint16_t ignition6EndTooth = 0;
uint16_t ignition7EndTooth = 0;
uint16_t ignition8EndTooth = 0;

int16_t toothAngles[24]; //An array for storing fixed tooth angles. Currently sized at 24 for the GM 24X decoder, but may grow later if there are other decoders that use this style


/** Universal (shared between decoders) decoder routines.
*
* @defgroup dec_uni Universal Decoder Routines
* 
* @{
*/
// whichTooth - 0 for Primary (Crank), 1 for Secondary (Cam)

/** Add tooth log entry to toothHistory (array).
 * Enabled by (either) currentStatus.toothLogEnabled and currentStatus.compositeLogEnabled.
 * @param toothTime - Tooth Time
 * @param whichTooth - 0 for Primary (Crank), 1 for Secondary (Cam)
 */
static inline void addToothLogEntry(unsigned long toothTime, bool whichTooth)
{
  if(BIT_CHECK(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY)) { return; }
  //High speed tooth logging history
  if( (currentStatus.toothLogEnabled == true) || (currentStatus.compositeLogEnabled == true) ) 
  {
    bool valueLogged = false;
    if(currentStatus.toothLogEnabled == true)
    {
      //Tooth log only works on the Crank tooth
      if(whichTooth == TOOTH_CRANK)
      { 
        toothHistory[toothHistoryIndex] = toothTime; //Set the value in the log. 
        valueLogged = true;
      } 
    }
    else if(currentStatus.compositeLogEnabled == true)
    {
      compositeLogHistory[toothHistoryIndex] = 0;
      if(READ_PRI_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_PRI); }
      if(READ_SEC_TRIGGER() == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SEC); }
      if(whichTooth == TOOTH_CAM) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_TRIG); }
      if(currentStatus.hasSync == true) { BIT_SET(compositeLogHistory[toothHistoryIndex], COMPOSITE_LOG_SYNC); }

      toothHistory[toothHistoryIndex] = micros() - compositeLastToothTime;
      compositeLastToothTime = micros();
      valueLogged = true;
    }

    //If there has been a value logged above, update the indexes
    if(valueLogged == true)
    {
      if(toothHistoryIndex == (TOOTH_LOG_BUFFER-1)) { toothHistoryIndex = 0; }
      else { toothHistoryIndex++; }

      uint16_t absoluteToothHistoryIndex = toothHistoryIndex;
      if(toothHistoryIndex < toothHistorySerialIndex)
      {
        //If the main history index is lower than the serial index, it means that this has looped. To calculate the delta between the two indexes, add the buffer size back on 
        absoluteToothHistoryIndex += TOOTH_LOG_BUFFER;
      }
      //Check whether the current index is ahead of the serial index by at least the size of the log
      if( (absoluteToothHistoryIndex - toothHistorySerialIndex) >= TOOTH_LOG_SIZE ) { BIT_SET(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY); }
      else { BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY); } //Tooth log is not yet ahead of the serial index by enough, so mark the log as not yet ready
    }


  } //Tooth/Composite log enabled
}

/** Interrupt handler for primary trigger.
* This function is called on both the rising and falling edges of the primary trigger, when either the 
* composite or tooth loggers are turned on. 
*/
void loggerPrimaryISR()
{
  validTrigger = false; //This value will be set to the return value of the decoder function, indicating whether or not this pulse passed the filters
  bool validEdge = false; //This is set true below if the edge 
  /* 
  Need to still call the standard decoder trigger. 
  Two checks here:
  1) If the primary trigger is RISING, then check whether the primary is currently HIGH
  2) If the primary trigger is FALLING, then check whether the primary is currently LOW
  If either of these are true, the primary decoder funtino is called
  */
  if( ( (primaryTriggerEdge == RISING) && (READ_PRI_TRIGGER() == HIGH) ) || ( (primaryTriggerEdge == FALLING) && (READ_PRI_TRIGGER() == LOW) ) || (primaryTriggerEdge == CHANGE) )
  {
    triggerHandler();
    validEdge = true;
  }
  if( (currentStatus.toothLogEnabled == true) && (validTrigger == true) )
  {
    //Tooth logger only logs when the edge was correct
    if(validEdge == true) { addToothLogEntry(curGap, TOOTH_CRANK); }
  }
  //else if( (currentStatus.compositeLogEnabled == true) && (validTrigger == true) )
  else if( (currentStatus.compositeLogEnabled == true) )
  {
    //Composite logger adds an entry regardless of which edge it was
    addToothLogEntry(curGap, TOOTH_CRANK);
  }
}

/** Interrupt handler for secondary trigger.
* As loggerPrimaryISR, but for the secondary trigger.
*/
void loggerSecondaryISR()
{
  validTrigger = false; //This value will be set to the return value of the decoder function, indicating whether or not this pulse passed the filters
  validTrigger = true;
  /* 3 checks here:
  1) If the primary trigger is RISING, then check whether the primary is currently HIGH
  2) If the primary trigger is FALLING, then check whether the primary is currently LOW
  3) The secondary trigger is CHANGING
  If any of these are true, the primary decoder funtion is called
  */
  if( ( (secondaryTriggerEdge == RISING) && (READ_SEC_TRIGGER() == HIGH) ) || ( (secondaryTriggerEdge == FALLING) && (READ_SEC_TRIGGER() == LOW) ) || (secondaryTriggerEdge == CHANGE) )
  {
    triggerSecondaryHandler();
  }
  //No tooth logger for the secondary input
  if( (currentStatus.compositeLogEnabled == true) && (validTrigger == true) )
  {
    //Composite logger adds an entry regardless of which edge it was
    addToothLogEntry(curGap2, TOOTH_CAM);
  }
}

/** Compute RPM.
* As nearly all the decoders use a common method of determining RPM (The time the last full revolution took) A common function is simpler.
* @param degreesOver - the number of crank degrees between tooth #1s. Some patterns have a tooth #1 every crank rev, others are every cam rev.
* @return RPM
*/
static inline uint16_t stdGetRPM(uint16_t degreesOver)
{
  uint16_t tempRPM = 0;

  if( currentStatus.hasSync == true )
  {
    if( (currentStatus.RPM < currentStatus.crankRPM) && (currentStatus.startRevolutions == 0) ) { tempRPM = 0; } //Prevents crazy RPM spike when there has been less than 1 full revolution
    else if( (toothOneTime == 0) || (toothOneMinusOneTime == 0) ) { tempRPM = 0; }
    else
    {
      noInterrupts();
      revolutionTime = (toothOneTime - toothOneMinusOneTime); //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
      interrupts();
      if(degreesOver == scaleCrankAngle(720)) { revolutionTime = revolutionTime / 2; }
      tempRPM = (US_IN_MINUTE / revolutionTime); //Calc RPM based on last full revolution time (Faster as /)
      if(tempRPM >= MAX_RPM) { tempRPM = currentStatus.RPM; } //Sanity check
    }
  }
  else { tempRPM = 0; }

  return tempRPM;
}

/**
 * Sets the new filter time based on the current settings.
 * This ONLY works for even spaced decoders.
 */
static inline void setFilter(unsigned long curGap)
{
  if(configPage4.triggerFilter == 0) { triggerFilterTime = 0; } //trigger filter is turned off.
  else if(configPage4.triggerFilter == 1) { triggerFilterTime = curGap >> 2; } //Lite filter level is 25% of previous gap
  else if(configPage4.triggerFilter == 2) { triggerFilterTime = curGap >> 1; } //Medium filter level is 50% of previous gap
  else if (configPage4.triggerFilter == 3) { triggerFilterTime = (curGap * 3) >> 2; } //Aggressive filter level is 75% of previous gap
  else { triggerFilterTime = 0; } //trigger filter is turned off.
}

/**
This is a special case of RPM measure that is based on the time between the last 2 teeth rather than the time of the last full revolution.
This gives much more volatile reading, but is quite useful during cranking, particularly on low resolution patterns.
It can only be used on patterns where the teeth are evently spaced.
It takes an argument of the full (COMPLETE) number of teeth per revolution.
For a missing tooth wheel, this is the number if the tooth had NOT been missing (Eg 36-1 = 36)
*/
static inline int crankingGetRPM(byte totalTeeth, bigAngle_t degreesOver)
{
  uint16_t tempRPM = 0;
  if( (currentStatus.startRevolutions >= configPage4.StgCycles) && (currentStatus.hasSync == true) )
  {
    if( (toothLastToothTime > 0) && (toothLastMinusOneToothTime > 0) && (toothLastToothTime > toothLastMinusOneToothTime) )
    {
      noInterrupts();
      revolutionTime = (toothLastToothTime - toothLastMinusOneToothTime) * totalTeeth;
      interrupts();
      if(degreesOver == scaleCrankAngle(720)) { revolutionTime = revolutionTime / 2; }
      tempRPM = (US_IN_MINUTE / revolutionTime);
      if( tempRPM >= MAX_RPM ) { tempRPM = currentStatus.RPM; } //Sanity check. This can prevent spiking caused by noise on individual teeth. The new RPM should never be above 4x the cranking setting value (Remembering that this function is only called is the current RPM is less than the cranking setting)
    }
  }

  return tempRPM;
}

/**
On decoders that are enabled for per tooth based timing adjustments, this function performs the timer compare changes on the schedules themselves
For each ignition channel, a check is made whether we're at the relevant tooth and whether that ignition schedule is currently running
Only if both these conditions are met will the schedule be updated with the latest timing information.
If it's the correct tooth, but the schedule is not yet started, calculate and an end compare value (This situation occurs when both the start and end of the ignition pulse happen after the end tooth, but before the next tooth)
*/
#define MIN_CYCLES_FOR_ENDCOMPARE 6
static inline void checkPerToothTiming(bigAngle_t crankAngle, uint16_t currentTooth)
{
  if ( (fixedCrankingOverride == 0) && (currentStatus.RPM > 0) )
  {
    if ( (currentTooth == ignition1EndTooth) )
    {
      if( (ignitionSchedule1.Status == RUNNING) ) { IGN1_COMPARE = IGN1_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition1EndAngle - crankAngle) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule1.endCompare = IGN1_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition1EndAngle - crankAngle) ) ) ); ignitionSchedule1.endScheduleSetByDecoder = true; }
    }
    else if ( (currentTooth == ignition2EndTooth) )
    {
      if( (ignitionSchedule2.Status == RUNNING) ) { IGN2_COMPARE = IGN2_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition2EndAngle - crankAngle) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule2.endCompare = IGN2_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition2EndAngle - crankAngle) ) ) ); ignitionSchedule2.endScheduleSetByDecoder = true; }
    }
    else if ( (currentTooth == ignition3EndTooth) )
    {
      if( (ignitionSchedule3.Status == RUNNING) ) { IGN3_COMPARE = IGN3_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition3EndAngle - crankAngle) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule3.endCompare = IGN3_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition3EndAngle - crankAngle) ) ) ); ignitionSchedule3.endScheduleSetByDecoder = true; }
    }
    else if ( (currentTooth == ignition4EndTooth) )
    {
      if( (ignitionSchedule4.Status == RUNNING) ) { IGN4_COMPARE = IGN4_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition4EndAngle - crankAngle) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule4.endCompare = IGN4_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition4EndAngle - crankAngle) ) ) ); ignitionSchedule4.endScheduleSetByDecoder = true; }
    }
#if IGN_CHANNELS >= 5
    else if ( (currentTooth == ignition5EndTooth) )
    {
      if( (ignitionSchedule5.Status == RUNNING) ) { IGN5_COMPARE = IGN5_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition5EndAngle - crankAngle) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule5.endCompare = IGN5_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition5EndAngle - crankAngle) ) ) ); ignitionSchedule5.endScheduleSetByDecoder = true; }
    }
#endif
#if IGN_CHANNELS >= 6
    else if ( (currentTooth == ignition6EndTooth) )
    {
      if( (ignitionSchedule6.Status == RUNNING) ) { IGN6_COMPARE = IGN6_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition6EndAngle - crankAngle) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule6.endCompare = IGN6_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition6EndAngle - crankAngle) ) ) ); ignitionSchedule6.endScheduleSetByDecoder = true; }
    }
#endif
#if IGN_CHANNELS >= 7
    else if ( (currentTooth == ignition7EndTooth) )
    {
      if( (ignitionSchedule7.Status == RUNNING) ) { IGN7_COMPARE = IGN7_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition7EndAngle - crankAngle) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule7.endCompare = IGN7_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition7EndAngle - crankAngle) ) ) ); ignitionSchedule7.endScheduleSetByDecoder = true; }
    }
#endif
#if IGN_CHANNELS >= 8
    else if ( (currentTooth == ignition8EndTooth) )
    {
      if( (ignitionSchedule8.Status == RUNNING) ) { IGN8_COMPARE = IGN8_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition8EndAngle - crankAngle) ) ) ); }
      else if(currentStatus.startRevolutions > MIN_CYCLES_FOR_ENDCOMPARE) { ignitionSchedule8.endCompare = IGN8_COUNTER + uS_TO_TIMER_COMPARE( fastDegreesToUS( ignitionLimits( (ignition8EndAngle - crankAngle) ) ) ); ignitionSchedule8.endScheduleSetByDecoder = true; }
    }
#endif
  }
}
/** @} */
  
/** A (single) multi-tooth wheel with one of more 'missing' teeth.
* The first tooth after the missing one is considered number 1 and is the basis for the trigger angle.
* Note: This decoder does not currently support dual wheel (ie missing tooth + single tooth on cam).
* @defgroup dec_miss Missing tooth wheel
* @{
*/
void triggerSetup_missingTooth()
{
  triggerToothAngle = scaleCrankAngle(360) / configPage4.triggerTeeth; //The number of degrees that passes from tooth to tooth
  if(configPage4.TrigSpeed == CAM_SPEED) { triggerToothAngle = scaleCrankAngle(720) / configPage4.triggerTeeth; } //Account for cam speed missing tooth
  triggerActualTeeth = configPage4.triggerTeeth - configPage4.triggerMissingTeeth; //The number of physical teeth on the wheel. Doing this here saves us a calculation each time in the interrupt
  triggerFilterTime = (1000000 / (MAX_RPM / 60 * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be disgarded as noise
  if (configPage4.trigPatternSec == SEC_TRIGGER_4_1)
  {
    triggerSecFilterTime = 1000000 * 60 / MAX_RPM / 4 / 2;
  }
  else 
  {
    triggerSecFilterTime = (1000000 / (MAX_RPM / 60));
  }
  secondDerivEnabled = false;
  decoderIsSequential = false;
  checkSyncToothCount = (configPage4.triggerTeeth) >> 1; //50% of the total teeth.
  toothLastMinusOneToothTime = 0;
  toothCurrentCount = 0;
  secondaryToothCount = 0; 
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  MAX_STALL_TIME = (3333UL * scaleCrankAngleDown(triggerToothAngle) * (configPage4.triggerMissingTeeth + 1)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}

void triggerPri_missingTooth()
{
   curTime = micros();
   curGap = curTime - toothLastToothTime;
   if ( curGap >= triggerFilterTime ) //Pulses should never be less than triggerFilterTime, so if they are it means a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
   {
     toothCurrentCount++; //Increment the tooth counter
     validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

     //if(toothCurrentCount > checkSyncToothCount || currentStatus.hasSync == false)
      if( (toothLastToothTime > 0) && (toothLastMinusOneToothTime > 0) )
      {
        bool isMissingTooth = false;

        /*
        Performance Optimisation:
        Only need to try and detect the missing tooth if:
        1. WE don't have sync yet
        2. We have sync and are in the final 1/4 of the wheel (Missing tooth will/should never occur in the first 3/4)
        3. RPM is under 2000. This is to ensure that we don't interfer with strange timing when cranking or idling. Optimisation not really required at these speeds anyway
        */
        if( (currentStatus.hasSync == false) || (currentStatus.RPM < 2000) || (toothCurrentCount >= (3 * triggerActualTeeth >> 2)) )
        {
          //Begin the missing tooth detection
          //If the time between the current tooth and the last is greater than 1.5x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after the gap
          if(configPage4.triggerMissingTeeth == 1) { targetGap = (3 * (toothLastToothTime - toothLastMinusOneToothTime)) >> 1; } //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one) (Uses bitshift to multiply by 3 then divide by 2. Much faster than multiplying by 1.5)
          else { targetGap = ((toothLastToothTime - toothLastMinusOneToothTime)) * configPage4.triggerMissingTeeth; } //Multiply by 2 (Checks for a gap 2x greater than the last one)

          if( (toothLastToothTime == 0) || (toothLastMinusOneToothTime == 0) ) { curGap = 0; }

          if ( (curGap > targetGap) || (toothCurrentCount > triggerActualTeeth) )
          {
            //Missing tooth detected
            isMissingTooth = true;
            if( (toothCurrentCount < triggerActualTeeth) && (currentStatus.hasSync == true) ) 
            { 
                //This occurs when we're at tooth #1, but haven't seen all the other teeth. This indicates a signal issue so we flag lost sync so this will attempt to resync on the next revolution.
                currentStatus.hasSync = false;
                BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); //No sync at all, so also clear HalfSync bit.
                currentStatus.syncLossCounter++;
            }
            //This is to handle a special case on startup where sync can be obtained and the system immediately thinks the revs have jumped:
            //else if (currentStatus.hasSync == false && toothCurrentCount < checkSyncToothCount ) { triggerFilterTime = 0; }
            else
            {
                if(currentStatus.hasSync == true)
                {
                  currentStatus.startRevolutions++; //Counter
                  if ( configPage4.TrigSpeed == CAM_SPEED ) { currentStatus.startRevolutions++; } //Add an extra revolution count if we're running at cam speed
                }
                else { currentStatus.startRevolutions = 0; }
                
                toothCurrentCount = 1;
                if (configPage4.trigPatternSec == SEC_TRIGGER_POLL) // at tooth one check if the cam sensor is high or low in poll level mode
                {
                  if (configPage4.PollLevelPolarity == READ_SEC_TRIGGER()) { revolutionOne = 1; }
                  else { revolutionOne = 0; }
                }
                else {revolutionOne = !revolutionOne;} //Flip sequential revolution tracker if poll level is not used
                toothOneMinusOneTime = toothOneTime;
                toothOneTime = curTime;

                //if Sequential fuel or ignition is in use, further checks are needed before determining sync
                if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) || (configPage2.injLayout == INJ_SEQUENTIAL) )
                {
                  //If either fuel or ignition is sequential, only declare sync if the cam tooth has been seen OR if the missing wheel is on the cam
                  if( (secondaryToothCount > 0) || (configPage4.TrigSpeed == CAM_SPEED) || (configPage4.trigPatternSec == SEC_TRIGGER_POLL) )
                  {
                    currentStatus.hasSync = true;
                    BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); //the engine is fully synced so clear the Half Sync bit
                    if(configPage4.trigPatternSec == SEC_TRIGGER_SINGLE) { secondaryToothCount = 0; } //Reset the secondary tooth counter to prevent it overflowing
                  }
                  else if(currentStatus.hasSync != true) { BIT_SET(currentStatus.status3, BIT_STATUS3_HALFSYNC); } //If there is primary trigger but no secondary we only have half sync.
                }
                else { currentStatus.hasSync = true;  BIT_CLEAR(currentStatus.status3, BIT_STATUS3_HALFSYNC); } //If nothing is using sequential, we have sync and also clear half sync bit

                triggerFilterTime = 0; //This is used to prevent a condition where serious intermitent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
                toothLastMinusOneToothTime = toothLastToothTime;
                toothLastToothTime = curTime;
                triggerToothAngleIsCorrect = false; //The tooth angle is double at this point
            }
          }
        }
        
        if(isMissingTooth == false)
        {
          //Regular (non-missing) tooth
          setFilter(curGap);
          toothLastMinusOneToothTime = toothLastToothTime;
          toothLastToothTime = curTime;
          triggerToothAngleIsCorrect = true;
        }
      }
      else
      {
        //We fall here on initial startup when enough teeth have not yet been seen
        toothLastMinusOneToothTime = toothLastToothTime;
        toothLastToothTime = curTime;
      }
     

      //NEW IGNITION MODE
      if( (configPage2.perToothIgn == true) && (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) ) 
      {
        bigAngle_t crankAngle = ( (toothCurrentCount-1) * triggerToothAngle ) + scaleCrankAngle(configPage4.triggerAngle);
        crankAngle = ignitionLimits(crankAngle);
        if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (revolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) )
        {
          crankAngle += scaleCrankAngle(360);
          checkPerToothTiming(crankAngle, (configPage4.triggerTeeth + toothCurrentCount)); 
        }
        else{ checkPerToothTiming(crankAngle, toothCurrentCount); }
      }
   }
}

void triggerSec_missingTooth()
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;

  //Safety check for initial startup
  if( (toothLastSecToothTime == 0) )
  { 
    curGap2 = 0; 
    toothLastSecToothTime = curTime2;
  }

  if ( curGap2 >= triggerSecFilterTime )
  {
    if ( configPage4.trigPatternSec == SEC_TRIGGER_4_1 )
    {
      targetGap2 = (3 * (toothLastSecToothTime - toothLastMinusOneSecToothTime)) >> 1; //If the time between the current tooth and the last is greater than 1.5x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after the gap
      toothLastMinusOneSecToothTime = toothLastSecToothTime;
      if ( (curGap2 >= targetGap2) || (secondaryToothCount > 3) )
      {
        secondaryToothCount = 1;
        revolutionOne = 1; //Sequential revolution reset
        triggerSecFilterTime = 0; //This is used to prevent a condition where serious intermitent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
      }
      else
      {
        triggerSecFilterTime = curGap2 >> 2; //Set filter at 25% of the current speed. Filter can only be recalc'd for the regular teeth, not the missing one.
        secondaryToothCount++;
      }
    }
    else if ( configPage4.trigPatternSec == SEC_TRIGGER_SINGLE )
    {
      //Standard single tooth cam trigger
      revolutionOne = 1; //Sequential revolution reset
      triggerSecFilterTime = curGap2 >> 1; //Next secondary filter is half the current gap
      secondaryToothCount++;
    }
    toothLastSecToothTime = curTime2;
  } //Trigger filter

  //Record the VVT Angle
  if( (configPage6.vvtEnabled > 0) && (revolutionOne == 1) )
  {
    bigAngle_t curAngle;
    curAngle = getCrankAngle();
    while(curAngle > scaleCrankAngle(360)) { curAngle -= scaleCrankAngle(360); }
    curAngle -= scaleCrankAngle(configPage4.triggerAngle); //Value at TDC
    if( configPage6.vvtMode == VVT_MODE_CLOSED_LOOP ) { curAngle -= scaleCrankAngle(configPage10.vvtCL0DutyAng); }

    currentStatus.vvt1Angle = ANGLE_FILTER( (curAngle << 1), configPage4.ANGLEFILTER_VVT, currentStatus.vvt1Angle);
    /* This is only for testing spark angles, connect ignition 1 to VVT1 input
    bigAngle_t curAngle;
    curAngle = scaleCrankAngle(360) - getCrankAngle(); //Value at TDC
    if( configPage6.vvtMode == VVT_MODE_CLOSED_LOOP ) { curAngle -= scaleCrankAngle(configPage10.vvtCL0DutyAng); }
    while(curAngle > scaleCrankAngle(360)) { curAngle -= scaleCrankAngle(360); }

    currentStatus.vvt1Angle = ANGLE_FILTER(curAngle, configPage4.ANGLEFILTER_VVT, currentStatus.vvt1Angle);
    //Apply this to speeduino.ini
    //vvt1AngleGauge    = vvt1Angle,     "VVT Angle",          "deg",   -20,  100,      0,    -5,    70,  90, 3, 3
    //vvt1Angle        = scalar,   S16,    93, "deg",    0.125, 0.000
    */
  }
}

void triggerThird_missingTooth()
{
  //Record the VVT2 Angle (the only purpose of the third trigger)
  bigAngle_t curAngle;
  curAngle = getCrankAngle();
  while(curAngle > scaleCrankAngle(360)) { curAngle -= scaleCrankAngle(360); }
  curAngle -= scaleCrankAngle(configPage4.triggerAngle); //Value at TDC
  if( configPage6.vvtMode == VVT_MODE_CLOSED_LOOP ) { curAngle -= scaleCrankAngle(configPage4.vvt2CL0DutyAng); }
  //currentStatus.vvt2Angle = int8_t (curAngle); //vvt1Angle is only int8, but +/-127 degrees is enough for VVT control
  currentStatus.vvt2Angle = ANGLE_FILTER( (curAngle << 1), configPage4.ANGLEFILTER_VVT, currentStatus.vvt2Angle);
}

uint16_t getRPM_missingTooth()
{
  uint16_t tempRPM = 0;
  if( currentStatus.RPM < currentStatus.crankRPM )
  {
    if(toothCurrentCount != 1)
    {
      if(configPage4.TrigSpeed == CAM_SPEED) { tempRPM = crankingGetRPM(configPage4.triggerTeeth, scaleCrankAngle(720)); } //Account for cam speed
      else { tempRPM = crankingGetRPM(configPage4.triggerTeeth, scaleCrankAngle(360)); }
    }
    else { tempRPM = currentStatus.RPM; } //Can't do per tooth RPM if we're at tooth #1 as the missing tooth messes the calculation
  }
  else
  {
    if(configPage4.TrigSpeed == CAM_SPEED) { tempRPM = stdGetRPM(scaleCrankAngle(720)); } //Account for cam speed
    else { tempRPM = stdGetRPM(scaleCrankAngle(360)); }
  }
  return tempRPM;
}

bigAngle_t getCrankAngle_missingTooth()
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    bool tempRevolutionOne;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempRevolutionOne = revolutionOne;
    tempToothLastToothTime = toothLastToothTime;
    interrupts();

    bigAngle_t crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + scaleCrankAngle(configPage4.triggerAngle); //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    
    //Sequential check (simply sets whether we're on the first or 2nd revoltuion of the cycle)
    if ( (tempRevolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) ) { crankAngle += scaleCrankAngle(360); }

    lastCrankAngleCalc = micros();
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

    if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
    else if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}

void triggerSetEndTeeth_missingTooth()
{
  byte toothAdder = 0;
  if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (configPage4.TrigSpeed == CRANK_SPEED) ) { toothAdder = configPage4.triggerTeeth; }

  //Temp variables are used here to avoid potential issues if a trigger interrupt occurs part way through this function

  int16_t tempIgnition1EndTooth;
  tempIgnition1EndTooth = ( (ignition1EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition1EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition1EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition1EndTooth <= 0) { tempIgnition1EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition1EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition1EndTooth = (triggerActualTeeth + toothAdder); }
  ignition1EndTooth = tempIgnition1EndTooth;

  int16_t tempIgnition2EndTooth;
  tempIgnition2EndTooth = ( (ignition2EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition2EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition2EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition2EndTooth <= 0) { tempIgnition2EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition2EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition2EndTooth = (triggerActualTeeth + toothAdder); }
  ignition2EndTooth = tempIgnition2EndTooth;

  int16_t tempIgnition3EndTooth;
  tempIgnition3EndTooth = ( (ignition3EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition3EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition3EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition3EndTooth <= 0) { tempIgnition3EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition3EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition3EndTooth = (triggerActualTeeth + toothAdder); }
  ignition3EndTooth = tempIgnition3EndTooth;

  int16_t tempIgnition4EndTooth;
  tempIgnition4EndTooth = ( (ignition4EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition4EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition4EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition4EndTooth <= 0) { tempIgnition4EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition4EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition4EndTooth = (triggerActualTeeth + toothAdder); }
  ignition4EndTooth = tempIgnition4EndTooth;

#if IGN_CHANNELS >= 5
  int16_t tempIgnition5EndTooth;
  tempIgnition5EndTooth = ( (ignition5EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition5EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition5EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition5EndTooth <= 0) { tempIgnition5EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition5EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition5EndTooth = (triggerActualTeeth + toothAdder); }
  ignition5EndTooth = tempIgnition5EndTooth;
#endif
#if IGN_CHANNELS >= 6
  int16_t tempIgnition6EndTooth;
  tempIgnition6EndTooth = ( (ignition6EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition6EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition6EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition6EndTooth <= 0) { tempIgnition6EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition6EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition6EndTooth = (triggerActualTeeth + toothAdder); }
  ignition6EndTooth = tempIgnition6EndTooth;
#endif
#if IGN_CHANNELS >= 7
  int16_t tempIgnition7EndTooth;
  tempIgnition7EndTooth = ( (ignition7EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition7EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition7EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition7EndTooth <= 0) { tempIgnition7EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition7EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition7EndTooth = (triggerActualTeeth + toothAdder); }
  ignition7EndTooth = tempIgnition7EndTooth;
#endif
#if IGN_CHANNELS >= 8
  int16_t tempIgnition8EndTooth;
  tempIgnition8EndTooth = ( (ignition8EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition8EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition8EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition8EndTooth <= 0) { tempIgnition8EndTooth += (configPage4.triggerTeeth + toothAdder); }
  if((uint16_t)tempIgnition8EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition8EndTooth = (triggerActualTeeth + toothAdder); }
  ignition8EndTooth = tempIgnition8EndTooth;
#endif

  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

/** Dual wheels - 2 wheels located either both on the crank or with the primary on the crank and the secondary on the cam.
Note: There can be no missing teeth on the primary wheel.
* @defgroup dec_dual Dual wheels
* @{
*/
/** Dual Wheel Setup.
 * 
 * */
void triggerSetup_DualWheel()
{
  triggerToothAngle = scaleCrankAngle(360) / configPage4.triggerTeeth; //The number of degrees that passes from tooth to tooth
  if(configPage4.TrigSpeed == 1) { triggerToothAngle = scaleCrankAngle(720) / configPage4.triggerTeeth; } //Account for cam speed
  toothCurrentCount = 255; //Default value
  triggerFilterTime = (1000000 / (MAX_RPM / 60 * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be disgarded as noise
  triggerSecFilterTime = (1000000 / (MAX_RPM / 60 * 2)) / 2; //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)
  secondDerivEnabled = false;
  decoderIsSequential = true;
  triggerToothAngleIsCorrect = true; //This is always true for this pattern
  MAX_STALL_TIME = (3333UL * scaleCrankAngleDown(triggerToothAngle)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}

/** Dual Wheel Primary.
 * 
 * */
void triggerPri_DualWheel()
{
    curTime = micros();
    curGap = curTime - toothLastToothTime;
    if ( curGap >= triggerFilterTime )
    {
      toothCurrentCount++; //Increment the tooth counter
      validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

      toothLastMinusOneToothTime = toothLastToothTime;
      toothLastToothTime = curTime;

      if ( currentStatus.hasSync == true )
      {
        if ( (toothCurrentCount == 1) || (toothCurrentCount > configPage4.triggerTeeth) )
        {
          toothCurrentCount = 1;
          revolutionOne = !revolutionOne; //Flip sequential revolution tracker
          toothOneMinusOneTime = toothOneTime;
          toothOneTime = curTime;
          currentStatus.startRevolutions++; //Counter
        }

        setFilter(curGap); //Recalc the new filter value
      }

      //NEW IGNITION MODE
      if( (configPage2.perToothIgn == true) && (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) ) 
      {
        bigAngle_t crankAngle = ( (toothCurrentCount-1) * triggerToothAngle ) + scaleCrankAngle(configPage4.triggerAngle);
        if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (revolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) )
        {
          crankAngle += scaleCrankAngle(360);
          checkPerToothTiming(crankAngle, (configPage4.triggerTeeth + toothCurrentCount)); 
        }
        else{ checkPerToothTiming(crankAngle, toothCurrentCount); }
      }
   } //Trigger filter
}
/** Dual Wheel Secondary.
 * 
 * */
void triggerSec_DualWheel()
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;
  if ( curGap2 >= triggerSecFilterTime )
  {
    toothLastSecToothTime = curTime2;
    triggerSecFilterTime = curGap2 >> 2; //Set filter at 25% of the current speed

    if(currentStatus.hasSync == false)
    {
      toothLastToothTime = micros();
      toothLastMinusOneToothTime = micros() - (6000000 / configPage4.triggerTeeth); //Fixes RPM at 10rpm until a full revolution has taken place
      toothCurrentCount = configPage4.triggerTeeth;

      currentStatus.hasSync = true;
    }
    else 
    {
      if ( (toothCurrentCount != configPage4.triggerTeeth) && (currentStatus.startRevolutions > 2)) { currentStatus.syncLossCounter++; } //Indicates likely sync loss.
      if (configPage4.useResync == 1) { toothCurrentCount = configPage4.triggerTeeth; }
    }

    revolutionOne = 1; //Sequential revolution reset
  }
  else 
  {
    triggerSecFilterTime = revolutionTime >> 1; //Set filter at 25% of the current cam speed. This needs to be performed here to prevent a situation where the RPM and triggerSecFilterTime get out of alignment and curGap2 never exceeds the filter value
  } //Trigger filter
}
/** Dual Wheel - Get RPM.
 * 
 * */
uint16_t getRPM_DualWheel()
{
  uint16_t tempRPM = 0;
  if( currentStatus.hasSync == true )
  {
    if(currentStatus.RPM < currentStatus.crankRPM) { tempRPM = crankingGetRPM(configPage4.triggerTeeth, scaleCrankAngle(360)); }
    else { tempRPM = stdGetRPM(scaleCrankAngle(360)); }
  }
  return tempRPM;
}
/** Dual Wheel - Get Crank angle.
 * 
 * */
bigAngle_t getCrankAngle_DualWheel()
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    bool tempRevolutionOne;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempToothLastToothTime = toothLastToothTime;
    tempRevolutionOne = revolutionOne;
    lastCrankAngleCalc = micros();
    interrupts();

    //Handle case where the secondary tooth was the last one seen
    if(tempToothCurrentCount == 0) { tempToothCurrentCount = configPage4.triggerTeeth; }

    bigAngle_t crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + scaleCrankAngle(configPage4.triggerAngle); //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.

    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

    //Sequential check (simply sets whether we're on the first or 2nd revoltuion of the cycle)
    if (tempRevolutionOne) { crankAngle += scaleCrankAngle(360); }

    if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
    if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}
/** Dual Wheel - Set End Teeth.
 * 
 * */
void triggerSetEndTeeth_DualWheel()
{
  //The toothAdder variable is used for when a setup is running sequentially, but the primary wheel is running at crank speed. This way the count of teeth will go up to 2* the number of primary teeth to allow for a sequential count. 
  byte toothAdder = 0;
  if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (configPage4.TrigSpeed == CRANK_SPEED) ) { toothAdder = configPage4.triggerTeeth; }

  int16_t tempIgnition1EndTooth;
  tempIgnition1EndTooth = ( (ignition1EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) );
  if(tempIgnition1EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition1EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition1EndTooth <= 0) { tempIgnition1EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition1EndTooth = tempIgnition1EndTooth;

  int16_t tempIgnition2EndTooth;
  tempIgnition2EndTooth = ( (ignition2EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) );
  if(tempIgnition2EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition2EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition2EndTooth <= 0) { tempIgnition2EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition2EndTooth = tempIgnition2EndTooth;

  int16_t tempIgnition3EndTooth;
  tempIgnition3EndTooth = ( (ignition3EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) );
  if(tempIgnition3EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition3EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition3EndTooth <= 0) { tempIgnition3EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition3EndTooth = tempIgnition3EndTooth;

  int16_t tempIgnition4EndTooth;
  tempIgnition4EndTooth = ( (ignition4EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) );
  if(tempIgnition4EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition4EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition4EndTooth <= 0) { tempIgnition4EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition4EndTooth = tempIgnition4EndTooth;

#if IGN_CHANNELS >= 5
  int16_t tempIgnition5EndTooth;
  tempIgnition5EndTooth = ( (ignition5EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) );
  if(tempIgnition5EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition5EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition5EndTooth <= 0) { tempIgnition5EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition5EndTooth = tempIgnition5EndTooth;
#endif
#if IGN_CHANNELS >= 6
  int16_t tempIgnition6EndTooth;
  tempIgnition6EndTooth = ( (ignition6EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) );
  if(tempIgnition6EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition6EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition6EndTooth <= 0) { tempIgnition6EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition6EndTooth = tempIgnition6EndTooth;
#endif
#if IGN_CHANNELS >= 7
  int16_t tempIgnition7EndTooth;
  tempIgnition7EndTooth = ( (ignition7EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) );
  if(tempIgnition7EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition7EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition7EndTooth <= 0) { tempIgnition7EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition7EndTooth = tempIgnition7EndTooth;
#endif
#if IGN_CHANNELS >= 8
  int16_t tempIgnition8EndTooth;
  tempIgnition8EndTooth = ( (ignition8EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) );
  if(tempIgnition8EndTooth > (configPage4.triggerTeeth + toothAdder)) { tempIgnition8EndTooth -= (configPage4.triggerTeeth + toothAdder); }
  if(tempIgnition8EndTooth <= 0) { tempIgnition8EndTooth += (configPage4.triggerTeeth + toothAdder); }
  ignition8EndTooth = tempIgnition8EndTooth;
#endif

  lastToothCalcAdvance = currentStatus.advance;

}
/** @} */

/** Basic Distributor where tooth count is equal to the number of cylinders and teeth are evenly spaced on the cam.
* No position sensing (Distributor is retained) so crank angle is
* a made up figure based purely on the first teeth to be seen.
* Note: This is a very simple decoder. See http://www.megamanual.com/ms2/GM_7pinHEI.htm
* @defgroup dec_dist Basic Distributor
* @{
*/
void triggerSetup_BasicDistributor()
{
  triggerActualTeeth = configPage2.nCylinders;
  if(triggerActualTeeth == 0) { triggerActualTeeth = 1; }
  triggerToothAngle = scaleCrankAngle(720) / triggerActualTeeth; //The number of degrees that passes from tooth to tooth
  triggerFilterTime = 60000000L / MAX_RPM / configPage2.nCylinders; // Minimum time required between teeth
  triggerFilterTime = triggerFilterTime / 2; //Safety margin
  triggerFilterTime = 0;
  secondDerivEnabled = false;
  decoderIsSequential = false;
  toothCurrentCount = 0; //Default value
  decoderHasFixedCrankingTiming = true;
  triggerToothAngleIsCorrect = true;
  if(configPage2.nCylinders <= 4) { MAX_STALL_TIME = (1851UL * scaleCrankAngleDown(triggerToothAngle)); }//Minimum 90rpm. (1851uS is the time per degree at 90rpm). This uses 90rpm rather than 50rpm due to the potentially very high stall time on a 4 cylinder if we wait that long.
  else { MAX_STALL_TIME = (3200UL * scaleCrankAngleDown(triggerToothAngle)); } //Minimum 50rpm. (3200uS is the time per degree at 50rpm).

}

void triggerPri_BasicDistributor()
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  if ( (curGap >= triggerFilterTime) )
  {
    if(currentStatus.hasSync == true) { setFilter(curGap); } //Recalc the new filter value
    else { triggerFilterTime = 0; } //If we don't yet have sync, ensure that the filter won't prevent future valid pulses from being ignored. 
    
    if( (toothCurrentCount == triggerActualTeeth) || (currentStatus.hasSync == false) ) //Check if we're back to the beginning of a revolution
    {
      toothCurrentCount = 1; //Reset the counter
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.hasSync = true;
      currentStatus.startRevolutions++; //Counter
    }
    else
    {
      if( (toothCurrentCount < triggerActualTeeth) ) { toothCurrentCount++; } //Increment the tooth counter
      else
      {
        //This means toothCurrentCount is greater than triggerActualTeeth, which is bad.
        //If we have sync here then there's a problem. Throw a sync loss
        if( currentStatus.hasSync == true ) 
        { 
          currentStatus.syncLossCounter++;
          currentStatus.hasSync = false;
        }
      }
      
    }

    validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

    if ( configPage4.ignCranklock && BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
    {
      endCoil1Charge();
      endCoil2Charge();
      endCoil3Charge();
      endCoil4Charge();
    }

    if(configPage2.perToothIgn == true)
    {
      bigAngle_t crankAngle = ( (toothCurrentCount-1) * triggerToothAngle ) + scaleCrankAngle(configPage4.triggerAngle);
      crankAngle = ignitionLimits((crankAngle));
      if(toothCurrentCount > (triggerActualTeeth/2) ) { checkPerToothTiming(crankAngle, (toothCurrentCount - (triggerActualTeeth/2))); }
      else { checkPerToothTiming(crankAngle, toothCurrentCount); }
    }

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;
  } //Trigger filter
}
void triggerSec_BasicDistributor() { return; } //Not required
uint16_t getRPM_BasicDistributor()
{
  uint16_t tempRPM;
  if( currentStatus.RPM < currentStatus.crankRPM)
  { 
    tempRPM = crankingGetRPM(triggerActualTeeth, scaleCrankAngle(720));
  } 
  else { tempRPM = stdGetRPM(scaleCrankAngle(720)); }

  MAX_STALL_TIME = revolutionTime << 1; //Set the stall time to be twice the current RPM. This is a safe figure as there should be no single revolution where this changes more than this
  if(triggerActualTeeth == 1) { MAX_STALL_TIME = revolutionTime << 1; } //Special case for 1 cylinder engines that only get 1 pulse every scaleCrankAngle(720) degrees
  if(MAX_STALL_TIME < 366667UL) { MAX_STALL_TIME = 366667UL; } //Check for 50rpm minimum

  return tempRPM;

}
bigAngle_t getCrankAngle_BasicDistributor()
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempToothLastToothTime = toothLastToothTime;
    lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    interrupts();

    bigAngle_t crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + scaleCrankAngle(configPage4.triggerAngle); //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    
    //Estimate the number of degrees travelled since the last tooth}
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);

    //crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_TOOTH);
    

    if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
    if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}

void triggerSetEndTeeth_BasicDistributor()
{

  int tempEndAngle = (ignition1EndAngle - scaleCrankAngle(configPage4.triggerAngle));
  tempEndAngle = ignitionLimits((tempEndAngle));

  
  if( (tempEndAngle > 180) || (tempEndAngle <= 0) )
  {
    ignition1EndTooth = 2;
    ignition2EndTooth = 1;
  }
  else
  {
    ignition1EndTooth = 1;
    ignition2EndTooth = 2;
  }


  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

/** Decode GM 7X trigger wheel with six equally spaced teeth and a seventh tooth for cylinder identification.
* Note: Within the decoder code pf GM7X, the sync tooth is referred to as tooth #3 rather than tooth #7. This makes for simpler angle calculations
* (See: http://www.speeduino.com/forum/download/file.php?id=4743 ).
* @defgroup dec_gm7x GM7X
* @{
*/
void triggerSetup_GM7X()
{
  triggerToothAngle = scaleCrankAngle(360) / 6; //The number of degrees that passes from tooth to tooth
  secondDerivEnabled = false;
  decoderIsSequential = false;
  MAX_STALL_TIME = (3333UL * scaleCrankAngleDown(triggerToothAngle)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}

void triggerPri_GM7X()
{
    lastGap = curGap;
    curTime = micros();
    curGap = curTime - toothLastToothTime;
    toothCurrentCount++; //Increment the tooth counter
    validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

    if( (toothLastToothTime > 0) && (toothLastMinusOneToothTime > 0) )
    {
      if( toothCurrentCount > 7 )
      {
        toothCurrentCount = 1;
        toothOneMinusOneTime = toothOneTime;
        toothOneTime = curTime;

        triggerToothAngleIsCorrect = true;
      }
      else
      {
        targetGap = (lastGap) >> 1; //The target gap is set at half the last tooth gap
        if ( curGap < targetGap ) //If the gap between this tooth and the last one is less than half of the previous gap, then we are very likely at the magical 3rd tooth
        {
          toothCurrentCount = 3;
          currentStatus.hasSync = true;
          triggerToothAngleIsCorrect = false;
          currentStatus.startRevolutions++; //Counter
        }
        else
        {
          triggerToothAngleIsCorrect = true;
        }
      }
    }

    //New ignition mode!
    if(configPage2.perToothIgn == true)
    {
      if(toothCurrentCount != 3) //Never do the check on the extra tooth. It's not needed anyway
      {
        //configPage4.triggerAngle must currently be below 48 and above -81
        bigAngle_t crankAngle;
        if( toothCurrentCount < 3 )
        {
          crankAngle = ((toothCurrentCount - 1) * triggerToothAngle) + scaleCrankAngle(42 + configPage4.triggerAngle); //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
        }
        else
        {
          crankAngle = ((toothCurrentCount - 2) * triggerToothAngle) + scaleCrankAngle(42 + configPage4.triggerAngle); //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
        }
        checkPerToothTiming(crankAngle, toothCurrentCount);
      } 
    }

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;


}
void triggerSec_GM7X() { return; } //Not required
uint16_t getRPM_GM7X()
{
   return stdGetRPM(scaleCrankAngle(360));
}
bigAngle_t getCrankAngle_GM7X()
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempToothLastToothTime = toothLastToothTime;
    lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    interrupts();

    //Check if the last tooth seen was the reference tooth (Number 3). All others can be calculated, but tooth 3 has a unique angle
    bigAngle_t crankAngle;
    if( tempToothCurrentCount < 3 )
    {
      crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + scaleCrankAngle(42 + configPage4.triggerAngle); //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    }
    else if( tempToothCurrentCount == 3 )
    {
      crankAngle = scaleCrankAngle(112);
    }
    else
    {
      crankAngle = ((tempToothCurrentCount - 2) * triggerToothAngle) + scaleCrankAngle(42 + configPage4.triggerAngle); //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    }

    //Estimate the number of degrees travelled since the last tooth}
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

    if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
    if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += scaleCrankAngle(360); }

    return crankAngle;
}

void triggerSetEndTeeth_GM7X()
{

  lastToothCalcAdvance = currentStatus.advance;

  if(currentStatus.advance < scaleCrankAngle(18) ) 
  { 
    ignition1EndTooth = 7;
    ignition2EndTooth = 2;
    ignition3EndTooth = 5;
  }
  else 
  { 
    ignition1EndTooth = 6;
    ignition2EndTooth = 1;
    ignition3EndTooth = 4;
  }
}
/** @} */

/** Mitsubishi 4G63 / NA/NB Miata + MX-5 / 4/2.
Note: raw.githubusercontent.com/noisymime/speeduino/master/reference/wiki/decoders/4g63_trace.png
Tooth #1 is defined as the next crank tooth after the crank signal is HIGH when the cam signal is falling.
Tooth number one is at 355* ATDC.
* @defgroup dec_mitsu_miata Mistsubishi 4G63 and Miata + MX-5
* @{
*/
void triggerSetup_4G63()
{
  triggerToothAngle = scaleCrankAngle(180); //The number of degrees that passes from tooth to tooth (primary)
  toothCurrentCount = 99; //Fake tooth count represents no sync
  secondDerivEnabled = false;
  decoderIsSequential = true;
  decoderHasFixedCrankingTiming = true;
  triggerToothAngleIsCorrect = true;
  MAX_STALL_TIME = 366667UL; //Minimum 50rpm based on the 110 degree tooth spacing
  if(initialisationComplete == false) { toothLastToothTime = micros(); } //Set a startup value here to avoid filter errors when starting. This MUST have the initi check to prevent the fuel pump just staying on all the time
  //decoderIsLowRes = true;

  //Note that these angles are for every rising and falling edge
  if(configPage2.nCylinders == 6)
  {
    //New values below
    toothAngles[0] = scaleCrankAngle(715); //Rising edge of tooth #1
    toothAngles[1] = scaleCrankAngle(45);  //Falling edge of tooth #1
    toothAngles[2] = scaleCrankAngle(115); //Rising edge of tooth #2
    toothAngles[3] = scaleCrankAngle(165); //Falling edge of tooth #2
    toothAngles[4] = scaleCrankAngle(235); //Rising edge of tooth #3
    toothAngles[5] = scaleCrankAngle(285); //Falling edge of tooth #3

    toothAngles[6] = scaleCrankAngle(355); //Rising edge of tooth #4
    toothAngles[7] = scaleCrankAngle(405); //Falling edge of tooth #4
    toothAngles[8] = scaleCrankAngle(475); //Rising edge of tooth #5
    toothAngles[9] = scaleCrankAngle(525); //Falling edge of tooth $5
    toothAngles[10] = scaleCrankAngle(595); //Rising edge of tooth #6
    toothAngles[11] = scaleCrankAngle(645); //Falling edge of tooth #6

    triggerActualTeeth = 12; //Both sides of all teeth over scaleCrankAngle(720) degrees
  }
  else
  {
    // 70 / 110 for 4 cylinder
    toothAngles[0] = scaleCrankAngle(715); //Falling edge of tooth #1
    toothAngles[1] = scaleCrankAngle(105); //Rising edge of tooth #2
    toothAngles[2] = scaleCrankAngle(175); //Falling edge of tooth #2
    toothAngles[3] = scaleCrankAngle(285); //Rising edge of tooth #1

    toothAngles[4] = scaleCrankAngle(355); //Falling edge of tooth #1
    toothAngles[5] = scaleCrankAngle(465); //Rising edge of tooth #2
    toothAngles[6] = scaleCrankAngle(535); //Falling edge of tooth #2
    toothAngles[7] = scaleCrankAngle(645); //Rising edge of tooth #1

    triggerActualTeeth = 8;
  }

  triggerFilterTime = 1500; //10000 rpm, assuming we're triggering on both edges off the crank tooth.
  triggerSecFilterTime = (int)(1000000 / (MAX_RPM / 60 * 2)) / 2; //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)
  triggerSecFilterTime_duration = 4000;
  secondaryLastToothTime = 0;
}

void triggerPri_4G63()
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  if ( (curGap >= triggerFilterTime) || (currentStatus.startRevolutions == 0) )
  {
    validTrigger = true; //Flag that this pulse was accepted as a valid trigger
    triggerFilterTime = curGap >> 2; //This only applies during non-sync conditions. If there is sync then triggerFilterTime gets changed again below with a better value.

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    toothCurrentCount++;

    if( (toothCurrentCount == 1) || (toothCurrentCount > triggerActualTeeth) ) //Trigger is on CHANGE, hence 4 pulses = 1 crank rev (or 6 pulses for 6 cylinders)
    {
       toothCurrentCount = 1; //Reset the counter
       toothOneMinusOneTime = toothOneTime;
       toothOneTime = curTime;
       currentStatus.startRevolutions++; //Counter
    }

    if (currentStatus.hasSync == true)
    {
      if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && configPage4.ignCranklock && (currentStatus.startRevolutions >= configPage4.StgCycles))
      {
        if(configPage2.nCylinders == 4)
        {
          //This operates in forced wasted spark mode during cranking to align with crank teeth
          if( (toothCurrentCount == 1) || (toothCurrentCount == 5) ) { endCoil1Charge(); endCoil3Charge(); }
          else if( (toothCurrentCount == 3) || (toothCurrentCount == 7) ) { endCoil2Charge(); endCoil4Charge(); }
        }
        else if(configPage2.nCylinders == 6)
        {
          if( (toothCurrentCount == 1) || (toothCurrentCount == 7) ) { endCoil1Charge(); }
          else if( (toothCurrentCount == 3) || (toothCurrentCount == 9) ) { endCoil2Charge(); }
          else if( (toothCurrentCount == 5) || (toothCurrentCount == 11) ) { endCoil3Charge(); }
        }
      }

      //Whilst this is an uneven tooth pattern, if the specific angle between the last 2 teeth is specified, 1st deriv prediction can be used
      if( (configPage4.triggerFilter == 1) || (currentStatus.RPM < 1400) )
      {
        //Lite filter
        if( (toothCurrentCount == 1) || (toothCurrentCount == 3) || (toothCurrentCount == 5) || (toothCurrentCount == 7) || (toothCurrentCount == 9) || (toothCurrentCount == 11) )
        {
          if(configPage2.nCylinders == 4)
          {
            triggerToothAngle = scaleCrankAngle(70);
            triggerFilterTime = curGap; //Trigger filter is set to whatever time it took to do 70 degrees (Next trigger is 110 degrees away)
          }
          else if(configPage2.nCylinders == 6)
          {
            triggerToothAngle = scaleCrankAngle(70);
            triggerFilterTime = (curGap >> 2); //Trigger filter is set to (70/4)=17.5=17 degrees (Next trigger is 50 degrees away).
          }
        }
        else
        {
          if(configPage2.nCylinders == 4)
          {
            triggerToothAngle = scaleCrankAngle(110);
            triggerFilterTime = (curGap * 3) >> 3; //Trigger filter is set to (110*3)/8=41.25=41 degrees (Next trigger is 70 degrees away).
          }
          else if(configPage2.nCylinders == 6)
          {
            triggerToothAngle = scaleCrankAngle(50);
            triggerFilterTime = curGap >> 1; //Trigger filter is set to 25 degrees (Next trigger is 70 degrees away).
          }
        }
      }
      else if(configPage4.triggerFilter == 2)
      {
        //Medium filter level
        if( (toothCurrentCount == 1) || (toothCurrentCount == 3) || (toothCurrentCount == 5) || (toothCurrentCount == 7) || (toothCurrentCount == 9) || (toothCurrentCount == 11) )
        { 
          triggerToothAngle = scaleCrankAngle(70); 
          if(configPage2.nCylinders == 4)
          { 
            triggerFilterTime = (curGap * 5) >> 2 ; //87.5 degrees with a target of 110
          }
          else
          {
            triggerFilterTime = curGap >> 1 ; //35 degrees with a target of 50
          }
        } 
        else 
        { 
          if(configPage2.nCylinders == 4)
          { 
            triggerToothAngle = scaleCrankAngle(110); 
            triggerFilterTime = (curGap >> 1); //55 degrees with a target of 70
          }
          else
          {
            triggerToothAngle = scaleCrankAngle(50); 
            triggerFilterTime = (curGap * 3) >> 2; //Trigger filter is set to (50*3)/4=37.5=37 degrees (Next trigger is 70 degrees away).
          }
        } 
      }
      else if (configPage4.triggerFilter == 3)
      {
        //Aggressive filter level
        if( (toothCurrentCount == 1) || (toothCurrentCount == 3) || (toothCurrentCount == 5) || (toothCurrentCount == 7) || (toothCurrentCount == 9) || (toothCurrentCount == 11) )
        { 
          triggerToothAngle = scaleCrankAngle(70); 
          if(configPage2.nCylinders == 4)
          { 
            triggerFilterTime = (curGap * 11) >> 3;//96.26 degrees with a target of 110
          }
          else
          {
            triggerFilterTime = curGap >> 1 ; //35 degrees with a target of 50
          }
        } 
        else 
        { 
          if(configPage2.nCylinders == 4)
          { 
            triggerToothAngle = scaleCrankAngle(110); 
            triggerFilterTime = (curGap * 9) >> 5; //61.87 degrees with a target of 70
          }
          else
          {
            triggerToothAngle = scaleCrankAngle(50); 
            triggerFilterTime = curGap; //50 degrees with a target of 70
          }
        } 
      }
      else
      {
        //trigger filter is turned off.
        triggerFilterTime = 0;
        if( (toothCurrentCount == 1) || (toothCurrentCount == 3) || (toothCurrentCount == 5) || (toothCurrentCount == 7) || (toothCurrentCount == 9) || (toothCurrentCount == 11) )
        { 
          triggerToothAngle = scaleCrankAngle(70);
        } 
        else 
        { 
          if(configPage2.nCylinders == 4) { triggerToothAngle = scaleCrankAngle(110); }
          else  { triggerToothAngle = scaleCrankAngle(50); }
        }
      }

      //EXPERIMENTAL!
      //New ignition mode is ONLY available on 4g63 when the trigger angle is set to the stock value of 0.
      if( (configPage2.perToothIgn == true) && (configPage4.triggerAngle == 0) )
      {
        if( (configPage2.nCylinders == 4) && (currentStatus.advance > (0)) )
        {
          bigAngle_t crankAngle = ignitionLimits( toothAngles[(toothCurrentCount-1)] );

          //Handle non-sequential tooth counts 
          if( (configPage4.sparkMode != IGN_MODE_SEQUENTIAL) && (toothCurrentCount > configPage2.nCylinders) ) { checkPerToothTiming(crankAngle, (toothCurrentCount-configPage2.nCylinders) ); }
          else { checkPerToothTiming(crankAngle, toothCurrentCount); }
        }
      }
    } //Has sync
    else
    {
      triggerSecFilterTime = 0;
      //New secondary method of determining sync
      if(READ_PRI_TRIGGER() == true)
      {
        if(READ_SEC_TRIGGER() == true) { revolutionOne = true; }
        else { revolutionOne = false; }
      }
      else
      {
        if( (READ_SEC_TRIGGER() == false) && (revolutionOne == true) ) 
        { 
          //Crank is low, cam is low and the crank pulse STARTED when the cam was high. 
          if(configPage2.nCylinders == 4) { toothCurrentCount = 1; } //Means we're at 5* BTDC on a 4G63 4 cylinder
          //else if(configPage2.nCylinders == 6) { toothCurrentCount = 8; } 
        } 
        //If sequential is ever enabled, the below toothCurrentCount will need to change:
        else if( (READ_SEC_TRIGGER() == true) && (revolutionOne == true) ) 
        { 
          //Crank is low, cam is high and the crank pulse STARTED when the cam was high. 
          if(configPage2.nCylinders == 4) { toothCurrentCount = 5; } //Means we're at 5* BTDC on a 4G63 4 cylinder
          else if(configPage2.nCylinders == 6) { toothCurrentCount = 2; currentStatus.hasSync = true; } //Means we're at 45* ATDC on 6G72 6 cylinder
        } 
      }
    }
  } //Filter time

}
void triggerSec_4G63()
{
  //byte crankState = READ_PRI_TRIGGER();
  //First filter is a duration based one to ensure the pulse was of sufficient length (time)
  //if(READ_SEC_TRIGGER()) { secondaryLastToothTime1 = micros(); return; }
  if(currentStatus.hasSync == true)
  {
  //1166 is the time taken to cross 70 degrees at 10k rpm
  //if ( (micros() - secondaryLastToothTime1) < triggerSecFilterTime_duration ) { return; }
  //triggerSecFilterTime_duration = (micros() - secondaryLastToothTime1) >> 1;
  }


  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;
  if ( (curGap2 >= triggerSecFilterTime) )//|| (currentStatus.startRevolutions == 0) )
  {
    toothLastSecToothTime = curTime2;
    validTrigger = true; //Flag that this pulse was accepted as a valid trigger
    //addToothLogEntry(curGap, TOOTH_CAM);

    triggerSecFilterTime = curGap2 >> 1; //Basic 50% filter for the secondary reading
    //More aggressive options:
    //62.5%:
    //triggerSecFilterTime = (curGap2 * 9) >> 5;
    //75%:
    //triggerSecFilterTime = (curGap2 * 6) >> 3;

    //if( (currentStatus.RPM < currentStatus.crankRPM) || (currentStatus.hasSync == false) )
    if( (currentStatus.hasSync == false) )
    {

      triggerFilterTime = 1500; //If this is removed, can have trouble getting sync again after the engine is turned off (but ECU not reset).
      triggerSecFilterTime = triggerSecFilterTime >> 1; //Divide the secondary filter time by 2 again, making it 25%. Only needed when cranking
      if(READ_PRI_TRIGGER() == true)
      {
        if(configPage2.nCylinders == 4)
        { 
          if(toothCurrentCount == 8) { currentStatus.hasSync = true; } //Is 8 for sequential, was 4
        }
        else if(configPage2.nCylinders == 6) 
        { 
          if(toothCurrentCount == 7) { currentStatus.hasSync = true; }
        }

      }
      else
      {
        if(configPage2.nCylinders == 4)
        { 
          if(toothCurrentCount == 5) { currentStatus.hasSync = true; } //Is 5 for sequential, was 1
        }
        //Cannot gain sync for 6 cylinder here. 
      }
    }

    //if ( (micros() - secondaryLastToothTime1) < triggerSecFilterTime_duration && configPage2.useResync )
    if ( (currentStatus.RPM < currentStatus.crankRPM) || (configPage4.useResync == 1) )
    {
      if( (currentStatus.hasSync == true) && (configPage2.nCylinders == 4) )
      {
        triggerSecFilterTime_duration = (micros() - secondaryLastToothTime1) >> 1;
        if(READ_PRI_TRIGGER() == true)
        {
          //Whilst we're cranking and have sync, we need to watch for noise pulses.
          if(toothCurrentCount != 8) 
          { 
            // This should never be true, except when there's noise
            currentStatus.hasSync = false; 
            currentStatus.syncLossCounter++;
          } 
          else { toothCurrentCount = 8; } //Why? Just why?
        }
      } //Has sync and 4 cylinder 
    } // Use resync or cranking
  } //Trigger filter
}


uint16_t getRPM_4G63()
{
  uint16_t tempRPM = 0;
  //During cranking, RPM is calculated 4 times per revolution, once for each rising/falling of the crank signal.
  //Because these signals aren't even (Alternating 110 and 70 degrees), this needs a special function
  if(currentStatus.hasSync == true)
  {
    if( (currentStatus.RPM < currentStatus.crankRPM)  )
    {
      int tempToothAngle;
      unsigned long toothTime;
      if( (toothLastToothTime == 0) || (toothLastMinusOneToothTime == 0) ) { tempRPM = 0; }
      else
      {
        noInterrupts();
        tempToothAngle = triggerToothAngle;
        toothTime = (toothLastToothTime - toothLastMinusOneToothTime); //Note that trigger tooth angle changes between 70 and 110 depending on the last tooth that was seen (or 70/50 for 6 cylinders)
        interrupts();
        toothTime = toothTime * 36;
        tempRPM = ((unsigned long)tempToothAngle * 6000000UL) / toothTime;
        revolutionTime = (10UL * toothTime) / tempToothAngle;
        MAX_STALL_TIME = 366667UL; // 50RPM
      }
    }
    else
    {
      tempRPM = stdGetRPM(scaleCrankAngle(720));
      //EXPERIMENTAL! Add/subtract RPM based on the last rpmDOT calc
      //tempRPM += (micros() - toothOneTime) * currentStatus.rpmDOT
      MAX_STALL_TIME = revolutionTime << 1; //Set the stall time to be twice the current RPM. This is a safe figure as there should be no single revolution where this changes more than this
      if(MAX_STALL_TIME < 366667UL) { MAX_STALL_TIME = 366667UL; } //Check for 50rpm minimum
    }
  }

  return tempRPM;
}

bigAngle_t getCrankAngle_4G63()
{
    bigAngle_t crankAngle = 0;
    if(currentStatus.hasSync == true)
    {
      //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
      unsigned long tempToothLastToothTime;
      int tempToothCurrentCount;
      //Grab some variables that are used in the trigger code and assign them to temp variables.
      noInterrupts();
      tempToothCurrentCount = toothCurrentCount;
      tempToothLastToothTime = toothLastToothTime;
      lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
      interrupts();

      crankAngle = toothAngles[(tempToothCurrentCount - 1)] + scaleCrankAngle(configPage4.triggerAngle); //Perform a lookup of the fixed toothAngles array to find what the angle of the last tooth passed was.

      //Estimate the number of degrees travelled since the last tooth}
      elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
      crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_TOOTH);

      if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
      if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
      if (crankAngle < 0) { crankAngle += scaleCrankAngle(360); }
    }
    return crankAngle;
}

void triggerSetEndTeeth_4G63()
{
  if(configPage2.nCylinders == 4)
  {
    if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL) 
    { 
      ignition1EndTooth = 8;
      ignition2EndTooth = 2;
      ignition3EndTooth = 4;
      ignition4EndTooth = 6;
    }
    else
    {
      ignition1EndTooth = 4;
      ignition2EndTooth = 2;
      ignition3EndTooth = 4; //Not used
      ignition4EndTooth = 2;
    }
  }
  if(configPage2.nCylinders == 6)
  {
    if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL) 
    { 
      //This should never happen as 6 cylinder sequential not supported
      ignition1EndTooth = 8;
      ignition2EndTooth = 2;
      ignition3EndTooth = 4;
      ignition4EndTooth = 6;
    }
    else
    {
      ignition1EndTooth = 6;
      ignition2EndTooth = 2;
      ignition3EndTooth = 4;
      ignition4EndTooth = 2; //Not used
    }
  }


  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

/** GM 24X Decoder (eg early LS1 1996-2005).
Note: Useful references:
* 
- www.vems.hu/wiki/index.php?page=MembersPage%2FJorgenKarlsson%2FTwentyFourX

Provided that the cam signal is used, this decoder simply counts the teeth and then looks their angles up against a lookup table. The cam signal is used to determine tooth #1
* @defgroup dec_gm GM 24X
* @{
*/
void triggerSetup_24X()
{
  triggerToothAngle = scaleCrankAngle(15); //The number of degrees that passes from tooth to tooth (primary)
  toothAngles[0] = scaleCrankAngle(12);
  toothAngles[1] = scaleCrankAngle(18);
  toothAngles[2] = scaleCrankAngle(33);
  toothAngles[3] = scaleCrankAngle(48);
  toothAngles[4] = scaleCrankAngle(63);
  toothAngles[5] = scaleCrankAngle(78);
  toothAngles[6] = scaleCrankAngle(102);
  toothAngles[7] = scaleCrankAngle(108);
  toothAngles[8] = scaleCrankAngle(123);
  toothAngles[9] = scaleCrankAngle(138);
  toothAngles[10] = scaleCrankAngle(162);
  toothAngles[11] = scaleCrankAngle(177);
  toothAngles[12] = scaleCrankAngle(183);
  toothAngles[13] = scaleCrankAngle(198);
  toothAngles[14] = scaleCrankAngle(222);
  toothAngles[15] = scaleCrankAngle(237);
  toothAngles[16] = scaleCrankAngle(252);
  toothAngles[17] = scaleCrankAngle(258);
  toothAngles[18] = scaleCrankAngle(282);
  toothAngles[19] = scaleCrankAngle(288);
  toothAngles[20] = scaleCrankAngle(312);
  toothAngles[21] = scaleCrankAngle(327);
  toothAngles[22] = scaleCrankAngle(342);
  toothAngles[23] = scaleCrankAngle(357);

  MAX_STALL_TIME = (3333UL * scaleCrankAngleDown(triggerToothAngle)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  if(initialisationComplete == false) { toothCurrentCount = 25; toothLastToothTime = micros(); } //Set a startup value here to avoid filter errors when starting. This MUST have the init check to prevent the fuel pump just staying on all the time
  secondDerivEnabled = false;
  decoderIsSequential = true;
  triggerToothAngleIsCorrect = true;
}

void triggerPri_24X()
{
  if(toothCurrentCount == 25) { currentStatus.hasSync = false; } //Indicates sync has not been achieved (Still waiting for 1 revolution of the crank to take place)
  else
  {
    curTime = micros();
    curGap = curTime - toothLastToothTime;

    if(toothCurrentCount == 0)
    {
       toothCurrentCount = 1; //Reset the counter
       toothOneMinusOneTime = toothOneTime;
       toothOneTime = curTime;
       revolutionOne = !revolutionOne; //Sequential revolution flip
       currentStatus.hasSync = true;
       currentStatus.startRevolutions++; //Counter
       triggerToothAngle = scaleCrankAngle(15); //Always 15 degrees for tooth #15
    }
    else
    {
      toothCurrentCount++; //Increment the tooth counter
      triggerToothAngle = toothAngles[(toothCurrentCount-1)] - toothAngles[(toothCurrentCount-2)]; //Calculate the last tooth gap in degrees
    }

    validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

    toothLastToothTime = curTime;


  }
}

void triggerSec_24X()
{
  toothCurrentCount = 0; //All we need to do is reset the tooth count back to zero, indicating that we're at the beginning of a new revolution
  revolutionOne = 1; //Sequential revolution reset
}

uint16_t getRPM_24X()
{
   return stdGetRPM(scaleCrankAngle(360));
}

bigAngle_t getCrankAngle_24X()
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount, tempRevolutionOne;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempToothLastToothTime = toothLastToothTime;
    tempRevolutionOne = revolutionOne;
    lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    interrupts();

    bigAngle_t crankAngle;
    if (tempToothCurrentCount == 0) { crankAngle = scaleCrankAngle(configPage4.triggerAngle); } //This is the special case to handle when the 'last tooth' seen was the cam tooth. 0 is the angle at which the crank tooth goes high (Within scaleCrankAngle(360) degrees).
    else { crankAngle = toothAngles[(tempToothCurrentCount - 1)] + scaleCrankAngle(configPage4.triggerAngle);} //Perform a lookup of the fixed toothAngles array to find what the angle of the last tooth passed was.

    //Estimate the number of degrees travelled since the last tooth}
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

    //Sequential check (simply sets whether we're on the first or 2nd revoltuion of the cycle)
    if (tempRevolutionOne == 1) { crankAngle += scaleCrankAngle(360); }

    if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
    if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += scaleCrankAngle(360); }

    return crankAngle;
}

void triggerSetEndTeeth_24X()
{


  lastToothCalcAdvance = currentStatus.advance;
}

/** @} */

/** Jeep 2000 - 24 crank teeth over 720 degrees, in groups of 4 ('91 to 2000 6 cylinder Jeep engines).
* Crank wheel is high for 360 crank degrees. Quite similar to the 24X setup.
* As we only need timing within 360 degrees, only 12 tooth angles are defined.
* Tooth number 1 represents the first tooth seen after the cam signal goes high.
* www.speeduino.com/forum/download/file.php?id=205
* @defgroup dec_jeep Jeep 2000 (6 cyl)
* @{
*/
void triggerSetup_Jeep2000()
{
  triggerToothAngle = scaleCrankAngle(0); //The number of degrees that passes from tooth to tooth (primary)
  toothAngles[0] = scaleCrankAngle(174);
  toothAngles[1] = scaleCrankAngle(194);
  toothAngles[2] = scaleCrankAngle(214);
  toothAngles[3] = scaleCrankAngle(234);
  toothAngles[4] = scaleCrankAngle(294);
  toothAngles[5] = scaleCrankAngle(314);
  toothAngles[6] = scaleCrankAngle(334);
  toothAngles[7] = scaleCrankAngle(354);
  toothAngles[8] = scaleCrankAngle(414);
  toothAngles[9] = scaleCrankAngle(434);
  toothAngles[10] = scaleCrankAngle(454);
  toothAngles[11] = scaleCrankAngle(474);

  MAX_STALL_TIME = (3333UL * 60); //Minimum 50rpm. (3333uS is the time per degree at 50rpm). Largest gap between teeth is 60 degrees.
  if(initialisationComplete == false) { toothCurrentCount = 13; toothLastToothTime = micros(); } //Set a startup value here to avoid filter errors when starting. This MUST have the initi check to prevent the fuel pump just staying on all the time
  secondDerivEnabled = false;
  decoderIsSequential = false;
  triggerToothAngleIsCorrect = true;
}

void triggerPri_Jeep2000()
{
  if(toothCurrentCount == 13) { currentStatus.hasSync = false; } //Indicates sync has not been achieved (Still waiting for 1 revolution of the crank to take place)
  else
  {
    curTime = micros();
    curGap = curTime - toothLastToothTime;
    if ( curGap >= triggerFilterTime )
    {
      if(toothCurrentCount == 0)
      {
         toothCurrentCount = 1; //Reset the counter
         toothOneMinusOneTime = toothOneTime;
         toothOneTime = curTime;
         currentStatus.hasSync = true;
         currentStatus.startRevolutions++; //Counter
         triggerToothAngle = scaleCrankAngle(60); //There are groups of 4 pulses (Each 20 degrees apart), with each group being 60 degrees apart. Hence #1 is always 60
      }
      else
      {
        toothCurrentCount++; //Increment the tooth counter
        triggerToothAngle = toothAngles[(toothCurrentCount-1)] - toothAngles[(toothCurrentCount-2)]; //Calculate the last tooth gap in degrees
      }

      setFilter(curGap); //Recalc the new filter value

      validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

      toothLastMinusOneToothTime = toothLastToothTime;
      toothLastToothTime = curTime;
    } //Trigger filter
  } //Sync check
}
void triggerSec_Jeep2000()
{
  toothCurrentCount = 0; //All we need to do is reset the tooth count back to zero, indicating that we're at the beginning of a new revolution
  return;
}

uint16_t getRPM_Jeep2000()
{
   return stdGetRPM(scaleCrankAngle(360));
}
bigAngle_t getCrankAngle_Jeep2000()
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempToothLastToothTime = toothLastToothTime;
    lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    interrupts();

    bigAngle_t crankAngle;
    if (toothCurrentCount == 0) { crankAngle = scaleCrankAngle(146 + configPage4.triggerAngle); } //This is the special case to handle when the 'last tooth' seen was the cam tooth. 146 is the angle at which the crank tooth goes high.
    else { crankAngle = toothAngles[(tempToothCurrentCount - 1)] + scaleCrankAngle(configPage4.triggerAngle);} //Perform a lookup of the fixed toothAngles array to find what the angle of the last tooth passed was.

    //Estimate the number of degrees travelled since the last tooth}
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

    if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
    if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += scaleCrankAngle(360); }

    return crankAngle;
}

void triggerSetEndTeeth_Jeep2000()
{

  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */


/** Audi with 135 teeth on the crank and 1 tooth on the cam.
* This is very similar to the dual wheel decoder, however due to the 135 teeth not dividing evenly into 360,
* only every 3rd crank tooth is used in calculating the crank angle. This effectively makes it a 45 tooth dual wheel setup.
* @defgroup dec_audi135 Audi 135
* @{

*/
void triggerSetup_Audi135()
{
  triggerToothAngle = 8; //135/3 = 45, scaleCrankAngle(360)/45 = 8 degrees every 3 teeth
  toothCurrentCount = 255; //Default value
  toothSystemCount = 0;
  triggerFilterTime = (unsigned long)(1000000 / (MAX_RPM / 60 * 135UL)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be disgarded as noise
  triggerSecFilterTime = (int)(1000000 / (MAX_RPM / 60 * 2)) / 2; //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)
  MAX_STALL_TIME = (3333UL * scaleCrankAngleDown(triggerToothAngle)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  secondDerivEnabled = false;
  decoderIsSequential = true;
  triggerToothAngleIsCorrect = true;
}

void triggerPri_Audi135()
{
   curTime = micros();
   curGap = curTime - toothSystemLastToothTime;
   if ( (curGap > triggerFilterTime) || (currentStatus.startRevolutions == 0) )
   {
     toothSystemCount++;

     if ( currentStatus.hasSync == false ) { toothLastToothTime = curTime; }
     else
     {
       if ( toothSystemCount >= 3 )
       {
         //We only proceed for every third tooth

         validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)
         toothSystemLastToothTime = curTime;
         toothSystemCount = 0;
         toothCurrentCount++; //Increment the tooth counter

         if ( (toothCurrentCount == 1) || (toothCurrentCount > 45) )
         {
           toothCurrentCount = 1;
           toothOneMinusOneTime = toothOneTime;
           toothOneTime = curTime;
           revolutionOne = !revolutionOne;
           currentStatus.startRevolutions++; //Counter
         }

         setFilter(curGap); //Recalc the new filter value

         toothLastMinusOneToothTime = toothLastToothTime;
         toothLastToothTime = curTime;
       } //3rd tooth check
     } // Sync check
   } // Trigger filter
}

void triggerSec_Audi135()
{
  /*
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;
  if ( curGap2 < triggerSecFilterTime ) { return; }
  toothLastSecToothTime = curTime2;
  */

  if( currentStatus.hasSync == false )
  {
    toothCurrentCount = 0;
    currentStatus.hasSync = true;
    toothSystemCount = 3; //Need to set this to 3 so that the next primary tooth is counted
  }
  else if (configPage4.useResync == 1) { toothCurrentCount = 0; toothSystemCount = 3; }
  else if ( (currentStatus.startRevolutions < 100) && (toothCurrentCount != 45) ) { toothCurrentCount = 0; }
  revolutionOne = 1; //Sequential revolution reset
}

uint16_t getRPM_Audi135()
{
   return stdGetRPM(scaleCrankAngle(360));
}

bigAngle_t getCrankAngle_Audi135()
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    bool tempRevolutionOne;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempToothLastToothTime = toothLastToothTime;
    tempRevolutionOne = revolutionOne;
    lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    interrupts();

    //Handle case where the secondary tooth was the last one seen
    if(tempToothCurrentCount == 0) { tempToothCurrentCount = 45; }

    bigAngle_t crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + scaleCrankAngle(configPage4.triggerAngle); //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    
    //Estimate the number of degrees travelled since the last tooth}
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

    //Sequential check (simply sets whether we're on the first or 2nd revoltuion of the cycle)
    if (tempRevolutionOne) { crankAngle += scaleCrankAngle(360); }

    if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
    else if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}

void triggerSetEndTeeth_Audi135()
{
  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */
/** Honda D17 (1.7 liter 4 cyl SOHC).
* 
* @defgroup dec_honda_d17 Honda D17
* @{
*/
void triggerSetup_HondaD17()
{
  triggerToothAngle = scaleCrankAngle(360) / 12; //The number of degrees that passes from tooth to tooth
  MAX_STALL_TIME = (3333UL * scaleCrankAngleDown(triggerToothAngle)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  secondDerivEnabled = false;
  decoderIsSequential = false;
}

void triggerPri_HondaD17()
{
   lastGap = curGap;
   curTime = micros();
   curGap = curTime - toothLastToothTime;
   toothCurrentCount++; //Increment the tooth counter

   validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

   //
   if( (toothCurrentCount == 13) && (currentStatus.hasSync == true) )
   {
     toothCurrentCount = 0;
   }
   else if( (toothCurrentCount == 1) && (currentStatus.hasSync == true) )
   {
     toothOneMinusOneTime = toothOneTime;
     toothOneTime = curTime;
     currentStatus.startRevolutions++; //Counter

     toothLastMinusOneToothTime = toothLastToothTime;
     toothLastToothTime = curTime;
   }
   else
   {
     //13th tooth
     targetGap = (lastGap) >> 1; //The target gap is set at half the last tooth gap
     if ( curGap < targetGap) //If the gap between this tooth and the last one is less than half of the previous gap, then we are very likely at the magical 13th tooth
     {
       toothCurrentCount = 0;
       currentStatus.hasSync = true;
     }
     else
     {
       //The tooth times below don't get set on tooth 13(The magical 13th tooth should not be considered for any calculations that use those times)
       toothLastMinusOneToothTime = toothLastToothTime;
       toothLastToothTime = curTime;
     }
   }

}
void triggerSec_HondaD17() { return; } //The 4+1 signal on the cam is yet to be supported
uint16_t getRPM_HondaD17()
{
   return stdGetRPM(scaleCrankAngle(360));
}
bigAngle_t getCrankAngle_HondaD17()
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempToothLastToothTime = toothLastToothTime;
    lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    interrupts();

    //Check if the last tooth seen was the reference tooth 13 (Number 0 here). All others can be calculated, but tooth 3 has a unique angle
    bigAngle_t crankAngle;
    if( tempToothCurrentCount == 0 )
    {
      crankAngle = (11 * triggerToothAngle) + scaleCrankAngle(configPage4.triggerAngle); //if temptoothCurrentCount is 0, the last tooth seen was the 13th one. Based on this, ignore the 13th tooth and use the 12th one as the last reference.
    }
    else
    {
      crankAngle = scaleCrankAngle(((tempToothCurrentCount - 1) * triggerToothAngle) + configPage4.triggerAngle); //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    }

    //Estimate the number of degrees travelled since the last tooth}
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

    if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
    if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += scaleCrankAngle(360); }

    return crankAngle;
}

void triggerSetEndTeeth_HondaD17()
{
  lastToothCalcAdvance = currentStatus.advance;
}

/** @} */

/** Miata '99 to '05 with 4x 70 degree duration teeth running at cam speed.
Teeth believed to be at the same angles as the 4g63 decoder.
Tooth #1 is defined as the next crank tooth after the crank signal is HIGH when the cam signal is falling.
Tooth number one is at 355* ATDC.
* (See: www.forum.diyefi.org/viewtopic.php?f=56&t=1077)
* @defgroup miata_99_05 Miata '99 to '05
* @{
*/
void triggerSetup_Miata9905()
{
  triggerToothAngle = scaleCrankAngle(90); //The number of degrees that passes from tooth to tooth (primary)
  toothCurrentCount = 99; //Fake tooth count represents no sync
  secondDerivEnabled = false;
  decoderIsSequential = true;
  triggerActualTeeth = 8;

  if(initialisationComplete == false) { secondaryToothCount = 0; toothLastToothTime = micros(); } //Set a startup value here to avoid filter errors when starting. This MUST have the initi check to prevent the fuel pump just staying on all the time
  else { toothLastToothTime = 0; }
  toothLastMinusOneToothTime = 0;

  //Note that these angles are for every rising and falling edge

  /*
  toothAngles[0] = 350;
  toothAngles[1] = 100;
  toothAngles[2] = 170;
  toothAngles[3] = 280;
  */

  toothAngles[0] = scaleCrankAngle(710); //
  toothAngles[1] = scaleCrankAngle(100); //First crank pulse after the SINGLE cam pulse
  toothAngles[2] = scaleCrankAngle(170); //
  toothAngles[3] = scaleCrankAngle(280); //
  toothAngles[4] = scaleCrankAngle(350); //
  toothAngles[5] = scaleCrankAngle(460); //First crank pulse AFTER the DOUBLE cam pulse
  toothAngles[6] = scaleCrankAngle(530); //
  toothAngles[7] = scaleCrankAngle(640); //

  MAX_STALL_TIME = (3333UL * scaleCrankAngleDown(triggerToothAngle)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  triggerFilterTime = 1500; //10000 rpm, assuming we're triggering on both edges off the crank tooth.
  triggerSecFilterTime = 0; //Need to figure out something better for this
  decoderHasFixedCrankingTiming = true;
  triggerToothAngleIsCorrect = true;
}

void triggerPri_Miata9905()
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  if ( (curGap >= triggerFilterTime) || (currentStatus.startRevolutions == 0) )
  {
    toothCurrentCount++;
    validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)
    if( (toothCurrentCount == (triggerActualTeeth + 1)) )
    {
       toothCurrentCount = 1; //Reset the counter
       toothOneMinusOneTime = toothOneTime;
       toothOneTime = curTime;
       //currentStatus.hasSync = true;
       currentStatus.startRevolutions++; //Counter
    }
    else
    {
      if( (currentStatus.hasSync == false) || (configPage4.useResync == true) )
      {
        if(secondaryToothCount == 2)
        {
          toothCurrentCount = 6;
          currentStatus.hasSync = true;
        }
      }
    }

    if (currentStatus.hasSync == true)
    {

      //Whilst this is an uneven tooth pattern, if the specific angle between the last 2 teeth is specified, 1st deriv prediction can be used
      if( (configPage4.triggerFilter == 1) || (currentStatus.RPM < 1400) )
      {
        //Lite filter
        if( (toothCurrentCount == 1) || (toothCurrentCount == 3) || (toothCurrentCount == 5) || (toothCurrentCount == 7) ) { triggerToothAngle = 70; triggerFilterTime = curGap; } //Trigger filter is set to whatever time it took to do 70 degrees (Next trigger is 110 degrees away)
        else { triggerToothAngle = scaleCrankAngle(110); triggerFilterTime = (curGap * 3) >> 3; } //Trigger filter is set to (110*3)/8=41.25=41 degrees (Next trigger is 70 degrees away).
      }
      else if(configPage4.triggerFilter == 2)
      {
        //Medium filter level
        if( (toothCurrentCount == 1) || (toothCurrentCount == 3) || (toothCurrentCount == 5) || (toothCurrentCount == 7) ) { triggerToothAngle = 70; triggerFilterTime = (curGap * 5) >> 2 ; } //87.5 degrees with a target of 110
        else { triggerToothAngle = scaleCrankAngle(110); triggerFilterTime = (curGap >> 1); } //55 degrees with a target of 70
      }
      else if (configPage4.triggerFilter == 3)
      {
        //Aggressive filter level
        if( (toothCurrentCount == 1) || (toothCurrentCount == 3) || (toothCurrentCount == 5) || (toothCurrentCount == 7) ) { triggerToothAngle = 70; triggerFilterTime = (curGap * 11) >> 3 ; } //96.26 degrees with a target of 110
        else { triggerToothAngle = scaleCrankAngle(110); triggerFilterTime = (curGap * 9) >> 5; } //61.87 degrees with a target of 70
      }
      else if (configPage4.triggerFilter == 0)
      {
        //trigger filter is turned off.
        triggerFilterTime = 0;
        triggerSecFilterTime = 0;
        if( (toothCurrentCount == 1) || (toothCurrentCount == 3) || (toothCurrentCount == 5) || (toothCurrentCount == 7) ) { triggerToothAngle = 70; } //96.26 degrees with a target of 110
        else { triggerToothAngle = scaleCrankAngle(110); }
      }

      //EXPERIMENTAL!
      //New ignition mode is ONLY available on 9905 when the trigger angle is set to the stock value of 0.
      if( (configPage2.perToothIgn == true) || (configPage4.triggerAngle == 0) )
      {
        if (currentStatus.advance > scaleCrankAngle(0))
        {
          bigAngle_t crankAngle = ignitionLimits( toothAngles[(toothCurrentCount-1)] );

          //Handle non-sequential tooth counts 
          if( (configPage4.sparkMode != IGN_MODE_SEQUENTIAL) && (toothCurrentCount > configPage2.nCylinders) ) { checkPerToothTiming(crankAngle, (toothCurrentCount-configPage2.nCylinders) ); }
          else { checkPerToothTiming(crankAngle, toothCurrentCount); }
        }
      }

    } //Has sync

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    //if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && configPage4.ignCranklock)
    if ( (currentStatus.RPM < (currentStatus.crankRPM + 30)) && (configPage4.ignCranklock) ) //The +30 here is a safety margin. When switching from fixed timing to normal, there can be a situation where a pulse started when fixed and ending when in normal mode causes problems. This prevents that.
    {
      if( (toothCurrentCount == 1) || (toothCurrentCount == 5) ) { endCoil1Charge(); endCoil3Charge(); }
      else if( (toothCurrentCount == 3) || (toothCurrentCount == 7) ) { endCoil2Charge(); endCoil4Charge(); }
    }
    secondaryToothCount = 0;
  } //Trigger filter

}

void triggerSec_Miata9905()
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;

  if(BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) || (currentStatus.hasSync == false) )
  {
    triggerFilterTime = 1500; //If this is removed, can have trouble getting sync again after the engine is turned off (but ECU not reset).
  }

  if ( curGap2 >= triggerSecFilterTime )
  {
    toothLastSecToothTime = curTime2;
    lastGap = curGap2;
    secondaryToothCount++;

    //TODO Add some secondary filtering here

    //Record the VVT tooth time
    if( (toothCurrentCount == 1) && (curTime2 > toothLastToothTime) )
    {
      lastVVTtime = curTime2 - toothLastToothTime;
    }
  }
}

uint16_t getRPM_Miata9905()
{
  //During cranking, RPM is calculated 4 times per revolution, once for each tooth on the crank signal.
  //Because these signals aren't even (Alternating 110 and 70 degrees), this needs a special function
  uint16_t tempRPM = 0;
  if( (currentStatus.RPM < currentStatus.crankRPM) && (currentStatus.hasSync == true) )
  {
    if( (toothLastToothTime == 0) || (toothLastMinusOneToothTime == 0) ) { tempRPM = 0; }
    else
    {
      int tempToothAngle;
      unsigned long toothTime;
      noInterrupts();
      tempToothAngle = triggerToothAngle;
      toothTime = (toothLastToothTime - toothLastMinusOneToothTime); //Note that trigger tooth angle changes between 70 and 110 depending on the last tooth that was seen
      interrupts();
      toothTime = toothTime * 36;
      tempRPM = ((unsigned long)tempToothAngle * 6000000UL) / toothTime;
      revolutionTime = (10UL * toothTime) / tempToothAngle;
      MAX_STALL_TIME = 366667UL; // 50RPM
    }
  }
  else
  {
    tempRPM = stdGetRPM(scaleCrankAngle(720));
    MAX_STALL_TIME = revolutionTime << 1; //Set the stall time to be twice the current RPM. This is a safe figure as there should be no single revolution where this changes more than this
    if(MAX_STALL_TIME < 366667UL) { MAX_STALL_TIME = 366667UL; } //Check for 50rpm minimum
  }

  return tempRPM;
}

bigAngle_t getCrankAngle_Miata9905()
{
    bigAngle_t crankAngle = 0;
    //if(currentStatus.hasSync == true)
    {
      //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
      unsigned long tempToothLastToothTime;
      int tempToothCurrentCount;
      //Grab some variables that are used in the trigger code and assign them to temp variables.
      noInterrupts();
      tempToothCurrentCount = toothCurrentCount;
      tempToothLastToothTime = toothLastToothTime;
      lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
      interrupts();

      crankAngle = toothAngles[(tempToothCurrentCount - 1)] + scaleCrankAngle(configPage4.triggerAngle); //Perform a lookup of the fixed toothAngles array to find what the angle of the last tooth passed was.

      //Estimate the number of degrees travelled since the last tooth}
      elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
      crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

      if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
      if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
      if (crankAngle < 0) { crankAngle += scaleCrankAngle(360); }
    }

    return crankAngle;
}

int getCamAngle_Miata9905()
{
  int16_t curAngle;
  //lastVVTtime is the time between tooth #1 (10* BTDC) and the single cam tooth. 
  //All cam angles in in BTDC, so the actual advance angle is 370 - fastTimeToAngle(lastVVTtime) - <the angle of the cam at 0 advance>
  curAngle = scaleCrankAngle(370) - fastTimeToAngle(lastVVTtime) - scaleCrankAngle(configPage10.vvtCLMinAng);
  currentStatus.vvt1Angle = ANGLE_FILTER( (curAngle << 1), configPage4.ANGLEFILTER_VVT, currentStatus.vvt1Angle);

  return currentStatus.vvt1Angle;
}

void triggerSetEndTeeth_Miata9905()
{

  if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL) 
  { 
    if(currentStatus.advance >= 10)
    {
      ignition1EndTooth = 8;
      ignition2EndTooth = 2;
      ignition3EndTooth = 4;
      ignition4EndTooth = 6;
    }
    else if (currentStatus.advance > scaleCrankAngle(0))
    {
      ignition1EndTooth = 1;
      ignition2EndTooth = 3;
      ignition3EndTooth = 5;
      ignition4EndTooth = 7;
    }
    
  }
  else
  {
    if(currentStatus.advance >= scaleCrankAngle(10))
    {
      ignition1EndTooth = 4;
      ignition2EndTooth = 2;
      ignition3EndTooth = 4; //Not used
      ignition4EndTooth = 2; //Not used
    }
    else if(currentStatus.advance > scaleCrankAngle(0))
    {
      ignition1EndTooth = 1;
      ignition2EndTooth = 3;
      ignition3EndTooth = 1; //Not used
      ignition4EndTooth = 3; //Not used
    }
  }

  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

/** Mazda AU version.
Tooth #2 is defined as the next crank tooth after the single cam tooth.
Tooth number one is at 348* ATDC.
* @defgroup mazda_au Mazda AU
* @{
*/
void triggerSetup_MazdaAU()
{
  triggerToothAngle = scaleCrankAngle(108); //The number of degrees that passes from tooth to tooth (primary). This is the maximum gap
  toothCurrentCount = 99; //Fake tooth count represents no sync
  secondaryToothCount = 0; //Needed for the cam tooth tracking
  secondDerivEnabled = false;
  decoderIsSequential = true;

  toothAngles[0] = scaleCrankAngle(348); //tooth #1
  toothAngles[1] = scaleCrankAngle(96);  //tooth #2
  toothAngles[2] = scaleCrankAngle(168); //tooth #3
  toothAngles[3] = scaleCrankAngle(276); //tooth #4

  MAX_STALL_TIME = (3333UL * scaleCrankAngleDown(triggerToothAngle)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  triggerFilterTime = 1500; //10000 rpm, assuming we're triggering on both edges off the crank tooth.
  triggerSecFilterTime = (int)(1000000 / (MAX_RPM / 60 * 2)) / 2; //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)
  decoderHasFixedCrankingTiming = true;
}

void triggerPri_MazdaAU()
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  if ( curGap >= triggerFilterTime )
  {
    validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

    toothCurrentCount++;
    if( (toothCurrentCount == 1) || (toothCurrentCount == 5) ) //Trigger is on CHANGE, hence 4 pulses = 1 crank rev
    {
       toothCurrentCount = 1; //Reset the counter
       toothOneMinusOneTime = toothOneTime;
       toothOneTime = curTime;
       currentStatus.hasSync = true;
       currentStatus.startRevolutions++; //Counter
    }

    if (currentStatus.hasSync == true)
    {
      // Locked cranking timing is available, fixed at 12* BTDC
      if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && configPage4.ignCranklock )
      {
        if( toothCurrentCount == 1 ) { endCoil1Charge(); }
        else if( toothCurrentCount == 3 ) { endCoil2Charge(); }
      }

      //Whilst this is an uneven tooth pattern, if the specific angle between the last 2 teeth is specified, 1st deriv prediction can be used
      if( (toothCurrentCount == 1) || (toothCurrentCount == 3) ) { triggerToothAngle = scaleCrankAngle(72); triggerFilterTime = curGap; } //Trigger filter is set to whatever time it took to do 72 degrees (Next trigger is 108 degrees away)
      else { triggerToothAngle = scaleCrankAngle(108); triggerFilterTime = (curGap * 3) >> 3; } //Trigger filter is set to (108*3)/8=40 degrees (Next trigger is 70 degrees away).

      toothLastMinusOneToothTime = toothLastToothTime;
      toothLastToothTime = curTime;
    } //Has sync
  } //Filter time
}

void triggerSec_MazdaAU()
{
  curTime2 = micros();
  lastGap = curGap2;
  curGap2 = curTime2 - toothLastSecToothTime;
  //if ( curGap2 < triggerSecFilterTime ) { return; }
  toothLastSecToothTime = curTime2;

  //if(BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) || currentStatus.hasSync == false)
  if(currentStatus.hasSync == false)
  {
    //we find sync by looking for the 2 teeth that are close together. The next crank tooth after that is the one we're looking for.
    //For the sake of this decoder, the lone cam tooth will be designated #1
    if(secondaryToothCount == 2)
    {
      toothCurrentCount = 1;
      currentStatus.hasSync = true;
    }
    else
    {
      triggerFilterTime = 1500; //In case the engine has been running and then lost sync.
      targetGap = (lastGap) >> 1; //The target gap is set at half the last tooth gap
      if ( curGap2 < targetGap) //If the gap between this tooth and the last one is less than half of the previous gap, then we are very likely at the extra (3rd) tooth on the cam). This tooth is located at 421 crank degrees (aka 61 degrees) and therefore the last crank tooth seen was number 1 (At 350 degrees)
      {
        secondaryToothCount = 2;
      }
    }
    secondaryToothCount++;
  }
}


uint16_t getRPM_MazdaAU()
{
  uint16_t tempRPM = 0;

  if (currentStatus.hasSync == true)
  {
    //During cranking, RPM is calculated 4 times per revolution, once for each tooth on the crank signal.
    //Because these signals aren't even (Alternating 108 and 72 degrees), this needs a special function
    if(currentStatus.RPM < currentStatus.crankRPM)
    {
      int tempToothAngle;
      noInterrupts();
      tempToothAngle = triggerToothAngle;
      revolutionTime = (toothLastToothTime - toothLastMinusOneToothTime); //Note that trigger tooth angle changes between 72 and 108 depending on the last tooth that was seen
      interrupts();
      revolutionTime = revolutionTime * 36;
      tempRPM = (tempToothAngle * 60000000L) / revolutionTime;
    }
    else { tempRPM = stdGetRPM(scaleCrankAngle(360)); }
  }
  return tempRPM;
}

bigAngle_t getCrankAngle_MazdaAU()
{
    bigAngle_t crankAngle = 0;
    if(currentStatus.hasSync == true)
    {
      //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
      unsigned long tempToothLastToothTime;
      int tempToothCurrentCount;
      //Grab some variables that are used in the trigger code and assign them to temp variables.
      noInterrupts();
      tempToothCurrentCount = toothCurrentCount;
      tempToothLastToothTime = toothLastToothTime;
      lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
      interrupts();

      crankAngle = toothAngles[(tempToothCurrentCount - 1)] + scaleCrankAngle(configPage4.triggerAngle); //Perform a lookup of the fixed toothAngles array to find what the angle of the last tooth passed was.

      //Estimate the number of degrees travelled since the last tooth}
      elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
      crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

      if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
      if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
      if (crankAngle < 0) { crankAngle += scaleCrankAngle(360); }
    }

    return crankAngle;
}

void triggerSetEndTeeth_MazdaAU()
{
  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

/** Non-360 Dual wheel with 2 wheels located either both on the crank or with the primary on the crank and the secondary on the cam.
There can be no missing teeth on the primary wheel.
* @defgroup dec_non360 Non-360 Dual wheel
* @{
*/
void triggerSetup_non360()
{
  triggerToothAngle = (scaleCrankAngle(360) * configPage4.TrigAngMul) / configPage4.triggerTeeth; //The number of degrees that passes from tooth to tooth multiplied by the additional multiplier
  toothCurrentCount = 255; //Default value
  triggerFilterTime = (1000000 / (MAX_RPM / 60 * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be disgarded as noise
  triggerSecFilterTime = (1000000 / (MAX_RPM / 60 * 2)) / 2; //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)
  secondDerivEnabled = false;
  decoderIsSequential = true;
  MAX_STALL_TIME = (3333UL * scaleCrankAngleDown(triggerToothAngle)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}


void triggerPri_non360()
{
  //This is not used, the trigger is identical to the dual wheel one, so that is used instead.
}

void triggerSec_non360()
{
  //This is not used, the trigger is identical to the dual wheel one, so that is used instead.
}

uint16_t getRPM_non360()
{
  uint16_t tempRPM = 0;
  if( (currentStatus.hasSync == true) && (toothCurrentCount != 0) )
  {
    if(currentStatus.RPM < currentStatus.crankRPM) { tempRPM = crankingGetRPM(configPage4.triggerTeeth, scaleCrankAngle(360)); }
    else { tempRPM = stdGetRPM(scaleCrankAngle(360)); }
  }
  return tempRPM;
}

bigAngle_t getCrankAngle_non360()
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempToothLastToothTime = toothLastToothTime;
    lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    interrupts();

    //Handle case where the secondary tooth was the last one seen
    if(tempToothCurrentCount == 0) { tempToothCurrentCount = configPage4.triggerTeeth; }

    //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    bigAngle_t crankAngle = (tempToothCurrentCount - 1) * triggerToothAngle;
    crankAngle = (crankAngle / configPage4.TrigAngMul) + scaleCrankAngle(configPage4.triggerAngle); //Have to divide by the multiplier to get back to actual crank angle.

    //Estimate the number of degrees travelled since the last tooth}
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

    if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
    if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += scaleCrankAngle(360); }

    return crankAngle;
}

void triggerSetEndTeeth_non360()
{
  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

/** Nissan 360 tooth on cam (Optical trigger disc inside distributor housing).
See http://wiki.r31skylineclub.com/index.php/Crank_Angle_Sensor .
* @defgroup dec_nissan360 Nissan 360 tooth on cam
* @{
*/
void triggerSetup_Nissan360()
{
  triggerFilterTime = (1000000 / (MAX_RPM / 60 * 360UL)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be disgarded as noise
  triggerSecFilterTime = (int)(1000000 / (MAX_RPM / 60 * 2)) / 2; //Same as above, but fixed at 2 teeth on the secondary input and divided by 2 (for cam speed)
  secondaryToothCount = 0; //Initially set to 0 prior to calculating the secondary window duration
  secondDerivEnabled = false;
  decoderIsSequential = true;
  toothCurrentCount = 1;
  triggerToothAngle = 2;
  MAX_STALL_TIME = (3333UL * scaleCrankAngleDown(triggerToothAngle)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}


void triggerPri_Nissan360()
{
   curTime = micros();
   curGap = curTime - toothLastToothTime;
   //if ( curGap < triggerFilterTime ) { return; }
   toothCurrentCount++; //Increment the tooth counter
   validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

   toothLastMinusOneToothTime = toothLastToothTime;
   toothLastToothTime = curTime;

   if ( currentStatus.hasSync == true )
   {
     if ( toothCurrentCount == 361 ) //2 complete crank revolutions
     {
       toothCurrentCount = 1;
       toothOneMinusOneTime = toothOneTime;
       toothOneTime = curTime;
       currentStatus.startRevolutions++; //Counter
     }
     //Recalc the new filter value
     //setFilter(curGap);

     //EXPERIMENTAL!
     if(configPage2.perToothIgn == true)
     {
        bigAngle_t crankAngle = ( (toothCurrentCount-1) * 2 ) + scaleCrankAngle(configPage4.triggerAngle);
        if(crankAngle > CRANK_ANGLE_MAX_IGN) 
        { 
          crankAngle -= CRANK_ANGLE_MAX_IGN;
          checkPerToothTiming(crankAngle, (toothCurrentCount/2) );
        }
        else
        {
          checkPerToothTiming(crankAngle, toothCurrentCount);
        }
       
     }

     timePerDegree = curGap << (ANGLE_SHIFTS - 1);; //The time per crank degree is simply the time between this tooth and the last one divided by 2
   }
}

void triggerSec_Nissan360()
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;
  //if ( curGap2 < triggerSecFilterTime ) { return; }
  toothLastSecToothTime = curTime2;
  //OPTIONAL: Set filter at 25% of the current speed
  //triggerSecFilterTime = curGap2 >> 2;


  //Calculate number of primary teeth that this window has been active for
  byte trigEdge;
  if(configPage4.TrigEdgeSec == 0) { trigEdge = LOW; }
  else { trigEdge = HIGH; }

  if( (secondaryToothCount == 0) || (READ_SEC_TRIGGER() == trigEdge) ) { secondaryToothCount = toothCurrentCount; } //This occurs on the first rotation upon powerup OR the start of a secondary window
  else
  {
    //If we reach here, we are at the end of a secondary window
    byte secondaryDuration = toothCurrentCount - secondaryToothCount; //How many primary teeth have passed during the duration of this secondary window

    if(currentStatus.hasSync == false)
    {
      if(configPage2.nCylinders == 4)
      {
        //Supported pattern is where all the inner windows as a different size (Most SR engines)
        //These equate to 4,8,12,16 teeth spacings
        if( (secondaryDuration >= 15) && (secondaryDuration <= 17) ) //Duration of window = 16 primary teeth
        {
          toothCurrentCount = 16; //End of first window (The longest) occurs 16 teeth after TDC
          currentStatus.hasSync = true;
        }
        else if( (secondaryDuration >= 11) && (secondaryDuration <= 13) ) //Duration of window = 12 primary teeth
        {
          toothCurrentCount = 102; //End of second window is after 90+12 primary teeth
          currentStatus.hasSync = true;
        }
        else if( (secondaryDuration >= 7) && (secondaryDuration <= 9) ) //Duration of window = 8 primary teeth
        {
          toothCurrentCount = 188; //End of third window is after 90+90+8 primary teeth
          currentStatus.hasSync = true;
        }
        else if( (secondaryDuration >= 3) && (secondaryDuration <= 5) ) //Duration of window = 4 primary teeth
        {
          toothCurrentCount = 274; //End of fourth window is after 90+90+90+4 primary teeth
          currentStatus.hasSync = true;
        }
        else { currentStatus.hasSync = false; currentStatus.syncLossCounter++; } //This should really never happen
      }
      else if(configPage2.nCylinders == 6)
      {
        //Pattern on the 6 cylinders is 4-8-12-16-20-24
        if( (secondaryDuration >= 3) && (secondaryDuration <= 5) ) //Duration of window = 4 primary teeth
        {
          toothCurrentCount = 124; //End of smallest window is after 60+60+4 primary teeth
          currentStatus.hasSync = true;
        }
      }
      else if(configPage2.nCylinders == 8)
      {
        //V8 Optispark
        //Pattern on the 8 cylinders is the same as the 6 cylinder 4-8-12-16-20-24
        if( (secondaryDuration >= 6) && (secondaryDuration <= 8) ) //Duration of window = 16 primary teeth
        {
          toothCurrentCount = 56; //End of the shortest of the individual windows. Occurs at 102 crank degrees. 
          currentStatus.hasSync = true;
        }
      }
      else { currentStatus.hasSync = false; } //This should really never happen (Only 4, 6 and 8 cylinder engines for this pattern)
    }
    else
    {
      if (configPage4.useResync == true)
      {
        //Already have sync, but do a verify every scaleCrankAngle(720) degrees.
        if(configPage2.nCylinders == 4)
        {
          if( (secondaryDuration >= 15) && (secondaryDuration <= 17) ) //Duration of window = 16 primary teeth
          {
            toothCurrentCount = 16; //End of first window (The longest) occurs 16 teeth after TDC
          }
        }
        else if(configPage2.nCylinders == 6)
        {
          if(secondaryDuration == 4)
          {
            //toothCurrentCount = 304;
          }
        } //Cylinder count
      } //use resync
    } //Has sync
  } //First getting sync or not
}

uint16_t getRPM_Nissan360()
{
  //Can't use stdGetRPM as there is no separate cranking RPM calc (stdGetRPM returns 0 if cranking)
  uint16_t tempRPM;
  if( (currentStatus.hasSync == true) && (toothLastToothTime != 0) && (toothLastMinusOneToothTime != 0) )
  {
    if(currentStatus.startRevolutions < 2)
    {
      noInterrupts();
      revolutionTime = (toothLastToothTime - toothLastMinusOneToothTime) * scaleCrankAngle(180); //Each tooth covers 2 crank degrees, so multiply by 180 to get a full revolution time. 
      interrupts();
    }
    else
    {
      noInterrupts();
      revolutionTime = (toothOneTime - toothOneMinusOneTime) >> 1; //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
      interrupts();
    }
    tempRPM = (US_IN_MINUTE / revolutionTime); //Calc RPM based on last full revolution time (Faster as /)
    if(tempRPM >= MAX_RPM) { tempRPM = currentStatus.RPM; } //Sanity check
    MAX_STALL_TIME = revolutionTime << 1; //Set the stall time to be twice the current RPM. This is a safe figure as there should be no single revolution where this changes more than this
  }
  else { tempRPM = 0; }

  return tempRPM;
}

bigAngle_t getCrankAngle_Nissan360()
{
  //As each tooth represents 2 crank degrees, we only need to determine whether we're more or less than halfway between teeth to know whether to add another 1 degrees
  bigAngle_t crankAngle = 0;
  int tempToothLastToothTime;
  int tempToothLastMinusOneToothTime;
  int tempToothCurrentCount;

  noInterrupts();
  tempToothLastToothTime = toothLastToothTime;
  tempToothLastMinusOneToothTime = toothLastMinusOneToothTime;
  tempToothCurrentCount = toothCurrentCount;
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
  interrupts();

  crankAngle = ( (tempToothCurrentCount - 1) * 2) + scaleCrankAngle(configPage4.triggerAngle);
  unsigned long halfTooth = (tempToothLastToothTime - tempToothLastMinusOneToothTime) / 2;
  if (elapsedTime > halfTooth)
  {
    //Means we're over halfway to the next tooth, so add on 1 degree
    crankAngle += 1;
  }

  if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
  if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
  if (crankAngle < 0) { crankAngle += scaleCrankAngle(360); }

  return crankAngle;
}

void triggerSetEndTeeth_Nissan360()
{
  //This uses 4 prior teeth, just to ensure there is sufficient time to set the schedule etc
  byte offset_teeth = 4;
  if((ignition1EndAngle - offset_teeth) > scaleCrankAngle(configPage4.triggerAngle)) { ignition1EndTooth = ( (ignition1EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / 2 ) - offset_teeth; }
  else { ignition1EndTooth = ( (ignition1EndAngle + scaleCrankAngle(720) - scaleCrankAngle(configPage4.triggerAngle)) / 2 ) - offset_teeth; }
  if((ignition2EndAngle - offset_teeth) > scaleCrankAngle(configPage4.triggerAngle)) { ignition2EndTooth = ( (ignition2EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / 2 ) - offset_teeth; }
  else { ignition2EndTooth = ( (ignition2EndAngle + scaleCrankAngle(720) - scaleCrankAngle(configPage4.triggerAngle)) / 2 ) - offset_teeth; }
  if((ignition3EndAngle - offset_teeth) > scaleCrankAngle(configPage4.triggerAngle)) { ignition3EndTooth = ( (ignition3EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / 2 ) - offset_teeth; }
  else { ignition3EndTooth = ( (ignition3EndAngle + scaleCrankAngle(720) - scaleCrankAngle(configPage4.triggerAngle)) / 2 ) - offset_teeth; }
  if((ignition4EndAngle - offset_teeth) > scaleCrankAngle(configPage4.triggerAngle)) { ignition4EndTooth = ( (ignition4EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / 2 ) - offset_teeth; }
  else { ignition4EndTooth = ( (ignition4EndAngle + scaleCrankAngle(720) - scaleCrankAngle(configPage4.triggerAngle)) / 2 ) - offset_teeth; }

  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

/** Subaru 6/7 Trigger pattern decoder for 6 tooth (irregularly spaced) crank and 7 tooth (also fairly irregular) cam wheels (eg late 90's Impreza 2.2).
This seems to be present in late 90's Subaru. In 2001 Subaru moved to 36-2-2-2 (See: http://www.vems.hu/wiki/index.php?page=InputTrigger%2FSubaruTrigger ).
* @defgroup dec_subaru_6_7 Subaru 6/7
* @{
*/
void triggerSetup_Subaru67()
{
  triggerFilterTime = (1000000 / (MAX_RPM / 60 * 360UL)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be disgarded as noise
  triggerSecFilterTime = 0;
  secondaryToothCount = 0; //Initially set to 0 prior to calculating the secondary window duration
  secondDerivEnabled = false;
  decoderIsSequential = true;
  toothCurrentCount = 1;
  triggerToothAngle = scaleCrankAngle(2);
  triggerToothAngleIsCorrect = false;
  toothSystemCount = 0;
  MAX_STALL_TIME = (3333UL * 93); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)

  toothAngles[0] = 710; //tooth #1
  toothAngles[1] = 83; //tooth #2
  toothAngles[2] = 115; //tooth #3
  toothAngles[3] = 170; //tooth #4
  toothAngles[4] = toothAngles[1] + scaleCrankAngle(180);
  toothAngles[5] = toothAngles[2] + scaleCrankAngle(180);
  toothAngles[6] = toothAngles[3] + scaleCrankAngle(180);
  toothAngles[7] = toothAngles[1] + scaleCrankAngle(360);
  toothAngles[8] = toothAngles[2] + scaleCrankAngle(360);
  toothAngles[9] = toothAngles[3] + scaleCrankAngle(360);
  toothAngles[10] = toothAngles[1] + scaleCrankAngle(540);
  toothAngles[11] = toothAngles[2] + scaleCrankAngle(540);
}


void triggerPri_Subaru67()
{
   curTime = micros();
   //curGap = curTime - toothLastToothTime;
   //if ( curGap < triggerFilterTime ) { return; }
   toothCurrentCount++; //Increment the tooth counter
   toothSystemCount++; //Used to count the number of primary pulses that have occurred since the last secondary. Is part of the noise filtering system.
   validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

   toothLastMinusOneToothTime = toothLastToothTime;
   toothLastToothTime = curTime;

   if ( (currentStatus.hasSync == false) || (configPage4.useResync == true) )
   {
     if(toothCurrentCount > 12) { toothCurrentCount = toothCurrentCount % 12; } //Because toothCurrentCount is not being reset when hitting tooth 1, we manually loop it here. 

     //Sync is determined by counting the number of cam teeth that have passed between the crank teeth
     switch(secondaryToothCount)
     {
        case 0:
          //If no teeth have passed, we can't do anything
          break;

        case 1:
          //Can't do anything with a single pulse from the cam either (We need either 2 or 3 pulses)
          secondaryToothCount = 0;
          break;

        case 2:
          toothCurrentCount = 8;
          //currentStatus.hasSync = true;
          secondaryToothCount = 0;
          break;

        case 3:
          //toothCurrentCount = 2;
          if( toothCurrentCount == 2)
          {
            currentStatus.hasSync = true;
          }
          secondaryToothCount = 0;
          break;

        default:
          //Almost certainly due to noise or cranking stop/start
          currentStatus.hasSync = false;
          triggerToothAngleIsCorrect = false;
          currentStatus.syncLossCounter++;
          secondaryToothCount = 0;
          break;

     }
   }

   //Check sync again
   if ( currentStatus.hasSync == true )
   {
      //Locked timing during cranking. This is fixed at 10* BTDC.
      if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && configPage4.ignCranklock)
      {
        if( (toothCurrentCount == 1) || (toothCurrentCount == 7) ) { endCoil1Charge(); endCoil3Charge(); }
        else if( (toothCurrentCount == 4) || (toothCurrentCount == 10) ) { endCoil2Charge(); endCoil4Charge(); }
      }

      if ( toothCurrentCount > 12 ) //2 complete crank revolutions
      {
        toothCurrentCount = 1;
        toothOneMinusOneTime = toothOneTime;
        toothOneTime = curTime;
        currentStatus.startRevolutions++; //Counter
      }

      //Set the last angle between teeth for better calc accuracy
      if(toothCurrentCount == 1) { triggerToothAngle = scaleCrankAngle(55); } //Special case for tooth 1
      else if(toothCurrentCount == 2) { triggerToothAngle = scaleCrankAngle(93); } //Special case for tooth 2
      else { triggerToothAngle = toothAngles[(toothCurrentCount-1)] - toothAngles[(toothCurrentCount-2)]; }
      triggerToothAngleIsCorrect = true;


      //NEW IGNITION MODE
      if( (configPage2.perToothIgn == true) && (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) ) 
      {
        bigAngle_t crankAngle = toothAngles[(toothCurrentCount - 1)] + scaleCrankAngle(configPage4.triggerAngle);
        if( (configPage4.sparkMode != IGN_MODE_SEQUENTIAL) )
        {
          crankAngle = ignitionLimits( toothAngles[(toothCurrentCount-1)] );

          //Handle non-sequential tooth counts 
          if( (configPage4.sparkMode != IGN_MODE_SEQUENTIAL) && (toothCurrentCount > 6) ) { checkPerToothTiming(crankAngle, (toothCurrentCount-6) ); }
          else { checkPerToothTiming(crankAngle, toothCurrentCount); }
        }
        else{ checkPerToothTiming(crankAngle, toothCurrentCount); }
      }
   //Recalc the new filter value
   //setFilter(curGap);
   }
 }

void triggerSec_Subaru67()
{
  if( (toothSystemCount == 0) || (toothSystemCount == 3) )
  {
    curTime2 = micros();
    curGap2 = curTime2 - toothLastSecToothTime;
    
    if ( curGap2 > triggerSecFilterTime ) 
    {
      toothLastSecToothTime = curTime2;
      secondaryToothCount++;
      toothSystemCount = 0;

      if(secondaryToothCount > 1)
      {
        //Set filter at 25% of the current speed
        //Note that this can only be set on the 2nd or 3rd cam tooth in each set. 
        triggerSecFilterTime = curGap2 >> 2;
      }
      else { triggerSecFilterTime = 0; } //Filter disabled

    }
  }
  else
  {
    //Sanity check
    if(toothSystemCount > 3)
    { 
      toothSystemCount = 0; 
      secondaryToothCount = 1;
    }
  }

}

uint16_t getRPM_Subaru67()
{
  //if(currentStatus.RPM < currentStatus.crankRPM) { return crankingGetRPM(configPage4.triggerTeeth); }

  uint16_t tempRPM = 0;
  if(currentStatus.startRevolutions > 0)
  {
    //As the tooth count is over scaleCrankAngle(720) degrees
    tempRPM = stdGetRPM(scaleCrankAngle(720));
  }
  return tempRPM;
}

bigAngle_t getCrankAngle_Subaru67()
{
  bigAngle_t crankAngle = 0;
  if( currentStatus.hasSync == true )
  {
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempToothLastToothTime = toothLastToothTime;
    lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    interrupts();

    crankAngle = toothAngles[(tempToothCurrentCount - 1)] + scaleCrankAngle(configPage4.triggerAngle); //Perform a lookup of the fixed toothAngles array to find what the angle of the last tooth passed was.

    //Estimate the number of degrees travelled since the last tooth}
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_TOOTH);

    if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
    if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += scaleCrankAngle(360); }
  }

  return crankAngle;
}

void triggerSetEndTeeth_Subaru67()
{
  if(configPage4.sparkMode == IGN_MODE_SEQUENTIAL)
  {
    //if(ignition1EndAngle < 710) { ignition1EndTooth = 12; }
    if(currentStatus.advance >= scaleCrankAngle(10) ) 
    { 
      ignition1EndTooth = 12;
      ignition2EndTooth = 3;
      ignition3EndTooth = 6;
      ignition4EndTooth = 9;
    }
    else 
    { 
      ignition1EndTooth = 1;
      ignition2EndTooth = 4;
      ignition3EndTooth = 7;
      ignition4EndTooth = 10;
    }
  }
  else    
  {
    if(currentStatus.advance >= scaleCrankAngle(10) ) 
    { 
      ignition1EndTooth = 6;
      ignition2EndTooth = 3;
      //ignition3EndTooth = 6;
      //ignition4EndTooth = 9;
    }
    else 
    { 
      ignition1EndTooth = 1;
      ignition2EndTooth = 4;
      //ignition3EndTooth = 7;
      //ignition4EndTooth = 10;
    }
  }
  
  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

/** Daihatsu +1 trigger for 3 and 4 cylinder engines.
* Tooth equal to the number of cylinders are evenly spaced on the cam. No position sensing (Distributor is retained),
* so crank angle is a made up figure based purely on the first teeth to be seen.
* Note: This is a very simple decoder. See http://www.megamanual.com/ms2/GM_7pinHEI.htm
* @defgroup dec_daihatsu Daihatsu (3  and 4 cyl.)
* @{
*/
void triggerSetup_Daihatsu()
{
  triggerActualTeeth = configPage2.nCylinders + 1;
  triggerToothAngle = scaleCrankAngle(720) / triggerActualTeeth; //The number of degrees that passes from tooth to tooth
  triggerFilterTime = 60000000L / MAX_RPM / configPage2.nCylinders; // Minimum time required between teeth
  triggerFilterTime = triggerFilterTime / 2; //Safety margin
  secondDerivEnabled = false;
  decoderIsSequential = false;

  MAX_STALL_TIME = (1851UL * scaleCrankAngleDown(triggerToothAngle))*4;//Minimum 90rpm. (1851uS is the time per degree at 90rpm). This uses 90rpm rather than 50rpm due to the potentially very high stall time on a 4 cylinder if we wait that long.

  if(configPage2.nCylinders == 3)
  {
    toothAngles[0] = scaleCrankAngle(0); //tooth #1
    toothAngles[1] = scaleCrankAngle(30); //tooth #2 (Extra tooth)
    toothAngles[2] = scaleCrankAngle(240); //tooth #3
    toothAngles[3] = scaleCrankAngle(480); //tooth #4
  }
  else
  {
    //Should be 4 cylinders here
    toothAngles[0] = scaleCrankAngle(0); //tooth #1
    toothAngles[1] = scaleCrankAngle(30); //tooth #2 (Extra tooth)
    toothAngles[2] = scaleCrankAngle(180); //tooth #3
    toothAngles[3] = scaleCrankAngle(360); //tooth #4
    toothAngles[4] = scaleCrankAngle(540); //tooth #5
  }
}

void triggerPri_Daihatsu()
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;

  //if ( curGap >= triggerFilterTime || (currentStatus.startRevolutions == 0 )
  {
    toothSystemCount++;
    validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

    if (currentStatus.hasSync == true)
    {
      if( (toothCurrentCount == triggerActualTeeth) ) //Check if we're back to the beginning of a revolution
      {
         toothCurrentCount = 1; //Reset the counter
         toothOneMinusOneTime = toothOneTime;
         toothOneTime = curTime;
         currentStatus.hasSync = true;
         currentStatus.startRevolutions++; //Counter

         //Need to set a special filter time for the next tooth
         triggerFilterTime = 20; //Fix this later
      }
      else
      {
        toothCurrentCount++; //Increment the tooth counter
        setFilter(curGap); //Recalc the new filter value
      }

      if ( configPage4.ignCranklock && BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
      {
        //This locks the cranking timing to 0 degrees BTDC (All the triggers allow for)
        if(toothCurrentCount == 1) { endCoil1Charge(); }
        else if(toothCurrentCount == 2) { endCoil2Charge(); }
        else if(toothCurrentCount == 3) { endCoil3Charge(); }
        else if(toothCurrentCount == 4) { endCoil4Charge(); }
      }
    }
    else //NO SYNC
    {
      //
      if(toothSystemCount >= 3) //Need to have seen at least 3 teeth to determine SYNC
      {
        unsigned long targetTime;
        //We need to try and find the extra tooth (#2) which is located 30 degrees after tooth #1
        //Aim for tooth times less than about 60 degrees
        if(configPage2.nCylinders == 3)
        {
          targetTime = (toothLastToothTime -  toothLastMinusOneToothTime) / 4; //Teeth are 240 degrees apart for 3 cylinder. 240/4 = 60
        }
        else
        {
          targetTime = ((toothLastToothTime -  toothLastMinusOneToothTime) * 3) / 8; //Teeth are 180 degrees apart for 4 cylinder. (180*3)/8 = 67
        }
        if(curGap < targetTime)
        {
          //Means we're on the extra tooth here
          toothCurrentCount = 2; //Reset the counter
          currentStatus.hasSync = true;
          triggerFilterTime = targetTime; //Lazy, but it works
        }
      }
    }

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;
  } //Trigger filter
}
void triggerSec_Daihatsu() { return; } //Not required (Should never be called in the first place)

uint16_t getRPM_Daihatsu()
{
  uint16_t tempRPM = 0;
  if( (currentStatus.RPM < currentStatus.crankRPM) && false) //Disable special cranking processing for now
  {
    //Cn't use standard cranking RPM functin due to extra tooth
    if( currentStatus.hasSync == true )
    {
      if(toothCurrentCount == 2) { tempRPM = currentStatus.RPM; }
      else if (toothCurrentCount == 3) { tempRPM = currentStatus.RPM; }
      else
      {
        noInterrupts();
        revolutionTime = (toothLastToothTime - toothLastMinusOneToothTime) * (triggerActualTeeth-1);
        interrupts();
        tempRPM = (US_IN_MINUTE / revolutionTime);
        if(tempRPM >= MAX_RPM) { tempRPM = currentStatus.RPM; } //Sanity check
      } //is tooth #2
    }
    else { tempRPM = 0; } //No sync
  }
  else
  { tempRPM = stdGetRPM(scaleCrankAngle(720)); } //TRacking over 2 crank revolutions

  return tempRPM;

}

bigAngle_t getCrankAngle_Daihatsu()
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    bigAngle_t crankAngle;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempToothLastToothTime = toothLastToothTime;
    lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
    interrupts();

    crankAngle = toothAngles[tempToothCurrentCount-1] + scaleCrankAngle(configPage4.triggerAngle); //Crank angle of the last tooth seen

    //Estimate the number of degrees travelled since the last tooth}
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

    if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
    if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}

void triggerSetEndTeeth_Daihatsu()
{
  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

/** Harley Davidson (V2) with 2 unevenly Spaced Teeth.
Within the decoder code, the sync tooth is referred to as tooth #1. Derived from GMX7 and adapted for Harley.
Only rising Edge is used for simplicity.The second input is ignored, as it does not help to desolve cam position.
* @defgroup dec_harley Harley Davidson
* @{
*/
void triggerSetup_Harley()
{
  triggerToothAngle = scaleCrankAngle(0); // The number of degrees that passes from tooth to tooth, ev. 0. It alternates uneven
  secondDerivEnabled = false;
  decoderIsSequential = false;
  MAX_STALL_TIME = (3333UL * 60); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
  if(initialisationComplete == false) { toothLastToothTime = micros(); } //Set a startup value here to avoid filter errors when starting. This MUST have the initi check to prevent the fuel pump just staying on all the time
  triggerFilterTime = 1500;
}

void triggerPri_Harley()
{
  lastGap = curGap;
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  setFilter(curGap); // Filtering adjusted according to setting
  if (curGap > triggerFilterTime)
  {
    if ( READ_PRI_TRIGGER() == HIGH) // Has to be the same as in main() trigger-attach, for readability we do it this way.
    {
        validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)
        targetGap = lastGap ; //Gap is the Time to next toothtrigger, so we know where we are
        toothCurrentCount++;
        if (curGap > targetGap)
        {
          toothCurrentCount = 1;
          triggerToothAngle = 0;// Has to be equal to Angle Routine
          toothOneMinusOneTime = toothOneTime;
          toothOneTime = curTime;
          currentStatus.hasSync = true;
        }
        else
        {
          toothCurrentCount = 2;
          //     toothOneMinusOneTime = toothOneTime;
          //     toothOneTime = curTime;
        }
        toothLastMinusOneToothTime = toothLastToothTime;
        toothLastToothTime = curTime;
        currentStatus.startRevolutions++; //Counter
    }
    else
    {
      if (currentStatus.hasSync == true) { currentStatus.syncLossCounter++; }
      currentStatus.hasSync = false;
      toothCurrentCount = 0;
    } //Primary trigger high
  } //Trigger filter
}


void triggerSec_Harley()
// Needs to be enabled in main()
{
  return;// No need for now. The only thing it could help to sync more quikly or confirm position.
} // End Sec Trigger


uint16_t getRPM_Harley()
{
  uint16_t tempRPM = 0;
  if (currentStatus.hasSync == true)
  {
    if ( currentStatus.RPM < (unsigned int)(configPage4.crankRPM * 100) )
    {
      // Kein Unterschied mit dieser Option
      bigAngle_t tempToothAngle;
      unsigned long toothTime;
      if ( (toothLastToothTime == 0) || (toothLastMinusOneToothTime == 0) ) { tempRPM = 0; }
      else
      {
        noInterrupts();
        tempToothAngle = triggerToothAngle;
        /* High-res mode
          if(toothCurrentCount == 1) { tempToothAngle = scaleCrankAngle(129); }
          else { tempToothAngle = toothAngles[toothCurrentCount-1] - toothAngles[toothCurrentCount-2]; }
        */
        revolutionTime = (toothOneTime - toothOneMinusOneTime); //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
        toothTime = (toothLastToothTime - toothLastMinusOneToothTime); //Note that trigger tooth angle changes between 129 and 332 depending on the last tooth that was seen
        interrupts();
        toothTime = toothTime * 36;
        tempRPM = ((unsigned long)tempToothAngle * 6000000UL) / toothTime;
      }
    }
    else {
      tempRPM = stdGetRPM(scaleCrankAngle(360));
    }
  }
  return tempRPM;
}


bigAngle_t getCrankAngle_Harley()
{
  //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  unsigned long tempToothLastToothTime;
  int tempToothCurrentCount;
  //Grab some variables that are used in the trigger code and assign them to temp variables.
  noInterrupts();
  tempToothCurrentCount = toothCurrentCount;
  tempToothLastToothTime = toothLastToothTime;
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
  interrupts();

  //Check if the last tooth seen was the reference tooth (Number 3). All others can be calculated, but tooth 3 has a unique angle
  bigAngle_t crankAngle;
  if ( (tempToothCurrentCount == 1) || (tempToothCurrentCount == 3) )
  {
    crankAngle = scaleCrankAngle(0 + configPage4.triggerAngle); //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
  }
  else {
    crankAngle = scaleCrankAngle(157 + configPage4.triggerAngle);
  }

  //Estimate the number of degrees travelled since the last tooth}
  elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
  crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

  if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
  if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
  if (crankAngle < 0) { crankAngle += scaleCrankAngle(360); }

  return crankAngle;
}

void triggerSetEndTeeth_Harley()
{
  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

//************************************************************************************************************************

/** 36-2-2-2 crank based trigger wheel.
* A crank based trigger with a nominal 36 teeth, but 6 of these removed in 3 groups of 2.
* 2 of these groups are located concurrently.
* Note: This decoder supports both the H4 version (13-missing-16-missing-1-missing) and the H6 version of 36-2-2-2 (19-missing-10-missing-1-missing).
* The decoder checks which pattern is selected in order to determine the tooth number
* Note: www.thefactoryfiveforum.com/attachment.php?attachmentid=34279&d=1412431418
* 
* @defgroup dec_36_2_2_2 36-2-2-2 Trigger wheel
* @{
*/
void triggerSetup_ThirtySixMinus222()
{
  triggerToothAngle = scaleCrankAngle(10); //The number of degrees that passes from tooth to tooth
  triggerActualTeeth = 30; //The number of physical teeth on the wheel. Doing this here saves us a calculation each time in the interrupt
  triggerFilterTime = (int)(1000000 / (MAX_RPM / 60 * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be disgarded as noise
  secondDerivEnabled = false;
  decoderIsSequential = false;
  checkSyncToothCount = (configPage4.triggerTeeth) >> 1; //50% of the total teeth.
  toothLastMinusOneToothTime = 0;
  toothCurrentCount = 0;
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  MAX_STALL_TIME = (3333UL * scaleCrankAngleDown(triggerToothAngle) * 2 ); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}

void triggerPri_ThirtySixMinus222()
{
   curTime = micros();
   curGap = curTime - toothLastToothTime;
   if ( curGap >= triggerFilterTime ) //Pulses should never be less than triggerFilterTime, so if they are it means a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
   {
     toothCurrentCount++; //Increment the tooth counter
     validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

     //Begin the missing tooth detection
     //If the time between the current tooth and the last is greater than 2x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after a gap
     //toothSystemCount is used to keep track of which missed tooth we're on. It will be set to 1 if that last tooth seen was the middle one in the -2-2 area. At all other times it will be 0
     if(toothSystemCount == 0) { targetGap = ((toothLastToothTime - toothLastMinusOneToothTime)) * 2; } //Multiply by 2 (Checks for a gap 2x greater than the last one)


     if( (toothLastToothTime == 0) || (toothLastMinusOneToothTime == 0) ) { curGap = 0; }

     if ( (curGap > targetGap) )
     {
       {
         if(toothSystemCount == 1)
         {
           //This occurs when we're at the first tooth after the 2 lots of 2x missing tooth.
           toothCurrentCount = 19;
           toothSystemCount = 0;
           currentStatus.hasSync = true;
         }
         else
         {
           //We've seen a missing tooth set, but do not yet know whether it is the single one or the double one.
           toothSystemCount = 1;
           toothCurrentCount++;
           toothCurrentCount++; //Accurately reflect the actual tooth count, including the skipped ones
         }
         triggerToothAngleIsCorrect = false; //The tooth angle is double at this point
         triggerFilterTime = 0; //This is used to prevent a condition where serious intermitent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
       }
     }
     else
     {
       if(toothCurrentCount > 36)
       {
         //Means a complete rotation has occured.
         toothCurrentCount = 1;
         revolutionOne = !revolutionOne; //Flip sequential revolution tracker
         toothOneMinusOneTime = toothOneTime;
         toothOneTime = curTime;
         currentStatus.startRevolutions++; //Counter

       }
       else if(toothSystemCount == 1)
       {
         //This occurs when a set of missing teeth had been seen, but the next one was NOT missing.
         toothCurrentCount = 35;
         currentStatus.hasSync = true;
       }

       //Filter can only be recalc'd for the regular teeth, not the missing one.
       setFilter(curGap);

       triggerToothAngleIsCorrect = true;
       toothSystemCount = 0;
     }

     toothLastMinusOneToothTime = toothLastToothTime;
     toothLastToothTime = curTime;

     //EXPERIMENTAL!
     if(configPage2.perToothIgn == true)
     {
       bigAngle_t crankAngle = ( (toothCurrentCount-1) * triggerToothAngle ) + scaleCrankAngle(configPage4.triggerAngle);
       crankAngle = ignitionLimits(crankAngle);
       checkPerToothTiming(crankAngle, toothCurrentCount);
     }

   }
}

void triggerSec_ThirtySixMinus222()
{
  //NOT USED - This pattern uses the missing tooth version of this function
}

uint16_t getRPM_ThirtySixMinus222()
{
  uint16_t tempRPM = 0;
  if( currentStatus.RPM < currentStatus.crankRPM)
  {
    
    if( (configPage2.nCylinders == 4) && (toothCurrentCount != 19) && (toothCurrentCount != 16) && (toothCurrentCount != 34) && (triggerToothAngleIsCorrect == true) )
    {
      tempRPM = crankingGetRPM(36, scaleCrankAngle(360));
    }
    else if( (configPage2.nCylinders == 6) && (toothCurrentCount != 9) && (toothCurrentCount != 12) && (toothCurrentCount != 33) && (triggerToothAngleIsCorrect == true) )
    {
      tempRPM = crankingGetRPM(36, scaleCrankAngle(360));
    }
    else { tempRPM = currentStatus.RPM; } //Can't do per tooth RPM if we're at tooth #1 as the missing tooth messes the calculation
  }
  else
  {
    tempRPM = stdGetRPM(scaleCrankAngle(360));
  }
  return tempRPM;
}

bigAngle_t getCrankAngle_ThirtySixMinus222()
{
    //NOT USED - This pattern uses the missing tooth version of this function
    return 0;
}

void triggerSetEndTeeth_ThirtySixMinus222()
{
  if(currentStatus.advance < scaleCrankAngle(10)) { ignition1EndTooth = 36; }
  else if(currentStatus.advance < scaleCrankAngle(20)) { ignition1EndTooth = 35; }
  else if(currentStatus.advance < scaleCrankAngle(30)) { ignition1EndTooth = 34; }
  else { ignition1EndTooth = 31; }

  if(currentStatus.advance < scaleCrankAngle(30)) { ignition2EndTooth = 16; }
  else { ignition2EndTooth = 13; }

  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

//************************************************************************************************************************

/** 36-2-1 / Mistsubishi 4B11 - A crank based trigger with a nominal 36 teeth, but with 1 single and 1 double missing tooth.
* @defgroup dec_36_2_1 36-2-1 For Mistsubishi 4B11
* @{
*/
void triggerSetup_ThirtySixMinus21()
{
  triggerToothAngle = scaleCrankAngle(10); //The number of degrees that passes from tooth to tooth
  triggerActualTeeth = 33; //The number of physical teeth on the wheel. Doing this here saves us a calculation each time in the interrupt. Not Used
  triggerFilterTime = (1000000 / (MAX_RPM / 60 * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be disgarded as noise
  secondDerivEnabled = false;
  decoderIsSequential = false;
  checkSyncToothCount = (configPage4.triggerTeeth) >> 1; //50% of the total teeth.
  toothLastMinusOneToothTime = 0;
  toothCurrentCount = 0;
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  MAX_STALL_TIME = (3333UL * scaleCrankAngleDown(triggerToothAngle) * 2 ); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}

void triggerPri_ThirtySixMinus21()
{
   curTime = micros();
   curGap = curTime - toothLastToothTime;
   if ( curGap >= triggerFilterTime ) //Pulses should never be less than triggerFilterTime, so if they are it means a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
   {
     toothCurrentCount++; //Increment the tooth counter
     validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

     //Begin the missing tooth detection
     //If the time between the current tooth and the last is greater than 2x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after a gap
    
     targetGap2 = (3 * (toothLastToothTime - toothLastMinusOneToothTime)) ; //Multiply by 3 (Checks for a gap 3x greater than the last one)
     targetGap = targetGap2 >> 1;  //Multiply by 1.5 (Checks for a gap 1.5x greater than the last one) (Uses bitshift to divide by 2 as in the missing tooth deocder)

     if( (toothLastToothTime == 0) || (toothLastMinusOneToothTime == 0) ) { curGap = 0; }

     if ( (curGap > targetGap) )
     {
      if ( (curGap < targetGap2))
       {
           //we are at the tooth after the single gap
           toothCurrentCount = 20; //it's either 19 or 20, need to clarify engine direction!
           currentStatus.hasSync = true;
        }
        else 
        {
          //we are at the tooth after the double gap
          toothCurrentCount = 1; 
          currentStatus.hasSync = true;
        }
 
         triggerToothAngleIsCorrect = false; //The tooth angle is double at this point
         triggerFilterTime = 0; //This is used to prevent a condition where serious intermitent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
       }
     }
     else
     {
       if(  (toothCurrentCount > 36) || ( toothCurrentCount==1)  )
       {
         //Means a complete rotation has occured.
         toothCurrentCount = 1;
         revolutionOne = !revolutionOne; //Flip sequential revolution tracker
         toothOneMinusOneTime = toothOneTime;
         toothOneTime = curTime;
         currentStatus.startRevolutions++; //Counter

       }

       //Filter can only be recalc'd for the regular teeth, not the missing one.
       setFilter(curGap);

       triggerToothAngleIsCorrect = true;

     }

     toothLastMinusOneToothTime = toothLastToothTime;
     toothLastToothTime = curTime;

     //EXPERIMENTAL!
     if(configPage2.perToothIgn == true)
     {
       bigAngle_t crankAngle = ( (toothCurrentCount-1) * triggerToothAngle ) + scaleCrankAngle(configPage4.triggerAngle);
       crankAngle = ignitionLimits(crankAngle);
       checkPerToothTiming(crankAngle, toothCurrentCount);
     }

   
}

void triggerSec_ThirtySixMinus21()
{
  //NOT USED - This pattern uses the missing tooth version of this function
}

uint16_t getRPM_ThirtySixMinus21()
{
  uint16_t tempRPM = 0;
  if( currentStatus.RPM < currentStatus.crankRPM)
  {
    if( (toothCurrentCount != 20) && (triggerToothAngleIsCorrect == true) )
    {
      tempRPM = crankingGetRPM(36, scaleCrankAngle(360));
    }
    else { tempRPM = currentStatus.RPM; } //Can't do per tooth RPM if we're at tooth #1 as the missing tooth messes the calculation
  }
  else
  {
    tempRPM = stdGetRPM(scaleCrankAngle(360));
  }
  return tempRPM;
}

bigAngle_t getCrankAngle_ThirtySixMinus21()
{
    //NOT USED - This pattern uses the missing tooth version of this function
    return 0;
}

void triggerSetEndTeeth_ThirtySixMinus21()
{
  ignition1EndTooth = 10; 
  ignition2EndTooth = 28; // Arbritrarilly picked  at 180°.
 

  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

//************************************************************************************************************************

/** DSM 420a, For the DSM Eclipse with 16 teeth total on the crank.
* Tracks the falling side of the signal.
* Sync is determined by watching for a falling edge on the secondary signal and checking if the primary signal is high then.
* https://github.com/noisymime/speeduino/issues/133
* @defgroup dec_dsm_420a DSM 420a, For the DSM Eclipse
* @{
*/
void triggerSetup_420a()
{
  triggerFilterTime = (1000000 / (MAX_RPM / 60 * 360UL)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be disgarded as noise
  triggerSecFilterTime = 0;
  secondaryToothCount = 0; //Initially set to 0 prior to calculating the secondary window duration
  secondDerivEnabled = false;
  decoderIsSequential = true;
  toothCurrentCount = 1;
  triggerToothAngle = 20; //Is only correct for the 4 short pulses before each TDC
  triggerToothAngleIsCorrect = false;
  toothSystemCount = 0;
  MAX_STALL_TIME = (3333UL * 93); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)

  toothAngles[0] = 711; //tooth #1, just before #1 TDC
  toothAngles[1] = 111;
  toothAngles[2] = 131;
  toothAngles[3] = 151;
  toothAngles[4] = 171; //Just before #3 TDC
  toothAngles[5] = toothAngles[1] + 180;
  toothAngles[6] = toothAngles[2] + 180;
  toothAngles[7] = toothAngles[3] + 180;
  toothAngles[8] = toothAngles[4] + 180; //Just before #4 TDC
  toothAngles[9]  = toothAngles[1] + 360;
  toothAngles[10] = toothAngles[2] + 360;
  toothAngles[11] = toothAngles[3] + 360;
  toothAngles[12] = toothAngles[4] + 360; //Just before #2 TDC
  toothAngles[13] = toothAngles[1] + 540;
  toothAngles[14] = toothAngles[2] + 540;
  toothAngles[15] = toothAngles[3] + 540;
}

void triggerPri_420a()
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  if ( curGap >= triggerFilterTime ) //Pulses should never be less than triggerFilterTime, so if they are it means a false trigger. (A 36-1 wheel at 8000pm will have triggers approx. every 200uS)
  {
    toothCurrentCount++; //Increment the tooth counter
    validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

    if( (toothLastToothTime == 0) || (toothLastMinusOneToothTime == 0) ) { curGap = 0; }

    if( (toothCurrentCount > 16) && (currentStatus.hasSync == true) )
    {
      //Means a complete rotation has occured.
      toothCurrentCount = 1;
      toothOneMinusOneTime = toothOneTime;
      toothOneTime = curTime;
      currentStatus.startRevolutions++; //Counter
    }

    //Filter can only be recalc'd for the regular teeth, not the missing one.
    //setFilter(curGap);
    triggerFilterTime = 0;

    triggerToothAngleIsCorrect = false;

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    //EXPERIMENTAL!
    if(configPage2.perToothIgn == true)
    {
      int16_t crankAngle = ( toothAngles[(toothCurrentCount-1)] ) + scaleCrankAngle(configPage4.triggerAngle);
      crankAngle = ignitionLimits(crankAngle);
      checkPerToothTiming(crankAngle, toothCurrentCount);
    }
  }
}

void triggerSec_420a()
{
  //Secondary trigger is only on falling edge

  if(READ_PRI_TRIGGER() == true)
  {
    //Secondary signal is falling and primary signal is HIGH
    if( currentStatus.hasSync == false )
    {
      //If we don't have sync, then assume the signal is good
      toothCurrentCount = 13;
      currentStatus.hasSync = true;
    }
    else
    {
      //If we DO have sync, then check that the tooth count matches what we expect
      if(toothCurrentCount != 13)
      {
        currentStatus.syncLossCounter++;
        toothCurrentCount = 13;
      }
    }

  }
  else
  {
    //Secondary signal is falling and primary signal is LOW
    if( currentStatus.hasSync == false )
    {
      //If we don't have sync, then assume the signal is good
      toothCurrentCount = 5;
      currentStatus.hasSync = true;
    }
    else
    {
      //If we DO have sync, then check that the tooth count matches what we expect
      if(toothCurrentCount != 5)
      {
        currentStatus.syncLossCounter++;
        toothCurrentCount = 5;
      }
    }
  }
}

uint16_t getRPM_420a()
{
  uint16_t tempRPM = 0;
  if( currentStatus.RPM < currentStatus.crankRPM)
  {
    //Possibly look at doing special handling for cranking in the future, but for now just use the standard method
    tempRPM = stdGetRPM(720);
  }
  else
  {
    tempRPM = stdGetRPM(720);
  }
  return tempRPM;
}

bigAngle_t getCrankAngle_420a()
{
  //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
  unsigned long tempToothLastToothTime;
  int tempToothCurrentCount;
  //Grab some variables that are used in the trigger code and assign them to temp variables.
  noInterrupts();
  tempToothCurrentCount = toothCurrentCount;
  tempToothLastToothTime = toothLastToothTime;
  lastCrankAngleCalc = micros(); //micros() is no longer interrupt safe
  interrupts();

  bigAngle_t crankAngle;
  crankAngle = toothAngles[(tempToothCurrentCount - 1)] + scaleCrankAngle(configPage4.triggerAngle); //Perform a lookup of the fixed toothAngles array to find what the angle of the last tooth passed was.

  //Estimate the number of degrees travelled since the last tooth}
  elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
  crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

  if (crankAngle >= 720) { crankAngle -= 720; }
  if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
  if (crankAngle < 0) { crankAngle += 360; }

  return crankAngle;
}

void triggerSetEndTeeth_420a()
{
  if(currentStatus.advance < 9)
  {
    ignition1EndTooth = 1;
    ignition2EndTooth = 5;
    ignition3EndTooth = 9;
    ignition4EndTooth = 13;  
  }
  else
  {
    ignition1EndTooth = 16;
    ignition2EndTooth = 4;
    ignition3EndTooth = 8;
    ignition4EndTooth = 12;  
  }

  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

/** Weber-Marelli trigger setup with 2 wheels, 4 teeth 90deg apart on crank and 2 90deg apart on cam.
Uses DualWheel decoders, There can be no missing teeth on the primary wheel.
* @defgroup dec_weber_marelli Weber-Marelli
* @{
*/
void triggerPri_Webber()
{
  curTime = micros();
  curGap = curTime - toothLastToothTime;
  if ( curGap >= triggerFilterTime )
  {
    toothCurrentCount++; //Increment the tooth counter
    if (checkSyncToothCount > 0) { checkSyncToothCount++; }
    if ( triggerSecFilterTime <= curGap ) { triggerSecFilterTime = curGap + (curGap>>1); } //150% crank tooth
    validTrigger = true; //Flag this pulse as being a valid trigger (ie that it passed filters)

    toothLastMinusOneToothTime = toothLastToothTime;
    toothLastToothTime = curTime;

    if ( currentStatus.hasSync == true )
    {
      if ( (toothCurrentCount == 1) || (toothCurrentCount > configPage4.triggerTeeth) )
      {
        toothCurrentCount = 1;
        revolutionOne = !revolutionOne; //Flip sequential revolution tracker
        toothOneMinusOneTime = toothOneTime;
        toothOneTime = curTime;
        currentStatus.startRevolutions++; //Counter
      }

      setFilter(curGap); //Recalc the new filter value
    }
    else
    {
      if ( (secondaryToothCount == 1) && (checkSyncToothCount == 4) )
      {
        toothCurrentCount = 2;
        currentStatus.hasSync = true;
        revolutionOne = 0; //Sequential revolution reset
      }
    }

    //NEW IGNITION MODE
    if( (configPage2.perToothIgn == true) && (!BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) ) 
    {
      int16_t crankAngle = ( (toothCurrentCount-1) * triggerToothAngle ) + scaleCrankAngle(configPage4.triggerAngle);
      if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (revolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) )
      {
        crankAngle += 360;
        checkPerToothTiming(crankAngle, (configPage4.triggerTeeth + toothCurrentCount)); 
      }
      else{ checkPerToothTiming(crankAngle, toothCurrentCount); }
    }
  } //Trigger filter
}

void triggerSec_Webber()
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;

  if ( curGap2 >= triggerSecFilterTime )
  {
    toothLastSecToothTime = curTime2;

    if ( (secondaryToothCount == 2) && (checkSyncToothCount == 3) )
    {
      if(currentStatus.hasSync == false)
      {
        toothLastToothTime = micros();
        toothLastMinusOneToothTime = micros() - 1500000; //Fixes RPM at 10rpm until a full revolution has taken place
        toothCurrentCount = configPage4.triggerTeeth-1;

        currentStatus.hasSync = true;
      }
      else
      {
        if ( (toothCurrentCount != (configPage4.triggerTeeth-1U)) && (currentStatus.startRevolutions > 2U)) { currentStatus.syncLossCounter++; } //Indicates likely sync loss.
        if (configPage4.useResync == 1) { toothCurrentCount = configPage4.triggerTeeth-1; }
      }
      revolutionOne = 1; //Sequential revolution reset
      triggerSecFilterTime = curGap << 2; //4 crank teeth
      secondaryToothCount = 1; //Next tooth should be first
    } //Running, on first CAM pulse restart crank teet count, on second the counter should be 3
    else if ( (currentStatus.hasSync == false) && (toothCurrentCount >= 3) && (secondaryToothCount == 0) )
    {
      toothLastToothTime = micros();
      toothLastMinusOneToothTime = micros() - 1500000; //Fixes RPM at 10rpm until a full revolution has taken place
      toothCurrentCount = 1;
      revolutionOne = 1; //Sequential revolution reset

      currentStatus.hasSync = true;
    } //First start, between gaps on CAM pulses have 2 teeth, sync on first CAM pulse if seen 3 teeth or more
    else
    {
      triggerSecFilterTime = curGap + (curGap>>1); //150% crank tooth
      secondaryToothCount++;
      checkSyncToothCount = 1; //Tooth 1 considered as already been seen
    } //First time might fall here, second CAM tooth will
  }
  else
  {
    triggerSecFilterTime = curGap + (curGap>>1); //Noise region, using 150% of crank tooth
    checkSyncToothCount = 1; //Reset tooth counter
  } //Trigger filter
}
/** @} */

/** Ford ST170 - a dedicated decoder for 01-04 Ford Focus ST170/SVT engine.
Standard 36-1 trigger wheel running at crank speed and 8-3 trigger wheel running at cam speed.
* @defgroup dec_ford_st170 Ford ST170 (01-04 Focus)
* @{
*/
void triggerSetup_FordST170()
{
  //Set these as we are using the existing missing tooth primary decoder and these will never change.
  configPage4.triggerTeeth = 36;  
  configPage4.triggerMissingTeeth = 1;
  configPage4.TrigSpeed = CRANK_SPEED;

  triggerToothAngle = scaleCrankAngle(360) / configPage4.triggerTeeth; //The number of degrees that passes from tooth to tooth
  triggerActualTeeth = configPage4.triggerTeeth - configPage4.triggerMissingTeeth; //The number of physical teeth on the wheel. Doing this here saves us a calculation each time in the interrupt
  triggerFilterTime = (1000000 / (MAX_RPM / 60 * configPage4.triggerTeeth)); //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be disgarded as noise
  
  triggerSecFilterTime = 1000000 * 60 / MAX_RPM / 8 / 2; //Cam pattern is 8-3, so 2 nearest teeth are 90 deg crank angle apart. Cam can be advanced by 60 deg, so going from fully retarded to fully advanced closes the gap to 30 deg. Zetec cam pulleys aren't keyed from factory, so I substracted additional 10 deg to avoid filter to be too agressive. And there you have it 720/20=36.
  
  secondDerivEnabled = false;
  decoderIsSequential = true;
  checkSyncToothCount = (36) >> 1; //50% of the total teeth.
  toothLastMinusOneToothTime = 0;
  toothCurrentCount = 0;
  secondaryToothCount = 0; 
  toothOneTime = 0;
  toothOneMinusOneTime = 0;
  MAX_STALL_TIME = (3333UL * scaleCrankAngleDown(triggerToothAngle) * (1 + 1)); //Minimum 50rpm. (3333uS is the time per degree at 50rpm)
}

void triggerSec_FordST170()
{
  curTime2 = micros();
  curGap2 = curTime2 - toothLastSecToothTime;

  //Safety check for initial startup
  if( (toothLastSecToothTime == 0) )
  { 
    curGap2 = 0; 
    toothLastSecToothTime = curTime2;
  }

  if ( curGap2 >= triggerSecFilterTime )
  {
      targetGap2 = (3 * (toothLastSecToothTime - toothLastMinusOneSecToothTime)) >> 1; //If the time between the current tooth and the last is greater than 1.5x the time between the last tooth and the tooth before that, we make the assertion that we must be at the first tooth after the gap
      toothLastMinusOneSecToothTime = toothLastSecToothTime;
      if ( (curGap2 >= targetGap2) || (secondaryToothCount == 5) )
      {
        secondaryToothCount = 1;
        revolutionOne = 1; //Sequential revolution reset
        triggerSecFilterTime = 0; //This is used to prevent a condition where serious intermitent signals (Eg someone furiously plugging the sensor wire in and out) can leave the filter in an unrecoverable state
      }
      else
      {
        triggerSecFilterTime = curGap2 >> 2; //Set filter at 25% of the current speed. Filter can only be recalc'd for the regular teeth, not the missing one.
        secondaryToothCount++;
      }

    toothLastSecToothTime = curTime2;

    //Record the VVT Angle
    //We use the first tooth after the long gap as our reference, this remains in the same engine
    //cycle even when the VVT is at either end of its full swing.
    if( (configPage6.vvtEnabled > 0) && (revolutionOne == 1) && (secondaryToothCount == 1) )
    {
      int16_t curAngle;
      curAngle = getCrankAngle();
      while(curAngle > 360) { curAngle -= 360; }
      if( configPage6.vvtMode == VVT_MODE_CLOSED_LOOP )
      {
        curAngle = ANGLE_FILTER( (curAngle << 1), configPage4.ANGLEFILTER_VVT, curAngle);
        currentStatus.vvt1Angle = 360 - curAngle - configPage10.vvtCL0DutyAng;
      }
    }
  } //Trigger filter
}

uint16_t getRPM_FordST170()
{
  uint16_t tempRPM = 0;
  if( currentStatus.RPM < currentStatus.crankRPM )
  {
    if(toothCurrentCount != 1)
    {
      tempRPM = crankingGetRPM(36, scaleCrankAngle(360));
    }
    else { tempRPM = currentStatus.RPM; } //Can't do per tooth RPM if we're at tooth #1 as the missing tooth messes the calculation
  }
  else
  {
    tempRPM = stdGetRPM(scaleCrankAngle(360));
  }
  return tempRPM;
}

bigAngle_t getCrankAngle_FordST170()
{
    //This is the current angle ATDC the engine is at. This is the last known position based on what tooth was last 'seen'. It is only accurate to the resolution of the trigger wheel (Eg 36-1 is 10 degrees)
    unsigned long tempToothLastToothTime;
    int tempToothCurrentCount;
    bool tempRevolutionOne;
    //Grab some variables that are used in the trigger code and assign them to temp variables.
    noInterrupts();
    tempToothCurrentCount = toothCurrentCount;
    tempRevolutionOne = revolutionOne;
    tempToothLastToothTime = toothLastToothTime;
    interrupts();

    bigAngle_t crankAngle = ((tempToothCurrentCount - 1) * triggerToothAngle) + scaleCrankAngle(configPage4.triggerAngle); //Number of teeth that have passed since tooth 1, multiplied by the angle each tooth represents, plus the angle that tooth 1 is ATDC. This gives accuracy only to the nearest tooth.
    
    //Sequential check (simply sets whether we're on the first or 2nd revoltuion of the cycle)
    if ( (tempRevolutionOne == true) && (configPage4.TrigSpeed == CRANK_SPEED) ) { crankAngle += scaleCrankAngle(360); }

    lastCrankAngleCalc = micros();
    elapsedTime = (lastCrankAngleCalc - tempToothLastToothTime);
    crankAngle += timeToAngle(elapsedTime, CRANKMATH_METHOD_INTERVAL_REV);

    if (crankAngle >= scaleCrankAngle(720)) { crankAngle -= scaleCrankAngle(720); }
    else if (crankAngle > CRANK_ANGLE_MAX) { crankAngle -= CRANK_ANGLE_MAX; }
    if (crankAngle < 0) { crankAngle += CRANK_ANGLE_MAX; }

    return crankAngle;
}

void triggerSetEndTeeth_FordST170()
{
  byte toothAdder = 0;
   if( (configPage4.sparkMode == IGN_MODE_SEQUENTIAL) && (configPage4.TrigSpeed == CRANK_SPEED) ) { toothAdder = 36; }

  //Temp variables are used here to avoid potential issues if a trigger interrupt occurs part way through this function

  int16_t tempIgnition1EndTooth;
  tempIgnition1EndTooth = ( (ignition1EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition1EndTooth > (36 + toothAdder)) { tempIgnition1EndTooth -= (36 + toothAdder); }
  if(tempIgnition1EndTooth <= 0) { tempIgnition1EndTooth += (36 + toothAdder); }
  if((uint16_t)tempIgnition1EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition1EndTooth = (triggerActualTeeth + toothAdder); }
  ignition1EndTooth = tempIgnition1EndTooth;

  int16_t tempIgnition2EndTooth;
  tempIgnition2EndTooth = ( (ignition2EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition2EndTooth > (36 + toothAdder)) { tempIgnition2EndTooth -= (36 + toothAdder); }
  if(tempIgnition2EndTooth <= 0) { tempIgnition2EndTooth += (36 + toothAdder); }
  if((uint16_t)tempIgnition2EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition2EndTooth = (triggerActualTeeth + toothAdder); }
  ignition2EndTooth = tempIgnition2EndTooth;

  int16_t tempIgnition3EndTooth;
  tempIgnition3EndTooth = ( (ignition3EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition3EndTooth > (36 + toothAdder)) { tempIgnition3EndTooth -= (36 + toothAdder); }
  if(tempIgnition3EndTooth <= 0) { tempIgnition3EndTooth += (36 + toothAdder); }
  if((uint16_t)tempIgnition3EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition3EndTooth = (triggerActualTeeth + toothAdder); }
  ignition3EndTooth = tempIgnition3EndTooth;

  int16_t tempIgnition4EndTooth;
  tempIgnition4EndTooth = ( (ignition4EndAngle - scaleCrankAngle(configPage4.triggerAngle)) / (int16_t)(triggerToothAngle) ) - 1;
  if(tempIgnition4EndTooth > (36 + toothAdder)) { tempIgnition4EndTooth -= (36 + toothAdder); }
  if(tempIgnition4EndTooth <= 0) { tempIgnition4EndTooth += (36 + toothAdder); }
  if((uint16_t)tempIgnition4EndTooth > (triggerActualTeeth + toothAdder)) { tempIgnition4EndTooth = (triggerActualTeeth + toothAdder); }
  ignition4EndTooth = tempIgnition4EndTooth;
  // Removed ign channels >4 as an ST170 engine is a 4 cylinder

  lastToothCalcAdvance = currentStatus.advance;
}
/** @} */

