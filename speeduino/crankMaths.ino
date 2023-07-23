#include "globals.h"
#include "crankMaths.h"
#include "decoders.h"
#include "timers.h"
#include "maths.h"

volatile uint16_t timePerDegree;
volatile uint16_t timePerDegreex16;
volatile uint16_t degreesPeruSx32768;

#define SECOND_DERIV_ENABLED                0          

//These are only part of the experimental 2nd deriv calcs
#if SECOND_DERIV_ENABLED!=0
byte deltaToothCount = 0; //The last tooth that was used with the deltaV calc
int rpmDelta;
#endif

uint32_t angleToTimeIntervalTooth(uint16_t angle) {
  noInterrupts();
  if(BIT_CHECK(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT))
  {
    unsigned long toothTime = (toothLastToothTime - toothLastMinusOneToothTime);
    uint16_t tempTriggerToothAngle = triggerToothAngle; // triggerToothAngle is set by interrupts
    interrupts();
    
    return (toothTime * (uint32_t)angle) / tempTriggerToothAngle;
  }
  //Safety check. This can occur if the last tooth seen was outside the normal pattern etc
  else { 
    interrupts();
    return angleToTimeIntervalRev(angle); 
  }
}


uint16_t timeToAngleIntervalTooth(uint32_t time)
{
    noInterrupts();
    //Still uses a last interval method (ie retrospective), but bases the interval on the gap between the 2 most recent teeth rather than the last full revolution
    if(BIT_CHECK(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT))
    {
      unsigned long toothTime = (toothLastToothTime - toothLastMinusOneToothTime);
      uint16_t tempTriggerToothAngle = triggerToothAngle; // triggerToothAngle is set by interrupts
      interrupts();

      return (unsigned long)(time * (uint32_t)tempTriggerToothAngle) / toothTime;
    }
    else { 
      interrupts();
      //Safety check. This can occur if the last tooth seen was outside the normal pattern etc
      return timeToAngleDegPerMicroSec(time);
    }
}

void doCrankSpeedCalcs(void)
{
     //********************************************************
      //How fast are we going? Need to know how long (uS) it will take to get from one tooth to the next. We then use that to estimate how far we are between the last tooth and the next one
      //We use a 1st Deriv acceleration prediction, but only when there is an even spacing between primary sensor teeth
      //Any decoder that has uneven spacing has its triggerToothAngle set to 0
      //THIS IS CURRENTLY DISABLED FOR ALL DECODERS! It needs more work. 
#if SECOND_DERIV_ENABLED!=0
      if( (BIT_CHECK(decoderState, BIT_DECODER_2ND_DERIV)) && (toothHistoryIndex >= 3) && (currentStatus.RPM < 2000) ) //toothHistoryIndex must be greater than or equal to 3 as we need the last 3 entries. Currently this mode only runs below 3000 rpm
      {
        //Only recalculate deltaV if the tooth has changed since last time (DeltaV stays the same until the next tooth)
        //if (deltaToothCount != toothCurrentCount)
        {
          deltaToothCount = toothCurrentCount;
          int angle1, angle2; //These represent the crank angles that are travelled for the last 2 pulses
          if(configPage4.TrigPattern == 4)
          {
            //Special case for 70/110 pattern on 4g63
            angle2 = triggerToothAngle; //Angle 2 is the most recent
            if (angle2 == 70) { angle1 = 110; }
            else { angle1 = 70; }
          }
          else if(configPage4.TrigPattern == 0)
          {
            //Special case for missing tooth decoder where the missing tooth was one of the last 2 seen
            if(toothCurrentCount == 1) { angle2 = 2*triggerToothAngle; angle1 = triggerToothAngle; }
            else if(toothCurrentCount == 2) { angle1 = 2*triggerToothAngle; angle2 = triggerToothAngle; }
            else { angle1 = triggerToothAngle; angle2 = triggerToothAngle; }
          }
          else { angle1 = triggerToothAngle; angle2 = triggerToothAngle; }

          uint32_t toothDeltaV = (1000000L * angle2 / toothHistory[toothHistoryIndex]) - (1000000L * angle1 / toothHistory[toothHistoryIndex-1]);
          uint32_t toothDeltaT = toothHistory[toothHistoryIndex];
          //long timeToLastTooth = micros() - toothLastToothTime;

          rpmDelta = (toothDeltaV << 10) / (6 * toothDeltaT);
        }

          timePerDegreex16 = ldiv( 2666656L, currentStatus.RPM + rpmDelta).quot; //This gives accuracy down to 0.1 of a degree and can provide noticeably better timing results on low resolution triggers
      }
      else
#endif
      {
        //If we can, attempt to get the timePerDegree by comparing the times of the last two teeth seen. This is only possible for evenly spaced teeth
        noInterrupts();
        if( (BIT_CHECK(decoderState, BIT_DECODER_TOOTH_ANG_CORRECT)) && (toothLastToothTime > toothLastMinusOneToothTime) && (abs(currentStatus.rpmDOT) > 30) )
        {
          //noInterrupts();
          unsigned long tempToothLastToothTime = toothLastToothTime;
          unsigned long tempToothLastMinusOneToothTime = toothLastMinusOneToothTime;
          uint16_t tempTriggerToothAngle = triggerToothAngle;
          interrupts();
          timePerDegreex16 = udiv_32_16((tempToothLastToothTime - tempToothLastMinusOneToothTime)*16UL, tempTriggerToothAngle);
        }
        else
        {
          //long timeThisRevolution = (micros_safe() - toothOneTime);
          interrupts();
          //Take into account any likely acceleration that has occurred since the last full revolution completed:
          //long rpm_adjust = (timeThisRevolution * (long)currentStatus.rpmDOT) / 1000000; 
          timePerDegreex16 = udiv_32_16( US_PER_DEG_PER_RPM*16U, max(MIN_RPM, (unsigned long)currentStatus.RPM)); 
        }
      }
      degreesPeruSx32768 = udiv_32_16(32768UL * 16UL, timePerDegreex16);
      timePerDegree = timePerDegreex16 / 16U;
}
