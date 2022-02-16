#include "globals.h"
#include "crankMaths.h"
#include "decoders.h"
#include "timers.h"
#include "maths.h"

volatile uint16_t timePerDegree;
volatile uint16_t timePerDegreex16;
volatile uint16_t degreesPeruSx2048;
volatile unsigned long degreesPeruSx32768;

//These are only part of the experimental 2nd deriv calcs
byte deltaToothCount = 0; //The last tooth that was used with the deltaV calc
int rpmDelta;

/*
* Converts a crank angle into a time from or since that angle occurred.
* Positive angles are assumed to be in the future, negative angles in the past:
*   * Future angle calculations will use a predicted speed/acceleration
*   * Past angle calculations will use the known speed
* 
* Currently 4 methods are planned and/or available:
* 1) Last interval based on a full revolution
* 2) Last interval based on the time between the last 2 teeth (Crank Pattern dependant)
* 3) Closed loop error correction (Alpha-beta filter) 
* 4) 2nd derivative prediction (Speed + acceleration)
*/
unsigned long angleToTime(int16_t angle, byte method)
{
    unsigned long returnTime = 0;

    if( (method == CRANKMATH_METHOD_INTERVAL_REV) || (method == CRANKMATH_METHOD_INTERVAL_DEFAULT) )
    {
      #ifdef USE_LIBDIVIDE
        returnTime = libdivide::libdivide_u32_do(angle * revolutionTime, &libdiv_u32_360);
      #else
        returnTime = ((angle * revolutionTime) / 360);
      #endif
        //returnTime = angle * (unsigned long)timePerDegree;
    }
    else if (method == CRANKMATH_METHOD_INTERVAL_TOOTH)
    {
        //Still uses a last interval method (ie retrospective), but bases the interval on the gap between the 2 most recent teeth rather than the last full revolution
        if(triggerToothAngleIsCorrect == true)
        {
          noInterrupts();
          unsigned long toothTime = (toothLastToothTime - toothLastMinusOneToothTime);
          interrupts();
          
          returnTime = ( (toothTime * angle) / triggerToothAngle );
        }
        else { returnTime = angleToTime(angle, CRANKMATH_METHOD_INTERVAL_REV); } //Safety check. This can occur if the last tooth seen was outside the normal pattern etc
    }

    return returnTime;
}

/*
* Convert a time (uS) into an angle at current speed
* Currently 4 methods are planned and/or available:
* 1) Last interval based on a full revolution
* 2) Last interval based on the time between the last 2 teeth (Crank Pattern dependant)
* 3) Closed loop error correction (Alpha-beta filter) 
* 4) 2nd derivative prediction (Speed + acceleration)
*/
uint16_t timeToAngle(unsigned long time, byte method)
{
    uint16_t returnAngle = 0;

    if( (method == CRANKMATH_METHOD_INTERVAL_REV) || (method == CRANKMATH_METHOD_INTERVAL_DEFAULT) )
    {
        //A last interval method of calculating angle that does not take into account any acceleration. The interval used is the time taken to complete the last full revolution
        //degreesPeruSx2048 is the number of degrees the crank moves per uS. This value is almost always <1uS, so it is multiplied by 2048. This allows an angle calcuation with only a multiply and a bitshift without any appreciable drop in accuracy
        returnAngle = fastTimeToAngle(time); 
    }
    else if (method == CRANKMATH_METHOD_INTERVAL_TOOTH)
    {
        //Still uses a last interval method (ie retrospective), but bases the interval on the gap between the 2 most recent teeth rather than the last full revolution
        if(triggerToothAngleIsCorrect == true)
        {
          noInterrupts();
          unsigned long toothTime = (toothLastToothTime - toothLastMinusOneToothTime);
          interrupts();

          returnAngle = ( (unsigned long)(time * triggerToothAngle) / toothTime );
        }
        else { returnAngle = timeToAngle(time, CRANKMATH_METHOD_INTERVAL_REV); } //Safety check. This can occur if the last tooth seen was outside the normal pattern etc
    }
    else if (method == CRANKMATH_METHOD_ALPHA_BETA)
    {
        //Not yet implemented. Default to Rev
        returnAngle = timeToAngle(time, CRANKMATH_METHOD_INTERVAL_REV);
    }
    else if (method == CRANKMATH_METHOD_2ND_DERIVATIVE)
    {
        //Not yet implemented. Default to Rev
        returnAngle = timeToAngle(time, CRANKMATH_METHOD_INTERVAL_REV);
    }

   return returnAngle;
    
}

void doCrankSpeedCalcs()
{
     //********************************************************
      //How fast are we going? Need to know how long (uS) it will take to get from one tooth to the next. We then use that to estimate how far we are between the last tooth and the next one
      //We use a 1st Deriv accleration prediction, but only when there is an even spacing between primary sensor teeth
      //Any decoder that has uneven spacing has its triggerToothAngle set to 0
      //THIS IS CURRENTLY DISABLED FOR ALL DECODERS! It needs more work. 
      if( (secondDerivEnabled > 0) && (toothHistoryIndex >= 3) && (currentStatus.RPM < 2000) ) //toothHistoryIndex must be greater than or equal to 3 as we need the last 3 entries. Currently this mode only runs below 3000 rpm
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

          timePerDegreex16 = ldiv( 2666656L, currentStatus.RPM + rpmDelta).quot; //This give accuracy down to 0.1 of a degree and can provide noticably better timing results on low res triggers
          timePerDegree = timePerDegreex16 / 16;
      }
      else
      {
        //If we can, attempt to get the timePerDegree by comparing the times of the last two teeth seen. This is only possible for evenly spaced teeth
        noInterrupts();
        if( (triggerToothAngleIsCorrect == true) && (toothLastToothTime > toothLastMinusOneToothTime) && (abs(currentStatus.rpmDOT) > 30) )
        {
          //noInterrupts();
          unsigned long tempToothLastToothTime = toothLastToothTime;
          unsigned long tempToothLastMinusOneToothTime = toothLastMinusOneToothTime;
          uint16_t tempTriggerToothAngle = triggerToothAngle;
          interrupts();
          timePerDegreex16 = (unsigned long)( (tempToothLastToothTime - tempToothLastMinusOneToothTime)*16) / tempTriggerToothAngle;
          timePerDegree = timePerDegreex16 / 16;
        }
        else
        {
          //long timeThisRevolution = (micros_safe() - toothOneTime);
          interrupts();
          //Take into account any likely accleration that has occurred since the last full revolution completed:
          //long rpm_adjust = (timeThisRevolution * (long)currentStatus.rpmDOT) / 1000000; 
          long rpm_adjust = 0;
          timePerDegreex16 = ldiv( 2666656L, currentStatus.RPM + rpm_adjust).quot; //The use of a x16 value gives accuracy down to 0.1 of a degree and can provide noticably better timing results on low res triggers
          timePerDegree = timePerDegreex16 / 16;
        }
      }
      degreesPeruSx2048 = 2048 / timePerDegree;
      degreesPeruSx32768 = 524288 / timePerDegreex16;
}