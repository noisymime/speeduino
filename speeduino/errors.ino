/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
 * Sets the next available error
 * Returns the error number or 0 if there is no more room for errors
 */
#include "globals.h"
#include "errors.h"

byte errorCount = 0;
byte errorCodes[4];

byte setError(byte errorID)
{
  if(errorCount < MAX_ERRORS)
  {
    errorCodes[errorCount] = errorID;
    errorCount++;
    if(errorCount == 1) { BIT_SET(currentStatus.spark, BIT_SPARK_ERROR); } //Enable the error indicator
  }
  return errorCount;
}

void clearError(byte errorID)
{
  byte clearedError = 255;

  if (errorID == errorCodes[0]) { clearedError = 0; }
  else if(errorID == errorCodes[1]) { clearedError = 1; }
  else if(errorID == errorCodes[2]) { clearedError = 2; }
  else if(errorID == errorCodes[3]) { clearedError = 3; }

  if(clearedError < MAX_ERRORS)
  {
    errorCodes[clearedError] = ERR_NONE;
    //Clear the required error and move any from above it 'down' in the error array
    for (byte x=clearedError; x < (errorCount-1); x++)
    {
      errorCodes[x] = errorCodes[x+1];
      errorCodes[x+1] = ERR_NONE;
    }

    errorCount--;
    if(errorCount == 0) { BIT_CLEAR(currentStatus.spark, BIT_SPARK_ERROR); } //Enable the error indicator
  }
}

byte getNextError(void)
{
  packedError currentError;
  byte currentErrorNum = 0;

  if(errorCount > 0)
  {
    //We alternate through the errors once per second
    currentErrorNum = currentStatus.secl % errorCount; //Which error number will be returned. This changes once per second. 

    currentError.errorNum = currentErrorNum;
    currentError.errorID = errorCodes[currentErrorNum];
  }
  else
  {
    currentError.errorNum = 0;
    currentError.errorID = 0;
  }


  return *(byte*)&currentError; //Ugly, but this forces the cast of the currentError struct to a byte.
}
