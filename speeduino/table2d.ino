/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
Because the size of the table is dynamic, this functino is required to reallocate the array sizes
Note that this may clear some of the existing values of the table
*/
#include "table2d.h"
#if !defined(UNIT_TEST)
#include "globals.h"
#endif


static inline uint8_t getCacheTime() {
#if !defined(UNIT_TEST)
  return currentStatus.secl;
#else
  return 0;
#endif
}

/*
This function pulls a 1D linear interpolated (ie averaged) value from a 2D table
ie: Given a value on the X axis, it returns a Y value that coresponds to the point on the curve between the nearest two defined X values

This function must take into account whether a table contains 8-bit or 16-bit values.
Unfortunately this means many of the lines are duplicated depending on this
*/
int table2D_getValue(struct table2D *fromTable, int X_in)
{
  //Orig memory usage = 5414
  int returnValue = 0;
  bool valueFound = false;

  int X = X_in;
  int xMinValue, xMaxValue;
  int xMin = 0;
  int xMax = fromTable->xSize-1;

  //Check whether the X input is the same as last time this ran
  if( (X_in == fromTable->lastInput) && (fromTable->cacheTime == getCacheTime()) )
  {
    returnValue = fromTable->lastOutput;
    valueFound = true;
  }
  //If the requested X value is greater/small than the maximum/minimum bin, simply return that value
  else if(X >= table2D_getAxisValue(fromTable, xMax))
  {
    returnValue = table2D_getRawValue(fromTable, xMax);
    valueFound = true;
  }
  else if(X <= table2D_getAxisValue(fromTable, xMin))
  {
    returnValue = table2D_getRawValue(fromTable, xMin);
    valueFound = true;
  }
  //Finally if none of that is found
  else
  {
    fromTable->cacheTime = getCacheTime(); //As we're not using the cache value, set the current secl value to track when this new value was calc'd

    //1st check is whether we're still in the same X bin as last time
    xMaxValue = table2D_getAxisValue(fromTable, fromTable->lastXMax);
    xMinValue = table2D_getAxisValue(fromTable, fromTable->lastXMin);
    if ( (X <= xMaxValue) && (X > xMinValue) )
    {
      xMax = fromTable->lastXMax;
      xMin = fromTable->lastXMin;
    }
    else
    {
      //If we're not in the same bin, loop through to find where we are
      xMaxValue = table2D_getAxisValue(fromTable, fromTable->xSize-1); // init xMaxValue in preparation for loop.
      for (int x = fromTable->xSize-1; x > 0; x--)
      {
        xMinValue = table2D_getAxisValue(fromTable, x-1); // fetch next Min

        //Checks the case where the X value is exactly what was requested
        if (X == xMaxValue)
        {
          returnValue = table2D_getRawValue(fromTable, x); //Simply return the coresponding value
          valueFound = true;
          break;
        }
        else if (X > xMinValue)
        {
          // Value is in the current bin
          xMax = x;
          fromTable->lastXMax = xMax;
          xMin = x-1;
          fromTable->lastXMin = xMin;
          break;
        }
        // Otherwise, continue to next bin
        xMaxValue = xMinValue; // for the next bin, our Min is their Max
      }
    }
  } //X_in same as last time

  if (valueFound == false)
  {
    int16_t m = X - xMinValue;
    int16_t n = xMaxValue - xMinValue;

    int16_t yMax = table2D_getRawValue(fromTable, xMax);
    int16_t yMin = table2D_getRawValue(fromTable, xMin);

    /* Float version (if m, yMax, yMin and n were float's)
       int yVal = (m * (yMax - yMin)) / n;
    */
    
    //Non-Float version
    int16_t yVal = ( ((int32_t) m) * (yMax-yMin) ) / n;
    returnValue = yMin + yVal;
  }

  fromTable->lastInput = X_in;
  fromTable->lastOutput = returnValue;

  return returnValue;
}

/**
 * @brief Returns an axis (bin) value from the 2D table. This works regardless of whether that axis is bytes or int16_ts
 * 
 * @param fromTable 
 * @param X_in 
 * @return int16_t 
 */
int16_t table2D_getAxisValue(struct table2D *fromTable, byte X_in)
{
  int returnValue = 0;

  if(fromTable->axisSize == SIZE_INT) { returnValue = ((int16_t*)fromTable->axisX)[X_in]; }
  else if(fromTable->axisSize == SIZE_BYTE) { returnValue = ((uint8_t*)fromTable->axisX)[X_in]; }

  return returnValue;
}

/**
 * @brief Returns an value from the 2D table given an index value. No interpolation is performed
 * 
 * @param fromTable 
 * @param X_index 
 * @return int16_t 
 */
int16_t table2D_getRawValue(struct table2D *fromTable, byte X_index)
{
  int returnValue = 0;

  if(fromTable->valueSize == SIZE_INT) { returnValue = ((int16_t*)fromTable->values)[X_index]; }
  else if(fromTable->valueSize == SIZE_BYTE) { returnValue = ((uint8_t*)fromTable->values)[X_index]; }

  return returnValue;
}