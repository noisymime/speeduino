#ifndef CRANKMATHS_H
#define CRANKMATHS_H

#include "maths.h"
#include "globals.h"

/**
 * @brief Makes one pass at nudging the angle to within [0,CRANK_ANGLE_MAX_IGN]
 * 
 * @param angle A crank angle in degrees
 * @return int16_t 
 */
static inline int16_t ignitionLimits(int16_t angle)
{
    return nudge(0, CRANK_ANGLE_MAX_IGN, angle, CRANK_ANGLE_MAX_IGN);
}

/**
 * @brief Makes one pass at nudging the angle to within [0,CRANK_ANGLE_MAX_INJ]
 * 
 * @param angle A crank angle in degrees
 * @return int16_t 
 */
static inline int16_t injectorLimits(int16_t angle)
{
    int16_t tempAngle = angle;
    if(tempAngle < 0) { tempAngle = tempAngle + CRANK_ANGLE_MAX_INJ; }
    while(tempAngle > CRANK_ANGLE_MAX_INJ ) { tempAngle -= CRANK_ANGLE_MAX_INJ; }
    return tempAngle;
}

/** @brief At 1 RPM, each degree of angular rotation takes this many microseconds */
#define MICROS_PER_DEG_1_RPM INT32_C(166667)

/** @brief The maximum rpm that the ECU will attempt to run at. 
 * 
 * It is NOT related to the rev limiter, but is instead dictates how fast certain operations will be
 * allowed to run. Lower number gives better performance 
 **/
#define MAX_RPM INT16_C(18000)

/** @brief Absolute minimum RPM that the crank math (& therefore all of Speeduino) can be used with 
 * 
 * This is dictated by the use of uint16_t as the base type for storing
 * angle<->time conversion factor (degreesPerMicro)
*/
#define MIN_RPM ((MICROS_PER_DEG_1_RPM/(UINT16_MAX/16UL))+1UL)

/**
 * @brief Set the revolution time, from which some of the degree<-->angle conversions are derived
 * 
 * @param revolutionTime The crank revolution time.
 */
void setAngleConverterRevolutionTime(uint32_t revolutionTime);

/**
 * @name Converts angular degrees to the time interval that amount of rotation
 * will take at current RPM.
 * 
 * Based on angle of [0,720] and min/max RPM, result ranges from
 * 9 (MAX_RPM, 1 deg) to 2926828 (MIN_RPM, 720 deg)
 *
 * @param angle Angle in degrees
 * @return Time interval in uS
 */
uint32_t angleToTimeMicroSecPerDegree(uint16_t angle);

/**
 * @name Converts a time interval in microsecods to the equivalent degrees of angular (crank)
 * rotation at current RPM.
 *
 * Inverse of angleToTimeMicroSecPerDegree
 *
 * @param time Time interval in uS
 * @return Angle in degrees
 */
uint16_t timeToAngleDegPerMicroSec(uint32_t time);

#endif