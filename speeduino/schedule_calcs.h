#pragma once

#include <stdint.h>
#include "scheduler.h"

extern int ignition1StartAngle;
extern int ignition2StartAngle;
extern int ignition3StartAngle;
extern int ignition4StartAngle;
extern int ignition5StartAngle;
extern int ignition6StartAngle;
extern int ignition7StartAngle;
extern int ignition8StartAngle;

extern int channel1IgnDegrees; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */
extern int channel2IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
extern int channel3IgnDegrees; /**< The number of crank degrees until cylinder 3 (and 5/6/7/8) is at TDC */
extern int channel4IgnDegrees; /**< The number of crank degrees until cylinder 4 (and 5/6/7/8) is at TDC */
extern int channel5IgnDegrees; /**< The number of crank degrees until cylinder 5 is at TDC */
extern int channel6IgnDegrees; /**< The number of crank degrees until cylinder 6 is at TDC */
extern int channel7IgnDegrees; /**< The number of crank degrees until cylinder 7 is at TDC */
extern int channel8IgnDegrees; /**< The number of crank degrees until cylinder 8 is at TDC */
extern int channel1InjDegrees; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */
extern int channel2InjDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
extern int channel3InjDegrees; /**< The number of crank degrees until cylinder 3 (and 5/6/7/8) is at TDC */
extern int channel4InjDegrees; /**< The number of crank degrees until cylinder 4 (and 5/6/7/8) is at TDC */
extern int channel5InjDegrees; /**< The number of crank degrees until cylinder 5 is at TDC */
extern int channel6InjDegrees; /**< The number of crank degrees until cylinder 6 is at TDC */
extern int channel7InjDegrees; /**< The number of crank degrees until cylinder 7 is at TDC */
extern int channel8InjDegrees; /**< The number of crank degrees until cylinder 8 is at TDC */

inline uint16_t __attribute__((always_inline)) calculateInjectorStartAngle(uint16_t PWdivTimerPerDegree, int16_t injChannelDegrees);

inline uint32_t __attribute__((always_inline)) calculateInjectorTimeout(const FuelSchedule &schedule, int channelInjDegrees, int injectorStartAngle, int crankAngle);

inline void __attribute__((always_inline)) calculateIgnitionAngle(const int dwellAngle, const uint16_t channelIgnDegrees, int *pEndAngle, int *pStartAngle);

// Ignition for rotary.
inline void calculateIgnitionTrailingRotary(int dwellAngle, int rotarySplitDegrees, int leadIgnitionAngle, int *pEndAngle, int *pStartAngle);

// ignition 3 for rotary
inline void __attribute__((always_inline)) calculateIgnitionAngle3(int dwellAngle, int rotarySplitDegrees);
// ignition 4 for rotary
inline void __attribute__((always_inline)) calculateIgnitionAngle4(int dwellAngle, int rotarySplitDegrees);

inline uint32_t __attribute__((always_inline)) calculateIgnitionTimeout(const Schedule &schedule, int startAngle, int channelIgnDegrees, int crankAngle);

#include "schedule_calcs.hpp"