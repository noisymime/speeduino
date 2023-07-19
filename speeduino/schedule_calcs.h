#pragma once

#include <stdint.h>
#include "scheduler.h"

extern byte channelInjEnabled;

extern int ignition1StartAngle;
extern int ignition1EndAngle;
extern int channel1IgnDegrees; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */

extern int ignition2StartAngle;
extern int ignition2EndAngle;
extern int channel2IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */

extern int ignition3StartAngle;
extern int ignition3EndAngle;
extern int channel3IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */

extern int ignition4StartAngle;
extern int ignition4EndAngle;
extern int channel4IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */

#if (IGN_CHANNELS >= 5)
extern int ignition5StartAngle;
extern int ignition5EndAngle;
extern int channel5IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif
#if (IGN_CHANNELS >= 6)
extern int ignition6StartAngle;
extern int ignition6EndAngle;
extern int channel6IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif
#if (IGN_CHANNELS >= 7)
extern int ignition7StartAngle;
extern int ignition7EndAngle;
extern int channel7IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif
#if (IGN_CHANNELS >= 8)
extern int ignition8StartAngle;
extern int ignition8EndAngle;
extern int channel8IgnDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
#endif

extern int channel1InjDegrees; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */
extern int channel2InjDegrees; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
extern int channel3InjDegrees; /**< The number of crank degrees until cylinder 3 (and 5/6/7/8) is at TDC */
extern int channel4InjDegrees; /**< The number of crank degrees until cylinder 4 (and 5/6/7/8) is at TDC */
#if (INJ_CHANNELS >= 5)
extern int channel5InjDegrees; /**< The number of crank degrees until cylinder 5 is at TDC */
#endif
#if (INJ_CHANNELS >= 6)
extern int channel6InjDegrees; /**< The number of crank degrees until cylinder 6 is at TDC */
#endif
#if (INJ_CHANNELS >= 7)
extern int channel7InjDegrees; /**< The number of crank degrees until cylinder 7 is at TDC */
#endif
#if (INJ_CHANNELS >= 8)
extern int channel8InjDegrees; /**< The number of crank degrees until cylinder 8 is at TDC */
#endif

inline uint16_t __attribute__((always_inline)) calculateInjectorStartAngle(uint16_t PWdivTimerPerDegree, int16_t injChannelDegrees, uint16_t injAngle);

inline uint32_t __attribute__((always_inline)) calculateInjectorTimeout(const FuelSchedule &schedule, int channelInjDegrees, int injectorStartAngle, int crankAngle);

inline void __attribute__((always_inline)) calculateIgnitionAngle(const int dwellAngle, const uint16_t channelIgnDegrees, int8_t advance, int *pEndAngle, int *pStartAngle);

// Ignition for rotary.
inline void __attribute__((always_inline))  calculateIgnitionTrailingRotary(int dwellAngle, int rotarySplitDegrees, int leadIgnitionAngle, int *pEndAngle, int *pStartAngle);

inline uint32_t __attribute__((always_inline)) calculateIgnitionTimeout(const Schedule &schedule, int startAngle, int channelIgnDegrees, int crankAngle);

#include "schedule_calcs.hpp"