#include "schedule_calcs.h"

int ignition1StartAngle = 0;
int ignition2StartAngle = 0;
int ignition3StartAngle = 0;
int ignition4StartAngle = 0;
int ignition5StartAngle = 0;
int ignition6StartAngle = 0;
int ignition7StartAngle = 0;
int ignition8StartAngle = 0;

int channel1IgnDegrees = 0; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */
int channel2IgnDegrees = 0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
int channel3IgnDegrees = 0; /**< The number of crank degrees until cylinder 3 (and 5/6/7/8) is at TDC */
int channel4IgnDegrees = 0; /**< The number of crank degrees until cylinder 4 (and 5/6/7/8) is at TDC */
int channel5IgnDegrees = 0; /**< The number of crank degrees until cylinder 5 is at TDC */
int channel6IgnDegrees = 0; /**< The number of crank degrees until cylinder 6 is at TDC */
int channel7IgnDegrees = 0; /**< The number of crank degrees until cylinder 7 is at TDC */
int channel8IgnDegrees = 0; /**< The number of crank degrees until cylinder 8 is at TDC */
int channel1InjDegrees = 0; /**< The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones) */
int channel2InjDegrees = 0; /**< The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC */
int channel3InjDegrees = 0; /**< The number of crank degrees until cylinder 3 (and 5/6/7/8) is at TDC */
int channel4InjDegrees = 0; /**< The number of crank degrees until cylinder 4 (and 5/6/7/8) is at TDC */
int channel5InjDegrees = 0; /**< The number of crank degrees until cylinder 5 is at TDC */
int channel6InjDegrees = 0; /**< The number of crank degrees until cylinder 6 is at TDC */
int channel7InjDegrees = 0; /**< The number of crank degrees until cylinder 7 is at TDC */
int channel8InjDegrees = 0; /**< The number of crank degrees until cylinder 8 is at TDC */

