/** @file
Injector and Ignition (on/off) scheduling (structs).

This scheduler is designed to maintain 2 schedules for use by the fuel and ignition systems.
It functions by waiting for the overflow vectors from each of the timers in use to overflow, which triggers an interrupt.

## Technical

Currently I am prescaling the 16-bit timers to 256 for injection and 64 for ignition.
This means that the counter increments every 16us (injection) / 4uS (ignition) and will overflow every 1048576uS.

    Max Period = (Prescale)*(1/Frequency)*(2^17)

For more details see https://playground.arduino.cc/Code/Timer1/ (OLD: http://playground.arduino.cc/code/timer1 ).
This means that the precision of the scheduler is:

- 16uS (+/- 8uS of target) for fuel
- 4uS (+/- 2uS) for ignition

## Features

This differs from most other schedulers in that its calls are non-recurring (ie when you schedule an event at a certain time and once it has occurred,
it will not reoccur unless you explicitly ask/re-register for it).
Each timer can have only 1 callback associated with it at any given time. If you call the setCallback function a 2nd time,
the original schedule will be overwritten and not occur.

## Timer identification

Arduino timers usage for injection and ignition schedules:
- timer3 is used for schedule 1(?) (fuel 1,2,3,4 ign 7,8)
- timer4 is used for schedule 2(?) (fuel 5,6 ign 4,5,6)
- timer5 is used ... (fuel 7,8, ign 1,2,3)

Timers 3,4 and 5 are 16-bit timers (ie count to 65536).
See page 136 of the processors datasheet: http://www.atmel.com/Images/doc2549.pdf .

256 prescale gives tick every 16uS.
256 prescale gives overflow every 1048576uS (This means maximum wait time is 1.0485 seconds).

*/
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "globals.h"

#define USE_IGN_REFRESH
#define IGNITION_REFRESH_THRESHOLD  30 //Time in uS that the refresh functions will check to ensure there is enough time before changing the end compare

extern void (*inj1StartFunction)();
extern void (*inj1EndFunction)();
extern void (*inj2StartFunction)();
extern void (*inj2EndFunction)();
extern void (*inj3StartFunction)();
extern void (*inj3EndFunction)();
extern void (*inj4StartFunction)();
extern void (*inj4EndFunction)();
extern void (*inj5StartFunction)();
extern void (*inj5EndFunction)();
extern void (*inj6StartFunction)();
extern void (*inj6EndFunction)();
extern void (*inj7StartFunction)();
extern void (*inj7EndFunction)();
extern void (*inj8StartFunction)();
extern void (*inj8EndFunction)();

/** @name IgnitionCallbacks
 * These are the (global) function pointers that get called to begin and end the ignition coil charging.
 * They are required for the various spark output modes.
 * @{
*/
extern void (*ign1StartFunction)();
extern void (*ign1EndFunction)();
extern void (*ign2StartFunction)();
extern void (*ign2EndFunction)();
extern void (*ign3StartFunction)();
extern void (*ign3EndFunction)();
extern void (*ign4StartFunction)();
extern void (*ign4EndFunction)();
extern void (*ign5StartFunction)();
extern void (*ign5EndFunction)();
extern void (*ign6StartFunction)();
extern void (*ign6EndFunction)();
extern void (*ign7StartFunction)();
extern void (*ign7EndFunction)();
extern void (*ign8StartFunction)();
extern void (*ign8EndFunction)();
/** @} */

void initialiseSchedulers();
void beginInjectorPriming();
void setFuelSchedule1(unsigned long timeout, unsigned long duration);
void setFuelSchedule2(unsigned long timeout, unsigned long duration);
void setFuelSchedule3(unsigned long timeout, unsigned long duration);
void setFuelSchedule4(unsigned long timeout, unsigned long duration);
//void setFuelSchedule5(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)()); //Schedule 5 remains a special case for now due to the way it's implemented 
void setFuelSchedule5(unsigned long timeout, unsigned long duration);
void setFuelSchedule6(unsigned long timeout, unsigned long duration);
void setFuelSchedule7(unsigned long timeout, unsigned long duration);
void setFuelSchedule8(unsigned long timeout, unsigned long duration);
void setIgnitionSchedule1(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)());
void setIgnitionSchedule2(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)());
void setIgnitionSchedule3(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)());
void setIgnitionSchedule4(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)());
void setIgnitionSchedule5(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)());
void setIgnitionSchedule6(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)());
void setIgnitionSchedule7(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)());
void setIgnitionSchedule8(void (*startCallback)(), unsigned long timeout, unsigned long duration, void(*endCallback)());

inline void refreshIgnitionSchedule1(unsigned long timeToEnd) __attribute__((always_inline));

//The ARM cores use seprate functions for their ISRs
#if defined(ARDUINO_ARCH_STM32) || defined(CORE_TEENSY)
  static inline void fuelSchedule1Interrupt();
  static inline void fuelSchedule2Interrupt();
  static inline void fuelSchedule3Interrupt();
  static inline void fuelSchedule4Interrupt();
#if (INJ_CHANNELS >= 5)
  static inline void fuelSchedule5Interrupt();
#endif
#if (INJ_CHANNELS >= 6)
  static inline void fuelSchedule6Interrupt();
#endif
#if (INJ_CHANNELS >= 7)
  static inline void fuelSchedule7Interrupt();
#endif
#if (INJ_CHANNELS >= 8)
  static inline void fuelSchedule8Interrupt();
#endif
#if (IGN_CHANNELS >= 1)
  static inline void ignitionSchedule1Interrupt();
#endif
#if (IGN_CHANNELS >= 2)
  static inline void ignitionSchedule2Interrupt();
#endif
#if (IGN_CHANNELS >= 3)
  static inline void ignitionSchedule3Interrupt();
#endif
#if (IGN_CHANNELS >= 4)
  static inline void ignitionSchedule4Interrupt();
#endif
#if (IGN_CHANNELS >= 5)
  static inline void ignitionSchedule5Interrupt();
#endif
#if (IGN_CHANNELS >= 6)
  static inline void ignitionSchedule6Interrupt();
#endif
#if (IGN_CHANNELS >= 7)
  static inline void ignitionSchedule7Interrupt();
#endif
#if (IGN_CHANNELS >= 8)
  static inline void ignitionSchedule8Interrupt();
#endif
#endif
/** Schedule statuses.
 * - OFF - Schedule turned off and there is no scheduled plan
 * - PENDING - There's a scheduled plan, but is has not started to run yet
 * - STAGED - (???, Not used)
 * - RUNNING - Schedule is currently running
 */
enum ScheduleStatus {OFF, PENDING, STAGED, RUNNING}; //The statuses that a schedule can have

/** Ignition schedule.
 */
struct Schedule {
  volatile unsigned long duration;///< Scheduled duration (uS ?)
  volatile ScheduleStatus Status; ///< Schedule status: OFF, PENDING, STAGED, RUNNING
  volatile byte schedulesSet;     ///< A counter of how many times the schedule has been set
  void (*StartCallback)();        ///< Start Callback function for schedule
  void (*EndCallback)();          ///< End Callback function for schedule
  volatile unsigned long startTime; /**< The system time (in uS) that the schedule started, used by the overdwell protection in timers.ino */
  volatile COMPARE_TYPE startCompare; ///< The counter value of the timer when this will start
  volatile COMPARE_TYPE endCompare;   ///< The counter value of the timer when this will end

  COMPARE_TYPE nextStartCompare;      ///< Planned start of next schedule (when current schedule is RUNNING)
  COMPARE_TYPE nextEndCompare;        ///< Planned end of next schedule (when current schedule is RUNNING)
  volatile bool hasNextSchedule = false; ///< Enable flag for planned next schedule (when current schedule is RUNNING)
  volatile bool endScheduleSetByDecoder = false;
};
/** Fuel injection schedule.
* Fuel schedules don't use the callback pointers, or the startTime/endScheduleSetByDecoder variables.
* They are removed in this struct to save RAM.
*/
struct FuelSchedule {
  volatile unsigned long duration;///< Scheduled duration (uS ?)
  volatile ScheduleStatus Status; ///< Schedule status: OFF, PENDING, STAGED, RUNNING
  volatile byte schedulesSet; ///< A counter of how many times the schedule has been set
  volatile COMPARE_TYPE startCompare; ///< The counter value of the timer when this will start
  volatile COMPARE_TYPE endCompare;   ///< The counter value of the timer when this will end

  COMPARE_TYPE nextStartCompare;
  COMPARE_TYPE nextEndCompare;
  volatile bool hasNextSchedule = false;
};

//volatile Schedule *timer3Aqueue[4];
//Schedule *timer3Bqueue[4];
//Schedule *timer3Cqueue[4];

extern FuelSchedule fuelSchedule1;
extern FuelSchedule fuelSchedule2;
extern FuelSchedule fuelSchedule3;
extern FuelSchedule fuelSchedule4;
extern FuelSchedule fuelSchedule5;
extern FuelSchedule fuelSchedule6;
extern FuelSchedule fuelSchedule7;
extern FuelSchedule fuelSchedule8;

extern Schedule ignitionSchedule1;
extern Schedule ignitionSchedule2;
extern Schedule ignitionSchedule3;
extern Schedule ignitionSchedule4;
extern Schedule ignitionSchedule5;
extern Schedule ignitionSchedule6;
extern Schedule ignitionSchedule7;
extern Schedule ignitionSchedule8;

//IgnitionSchedule nullSchedule; //This is placed at the end of the queue. It's status will always be set to OFF and hence will never perform any action within an ISR

static inline COMPARE_TYPE setQueue(volatile Schedule *queue[], Schedule *schedule1, Schedule *schedule2, unsigned int CNT)
{
  //Create an array of all the upcoming targets, relative to the current count on the timer
  unsigned int tmpQueue[4];

  //Set the initial queue state. This order matches the tmpQueue order
  if(schedule1->Status == OFF)
  {
    queue[0] = schedule2;
    queue[1] = schedule2;
    tmpQueue[0] = schedule2->startCompare - CNT;
    tmpQueue[1] = schedule2->endCompare - CNT;
  }
  else
  {
    queue[0] = schedule1;
    queue[1] = schedule1;
    tmpQueue[0] = schedule1->startCompare - CNT;
    tmpQueue[1] = schedule1->endCompare - CNT;
  }

  if(schedule2->Status == OFF)
  {
    queue[2] = schedule1;
    queue[3] = schedule1;
    tmpQueue[2] = schedule1->startCompare - CNT;
    tmpQueue[3] = schedule1->endCompare - CNT;
  }
  else
  {
    queue[2] = schedule2;
    queue[3] = schedule2;
    tmpQueue[2] = schedule2->startCompare - CNT;
    tmpQueue[3] = schedule2->endCompare - CNT;
  }


  //Sort the queues. Both queues are kept in sync.
  //This implements a sorting networking based on the Bose-Nelson sorting network
  //See: pages.ripco.net/~jgamble/nw.html
  #define SWAP(x,y) if(tmpQueue[y] < tmpQueue[x]) { unsigned int tmp = tmpQueue[x]; tmpQueue[x] = tmpQueue[y]; tmpQueue[y] = tmp; volatile Schedule *tmpS = queue[x]; queue[x] = queue[y]; queue[y] = tmpS; }
  /*SWAP(0, 1); */ //Likely not needed
  /*SWAP(2, 3); */ //Likely not needed
  SWAP(0, 2);
  SWAP(1, 3);
  SWAP(1, 2);

  //Return the next compare time in the queue
  return tmpQueue[0] + CNT; //Return the
}

/*
 * Moves all the Schedules in a queue forward one position.
 * The current item (0) is discarded
 * The final queue slot is set to nullSchedule to indicate that no action should be taken
 */
static inline unsigned int popQueue(volatile Schedule *queue[])
{
  queue[0] = queue[1];
  queue[1] = queue[2];
  queue[2] = queue[3];
  //queue[3] = &nullSchedule;

  unsigned int returnCompare;
  if( queue[0]->Status == PENDING ) { returnCompare = queue[0]->startCompare; }
  else { returnCompare = queue[0]->endCompare; }

  return returnCompare;
}


#endif // SCHEDULER_H
