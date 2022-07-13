

/*
NOTE - This file and it's associated functions need a CLEARER NAME

//Purpose
We're implementing a lower frequency interrupt loop to perform calculations that are needed less often, some of which depend on time having passed (delta/time) to be meaningful.


//Technical
Timer2 is only 8bit so we are setting the prescaler to 128 to get the most out of it. This means that the counter increments every 0.008ms and the overflow at 256 will be after 2.048ms
Max Period = (Prescale)*(1/Frequency)*(2^8)
(See arduinomega.blogspot.com.au/2011/05/timer2-and-overflow-interrupt-lets-get.html)

We're after a 1ms interval so we'll need 131 intervals to reach this ( 1ms / 0.008ms per tick = 125).
Hence we will preload the timer with 131 cycles to leave 125 until overflow (1ms).

*/
#ifndef TIMERS_H
#define TIMERS_H

#define SET_COMPARE(compare, value) compare = (COMPARE_TYPE)(value) // It is important that we cast this to the actual overflow limit of the timer. The compare variables type can be bigger than the timer overflow.

volatile bool tachoAlt = false;
#define TACHO_PULSE_HIGH() *tach_pin_port |= (tach_pin_mask)
#define TACHO_PULSE_LOW() *tach_pin_port &= ~(tach_pin_mask)
enum TachoOutputStatus {TACHO_INACTIVE, READY, ACTIVE}; //The 3 statuses that the tacho output pulse can have. NOTE: Cannot just use 'INACTIVE' as this is already defined within the Teensy Libs

volatile uint8_t tachoEndTime; //The time (in ms) that the tacho pulse needs to end at
volatile TachoOutputStatus tachoOutputFlag;

volatile byte loop33ms;
volatile byte loop66ms;
volatile byte loop100ms;
volatile byte loop250ms;
volatile int loopSec;

volatile unsigned int dwellLimit_uS;
volatile uint16_t lastRPM_100ms; //Need to record this for rpmDOT calculation
volatile uint16_t last250msLoopCount = 1000; //Set to effectively random number on startup. Just need this to be different to what mainLoopCount equals initially (Probably 0)

#if defined (CORE_TEENSY)
  IntervalTimer lowResTimer;
  void oneMSInterval();
#elif defined (ARDUINO_ARCH_STM32)
  void oneMSInterval();
#endif
void initialiseTimers();

#endif // TIMERS_H
