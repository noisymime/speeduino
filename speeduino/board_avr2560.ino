
#if defined(CORE_AVR)
#include "globals.h"
#include "auxiliaries.h"

// Prescaler values for timers 1-3-4-5. Refer to www.instructables.com/files/orig/F3T/TIKL/H3WSA4V7/F3TTIKLH3WSA4V7.jpg
#define TIMER_PRESCALER_OFF  ((0<<CS12)|(0<<CS11)|(0<<CS10))
#define TIMER_PRESCALER_1    ((0<<CS12)|(0<<CS11)|(1<<CS10))
#define TIMER_PRESCALER_8    ((0<<CS12)|(1<<CS11)|(0<<CS10))
#define TIMER_PRESCALER_64   ((0<<CS12)|(1<<CS11)|(1<<CS10))
#define TIMER_PRESCALER_256  ((1<<CS12)|(0<<CS11)|(0<<CS10))
#define TIMER_PRESCALER_1024 ((1<<CS12)|(0<<CS11)|(1<<CS10))

#define TIMER_MODE_NORMAL    ((0<<WGM01)|(0<<WGM00))
#define TIMER_MODE_PWM       ((0<<WGM01)|(1<<WGM00))
#define TIMER_MODE_CTC       ((1<<WGM01)|(0<<WGM00))
#define TIMER_MODE_FASTPWM   ((1<<WGM01)|(1<<WGM00))

void initBoard()
{
    /*
    ***********************************************************************************************************
    * General
    */
    configPage9.intcan_available = 0;   // AVR devices do NOT have internal canbus

    /*
    ***********************************************************************************************************
    * Auxiliaries
    */
    //PWM used by the Boost and VVT outputs. C Channel is used by ign5
    TCCR1B = TIMER_PRESCALER_OFF;   //Disable Timer1 while we set it up
    TCNT1  = 0;                     //Reset Timer Count
    TCCR1A = TIMER_MODE_NORMAL;     //Timer1 Control Reg A: Wave Gen Mode normal (Simply counts up from 0 to 65535 (16-bit int)
    TCCR1B = TIMER_PRESCALER_256;   //Timer1 Control Reg B: Timer Prescaler set to 256. 1 tick = 16uS.
    TIFR1 = (1 << OCF1A) | (1<<OCF1B) | (1<<OCF1C) | (1<<TOV1) | (1<<ICF1); //Clear the compare flags, overflow flag and external input flag bits

    boost_pwm_max_count = 1000000L / (16 * configPage6.boostFreq * 2); //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle. The x2 is there because the frequency is stored at half value (in a byte) to allow frequencies up to 511Hz
    vvt_pwm_max_count = 1000000L / (16 * configPage6.vvtFreq * 2); //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle
    // put idle_pwm_max_count calculation here?

    /*
    ***********************************************************************************************************
    * Timers
    */
    //Configure Timer2 for our low-freq interrupt code.
    TCCR2B = TIMER_PRESCALER_OFF;   //Disable Timer2 while we set it up
    TCNT2  = 131;                   //Preload timer2 with 131 cycles, leaving 125 till overflow. As the timer runs at 125Khz, this causes overflow to occur at 1Khz = 1ms
    TIMSK2 = (1<<TOIE2);            //Timer2 Set Overflow Interrupt enabled.
    TCCR2A = TIMER_MODE_NORMAL;     //Timer2 Control Reg A: Wave Gen Mode normal
    /* Now configure the prescaler to CPU clock divided by 128 = 125Khz */
    TCCR2B = (1<<CS22)  | (1<<CS20); // Set bits. This timer uses different prescaler values, thus we cannot use the defines above.
    TIFR2 = (1 << OCF2A) | (1<<OCF2B) | (1<<TOV2); //Clear the compare flag bits and overflow flag bit

    //Enable the watchdog timer for 2 second resets (Good reference: www.tushev.org/articles/arduino/5/arduino-and-watchdog-timer)
    //Boooooooooo WDT is currently broken on Mega 2560 bootloaders :(
    //wdt_enable(WDTO_2S);

    /*
    ***********************************************************************************************************
    * Schedules
    * */
    //Much help in this from www.arduinomega.blogspot.com.au/2011/05/timer2-and-overflow-interrupt-lets-get.html
    //Fuel Schedules, which uses timer 3
    TCCR3B = TIMER_PRESCALER_OFF;   //Disable Timer3 while we set it up
    TCNT3  = 0;                     //Reset Timer Count
    TCCR3A = TIMER_MODE_NORMAL;     //Timer3 Control Reg A: Wave Gen Mode normal
    TCCR3B = TIMER_PRESCALER_64;    //Timer3 Control Reg B: Timer Prescaler set to 64.
    TIFR3 = (1 << OCF3A) | (1<<OCF3B) | (1<<OCF3C) | (1<<TOV3) | (1<<ICF3); //Clear the compare flags, overflow flag and external input flag bits

    //Ignition Schedules, which uses timer 5. This is also used by the fast version of micros(). If the speed of this timer is changed from 4uS ticks, that MUST be changed as well. See globals.h and timers.ino
    TCCR5B = TIMER_PRESCALER_OFF;   //Disable Timer5 while we set it up
    TCNT5  = 0;                     //Reset Timer Count
    TCCR5A = TIMER_MODE_NORMAL;     //Timer5 Control Reg A: Wave Gen Mode normal
    TCCR5B = TIMER_PRESCALER_64;    //Timer5 Control Reg B: Timer Prescaler set to 64.
    TIFR5 = (1 << OCF5A) | (1<<OCF5B) | (1<<OCF5C) | (1<<TOV5) | (1<<ICF5); //Clear the compare flags, overflow flag and external input flag bits
    
    #if defined(TIMER5_MICROS)
      TIMSK5 |= (1 << TOIE5); //Enable the timer5 overflow interrupt (See timers.ino for ISR)
      TIMSK0 &= ~_BV(TOIE0); // disable timer0 overflow interrupt
    #endif

    //The remaining Schedules (Fuel schedule 4 and ignition schedules 4 and 5) use Timer4
    TCCR4B = TIMER_PRESCALER_OFF;   //Disable Timer4 while we set it up
    TCNT4  = 0;                     //Reset Timer Count
    TCCR4A = TIMER_MODE_NORMAL;     //Timer4 Control Reg A: Wave Gen Mode normal
    TCCR4B = TIMER_PRESCALER_64;    //Timer4 Control Reg B: Timer Prescaler set to 64.
    TIFR4 = (1 << OCF4A) | (1<<OCF4B) | (1<<OCF4C) | (1<<TOV4) | (1<<ICF4); //Clear the compare flags, overflow flag and external input flag bits

}

/*
  Returns how much free dynamic memory exists (between heap and stack)
  This function is one big MISRA violation. MISRA advisories forbid directly poking at memory addresses, however there is no other way of determining heap size on embedded systems.
*/
uint16_t freeRam()
{
    extern int __heap_start, *__brkval;
    int currentVal;
    uint16_t v;

    if(__brkval == 0) { currentVal = (int) &__heap_start; }
    else { currentVal = (int) __brkval; }

    //Old version:
    //return (uint16_t) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
    /* cppcheck-suppress misra-c2012-11.4 ; DEVIATION(D3) */
    return (uint16_t) &v - currentVal; //cppcheck-suppress misra-c2012-11.4
}

void doSystemReset() { return; }
void jumpToBootloader() { return; }
#if defined(TIMER5_MICROS)
//This is used by the fast version of micros(). We just need to increment the timer overflow counter
ISR(TIMER5_OVF_vect)
{
    ++timer5_overflow_count;
}

static inline unsigned long micros_safe()
{
  unsigned long newMicros;
  noInterrupts();
  newMicros = (((timer5_overflow_count << 16) + TCNT5) * 4);
  interrupts();

  return newMicros;
} 
#endif //TIMER5_MICROS

#endif //CORE_AVR
