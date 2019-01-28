#if defined(CORE_STM32)
#include "board_stm32.h"
#include "globals.h"
#include "auxiliaries.h"
#include "idle.h"
#include "scheduler.h"
#include "HardwareTimer.h"
#if defined(STM32F4)
    //These should really be in the stm32GENERIC libs, but for somereason they only have timers 1-4
    #include <stm32_TIM_variant_11.h>
    HardwareTimer Timer5(TIM5, chip_tim5, sizeof(chip_tim5) / sizeof(chip_tim5[0]));
    HardwareTimer Timer8(TIM8, chip_tim8, sizeof(chip_tim8) / sizeof(chip_tim8[0]));
#endif

void initBoard()
{
    /*
    ***********************************************************************************************************
    * General
    */
   #define FLASH_LENGTH 8192


    /*
    ***********************************************************************************************************
    * Idle
    */
    if( (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_OL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_CL) )
    {
        idle_pwm_max_count = 1000000L / (configPage6.idleFreq * 2); //Converts the frequency in Hz to the number of ticks (at 2uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 5KHz
    } 

    //This must happen at the end of the idle init
    Timer1.setMode(4, TIMER_OUTPUT_COMPARE);
    //timer_set_mode(TIMER1, 4, TIMER_OUTPUT_COMPARE;
    if(idle_pwm_max_count > 0) { Timer1.attachInterrupt(4, idleInterrupt);} //on first flash the configPage4.iacAlgorithm is invalid
    Timer1.resume();


    /*
    ***********************************************************************************************************
    * Timers
    */
    #if defined(ARDUINO_BLACK_F407VE) || defined(STM32F4) || defined(_STM32F4_)
        Timer8.setPeriod(1000);  // Set up period
        Timer8.setMode(1, TIMER_OUTPUT_COMPARE);
        Timer8.attachInterrupt(1, oneMSInterval);
        Timer8.resume(); //Start Timer
    #else
        Timer4.setPeriod(1000);  // Set up period
        Timer4.setMode(1, TIMER_OUTPUT_COMPARE);
        Timer4.attachInterrupt(1, oneMSInterval);
        Timer4.resume(); //Start Timer
    #endif
    pinMode(LED_BUILTIN, OUTPUT); //Visual WDT

    /*
    ***********************************************************************************************************
    * Auxilliaries
    */
    //2uS resolution Min 8Hz, Max 5KHz
    boost_pwm_max_count = 1000000L / (2 * configPage6.boostFreq * 2); //Converts the frequency in Hz to the number of ticks (at 2uS) it takes to complete 1 cycle. The x2 is there because the frequency is stored at half value (in a byte) to allow freqneucies up to 511Hz
    vvt_pwm_max_count = 1000000L / (2 * configPage6.vvtFreq * 2); //Converts the frequency in Hz to the number of ticks (at 2uS) it takes to complete 1 cycle

    //Need to be initialised last due to instant interrupt
    Timer1.setMode(2, TIMER_OUTPUT_COMPARE);
    Timer1.setMode(3, TIMER_OUTPUT_COMPARE);
    if(boost_pwm_max_count > 0) { Timer1.attachInterrupt(2, boostInterrupt);}
    if(vvt_pwm_max_count > 0) { Timer1.attachInterrupt(3, vvtInterrupt);}
    Timer1.resume();

    /*
    ***********************************************************************************************************
    * Schedules
    */
    #if defined(ARDUINO_ARCH_STM32) // STM32GENERIC core
        //see https://github.com/rogerclarkmelbourne/Arduino_STM32/blob/754bc2969921f1ef262bd69e7faca80b19db7524/STM32F1/system/libmaple/include/libmaple/timer.h#L444
        Timer1.setPrescaleFactor((HAL_RCC_GetHCLKFreq() * 2U)-1);  //2us resolution
        Timer2.setPrescaleFactor((HAL_RCC_GetHCLKFreq() * 2U)-1);  //2us resolution
        Timer3.setPrescaleFactor((HAL_RCC_GetHCLKFreq() * 2U)-1);  //2us resolution
    #else //libmaple core aka STM32DUINO
        //see https://github.com/rogerclarkmelbourne/Arduino_STM32/blob/754bc2969921f1ef262bd69e7faca80b19db7524/STM32F1/system/libmaple/include/libmaple/timer.h#L444
        #if defined (STM32F1) || defined(__STM32F1__)
            //(CYCLES_PER_MICROSECOND == 72, APB2 at 72MHz, APB1 at 36MHz).
            //Timer2 to 4 is on APB1, Timer1 on APB2.   http://www.st.com/resource/en/datasheet/stm32f103cb.pdf sheet 12
            Timer1.setPrescaleFactor((72 * 2U)-1); //2us resolution
            Timer2.setPrescaleFactor((36 * 2U)-1); //2us resolution
            Timer3.setPrescaleFactor((36 * 2U)-1); //2us resolution
        #elif defined(STM32F4)
            //(CYCLES_PER_MICROSECOND == 168, APB2 at 84MHz, APB1 at 42MHz).
            //Timer2 to 14 is on APB1, Timers 1, 8, 9 and 10 on APB2.   http://www.st.com/resource/en/datasheet/stm32f407vg.pdf sheet 120
            Timer1.setPrescaleFactor((84 * 2U)-1); //2us resolution
            Timer2.setPrescaleFactor((42 * 2U)-1); //2us resolution
            Timer3.setPrescaleFactor((42 * 2U)-1); //2us resolution
        #endif
    #endif
    Timer2.setMode(1, TIMER_OUTPUT_COMPARE);
    Timer2.setMode(2, TIMER_OUTPUT_COMPARE);
    Timer2.setMode(3, TIMER_OUTPUT_COMPARE);
    Timer2.setMode(4, TIMER_OUTPUT_COMPARE);

    Timer3.setMode(1, TIMER_OUTPUT_COMPARE);
    Timer3.setMode(2, TIMER_OUTPUT_COMPARE);
    Timer3.setMode(3, TIMER_OUTPUT_COMPARE);
    Timer3.setMode(4, TIMER_OUTPUT_COMPARE);
    Timer1.setMode(1, TIMER_OUTPUT_COMPARE);

    //Attach interupt functions
    //Injection
    Timer2.attachInterrupt(1, fuelSchedule1Interrupt);
    Timer2.attachInterrupt(2, fuelSchedule2Interrupt);
    Timer2.attachInterrupt(3, fuelSchedule3Interrupt);
    Timer2.attachInterrupt(4, fuelSchedule4Interrupt);
    #if (INJ_CHANNELS >= 5)
    Timer5.attachInterrupt(1, fuelSchedule5Interrupt);
    #endif
    #if (INJ_CHANNELS >= 6)
    Timer5.attachInterrupt(2, fuelSchedule6Interrupt);
    #endif
    #if (INJ_CHANNELS >= 7)
    Timer5.attachInterrupt(3, fuelSchedule7Interrupt);
    #endif
    #if (INJ_CHANNELS >= 8)
    Timer5.attachInterrupt(4, fuelSchedule8Interrupt);
    #endif

    //Ignition
    #if (IGN_CHANNELS >= 1) 
    Timer3.attachInterrupt(1, ignitionSchedule1Interrupt); 
    #endif
    #if (IGN_CHANNELS >= 2)
    Timer3.attachInterrupt(2, ignitionSchedule2Interrupt);
    #endif
    #if (IGN_CHANNELS >= 3)
    Timer3.attachInterrupt(3, ignitionSchedule3Interrupt);
    #endif
    #if (IGN_CHANNELS >= 4)
    Timer3.attachInterrupt(4, ignitionSchedule4Interrupt);
    #endif
    #if (IGN_CHANNELS >= 5)
    Timer4.attachInterrupt(1, ignitionSchedule5Interrupt);
    #endif
    #if (IGN_CHANNELS >= 6)
    Timer4.attachInterrupt(2, ignitionSchedule6Interrupt);
    #endif
    #if (IGN_CHANNELS >= 7)
    Timer4.attachInterrupt(3, ignitionSchedule7Interrupt);
    #endif
    #if (IGN_CHANNELS >= 8)
    Timer4.attachInterrupt(4, ignitionSchedule8Interrupt);
    #endif

    Timer1.resume();
    Timer2.resume();
    Timer3.resume();
    #if (IGN_CHANNELS >= 5)
    Timer4.resume();
    #endif
    #if (INJ_CHANNELS >= 5)
    Timer5.resume();
    #endif
}

uint16_t freeRam()
{
    char top = 't';
    return &top - reinterpret_cast<char*>(sbrk(0));
}

#endif