#ifndef STM32_H
#define STM32_H
#if defined(CORE_STM32_GENERIC)

/*
***********************************************************************************************************
* General
*/
  #define PORT_TYPE uint32_t
  #define PINMASK_TYPE uint32_t
  #define COMPARE_TYPE uint16_t
  #define COUNTER_TYPE uint16_t
  #define SERIAL_BUFFER_SIZE 517 //Size of the serial buffer used by new comms protocol. For SD transfers this must be at least 512 + 1 (flag) + 4 (sector)
  #define TIMER_RESOLUTION 2
  #define micros_safe() micros() //timer5 method is not used on anything but AVR, the micros_safe() macro is simply an alias for the normal micros()
  #if defined(SRAM_AS_EEPROM)
    #define EEPROM_LIB_H "src/BackupSram/BackupSramAsEEPROM.h"
  #elif defined(FRAM_AS_EEPROM) //https://github.com/VitorBoss/FRAM
    #define EEPROM_LIB_H <Fram.h>
    typedef uint16_t eeprom_address_t;
  #else
    #define EEPROM_LIB_H "src/SPIAsEEPROM/SPIAsEEPROM.h"
    typedef uint16_t eeprom_address_t;
  #endif

  #ifndef USE_SERIAL3
  #define USE_SERIAL3
  #endif
  void initBoard();
  uint16_t freeRam();
  void doSystemReset();
  void jumpToBootloader();

  #define pinIsReserved(pin)  ( ((pin) == PA11) || ((pin) == PA12) ) //Forbidden pins like USB

  #ifndef Serial
    #define Serial Serial1
  #endif

  #if defined(FRAM_AS_EEPROM)
    #include <Fram.h>
    #if defined(STM32F407xx)
    extern FramClass EEPROM; /*(mosi, miso, sclk, ssel, clockspeed) 31/01/2020*/
    #else
    extern FramClass EEPROM; //Blue/Black Pills
    #endif
  #endif

  #if !defined (A0)
    #define A0  PA0
    #define A1  PA1
    #define A2  PA2
    #define A3  PA3
    #define A4  PA4
    #define A5  PA5
    #define A6  PA6
    #define A7  PA7
    #define A8  PB0
    #define A9  PB1
  #endif
    //STM32F1 have only 10 12bit adc
  #if !defined (A10)
    #define A10  PA0
    #define A11  PA1
    #define A12  PA2
    #define A13  PA3
    #define A14  PA4
    #define A15  PA5
  #endif

  #ifndef PB11 //Hack for F4 BlackPills
    #define PB11 PB10
  #endif
#define PWM_FAN_AVAILABLE

/*
***********************************************************************************************************
* Schedules
* Timers Table for STM32F1
*   TIMER1    TIMER2    TIMER3    TIMER4
* 1 - FAN   1 - INJ1  1 - IGN1  1 - oneMSInterval
* 2 - BOOST 2 - INJ2  2 - IGN2  2 -
* 3 - VVT   3 - INJ3  3 - IGN3  3 -
* 4 - IDLE  4 - INJ4  4 - IGN4  4 -
*
* Timers Table for STM32F4
*   TIMER1    TIMER2    TIMER3    TIMER4    TIMER5    TIMER11
* 1 - FAN   1 - INJ1  1 - IGN1  1 - IGN5  1 - INJ5  1 - oneMSInterval
* 2 - BOOST 2 - INJ2  2 - IGN2  2 - IGN6  2 - INJ6  2 - 
* 3 - VVT   3 - INJ3  3 - IGN3  3 - IGN7  3 - INJ7  3 - 
* 4 - IDLE  4 - INJ4  4 - IGN4  4 - IGN8  4 - INJ8  4 - 
*
*/
  #define MAX_TIMER_PERIOD 65535*2 //The longest period of time (in uS) that the timer can permit (IN this case it is 65535 * 2, as each timer tick is 2uS)
  #define uS_TO_TIMER_COMPARE(uS) (uS >> 1) //Converts a given number of uS into the required number of timer ticks until that time has passed.

  #define FUEL1_COUNTER (TIM2)->CNT
  #define FUEL2_COUNTER (TIM2)->CNT
  #define FUEL3_COUNTER (TIM2)->CNT
  #define FUEL4_COUNTER (TIM2)->CNT

  #define FUEL1_COMPARE (TIM2)->CCR1
  #define FUEL2_COMPARE (TIM2)->CCR2
  #define FUEL3_COMPARE (TIM2)->CCR3
  #define FUEL4_COMPARE (TIM2)->CCR4

  #define IGN1_COUNTER  (TIM3)->CNT
  #define IGN2_COUNTER  (TIM3)->CNT
  #define IGN3_COUNTER  (TIM3)->CNT
  #define IGN4_COUNTER  (TIM3)->CNT

  #define IGN1_COMPARE (TIM3)->CCR1
  #define IGN2_COMPARE (TIM3)->CCR2
  #define IGN3_COMPARE (TIM3)->CCR3
  #define IGN4_COMPARE (TIM3)->CCR4

  #ifndef SMALL_FLASH_MODE
      #define FUEL5_COUNTER (TIM5)->CNT
      #define FUEL6_COUNTER (TIM5)->CNT
      #define FUEL7_COUNTER (TIM5)->CNT
      #define FUEL8_COUNTER (TIM5)->CNT

      #define FUEL5_COMPARE (TIM5)->CCR1
      #define FUEL6_COMPARE (TIM5)->CCR2
      #define FUEL7_COMPARE (TIM5)->CCR3
      #define FUEL8_COMPARE (TIM5)->CCR4

      #define IGN5_COUNTER  (TIM4)->CNT
      #define IGN6_COUNTER  (TIM4)->CNT
      #define IGN7_COUNTER  (TIM4)->CNT
      #define IGN8_COUNTER  (TIM4)->CNT

      #define IGN5_COMPARE (TIM4)->CCR1
      #define IGN6_COMPARE (TIM4)->CCR2
      #define IGN7_COMPARE (TIM4)->CCR3
      #define IGN8_COMPARE (TIM4)->CCR4
  #endif
  //github.com/rogerclarkmelbourne/Arduino_STM32/blob/754bc2969921f1ef262bd69e7faca80b19db7524/STM32F1/system/libmaple/include/libmaple/timer.h#L444
  #define FUEL1_TIMER_ENABLE() (TIM3)->SR = ~TIM_FLAG_CC1; (TIM3)->DIER |= TIM_DIER_CC1IE
  #define FUEL2_TIMER_ENABLE() (TIM3)->SR = ~TIM_FLAG_CC2; (TIM3)->DIER |= TIM_DIER_CC2IE
  #define FUEL3_TIMER_ENABLE() (TIM3)->SR = ~TIM_FLAG_CC3; (TIM3)->DIER |= TIM_DIER_CC3IE
  #define FUEL4_TIMER_ENABLE() (TIM3)->SR = ~TIM_FLAG_CC4; (TIM3)->DIER |= TIM_DIER_CC4IE

  #define FUEL1_TIMER_DISABLE() (TIM3)->DIER &= ~TIM_DIER_CC1IE
  #define FUEL2_TIMER_DISABLE() (TIM3)->DIER &= ~TIM_DIER_CC2IE
  #define FUEL3_TIMER_DISABLE() (TIM3)->DIER &= ~TIM_DIER_CC3IE
  #define FUEL4_TIMER_DISABLE() (TIM3)->DIER &= ~TIM_DIER_CC4IE

  #define IGN1_TIMER_ENABLE() (TIM2)->SR = ~TIM_FLAG_CC1; (TIM2)->DIER |= TIM_DIER_CC1IE
  #define IGN2_TIMER_ENABLE() (TIM2)->SR = ~TIM_FLAG_CC2; (TIM2)->DIER |= TIM_DIER_CC2IE
  #define IGN3_TIMER_ENABLE() (TIM2)->SR = ~TIM_FLAG_CC3; (TIM2)->DIER |= TIM_DIER_CC3IE
  #define IGN4_TIMER_ENABLE() (TIM2)->SR = ~TIM_FLAG_CC4; (TIM2)->DIER |= TIM_DIER_CC4IE

  #define IGN1_TIMER_DISABLE() (TIM2)->DIER &= ~TIM_DIER_CC1IE
  #define IGN2_TIMER_DISABLE() (TIM2)->DIER &= ~TIM_DIER_CC2IE
  #define IGN3_TIMER_DISABLE() (TIM2)->DIER &= ~TIM_DIER_CC3IE
  #define IGN4_TIMER_DISABLE() (TIM2)->DIER &= ~TIM_DIER_CC4IE

  #ifndef SMALL_FLASH_MODE
    #define FUEL5_TIMER_ENABLE() (TIM5)->SR = ~TIM_FLAG_CC1; (TIM5)->DIER |= TIM_DIER_CC1IE
    #define FUEL6_TIMER_ENABLE() (TIM5)->SR = ~TIM_FLAG_CC2; (TIM5)->DIER |= TIM_DIER_CC2IE
    #define FUEL7_TIMER_ENABLE() (TIM5)->SR = ~TIM_FLAG_CC3; (TIM5)->DIER |= TIM_DIER_CC3IE
    #define FUEL8_TIMER_ENABLE() (TIM5)->SR = ~TIM_FLAG_CC4; (TIM5)->DIER |= TIM_DIER_CC4IE

    #define FUEL5_TIMER_DISABLE() (TIM5)->DIER &= ~TIM_DIER_CC1IE
    #define FUEL6_TIMER_DISABLE() (TIM5)->DIER &= ~TIM_DIER_CC2IE
    #define FUEL7_TIMER_DISABLE() (TIM5)->DIER &= ~TIM_DIER_CC3IE
    #define FUEL8_TIMER_DISABLE() (TIM5)->DIER &= ~TIM_DIER_CC4IE

    #define IGN5_TIMER_ENABLE() (TIM4)->SR = ~TIM_FLAG_CC1; (TIM4)->DIER |= TIM_DIER_CC1IE
    #define IGN6_TIMER_ENABLE() (TIM4)->SR = ~TIM_FLAG_CC2; (TIM4)->DIER |= TIM_DIER_CC2IE
    #define IGN7_TIMER_ENABLE() (TIM4)->SR = ~TIM_FLAG_CC3; (TIM4)->DIER |= TIM_DIER_CC3IE
    #define IGN8_TIMER_ENABLE() (TIM4)->SR = ~TIM_FLAG_CC4; (TIM4)->DIER |= TIM_DIER_CC4IE

    #define IGN5_TIMER_DISABLE() (TIM4)->DIER &= ~TIM_DIER_CC1IE
    #define IGN6_TIMER_DISABLE() (TIM4)->DIER &= ~TIM_DIER_CC2IE
    #define IGN7_TIMER_DISABLE() (TIM4)->DIER &= ~TIM_DIER_CC3IE
    #define IGN8_TIMER_DISABLE() (TIM4)->DIER &= ~TIM_DIER_CC4IE
  #endif

/*
***********************************************************************************************************
* Auxiliaries
*/
  #define ENABLE_BOOST_TIMER()  (TIM1)->SR = ~TIM_FLAG_CC2; (TIM1)->DIER |= TIM_DIER_CC2IE
  #define DISABLE_BOOST_TIMER() (TIM1)->DIER &= ~TIM_DIER_CC2IE

  #define ENABLE_VVT_TIMER()    (TIM1)->SR = ~TIM_FLAG_CC3; (TIM1)->DIER |= TIM_DIER_CC3IE
  #define DISABLE_VVT_TIMER()   (TIM1)->DIER &= ~TIM_DIER_CC3IE

  #define ENABLE_FAN_TIMER()  (TIM1)->SR = ~TIM_FLAG_CC1; (TIM1)->DIER |= TIM_DIER_CC1IE
  #define DISABLE_FAN_TIMER() (TIM1)->DIER &= ~TIM_DIER_CC1IE

  #define BOOST_TIMER_COMPARE   (TIM1)->CCR2
  #define BOOST_TIMER_COUNTER   (TIM1)->CNT
  #define VVT_TIMER_COMPARE     (TIM1)->CCR3
  #define VVT_TIMER_COUNTER     (TIM1)->CNT
  #define FAN_TIMER_COMPARE     (TIM1)->CCR1
  #define FAN_TIMER_COUNTER     (TIM1)->CNT

/*
***********************************************************************************************************
* Idle
*/
  #define IDLE_COUNTER   (TIM1)->CNT
  #define IDLE_COMPARE   (TIM1)->CCR4

  #define IDLE_TIMER_ENABLE()  (TIM1)->SR = ~TIM_FLAG_CC4; (TIM1)->DIER |= TIM_DIER_CC4IE
  #define IDLE_TIMER_DISABLE() (TIM1)->DIER &= ~TIM_DIER_CC4IE
/*
***********************************************************************************************************
* Timers
*/


/*
***********************************************************************************************************
* CAN / Second serial
*/
#if defined(STM32GENERIC) // STM32GENERIC core
  SerialUART &CANSerial = Serial2;
#else //libmaple core aka STM32DUINO
  HardwareSerial &CANSerial = Serial2;
#endif

#endif //CORE_STM32
#endif //STM32_H
