#ifndef STM32OFFICIAL_H
#define STM32OFFICIAL_H
#include <Arduino.h>
#if defined(STM32_CORE_VERSION_MAJOR)
#include <HardwareTimer.h>
#include <HardwareSerial.h>
#include "STM32RTC.h"
#include <SPI.h>

#if defined(STM32F1)
#include "stm32f1xx_ll_tim.h"
#elif defined(STM32F3)
#include "stm32f3xx_ll_tim.h"
#elif defined(STM32F4)
#include "stm32f4xx_ll_tim.h"
#else /*Default should be STM32F4*/
#include "stm32f4xx_ll_tim.h"
#endif

/*
***********************************************************************************************************
* General
*/
#define PORT_TYPE uint32_t
#define PINMASK_TYPE uint32_t
#define COMPARE_TYPE uint16_t
#define COUNTER_TYPE uint16_t
#define SERIAL_BUFFER_SIZE 517 //Size of the serial buffer used by new comms protocol. For SD transfers this must be at least 512 + 1 (flag) + 4 (sector)
#define FPU_MAX_SIZE 32 //Size of the FPU buffer. 0 means no FPU.
#define micros_safe() micros() //timer5 method is not used on anything but AVR, the micros_safe() macro is simply an alias for the normal micros()
#define TIMER_RESOLUTION 4

#if defined(USER_BTN) 
  #define EEPROM_RESET_PIN USER_BTN //onboard key0 for black STM32F407 boards and blackpills, keep pressed during boot to reset eeprom
#endif

#if defined(STM32F407xx)
  //Comment out this to disable SD logging for STM32 if needed. Currently SD logging for STM32 is experimental feature for F407.
  #define SD_LOGGING
#endif

#if defined SD_LOGGING
  #define RTC_ENABLED
  //SD logging with STM32 uses SD card in SPI mode, because used SD library doesn't support SDIO implementation. By default SPI3 is used that uses same pins as SDIO also, but in different order.
  extern SPIClass SD_SPI; //SPI3_MOSI, SPI3_MISO, SPI3_SCK
  #define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50), &SD_SPI)
  //Alternatively same SPI bus can be used as there is for SPI flash. But this is not recommended due to slower speed and other possible problems.
  //#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(50), &SPI_for_flash)
#endif

#define USE_SERIAL3

//When building for Black board Serial1 is instantiated, building generic STM32F4x7 has serial2 and serial 1 must be done here
#if SERIAL_UART_INSTANCE == 2
HardwareSerial Serial1(PA10, PA9);
#endif

#define RTC_LIB_H "STM32RTC.h"

extern STM32RTC& rtc;

void initBoard();
uint16_t freeRam();
void doSystemReset();
void jumpToBootloader();
extern "C" char* sbrk(int incr);

// Reserved pins for bluepill & blackpill boards
#if defined(ARDUINO_BLUEPILL_F103C8)  || defined(ARDUINO_BLUEPILL_F103CB) \
 || defined(ARDUINO_BLACKPILL_F401CC) || defined(ARDUINO_BLACKPILL_F401CE) \
 || defined(ARDUINO_BLACKPILL_F411CE)

  // PA11 USB_D+, PA12 USB_D-, PC13 LED_BUILTIN, PC14 OSC32_IN, PC15 OSC32_OUT, PB13 SCK2, PB14 MISO2, PB15 MOSI2. No support for onboard external flash on SPI1 because of ADC conflicts
  #ifdef USE_SPI_EEPROM
    #define pinIsReserved(pin)  ( ((pin) == PA11) || ((pin) == PA12) || ((pin) == PC13) || ((pin) == PC14) || ((pin) == PC15) || ((pin) == PB13) || ((pin) == PB14) || ((pin) == PB15) || ((pin) == USE_SPI_EEPROM))
  #else
    #define pinIsReserved(pin)  ( ((pin) == PA11) || ((pin) == PA12) || ((pin) == PC13) || ((pin) == PC14) || ((pin) == PC15) )
  #endif

  #ifndef PB11 //Hack for F4 BlackPills
    #define PB11 PB10
  #endif

  //Hack to allow compilation on small STM boards
  #ifndef A10
    #define A10  PA0
    #define A11  PA1
    #define A12  PA2
    #define A13  PA3
    #define A14  PA4
    #define A15  PA5
  #endif

// Reserved pins for other boards
#else
  #ifdef USE_SPI_EEPROM
    #define pinIsReserved(pin)  ( ((pin) == PA11) || ((pin) == PA12) || ((pin) == PB3) || ((pin) == PB4) || ((pin) == PB5) || ((pin) == USE_SPI_EEPROM) ) //Forbidden pins like USB
  #else
    #define pinIsReserved(pin)  ( ((pin) == PA11) || ((pin) == PA12) || ((pin) == PB3) || ((pin) == PB4) || ((pin) == PB5) || ((pin) == PB0) ) //Forbidden pins like USB
  #endif
#endif

#define PWM_FAN_AVAILABLE

#ifndef LED_BUILTIN
  #define LED_BUILTIN PA7
#endif

// Try to save space on devices with only 128k of internal flash
#if defined(STM32F103xB) || (defined(STM32F401xC) && !defined(ARDUINO_BLACKPILL_F401CC))  // F401CC has 256k flash but is matched by STM32F401xC
  #define SMALL_FLASH_MODE
#endif

/*
***********************************************************************************************************
* EEPROM emulation
*/
#if defined(SRAM_AS_EEPROM)
    #define EEPROM_LIB_H "src/BackupSram/BackupSramAsEEPROM.h"
    typedef uint16_t eeprom_address_t;
    #include EEPROM_LIB_H
    extern BackupSramAsEEPROM EEPROM;

#elif defined(USE_SPI_EEPROM)
    #define EEPROM_LIB_H "src/SPIAsEEPROM/SPIAsEEPROM.h"
    typedef uint16_t eeprom_address_t;
    #include EEPROM_LIB_H
    extern SPIClass SPI_for_flash; //SPI1_MOSI, SPI1_MISO, SPI1_SCK
 
    //windbond W25Q16 SPI flash EEPROM emulation
    extern EEPROM_Emulation_Config EmulatedEEPROMMconfig;
    extern Flash_SPI_Config SPIconfig;
    extern SPI_EEPROM_Class EEPROM;

#elif defined(FRAM_AS_EEPROM) //https://github.com/VitorBoss/FRAM
    #define EEPROM_LIB_H "src/FRAM/Fram.h"
    typedef uint16_t eeprom_address_t;
    #include EEPROM_LIB_H

    #if defined(STM32F407xx)
      extern FramClass EEPROM; /*(mosi, miso, sclk, ssel, clockspeed) 31/01/2020*/
    #else
      extern FramClass EEPROM; //Blue/Black Pills
    #endif

#else //default case, internal flash as EEPROM
  #define EEPROM_LIB_H "src/SPIAsEEPROM/SPIAsEEPROM.h"
  typedef uint16_t eeprom_address_t;
  #include EEPROM_LIB_H
  extern InternalSTM32F4_EEPROM_Class EEPROM;

#endif

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
*   TIMER1  |  TIMER2  |  TIMER3  |  TIMER4  |  TIMER5  |  TIMER11
* 1 - FAN  |1 - INJ1  |1 - IGN1  |1 - IGN5  |1 - INJ5  |1 - oneMSInterval
* 2 - BOOST |2 - INJ2  |2 - IGN2  |2 - IGN6  |2 - INJ6  |
* 3 - VVT   |3 - INJ3  |3 - IGN3  |3 - IGN7  |3 - INJ7  |
* 4 - IDLE  |4 - INJ4  |4 - IGN4  |4 - IGN8  |4 - INJ8  | 
*/
#define MAX_TIMER_PERIOD 65535*4 //The longest period of time (in uS) that the timer can permit (IN this case it is 65535 * 4, as each timer tick is 4uS)
#define uS_TO_TIMER_COMPARE(uS) (uS>>2) //Converts a given number of uS into the required number of timer ticks until that time has passed.

#define FUEL1_COUNTER (TIM3)->CNT
#define FUEL2_COUNTER (TIM3)->CNT
#define FUEL3_COUNTER (TIM3)->CNT
#define FUEL4_COUNTER (TIM3)->CNT

#define FUEL1_COMPARE (TIM3)->CCR1
#define FUEL2_COMPARE (TIM3)->CCR2
#define FUEL3_COMPARE (TIM3)->CCR3
#define FUEL4_COMPARE (TIM3)->CCR4

#define IGN1_COUNTER  (TIM2)->CNT
#define IGN2_COUNTER  (TIM2)->CNT
#define IGN3_COUNTER  (TIM2)->CNT
#define IGN4_COUNTER  (TIM2)->CNT

#define IGN1_COMPARE (TIM2)->CCR1
#define IGN2_COMPARE (TIM2)->CCR2
#define IGN3_COMPARE (TIM2)->CCR3
#define IGN4_COMPARE (TIM2)->CCR4

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

#define FUEL1_TIMER_ENABLE() (TIM3)->CR1 |= TIM_CR1_CEN; (TIM3)->SR = ~TIM_FLAG_CC1; (TIM3)->DIER |= TIM_DIER_CC1IE
#define FUEL2_TIMER_ENABLE() (TIM3)->CR1 |= TIM_CR1_CEN; (TIM3)->SR = ~TIM_FLAG_CC2; (TIM3)->DIER |= TIM_DIER_CC2IE
#define FUEL3_TIMER_ENABLE() (TIM3)->CR1 |= TIM_CR1_CEN; (TIM3)->SR = ~TIM_FLAG_CC3; (TIM3)->DIER |= TIM_DIER_CC3IE
#define FUEL4_TIMER_ENABLE() (TIM3)->CR1 |= TIM_CR1_CEN; (TIM3)->SR = ~TIM_FLAG_CC4; (TIM3)->DIER |= TIM_DIER_CC4IE

#define FUEL1_TIMER_DISABLE() (TIM3)->DIER &= ~TIM_DIER_CC1IE
#define FUEL2_TIMER_DISABLE() (TIM3)->DIER &= ~TIM_DIER_CC2IE
#define FUEL3_TIMER_DISABLE() (TIM3)->DIER &= ~TIM_DIER_CC3IE
#define FUEL4_TIMER_DISABLE() (TIM3)->DIER &= ~TIM_DIER_CC4IE

#define IGN1_TIMER_ENABLE() (TIM2)->CR1 |= TIM_CR1_CEN; (TIM2)->SR = ~TIM_FLAG_CC1; (TIM2)->DIER |= TIM_DIER_CC1IE
#define IGN2_TIMER_ENABLE() (TIM2)->CR1 |= TIM_CR1_CEN; (TIM2)->SR = ~TIM_FLAG_CC2; (TIM2)->DIER |= TIM_DIER_CC2IE
#define IGN3_TIMER_ENABLE() (TIM2)->CR1 |= TIM_CR1_CEN; (TIM2)->SR = ~TIM_FLAG_CC3; (TIM2)->DIER |= TIM_DIER_CC3IE
#define IGN4_TIMER_ENABLE() (TIM2)->CR1 |= TIM_CR1_CEN; (TIM2)->SR = ~TIM_FLAG_CC4; (TIM2)->DIER |= TIM_DIER_CC4IE

#define IGN1_TIMER_DISABLE() (TIM2)->DIER &= ~TIM_DIER_CC1IE
#define IGN2_TIMER_DISABLE() (TIM2)->DIER &= ~TIM_DIER_CC2IE
#define IGN3_TIMER_DISABLE() (TIM2)->DIER &= ~TIM_DIER_CC3IE
#define IGN4_TIMER_DISABLE() (TIM2)->DIER &= ~TIM_DIER_CC4IE

#define FUEL5_TIMER_ENABLE() (TIM5)->CR1 |= TIM_CR1_CEN; (TIM5)->CR1 |= TIM_CR1_CEN; (TIM5)->SR = ~TIM_FLAG_CC1; (TIM5)->DIER |= TIM_DIER_CC1IE
#define FUEL6_TIMER_ENABLE() (TIM5)->CR1 |= TIM_CR1_CEN; (TIM5)->CR1 |= TIM_CR1_CEN; (TIM5)->SR = ~TIM_FLAG_CC2; (TIM5)->DIER |= TIM_DIER_CC2IE
#define FUEL7_TIMER_ENABLE() (TIM5)->CR1 |= TIM_CR1_CEN; (TIM5)->CR1 |= TIM_CR1_CEN; (TIM5)->SR = ~TIM_FLAG_CC3; (TIM5)->DIER |= TIM_DIER_CC3IE
#define FUEL8_TIMER_ENABLE() (TIM5)->CR1 |= TIM_CR1_CEN; (TIM5)->CR1 |= TIM_CR1_CEN; (TIM5)->SR = ~TIM_FLAG_CC4; (TIM5)->DIER |= TIM_DIER_CC4IE

#define FUEL5_TIMER_DISABLE() (TIM5)->DIER &= ~TIM_DIER_CC1IE
#define FUEL6_TIMER_DISABLE() (TIM5)->DIER &= ~TIM_DIER_CC2IE
#define FUEL7_TIMER_DISABLE() (TIM5)->DIER &= ~TIM_DIER_CC3IE
#define FUEL8_TIMER_DISABLE() (TIM5)->DIER &= ~TIM_DIER_CC4IE

#define IGN5_TIMER_ENABLE() (TIM4)->CR1 |= TIM_CR1_CEN; (TIM4)->SR = ~TIM_FLAG_CC1; (TIM4)->DIER |= TIM_DIER_CC1IE
#define IGN6_TIMER_ENABLE() (TIM4)->CR1 |= TIM_CR1_CEN; (TIM4)->SR = ~TIM_FLAG_CC2; (TIM4)->DIER |= TIM_DIER_CC2IE
#define IGN7_TIMER_ENABLE() (TIM4)->CR1 |= TIM_CR1_CEN; (TIM4)->SR = ~TIM_FLAG_CC3; (TIM4)->DIER |= TIM_DIER_CC3IE
#define IGN8_TIMER_ENABLE() (TIM4)->CR1 |= TIM_CR1_CEN; (TIM4)->SR = ~TIM_FLAG_CC4; (TIM4)->DIER |= TIM_DIER_CC4IE

#define IGN5_TIMER_DISABLE() (TIM4)->DIER &= ~TIM_DIER_CC1IE
#define IGN6_TIMER_DISABLE() (TIM4)->DIER &= ~TIM_DIER_CC2IE
#define IGN7_TIMER_DISABLE() (TIM4)->DIER &= ~TIM_DIER_CC3IE
#define IGN8_TIMER_DISABLE() (TIM4)->DIER &= ~TIM_DIER_CC4IE

/*
***********************************************************************************************************
* Auxiliaries
*/
#define ENABLE_BOOST_TIMER()  (TIM1)->SR = ~TIM_FLAG_CC2; (TIM1)->DIER |= TIM_DIER_CC2IE; (TIM1)->CR1 |= TIM_CR1_CEN;  
#define DISABLE_BOOST_TIMER() (TIM1)->DIER &= ~TIM_DIER_CC2IE

#define ENABLE_VVT_TIMER()    (TIM1)->SR = ~TIM_FLAG_CC3; (TIM1)->DIER |= TIM_DIER_CC3IE; (TIM1)->CR1 |= TIM_CR1_CEN;  
#define DISABLE_VVT_TIMER()   (TIM1)->DIER &= ~TIM_DIER_CC3IE

#define ENABLE_FAN_TIMER()  (TIM1)->SR = ~TIM_FLAG_CC1; (TIM1)->DIER |= TIM_DIER_CC1IE; (TIM1)->CR1 |= TIM_CR1_CEN;  
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

#define IDLE_TIMER_ENABLE()  (TIM1)->SR = ~TIM_FLAG_CC4; (TIM1)->DIER |= TIM_DIER_CC4IE; (TIM1)->CR1 |= TIM_CR1_CEN;
#define IDLE_TIMER_DISABLE() (TIM1)->DIER &= ~TIM_DIER_CC4IE

/*
***********************************************************************************************************
* Timers
*/
extern HardwareTimer Timer1;
extern HardwareTimer Timer2;
extern HardwareTimer Timer3;
extern HardwareTimer Timer4;
#if !defined(STM32F103xB) // F103 has only 4 timers
extern HardwareTimer Timer5;
#if defined(TIM11)
extern HardwareTimer Timer11;
#elif defined(TIM7)
extern HardwareTimer Timer11;
#endif
#endif

#if ((STM32_CORE_VERSION_MINOR<=8) & (STM32_CORE_VERSION_MAJOR==1)) 
void oneMSInterval(HardwareTimer*);
void boostInterrupt(HardwareTimer*);
void fuelSchedule1Interrupt(HardwareTimer*);
void fuelSchedule2Interrupt(HardwareTimer*);
void fuelSchedule3Interrupt(HardwareTimer*);
void fuelSchedule4Interrupt(HardwareTimer*);
#if (INJ_CHANNELS >= 5)
void fuelSchedule5Interrupt(HardwareTimer*);
#endif
#if (INJ_CHANNELS >= 6)
void fuelSchedule6Interrupt(HardwareTimer*);
#endif
#if (INJ_CHANNELS >= 7)
void fuelSchedule7Interrupt(HardwareTimer*);
#endif
#if (INJ_CHANNELS >= 8)
void fuelSchedule8Interrupt(HardwareTimer*);
#endif
void idleInterrupt(HardwareTimer*);
void vvtInterrupt(HardwareTimer*);
void fanInterrupt(HardwareTimer*);
void ignitionSchedule1Interrupt(HardwareTimer*);
void ignitionSchedule2Interrupt(HardwareTimer*);
void ignitionSchedule3Interrupt(HardwareTimer*);
void ignitionSchedule4Interrupt(HardwareTimer*);
#if (IGN_CHANNELS >= 5)
void ignitionSchedule5Interrupt(HardwareTimer*);
#endif
#if (IGN_CHANNELS >= 6)
void ignitionSchedule6Interrupt(HardwareTimer*);
#endif
#if (IGN_CHANNELS >= 7)
void ignitionSchedule7Interrupt(HardwareTimer*);
#endif
#if (IGN_CHANNELS >= 8)
void ignitionSchedule8Interrupt(HardwareTimer*);
#endif
#endif //End core<=1.8

/*
***********************************************************************************************************
* CAN / Second serial
*/
#if HAL_CAN_MODULE_ENABLED
#define NATIVE_CAN_AVAILABLE
#include <src/STM32_CAN/STM32_CAN.h>
//This activates CAN1 interface on STM32, but it's named as Can0, because that's how Teensy implementation is done
extern STM32_CAN Can0;

static CAN_message_t outMsg;
static CAN_message_t inMsg;
#endif

#endif //CORE_STM32
#endif //STM32_H
