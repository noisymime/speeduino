#ifndef TEENSY35_H
#define TEENSY35_H
#if defined(CORE_TEENSY) && defined(CORE_TEENSY35)

/*
***********************************************************************************************************
* General
*/
  void initBoard();
  uint16_t freeRam();
  void doSystemReset();
  void jumpToBootloader();
  time_t getTeensy3Time();
  #define PORT_TYPE uint8_t //Size of the port variables
  #define PINMASK_TYPE uint8_t
  #define COMPARE_TYPE uint16_t
  #define COUNTER_TYPE uint16_t
  #define SERIAL_BUFFER_SIZE 517 //Size of the serial buffer used by new comms protocol. For SD transfers this must be at least 512 + 1 (flag) + 4 (sector)
  #define SD_LOGGING //SD logging enabled by default for Teensy 3.5 as it has the slot built in
  #define BOARD_MAX_DIGITAL_PINS 34
  #define BOARD_MAX_IO_PINS 34 //digital pins + analog channels + 1
  #ifdef USE_SPI_EEPROM
    #define EEPROM_LIB_H "src/SPIAsEEPROM/SPIAsEEPROM.h"
    typedef uint16_t eeprom_address_t;
  #else
    #define EEPROM_LIB_H <EEPROM.h>
    typedef int eeprom_address_t;
  #endif
  #define RTC_ENABLED
  #define RTC_LIB_H "TimeLib.h"
  #define SD_CONFIG  SdioConfig(FIFO_SDIO) //Set Teensy to use SDIO in FIFO mode. This is the fastest SD mode on Teensy as it offloads most of the writes

  #define micros_safe() micros() //timer5 method is not used on anything but AVR, the micros_safe() macro is simply an alias for the normal micros()
  #define PWM_FAN_AVAILABLE
  #define pinIsReserved(pin)  ( ((pin) == 0) || ((pin) == 1) || ((pin) == 3) || ((pin) == 4) ) //Forbidden pins like USB

/*
***********************************************************************************************************
* Schedules
*/
  //shawnhymel.com/661/learning-the-teensy-lc-interrupt-service-routines/
  #define FUEL1_COUNTER FTM0_CNT
  #define FUEL2_COUNTER FTM0_CNT
  #define FUEL3_COUNTER FTM0_CNT
  #define FUEL4_COUNTER FTM0_CNT
  #define FUEL5_COUNTER FTM3_CNT
  #define FUEL6_COUNTER FTM3_CNT
  #define FUEL7_COUNTER FTM3_CNT
  #define FUEL8_COUNTER FTM3_CNT

  #define IGN1_COUNTER  FTM0_CNT
  #define IGN2_COUNTER  FTM0_CNT
  #define IGN3_COUNTER  FTM0_CNT
  #define IGN4_COUNTER  FTM0_CNT
  #define IGN5_COUNTER  FTM3_CNT
  #define IGN6_COUNTER  FTM3_CNT
  #define IGN7_COUNTER  FTM3_CNT
  #define IGN8_COUNTER  FTM3_CNT

  #define FUEL1_COMPARE FTM0_C0V
  #define FUEL2_COMPARE FTM0_C1V
  #define FUEL3_COMPARE FTM0_C2V
  #define FUEL4_COMPARE FTM0_C3V
  #define FUEL5_COMPARE FTM3_C0V
  #define FUEL6_COMPARE FTM3_C1V
  #define FUEL7_COMPARE FTM3_C2V
  #define FUEL8_COMPARE FTM3_C3V

  #define IGN1_COMPARE  FTM0_C4V
  #define IGN2_COMPARE  FTM0_C5V
  #define IGN3_COMPARE  FTM0_C6V
  #define IGN4_COMPARE  FTM0_C7V
  #define IGN5_COMPARE  FTM3_C4V
  #define IGN6_COMPARE  FTM3_C5V
  #define IGN7_COMPARE  FTM3_C6V
  #define IGN8_COMPARE  FTM3_C7V

  #define FUEL1_TIMER_ENABLE() FTM0_C0SC |= FTM_CSC_CHIE //Write 1 to the CHIE (Channel Interrupt Enable) bit of channel 0 Status/Control
  #define FUEL2_TIMER_ENABLE() FTM0_C1SC |= FTM_CSC_CHIE
  #define FUEL3_TIMER_ENABLE() FTM0_C2SC |= FTM_CSC_CHIE
  #define FUEL4_TIMER_ENABLE() FTM0_C3SC |= FTM_CSC_CHIE
  #define FUEL5_TIMER_ENABLE() FTM3_C0SC |= FTM_CSC_CHIE
  #define FUEL6_TIMER_ENABLE() FTM3_C1SC |= FTM_CSC_CHIE
  #define FUEL7_TIMER_ENABLE() FTM3_C2SC |= FTM_CSC_CHIE
  #define FUEL8_TIMER_ENABLE() FTM3_C3SC |= FTM_CSC_CHIE

  #define FUEL1_TIMER_DISABLE() FTM0_C0SC &= ~FTM_CSC_CHIE //Write 0 to the CHIE (Channel Interrupt Enable) bit of channel 0 Status/Control
  #define FUEL2_TIMER_DISABLE() FTM0_C1SC &= ~FTM_CSC_CHIE
  #define FUEL3_TIMER_DISABLE() FTM0_C2SC &= ~FTM_CSC_CHIE
  #define FUEL4_TIMER_DISABLE() FTM0_C3SC &= ~FTM_CSC_CHIE
  #define FUEL5_TIMER_DISABLE() FTM3_C0SC &= ~FTM_CSC_CHIE //Write 0 to the CHIE (Channel Interrupt Enable) bit of channel 0 Status/Control
  #define FUEL6_TIMER_DISABLE() FTM3_C1SC &= ~FTM_CSC_CHIE
  #define FUEL7_TIMER_DISABLE() FTM3_C2SC &= ~FTM_CSC_CHIE
  #define FUEL8_TIMER_DISABLE() FTM3_C3SC &= ~FTM_CSC_CHIE

  #define IGN1_TIMER_ENABLE() FTM0_C4SC |= FTM_CSC_CHIE
  #define IGN2_TIMER_ENABLE() FTM0_C5SC |= FTM_CSC_CHIE
  #define IGN3_TIMER_ENABLE() FTM0_C6SC |= FTM_CSC_CHIE
  #define IGN4_TIMER_ENABLE() FTM0_C7SC |= FTM_CSC_CHIE
  #define IGN5_TIMER_ENABLE() FTM3_C4SC |= FTM_CSC_CHIE
  #define IGN6_TIMER_ENABLE() FTM3_C5SC |= FTM_CSC_CHIE
  #define IGN7_TIMER_ENABLE() FTM3_C6SC |= FTM_CSC_CHIE
  #define IGN8_TIMER_ENABLE() FTM3_C7SC |= FTM_CSC_CHIE

  #define IGN1_TIMER_DISABLE() FTM0_C4SC &= ~FTM_CSC_CHIE
  #define IGN2_TIMER_DISABLE() FTM0_C5SC &= ~FTM_CSC_CHIE
  #define IGN3_TIMER_DISABLE() FTM0_C6SC &= ~FTM_CSC_CHIE
  #define IGN4_TIMER_DISABLE() FTM0_C7SC &= ~FTM_CSC_CHIE
  #define IGN5_TIMER_DISABLE() FTM3_C4SC &= ~FTM_CSC_CHIE
  #define IGN6_TIMER_DISABLE() FTM3_C5SC &= ~FTM_CSC_CHIE
  #define IGN7_TIMER_DISABLE() FTM3_C6SC &= ~FTM_CSC_CHIE
  #define IGN8_TIMER_DISABLE() FTM3_C7SC &= ~FTM_CSC_CHIE

  #define MAX_TIMER_PERIOD 139808 // 2.13333333uS * 65535
  #define uS_TO_TIMER_COMPARE(uS) ((uS * 15) >> 5) //Converts a given number of uS into the required number of timer ticks until that time has passed.

/*
***********************************************************************************************************
* Auxiliaries
*/
  #define ENABLE_BOOST_TIMER()  FTM1_C0SC |= FTM_CSC_CHIE
  #define DISABLE_BOOST_TIMER() FTM1_C0SC &= ~FTM_CSC_CHIE

  #define ENABLE_VVT_TIMER()    FTM1_C1SC |= FTM_CSC_CHIE
  #define DISABLE_VVT_TIMER()   FTM1_C1SC &= ~FTM_CSC_CHIE
  
  #define ENABLE_FAN_TIMER()    FTM2_C1SC |= FTM_CSC_CHIE
  #define DISABLE_FAN_TIMER()   FTM2_C1SC &= ~FTM_CSC_CHIE

  #define BOOST_TIMER_COMPARE   FTM1_C0V
  #define BOOST_TIMER_COUNTER   FTM1_CNT
  #define VVT_TIMER_COMPARE     FTM1_C1V
  #define VVT_TIMER_COUNTER     FTM1_CNT
  #define FAN_TIMER_COMPARE     FTM2_C1V
  #define FAN_TIMER_COUNTER     FTM2_CNT

  void boostInterrupt();
  void vvtInterrupt();
  void fanInterrupt();

/*
***********************************************************************************************************
* Idle
*/
  #define IDLE_COUNTER FTM2_CNT
  #define IDLE_COMPARE FTM2_C0V

  #define IDLE_TIMER_ENABLE() FTM2_C0SC |= FTM_CSC_CHIE
  #define IDLE_TIMER_DISABLE() FTM2_C0SC &= ~FTM_CSC_CHIE

  void idleInterrupt();

/*
***********************************************************************************************************
* CAN / Second serial
*/
   #define USE_SERIAL3               // Secondary serial port to use
  #include <FlexCAN_T4.h>
#if defined(__MK64FX512__)         // use for Teensy 3.5 only 
  extern FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
#elif defined(__MK66FX1M0__)         // use for Teensy 3.6 only
  extern FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
  extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1; 
#endif
  static CAN_message_t outMsg;
  static CAN_message_t inMsg;
  #define NATIVE_CAN_AVAILABLE
#endif //CORE_TEENSY
#endif //TEENSY35_H
