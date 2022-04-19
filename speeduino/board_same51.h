#ifndef SAME51_H
#define SAME51_H
#if defined(CORE_SAME51)

#include "sam.h"

/*
***********************************************************************************************************
* General
*/
  #define PORT_TYPE uint32_t //Size of the port variables (Eg inj1_pin_port). Most systems use a byte, but SAMD21 is a 32-bit unsigned int
  #define BOARD_MAX_DIGITAL_PINS 54 //digital pins +1
  #define BOARD_MAX_IO_PINS  58 //digital pins + analog channels + 1

  //#define PORT_TYPE uint8_t //Size of the port variables (Eg inj1_pin_port).
  #define PINMASK_TYPE uint8_t
  #define COMPARE_TYPE uint16_t
  #define COUNTER_TYPE uint16_t
  #define SERIAL_BUFFER_SIZE 257 //Size of the serial buffer used by new comms protocol. Additional 1 byte is for flag
  #ifdef USE_SPI_EEPROM
    #define EEPROM_LIB_H "src/SPIAsEEPROM/SPIAsEEPROM.h"
    typedef uint16_t eeprom_address_t;
    #include EEPROM_LIB_H
    //SPIClass SPI_for_flash(1, 2, 3); //SPI1_MOSI, SPI1_MISO, SPI1_SCK
    SPIClass SPI_for_flash = SPI; //SPI1_MOSI, SPI1_MISO, SPI1_SCK
 
    //windbond W25Q16 SPI flash EEPROM emulation
    EEPROM_Emulation_Config EmulatedEEPROMMconfig{255UL, 4096UL, 31, 0x00100000UL};
    //Flash_SPI_Config SPIconfig{USE_SPI_EEPROM, SPI_for_flash};
    SPI_EEPROM_Class EEPROM(EmulatedEEPROMMconfig, SPIconfig);
  #else
    //#define EEPROM_LIB_H <EEPROM.h>
    #define EEPROM_LIB_H "src/FlashStorage/FlashAsEEPROM.h"
    typedef uint16_t eeprom_address_t;
  #endif
  #define RTC_LIB_H "TimeLib.h"
  void initBoard();
  uint16_t freeRam();
  void doSystemReset();
  void jumpToBootloader();

  #if defined(TIMER5_MICROS)
    /*#define micros() (((timer5_overflow_count << 16) + TCNT5) * 4) */ //Fast version of micros() that uses the 4uS tick of timer5. See timers.ino for the overflow ISR of timer5
    #define millis() (ms_counter) //Replaces the standard millis() function with this macro. It is both faster and more accurate. See timers.ino for its counter increment.
    static inline unsigned long micros_safe(); //A version of micros() that is interrupt safe
  #else
    #define micros_safe() micros() //If the timer5 method is not used, the micros_safe() macro is simply an alias for the normal micros()
  #endif
  #define pinIsReserved(pin)  ( ((pin) == 0) ) //Forbidden pins like USB on other boards

  //Additional analog pins (These won't work without other changes)
  #define PIN_A6               (8ul)
  #define PIN_A7               (9ul)
  #define PIN_A8               (10ul)
  #define PIN_A9               (11ul)
  #define PIN_A13               (9ul)
  #define PIN_A14               (9ul)
  #define PIN_A15               (9ul)

  static const uint8_t A7  = PIN_A7;
  static const uint8_t A8  = PIN_A8;
  static const uint8_t A9  = PIN_A9;
  static const uint8_t A13  = PIN_A13;
  static const uint8_t A14  = PIN_A14;
  static const uint8_t A15  = PIN_A15;

/*
***********************************************************************************************************
* Schedules
*/
  //See : https://electronics.stackexchange.com/questions/325159/the-value-of-the-tcc-counter-on-an-atsam-controller-always-reads-as-zero
  // SAME512 Timer channel list: https://user-images.githubusercontent.com/11770912/62131781-2e150b80-b31f-11e9-9970-9a6c2356a17c.png
  #define FUEL1_COUNTER TCC0->COUNT.reg
  #define FUEL2_COUNTER TCC0->COUNT.reg
  #define FUEL3_COUNTER TCC0->COUNT.reg
  #define FUEL4_COUNTER TCC0->COUNT.reg
  //The below are NOT YET RIGHT!
  #define FUEL5_COUNTER TCC1->COUNT.reg
  #define FUEL6_COUNTER TCC1->COUNT.reg
  #define FUEL7_COUNTER TCC1->COUNT.reg
  #define FUEL8_COUNTER TCC1->COUNT.reg

  #define IGN1_COUNTER  TCC1->COUNT.reg
  #define IGN2_COUNTER  TCC1->COUNT.reg
  #define IGN3_COUNTER  TCC2->COUNT.reg
  #define IGN4_COUNTER  TCC2->COUNT.reg
  //The below are NOT YET RIGHT!
  #define IGN5_COUNTER  TCC1->COUNT.reg
  #define IGN6_COUNTER  TCC1->COUNT.reg
  #define IGN7_COUNTER  TCC2->COUNT.reg
  #define IGN8_COUNTER  TCC2->COUNT.reg

  #define FUEL1_COMPARE TCC0->CC[0].bit.CC
  #define FUEL2_COMPARE TCC0->CC[1].bit.CC
  #define FUEL3_COMPARE TCC0->CC[2].bit.CC
  #define FUEL4_COMPARE TCC0->CC[3].bit.CC
  //The below are NOT YET RIGHT!
  #define FUEL5_COMPARE TCC1->CC[0].bit.CC
  #define FUEL6_COMPARE TCC1->CC[1].bit.CC
  #define FUEL7_COMPARE TCC1->CC[2].bit.CC
  #define FUEL8_COMPARE TCC1->CC[3].bit.CC

  #define IGN1_COMPARE  TCC1->CC[0].bit.CC
  #define IGN2_COMPARE  TCC1->CC[1].bit.CC
  #define IGN3_COMPARE  TCC2->CC[0].bit.CC
  #define IGN4_COMPARE  TCC2->CC[1].bit.CC
  //The below are NOT YET RIGHT!
  #define IGN5_COMPARE  TCC1->CC[0].bit.CC
  #define IGN6_COMPARE  TCC1->CC[1].bit.CC
  #define IGN7_COMPARE  TCC2->CC[0].bit.CC
  #define IGN8_COMPARE  TCC2->CC[1].bit.CC

  #define FUEL1_TIMER_ENABLE() TCC0->INTENSET.bit.MC0 = 0x1
  #define FUEL2_TIMER_ENABLE() TCC0->INTENSET.bit.MC1 = 0x1
  #define FUEL3_TIMER_ENABLE() TCC0->INTENSET.bit.MC2 = 0x1
  #define FUEL4_TIMER_ENABLE() TCC0->INTENSET.bit.MC3 = 0x1
  //The below are NOT YET RIGHT!
  #define FUEL5_TIMER_ENABLE() TCC0->INTENSET.bit.MC0 = 0x1
  #define FUEL6_TIMER_ENABLE() TCC0->INTENSET.bit.MC1 = 0x1
  #define FUEL7_TIMER_ENABLE() TCC0->INTENSET.bit.MC2 = 0x1
  #define FUEL8_TIMER_ENABLE() TCC0->INTENSET.bit.MC3 = 0x1

  #define FUEL1_TIMER_DISABLE() TCC0->INTENSET.bit.MC0 = 0x0
  #define FUEL2_TIMER_DISABLE() TCC0->INTENSET.bit.MC0 = 0x0
  #define FUEL3_TIMER_DISABLE() TCC0->INTENSET.bit.MC0 = 0x0
  #define FUEL4_TIMER_DISABLE() TCC0->INTENSET.bit.MC0 = 0x0
  //The below are NOT YET RIGHT!
  #define FUEL5_TIMER_DISABLE() TCC0->INTENSET.bit.MC0 = 0x0
  #define FUEL6_TIMER_DISABLE() TCC0->INTENSET.bit.MC0 = 0x0
  #define FUEL7_TIMER_DISABLE() TCC0->INTENSET.bit.MC0 = 0x0
  #define FUEL8_TIMER_DISABLE() TCC0->INTENSET.bit.MC0 = 0x0

  #define IGN1_TIMER_ENABLE() TCC1->INTENSET.bit.MC0 = 0x1
  #define IGN2_TIMER_ENABLE() TCC1->INTENSET.bit.MC1 = 0x1
  #define IGN3_TIMER_ENABLE() TCC2->INTENSET.bit.MC0 = 0x1
  #define IGN4_TIMER_ENABLE() TCC2->INTENSET.bit.MC1 = 0x1
  //The below are NOT YET RIGHT!
  #define IGN5_TIMER_ENABLE() TCC1->INTENSET.bit.MC0 = 0x1
  #define IGN6_TIMER_ENABLE() TCC1->INTENSET.bit.MC1 = 0x1
  #define IGN7_TIMER_ENABLE() TCC2->INTENSET.bit.MC0 = 0x1
  #define IGN8_TIMER_ENABLE() TCC2->INTENSET.bit.MC1 = 0x1

  #define IGN1_TIMER_DISABLE() TCC1->INTENSET.bit.MC0 = 0x0
  #define IGN2_TIMER_DISABLE() TCC1->INTENSET.bit.MC1 = 0x0
  #define IGN3_TIMER_DISABLE() TCC2->INTENSET.bit.MC0 = 0x0
  #define IGN4_TIMER_DISABLE() TCC2->INTENSET.bit.MC1 = 0x0
  //The below are NOT YET RIGHT!
  #define IGN5_TIMER_DISABLE() TCC1->INTENSET.bit.MC0 = 0x0
  #define IGN6_TIMER_DISABLE() TCC1->INTENSET.bit.MC1 = 0x0
  #define IGN7_TIMER_DISABLE() TCC2->INTENSET.bit.MC0 = 0x0
  #define IGN8_TIMER_DISABLE() TCC2->INTENSET.bit.MC1 = 0x0

  #define MAX_TIMER_PERIOD 139808 // 2.13333333uS * 65535
  #define MAX_TIMER_PERIOD_SLOW 139808
  #define uS_TO_TIMER_COMPARE(uS) ((uS * 15) >> 5) //Converts a given number of uS into the required number of timer ticks until that time has passed.
  //Hack compatibility with AVR timers that run at different speeds
  #define uS_TO_TIMER_COMPARE_SLOW(uS) ((uS * 15) >> 5)

/*
***********************************************************************************************************
* Auxiliaries
*/
  //Uses the 2nd TC
  //The 2nd TC is referred to as TC4
  #define ENABLE_BOOST_TIMER()  TC4->COUNT16.INTENSET.bit.MC0 = 0x1    // Enable match interrupts on compare channel 0 
  #define DISABLE_BOOST_TIMER() TC4->COUNT16.INTENSET.bit.MC0 = 0x0

  #define ENABLE_VVT_TIMER()    TC4->COUNT16.INTENSET.bit.MC1 = 0x1 
  #define DISABLE_VVT_TIMER()   TC4->COUNT16.INTENSET.bit.MC1 = 0x0

  #define BOOST_TIMER_COMPARE   TC4->COUNT16.CC[0].reg
  #define BOOST_TIMER_COUNTER   TC4->COUNT16.COUNT.bit.COUNT
  #define VVT_TIMER_COMPARE     TC4->COUNT16.CC[1].reg
  #define VVT_TIMER_COUNTER     TC4->COUNT16.COUNT.bit.COUNT

/*
***********************************************************************************************************
* Idle
*/
  //3rd TC is aliased as TC5
  #define IDLE_COUNTER TC5->COUNT16.COUNT.bit.COUNT
  #define IDLE_COMPARE TC5->COUNT16.CC[0].reg

  #define IDLE_TIMER_ENABLE() TC5->COUNT16.INTENSET.bit.MC0 = 0x1
  #define IDLE_TIMER_DISABLE() TC5->COUNT16.INTENSET.bit.MC0 = 0x0

/*
***********************************************************************************************************
* CAN / Second serial
*/
  Uart CANSerial (&sercom3, 0, 1, SERCOM_RX_PAD_1, UART_TX_PAD_0);

#endif //CORE_SAMD21
#endif //SAMD21_H
