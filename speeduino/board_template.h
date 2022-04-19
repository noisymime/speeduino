#ifndef TEMPLATE_H
#define TEMPLATE_H
#if defined(CORE_TEMPLATE)

/*
***********************************************************************************************************
* General
*/
  #define PORT_TYPE uint32_t //Size of the port variables (Eg inj1_pin_port). Most systems use a byte, but SAMD21 and possibly others are a 32-bit unsigned int
  #define PINMASK_TYPE uint32_t
  #define SERIAL_BUFFER_SIZE 517 //Size of the serial buffer used by new comms protocol. For SD transfers this must be at least 512 + 1 (flag) + 4 (sector)
  #define BOARD_MAX_IO_PINS  52 //digital pins + analog channels + 1
  #define BOARD_MAX_DIGITAL_PINS 52 //Pretty sure this isn't right
  #define EEPROM_LIB_H <EEPROM.h> //The name of the file that provides the EEPROM class
  typedef int eeprom_address_t;
  #define micros_safe() micros() //timer5 method is not used on anything but AVR, the micros_safe() macro is simply an alias for the normal micros()
  void initBoard();
  uint16_t freeRam();
  void doSystemReset();
  void jumpToBootloader();

  #define pinIsReserved(pin)  ( ((pin) == 0) ) //Forbidden pins like USB

/*
***********************************************************************************************************
* Schedules
*/
  
  #define FUEL1_COUNTER <register here>
  #define FUEL2_COUNTER <register here>
  #define FUEL3_COUNTER <register here>
  #define FUEL4_COUNTER <register here>
  //The below are optional, but recommended if there are sufficient timers/compares
  #define FUEL5_COUNTER <register here>
  #define FUEL6_COUNTER <register here>
  #define FUEL7_COUNTER <register here>
  #define FUEL8_COUNTER <register here>

  #define IGN1_COUNTER  <register here>
  #define IGN2_COUNTER  <register here>
  #define IGN3_COUNTER  <register here>
  #define IGN4_COUNTER  <register here>
  //The below are optional, but recommended if there are sufficient timers/compares
  #define IGN5_COUNTER  <register here>
  #define IGN6_COUNTER  <register here>
  #define IGN7_COUNTER  <register here>
  #define IGN8_COUNTER  <register here>

  #define FUEL1_COMPARE <register here>
  #define FUEL2_COMPARE <register here>
  #define FUEL3_COMPARE <register here>
  #define FUEL4_COMPARE <register here>
  //The below are optional, but recommended if there are sufficient timers/compares
  #define FUEL5_COMPARE <register here>
  #define FUEL6_COMPARE <register here>
  #define FUEL7_COMPARE <register here>
  #define FUEL8_COMPARE <register here>

  #define IGN1_COMPARE  <register here>
  #define IGN2_COMPARE  <register here>
  #define IGN3_COMPARE  <register here>
  #define IGN4_COMPARE  <register here>
  //The below are optional, but recommended if there are sufficient timers/compares
  #define IGN5_COMPARE  <register here>
  #define IGN6_COMPARE  <register here>
  #define IGN7_COMPARE  <register here>
  #define IGN8_COMPARE  <register here>

  #define FUEL1_TIMER_ENABLE() <macro here>
  #define FUEL2_TIMER_ENABLE() <macro here>
  #define FUEL3_TIMER_ENABLE() <macro here>
  #define FUEL4_TIMER_ENABLE() <macro here>
  //The below are optional, but recommended if there are sufficient timers/compares
  #define FUEL5_TIMER_ENABLE() <macro here>
  #define FUEL6_TIMER_ENABLE() <macro here>
  #define FUEL7_TIMER_ENABLE() <macro here>
  #define FUEL8_TIMER_ENABLE() <macro here>

  #define FUEL1_TIMER_DISABLE() <macro here>
  #define FUEL2_TIMER_DISABLE() <macro here>
  #define FUEL3_TIMER_DISABLE() <macro here>
  #define FUEL4_TIMER_DISABLE() <macro here>
  //The below are optional, but recommended if there are sufficient timers/compares
  #define FUEL5_TIMER_DISABLE() <macro here>
  #define FUEL6_TIMER_DISABLE() <macro here>
  #define FUEL7_TIMER_DISABLE() <macro here>
  #define FUEL8_TIMER_DISABLE() <macro here>

  #define IGN1_TIMER_ENABLE() <macro here>
  #define IGN2_TIMER_ENABLE() <macro here>
  #define IGN3_TIMER_ENABLE() <macro here>
  #define IGN4_TIMER_ENABLE() <macro here>
  //The below are optional, but recommended if there are sufficient timers/compares
  #define IGN5_TIMER_ENABLE() <macro here>
  #define IGN6_TIMER_ENABLE() <macro here>
  #define IGN7_TIMER_ENABLE() <macro here>
  #define IGN8_TIMER_ENABLE() <macro here>

  #define IGN1_TIMER_DISABLE() <macro here>
  #define IGN2_TIMER_DISABLE() <macro here>
  #define IGN3_TIMER_DISABLE() <macro here>
  #define IGN4_TIMER_DISABLE() <macro here>
  //The below are optional, but recommended if there are sufficient timers/compares
  #define IGN5_TIMER_DISABLE() <macro here>
  #define IGN6_TIMER_DISABLE() <macro here>
  #define IGN7_TIMER_DISABLE() <macro here>
  #define IGN8_TIMER_DISABLE() <macro here>

  
  #define MAX_TIMER_PERIOD 139808 //This is the maximum time, in uS, that the compare channels can run before overflowing. It is typically 65535 * <how long each tick represents>
  #define uS_TO_TIMER_COMPARE(uS) ((uS * 15) >> 5) //Converts a given number of uS into the required number of timer ticks until that time has passed.

/*
***********************************************************************************************************
* Auxiliaries
*/
  //macro functions for enabling and disabling timer interrupts for the boost and vvt functions
  #define ENABLE_BOOST_TIMER()  <macro here>
  #define DISABLE_BOOST_TIMER() <macro here>

  #define ENABLE_VVT_TIMER()    <macro here>
  #define DISABLE_VVT_TIMER()   <macro here>

  #define BOOST_TIMER_COMPARE   <register here>
  #define BOOST_TIMER_COUNTER   <register here>
  #define VVT_TIMER_COMPARE     <register here>
  #define VVT_TIMER_COUNTER     <register here>

/*
***********************************************************************************************************
* Idle
*/
  //Same as above, but for the timer controlling PWM idle
  #define IDLE_COUNTER          <register here>
  #define IDLE_COMPARE          <register here>

  #define IDLE_TIMER_ENABLE()   <macro here>
  #define IDLE_TIMER_DISABLE()  <macro here>

/*
***********************************************************************************************************
* CAN / Second serial
*/


#endif //CORE_TEMPLATE
#endif //TEMPLATE_H
