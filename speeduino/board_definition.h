#pragma once

/** @file
 * @brief Inclusion of board specific header files and board related definitions.
 * 
 * @note This file should be named "board.h", but one of the STM32 Arduino implementations
 * has a <board.h> include. Which picks up *this file* instead of the intended file :-( 
*/
#include <Arduino.h>

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #define CORE_AVR
  #include "board_avr2560.h"

  #if defined(__AVR_ATmega2561__)
    //This is a workaround to avoid having to change all the references to higher ADC channels. We simply define the channels (Which don't exist on the 2561) as being the same as A0-A7
    //These Analog inputs should never be used on any 2561 board definition (Because they don't exist on the MCU), so it will not cause any issues
    #define A8  A0
    #define A9  A1
    #define A10  A2
    #define A11  A3
    #define A12  A4
    #define A13  A5
    #define A14  A6
    #define A15  A7
  #endif
#elif defined(CORE_TEENSY)
  #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    #define CORE_TEENSY35
    #include "board_teensy35.h"
  #elif defined(__IMXRT1062__)
    #define CORE_TEENSY41
    #include "board_teensy41.h"
  #endif
#elif defined(STM32_MCU_SERIES) || defined(ARDUINO_ARCH_STM32) || defined(STM32)
  #define CORE_STM32
  #include "board_stm32_official.h"
#elif defined(__SAMD21G18A__)
  #define CORE_SAMD21
  #define CORE_SAM
  #include "board_samd21.h"
  #define INJ_CHANNELS 4
  #define IGN_CHANNELS 4
#elif defined(__SAMC21J18A__)
  #define CORE_SAMC21
  #define CORE_SAM
  #include "board_samc21.h"
#elif defined(__SAME51J19A__)
  #define CORE_SAME51
  #define CORE_SAM
  #include "board_same51.h"
#else
  #error Incorrect board selected. Please select the correct board (Usually Mega 2560) and upload again
#endif

// AVR & Teensy both support the AVR atomic macros
#if defined(CORE_AVR) || defined(CORE_TEENSY)
#include <util/atomic.h>
#else
// All others make a no-op
#define ATOMIC_BLOCK(type)
#define NONATOMIC_BLOCK(type)
#define	ATOMIC_RESTORESTATE
#define	ATOMIC_FORCEON
#define	NONATOMIC_RESTORESTATE 
#define	NONATOMIC_FORCEOFF
#endif