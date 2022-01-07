#include "board_stm32_official.h"
#if defined(STM32_CORE_VERSION_MAJOR)
#include "globals.h"
#include "auxiliaries.h"
#include "idle.h"
#include "scheduler.h"
#include "HardwareTimer.h"

#if HAL_CAN_MODULE_ENABLED
//This activates CAN1 interface on STM32, but it's named as Can0, because that's how Teensy implementation is done
STM32_CAN Can0 (CAN1, ALT_2, RX_SIZE_256, TX_SIZE_16);
/*
These CAN interfaces and pins are available for use, depending on the chip/package:
Default CAN1 pins are PA11 and PA12. Alternative (ALT) pins are PB8 & PB9 and ALT_2 pins are PD0 & PD1.
Default CAN2 pins are PB12 & PB13. Alternative (ALT) pins are PB5 & PB6.
Default CAN3 pins are PA8 & PA15. Alternative (ALT) pins are PB3 & PB4.
*/
#endif

#if defined(SRAM_AS_EEPROM)
    BackupSramAsEEPROM EEPROM;
#elif defined(USE_SPI_EEPROM)
    #if defined(STM32F407xx)
      SPIClass SPI_for_flash(PB5, PB4, PB3); //SPI1_MOSI, SPI1_MISO, SPI1_SCK
    #else //Blue/Black Pills
      SPIClass SPI_for_flash(PB15, PB14, PB13);
    #endif
 
    //windbond W25Q16 SPI flash EEPROM emulation
    EEPROM_Emulation_Config EmulatedEEPROMMconfig{255UL, 4096UL, 31, 0x00100000UL};
    Flash_SPI_Config SPIconfig{USE_SPI_EEPROM, SPI_for_flash};
    SPI_EEPROM_Class EEPROM(EmulatedEEPROMMconfig, SPIconfig);
#elif defined(FRAM_AS_EEPROM) //https://github.com/VitorBoss/FRAM
    #if defined(STM32F407xx)
      SPIClass SPI_for_FRAM(PB5, PB4, PB3); //SPI1_MOSI, SPI1_MISO, SPI1_SCK
      FramClass EEPROM(PB0, SPI_for_FRAM);
    #else //Blue/Black Pills
      SPIClass SPI_for_FRAM(PB15, PB14, PB13);
      FramClass EEPROM(PB12, SPI_for_FRAM);
    #endif
#elif defined(STM32F7xx)
  #if defined(DUAL_BANK)
    EEPROM_Emulation_Config EmulatedEEPROMMconfig{4UL, 131072UL, 2047UL, 0x08120000UL};
  #else
    EEPROM_Emulation_Config EmulatedEEPROMMconfig{2UL, 262144UL, 4095UL, 0x08180000UL};
  #endif
    InternalSTM32F7_EEPROM_Class EEPROM(EmulatedEEPROMMconfig);
#elif defined(STM32F401xC)
    EEPROM_Emulation_Config EmulatedEEPROMMconfig{1UL, 131072UL, 4095UL, 0x08020000UL};
    InternalSTM32F4_EEPROM_Class EEPROM(EmulatedEEPROMMconfig);
#elif defined(STM32F411xE)
    EEPROM_Emulation_Config EmulatedEEPROMMconfig{2UL, 131072UL, 4095UL, 0x08040000UL};
    InternalSTM32F4_EEPROM_Class EEPROM(EmulatedEEPROMMconfig);
#else //default case, internal flash as EEPROM for STM32F4
    EEPROM_Emulation_Config EmulatedEEPROMMconfig{4UL, 131072UL, 2047UL, 0x08080000UL};
    InternalSTM32F4_EEPROM_Class EEPROM(EmulatedEEPROMMconfig);
#endif


HardwareTimer Timer1(TIM1);
HardwareTimer Timer2(TIM2);
HardwareTimer Timer3(TIM3);
HardwareTimer Timer4(TIM4);
#if !defined(ARDUINO_BLUEPILL_F103C8) && !defined(ARDUINO_BLUEPILL_F103CB) //F103 just have 4 timers
HardwareTimer Timer5(TIM5);
#if defined(TIM11)
HardwareTimer Timer11(TIM11);
#elif defined(TIM7)
HardwareTimer Timer11(TIM7);
#endif
#endif

#ifdef RTC_ENABLED
STM32RTC& rtc = STM32RTC::getInstance();
#endif

  void initBoard()
  {
    /*
    ***********************************************************************************************************
    * General
    */
    #ifndef FLASH_LENGTH
      #define FLASH_LENGTH 8192
    #endif
    delay(10);

    /*
    ***********************************************************************************************************
    * Real Time clock for datalogging/time stamping
    */
    #ifdef RTC_ENABLED
      rtc.setClockSource(STM32RTC::LSE_CLOCK); //Initialize external clock for RTC. That is the only clock running of VBAT
      rtc.begin(); // initialize RTC 24H format
    #endif
    /*
    ***********************************************************************************************************
    * Idle
    */
    if( (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_OL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_CL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_OLCL))
    {
        idle_pwm_max_count = 1000000L / (TIMER_RESOLUTION * configPage6.idleFreq * 2); //Converts the frequency in Hz to the number of ticks (at 4uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 5KHz
    } 

    //This must happen at the end of the idle init
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer1.setMode(4, TIMER_OUTPUT_COMPARE);
    #else
    Timer1.setMode(4, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer1.attachInterrupt(4, idleInterrupt);  //on first flash the configPage4.iacAlgorithm is invalid


    /*
    ***********************************************************************************************************
    * Timers
    */
    #if defined(ARDUINO_BLUEPILL_F103C8) || defined(ARDUINO_BLUEPILL_F103CB)
      Timer4.setOverflow(1000, MICROSEC_FORMAT);  // Set up period
      #if ( STM32_CORE_VERSION_MAJOR < 2 )
      Timer4.setMode(1, TIMER_OUTPUT_COMPARE);
      Timer4.attachInterrupt(1, oneMSInterval);
      #else //2.0 forward
      Timer4.attachInterrupt(oneMSInterval);
      #endif
      Timer4.resume(); //Start Timer
    #else
      Timer11.setOverflow(1000, MICROSEC_FORMAT);  // Set up period
      #if ( STM32_CORE_VERSION_MAJOR < 2 )
      Timer11.setMode(1, TIMER_OUTPUT_COMPARE);
      Timer11.attachInterrupt(1, oneMSInterval);
      #else
      Timer11.attachInterrupt(oneMSInterval);
      #endif
      Timer11.resume(); //Start Timer
    #endif
    pinMode(LED_BUILTIN, OUTPUT); //Visual WDT

    /*
    ***********************************************************************************************************
    * Auxilliaries
    */
    //2uS resolution Min 8Hz, Max 5KHz

    boost_pwm_max_count = 1000000L / (TIMER_RESOLUTION * configPage6.boostFreq * 2); //Converts the frequency in Hz to the number of ticks (at 4uS) it takes to complete 1 cycle. The x2 is there because the frequency is stored at half value (in a byte) to allow freqneucies up to 511Hz
    vvt_pwm_max_count = 1000000L / (TIMER_RESOLUTION * configPage6.vvtFreq * 2); //Converts the frequency in Hz to the number of ticks (at 4uS) it takes to complete 1 cycle
    fan_pwm_max_count = 1000000L / (TIMER_RESOLUTION * configPage6.fanFreq * 2); //Converts the frequency in Hz to the number of ticks (at 4uS) it takes to complete 1 cycle

    //Need to be initialised last due to instant interrupt
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer1.setMode(1, TIMER_OUTPUT_COMPARE);
    Timer1.setMode(2, TIMER_OUTPUT_COMPARE);
    Timer1.setMode(3, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
	Timer1.setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer1.setMode(2, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer1.setMode(3, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer1.attachInterrupt(1, fanInterrupt);
    Timer1.attachInterrupt(2, boostInterrupt);
    Timer1.attachInterrupt(3, vvtInterrupt);

    /*
    ***********************************************************************************************************
    * Schedules
    */
    Timer1.setOverflow(0xFFFF, TICK_FORMAT);
    Timer2.setOverflow(0xFFFF, TICK_FORMAT);
    Timer3.setOverflow(0xFFFF, TICK_FORMAT);

    Timer1.setPrescaleFactor(((Timer1.getTimerClkFreq()/1000000) * TIMER_RESOLUTION)-1);   //4us resolution
    Timer2.setPrescaleFactor(((Timer2.getTimerClkFreq()/1000000) * TIMER_RESOLUTION)-1);   //4us resolution
    Timer3.setPrescaleFactor(((Timer3.getTimerClkFreq()/1000000) * TIMER_RESOLUTION)-1);   //4us resolution

    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer2.setMode(1, TIMER_OUTPUT_COMPARE);
    Timer2.setMode(2, TIMER_OUTPUT_COMPARE);
    Timer2.setMode(3, TIMER_OUTPUT_COMPARE);
    Timer2.setMode(4, TIMER_OUTPUT_COMPARE);

    Timer3.setMode(1, TIMER_OUTPUT_COMPARE);
    Timer3.setMode(2, TIMER_OUTPUT_COMPARE);
    Timer3.setMode(3, TIMER_OUTPUT_COMPARE);
    Timer3.setMode(4, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer2.setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer2.setMode(2, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer2.setMode(3, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer2.setMode(4, TIMER_OUTPUT_COMPARE_TOGGLE);

    Timer3.setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer3.setMode(2, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer3.setMode(3, TIMER_OUTPUT_COMPARE_TOGGLE);
    Timer3.setMode(4, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    //Attach interrupt functions
    //Injection
    Timer3.attachInterrupt(1, fuelSchedule1Interrupt);
    Timer3.attachInterrupt(2, fuelSchedule2Interrupt);
    Timer3.attachInterrupt(3, fuelSchedule3Interrupt);
    Timer3.attachInterrupt(4, fuelSchedule4Interrupt);
    #if (INJ_CHANNELS >= 5)
    Timer5.setOverflow(0xFFFF, TICK_FORMAT);
    Timer5.setPrescaleFactor(((Timer5.getTimerClkFreq()/1000000) * TIMER_RESOLUTION)-1);   //4us resolution
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer5.setMode(1, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer5.setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer5.attachInterrupt(1, fuelSchedule5Interrupt);
    #endif
    #if (INJ_CHANNELS >= 6)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer5.setMode(2, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer5.setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer5.attachInterrupt(2, fuelSchedule6Interrupt);
    #endif
    #if (INJ_CHANNELS >= 7)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer5.setMode(3, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer5.setMode(3, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer5.attachInterrupt(3, fuelSchedule7Interrupt);
    #endif
    #if (INJ_CHANNELS >= 8)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer5.setMode(4, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer5.setMode(4, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer5.attachInterrupt(4, fuelSchedule8Interrupt);
    #endif

    //Ignition
    Timer2.attachInterrupt(1, ignitionSchedule1Interrupt); 
    Timer2.attachInterrupt(2, ignitionSchedule2Interrupt);
    Timer2.attachInterrupt(3, ignitionSchedule3Interrupt);
    Timer2.attachInterrupt(4, ignitionSchedule4Interrupt);
    #if (IGN_CHANNELS >= 5)
    Timer4.setOverflow(0xFFFF, TICK_FORMAT);
    Timer4.setPrescaleFactor(((Timer4.getTimerClkFreq()/1000000) * TIMER_RESOLUTION)-1);   //4us resolution
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer4.setMode(1, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer4.setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer4.attachInterrupt(1, ignitionSchedule5Interrupt);
    #endif
    #if (IGN_CHANNELS >= 6)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer4.setMode(2, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer4.setMode(2, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer4.attachInterrupt(2, ignitionSchedule6Interrupt);
    #endif
    #if (IGN_CHANNELS >= 7)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer4.setMode(3, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer4.setMode(3, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer4.attachInterrupt(3, ignitionSchedule7Interrupt);
    #endif
    #if (IGN_CHANNELS >= 8)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer4.setMode(4, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer4.setMode(4, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer4.attachInterrupt(4, ignitionSchedule8Interrupt);
    #endif

    Timer1.resume();
    DISABLE_BOOST_TIMER();  //Make sure it is disabled. It's is enabled by default on the library
    DISABLE_VVT_TIMER();    //Make sure it is disabled. It's is enabled by default on the library
    IDLE_TIMER_DISABLE();   //Make sure it is disabled. It's is enabled by default on the library
    Timer2.resume();
    IGN1_TIMER_DISABLE(); //Make sure it is disabled. It's is enabled by default on the library
    IGN2_TIMER_DISABLE(); //Make sure it is disabled. It's is enabled by default on the library
    IGN3_TIMER_DISABLE(); //Make sure it is disabled. It's is enabled by default on the library
    IGN4_TIMER_DISABLE(); //Make sure it is disabled. It's is enabled by default on the library
    Timer3.resume();
    FUEL1_TIMER_DISABLE();  //Make sure it is disabled. It's is enabled by default on the library
    FUEL2_TIMER_DISABLE();  //Make sure it is disabled. It's is enabled by default on the library
    FUEL3_TIMER_DISABLE();  //Make sure it is disabled. It's is enabled by default on the library
    FUEL4_TIMER_DISABLE();  //Make sure it is disabled. It's is enabled by default on the library
    #if (IGN_CHANNELS >= 5)
    Timer4.resume();
    IGN5_TIMER_DISABLE(); //Make sure it is disabled. It's is enabled by default on the library
    IGN6_TIMER_DISABLE(); //Make sure it is disabled. It's is enabled by default on the library
    IGN7_TIMER_DISABLE(); //Make sure it is disabled. It's is enabled by default on the library
    IGN8_TIMER_DISABLE(); //Make sure it is disabled. It's is enabled by default on the library
    #endif
    #if (INJ_CHANNELS >= 5)
    Timer5.resume();
    FUEL5_TIMER_DISABLE();  //Make sure it is disabled. It's is enabled by default on the library
    FUEL6_TIMER_DISABLE();  //Make sure it is disabled. It's is enabled by default on the library
    FUEL7_TIMER_DISABLE();  //Make sure it is disabled. It's is enabled by default on the library
    FUEL8_TIMER_DISABLE();  //Make sure it is disabled. It's is enabled by default on the library
    #endif
  }

  uint16_t freeRam()
  {
      char top = 't';
      return &top - reinterpret_cast<char*>(sbrk(0));
  }

  void doSystemReset( void )
  {
    __disable_irq();
    NVIC_SystemReset();
  }

  void jumpToBootloader( void ) // https://github.com/3devo/Arduino_Core_STM32/blob/jumpSysBL/libraries/SrcWrapper/src/stm32/bootloader.c
  { // https://github.com/markusgritsch/SilF4ware/blob/master/SilF4ware/drv_reset.c
    #if !defined(STM32F103xB)
    HAL_RCC_DeInit();
    HAL_DeInit();
    SysTick->VAL = SysTick->LOAD = SysTick->CTRL = 0;
    SYSCFG->MEMRMP = 0x01;

    #if defined(STM32F7xx) || defined(STM32H7xx)
    const uint32_t DFU_addr = 0x1FF00000; // From AN2606
    #else
    const uint32_t DFU_addr = 0x1FFF0000; // Default for STM32F10xxx and STM32F40xxx/STM32F41xxx from AN2606
    #endif
    // This is assembly to prevent modifying the stack pointer after
    // loading it, and to ensure a jump (not call) to the bootloader.
    // Not sure if the barriers are really needed, they were taken from
    // https://github.com/GrumpyOldPizza/arduino-STM32L4/blob/ac659033eadd50cfe001ba1590a1362b2d87bb76/system/STM32L4xx/Source/boot_stm32l4xx.c#L159-L165
    asm volatile (
      "ldr r0, [%[DFU_addr], #0]   \n\t"  // get address of stack pointer
      "msr msp, r0            \n\t"  // set stack pointer
      "ldr r0, [%[DFU_addr], #4]   \n\t"  // get address of reset handler
      "dsb                    \n\t"  // data sync barrier
      "isb                    \n\t"  // instruction sync barrier
      "bx r0                  \n\t"  // branch to bootloader
      : : [DFU_addr] "l" (DFU_addr) : "r0"
    );
    __builtin_unreachable();
    #endif
  }

  /*
  ***********************************************************************************************************
  * Interrupt callback functions
  */
  #if ((STM32_CORE_VERSION_MINOR<=8) & (STM32_CORE_VERSION_MAJOR==1)) 
  void oneMSInterval(HardwareTimer*){oneMSInterval();}
  void boostInterrupt(HardwareTimer*){boostInterrupt();}
  void fuelSchedule1Interrupt(HardwareTimer*){fuelSchedule1Interrupt();}
  void fuelSchedule2Interrupt(HardwareTimer*){fuelSchedule2Interrupt();}
  void fuelSchedule3Interrupt(HardwareTimer*){fuelSchedule3Interrupt();}
  void fuelSchedule4Interrupt(HardwareTimer*){fuelSchedule4Interrupt();}
  #if (INJ_CHANNELS >= 5)
  void fuelSchedule5Interrupt(HardwareTimer*){fuelSchedule5Interrupt();}
  #endif
  #if (INJ_CHANNELS >= 6)
  void fuelSchedule6Interrupt(HardwareTimer*){fuelSchedule6Interrupt();}
  #endif
  #if (INJ_CHANNELS >= 7)
  void fuelSchedule7Interrupt(HardwareTimer*){fuelSchedule7Interrupt();}
  #endif
  #if (INJ_CHANNELS >= 8)
  void fuelSchedule8Interrupt(HardwareTimer*){fuelSchedule8Interrupt();}
  #endif
  void idleInterrupt(HardwareTimer*){idleInterrupt();}
  void vvtInterrupt(HardwareTimer*){vvtInterrupt();}
  void fanInterrupt(HardwareTimer*){fanInterrupt();}
  void ignitionSchedule1Interrupt(HardwareTimer*){ignitionSchedule1Interrupt();}
  void ignitionSchedule2Interrupt(HardwareTimer*){ignitionSchedule2Interrupt();}
  void ignitionSchedule3Interrupt(HardwareTimer*){ignitionSchedule3Interrupt();}
  void ignitionSchedule4Interrupt(HardwareTimer*){ignitionSchedule4Interrupt();}
  #if (IGN_CHANNELS >= 5)
  void ignitionSchedule5Interrupt(HardwareTimer*){ignitionSchedule5Interrupt();}
  #endif
  #if (IGN_CHANNELS >= 6)
  void ignitionSchedule6Interrupt(HardwareTimer*){ignitionSchedule6Interrupt();}
  #endif
  #if (IGN_CHANNELS >= 7)
  void ignitionSchedule7Interrupt(HardwareTimer*){ignitionSchedule7Interrupt();}
  #endif
  #if (IGN_CHANNELS >= 8)
  void ignitionSchedule8Interrupt(HardwareTimer*){ignitionSchedule8Interrupt();}
  #endif
  #endif //End core<=1.8
#endif
