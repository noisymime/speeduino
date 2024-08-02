#include "board_definition.h"

#if defined(STM32_CORE_VERSION_MAJOR)
#include "auxiliaries.h"
#include "idle.h"
#include "scheduler.h"
#include "HardwareTimer.h"
#include "timers.h"
#include "comms_secondary.h"

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

#if defined SD_LOGGING
    SPIClass SD_SPI(PC12, PC11, PC10); //SPI3_MOSI, SPI3_MISO, SPI3_SCK
#endif

#if defined(SRAM_AS_EEPROM)
    BackupSramAsEEPROM EEPROM;
#elif defined(USE_SPI_EEPROM)
    #if defined(STM32F407xx)
      SPIClass SPI_for_flash(PB5, PB4, PB3); //SPI1_MOSI, SPI1_MISO, SPI1_SCK
    #else //Blue/Black Pills
      SPIClass SPI_for_flash(PB15, PB14, PB13);
    #endif
 
    //winbond W25Q16 SPI flash EEPROM emulation
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

  /*
  ***********************************************************************************************************
  * Interrupt callback functions
  */
  #define IGNITION_INTERRUPT_NAME(index) CONCAT(CONCAT(ignitionSchedule, index), Interrupt)
  #define FUEL_INTERRUPT_NAME(index) CONCAT(CONCAT(fuelSchedule, index), Interrupt)


  #if ((STM32_CORE_VERSION_MINOR<=8) & (STM32_CORE_VERSION_MAJOR==1)) 
  void oneMSInterval(HardwareTimer*){oneMSInterval();}
  void boostInterrupt(HardwareTimer*){boostInterrupt();}
  void idleInterrupt(HardwareTimer*){idleInterrupt();}
  void vvtInterrupt(HardwareTimer*){vvtInterrupt();}
  void fanInterrupt(HardwareTimer*){fanInterrupt();}
  #define STM_FUEL_INTERRUPT(index) void FUEL_INTERRUPT_NAME(index)(HardwareTimer*) {moveToNextState(fuelSchedule ## index);}
  #define STM_IGNITION_INTERRUPT(index) void IGNITION_INTERRUPT_NAME(index)(HardwareTimer*) {moveToNextState(ignitionSchedule ## index);}
  #else //End core<=1.8
  #define STM_FUEL_INTERRUPT(index) void FUEL_INTERRUPT_NAME(index)(void) {moveToNextState(fuelSchedule ## index);}
  #define STM_IGNITION_INTERRUPT(index) void IGNITION_INTERRUPT_NAME(index)(void) {moveToNextState(ignitionSchedule ## index);}
  #endif

  STM_FUEL_INTERRUPT(1)
  STM_FUEL_INTERRUPT(2)
  STM_FUEL_INTERRUPT(3)
  STM_FUEL_INTERRUPT(4)
  #if (INJ_CHANNELS >= 5)
  STM_FUEL_INTERRUPT(5)
  #endif
  #if (INJ_CHANNELS >= 6)
  STM_FUEL_INTERRUPT(6)
  #endif
  #if (INJ_CHANNELS >= 7)
  STM_FUEL_INTERRUPT(7)
  #endif
  #if (INJ_CHANNELS >= 8)
  STM_FUEL_INTERRUPT(8)
  #endif

  STM_IGNITION_INTERRUPT(1)
  STM_IGNITION_INTERRUPT(2)
  STM_IGNITION_INTERRUPT(3)
  STM_IGNITION_INTERRUPT(4)
  #if (IGN_CHANNELS >= 5)
  STM_IGNITION_INTERRUPT(5)
  #endif
  #if (IGN_CHANNELS >= 6)
  STM_IGNITION_INTERRUPT(6)
  #endif
  #if (IGN_CHANNELS >= 7)
  STM_IGNITION_INTERRUPT(7)
  #endif
  #if (IGN_CHANNELS >= 8)
  STM_IGNITION_INTERRUPT(8)
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

    #ifndef HAVE_HWSERIAL2 //Hack to get the code to compile on BlackPills
    #define Serial2 Serial1
    #endif
    pSecondarySerial = &Serial2;

    /*
    ***********************************************************************************************************
    * Real Time clock for datalogging/time stamping
    */
    #ifdef RTC_ENABLED
      //Check if RTC time has been set earlier. If yes, RTC will use LSE_CLOCK. If not, default LSI_CLOCK is used, to prevent hanging on boot.
      if (rtc.isTimeSet()) {
        rtc.setClockSource(STM32RTC::LSE_CLOCK); //Initialise external clock for RTC if clock is set. That is the only clock running of VBAT
      }
      rtc.begin(); // initialise RTC 24H format
    #endif
    /*
    ***********************************************************************************************************
    * Idle
    */
    if( (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_OL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_CL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_OLCL))
    {
        idle_pwm_max_count = (uint16_t)(MICROS_PER_SEC / (TIMER_RESOLUTION * configPage6.idleFreq * 2U)); //Converts the frequency in Hz to the number of ticks (at 4uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 5KHz
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
    * Auxiliaries
    */
    //2uS resolution Min 8Hz, Max 5KHz
    boost_pwm_max_count = (uint16_t)(MICROS_PER_SEC / (TIMER_RESOLUTION * configPage6.boostFreq * 2U)); //Converts the frequency in Hz to the number of ticks (at 4uS) it takes to complete 1 cycle. The x2 is there because the frequency is stored at half value (in a byte) to allow frequencies up to 511Hz
    vvt_pwm_max_count = (uint16_t)(MICROS_PER_SEC / (TIMER_RESOLUTION * configPage6.vvtFreq * 2U)); //Converts the frequency in Hz to the number of ticks (at 4uS) it takes to complete 1 cycle
    fan_pwm_max_count = (uint16_t)(MICROS_PER_SEC / (TIMER_RESOLUTION * configPage6.fanFreq * 2U)); //Converts the frequency in Hz to the number of ticks (at 4uS) it takes to complete 1 cycle

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
    Timer3.attachInterrupt(1, FUEL_INTERRUPT_NAME(1));
    Timer3.attachInterrupt(2, FUEL_INTERRUPT_NAME(2));
    Timer3.attachInterrupt(3, FUEL_INTERRUPT_NAME(3));
    Timer3.attachInterrupt(4, FUEL_INTERRUPT_NAME(4));
    #if (INJ_CHANNELS >= 5)
    Timer5.setOverflow(0xFFFF, TICK_FORMAT);
    Timer5.setPrescaleFactor(((Timer5.getTimerClkFreq()/1000000) * TIMER_RESOLUTION)-1);   //4us resolution
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer5.setMode(1, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer5.setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer5.attachInterrupt(1, FUEL_INTERRUPT_NAME(5));
    #endif
    #if (INJ_CHANNELS >= 6)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer5.setMode(2, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer5.setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer5.attachInterrupt(2, FUEL_INTERRUPT_NAME(6));
    #endif
    #if (INJ_CHANNELS >= 7)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer5.setMode(3, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer5.setMode(3, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer5.attachInterrupt(3, FUEL_INTERRUPT_NAME(7));
    #endif
    #if (INJ_CHANNELS >= 8)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer5.setMode(4, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer5.setMode(4, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer5.attachInterrupt(4, FUEL_INTERRUPT_NAME(8));
    #endif

    //Ignition
    Timer2.attachInterrupt(1, IGNITION_INTERRUPT_NAME(1)); 
    Timer2.attachInterrupt(2, IGNITION_INTERRUPT_NAME(2));
    Timer2.attachInterrupt(3, IGNITION_INTERRUPT_NAME(3));
    Timer2.attachInterrupt(4, IGNITION_INTERRUPT_NAME(4));
    #if (IGN_CHANNELS >= 5)
    Timer4.setOverflow(0xFFFF, TICK_FORMAT);
    Timer4.setPrescaleFactor(((Timer4.getTimerClkFreq()/1000000) * TIMER_RESOLUTION)-1);   //4us resolution
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer4.setMode(1, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer4.setMode(1, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer4.attachInterrupt(1, IGNITION_INTERRUPT_NAME(5));
    #endif
    #if (IGN_CHANNELS >= 6)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer4.setMode(2, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer4.setMode(2, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer4.attachInterrupt(2, IGNITION_INTERRUPT_NAME(6));
    #endif
    #if (IGN_CHANNELS >= 7)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer4.setMode(3, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer4.setMode(3, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer4.attachInterrupt(3, IGNITION_INTERRUPT_NAME(7));
    #endif
    #if (IGN_CHANNELS >= 8)
    #if ( STM32_CORE_VERSION_MAJOR < 2 )
    Timer4.setMode(4, TIMER_OUTPUT_COMPARE);
    #else //2.0 forward
    Timer4.setMode(4, TIMER_OUTPUT_COMPARE_TOGGLE);
    #endif
    Timer4.attachInterrupt(4, IGNITION_INTERRUPT_NAME(8));
    #endif


  }

  uint16_t freeRam()
  {
    uint32_t freeRam;
    uint32_t stackTop;
    uint32_t heapTop;

    // current position of the stack.
    stackTop = (uint32_t)&stackTop;

    // current position of heap.
    void *hTop = malloc(1);
    heapTop = (uint32_t)hTop;
    free(hTop);
    freeRam = stackTop - heapTop;

    if(freeRam>0xFFFF){return 0xFFFF;}
    else{return freeRam;}
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

#endif