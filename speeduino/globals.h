/** @file
 * Global defines, macros, struct definitions (@ref statuses, @ref config2, @ref config4, config*), extern-definitions (for globally accessible vars).
 * 
 * ### Note on configuration struct layouts
 * 
 * Once the struct members have been assigned to certain "role" (in certain SW version), they should not be "moved around"
 * as the structs are stored onto EEPROM as-is and the offset and size of member needs to remain constant. Also removing existing struct members
 * would disturb layouts. Because of this a certain amount unused old members will be left into the structs. For the storage related reasons also the
 * bit fields are defined in byte-size (or multiple of ...) chunks.
 * 
 * ### Config Structs and 2D, 3D Tables
 * 
 * The config* structures contain information coming from tuning SW (e.g. TS) for 2D and 3D tables, where looked up value is not a result of direct
 * array lookup, but from interpolation algorithm. Because of standard, reusable interpolation routines associated with structs table2D and table3D,
 * the values from config are copied from config* structs to table2D (table3D destined configurations are not stored in config* structures).
 * 
 * ### Board choice
 * There's a C-preprocessor based "#if defined" logic present in this header file based on the Arduino IDE compiler set CPU
 * (+board?) type, e.g. `__AVR_ATmega2560__`. This respectively drives (withi it's "#if defined ..." block):
 * - The setting of various BOARD_* C-preprocessor variables (e.g. BOARD_MAX_ADC_PINS)
 * - Setting of BOARD_H (Board header) file (e.g. "board_avr2560.h"), which is later used to include the header file
 *   - Seems Arduino ide implicitly compiles and links respective .ino file (by it's internal build/compilation rules) (?)
 * - Setting of CPU (?) CORE_* variables (e.g. CORE_AVR), that is used across codebase to distinguish CPU.
 */
#ifndef GLOBALS_H
#define GLOBALS_H

#include "table2d.h"
#include "table3d.h"
#include "status.h"
#include "page_types.h"

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__)
  #define BOARD_MAX_DIGITAL_PINS 54 //digital pins +1
  #define BOARD_MAX_IO_PINS 70 //digital pins + analog channels + 1
  #define BOARD_MAX_ADC_PINS  15 //Number of analog pins
  #ifndef LED_BUILTIN
    #define LED_BUILTIN 13
  #endif
  #define CORE_AVR
  #define BOARD_H "board_avr2560.h"
  #ifndef INJ_CHANNELS
    #define INJ_CHANNELS 4
  #endif
  #ifndef IGN_CHANNELS
    #define IGN_CHANNELS 5
  #endif

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

  //#define TIMER5_MICROS

#elif defined(CORE_TEENSY)
  #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    #define CORE_TEENSY35
    #define BOARD_H "board_teensy35.h"
  #elif defined(__IMXRT1062__)
    #define CORE_TEENSY41
    #define BOARD_H "board_teensy41.h"
  #endif
  #define INJ_CHANNELS 8
  #define IGN_CHANNELS 8

#elif defined(STM32_MCU_SERIES) || defined(ARDUINO_ARCH_STM32) || defined(STM32)
  #define BOARD_H "board_stm32_official.h"
  #define CORE_STM32

  #define BOARD_MAX_ADC_PINS  NUM_ANALOG_INPUTS-1 //Number of analog pins from core.
  #if defined(STM32F407xx) //F407 can do 8x8 STM32F401/STM32F411 don't
   #define INJ_CHANNELS 8
   #define IGN_CHANNELS 8
  #else
   #define INJ_CHANNELS 4
   #define IGN_CHANNELS 5
  #endif
#elif defined(__SAMD21G18A__)
  #define BOARD_H "board_samd21.h"
  #define CORE_SAMD21
  #define CORE_SAM
  #define INJ_CHANNELS 4
  #define IGN_CHANNELS 4
#elif defined(__SAMC21J18A__)
  #define BOARD_H "board_samc21.h"
  #define CORE_SAMC21
  #define CORE_SAM
#elif defined(__SAME51J19A__)
  #define BOARD_H "board_same51.h"
  #define CORE_SAME51
  #define CORE_SAM
  #define INJ_CHANNELS 8
  #define IGN_CHANNELS 8
#else
  #error Incorrect board selected. Please select the correct board (Usually Mega 2560) and upload again
#endif

//This can only be included after the above section
#include BOARD_H //Note that this is not a real file, it is defined in globals.h. 

#define CRANK_ANGLE_MAX (max(CRANK_ANGLE_MAX_IGN, CRANK_ANGLE_MAX_INJ))

#define MICROS_PER_SEC INT32_C(1000000)
#define MICROS_PER_MIN INT32_C(MICROS_PER_SEC*60U)
#define MICROS_PER_HOUR INT32_C(MICROS_PER_MIN*60U)

#define SERIAL_PORT_PRIMARY   0
#define SERIAL_PORT_SECONDARY 3

#define BIT_TIMER_1HZ             0
#define BIT_TIMER_4HZ             1
#define BIT_TIMER_10HZ            2
#define BIT_TIMER_15HZ            3
#define BIT_TIMER_30HZ            4
#define BIT_TIMER_200HZ           6
#define BIT_TIMER_1KHZ            7

#ifndef UNIT_TEST 
#define TOOTH_LOG_SIZE      127
#else
#define TOOTH_LOG_SIZE      1
#endif

#define O2_CALIBRATION_PAGE   2U
#define IAT_CALIBRATION_PAGE  1U
#define CLT_CALIBRATION_PAGE  0U

// note the sequence of these defines which reference the bits used in a byte has moved when the third trigger & engine cycle was incorporated
#define COMPOSITE_LOG_PRI   0
#define COMPOSITE_LOG_SEC   1
#define COMPOSITE_LOG_THIRD 2 
#define COMPOSITE_LOG_TRIG 3
#define COMPOSITE_LOG_SYNC 4
#define COMPOSITE_ENGINE_CYCLE 5

#define INJ1_CMD_BIT      0
#define INJ2_CMD_BIT      1
#define INJ3_CMD_BIT      2
#define INJ4_CMD_BIT      3
#define INJ5_CMD_BIT      4
#define INJ6_CMD_BIT      5
#define INJ7_CMD_BIT      6
#define INJ8_CMD_BIT      7

#define IGN1_CMD_BIT      0
#define IGN2_CMD_BIT      1
#define IGN3_CMD_BIT      2
#define IGN4_CMD_BIT      3
#define IGN5_CMD_BIT      4
#define IGN6_CMD_BIT      5
#define IGN7_CMD_BIT      6
#define IGN8_CMD_BIT      7

#define ENGINE_PROTECT_BIT_RPM  0
#define ENGINE_PROTECT_BIT_MAP  1
#define ENGINE_PROTECT_BIT_OIL  2
#define ENGINE_PROTECT_BIT_AFR  3
#define ENGINE_PROTECT_BIT_COOLANT 4


#define CALIBRATION_TABLE_SIZE 512 ///< Calibration table size for CLT, IAT, O2
#define CALIBRATION_TEMPERATURE_OFFSET 40 /**< All temperature measurements are stored offset by 40 degrees.
This is so we can use an unsigned byte (0-255) to represent temperature ranges from -40 to 215 */
#define OFFSET_FUELTRIM 127U ///< The fuel trim tables are offset by 128 to allow for -128 to +128 values
#define OFFSET_IGNITION 40 ///< Ignition values from the main spark table are offset 40 degrees downwards to allow for negative spark timing

#define SERIAL_BUFFER_THRESHOLD 32 ///< When the serial buffer is filled to greater than this threshold value, the serial processing operations will be performed more urgently in order to avoid it overflowing. Serial buffer is 64 bytes long, so the threshold is set at half this as a reasonable figure

#define LOGGER_CSV_SEPARATOR_SEMICOLON  0
#define LOGGER_CSV_SEPARATOR_COMMA      1
#define LOGGER_CSV_SEPARATOR_TAB        2
#define LOGGER_CSV_SEPARATOR_SPACE      3

#define LOGGER_RATE_1HZ                 0
#define LOGGER_RATE_4HZ                 1
#define LOGGER_RATE_10HZ                2
#define LOGGER_RATE_30HZ                3

#define LOGGER_FILENAMING_OVERWRITE     0
#define LOGGER_FILENAMING_DATETIME      1
#define LOGGER_FILENAMING_SEQENTIAL     2

extern struct table3d16RpmLoad fuelTable; //16x16 fuel map
extern struct table3d16RpmLoad fuelTable2; //16x16 fuel map
extern struct table3d16RpmLoad ignitionTable; //16x16 ignition map
extern struct table3d16RpmLoad ignitionTable2; //16x16 ignition map
extern struct table3d16RpmLoad afrTable; //16x16 afr target map
extern struct table3d8RpmLoad stagingTable; //8x8 fuel staging table
extern struct table3d8RpmLoad boostTable; //8x8 boost map
extern struct table3d8RpmLoad boostTableLookupDuty; //8x8 boost map
extern struct table3d8RpmLoad vvtTable; //8x8 vvt map
extern struct table3d8RpmLoad vvt2Table; //8x8 vvt map
extern struct table3d8RpmLoad wmiTable; //8x8 wmi map

typedef table3d6RpmLoad trimTable3d; 

extern trimTable3d trim1Table; //6x6 Fuel trim 1 map
extern trimTable3d trim2Table; //6x6 Fuel trim 2 map
extern trimTable3d trim3Table; //6x6 Fuel trim 3 map
extern trimTable3d trim4Table; //6x6 Fuel trim 4 map
extern trimTable3d trim5Table; //6x6 Fuel trim 5 map
extern trimTable3d trim6Table; //6x6 Fuel trim 6 map
extern trimTable3d trim7Table; //6x6 Fuel trim 7 map
extern trimTable3d trim8Table; //6x6 Fuel trim 8 map

extern struct table3d4RpmLoad dwellTable; //4x4 Dwell map
extern struct table2D taeTable; //4 bin TPS Acceleration Enrichment map (2D)
extern struct table2D maeTable;
extern struct table2D WUETable; //10 bin Warm Up Enrichment map (2D)
extern struct table2D ASETable; //4 bin After Start Enrichment map (2D)
extern struct table2D ASECountTable; //4 bin After Start duration map (2D)
extern struct table2D PrimingPulseTable; //4 bin Priming pulsewidth map (2D)
extern struct table2D crankingEnrichTable; //4 bin cranking Enrichment map (2D)
extern struct table2D dwellVCorrectionTable; //6 bin dwell voltage correction (2D)
extern struct table2D injectorVCorrectionTable; //6 bin injector voltage correction (2D)
extern struct table2D injectorAngleTable; //4 bin injector timing curve (2D)
extern struct table2D IATDensityCorrectionTable; //9 bin inlet air temperature density correction (2D)
extern struct table2D baroFuelTable; //8 bin baro correction curve (2D)
extern struct table2D IATRetardTable; //6 bin ignition adjustment based on inlet air temperature  (2D)
extern struct table2D idleTargetTable; //10 bin idle target table for idle timing (2D)
extern struct table2D idleAdvanceTable; //6 bin idle advance adjustment table based on RPM difference  (2D)
extern struct table2D CLTAdvanceTable; //6 bin ignition adjustment based on coolant temperature  (2D)
extern struct table2D rotarySplitTable; //8 bin ignition split curve for rotary leading/trailing  (2D)
extern struct table2D flexFuelTable;  //6 bin flex fuel correction table for fuel adjustments (2D)
extern struct table2D flexAdvTable;   //6 bin flex fuel correction table for timing advance (2D)
extern struct table2D flexBoostTable; //6 bin flex fuel correction table for boost adjustments (2D)
extern struct table2D fuelTempTable;  //6 bin fuel temperature correction table for fuel adjustments (2D)
extern struct table2D knockWindowStartTable;
extern struct table2D knockWindowDurationTable;
extern struct table2D oilPressureProtectTable;
extern struct table2D wmiAdvTable; //6 bin wmi correction table for timing advance (2D)
extern struct table2D coolantProtectTable; //6 bin coolant temperature protection table for engine protection (2D)
extern struct table2D fanPWMTable;
extern struct table2D rollingCutTable;

//These are for the direct port manipulation of the injectors, coils and aux outputs
extern volatile PORT_TYPE *inj1_pin_port;
extern volatile PINMASK_TYPE inj1_pin_mask;
extern volatile PORT_TYPE *inj2_pin_port;
extern volatile PINMASK_TYPE inj2_pin_mask;
extern volatile PORT_TYPE *inj3_pin_port;
extern volatile PINMASK_TYPE inj3_pin_mask;
extern volatile PORT_TYPE *inj4_pin_port;
extern volatile PINMASK_TYPE inj4_pin_mask;
extern volatile PORT_TYPE *inj5_pin_port;
extern volatile PINMASK_TYPE inj5_pin_mask;
extern volatile PORT_TYPE *inj6_pin_port;
extern volatile PINMASK_TYPE inj6_pin_mask;
extern volatile PORT_TYPE *inj7_pin_port;
extern volatile PINMASK_TYPE inj7_pin_mask;
extern volatile PORT_TYPE *inj8_pin_port;
extern volatile PINMASK_TYPE inj8_pin_mask;

extern volatile PORT_TYPE *ign1_pin_port;
extern volatile PINMASK_TYPE ign1_pin_mask;
extern volatile PORT_TYPE *ign2_pin_port;
extern volatile PINMASK_TYPE ign2_pin_mask;
extern volatile PORT_TYPE *ign3_pin_port;
extern volatile PINMASK_TYPE ign3_pin_mask;
extern volatile PORT_TYPE *ign4_pin_port;
extern volatile PINMASK_TYPE ign4_pin_mask;
extern volatile PORT_TYPE *ign5_pin_port;
extern volatile PINMASK_TYPE ign5_pin_mask;
extern volatile PORT_TYPE *ign6_pin_port;
extern volatile PINMASK_TYPE ign6_pin_mask;
extern volatile PORT_TYPE *ign7_pin_port;
extern volatile PINMASK_TYPE ign7_pin_mask;
extern volatile PORT_TYPE *ign8_pin_port;
extern volatile PINMASK_TYPE ign8_pin_mask;

extern volatile PORT_TYPE *tach_pin_port;
extern volatile PINMASK_TYPE tach_pin_mask;
extern volatile PORT_TYPE *pump_pin_port;
extern volatile PINMASK_TYPE pump_pin_mask;

extern volatile PORT_TYPE *flex_pin_port;
extern volatile PINMASK_TYPE flex_pin_mask;

extern volatile PORT_TYPE *triggerPri_pin_port;
extern volatile PINMASK_TYPE triggerPri_pin_mask;
extern volatile PORT_TYPE *triggerSec_pin_port;
extern volatile PINMASK_TYPE triggerSec_pin_mask;
extern volatile PORT_TYPE *triggerThird_pin_port;
extern volatile PINMASK_TYPE triggerThird_pin_mask;

extern byte triggerInterrupt;
extern byte triggerInterrupt2;
extern byte triggerInterrupt3;


extern byte fpPrimeTime; //The time (in seconds, based on currentStatus.secl) that the fuel pump started priming
extern uint8_t softLimitTime; //The time (in 0.1 seconds, based on seclx10) that the soft limiter started
extern volatile uint16_t mainLoopCount;
extern uint32_t revolutionTime; //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
extern volatile unsigned long timer5_overflow_count; //Increments every time counter 5 overflows. Used for the fast version of micros()
extern volatile unsigned long ms_counter; //A counter that increments once per ms
extern uint16_t fixedCrankingOverride;
extern volatile uint32_t toothHistory[TOOTH_LOG_SIZE];
extern volatile uint8_t compositeLogHistory[TOOTH_LOG_SIZE];
extern volatile unsigned int toothHistoryIndex;
extern unsigned long currentLoopTime; /**< The time (in uS) that the current mainloop started */
extern volatile uint16_t ignitionCount; /**< The count of ignition events that have taken place since the engine started */
//The below shouldn't be needed and probably should be cleaned up, but the Atmel SAM (ARM) boards use a specific type for the trigger edge values rather than a simple byte/int
#if defined(CORE_SAMD21)
  extern PinStatus primaryTriggerEdge;
  extern PinStatus secondaryTriggerEdge;
  extern PinStatus tertiaryTriggerEdge;
#else
  extern byte primaryTriggerEdge;
  extern byte secondaryTriggerEdge;
  extern byte tertiaryTriggerEdge;
#endif
extern int CRANK_ANGLE_MAX_IGN;
extern int CRANK_ANGLE_MAX_INJ;       ///< The number of crank degrees that the system track over. 360 for wasted / timed batch and 720 for sequential
extern volatile uint32_t runSecsX10;  /**< Counter of seconds since cranking commenced (similar to runSecs) but in increments of 0.1 seconds */
extern volatile uint32_t seclx10;     /**< Counter of seconds since powered commenced (similar to secl) but in increments of 0.1 seconds */
extern volatile byte HWTest_INJ;      /**< Each bit in this variable represents one of the injector channels and it's HW test status */
extern volatile byte HWTest_INJ_Pulsed; /**< Each bit in this variable represents one of the injector channels and it's 50% HW test status */
extern volatile byte HWTest_IGN;      /**< Each bit in this variable represents one of the ignition channels and it's HW test status */
extern volatile byte HWTest_IGN_Pulsed; /**< Each bit in this variable represents one of the ignition channels and it's 50% HW test status */
extern byte maxIgnOutputs;            /**< Number of ignition outputs being used by the current tune configuration */
extern byte maxInjOutputs;            /**< Number of injection outputs being used by the current tune configuration */
extern byte resetControl; ///< resetControl needs to be here (as global) because using the config page (4) directly can prevent burning the setting
extern volatile byte TIMER_mask;
extern volatile byte LOOP_TIMER;

//These functions all do checks on a pin to determine if it is already in use by another (higher importance) function
#define pinIsInjector(pin)  ( ((pin) == pinInjector1) || ((pin) == pinInjector2) || ((pin) == pinInjector3) || ((pin) == pinInjector4) || ((pin) == pinInjector5) || ((pin) == pinInjector6) || ((pin) == pinInjector7) || ((pin) == pinInjector8) )
#define pinIsIgnition(pin)  ( ((pin) == pinCoil1) || ((pin) == pinCoil2) || ((pin) == pinCoil3) || ((pin) == pinCoil4) || ((pin) == pinCoil5) || ((pin) == pinCoil6) || ((pin) == pinCoil7) || ((pin) == pinCoil8) )
//#define pinIsOutput(pin)    ( pinIsInjector((pin)) || pinIsIgnition((pin)) || ((pin) == pinFuelPump) || ((pin) == pinFan) || ((pin) == pinAirConComp) || ((pin) == pinAirConFan)|| ((pin) == pinVVT_1) || ((pin) == pinVVT_2) || ( ((pin) == pinBoost) && configPage6.boostEnabled) || ((pin) == pinIdle1) || ((pin) == pinIdle2) || ((pin) == pinTachOut) || ((pin) == pinStepperEnable) || ((pin) == pinStepperStep) )
#define pinIsSensor(pin)    ( ((pin) == pinCLT) || ((pin) == pinIAT) || ((pin) == pinMAP) || ((pin) == pinTPS) || ((pin) == pinO2) || ((pin) == pinBat) || (((pin) == pinFlex) && (configPage2.flexEnabled != 0)) )
//#define pinIsUsed(pin)      ( pinIsSensor((pin)) || pinIsOutput((pin)) || pinIsReserved((pin)) )

extern byte pinInjector1; //Output pin injector 1
extern byte pinInjector2; //Output pin injector 2
extern byte pinInjector3; //Output pin injector 3
extern byte pinInjector4; //Output pin injector 4
extern byte pinInjector5; //Output pin injector 5
extern byte pinInjector6; //Output pin injector 6
extern byte pinInjector7; //Output pin injector 7
extern byte pinInjector8; //Output pin injector 8
extern byte injectorOutputControl; //Specifies whether the injectors are controlled directly (Via an IO pin) or using something like the MC33810
extern byte pinCoil1; //Pin for coil 1
extern byte pinCoil2; //Pin for coil 2
extern byte pinCoil3; //Pin for coil 3
extern byte pinCoil4; //Pin for coil 4
extern byte pinCoil5; //Pin for coil 5
extern byte pinCoil6; //Pin for coil 6
extern byte pinCoil7; //Pin for coil 7
extern byte pinCoil8; //Pin for coil 8
extern byte ignitionOutputControl; //Specifies whether the coils are controlled directly (Via an IO pin) or using something like the MC33810
extern byte pinTrigger; //The CAS pin
extern byte pinTrigger2; //The Cam Sensor pin known as secondary input
extern byte pinTrigger3;	//the 2nd cam sensor pin known as tertiary input
extern byte pinTPS;//TPS input pin
extern byte pinMAP; //MAP sensor pin
extern byte pinEMAP; //EMAP sensor pin
extern byte pinMAP2; //2nd MAP sensor (Currently unused)
extern byte pinIAT; //IAT sensor pin
extern byte pinCLT; //CLS sensor pin
extern byte pinO2; //O2 Sensor pin
extern byte pinO2_2; //second O2 pin
extern byte pinBat; //Battery voltage pin
extern byte pinDisplayReset; // OLED reset pin
extern byte pinTachOut; //Tacho output
extern byte pinFuelPump; //Fuel pump on/off
extern byte pinIdle1; //Single wire idle control
extern byte pinIdle2; //2 wire idle control (Not currently used)
extern byte pinIdleUp; //Input for triggering Idle Up
extern byte pinIdleUpOutput; //Output that follows (normal or inverted) the idle up pin
extern byte pinCTPS; //Input for triggering closed throttle state
extern byte pinFuel2Input; //Input for switching to the 2nd fuel table
extern byte pinSpark2Input; //Input for switching to the 2nd ignition table
extern byte pinSpareTemp1; // Future use only
extern byte pinSpareTemp2; // Future use only
extern byte pinSpareOut1; //Generic output
extern byte pinSpareOut2; //Generic output
extern byte pinSpareOut3; //Generic output
extern byte pinSpareOut4; //Generic output
extern byte pinSpareOut5; //Generic output
extern byte pinSpareOut6; //Generic output
extern byte pinSpareHOut1; //spare high current output
extern byte pinSpareHOut2; // spare high current output
extern byte pinSpareLOut1; // spare low current output
extern byte pinSpareLOut2; // spare low current output
extern byte pinSpareLOut3;
extern byte pinSpareLOut4;
extern byte pinSpareLOut5;
extern byte pinBoost;
extern byte pinVVT_1;		// vvt output 1
extern byte pinVVT_2;		// vvt output 2
extern byte pinFan;       // Cooling fan output
extern byte pinStepperDir; //Direction pin for the stepper motor driver
extern byte pinStepperStep; //Step pin for the stepper motor driver
extern byte pinStepperEnable; //Turning the DRV8825 driver on/off
extern byte pinLaunch;
extern byte pinIgnBypass; //The pin used for an ignition bypass (Optional)
extern byte pinFlex; //Pin with the flex sensor attached
extern byte pinVSS; 
extern byte pinBaro; //Pin that an external barometric pressure sensor is attached to (If used)
extern byte pinResetControl; // Output pin used control resetting the Arduino
extern byte pinFuelPressure;
extern byte pinOilPressure;
extern byte pinWMIEmpty; // Water tank empty sensor
extern byte pinWMIIndicator; // No water indicator bulb
extern byte pinWMIEnabled; // ON-OFF output to relay/pump/solenoid
extern byte pinMC33810_1_CS;
extern byte pinMC33810_2_CS;
extern byte pinSDEnable; //Input for manually enabling SD logging
#ifdef USE_SPI_EEPROM
  extern byte pinSPIFlash_CS;
#endif
extern byte pinAirConComp;    // Air conditioning compressor output
extern byte pinAirConFan;    // Stand-alone air conditioning fan output
extern byte pinAirConRequest; // Air conditioning request input

/* global variables */ // from speeduino.ino
//#ifndef UNIT_TEST

//#endif

extern struct statuses currentStatus; //The global status object
extern struct config2 configPage2;
extern struct config4 configPage4;
extern struct config6 configPage6;
extern struct config9 configPage9;
extern struct config10 configPage10;
extern struct config13 configPage13;
extern struct config15 configPage15;
//extern byte cltCalibrationTable[CALIBRATION_TABLE_SIZE]; /**< An array containing the coolant sensor calibration values */
//extern byte iatCalibrationTable[CALIBRATION_TABLE_SIZE]; /**< An array containing the inlet air temperature sensor calibration values */
//extern byte o2CalibrationTable[CALIBRATION_TABLE_SIZE]; /**< An array containing the O2 sensor calibration values */

extern uint16_t cltCalibration_bins[32];
extern uint16_t cltCalibration_values[32];
extern uint16_t iatCalibration_bins[32];
extern uint16_t iatCalibration_values[32];
extern uint16_t o2Calibration_bins[32];
extern uint8_t  o2Calibration_values[32]; // Note 8-bit values
extern struct table2D cltCalibrationTable; /**< A 32 bin array containing the coolant temperature sensor calibration values */
extern struct table2D iatCalibrationTable; /**< A 32 bin array containing the inlet air temperature sensor calibration values */
extern struct table2D o2CalibrationTable; /**< A 32 bin array containing the O2 sensor calibration values */

bool pinIsOutput(byte pin);
bool pinIsUsed(byte pin);

#endif // GLOBALS_H
