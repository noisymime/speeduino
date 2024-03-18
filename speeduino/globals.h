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
#include <Arduino.h>
#include "table2d.h"
#include "table3d.h"
#include <assert.h>
#include "src/FastCRC/FastCRC.h"

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
    #define BOARD_MAX_ADC_PINS  22 //Number of analog pins
  #elif defined(__IMXRT1062__)
    #define CORE_TEENSY41
    #define BOARD_H "board_teensy41.h"
    #define BOARD_MAX_ADC_PINS  17 //Number of analog pins
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

//Handy bitsetting macros
#define BIT_SET(a,b) ((a) |= (1U<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1U<<(b)))
#define BIT_CHECK(var,pos) !!((var) & (1U<<(pos)))
#define BIT_TOGGLE(var,pos) ((var)^= 1UL << (pos))
#define BIT_WRITE(var, pos, bitvalue) ((bitvalue) ? BIT_SET((var), (pos)) : bitClear((var), (pos)))

#define CRANK_ANGLE_MAX (max(CRANK_ANGLE_MAX_IGN, CRANK_ANGLE_MAX_INJ))

#define interruptSafe(c) (noInterrupts(); {c} interrupts();) //Wraps any code between nointerrupt and interrupt calls

#define MICROS_PER_SEC INT32_C(1000000)
#define MICROS_PER_MIN INT32_C(MICROS_PER_SEC*60U)
#define MICROS_PER_HOUR INT32_C(MICROS_PER_MIN*60U)

#define SERIAL_PORT_PRIMARY   0
#define SERIAL_PORT_SECONDARY 3

//Define the load algorithm
#define LOAD_SOURCE_MAP         0
#define LOAD_SOURCE_TPS         1
#define LOAD_SOURCE_IMAPEMAP    2

//Define bit positions within engine variable
#define BIT_ENGINE_RUN      0   // Engine running
#define BIT_ENGINE_CRANK    1   // Engine cranking
#define BIT_ENGINE_ASE      2   // after start enrichment (ASE)
#define BIT_ENGINE_WARMUP   3   // Engine in warmup
#define BIT_ENGINE_ACC      4   // in acceleration mode (TPS accel)
#define BIT_ENGINE_DCC      5   // in deceleration mode
#define BIT_ENGINE_MAPACC   6   // MAP acceleration mode
#define BIT_ENGINE_MAPDCC   7   // MAP deceleration mode

//Define masks for Status1
#define BIT_STATUS1_INJ1           0  //inj1
#define BIT_STATUS1_INJ2           1  //inj2
#define BIT_STATUS1_INJ3           2  //inj3
#define BIT_STATUS1_INJ4           3  //inj4
#define BIT_STATUS1_DFCO           4  //Deceleration fuel cutoff
#define BIT_STATUS1_BOOSTCUT       5  //Fuel component of MAP based boost cut out
#define BIT_STATUS1_TOOTHLOG1READY 6  //Used to flag if tooth log 1 is ready
#define BIT_STATUS1_TOOTHLOG2READY 7  //Used to flag if tooth log 2 is ready (Log is not currently used)

//Define masks for spark variable
#define BIT_SPARK_HLAUNCH         0  //Hard Launch indicator
#define BIT_SPARK_SLAUNCH         1  //Soft Launch indicator
#define BIT_SPARK_HRDLIM          2  //Hard limiter indicator
#define BIT_SPARK_SFTLIM          3  //Soft limiter indicator
#define BIT_SPARK_BOOSTCUT        4  //Spark component of MAP based boost cut out
#define BIT_SPARK_ERROR           5  // Error is detected
#define BIT_SPARK_IDLE            6  // idle on
#define BIT_SPARK_SYNC            7  // Whether engine has sync or not

#define BIT_SPARK2_FLATSH         0  //Flat shift hard cut
#define BIT_SPARK2_FLATSS         1  //Flat shift soft cut
#define BIT_SPARK2_SPARK2_ACTIVE  2
#define BIT_SPARK2_UNUSED4        3
#define BIT_SPARK2_UNUSED5        4
#define BIT_SPARK2_UNUSED6        5
#define BIT_SPARK2_UNUSED7        6
#define BIT_SPARK2_UNUSED8        7

#define BIT_TIMER_1HZ             0
#define BIT_TIMER_4HZ             1
#define BIT_TIMER_10HZ            2
#define BIT_TIMER_15HZ            3
#define BIT_TIMER_30HZ            4
#define BIT_TIMER_200HZ           6
#define BIT_TIMER_1KHZ            7

#define BIT_STATUS3_RESET_PREVENT 0 //Indicates whether reset prevention is enabled
#define BIT_STATUS3_NITROUS       1
#define BIT_STATUS3_FUEL2_ACTIVE  2
#define BIT_STATUS3_VSS_REFRESH   3
#define BIT_STATUS3_HALFSYNC      4 //shows if there is only sync from primary trigger, but not from secondary.
#define BIT_STATUS3_NSQUIRTS1     5
#define BIT_STATUS3_NSQUIRTS2     6
#define BIT_STATUS3_NSQUIRTS3     7

#define BIT_STATUS4_WMI_EMPTY     0 //Indicates whether the WMI tank is empty
#define BIT_STATUS4_VVT1_ERROR    1 //VVT1 cam angle within limits or not
#define BIT_STATUS4_VVT2_ERROR    2 //VVT2 cam angle within limits or not
#define BIT_STATUS4_FAN           3 //Fan Status
#define BIT_STATUS4_BURNPENDING   4
#define BIT_STATUS4_STAGING_ACTIVE 5
#define BIT_STATUS4_COMMS_COMPAT  6
#define BIT_STATUS4_ALLOW_LEGACY_COMMS       7

#define BIT_AIRCON_REQUEST        0 //Indicates whether the A/C button is pressed
#define BIT_AIRCON_COMPRESSOR     1 //Indicates whether the A/C compressor is running
#define BIT_AIRCON_RPM_LOCKOUT    2 //Indicates the A/C is locked out due to the RPM being too high/low, or the post-high/post-low-RPM "stand-down" lockout period
#define BIT_AIRCON_TPS_LOCKOUT    3 //Indicates the A/C is locked out due to high TPS, or the post-high-TPS "stand-down" lockout period
#define BIT_AIRCON_TURNING_ON     4 //Indicates the A/C request is on (i.e. A/C button pressed), the lockouts are off, however the start delay has not yet elapsed. This gives the idle up time to kick in before the compressor.
#define BIT_AIRCON_CLT_LOCKOUT    5 //Indicates the A/C is locked out either due to high coolant temp.
#define BIT_AIRCON_FAN            6 //Indicates whether the A/C fan is running
#define BIT_AIRCON_UNUSED8        7

#define VALID_MAP_MAX 1022 //The largest ADC value that is valid for the MAP sensor
#define VALID_MAP_MIN 2 //The smallest ADC value that is valid for the MAP sensor

#ifndef UNIT_TEST 
#define TOOTH_LOG_SIZE      127
#else
#define TOOTH_LOG_SIZE      1
#endif

#define O2_CALIBRATION_PAGE   2U
#define IAT_CALIBRATION_PAGE  1U
#define CLT_CALIBRATION_PAGE  0U

// note the sequence of these defines which refernce the bits used in a byte has moved when the third trigger & engine cycle was incorporated
#define COMPOSITE_LOG_PRI   0
#define COMPOSITE_LOG_SEC   1
#define COMPOSITE_LOG_THIRD 2 
#define COMPOSITE_LOG_TRIG 3
#define COMPOSITE_LOG_SYNC 4
#define COMPOSITE_ENGINE_CYCLE 5

#define EGO_TYPE_OFF      0
#define EGO_TYPE_NARROW   1
#define EGO_TYPE_WIDE     2

#define INJ_TYPE_PORT 0
#define INJ_TYPE_TBODY 1

#define INJ_PAIRED 0
#define INJ_SEMISEQUENTIAL 1
#define INJ_BANKED          2
#define INJ_SEQUENTIAL      3

#define INJ_PAIR_13_24      0
#define INJ_PAIR_14_23      1

#define OUTPUT_CONTROL_DIRECT   0
#define OUTPUT_CONTROL_MC33810  10

#define IGN_MODE_WASTED     0U
#define IGN_MODE_SINGLE     1U
#define IGN_MODE_WASTEDCOP  2U
#define IGN_MODE_SEQUENTIAL 3U
#define IGN_MODE_ROTARY     4U

#define SEC_TRIGGER_SINGLE  0
#define SEC_TRIGGER_4_1     1
#define SEC_TRIGGER_POLL    2
#define SEC_TRIGGER_5_3_2   3
#define SEC_TRIGGER_TOYOTA_3  4

#define ROTARY_IGN_FC       0
#define ROTARY_IGN_FD       1
#define ROTARY_IGN_RX8      2

#define BOOST_MODE_SIMPLE   0
#define BOOST_MODE_FULL     1

#define EN_BOOST_CONTROL_BARO   0
#define EN_BOOST_CONTROL_FIXED  1

#define WMI_MODE_SIMPLE       0
#define WMI_MODE_PROPORTIONAL 1
#define WMI_MODE_OPENLOOP     2
#define WMI_MODE_CLOSEDLOOP   3

#define HARD_CUT_FULL       0
#define HARD_CUT_ROLLING    1

#define EVEN_FIRE           0
#define ODD_FIRE            1

#define EGO_ALGORITHM_SIMPLE  0
#define EGO_ALGORITHM_PID     2

#define STAGING_MODE_TABLE  0
#define STAGING_MODE_AUTO   1

#define NITROUS_OFF         0
#define NITROUS_STAGE1      1
#define NITROUS_STAGE2      2
#define NITROUS_BOTH        3

#define PROTECT_CUT_OFF     0
#define PROTECT_CUT_IGN     1
#define PROTECT_CUT_FUEL    2
#define PROTECT_CUT_BOTH    3
#define PROTECT_IO_ERROR    7

#define AE_MODE_TPS         0
#define AE_MODE_MAP         1

#define AE_MODE_MULTIPLIER  0
#define AE_MODE_ADDER       1

#define KNOCK_MODE_OFF      0
#define KNOCK_MODE_DIGITAL  1
#define KNOCK_MODE_ANALOG   2

#define FUEL2_MODE_OFF      0
#define FUEL2_MODE_MULTIPLY 1
#define FUEL2_MODE_ADD      2
#define FUEL2_MODE_CONDITIONAL_SWITCH   3
#define FUEL2_MODE_INPUT_SWITCH 4

#define SPARK2_MODE_OFF      0
#define SPARK2_MODE_MULTIPLY 1
#define SPARK2_MODE_ADD      2
#define SPARK2_MODE_CONDITIONAL_SWITCH   3
#define SPARK2_MODE_INPUT_SWITCH 4

#define FUEL2_CONDITION_RPM 0
#define FUEL2_CONDITION_MAP 1
#define FUEL2_CONDITION_TPS 2
#define FUEL2_CONDITION_ETH 3

#define SPARK2_CONDITION_RPM 0
#define SPARK2_CONDITION_MAP 1
#define SPARK2_CONDITION_TPS 2
#define SPARK2_CONDITION_ETH 3

#define RESET_CONTROL_DISABLED             0
#define RESET_CONTROL_PREVENT_WHEN_RUNNING 1
#define RESET_CONTROL_PREVENT_ALWAYS       2
#define RESET_CONTROL_SERIAL_COMMAND       3

#define OPEN_LOOP_BOOST     0
#define CLOSED_LOOP_BOOST   1

#define SOFT_LIMIT_FIXED        0
#define SOFT_LIMIT_RELATIVE     1

#define VVT_MODE_ONOFF      0
#define VVT_MODE_OPEN_LOOP  1
#define VVT_MODE_CLOSED_LOOP 2
#define VVT_LOAD_MAP      0
#define VVT_LOAD_TPS      1

#define MULTIPLY_MAP_MODE_OFF   0
#define MULTIPLY_MAP_MODE_BARO  1
#define MULTIPLY_MAP_MODE_100   2

#define FOUR_STROKE         0U
#define TWO_STROKE          1U

#define GOING_LOW         0
#define GOING_HIGH        1

#define BATTV_COR_MODE_WHOLE 0
#define BATTV_COR_MODE_OPENTIME 1

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

#define LOGGER_DISABLED                 0
#define LOGGER_CSV                      1
#define LOGGER_BINARY                   2

#define LOGGER_RATE_1HZ                 0
#define LOGGER_RATE_4HZ                 1
#define LOGGER_RATE_10HZ                2
#define LOGGER_RATE_30HZ                3

#define LOGGER_FILENAMING_OVERWRITE     0
#define LOGGER_FILENAMING_DATETIME      1
#define LOGGER_FILENAMING_SEQENTIAL     2

extern const char TSfirmwareVersion[] PROGMEM;

extern const byte data_structure_version; //This identifies the data structure when reading / writing. Now in use: CURRENT_DATA_VERSION (migration on-the fly) ?

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
extern unsigned long revolutionTime; //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
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


/** The status struct with current values for all 'live' variables.
* In current version this is 64 bytes. Instantiated as global currentStatus.
* int *ADC (Analog-to-digital value / count) values contain the "raw" value from AD conversion, which get converted to
* unit based values in similar variable(s) without ADC part in name (see sensors.ino for reading of sensors).
*/
struct statuses {
  volatile bool hasSync : 1; /**< Flag for crank/cam position being known by decoders (See decoders.ino).
  This is used for sanity checking e.g. before logging tooth history or reading some sensors and computing readings. */
  bool initialisationComplete : 1; //Tracks whether the setup() function has run completely
  bool clutchTrigger : 1;
  bool previousClutchTrigger : 1;
  volatile bool fpPrimed : 1; //Tracks whether or not the fuel pump priming has been completed yet
  volatile bool injPrimed : 1; //Tracks whether or not the injector priming has been completed yet
  volatile bool tachoSweepEnabled : 1;
  volatile bool tachoAlt : 1;
    
  uint16_t RPM;   ///< RPM - Current Revs per minute
  byte RPMdiv100; ///< RPM value scaled (divided by 100) to fit a byte (0-255, e.g. 12000 => 120)
  long longRPM;   ///< RPM as long int (gets assigned to / maintained in statuses.RPM as well)
  uint16_t mapADC;
  int baroADC;
  long MAP;     ///< Manifold absolute pressure. Has to be a long for PID calcs (Boost control)
  int16_t EMAP; ///< EMAP ... (See @ref config6.useEMAP for EMAP enablement)
  uint16_t EMAPADC;
  byte baro;   ///< Barometric pressure is simply the initial MAP reading, taken before the engine is running. Alternatively, can be taken from an external sensor
  byte TPS;    /**< The current TPS reading (0% - 100%). Is the tpsADC value after the calibration is applied */
  byte tpsADC; /**< byte (valued: 0-255) representation of the TPS. Downsampled from the original 10-bit (0-1023) reading, but before any calibration is applied */
  int16_t tpsDOT; /**< TPS delta over time. Measures the % per second that the TPS is changing. Note that is signed value, because TPSdot can be also negative */
  byte TPSlast; /**< The previous TPS reading */
  int16_t mapDOT; /**< MAP delta over time. Measures the kpa per second that the MAP is changing. Note that is signed value, because MAPdot can be also negative */
  volatile int rpmDOT; /**< RPM delta over time (RPM increase / s ?) */
  byte VE;     /**< The current VE value being used in the fuel calculation. Can be the same as VE1 or VE2, or a calculated value of both. */
  byte VE1;    /**< The VE value from fuel table 1 */
  byte VE2;    /**< The VE value from fuel table 2, if in use (and required conditions are met) */
  byte O2;     /**< Primary O2 sensor reading */
  byte O2_2;   /**< Secondary O2 sensor reading */
  int coolant; /**< Coolant temperature reading */
  int cltADC;
  int IAT;     /**< Inlet air temperature reading */
  int iatADC;
  int batADC;
  int O2ADC;
  int O2_2ADC;
  int dwell;          ///< dwell (coil primary winding/circuit on) time (in ms * 10 ? See @ref correctionsDwell)
  volatile int16_t actualDwell;    ///< actual dwell time if new ignition mode is used (in uS)
  byte dwellCorrection; /**< The amount of correction being applied to the dwell time (in unit ...). */
  byte battery10;     /**< The current BRV in volts (multiplied by 10. Eg 12.5V = 125) */
  int8_t advance;     /**< The current advance value being used in the spark calculation. Can be the same as advance1 or advance2, or a calculated value of both */
  int8_t advance1;    /**< The advance value from ignition table 1 */
  int8_t advance2;    /**< The advance value from ignition table 2 */
  uint16_t corrections; /**< The total current corrections % amount */
  uint16_t AEamount;    /**< The amount of acceleration enrichment currently being applied. 100=No change. Varies above 255 */
  byte egoCorrection; /**< The amount of closed loop AFR enrichment currently being applied */
  byte wueCorrection; /**< The amount of warmup enrichment currently being applied */
  byte batCorrection; /**< The amount of battery voltage enrichment currently being applied */
  byte iatCorrection; /**< The amount of inlet air temperature adjustment currently being applied */
  byte baroCorrection; /**< The amount of correction being applied for the current baro reading */
  byte launchCorrection;   /**< The amount of correction being applied if launch control is active */
  byte flexCorrection;     /**< Amount of correction being applied to compensate for ethanol content */
  byte fuelTempCorrection; /**< Amount of correction being applied to compensate for fuel temperature */
  int8_t flexIgnCorrection;/**< Amount of additional advance being applied based on flex. Note the type as this allows for negative values */
  byte afrTarget;    /**< Current AFR Target looked up from AFR target table (x10 ? See @ref afrTable)*/
  byte CLIdleTarget; /**< The target idle RPM (when closed loop idle control is active) */
  bool idleUpActive; /**< Whether the externally controlled idle up is currently active */
  bool CTPSActive;   /**< Whether the externally controlled closed throttle position sensor is currently active */
  volatile byte ethanolPct; /**< Ethanol reading (if enabled). 0 = No ethanol, 100 = pure ethanol. Eg E85 = 85. */
  volatile int8_t fuelTemp;
  unsigned long AEEndTime; /**< The target end time used whenever AE (acceleration enrichment) is turned on */
  volatile byte status1; ///< Status bits (See BIT_STATUS1_* defines on top of this file)
  volatile byte spark;   ///< Spark status/control indicator bits (launch control, boost cut, spark errors, See BIT_SPARK_* defines)
  volatile byte spark2;  ///< Spark 2 ... (See also @ref config10 spark2* members and BIT_SPARK2_* defines)
  uint8_t engine; ///< Engine status bits (See BIT_ENGINE_* defines on top of this file)
  unsigned int PW1; ///< In uS
  unsigned int PW2; ///< In uS
  unsigned int PW3; ///< In uS
  unsigned int PW4; ///< In uS
  unsigned int PW5; ///< In uS
  unsigned int PW6; ///< In uS
  unsigned int PW7; ///< In uS
  unsigned int PW8; ///< In uS
  volatile byte runSecs; /**< Counter of seconds since cranking commenced (Maxes out at 255 to prevent overflow) */
  volatile byte secl; /**< Counter incrementing once per second. Will overflow after 255 and begin again. This is used by TunerStudio to maintain comms sync */
  volatile uint32_t loopsPerSecond; /**< A performance indicator showing the number of main loops that are being executed each second */ 
  bool launchingSoft; /**< Indicator showing whether soft launch control adjustments are active */
  bool launchingHard; /**< Indicator showing whether hard launch control adjustments are active */
  uint16_t freeRAM;
  unsigned int clutchEngagedRPM; /**< The RPM at which the clutch was last depressed. Used for distinguishing between launch control and flat shift */ 
  bool flatShiftingHard;
  volatile uint32_t startRevolutions; /**< A counter for how many revolutions have been completed since sync was achieved. */
  uint16_t boostTarget;
  byte testOutputs;   ///< Test Output bits (only first bit used/tested ?)
  bool testActive;    // Not in use ? Replaced by testOutputs ?
  uint16_t boostDuty; ///< Boost Duty percentage value * 100 to give 2 points of precision
  byte idleLoad;      ///< Either the current steps or current duty cycle for the idle control
  uint16_t canin[16]; ///< 16bit raw value of selected canin data for channels 0-15
  uint8_t current_caninchannel = 0; /**< Current CAN channel, defaults to 0 */
  uint16_t crankRPM = 400; /**< The actual cranking RPM limit. This is derived from the value in the config page, but saves us multiplying it every time it's used (Config page value is stored divided by 10) */
  volatile byte status3; ///< Status bits (See BIT_STATUS3_* defines on top of this file)
  int16_t flexBoostCorrection; /**< Amount of boost added based on flex */
  byte nitrous_status;
  byte nSquirts;  ///< Number of injector squirts per cycle (per injector)
  byte nChannels; /**< Number of fuel and ignition channels.  */
  int16_t fuelLoad;
  int16_t fuelLoad2;
  int16_t ignLoad;
  int16_t ignLoad2;
  bool fuelPumpOn; /**< Indicator showing the current status of the fuel pump */
  volatile byte syncLossCounter;
  byte knockRetard;
  bool knockActive;
  bool toothLogEnabled;
  byte compositeTriggerUsed; // 0 means composite logger disabled, 2 means use secondary input (1st cam), 3 means use tertiary input (2nd cam), 4 means log both cams together
  int16_t vvt1Angle; //Has to be a long for PID calcs (CL VVT control)
  byte vvt1TargetAngle;
  long vvt1Duty; //Has to be a long for PID calcs (CL VVT control)
  uint16_t injAngle;
  byte ASEValue;
  uint16_t vss;      /**< Current speed reading. Natively stored in kph and converted to mph in TS if required */
  bool idleUpOutputActive; /**< Whether the idle up output is currently active */
  byte gear;         /**< Current gear (Calculated from vss) */
  byte fuelPressure; /**< Fuel pressure in PSI */
  byte oilPressure;  /**< Oil pressure in PSI */
  byte engineProtectStatus;
  byte fanDuty;
  byte wmiPW;
  volatile byte status4; ///< Status bits (See BIT_STATUS4_* defines on top of this file)
  int16_t vvt2Angle; //Has to be a long for PID calcs (CL VVT control)
  byte vvt2TargetAngle;
  long vvt2Duty; //Has to be a long for PID calcs (CL VVT control)
  byte outputsStatus;
  byte TS_SD_Status; //TunerStudios SD card status
  byte airConStatus;
};

static inline bool HasAnySync(const statuses &status) {
  return status.hasSync || BIT_CHECK(status.status3, BIT_STATUS3_HALFSYNC);
}

/** Page 2 of the config - mostly variables that are required for fuel.
 * These are "non-live" EFI setting, engine and "system" variables that remain fixed once sent
 * (and stored to e.g. EEPROM) from configuration/tuning SW (from outside by USBserial/bluetooth).
 * Contains a lots of *Min, *Max (named) variables to constrain values to sane ranges.
 * See the ini file for further reference.
 * 
 */
struct config2 {

  byte aseTaperTime;
  byte aeColdPct;  //AE cold clt modifier %
  byte aeColdTaperMin; //AE cold modifier, taper start temp (full modifier, was ASE in early versions)
  byte aeMode : 2;      /**< Acceleration Enrichment mode. 0 = TPS, 1 = MAP. Values 2 and 3 reserved for potential future use (ie blended TPS / MAP) */
  byte battVCorMode : 1;
  byte SoftLimitMode : 1;
  byte useTachoSweep : 1;
  byte aeApplyMode : 1; ///< Acceleration enrichment calc mode: 0 = Multiply | 1 = Add (AE_MODE_ADDER)
  byte multiplyMAP : 2; ///< MAP value processing: 0 = off, 1 = div by currentStatus.baro, 2 = div by 100 (to gain usable value)
  byte wueValues[10];   ///< Warm up enrichment array (10 bytes, transferred to @ref WUETable)
  byte crankingPct;     ///< Cranking enrichment (See @ref config10, updates.ino)
  byte pinMapping;      ///< The board / ping mapping number / id to be used (See: @ref setPinMapping in init.ino)
  byte tachoPin : 6;    ///< Custom pin setting for tacho output (if != 0, override copied to pinTachOut, which defaults to board assigned tach pin)
  byte tachoDiv : 2;    ///< Whether to change the tacho speed ("half speed tacho" ?)
  byte tachoDuration;   //The duration of the tacho pulse in mS
  byte maeThresh;       /**< The MAPdot threshold that must be exceeded before AE is engaged */
  byte taeThresh;       /**< The TPSdot threshold that must be exceeded before AE is engaged */
  byte aeTime;
  byte taeMinChange;    /**< The minimum change in TPS that must be made before AE is engaged */
  byte maeMinChange;    /**< The minimum change in MAP that must be made before AE is engaged */

  //Display config bits
  byte displayB1 : 4;   //23
  byte displayB2 : 4;

  byte reqFuel;       //24
  byte divider;
  byte injTiming : 1; ///< Injector timing (aka. injector staging) 0=simultaneous, 1=alternating
  byte crkngAddCLTAdv : 1;
  byte includeAFR : 1; //< Enable AFR compensation ? (See also @ref config2.incorporateAFR)
  byte hardCutType : 1;
  byte ignAlgorithm : 3;
  byte indInjAng : 1;
  byte injOpen;     ///< Injector opening time (ms * 10)
  uint16_t injAng[4];

  //config1 in ini
  byte mapSample : 2;  ///< MAP sampling method (0=Instantaneous, 1=Cycle Average, 2=Cycle Minimum, 4=Ign. event average, See sensors.ino)
  byte strokes : 1;    ///< Engine cycle type: four-stroke (0) / two-stroke (1)
  byte injType : 1;    ///< Injector type 0=Port (INJ_TYPE_PORT), 1=Throttle Body / TBI (INJ_TYPE_TBODY)
  byte nCylinders : 4; ///< Number of cylinders

  //config2 in ini
  byte fuelAlgorithm : 3;///< Fuel algorithm - 0=Manifold pressure/MAP (LOAD_SOURCE_MAP, default, proven), 1=Throttle/TPS (LOAD_SOURCE_TPS), 2=IMAP/EMAP (LOAD_SOURCE_IMAPEMAP)
  byte fixAngEnable : 1; ///< Whether fixed/locked timing is enabled (0=disable, 1=enable, See @ref configPage4.FixAng)
  byte nInjectors : 4;   ///< Number of injectors


  //config3 in ini
  byte engineType : 1;  ///< Engine crank/ign phasing type: 0=even fire, 1=odd fire
  byte flexEnabled : 1; ///< Enable Flex fuel sensing (pin / interrupt)
  byte legacyMAP  : 1;  ///< Legacy MAP reading behaviour
  byte baroCorr : 1;    // Unused ?
  byte injLayout : 2;   /**< Injector Layout - 0=INJ_PAIRED (number outputs == number cyls/2, timed over 1 crank rev), 1=INJ_SEMISEQUENTIAL (like paired, but number outputs == number cyls, only for 4 cyl),
                         2=INJ_BANKED (2 outputs are used), 3=INJ_SEQUENTIAL (number outputs == number cyls, timed over full cycle, 2 crank revs) */
  byte perToothIgn : 1; ///< Experimental / New ignition mode ... (?) (See decoders.ino)
  byte dfcoEnabled : 1; ///< Whether or not DFCO (deceleration fuel cut-off) is turned on

  byte aeColdTaperMax;  ///< AE cold modifier, taper end temp (no modifier applied, was primePulse in early versions)
  byte dutyLim;
  byte flexFreqLow; //Lowest valid frequency reading from the flex sensor
  byte flexFreqHigh; //Highest valid frequency reading from the flex sensor

  byte boostMaxDuty;
  byte tpsMin;
  byte tpsMax;
  int8_t mapMin; //Must be signed
  uint16_t mapMax;
  byte fpPrime; ///< Time (In seconds) that the fuel pump should be primed for on power up
  byte stoich;  ///< Stoichiometric ratio (x10, so e.g. 14.7 => 147)
  uint16_t oddfire2; ///< The ATDC angle of channel 2 for oddfire
  uint16_t oddfire3; ///< The ATDC angle of channel 3 for oddfire
  uint16_t oddfire4; ///< The ATDC angle of channel 4 for oddfire

  byte idleUpPin : 6;
  byte idleUpPolarity : 1;
  byte idleUpEnabled : 1;

  byte idleUpAdder;
  byte aeTaperMin;
  byte aeTaperMax;

  byte iacCLminValue;
  byte iacCLmaxValue;
  byte boostMinDuty;

  int8_t baroMin; //Must be signed
  uint16_t baroMax;

  int8_t EMAPMin; //Must be signed
  uint16_t EMAPMax;

  byte fanWhenOff : 1;      ///< Allow running fan with engine off: 0 = Only run fan when engine is running, 1 = Allow even with engine off
  byte fanWhenCranking : 1; ///< Set whether the fan output will stay on when the engine is cranking (0=force off, 1=allow on)
  byte useDwellMap : 1;     ///< Setting to change between fixed dwell value and dwell map (0=Fixed value from @ref configPage4.dwellRun, 1=Use @ref dwellTable)
  byte fanEnable : 2;       ///< Fan mode. 0=Off, 1=On/Off, 2=PWM
  byte rtc_mode : 2;        // Unused ?
  byte incorporateAFR : 1;  ///< Enable AFR target (stoich/afrtgt) compensation in PW calculation
  byte asePct[4];           ///< Afterstart enrichment values (%)
  byte aseCount[4];         ///< Afterstart enrichment cycles. This is the number of ignition cycles that the afterstart enrichment % lasts for
  byte aseBins[4];          ///< Afterstart enrichment temperatures (x-axis) for (target) enrichment values
  byte primePulse[4];//Priming pulsewidth values (mS, copied to @ref PrimingPulseTable)
  byte primeBins[4]; //Priming temperatures (source,x-axis)

  byte CTPSPin : 6;
  byte CTPSPolarity : 1;
  byte CTPSEnabled : 1;

  byte idleAdvEnabled : 2;
  byte idleAdvAlgorithm : 1;
  byte idleAdvDelay : 5;
  
  byte idleAdvRPM;
  byte idleAdvTPS;

  byte injAngRPM[4];

  byte idleTaperTime;
  byte dfcoDelay;
  byte dfcoMinCLT;

  //VSS Stuff
  byte vssMode : 2; ///< VSS (Vehicle speed sensor) mode (0=none, 1=CANbus, 2,3=Interrupt driven)
  byte vssPin : 6; ///< VSS (Vehicle speed sensor) pin number
  
  uint16_t vssPulsesPerKm; ///< VSS (Vehicle speed sensor) pulses per Km
  byte vssSmoothing;
  uint16_t vssRatio1;
  uint16_t vssRatio2;
  uint16_t vssRatio3;
  uint16_t vssRatio4;
  uint16_t vssRatio5;
  uint16_t vssRatio6;

  byte idleUpOutputEnabled : 1;
  byte idleUpOutputInv : 1;
  byte idleUpOutputPin  : 6;

  byte tachoSweepMaxRPM;
  byte primingDelay;

  byte iacTPSlimit;
  byte iacRPMlimitHysteresis;

  int8_t rtc_trim;
  byte idleAdvVss;
  byte mapSwitchPoint;

  byte canBMWCluster : 1;
  byte canVAGCluster : 1;
  byte canWBO : 2 ;
  byte vssAuxCh : 4;

  byte decelAmount;

#if defined(CORE_AVR)
  };
#else
  } __attribute__((__packed__)); //The 32 bit systems require all structs to be fully packed
#endif

/** Page 4 of the config - variables required for ignition and rpm/crank phase /cam phase decoding.
* See the ini file for further reference.
*/
struct config4 {

  int16_t triggerAngle; ///< Angle (ATDC) when tooth No:1 on the primary wheel sends signal (-360 to +360 deg.)
  int8_t FixAng; ///< Fixed Ignition angle value (enabled by @ref configPage2.fixAngEnable, copied to ignFixValue, Negative values allowed, See corrections.ino)
  int8_t CrankAng; ///< Fixed start-up/cranking ignition angle (See: corrections.ino)
  byte TrigAngMul; ///< Multiplier for non evenly divisible tooth counts.

  byte TrigEdge : 1;  ///< Primary (RPM1) Trigger Edge - 0 - RISING, 1 = FALLING (Copied from this config to primaryTriggerEdge)
  byte TrigSpeed : 1; ///< Primary (RPM1) Trigger speed - 0 = crank speed (CRANK_SPEED), 1 = cam speed (CAM_SPEED), See decoders.ino
  byte IgInv : 1;     ///< Ignition signal invert (?) (GOING_LOW=0 (default by init.ino) / GOING_HIGH=1 )
  byte TrigPattern : 5; ///< Decoder configured (DECODER_MISSING_TOOTH, DECODER_BASIC_DISTRIBUTOR, DECODER_GM7X, ... See init.ino)

  byte TrigEdgeSec : 1; ///< Secondary (RPM2) Trigger Edge (See RPM1)
  byte fuelPumpPin : 6; ///< Fuel pump pin (copied as override to pinFuelPump, defaults to board default, See: init.ino)
  byte useResync : 1;

  byte sparkDur; ///< Spark duration in ms * 10
  byte trigPatternSec : 7; ///< Mode for Missing tooth secondary trigger - 0=single tooth cam wheel (SEC_TRIGGER_SINGLE), 1=4-1 (SEC_TRIGGER_4_1) or 2=poll level mode (SEC_TRIGGER_POLL)
  byte PollLevelPolarity : 1; //for poll level cam trigger. Sets if the cam trigger is supposed to be high or low for revolution one.
  uint8_t bootloaderCaps; //Capabilities of the bootloader over stock. e.g., 0=Stock, 1=Reset protection, etc.

  byte resetControlConfig : 2; /** Which method of reset control to use - 0=Disabled (RESET_CONTROL_DISABLED), 1=Prevent When Running (RESET_CONTROL_PREVENT_WHEN_RUNNING),
     2=Prevent Always (RESET_CONTROL_PREVENT_ALWAYS), 3=Serial Command (RESET_CONTROL_SERIAL_COMMAND) - Copied to resetControl (See init.ino, utilities.ino) */
  byte resetControlPin : 6;

  byte StgCycles; //The number of initial cycles before the ignition should fire when first cranking

  byte boostType : 1; ///< Boost Control type: 0=Open loop (OPEN_LOOP_BOOST), 1=closed loop (CLOSED_LOOP_BOOST)
  byte useDwellLim : 1; //Whether the dwell limiter is off or on
  byte sparkMode : 3; /** Ignition/Spark output mode - 0=Wasted spark (IGN_MODE_WASTED), 1=single channel (IGN_MODE_SINGLE),
      2=Wasted COP (IGN_MODE_WASTEDCOP), 3=Sequential (IGN_MODE_SEQUENTIAL), 4=Rotary (IGN_MODE_ROTARY) */
  byte triggerFilter : 2; //The mode of trigger filter being used (0=Off, 1=Light (Not currently used), 2=Normal, 3=Aggressive)
  byte ignCranklock : 1; //Whether or not the ignition timing during cranking is locked to a CAS (crank) pulse. Only currently valid for Basic distributor and 4G63.

  byte dwellCrank;    ///< Dwell time whilst cranking
  byte dwellRun;      ///< Dwell time whilst running
  byte triggerTeeth;  ///< The full count of teeth on the trigger wheel if there were no gaps
  byte triggerMissingTeeth; ///< The size of the tooth gap (ie number of missing teeth)
  byte crankRPM;      ///< RPM below which the engine is considered to be cranking
  byte floodClear;    ///< TPS (raw adc count? % ?) value that triggers flood clear mode (No fuel whilst cranking, See @ref correctionFloodClear())
  byte SoftRevLim;    ///< Soft rev limit (RPM/100)
  byte SoftLimRetard; ///< Amount soft limit (ignition) retard (degrees)
  byte SoftLimMax;    ///< Time the soft limit can run (units 0.1S)
  byte HardRevLim;    ///< Hard rev limit (RPM/100)
  byte taeBins[4];    ///< TPS based acceleration enrichment bins (Unit: %/s)
  byte taeValues[4];  ///< TPS based acceleration enrichment rates (Unit: % to add), values matched to thresholds of taeBins
  byte wueBins[10];   ///< Warmup Enrichment bins (Values are in @ref configPage2.wueValues OLD:configTable1)
  byte dwellLimit;
  byte dwellCorrectionValues[6]; ///< Correction table for dwell vs battery voltage
  byte iatRetBins[6]; ///< Inlet Air Temp timing retard curve bins (Unit: ...)
  byte iatRetValues[6]; ///< Inlet Air Temp timing retard curve values (Unit: ...)
  byte dfcoRPM;       ///< RPM at which DFCO turns off/on at
  byte dfcoHyster;    //Hysteris RPM for DFCO
  byte dfcoTPSThresh; //TPS must be below this figure for DFCO to engage (Unit: ...)

  byte ignBypassEnabled : 1; //Whether or not the ignition bypass is enabled
  byte ignBypassPin : 6; //Pin the ignition bypass is activated on
  byte ignBypassHiLo : 1; //Whether this should be active high or low.

  uint8_t ADCFILTER_TPS;
  uint8_t ADCFILTER_CLT;
  uint8_t ADCFILTER_IAT;
  uint8_t ADCFILTER_O2;
  uint8_t ADCFILTER_BAT;
  uint8_t ADCFILTER_MAP; //This is only used on Instantaneous MAP readings and is intentionally very weak to allow for faster response
  uint8_t ADCFILTER_BARO;
  
  byte cltAdvBins[6];   /**< Coolant Temp timing advance curve bins */
  byte cltAdvValues[6]; /**< Coolant timing advance curve values. These are translated by 15 to allow for negative values */

  byte maeBins[4];      /**< MAP based AE MAPdot bins */
  byte maeRates[4];     /**< MAP based AE values */

  int8_t batVoltCorrect; /**< Battery voltage calibration offset (Given as 10x value, e.g. 2v => 20) */

  byte baroFuelBins[8];
  byte baroFuelValues[8];

  byte idleAdvBins[6];
  byte idleAdvValues[6];

  byte engineProtectMaxRPM;

  int16_t vvt2CL0DutyAng;
  byte vvt2PWMdir : 1;
  byte inj4cylPairing : 2;
  byte dwellErrCorrect : 1;
  byte unusedBits4 : 4;
  byte ANGLEFILTER_VVT;
  byte FILTER_FLEX;
  byte vvtMinClt;
  byte vvtDelay;

#if defined(CORE_AVR)
  };
#else
  } __attribute__((__packed__)); //The 32 bi systems require all structs to be fully packed
#endif

/** Page 6 of the config - mostly variables that are required for AFR targets and closed loop.
See the ini file for further reference.
*/
struct config6 {

  byte egoAlgorithm : 2; ///< EGO Algorithm - Simple, PID, No correction
  byte egoType : 2;      ///< EGO Sensor Type 0=Disabled/None, 1=Narrowband, 2=Wideband
  byte boostEnabled : 1; ///< Boost control enabled 0 =off, 1 = on
  byte vvtEnabled : 1;   ///< 
  byte engineProtectType : 2;

  byte egoKP;
  byte egoKI;
  byte egoKD;
  byte egoTemp;     ///< The temperature above which closed loop is enabled
  byte egoCount;    ///< The number of ignition cycles per (ego AFR ?) step
  byte vvtMode : 2; ///< Valid VVT modes are 'on/off', 'open loop' and 'closed loop'
  byte vvtLoadSource : 2; ///< Load source for VVT (TPS or MAP)
  byte vvtPWMdir : 1; ///< VVT direction (normal or reverse)
  byte vvtCLUseHold : 1; //Whether or not to use a hold duty cycle (Most cases are Yes)
  byte vvtCLAlterFuelTiming : 1;
  byte boostCutEnabled : 1;
  byte egoLimit;    /// Maximum amount the closed loop EGO control will vary the fuelling
  byte ego_min;     /// AFR must be above this for closed loop to function
  byte ego_max;     /// AFR must be below this for closed loop to function
  byte ego_sdelay;  /// Time in seconds after engine starts that closed loop becomes available
  byte egoRPM;      /// RPM must be above this for closed loop to function
  byte egoTPSMax;   /// TPS must be below this for closed loop to function
  byte vvt1Pin : 6;
  byte useExtBaro : 1;
  byte boostMode : 1; /// Boost control mode: 0=Simple (BOOST_MODE_SIMPLE) or 1=full (BOOST_MODE_FULL)
  byte boostPin : 6;
  byte tachoMode : 1; /// Whether to use fixed tacho pulse duration or match to dwell duration
  byte useEMAP : 1;    ///< Enable EMAP
  byte voltageCorrectionBins[6]; //X axis bins for voltage correction tables
  byte injVoltageCorrectionValues[6]; //Correction table for injector PW vs battery voltage
  byte airDenBins[9];
  byte airDenRates[9];
  byte boostFreq;   /// Frequency of the boost PWM valve
  byte vvtFreq;     /// Frequency of the vvt PWM valve
  byte idleFreq;
  // Launch stuff, see beginning of speeduino.ino main loop
  byte launchPin : 6; ///< Launch (control ?) pin
  byte launchEnabled : 1; ///< Launch ...???... (control?) enabled
  byte launchHiLo : 1;  // 

  byte lnchSoftLim;
  int8_t lnchRetard; //Allow for negative advance value (ATDC)
  byte lnchHardLim;
  byte lnchFuelAdd;

  //PID values for idle needed to go here as out of room in the idle page
  byte idleKP;
  byte idleKI;
  byte idleKD;

  byte boostLimit; ///< Boost limit (Kpa). Stored value is actual (kPa) value divided by 2, allowing kPa values up to 511
  byte boostKP;
  byte boostKI;
  byte boostKD;

  byte lnchPullRes : 1;
  byte iacPWMrun : 1; ///< Run the PWM idle valve before engine is cranked over (0 = off, 1 = on)
  byte fuelTrimEnabled : 1;
  byte flatSEnable : 1; ///< Flat shift enable
  byte baroPin : 4;
  byte flatSSoftWin;
  int8_t flatSRetard;
  byte flatSArm;

  byte iacCLValues[10]; //Closed loop target RPM value
  byte iacOLStepVal[10]; //Open loop step values for stepper motors
  byte iacOLPWMVal[10]; //Open loop duty values for PMWM valves
  byte iacBins[10]; //Temperature Bins for the above 3 curves
  byte iacCrankSteps[4]; //Steps to use when cranking (Stepper motor)
  byte iacCrankDuty[4]; //Duty cycle to use on PWM valves when cranking
  byte iacCrankBins[4]; //Temperature Bins for the above 2 curves

  byte iacAlgorithm : 3; //Valid values are: "None", "On/Off", "PWM", "PWM Closed Loop", "Stepper", "Stepper Closed Loop"
  byte iacStepTime : 3; //How long to pulse the stepper for to ensure the step completes (ms)
  byte iacChannels : 1; //How many outputs to use in PWM mode (0 = 1 channel, 1 = 2 channels)
  byte iacPWMdir : 1; //Direction of the PWM valve. 0 = Normal = Higher RPM with more duty. 1 = Reverse = Lower RPM with more duty

  byte iacFastTemp; //Fast idle temp when using a simple on/off valve

  byte iacStepHome; //When using a stepper motor, the number of steps to be taken on startup to home the motor
  byte iacStepHyster; //Hysteresis temperature (*10). Eg 2.2C = 22

  byte fanInv : 1;        // Fan output inversion bit
  byte fanUnused : 1;
  byte fanPin : 6;
  byte fanSP;             // Cooling fan start temperature
  byte fanHyster;         // Fan hysteresis
  byte fanFreq;           // Fan PWM frequency
  byte fanPWMBins[4];     //Temperature Bins for the PWM fan control

#if defined(CORE_AVR)
  };
#else
  } __attribute__((__packed__)); //The 32 bit systems require all structs to be fully packed
#endif

/** Page 9 of the config - mostly deals with CANBUS control.
See ini file for further info (Config Page 10 in the ini).
*/
struct config9 {
  byte enable_secondarySerial:1;            //enable secondary serial
  byte intcan_available:1;                     //enable internal can module
  byte enable_intcan:1;
  byte secondarySerialProtocol:4;            //protocol for secondary serial. 0=Generic (Fixed list), 1=Generic (ini based list), 2=CAN, 3=msDroid, 4=Real Dash
  byte unused9_0:1;

  byte caninput_sel[16];                    //bit status on/Can/analog_local/digtal_local if input is enabled
  uint16_t caninput_source_can_address[16];        //u16 [15] array holding can address of input
  uint8_t caninput_source_start_byte[16];     //u08 [15] array holds the start byte number(value of 0-7)
  uint16_t caninput_source_num_bytes;     //u16 bit status of the number of bytes length 1 or 2
  
  byte caninputEndianess:1;
  //byte unused:2
  //...
  byte unused10_68;
  byte enable_candata_out : 1;
  byte canoutput_sel[8];
  uint16_t canoutput_param_group[8];
  uint8_t canoutput_param_start_byte[8];
  byte canoutput_param_num_bytes[8];

  byte unused10_110;
  byte unused10_111;
  byte egoMAPMax; //needs to be multiplied by 2 to get the proper value
  byte egoMAPMin; //needs to be multiplied by 2 to get the proper value
  byte speeduino_tsCanId:4;         //speeduino TS canid (0-14)
  uint16_t true_address;            //speeduino 11bit can address
  uint16_t realtime_base_address;   //speeduino 11 bit realtime base address
  uint16_t obd_address;             //speeduino OBD diagnostic address
  uint8_t Auxinpina[16];            //analog  pin number when internal aux in use
  uint8_t Auxinpinb[16];            // digital pin number when internal aux in use

  byte iacStepperInv : 1;  //stepper direction of travel to allow reversing. 0=normal, 1=inverted.
  byte iacCoolTime : 3; // how long to wait for the stepper to cool between steps

  byte boostByGearEnabled : 2;
  byte blankField : 1;
  byte iacStepperPower : 1; //Whether or not to power the stepper motor when not in use

  byte iacMaxSteps; // Step limit beyond which the stepper won't be driven. Should always be less than homing steps. Stored div 3 as per home steps.
  byte idleAdvStartDelay;     //delay for idle advance engage
  
  byte boostByGear1;
  byte boostByGear2;
  byte boostByGear3;
  byte boostByGear4;
  byte boostByGear5;
  byte boostByGear6;

  byte PWMFanDuty[4];
  byte hardRevMode : 2;
  byte coolantProtRPM[6];
  byte coolantProtTemp[6];

  byte unused10_179;
  byte dfcoTaperTime;
  byte dfcoTaperFuel;
  byte dfcoTaperAdvance;
  byte dfcoTaperEnable : 1;
  byte unused10_183 : 6;

  byte unused10_184;

  byte afrProtectEnabled : 2; /* < AFR protection enabled status. 0 = disabled, 1 = fixed mode, 2 = table mode */
  byte afrProtectMinMAP; /* < Minimum MAP. Stored value is divided by 2. Increments of 2 kPa, maximum 511 (?) kPa */
  byte afrProtectMinRPM; /* < Minimum RPM. Stored value is divded by 100. Increments of 100 RPM, maximum 25500 RPM */
  byte afrProtectMinTPS; /* < Minimum TPS. */
  byte afrProtectDeviation; /* < Maximum deviation from AFR target table. Stored value is multiplied by 10 */
  byte afrProtectCutTime; /* < Time in ms before cut. Stored value is divided by 100. Maximum of 2550 ms */
  byte afrProtectReactivationTPS; /* Disable engine protection cut once below this TPS percentage */
  
#if defined(CORE_AVR)
  };
#else
  } __attribute__((__packed__)); //The 32 bit systems require all structs to be fully packed
#endif

/** Page 10 - No specific purpose. Created initially for the cranking enrich curve.
192 bytes long.
See ini file for further info (Config Page 11 in the ini).
*/
struct config10 {
  byte crankingEnrichBins[4]; //Bytes 0-4
  byte crankingEnrichValues[4]; //Bytes 4-7

  //Byte 8
  byte rotaryType : 2;
  byte stagingEnabled : 1;
  byte stagingMode : 1;
  byte EMAPPin : 4;

  byte rotarySplitValues[8]; //Bytes 9-16
  byte rotarySplitBins[8]; //Bytes 17-24

  uint16_t boostSens; //Bytes 25-26
  byte boostIntv; //Byte 27
  uint16_t stagedInjSizePri; //Bytes 28-29
  uint16_t stagedInjSizeSec; //Bytes 30-31
  byte lnchCtrlTPS; //Byte 32

  uint8_t flexBoostBins[6]; //Bytes 33-38
  int16_t flexBoostAdj[6];  //kPa to be added to the boost target @ current ethanol (negative values allowed). Bytes 39-50
  uint8_t flexFuelBins[6]; //Bytes 51-56
  uint8_t flexFuelAdj[6];   //Fuel % @ current ethanol (typically 100% @ 0%, 163% @ 100%). Bytes 57-62
  uint8_t flexAdvBins[6]; //Bytes 63-68
  uint8_t flexAdvAdj[6];    //Additional advance (in degrees) @ current ethanol (typically 0 @ 0%, 10-20 @ 100%). NOTE: THIS SHOULD BE A SIGNED VALUE BUT 2d TABLE LOOKUP NOT WORKING WITH IT CURRENTLY!
                            //And another three corn rows die.
                            //Bytes 69-74

  //Byte 75
  byte n2o_enable : 2;
  byte n2o_arming_pin : 6;
  byte n2o_minCLT; //Byte 76
  byte n2o_maxMAP; //Byte 77
  byte n2o_minTPS; //Byte 78
  byte n2o_maxAFR; //Byte 79

  //Byte 80
  byte n2o_stage1_pin : 6;
  byte n2o_pin_polarity : 1;
  byte n2o_stage1_unused : 1;
  byte n2o_stage1_minRPM; //Byte 81
  byte n2o_stage1_maxRPM; //Byte 82
  byte n2o_stage1_adderMin; //Byte 83
  byte n2o_stage1_adderMax; //Byte 84
  byte n2o_stage1_retard; //Byte 85

  //Byte 86
  byte n2o_stage2_pin : 6;
  byte n2o_stage2_unused : 2;
  byte n2o_stage2_minRPM; //Byte 87
  byte n2o_stage2_maxRPM; //Byte 88
  byte n2o_stage2_adderMin; //Byte 89
  byte n2o_stage2_adderMax; //Byte 90
  byte n2o_stage2_retard; //Byte 91

  //Byte 92
  byte knock_mode : 2;
  byte knock_pin : 6;

  //Byte 93
  byte knock_trigger : 1;
  byte knock_pullup : 1;
  byte knock_limiterDisable : 1;
  byte knock_unused : 2;
  byte knock_count : 3;

  byte knock_threshold; //Byte 94
  byte knock_maxMAP; //Byte 95
  byte knock_maxRPM; //Byte 96
  byte knock_window_rpms[6]; //Bytes 97-102
  byte knock_window_angle[6]; //Bytes 103-108
  byte knock_window_dur[6]; //Bytes 109-114

  byte knock_maxRetard; //Byte 115
  byte knock_firstStep; //Byte 116
  byte knock_stepSize; //Byte 117
  byte knock_stepTime; //Byte 118
        
  byte knock_duration; //Time after knock retard starts that it should start recovering. Byte 119
  byte knock_recoveryStepTime; //Byte 120
  byte knock_recoveryStep; //Byte 121

  //Byte 122
  byte fuel2Algorithm : 3;
  byte fuel2Mode : 3;
  byte fuel2SwitchVariable : 2;

  //Bytes 123-124
  uint16_t fuel2SwitchValue;

  //Byte 125
  byte fuel2InputPin : 6;
  byte fuel2InputPolarity : 1;
  byte fuel2InputPullup : 1;

  byte vvtCLholdDuty; //Byte 126
  byte vvtCLKP; //Byte 127
  byte vvtCLKI; //Byte 128
  byte vvtCLKD; //Byte 129
  int16_t vvtCL0DutyAng; //Bytes 130-131
  uint8_t vvtCLMinAng; //Byte 132
  uint8_t vvtCLMaxAng; //Byte 133

  byte crankingEnrichTaper; //Byte 134

  byte fuelPressureEnable : 1; ///< Enable fuel pressure sensing from an analog pin (@ref pinFuelPressure)
  byte oilPressureEnable : 1;  ///< Enable oil pressure sensing from an analog pin (@ref pinOilPressure)
  byte oilPressureProtEnbl : 1;
  byte oilPressurePin : 5;

  byte fuelPressurePin : 5;
  byte unused11_165 : 3;
  
  int8_t fuelPressureMin;
  byte fuelPressureMax;
  int8_t oilPressureMin;
  byte oilPressureMax;

  byte oilPressureProtRPM[4];
  byte oilPressureProtMins[4];

  byte wmiEnabled : 1; // Byte 149
  byte wmiMode : 6;
  
  byte wmiAdvEnabled : 1;

  byte wmiTPS; // Byte 150
  byte wmiRPM; // Byte 151
  byte wmiMAP; // Byte 152
  byte wmiMAP2; // Byte 153
  byte wmiIAT; // Byte 154
  int8_t wmiOffset; // Byte 155

  byte wmiIndicatorEnabled : 1; // 156
  byte wmiIndicatorPin : 6;
  byte wmiIndicatorPolarity : 1;

  byte wmiEmptyEnabled : 1; // 157
  byte wmiEmptyPin : 6;
  byte wmiEmptyPolarity : 1;

  byte wmiEnabledPin; // 158

  byte wmiAdvBins[6]; //Bytes 159-164
  byte wmiAdvAdj[6];  //Additional advance (in degrees)
                      //Bytes 165-170
  byte vvtCLminDuty;
  byte vvtCLmaxDuty;
  byte vvt2Pin : 6;
  byte vvt2Enabled : 1;
  byte TrigEdgeThrd : 1;

  byte fuelTempBins[6];
  byte fuelTempValues[6]; //180

  //Byte 186
  byte spark2Algorithm : 3;
  byte spark2Mode : 3;
  byte spark2SwitchVariable : 2;

  //Bytes 187-188
  uint16_t spark2SwitchValue;

  //Byte 189
  byte spark2InputPin : 6;
  byte spark2InputPolarity : 1;
  byte spark2InputPullup : 1;

  //Byte 190
  byte oilPressureProtTime;

  //Byte 191
  byte lnchCtrlVss;

#if defined(CORE_AVR)
  };
#else
  } __attribute__((__packed__)); //The 32 bit systems require all structs to be fully packed
#endif
/** Config for programmable I/O comparison operation (between 2 vars).
 * Operations are implemented in utilities.ino (@ref checkProgrammableIO()).
 */
struct cmpOperation{
  uint8_t firstCompType : 3;  ///< First cmp. op (COMPARATOR_* ops, see below)
  uint8_t secondCompType : 3; ///< Second cmp. op (0=COMPARATOR_EQUAL, 1=COMPARATOR_NOT_EQUAL,2=COMPARATOR_GREATER,3=COMPARATOR_GREATER_EQUAL,4=COMPARATOR_LESS,5=COMPARATOR_LESS_EQUAL,6=COMPARATOR_CHANGE)
  uint8_t bitwise : 2; ///< BITWISE_AND, BITWISE_OR, BITWISE_XOR
};

/**
Page 13 - Programmable outputs logic rules.
128 bytes long. Rules implemented in utilities.ino @ref checkProgrammableIO().
*/
struct config13 {
  uint8_t outputInverted; ///< Invert (on/off) value before writing to output pin (for all programmable I/O:s).
  uint8_t kindOfLimiting; ///< Select which kind of output limiting are active (0 - minimum | 1 - maximum)
  uint8_t outputPin[8];   ///< Disable(0) or enable (set to valid pin number) Programmable Pin (output/target pin to set)
  uint8_t outputDelay[8]; ///< Output write delay for each programmable I/O (Unit: 0.1S)
  uint8_t firstDataIn[8]; ///< Set of first I/O vars to compare
  uint8_t secondDataIn[8];///< Set of second I/O vars to compare
  uint8_t outputTimeLimit[8]; ///< Output delay for each programmable I/O, kindOfLimiting bit dependent(Unit: 0.1S)
  uint8_t unused_13[8]; // Unused
  int16_t firstTarget[8]; ///< first  target value to compare with numeric comp
  int16_t secondTarget[8];///< second target value to compare with bitwise op
  //89bytes
  struct cmpOperation operation[8]; ///< I/O variable comparison operations (See @ref cmpOperation)

  uint16_t candID[8]; ///< Actual CAN ID need 16bits, this is a placeholder

  byte unused12_106_116[10];

  byte onboard_log_csv_separator :2;  //";", ",", "tab", "space"  
  byte onboard_log_file_style    :2;  // "Disabled", "CSV", "Binary", "INVALID" 
  byte onboard_log_file_rate     :2;  // "1Hz", "4Hz", "10Hz", "30Hz" 
  byte onboard_log_filenaming    :2;  // "Overwrite", "Date-time", "Sequential", "INVALID" 
  byte onboard_log_storage       :2;  // "sd-card", "INVALID", "INVALID", "INVALID" ;In the future maybe an onboard spi flash can be used, or switch between SDIO vs SPI sd card interfaces.
  byte onboard_log_trigger_boot  :1;  // "Disabled", "On boot"
  byte onboard_log_trigger_RPM   :1;  // "Disabled", "Enabled"
  byte onboard_log_trigger_prot  :1;  // "Disabled", "Enabled"
  byte onboard_log_trigger_Vbat  :1;  // "Disabled", "Enabled"
  byte onboard_log_trigger_Epin  :2;  // "Disabled", "polling", "toggle" , "INVALID" 
  uint16_t onboard_log_tr1_duration;  // Duration of logging that starts on boot
  byte onboard_log_tr2_thr_on;        //  "RPM",      100.0,  0.0,    0,     10000,  0
  byte onboard_log_tr2_thr_off;       //  "RPM",      100.0,  0.0,    0,     10000,  0
  byte onboard_log_tr3_thr_RPM   :1;  // "Disabled", "Enabled"
  byte onboard_log_tr3_thr_MAP   :1;  // "Disabled", "Enabled"
  byte onboard_log_tr3_thr_Oil   :1;  // "Disabled", "Enabled"
  byte onboard_log_tr3_thr_AFR   :1;  // "Disabled", "Enabled"     
  byte onboard_log_tr4_thr_on;        // "V",        0.1,   0.0,  0.0,  15.90,      2 ; * (  1 byte)    
  byte onboard_log_tr4_thr_off;       // "V",        0.1,   0.0,  0.0,  15.90,      2 ; * (  1 byte)   
  byte onboard_log_tr5_Epin_pin  :6;        // "pin",      0,    0, 0,  1,    255,        0 ;  
  byte unused13_125_2            :2;

  byte hwTestIgnDuration;
  byte hwTestInjDuration;

#if defined(CORE_AVR)
  };
#else
  } __attribute__((__packed__)); //The 32 bit systems require all structs to be fully packed
#endif

/**
Page 15 - second page for VVT and boost control.
256 bytes long. 
*/
struct config15 {
  byte boostControlEnable : 1; 
  byte unused15_1 : 7; //7bits unused
  byte boostDCWhenDisabled;
  byte boostControlEnableThreshold; //if fixed value enable set threshold here.
  
  //Byte 83 - Air conditioning binary points
  byte airConEnable : 1;
  byte airConCompPol : 1;
  byte airConReqPol : 1;
  byte airConTurnsFanOn : 1;
  byte airConFanEnabled : 1;
  byte airConFanPol : 1;
  byte airConUnused1 : 2;

  //Bytes 84-97 - Air conditioning analog points
  byte airConCompPin : 6;
  byte airConUnused2 : 2;
  byte airConReqPin : 6;
  byte airConUnused3 : 2;
  byte airConTPSCut;
  byte airConMinRPMdiv10;
  byte airConMaxRPMdiv100;
  byte airConClTempCut;
  byte airConIdleSteps;
  byte airConTPSCutTime;
  byte airConCompOnDelay;
  byte airConAfterStartDelay;
  byte airConRPMCutTime;
  byte airConFanPin : 6;
  byte airConUnused4 : 2;
  byte airConIdleUpRPMAdder;
  byte airConPwmFanMinDuty;

  int8_t rollingProtRPMDelta[4]; // Signed RPM value representing how much below the RPM limit. Divided by 10
  byte rollingProtCutPercent[4];
  
  //Bytes 106-255
  byte Unused15_106_255[150];

#if defined(CORE_AVR)
  };
#else
  } __attribute__((__packed__)); //The 32 bit systems require all structs to be fully packed
#endif

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
