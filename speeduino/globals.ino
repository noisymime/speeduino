/** @file
 * Instantiation of various (table2D, table3D) tables, volatile (interrupt modified) variables, Injector (1...8) enablement flags, etc.
 */
#include "globals.h"

const char TSfirmwareVersion[] PROGMEM = "Speeduino";

const byte data_structure_version = 2; //This identifies the data structure when reading / writing. (outdated ?)
FastCRC32 CRC32;

struct table3d16RpmLoad fuelTable; ///< 16x16 fuel map
struct table3d16RpmLoad fuelTable2; ///< 16x16 fuel map
struct table3d16RpmLoad ignitionTable; ///< 16x16 ignition map
struct table3d16RpmLoad ignitionTable2; ///< 16x16 ignition map
struct table3d16RpmLoad afrTable; ///< 16x16 afr target map
struct table3d8RpmLoad stagingTable; ///< 8x8 fuel staging table
struct table3d8RpmLoad boostTable; ///< 8x8 boost map
struct table3d8RpmLoad vvtTable; ///< 8x8 vvt map
struct table3d8RpmLoad vvt2Table; ///< 8x8 vvt2 map
struct table3d8RpmLoad wmiTable; ///< 8x8 wmi map
struct table3d6RpmLoad trim1Table; ///< 6x6 Fuel trim 1 map
struct table3d6RpmLoad trim2Table; ///< 6x6 Fuel trim 2 map
struct table3d6RpmLoad trim3Table; ///< 6x6 Fuel trim 3 map
struct table3d6RpmLoad trim4Table; ///< 6x6 Fuel trim 4 map
struct table3d6RpmLoad trim5Table; ///< 6x6 Fuel trim 5 map
struct table3d6RpmLoad trim6Table; ///< 6x6 Fuel trim 6 map
struct table3d6RpmLoad trim7Table; ///< 6x6 Fuel trim 7 map
struct table3d6RpmLoad trim8Table; ///< 6x6 Fuel trim 8 map
struct table3d4RpmLoad dwellTable; ///< 4x4 Dwell map
struct table2D taeTable; ///< 4 bin TPS Acceleration Enrichment map (2D)
struct table2D maeTable;
struct table2D WUETable; ///< 10 bin Warm Up Enrichment map (2D)
struct table2D ASETable; ///< 4 bin After Start Enrichment map (2D)
struct table2D ASECountTable; ///< 4 bin After Start duration map (2D)
struct table2D PrimingPulseTable; ///< 4 bin Priming pulsewidth map (2D)
struct table2D crankingEnrichTable; ///< 4 bin cranking Enrichment map (2D)
struct table2D dwellVCorrectionTable; ///< 6 bin dwell voltage correction (2D)
struct table2D injectorVCorrectionTable; ///< 6 bin injector voltage correction (2D)
struct table2D injectorAngleTable; ///< 4 bin injector angle curve (2D)
struct table2D IATDensityCorrectionTable; ///< 9 bin inlet air temperature density correction (2D)
struct table2D baroFuelTable; ///< 8 bin baro correction curve (2D)
struct table2D IATRetardTable; ///< 6 bin ignition adjustment based on inlet air temperature  (2D)
struct table2D idleTargetTable; ///< 10 bin idle target table for idle timing (2D)
struct table2D idleAdvanceTable; ///< 6 bin idle advance adjustment table based on RPM difference  (2D)
struct table2D CLTAdvanceTable; ///< 6 bin ignition adjustment based on coolant temperature  (2D)
struct table2D rotarySplitTable; ///< 8 bin ignition split curve for rotary leading/trailing  (2D)
struct table2D flexFuelTable;  ///< 6 bin flex fuel correction table for fuel adjustments (2D)
struct table2D flexAdvTable;   ///< 6 bin flex fuel correction table for timing advance (2D)
struct table2D flexBoostTable; ///< 6 bin flex fuel correction table for boost adjustments (2D)
struct table2D fuelTempTable;  ///< 6 bin flex fuel correction table for fuel adjustments (2D)
struct table2D knockWindowStartTable;
struct table2D knockWindowDurationTable;
struct table2D oilPressureProtectTable;
struct table2D wmiAdvTable; //6 bin wmi correction table for timing advance (2D)
struct table2D fanPWMTable;

/// volatile inj*_pin_port and  inj*_pin_mask vars are for the direct port manipulation of the injectors, coils and aux outputs.
volatile PORT_TYPE *inj1_pin_port;
volatile PINMASK_TYPE inj1_pin_mask;
volatile PORT_TYPE *inj2_pin_port;
volatile PINMASK_TYPE inj2_pin_mask;
volatile PORT_TYPE *inj3_pin_port;
volatile PINMASK_TYPE inj3_pin_mask;
volatile PORT_TYPE *inj4_pin_port;
volatile PINMASK_TYPE inj4_pin_mask;
volatile PORT_TYPE *inj5_pin_port;
volatile PINMASK_TYPE inj5_pin_mask;
volatile PORT_TYPE *inj6_pin_port;
volatile PINMASK_TYPE inj6_pin_mask;
volatile PORT_TYPE *inj7_pin_port;
volatile PINMASK_TYPE inj7_pin_mask;
volatile PORT_TYPE *inj8_pin_port;
volatile PINMASK_TYPE inj8_pin_mask;

volatile PORT_TYPE *ign1_pin_port;
volatile PINMASK_TYPE ign1_pin_mask;
volatile PORT_TYPE *ign2_pin_port;
volatile PINMASK_TYPE ign2_pin_mask;
volatile PORT_TYPE *ign3_pin_port;
volatile PINMASK_TYPE ign3_pin_mask;
volatile PORT_TYPE *ign4_pin_port;
volatile PINMASK_TYPE ign4_pin_mask;
volatile PORT_TYPE *ign5_pin_port;
volatile PINMASK_TYPE ign5_pin_mask;
volatile PORT_TYPE *ign6_pin_port;
volatile PINMASK_TYPE ign6_pin_mask;
volatile PORT_TYPE *ign7_pin_port;
volatile PINMASK_TYPE ign7_pin_mask;
volatile PORT_TYPE *ign8_pin_port;
volatile PINMASK_TYPE ign8_pin_mask;

volatile PORT_TYPE *tach_pin_port;
volatile PINMASK_TYPE tach_pin_mask;
volatile PORT_TYPE *pump_pin_port;
volatile PINMASK_TYPE pump_pin_mask;

volatile PORT_TYPE *flex_pin_port;
volatile PINMASK_TYPE flex_pin_mask;

volatile PORT_TYPE *triggerPri_pin_port;
volatile PINMASK_TYPE triggerPri_pin_mask;
volatile PORT_TYPE *triggerSec_pin_port;
volatile PINMASK_TYPE triggerSec_pin_mask;

/// These need to be here as they are used in both speeduino.ino and scheduler.ino
bool channel1InjEnabled = true;
bool channel2InjEnabled = false;
bool channel3InjEnabled = false;
bool channel4InjEnabled = false;
bool channel5InjEnabled = false;
bool channel6InjEnabled = false;
bool channel7InjEnabled = false;
bool channel8InjEnabled = false;

int ignition1EndAngle = 0;
int ignition2EndAngle = 0;
int ignition3EndAngle = 0;
int ignition4EndAngle = 0;
int ignition5EndAngle = 0;
int ignition6EndAngle = 0;
int ignition7EndAngle = 0;
int ignition8EndAngle = 0;

//These are variables used across multiple files
bool initialisationComplete = false; ///< Tracks whether the setup() function has run completely (true = has run)
byte fpPrimeTime = 0; ///< The time (in seconds, based on @ref statuses.secl) that the fuel pump started priming
uint16_t softStartTime = 0; //The time (in 0.1 seconds, based on seclx10) that the soft limiter started
volatile uint16_t mainLoopCount; //Main loop counter (incremented at each main loop rev., used for maintaining currentStatus.loopsPerSecond)
unsigned long revolutionTime; //The time in uS that one revolution would take at current speed (The time tooth 1 was last seen, minus the time it was seen prior to that)
volatile unsigned long timer5_overflow_count = 0; //Increments every time counter 5 overflows. Used for the fast version of micros()
volatile unsigned long ms_counter = 0; //A counter that increments once per ms
uint16_t fixedCrankingOverride = 0;
bool clutchTrigger;
bool previousClutchTrigger;
volatile uint32_t toothHistory[TOOTH_LOG_SIZE]; ///< Tooth trigger history - delta time (in uS) from last tooth (Indexed by @ref toothHistoryIndex)
volatile uint8_t compositeLogHistory[TOOTH_LOG_SIZE]; 
volatile bool fpPrimed = false; ///< Tracks whether or not the fuel pump priming has been completed yet
volatile bool injPrimed = false; ///< Tracks whether or not the injectors priming has been completed yet
volatile unsigned int toothHistoryIndex = 0; ///< Current index to @ref toothHistory array
unsigned long currentLoopTime; /**< The time (in uS) that the current mainloop started */
unsigned long previousLoopTime; /**< The time (in uS) that the previous mainloop started */
volatile uint16_t ignitionCount; /**< The count of ignition events that have taken place since the engine started */
#if defined(CORE_SAMD21)
  PinStatus primaryTriggerEdge;
  PinStatus secondaryTriggerEdge;
  PinStatus tertiaryTriggerEdge;
#else
  byte primaryTriggerEdge;
  byte secondaryTriggerEdge;
  byte tertiaryTriggerEdge;
#endif
int CRANK_ANGLE_MAX = 720;
int CRANK_ANGLE_MAX_IGN = 360;
int CRANK_ANGLE_MAX_INJ = 360; ///< The number of crank degrees that the system track over. 360 for wasted / timed batch and 720 for sequential
volatile uint32_t runSecsX10;
volatile uint32_t seclx10;
volatile byte HWTest_INJ = 0; /**< Each bit in this variable represents one of the injector channels and it's HW test status */
volatile byte HWTest_INJ_50pc = 0; /**< Each bit in this variable represents one of the injector channels and it's 50% HW test status */
volatile byte HWTest_IGN = 0; /**< Each bit in this variable represents one of the ignition channels and it's HW test status */
volatile byte HWTest_IGN_50pc = 0; 


//This needs to be here because using the config page directly can prevent burning the setting
byte resetControl = RESET_CONTROL_DISABLED;

volatile byte TIMER_mask;
volatile byte LOOP_TIMER;

/// Various pin numbering (Injectors, Ign outputs, CAS, Cam, Sensors. etc.) assignments
byte pinInjector1; ///< Output pin injector 1
byte pinInjector2; ///< Output pin injector 2
byte pinInjector3; ///< Output pin injector 3
byte pinInjector4; ///< Output pin injector 4
byte pinInjector5; ///< Output pin injector 5
byte pinInjector6; ///< Output pin injector 6
byte pinInjector7; ///< Output pin injector 7
byte pinInjector8; ///< Output pin injector 8
byte injectorOutputControl = OUTPUT_CONTROL_DIRECT; /**< Specifies whether the injectors are controlled directly (Via an IO pin)
    or using something like the MC33810. 0 = Direct (OUTPUT_CONTROL_DIRECT), 10 = MC33810 (OUTPUT_CONTROL_MC33810) */
byte pinCoil1; ///< Pin for coil 1
byte pinCoil2; ///< Pin for coil 2
byte pinCoil3; ///< Pin for coil 3
byte pinCoil4; ///< Pin for coil 4
byte pinCoil5; ///< Pin for coil 5
byte pinCoil6; ///< Pin for coil 6
byte pinCoil7; ///< Pin for coil 7
byte pinCoil8; ///< Pin for coil 8
byte ignitionOutputControl = OUTPUT_CONTROL_DIRECT; /**< Specifies whether the coils are controlled directly (Via an IO pin)
   or using something like the MC33810. 0 = Direct (OUTPUT_CONTROL_DIRECT), 10 = MC33810 (OUTPUT_CONTROL_MC33810) */
byte pinTrigger;  ///< RPM1 (Typically CAS=crankshaft angle sensor) pin
byte pinTrigger2; ///< RPM2 (Typically the Cam Sensor) pin
byte pinTrigger3;	///< the 2nd cam sensor pin
byte pinTPS;      //TPS input pin
byte pinMAP;      //MAP sensor pin
byte pinEMAP;     //EMAP sensor pin
byte pinMAP2;     //2nd MAP sensor (Currently unused)
byte pinIAT;      //IAT sensor pin
byte pinCLT;      //CLS sensor pin
byte pinO2;       //O2 Sensor pin
byte pinO2_2;     //second O2 pin
byte pinBat;      //Battery voltage pin
byte pinDisplayReset; // OLED reset pin
byte pinTachOut;  //Tacho output
byte pinFuelPump; //Fuel pump on/off
byte pinIdle1;    //Single wire idle control
byte pinIdle2;    //2 wire idle control (Not currently used)
byte pinIdleUp;   //Input for triggering Idle Up
byte pinIdleUpOutput; //Output that follows (normal or inverted) the idle up pin
byte pinCTPS;     //Input for triggering closed throttle state
byte pinFuel2Input;  //Input for switching to the 2nd fuel table
byte pinSpark2Input; //Input for switching to the 2nd ignition table
byte pinSpareTemp1;  // Future use only
byte pinSpareTemp2;  // Future use only
byte pinSpareOut1;  //Generic output
byte pinSpareOut2;  //Generic output
byte pinSpareOut3;  //Generic output
byte pinSpareOut4;  //Generic output
byte pinSpareOut5;  //Generic output
byte pinSpareOut6;  //Generic output
byte pinSpareHOut1; //spare high current output
byte pinSpareHOut2; // spare high current output
byte pinSpareLOut1; // spare low current output
byte pinSpareLOut2; // spare low current output
byte pinSpareLOut3;
byte pinSpareLOut4;
byte pinSpareLOut5;
byte pinBoost;
byte pinVVT_1;     ///< vvt (variable valve timing) output 1
byte pinVVT_2;     ///< vvt (variable valve timing) output 2
byte pinFan;       ///< Cooling fan output (on/off? See: auxiliaries.ino)
byte pinStepperDir; //Direction pin for the stepper motor driver
byte pinStepperStep; //Step pin for the stepper motor driver
byte pinStepperEnable; //Turning the DRV8825 driver on/off
byte pinLaunch;
byte pinIgnBypass; //The pin used for an ignition bypass (Optional)
byte pinFlex; //Pin with the flex sensor attached
byte pinVSS;  // VSS (Vehicle speed sensor) Pin
byte pinBaro; //Pin that an al barometric pressure sensor is attached to (If used)
byte pinResetControl; // Output pin used control resetting the Arduino
byte pinFuelPressure;
byte pinOilPressure;
byte pinWMIEmpty; // Water tank empty sensor
byte pinWMIIndicator; // No water indicator bulb
byte pinWMIEnabled; // ON-OFF output to relay/pump/solenoid 
byte pinMC33810_1_CS;
byte pinMC33810_2_CS;
#ifdef USE_SPI_EEPROM
  byte pinSPIFlash_CS;
#endif

struct statuses currentStatus; /**< The master global "live" status struct. Contains all values that are updated frequently and used across modules */
struct config2 configPage2;
struct config4 configPage4;
struct config6 configPage6;
struct config9 configPage9;
struct config10 configPage10;
struct config13 configPage13;

//byte cltCalibrationTable[CALIBRATION_TABLE_SIZE]; /**< An array containing the coolant sensor calibration values */
//byte iatCalibrationTable[CALIBRATION_TABLE_SIZE]; /**< An array containing the inlet air temperature sensor calibration values */
//byte o2CalibrationTable[CALIBRATION_TABLE_SIZE]; /**< An array containing the O2 sensor calibration values */

uint16_t cltCalibration_bins[32];
uint16_t cltCalibration_values[32];
struct table2D cltCalibrationTable;
uint16_t iatCalibration_bins[32];
uint16_t iatCalibration_values[32];
struct table2D iatCalibrationTable;
uint16_t o2Calibration_bins[32];
uint8_t o2Calibration_values[32];
struct table2D o2CalibrationTable; 
