#pragma once

#include <stdint.h>

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

#define IDLEADVANCE_MODE_OFF      0U
#define IDLEADVANCE_MODE_ADDED    1U
#define IDLEADVANCE_MODE_SWITCHED 2U

#define IDLEADVANCE_ALGO_TPS      0U
#define IDLEADVANCE_ALGO_CTPS     1U

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

  uint8_t dwellCrank;    ///< Dwell time whilst cranking
  uint8_t dwellRun;      ///< Dwell time whilst running
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
  byte afrProtectMinRPM; /* < Minimum RPM. Stored value is divided by 100. Increments of 100 RPM, maximum 25500 RPM */
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

  byte oilPressureProtTime;

  byte unused11_191_191;

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

//Define the load algorithm
#define LOAD_SOURCE_MAP         0
#define LOAD_SOURCE_TPS         1
#define LOAD_SOURCE_IMAPEMAP    2

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

#define EGO_ALGORITHM_SIMPLE   0U
#define EGO_ALGORITHM_INVALID1 1U
#define EGO_ALGORITHM_PID      2U
#define EGO_ALGORITHM_NONE     3U

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

#define KNOCK_MODE_OFF      0U
#define KNOCK_MODE_DIGITAL  1U
#define KNOCK_MODE_ANALOG   2U

#define KNOCK_TRIGGER_HIGH  0
#define KNOCK_TRIGGER_LOW   1

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
