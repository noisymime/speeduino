/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
#include "idle.h"
#include "maths.h"
#include "timers.h"
#include "src/PID_v1/PID_v1.h"

/*
These functions cover the PWM and stepper idle control
*/

/*
Idle Control
Currently limited to on/off control and open loop PWM and stepper drive
*/
integerPID idlePID(&currentStatus.longRPM, &idle_pid_target_value, &idle_cl_target_rpm, configPage6.idleKP, configPage6.idleKI, configPage6.idleKD, DIRECT); //This is the PID object if that algorithm is used. Needs to be global as it maintains state outside of each function call

//Any common functions associated with starting the Idle
//Typically this is enabling the PWM interrupt
static inline void enableIdle()
{
  if( (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_CL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_OL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_OLCL) )
  {
    IDLE_TIMER_ENABLE();
  }
  else if ( (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_CL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OLCL) )
  {

  }
}

void initialiseIdle()
{
  //By default, turn off the PWM interrupt (It gets turned on below if needed)
  IDLE_TIMER_DISABLE();

  //Pin masks must always be initialized, regardless of whether PWM idle is used. This is required for STM32 to prevent issues if the IRQ function fires on restat/overflow
  idle_pin_port = portOutputRegister(digitalPinToPort(pinIdle1));
  idle_pin_mask = digitalPinToBitMask(pinIdle1);
  idle2_pin_port = portOutputRegister(digitalPinToPort(pinIdle2));
  idle2_pin_mask = digitalPinToBitMask(pinIdle2);

  //Initialising comprises of setting the 2D tables with the relevant values from the config pages
  switch(configPage6.iacAlgorithm)
  {
    case IAC_ALGORITHM_NONE:       
      //Case 0 is no idle control ('None')
      break;

    case IAC_ALGORITHM_ONOFF:
      //Case 1 is on/off idle control
      if ((currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET) < configPage6.iacFastTemp)
      {
        digitalWrite(pinIdle1, HIGH);
        idleOn = true;
      }
      break;

    case IAC_ALGORITHM_PWM_OL:
      //Case 2 is PWM open loop
      iacPWMTable.xSize = 10;
      iacPWMTable.valueSize = SIZE_BYTE;
      iacPWMTable.axisSize = SIZE_BYTE;
      iacPWMTable.values = configPage6.iacOLPWMVal;
      iacPWMTable.axisX = configPage6.iacBins;


      iacCrankDutyTable.xSize = 4;
      iacCrankDutyTable.valueSize = SIZE_BYTE;
      iacCrankDutyTable.axisSize = SIZE_BYTE;
      iacCrankDutyTable.values = configPage6.iacCrankDuty;
      iacCrankDutyTable.axisX = configPage6.iacCrankBins;

      #if defined(CORE_AVR)
        idle_pwm_max_count = 1000000L / (16 * configPage6.idleFreq * 2); //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
      #elif defined(CORE_TEENSY35)
        idle_pwm_max_count = 1000000L / (32 * configPage6.idleFreq * 2); //Converts the frequency in Hz to the number of ticks (at 32uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
      #elif defined(CORE_TEENSY41)
        idle_pwm_max_count = 1000000L / (2 * configPage6.idleFreq * 2); //Converts the frequency in Hz to the number of ticks (at 2uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
      #endif
      enableIdle();
      break;

    case IAC_ALGORITHM_PWM_OLCL:
      //Case 6 is PWM closed loop with open loop table used as feed forward
      iacPWMTable.xSize = 10;
      iacPWMTable.valueSize = SIZE_BYTE;
      iacPWMTable.axisSize = SIZE_BYTE;
      iacPWMTable.values = configPage6.iacOLPWMVal;
      iacPWMTable.axisX = configPage6.iacBins;

      iacClosedLoopTable.xSize = 10;
      iacClosedLoopTable.valueSize = SIZE_BYTE;
      iacClosedLoopTable.axisSize = SIZE_BYTE;
      iacClosedLoopTable.values = configPage6.iacCLValues;
      iacClosedLoopTable.axisX = configPage6.iacBins;

      iacCrankDutyTable.xSize = 4;
      iacCrankDutyTable.valueSize = SIZE_BYTE;
      iacCrankDutyTable.axisSize = SIZE_BYTE;
      iacCrankDutyTable.values = configPage6.iacCrankDuty;
      iacCrankDutyTable.axisX = configPage6.iacCrankBins;

      #if defined(CORE_AVR)
        idle_pwm_max_count = 1000000L / (16 * configPage6.idleFreq * 2); //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
      #elif defined(CORE_TEENSY)
        idle_pwm_max_count = 1000000L / (32 * configPage6.idleFreq * 2); //Converts the frequency in Hz to the number of ticks (at 32uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
      #elif defined(CORE_TEENSY41)
        idle_pwm_max_count = 1000000L / (2 * configPage6.idleFreq * 2); //Converts the frequency in Hz to the number of ticks (at 2uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
      #endif
      idlePID.SetOutputLimits(percentage(configPage2.iacCLminDuty, idle_pwm_max_count<<2), percentage(configPage2.iacCLmaxDuty, idle_pwm_max_count<<2));
      idlePID.SetTunings(configPage6.idleKP, configPage6.idleKI, configPage6.idleKD);
      idlePID.SetMode(AUTOMATIC); //Turn PID on
      idle_pid_target_value = 0;
      idlePID.Initialize();
      idleCounter = 0;

      break;

    case IAC_ALGORITHM_PWM_CL:
      //Case 3 is PWM closed loop
      iacClosedLoopTable.xSize = 10;
      iacClosedLoopTable.valueSize = SIZE_BYTE;
      iacClosedLoopTable.axisSize = SIZE_BYTE;
      iacClosedLoopTable.values = configPage6.iacCLValues;
      iacClosedLoopTable.axisX = configPage6.iacBins;

      iacCrankDutyTable.xSize = 4;
      iacCrankDutyTable.valueSize = SIZE_BYTE;
      iacCrankDutyTable.axisSize = SIZE_BYTE;
      iacCrankDutyTable.values = configPage6.iacCrankDuty;
      iacCrankDutyTable.axisX = configPage6.iacCrankBins;

      #if defined(CORE_AVR)
        idle_pwm_max_count = 1000000L / (16 * configPage6.idleFreq * 2); //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
      #elif defined(CORE_TEENSY)
        idle_pwm_max_count = 1000000L / (32 * configPage6.idleFreq * 2); //Converts the frequency in Hz to the number of ticks (at 32uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
      #elif defined(CORE_TEENSY41)
        idle_pwm_max_count = 1000000L / (2 * configPage6.idleFreq * 2); //Converts the frequency in Hz to the number of ticks (at 2uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
      #endif
      idlePID.SetOutputLimits(percentage(configPage2.iacCLminDuty, idle_pwm_max_count<<2), percentage(configPage2.iacCLmaxDuty, idle_pwm_max_count<<2));
      idlePID.SetTunings(configPage6.idleKP, configPage6.idleKI, configPage6.idleKD);
      idlePID.SetMode(AUTOMATIC); //Turn PID on
      idle_pid_target_value = table2D_getValue(&iacCrankDutyTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
      idlePID.Initialize();
      idleCounter = 0;

      break;

    case IAC_ALGORITHM_STEP_OL:
      //Case 2 is Stepper open loop
      iacStepTable.xSize = 10;
      iacStepTable.valueSize = SIZE_BYTE;
      iacStepTable.axisSize = SIZE_BYTE;
      iacStepTable.values = configPage6.iacOLStepVal;
      iacStepTable.axisX = configPage6.iacBins;

      iacCrankStepsTable.xSize = 4;
      iacCrankStepsTable.valueSize = SIZE_BYTE;
      iacCrankStepsTable.axisSize = SIZE_BYTE;
      iacCrankStepsTable.values = configPage6.iacCrankSteps;
      iacCrankStepsTable.axisX = configPage6.iacCrankBins;
      iacStepTime_uS = configPage6.iacStepTime * 1000;
      iacCoolTime_uS = configPage9.iacCoolTime * 1000;

      if( completedHomeSteps < (configPage6.iacStepHome * 3) )
      {
        //Change between modes running make engine stall
        completedHomeSteps = 0;
        idleStepper.curIdleStep = 0;
        idleStepper.stepperStatus = SOFF;
      }

      if (! configPage9.iacStepperInv)
      {
        idleStepper.lessAirDirection = STEPPER_BACKWARD;
        idleStepper.moreAirDirection = STEPPER_FORWARD;
      }
      else
      {
        idleStepper.lessAirDirection = STEPPER_FORWARD;
        idleStepper.moreAirDirection = STEPPER_BACKWARD;
      }
      configPage6.iacPWMrun = false; // just in case. This needs to be false with stepper idle
      break;

    case IAC_ALGORITHM_STEP_CL:
      //Case 5 is Stepper closed loop
      iacClosedLoopTable.xSize = 10;
      iacClosedLoopTable.valueSize = SIZE_BYTE;
      iacClosedLoopTable.axisSize = SIZE_BYTE;
      iacClosedLoopTable.values = configPage6.iacCLValues;
      iacClosedLoopTable.axisX = configPage6.iacBins;

      iacCrankStepsTable.xSize = 4;
      iacCrankStepsTable.valueSize = SIZE_BYTE;
      iacCrankStepsTable.axisSize = SIZE_BYTE;
      iacCrankStepsTable.values = configPage6.iacCrankSteps;
      iacCrankStepsTable.axisX = configPage6.iacCrankBins;
      iacStepTime_uS = configPage6.iacStepTime * 1000;
      iacCoolTime_uS = configPage9.iacCoolTime * 1000;

      if( completedHomeSteps < (configPage6.iacStepHome * 3) )
      {
        //Change between modes running make engine stall
        completedHomeSteps = 0;
        idleStepper.curIdleStep = 0;
        idleStepper.stepperStatus = SOFF;
      }

      if (! configPage9.iacStepperInv)
      {
        idleStepper.lessAirDirection = STEPPER_BACKWARD;
        idleStepper.moreAirDirection = STEPPER_FORWARD;
      }
      else
      {
        idleStepper.lessAirDirection = STEPPER_FORWARD;
        idleStepper.moreAirDirection = STEPPER_BACKWARD;
      }

      idlePID.SetSampleTime(250); //4Hz means 250ms
      idlePID.SetOutputLimits((configPage6.iacOLStepVal[9]*3)<<1, (configPage9.iacMaxSteps * 3)<<2); //Maximum number of steps; always less than home steps count.
      idlePID.SetTunings(configPage6.idleKP, configPage6.idleKI, configPage6.idleKD);
      idlePID.SetMode(AUTOMATIC); //Turn PID on
      configPage6.iacPWMrun = false; // just in case. This needs to be false with stepper idle
      idle_pid_target_value = table2D_getValue(&iacClosedLoopTable, (currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET)) * 3;
      idlePID.Initialize();
      break;

    case IAC_ALGORITHM_STEP_OLCL:
      //Case 7 is Stepper closed loop with open loop table used as feed forward
      iacStepTable.xSize = 10;
      iacStepTable.valueSize = SIZE_BYTE;
      iacStepTable.axisSize = SIZE_BYTE;
      iacStepTable.values = configPage6.iacOLStepVal;
      iacStepTable.axisX = configPage6.iacBins;

      iacClosedLoopTable.xSize = 10;
      iacClosedLoopTable.valueSize = SIZE_BYTE;
      iacClosedLoopTable.axisSize = SIZE_BYTE;
      iacClosedLoopTable.values = configPage6.iacCLValues;
      iacClosedLoopTable.axisX = configPage6.iacBins;

      iacCrankStepsTable.xSize = 4;
      iacCrankStepsTable.valueSize = SIZE_BYTE;
      iacCrankStepsTable.axisSize = SIZE_BYTE;
      iacCrankStepsTable.values = configPage6.iacCrankSteps;
      iacCrankStepsTable.axisX = configPage6.iacCrankBins;
      iacStepTime_uS = configPage6.iacStepTime * 1000;
      iacCoolTime_uS = configPage9.iacCoolTime * 1000;

      if( completedHomeSteps < (configPage6.iacStepHome * 3) )
      {
        //Change between modes running make engine stall
        completedHomeSteps = 0;
        idleStepper.curIdleStep = 0;
        idleStepper.stepperStatus = SOFF;
      }

      if (! configPage9.iacStepperInv)
      {
        idleStepper.lessAirDirection = STEPPER_BACKWARD;
        idleStepper.moreAirDirection = STEPPER_FORWARD;
      }
      else
      {
        idleStepper.lessAirDirection = STEPPER_FORWARD;
        idleStepper.moreAirDirection = STEPPER_BACKWARD;
      }

      idlePID.SetSampleTime(250); //4Hz means 250ms
      idlePID.SetOutputLimits((configPage6.iacOLStepVal[9]*3)<<1, (configPage9.iacMaxSteps * 3)<<2); //Maximum number of steps; always less than home steps count.
      idlePID.SetTunings(configPage6.idleKP, configPage6.idleKI, configPage6.idleKD);
      idlePID.SetMode(AUTOMATIC); //Turn PID on
      configPage6.iacPWMrun = false; // just in case. This needs to be false with stepper idle
      idle_pid_target_value = 0;
      idlePID.Initialize();
      break;

    default:
      //Well this just shouldn't happen
      break;
  }

  initialiseIdleUpOutput();

  idleInitComplete = configPage6.iacAlgorithm; //Sets which idle method was initialised
  currentStatus.idleLoad = 0;
}

void initialiseIdleUpOutput()
{
  if (configPage2.idleUpOutputInv == 1) { idleUpOutputHIGH = LOW; idleUpOutputLOW = HIGH; }
  else { idleUpOutputHIGH = HIGH; idleUpOutputLOW = LOW; }

  digitalWrite(pinIdleUpOutput, idleUpOutputLOW); //Initiallise program with the idle up output in the off state
  currentStatus.idleUpOutputActive = false;

  idleUpOutput_pin_port = portOutputRegister(digitalPinToPort(pinIdleUpOutput));
  idleUpOutput_pin_mask = digitalPinToBitMask(pinIdleUpOutput);
}

/*
Checks whether a step is currently underway or whether the motor is in 'cooling' state (ie whether it's ready to begin another step or not)
Returns:
True: If a step is underway or motor is 'cooling'
False: If the motor is ready for another step
*/
static inline byte checkForStepping()
{
  bool isStepping = false;
  unsigned int timeCheck;
  
  if( (idleStepper.stepperStatus == STEPPING) || (idleStepper.stepperStatus == COOLING) )
  {
    if (idleStepper.stepperStatus == STEPPING)
    {
      timeCheck = iacStepTime_uS;
    }
    else 
    {
      timeCheck = iacCoolTime_uS;
    }

    if(micros_safe() > (idleStepper.stepStartTime + timeCheck) )
    {         
      if(idleStepper.stepperStatus == STEPPING)
      {
        //Means we're currently in a step, but it needs to be turned off
        digitalWrite(pinStepperStep, LOW); //Turn off the step
        idleStepper.stepStartTime = micros_safe();
        
        // if there is no cool time we can miss that step out completely.
        if (iacCoolTime_uS > 0)
        {
          idleStepper.stepperStatus = COOLING; //'Cooling' is the time the stepper needs to sit in LOW state before the next step can be made
        }
        else
        {
          idleStepper.stepperStatus = SOFF;  
        }
          
        isStepping = true;
      }
      else
      {
        //Means we're in COOLING status but have been in this state long enough. Go into off state
        idleStepper.stepperStatus = SOFF;
        digitalWrite(pinStepperEnable, HIGH); //Disable the DRV8825
      }
    }
    else
    {
      //Means we're in a step, but it doesn't need to turn off yet. No further action at this time
      isStepping = true;
    }
  }
  return isStepping;
}

/*
Performs a step
*/
static inline void doStep()
{
  if ( (idleStepper.targetIdleStep <= (idleStepper.curIdleStep - configPage6.iacStepHyster)) || (idleStepper.targetIdleStep >= (idleStepper.curIdleStep + configPage6.iacStepHyster)) ) //Hysteris check
  {
    // the home position for a stepper is pintle fully seated, i.e. no airflow.
    if(idleStepper.targetIdleStep < idleStepper.curIdleStep)
    {
      // we are moving toward the home position (reducing air)
      digitalWrite(pinStepperDir, idleStepper.lessAirDirection);
      idleStepper.curIdleStep--;
    }
    else
    if (idleStepper.targetIdleStep > idleStepper.curIdleStep)
    {
      // we are moving away from the home position (adding air).
      digitalWrite(pinStepperDir, idleStepper.moreAirDirection);
      idleStepper.curIdleStep++;
    }

    digitalWrite(pinStepperEnable, LOW); //Enable the DRV8825
    digitalWrite(pinStepperStep, HIGH);
    idleStepper.stepStartTime = micros_safe();
    idleStepper.stepperStatus = STEPPING;
    idleOn = true;
  }
}

/*
Checks whether the stepper has been homed yet. If it hasn't, will handle the next step
Returns:
True: If the system has been homed. No other action is taken
False: If the motor has not yet been homed. Will also perform another homing step.
*/
static inline byte isStepperHomed()
{
  bool isHomed = true; //As it's the most common scenario, default value is true
  if( completedHomeSteps < (configPage6.iacStepHome * 3) ) //Home steps are divided by 3 from TS
  {
    digitalWrite(pinStepperDir, idleStepper.lessAirDirection); //homing the stepper closes off the air bleed
    digitalWrite(pinStepperEnable, LOW); //Enable the DRV8825
    digitalWrite(pinStepperStep, HIGH);
    idleStepper.stepStartTime = micros_safe();
    idleStepper.stepperStatus = STEPPING;
    completedHomeSteps++;
    idleOn = true;
    isHomed = false;
  }
  return isHomed;
}

void idleControl()
{
  if( idleInitComplete != configPage6.iacAlgorithm) { initialiseIdle(); }
  if( (currentStatus.RPM > 0) || (configPage6.iacPWMrun == true) ) { enableIdle(); }

  //Check whether the idleUp is active
  if (configPage2.idleUpEnabled == true)
  {
    if (configPage2.idleUpPolarity == 0) { currentStatus.idleUpActive = !digitalRead(pinIdleUp); } //Normal mode (ground switched)
    else { currentStatus.idleUpActive = digitalRead(pinIdleUp); } //Inverted mode (5v activates idleUp)

    if (configPage2.idleUpOutputEnabled  == true)
    {
      if (currentStatus.idleUpActive == true)
      {
        digitalWrite(pinIdleUpOutput, idleUpOutputHIGH);
        currentStatus.idleUpOutputActive = true;
      }
      else
      {
        digitalWrite(pinIdleUpOutput, idleUpOutputLOW);
        currentStatus.idleUpOutputActive = false;
      }      
    }
  }
  else { currentStatus.idleUpActive = false; }

  bool PID_computed = false;
  switch(configPage6.iacAlgorithm)
  {
    case IAC_ALGORITHM_NONE:       //Case 0 is no idle control ('None')
      break;

    case IAC_ALGORITHM_ONOFF:      //Case 1 is on/off idle control
      if ( (currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET) < configPage6.iacFastTemp) //All temps are offset by 40 degrees
      {
        digitalWrite(pinIdle1, HIGH);
        idleOn = true;
        BIT_SET(currentStatus.spark, BIT_SPARK_IDLE); //Turn the idle control flag on
      }
      else if (idleOn)
      {
        digitalWrite(pinIdle1, LOW); 
        idleOn = false; 
        BIT_CLEAR(currentStatus.spark, BIT_SPARK_IDLE); //Turn the idle control flag on
      }
      break;

    case IAC_ALGORITHM_PWM_OL:      //Case 2 is PWM open loop
      //Check for cranking pulsewidth
      if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
      {
        //Currently cranking. Use the cranking table
        currentStatus.idleDuty = table2D_getValue(&iacCrankDutyTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //All temps are offset by 40 degrees
      }
      else if ( !BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN))
      {
        if( configPage6.iacPWMrun == true)
        {
          //Engine is not running or cranking, but the run before crank flag is set. Use the cranking table
          currentStatus.idleDuty = table2D_getValue(&iacCrankDutyTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //All temps are offset by 40 degrees
        }
      }
      else
      {
        
        if ( runSecsX10 < configPage2.idleTaperTime )
        {
          //Tapering between cranking IAC value and running
          currentStatus.idleDuty = map(runSecsX10, 0, configPage2.idleTaperTime,\
          table2D_getValue(&iacCrankDutyTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET),\
          table2D_getValue(&iacPWMTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET));
        }
        else
        {
          //Standard running
          currentStatus.idleDuty = table2D_getValue(&iacPWMTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //All temps are offset by 40 degrees
        }
      }

      if(currentStatus.idleUpActive == true) { currentStatus.idleDuty += configPage2.idleUpAdder; } //Add Idle Up amount if active
      if( currentStatus.idleDuty > 100 ) { currentStatus.idleDuty = 100; } //Safety Check
      if( currentStatus.idleDuty == 0 ) 
      { 
        disableIdle();
        BIT_CLEAR(currentStatus.spark, BIT_SPARK_IDLE); //Turn the idle control flag off
        break; 
      }
      BIT_SET(currentStatus.spark, BIT_SPARK_IDLE); //Turn the idle control flag on
      idle_pwm_target_value = percentage(currentStatus.idleDuty, idle_pwm_max_count);
      currentStatus.idleLoad = currentStatus.idleDuty;
      idleOn = true;
      
      break;

    case IAC_ALGORITHM_PWM_CL:    //Case 3 is PWM closed loop
        //No cranking specific value for closed loop (yet?)
      if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
      {
        //Currently cranking. Use the cranking table
        currentStatus.idleDuty = table2D_getValue(&iacCrankDutyTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //All temps are offset by 40 degrees
        currentStatus.idleLoad = currentStatus.idleDuty;
        idle_pwm_target_value = percentage(currentStatus.idleDuty, idle_pwm_max_count);
        idle_pid_target_value = idle_pwm_target_value << 2; //Resolution increased
        idlePID.Initialize(); //Update output to smooth transition
      }
      else if ( !BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN))
      {
        if( configPage6.iacPWMrun == true)
        {
          //Engine is not running or cranking, but the run before crank flag is set. Use the cranking table
          currentStatus.idleDuty = table2D_getValue(&iacCrankDutyTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //All temps are offset by 40 degrees
          currentStatus.idleLoad = currentStatus.idleDuty;
          idle_pwm_target_value = percentage(currentStatus.idleDuty, idle_pwm_max_count);
        }
      }
      else
      {
        currentStatus.CLIdleTarget = (byte)table2D_getValue(&iacClosedLoopTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //All temps are offset by 40 degrees
        idle_cl_target_rpm = (uint16_t)currentStatus.CLIdleTarget * 10; //Multiply the byte target value back out by 10
        if( (idleCounter & 31) == 1) { idlePID.SetTunings(configPage6.idleKP, configPage6.idleKI, configPage6.idleKD); } //This only needs to be run very infrequently, once every 32 calls to idleControl(). This is approx. once per second

        PID_computed = idlePID.Compute(true);
        if(PID_computed == true)
        {
          idle_pwm_target_value = idle_pid_target_value>>2; //increased resolution
          if( idle_pwm_target_value == 0 )
          { 
            disableIdle(); 
            BIT_CLEAR(currentStatus.spark, BIT_SPARK_IDLE); //Turn the idle control flag off
            break; 
          }
          BIT_SET(currentStatus.spark, BIT_SPARK_IDLE); //Turn the idle control flag on
          currentStatus.idleLoad = ((unsigned long)(idle_pwm_target_value * 100UL) / idle_pwm_max_count);
          if(currentStatus.idleUpActive == true) { currentStatus.idleDuty += configPage2.idleUpAdder; } //Add Idle Up amount if active

        }
        idleCounter++;
      }  
      break;


    case IAC_ALGORITHM_PWM_OLCL: //case 6 is PWM Open Loop table as feedforward term plus closed loop. 
      //No cranking specific value for closed loop (yet?)
      if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
      {
        //Currently cranking. Use the cranking table
        currentStatus.idleDuty = table2D_getValue(&iacCrankDutyTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //All temps are offset by 40 degrees
        currentStatus.idleLoad = currentStatus.idleDuty;
        idle_pwm_target_value = percentage(currentStatus.idleDuty, idle_pwm_max_count);
        idle_pid_target_value = idle_pwm_target_value << 2; //Resolution increased
        idlePID.Initialize(); //Update output to smooth transition
      }
      else
      {
        //Read the OL table as feedforward term
        FeedForwardTerm = percentage(table2D_getValue(&iacPWMTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET), idle_pwm_max_count<<2); //All temps are offset by 40 degrees
    
        currentStatus.CLIdleTarget = (byte)table2D_getValue(&iacClosedLoopTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //All temps are offset by 40 degrees
        idle_cl_target_rpm = (uint16_t)currentStatus.CLIdleTarget * 10; //Multiply the byte target value back out by 10
        if( (idleCounter & 31) == 1) { idlePID.SetTunings(configPage6.idleKP, configPage6.idleKI, configPage6.idleKD); } //This only needs to be run very infrequently, once every 32 calls to idleControl(). This is approx. once per 9 seconds
        if((currentStatus.RPM - idle_cl_target_rpm > configPage2.iacRPMlimitHysteresis*10) || (currentStatus.TPS > configPage2.iacTPSlimit)){ //reset integeral to zero when TPS is bigger than set value in TS (opening throttle so not idle anymore). OR when RPM higher than Idle Target + RPM Histeresis (comming back from high rpm with throttle closed) 
          idlePID.ResetIntegeral();
        }
        PID_computed = idlePID.Compute(true, FeedForwardTerm);

        if(PID_computed == true)
        {
          idle_pwm_target_value = idle_pid_target_value>>2; //increased resolution
          if( idle_pwm_target_value == 0 )
          { 
            disableIdle(); 
            BIT_CLEAR(currentStatus.spark, BIT_SPARK_IDLE); //Turn the idle control flag off
            break; 
          }
          BIT_SET(currentStatus.spark, BIT_SPARK_IDLE); //Turn the idle control flag on
          currentStatus.idleLoad = ((unsigned long)(idle_pwm_target_value * 100UL) / idle_pwm_max_count);
          if(currentStatus.idleUpActive == true) { currentStatus.idleDuty += configPage2.idleUpAdder; } //Add Idle Up amount if active

        }
        idleCounter++;
      }
        
    break;


    case IAC_ALGORITHM_STEP_OL:    //Case 4 is open loop stepper control
      //First thing to check is whether there is currently a step going on and if so, whether it needs to be turned off
      if( (checkForStepping() == false) && (isStepperHomed() == true) ) //Check that homing is complete and that there's not currently a step already taking place. MUST BE IN THIS ORDER!
      {
        //Check for cranking pulsewidth
        if( !BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN) ) //If ain't running it means off or cranking
        {
          //Currently cranking. Use the cranking table
          idleStepper.targetIdleStep = table2D_getValue(&iacCrankStepsTable, (currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET)) * 3; //All temps are offset by 40 degrees. Step counts are divided by 3 in TS. Multiply back out here
          if(currentStatus.idleUpActive == true) { idleStepper.targetIdleStep += configPage2.idleUpAdder; } //Add Idle Up amount if active

          //limit to the configured max steps. This must include any idle up adder, to prevent over-opening.
          if (idleStepper.targetIdleStep > (configPage9.iacMaxSteps * 3) )
          {
            idleStepper.targetIdleStep = configPage9.iacMaxSteps * 3;
          }

          doStep();
        }
        else
        {
          //Standard running
          if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ) && (currentStatus.RPM > 0))
          {
            if ( runSecsX10 < configPage2.idleTaperTime )
            {
              //Tapering between cranking IAC value and running
              idleStepper.targetIdleStep = map(runSecsX10, 0, configPage2.idleTaperTime,\
              table2D_getValue(&iacCrankStepsTable, (currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET)) * 3,\
              table2D_getValue(&iacStepTable, (currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET)) * 3);
            }
            else
            {
              //Standard running
              idleStepper.targetIdleStep = table2D_getValue(&iacStepTable, (currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET)) * 3; //All temps are offset by 40 degrees. Step counts are divided by 3 in TS. Multiply back out here
            }
            if(currentStatus.idleUpActive == true) { idleStepper.targetIdleStep += configPage2.idleUpAdder; } //Add Idle Up amount if active
            iacStepTime_uS = configPage6.iacStepTime * 1000;
            iacCoolTime_uS = configPage9.iacCoolTime * 1000;

            //limit to the configured max steps. This must include any idle up adder, to prevent over-opening.
            if (idleStepper.targetIdleStep > (configPage9.iacMaxSteps * 3) )
            {
              idleStepper.targetIdleStep = configPage9.iacMaxSteps * 3;
            }
          }
          doStep();
        }
        if( ((uint16_t)configPage9.iacMaxSteps * 3) > 255 ) { currentStatus.idleLoad = idleStepper.curIdleStep / 2; }//Current step count (Divided by 2 for byte)
        else { currentStatus.idleLoad = idleStepper.curIdleStep; }
      }
      //Set or clear the idle active flag
      if(idleStepper.targetIdleStep != idleStepper.curIdleStep) { BIT_SET(currentStatus.spark, BIT_SPARK_IDLE); }
      else { BIT_CLEAR(currentStatus.spark, BIT_SPARK_IDLE); }
      break;

    case IAC_ALGORITHM_STEP_OLCL:  //Case 7 is closed+open loop stepper control
    case IAC_ALGORITHM_STEP_CL:    //Case 5 is closed loop stepper control
      //First thing to check is whether there is currently a step going on and if so, whether it needs to be turned off
      if( (checkForStepping() == false) && (isStepperHomed() == true) ) //Check that homing is complete and that there's not currently a step already taking place. MUST BE IN THIS ORDER!
      {
        if( !BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN) ) //If ain't running it means off or cranking
        {
          //Currently cranking. Use the cranking table
          idleStepper.targetIdleStep = table2D_getValue(&iacCrankStepsTable, (currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET)) * 3; //All temps are offset by 40 degrees. Step counts are divided by 3 in TS. Multiply back out here
          if(currentStatus.idleUpActive == true) { idleStepper.targetIdleStep += configPage2.idleUpAdder; } //Add Idle Up amount if active

          //limit to the configured max steps. This must include any idle up adder, to prevent over-opening.
          if (idleStepper.targetIdleStep > (configPage9.iacMaxSteps * 3) )
          {
            idleStepper.targetIdleStep = configPage9.iacMaxSteps * 3;
          }
          
          doStep();
          idle_pid_target_value = idleStepper.targetIdleStep << 2; //Resolution increased
          idlePID.ResetIntegeral();
          FeedForwardTerm = idle_pid_target_value;
        }
        else 
        {
          if( BIT_CHECK(LOOP_TIMER, BIT_TIMER_10HZ) )
          {
            currentStatus.CLIdleTarget = (byte)table2D_getValue(&iacClosedLoopTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //All temps are offset by 40 degrees
            idle_cl_target_rpm = (uint16_t)currentStatus.CLIdleTarget * 10; //Multiply the byte target value back out by 10
            if( runSecsX10 < configPage2.idleTaperTime )
            {
              uint16_t minValue = table2D_getValue(&iacCrankStepsTable, (currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET)) * 3;
              if( idle_pid_target_value < minValue<<2 ) { idle_pid_target_value = minValue<<2; }
              uint16_t maxValue = idle_pid_target_value>>2;
              if( configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OLCL ) { maxValue = table2D_getValue(&iacStepTable, (currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET)) * 3; }

              //Tapering between cranking IAC value and running
              FeedForwardTerm = map(runSecsX10, 0, configPage2.idleTaperTime, minValue, maxValue)<<2;
              idle_pid_target_value = FeedForwardTerm;
            }
            else if (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OLCL)
            {
              //Standard running
              FeedForwardTerm = (table2D_getValue(&iacStepTable, (currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET)) * 3)<<2; //All temps are offset by 40 degrees. Step counts are divided by 3 in TS. Multiply back out here
              //reset integeral to zero when TPS is bigger than set value in TS (opening throttle so not idle anymore). OR when RPM higher than Idle Target + RPM Histeresis (comming back from high rpm with throttle closed) 
              if (((currentStatus.RPM - idle_cl_target_rpm) > configPage2.iacRPMlimitHysteresis*10) || (currentStatus.TPS > configPage2.iacTPSlimit) || lastDFCOValue )
              {
                idlePID.ResetIntegeral();
              }
            }
            else { FeedForwardTerm = idle_pid_target_value; }
          }

          PID_computed = idlePID.Compute(true, FeedForwardTerm);

          //If DFCO conditions are met keep output from changing
          if( (currentStatus.TPS > configPage2.iacTPSlimit) || lastDFCOValue
          || ((configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OLCL) && (runSecsX10 < configPage2.idleTaperTime)) )
          {
            idle_pid_target_value = FeedForwardTerm;
          }
          idleStepper.targetIdleStep = idle_pid_target_value>>2; //Increase resolution

        }
        if(currentStatus.idleUpActive == true) { idleStepper.targetIdleStep += configPage2.idleUpAdder; } //Add Idle Up amount if active
        //limit to the configured max steps. This must include any idle up adder, to prevent over-opening.
        if (idleStepper.targetIdleStep > (configPage9.iacMaxSteps * 3) )
        {
          idleStepper.targetIdleStep = configPage9.iacMaxSteps * 3;
        }
        doStep();

        if( ( (uint16_t)configPage9.iacMaxSteps * 3) > 255 ) { currentStatus.idleLoad = idleStepper.curIdleStep / 2; }//Current step count (Divided by 2 for byte)
        else { currentStatus.idleLoad = idleStepper.curIdleStep; }
      }
      //Set or clear the idle active flag
      if(idleStepper.targetIdleStep != idleStepper.curIdleStep) { BIT_SET(currentStatus.spark, BIT_SPARK_IDLE); }
      else { BIT_CLEAR(currentStatus.spark, BIT_SPARK_IDLE); }
      if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_1HZ)) //Use timer flag instead idle count
      {
        //This only needs to be run very infrequently, once per second
        idlePID.SetTunings(configPage6.idleKP, configPage6.idleKI, configPage6.idleKD);
        iacStepTime_uS = configPage6.iacStepTime * 1000;
        iacCoolTime_uS = configPage9.iacCoolTime * 1000;
      }
      break;

    default:
      //There really should be a valid idle type
      break;
  }
  lastDFCOValue = BIT_CHECK(currentStatus.status1, BIT_STATUS1_DFCO);

  //Check for 100% DC on PWM idle
  if( (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_OL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_CL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_OLCL) )
  {
    if(currentStatus.idleLoad == 100)
    {
      IDLE_TIMER_DISABLE();
      IDLE_PIN_HIGH();
    }
    else if(currentStatus.idleLoad > 0) { IDLE_TIMER_ENABLE(); }
  }
}


//This function simply turns off the idle PWM and sets the pin low
void disableIdle()
{
  if( (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_CL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_OL) )
  {
    IDLE_TIMER_DISABLE();
    digitalWrite(pinIdle1, LOW);
  }
  else if( (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_CL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OLCL) )
  {
    //Only disable the stepper motor if homing is completed
    if( (checkForStepping() == false) && (isStepperHomed() == true) )
    {
        /* for open loop stepper we should just move to the cranking position when
           disabling idle, since the only time this function is called in this scenario
           is if the engine stops.
        */
        idleStepper.targetIdleStep = table2D_getValue(&iacCrankStepsTable, (currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET)) * 3; //All temps are offset by 40 degrees. Step counts are divided by 3 in TS. Multiply back out here
        if(currentStatus.idleUpActive == true) { idleStepper.targetIdleStep += configPage2.idleUpAdder; } //Add Idle Up amount if active?

        //limit to the configured max steps. This must include any idle up adder, to prevent over-opening.
        if (idleStepper.targetIdleStep > (configPage9.iacMaxSteps * 3) )
        {
          idleStepper.targetIdleStep = configPage9.iacMaxSteps * 3;
        }
        idle_pid_target_value = idleStepper.targetIdleStep<<2;
    }
  }
  BIT_CLEAR(currentStatus.spark, BIT_SPARK_IDLE); //Turn the idle control flag off
  currentStatus.idleLoad = 0;
}

#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER1_COMPC_vect)
#else
void idleInterrupt() //Most ARM chips can simply call a function
#endif
{
  if (idle_pwm_state)
  {
    if (configPage6.iacPWMdir == 0)
    {
      //Normal direction
      *idle_pin_port &= ~(idle_pin_mask);  // Switch pin to low (1 pin mode)
      if(configPage6.iacChannels == 1) { *idle2_pin_port |= (idle2_pin_mask); } //If 2 idle channels are in use, flip idle2 to be the opposite of idle1
    }
    else
    {
      //Reversed direction
      *idle_pin_port |= (idle_pin_mask);  // Switch pin high
      if(configPage6.iacChannels == 1) { *idle2_pin_port &= ~(idle2_pin_mask); } //If 2 idle channels are in use, flip idle2 to be the opposite of idle1
    }
    SET_COMPARE(IDLE_COMPARE, IDLE_COUNTER + (idle_pwm_max_count - idle_pwm_cur_value) );
    idle_pwm_state = false;
  }
  else
  {
    if (configPage6.iacPWMdir == 0)
    {
      //Normal direction
      *idle_pin_port |= (idle_pin_mask);  // Switch pin high
      if(configPage6.iacChannels == 1) { *idle2_pin_port &= ~(idle2_pin_mask); } //If 2 idle channels are in use, flip idle2 to be the opposite of idle1
    }
    else
    {
      //Reversed direction
      *idle_pin_port &= ~(idle_pin_mask);  // Switch pin to low (1 pin mode)
      if(configPage6.iacChannels == 1) { *idle2_pin_port |= (idle2_pin_mask); } //If 2 idle channels are in use, flip idle2 to be the opposite of idle1
    }
    SET_COMPARE(IDLE_COMPARE, IDLE_COUNTER + idle_pwm_target_value);
    idle_pwm_cur_value = idle_pwm_target_value;
    idle_pwm_state = true;
  }
}
