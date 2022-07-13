/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
#include "globals.h"
#include "auxiliaries.h"
#include "maths.h"
#include "src/PID_v1/PID_v1.h"
#include "decoders.h"
#include "timers.h"

//Old PID method. Retained in case the new one has issues
//integerPID boostPID(&MAPx100, &boost_pwm_target_value, &boostTargetx100, configPage6.boostKP, configPage6.boostKI, configPage6.boostKD, DIRECT);
integerPID_ideal boostPID(&currentStatus.MAP, &currentStatus.boostDuty , &currentStatus.boostTarget, &configPage10.boostSens, &configPage10.boostIntv, configPage6.boostKP, configPage6.boostKI, configPage6.boostKD, DIRECT); //This is the PID object if that algorithm is used. Needs to be global as it maintains state outside of each function call
integerPID vvtPID(&vvt_pid_current_angle, &currentStatus.vvt1Duty, &vvt_pid_target_angle, configPage10.vvtCLKP, configPage10.vvtCLKI, configPage10.vvtCLKD, configPage6.vvtPWMdir); //This is the PID object if that algorithm is used. Needs to be global as it maintains state outside of each function call
integerPID vvt2PID(&vvt2_pid_current_angle, &currentStatus.vvt2Duty, &vvt2_pid_target_angle, configPage10.vvtCLKP, configPage10.vvtCLKI, configPage10.vvtCLKD, configPage4.vvt2PWMdir); //This is the PID object if that algorithm is used. Needs to be global as it maintains state outside of each function call

/*
Fan control
*/
void initialiseFan()
{
  fan_pin_port = portOutputRegister(digitalPinToPort(pinFan));
  fan_pin_mask = digitalPinToBitMask(pinFan);
  FAN_OFF();  //Initialise program with the fan in the off state
  BIT_CLEAR(currentStatus.status4, BIT_STATUS4_FAN);
  currentStatus.fanDuty = 0;

  #if defined(PWM_FAN_AVAILABLE)
    DISABLE_FAN_TIMER(); //disable FAN timer if available
    if ( configPage2.fanEnable == 2 ) // PWM Fan control
    {
      #if defined(CORE_TEENSY)
        fan_pwm_max_count = 1000000L / (32 * configPage6.fanFreq * 2); //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
      #endif
      fan_pwm_value = 0;
    }
  #endif
}

void fanControl()
{
  if( configPage2.fanEnable == 1 ) // regular on/off fan control
  {
    int onTemp = (int)configPage6.fanSP - CALIBRATION_TEMPERATURE_OFFSET;
    int offTemp = onTemp - configPage6.fanHyster;
    bool fanPermit = false;

    if ( configPage2.fanWhenOff == true) { fanPermit = true; }
    else { fanPermit = BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN); }

    if ( (currentStatus.coolant >= onTemp) && (fanPermit == true) )
    {
      //Fan needs to be turned on.
      if(BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && (configPage2.fanWhenCranking == 0))
      {
        //If the user has elected to disable the fan during cranking, make sure it's off 
        FAN_OFF();
        BIT_CLEAR(currentStatus.status4, BIT_STATUS4_FAN);
      }
      else 
      {
        FAN_ON();
        BIT_SET(currentStatus.status4, BIT_STATUS4_FAN);
      }
    }
    else if ( (currentStatus.coolant <= offTemp) || (!fanPermit) )
    {
      //Fan needs to be turned off. 
      FAN_OFF();
      BIT_CLEAR(currentStatus.status4, BIT_STATUS4_FAN);
    }
  }
  else if( configPage2.fanEnable == 2 )// PWM Fan control
  {
    bool fanPermit = false;
    if ( configPage2.fanWhenOff == true) { fanPermit = true; }
    else { fanPermit = BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN); }
    if (fanPermit == true)
      {
      if(BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) && (configPage2.fanWhenCranking == 0))
      {
        currentStatus.fanDuty = 0; //If the user has elected to disable the fan during cranking, make sure it's off 
        BIT_CLEAR(currentStatus.status4, BIT_STATUS4_FAN);
        #if defined(PWM_FAN_AVAILABLE)//PWM fan not available on Arduino MEGA
          DISABLE_FAN_TIMER();
        #endif
      }
      else
      {
        currentStatus.fanDuty = table2D_getValue(&fanPWMTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //In normal situation read PWM duty from the table
        #if defined(PWM_FAN_AVAILABLE)
          fan_pwm_value = halfPercentage(currentStatus.fanDuty, fan_pwm_max_count); //update FAN PWM value last
          if (currentStatus.fanDuty > 0)
          {
            ENABLE_FAN_TIMER();
            BIT_SET(currentStatus.status4, BIT_STATUS4_FAN);
          }
        #endif
      }
    }
    else if (!fanPermit)
    {
      currentStatus.fanDuty = 0; ////If the user has elected to disable the fan when engine is not running, make sure it's off 
      BIT_CLEAR(currentStatus.status4, BIT_STATUS4_FAN);
    }

    #if defined(PWM_FAN_AVAILABLE)
      if(currentStatus.fanDuty == 0)
      {
        //Make sure fan has 0% duty)
        FAN_OFF();
        BIT_CLEAR(currentStatus.status4, BIT_STATUS4_FAN);
        DISABLE_FAN_TIMER();
      }
      else if (currentStatus.fanDuty == 200)
      {
        //Make sure fan has 100% duty
        FAN_ON();
        BIT_SET(currentStatus.status4, BIT_STATUS4_FAN);
        DISABLE_FAN_TIMER();
      }
    #else //Just in case if user still has selected PWM fan in TS, even though it warns that it doesn't work on mega.
      if(currentStatus.fanDuty == 0)
      {
        //Make sure fan has 0% duty)
        FAN_OFF();
        BIT_CLEAR(currentStatus.status4, BIT_STATUS4_FAN);
      }
      else if (currentStatus.fanDuty > 0)
      {
        //Make sure fan has 100% duty
        FAN_ON();
        BIT_SET(currentStatus.status4, BIT_STATUS4_FAN);
      }
    #endif
  }
}

void initialiseAuxPWM()
{
  boost_pin_port = portOutputRegister(digitalPinToPort(pinBoost));
  boost_pin_mask = digitalPinToBitMask(pinBoost);
  vvt1_pin_port = portOutputRegister(digitalPinToPort(pinVVT_1));
  vvt1_pin_mask = digitalPinToBitMask(pinVVT_1);
  vvt2_pin_port = portOutputRegister(digitalPinToPort(pinVVT_2));
  vvt2_pin_mask = digitalPinToBitMask(pinVVT_2);
  n2o_stage1_pin_port = portOutputRegister(digitalPinToPort(configPage10.n2o_stage1_pin));
  n2o_stage1_pin_mask = digitalPinToBitMask(configPage10.n2o_stage1_pin);
  n2o_stage2_pin_port = portOutputRegister(digitalPinToPort(configPage10.n2o_stage2_pin));
  n2o_stage2_pin_mask = digitalPinToBitMask(configPage10.n2o_stage2_pin);
  n2o_arming_pin_port = portInputRegister(digitalPinToPort(configPage10.n2o_arming_pin));
  n2o_arming_pin_mask = digitalPinToBitMask(configPage10.n2o_arming_pin);

  //This is a safety check that will be true if the board is uninitialised. This prevents hangs on a new board that could otherwise try to write to an invalid pin port/mask (Without this a new Teensy 4.x hangs on startup)
  //The n2o_minTPS variable is capped at 100 by TS, so 255 indicates a new board.
  if(configPage10.n2o_minTPS == 255) { configPage10.n2o_enable = 0; }

  if(configPage10.n2o_enable > 0)
  {
    //The pin modes are only set if the if n2o is enabled to prevent them conflicting with other outputs. 
    if(configPage10.n2o_pin_polarity == 1) { pinMode(configPage10.n2o_arming_pin, INPUT_PULLUP); }
    else { pinMode(configPage10.n2o_arming_pin, INPUT); }
  }

  boostPID.SetOutputLimits(configPage2.boostMinDuty, configPage2.boostMaxDuty);
  if(configPage6.boostMode == BOOST_MODE_SIMPLE) { boostPID.SetTunings(SIMPLE_BOOST_P, SIMPLE_BOOST_I, SIMPLE_BOOST_D); }
  else { boostPID.SetTunings(configPage6.boostKP, configPage6.boostKI, configPage6.boostKD); }

  if( configPage6.vvtEnabled > 0)
  {
    currentStatus.vvt1Angle = 0;
    currentStatus.vvt2Angle = 0;

    #if defined(CORE_AVR)
      vvt_pwm_max_count = 1000000L / (16 * configPage6.vvtFreq * 2); //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
    #elif defined(CORE_TEENSY)
      vvt_pwm_max_count = 1000000L / (32 * configPage6.vvtFreq * 2); //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
    #endif

    if(configPage6.vvtMode == VVT_MODE_CLOSED_LOOP)
    {
      vvtPID.SetOutputLimits(configPage10.vvtCLminDuty, configPage10.vvtCLmaxDuty);
      vvtPID.SetTunings(configPage10.vvtCLKP, configPage10.vvtCLKI, configPage10.vvtCLKD);
      vvtPID.SetSampleTime(33); //30Hz is 33,33ms
      vvtPID.SetMode(AUTOMATIC); //Turn PID on
      if (configPage10.vvt2Enabled == 1) // same for VVT2 if it's enabled
      {
        vvt2PID.SetOutputLimits(configPage10.vvtCLminDuty, configPage10.vvtCLmaxDuty);
        vvt2PID.SetTunings(configPage10.vvtCLKP, configPage10.vvtCLKI, configPage10.vvtCLKD);
        vvt2PID.SetSampleTime(33); //30Hz is 33,33ms
        vvt2PID.SetMode(AUTOMATIC); //Turn PID on
      }
    }

    vvt1_pwm_value = 0;
    vvt2_pwm_value = 0;
    ENABLE_VVT_TIMER(); //Turn on the B compare unit (ie turn on the interrupt)
    BIT_CLEAR(currentStatus.status4, BIT_STATUS4_VVT1_ERROR);
    BIT_CLEAR(currentStatus.status4, BIT_STATUS4_VVT2_ERROR);
    vvtTimeHold = false;
    if (currentStatus.coolant >= (int)(configPage4.vvtMinClt - CALIBRATION_TEMPERATURE_OFFSET)) { vvtIsHot = true; } //Checks to see if coolant's already at operating temperature
  }
  if( (configPage6.vvtEnabled == 0) && (configPage10.wmiEnabled >= 1) )
  {
    // config wmi pwm output to use vvt output
    #if defined(CORE_AVR)
      vvt_pwm_max_count = 1000000L / (16 * configPage6.vvtFreq * 2); //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
    #elif defined(CORE_TEENSY)
      vvt_pwm_max_count = 1000000L / (32 * configPage6.vvtFreq * 2); //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
    #endif
    BIT_CLEAR(currentStatus.status4, BIT_STATUS4_WMI_EMPTY);
    currentStatus.wmiPW = 0;
    vvt1_pwm_value = 0;
    vvt2_pwm_value = 0;
    ENABLE_VVT_TIMER(); //Turn on the B compare unit (ie turn on the interrupt)
  }

  currentStatus.boostDuty = 0;
  boostCounter = 0;
  currentStatus.vvt1Duty = 0;
  currentStatus.vvt2Duty = 0;
  vvtCounter = 0;

  currentStatus.nitrous_status = NITROUS_OFF;

}

void boostByGear()
{
  if(configPage4.boostType == OPEN_LOOP_BOOST)
  {
    if( configPage9.boostByGearEnabled == 1 )
    {
      uint16_t combinedBoost = 0;
      switch (currentStatus.gear)
      {
        case 1:
          combinedBoost = ( ((uint16_t)configPage9.boostByGear1 * (uint16_t)get3DTableValue(&boostTable, (currentStatus.TPS * 2), currentStatus.RPM))  ) << 2;
          if( combinedBoost <= 10000 ){ currentStatus.boostDuty = combinedBoost; }
          else{ currentStatus.boostDuty = 10000; }
          break;
        case 2:
          combinedBoost = ( ((uint16_t)configPage9.boostByGear2 * (uint16_t)get3DTableValue(&boostTable, (currentStatus.TPS * 2), currentStatus.RPM))  ) << 2;
          if( combinedBoost <= 10000 ){ currentStatus.boostDuty = combinedBoost; }
          else{ currentStatus.boostDuty = 10000; }
          break;
        case 3:
          combinedBoost = ( ((uint16_t)configPage9.boostByGear3 * (uint16_t)get3DTableValue(&boostTable, (currentStatus.TPS * 2), currentStatus.RPM))  ) << 2;
          if( combinedBoost <= 10000 ){ currentStatus.boostDuty = combinedBoost; }
          else{ currentStatus.boostDuty = 10000; }
          break;
        case 4:
          combinedBoost = ( ((uint16_t)configPage9.boostByGear4 * (uint16_t)get3DTableValue(&boostTable, (currentStatus.TPS * 2), currentStatus.RPM))  ) << 2;
          if( combinedBoost <= 10000 ){ currentStatus.boostDuty = combinedBoost; }
          else{ currentStatus.boostDuty = 10000; }
          break;
        case 5:
          combinedBoost = ( ((uint16_t)configPage9.boostByGear5 * (uint16_t)get3DTableValue(&boostTable, (currentStatus.TPS * 2), currentStatus.RPM))  ) << 2;
          if( combinedBoost <= 10000 ){ currentStatus.boostDuty = combinedBoost; }
          else{ currentStatus.boostDuty = 10000; }
          break;
        case 6:
          combinedBoost = ( ((uint16_t)configPage9.boostByGear6 * (uint16_t)get3DTableValue(&boostTable, (currentStatus.TPS * 2), currentStatus.RPM))  ) << 2;
          if( combinedBoost <= 10000 ){ currentStatus.boostDuty = combinedBoost; }
          else{ currentStatus.boostDuty = 10000; }
          break;
        default:
          break;
      }
    }
    else if( configPage9.boostByGearEnabled == 2 ) 
    {
      switch (currentStatus.gear)
      {
        case 1:
          currentStatus.boostDuty = configPage9.boostByGear1 * 2 * 100;
          break;
        case 2:
          currentStatus.boostDuty = configPage9.boostByGear2 * 2 * 100;
          break;
        case 3:
          currentStatus.boostDuty = configPage9.boostByGear3 * 2 * 100;
          break;
        case 4:
          currentStatus.boostDuty = configPage9.boostByGear4 * 2 * 100;
          break;
        case 5:
          currentStatus.boostDuty = configPage9.boostByGear5 * 2 * 100;
          break;
        case 6:
          currentStatus.boostDuty = configPage9.boostByGear6 * 2 * 100;
          break;
        default:
          break;
      }
    }
  }
  else if (configPage4.boostType == CLOSED_LOOP_BOOST)
  {
    if( configPage9.boostByGearEnabled == 1 )
    {
      uint16_t combinedBoost = 0;
      switch (currentStatus.gear)
      {
        case 1:
          combinedBoost = ( ((uint16_t)configPage9.boostByGear1 * (uint16_t)get3DTableValue(&boostTable, (currentStatus.TPS * 2), currentStatus.RPM)) / 100 ) << 2;
          if( combinedBoost <= 511 ){ currentStatus.boostTarget = combinedBoost; }
          else{ currentStatus.boostTarget = 511; }
          break;
        case 2:
          combinedBoost = ( ((uint16_t)configPage9.boostByGear2 * (uint16_t)get3DTableValue(&boostTable, (currentStatus.TPS * 2), currentStatus.RPM)) / 100 ) << 2;
          if( combinedBoost <= 511 ){ currentStatus.boostTarget = combinedBoost; }
          else{ currentStatus.boostTarget = 511; }
          break;
        case 3:
          combinedBoost = ( ((uint16_t)configPage9.boostByGear3 * (uint16_t)get3DTableValue(&boostTable, (currentStatus.TPS * 2), currentStatus.RPM)) / 100 ) << 2;
          if( combinedBoost <= 511 ){ currentStatus.boostTarget = combinedBoost; }
          else{ currentStatus.boostTarget = 511; }
          break;
        case 4:
          combinedBoost = ( ((uint16_t)configPage9.boostByGear4 * (uint16_t)get3DTableValue(&boostTable, (currentStatus.TPS * 2), currentStatus.RPM)) / 100 ) << 2;
          if( combinedBoost <= 511 ){ currentStatus.boostTarget = combinedBoost; }
          else{ currentStatus.boostTarget = 511; }
          break;
        case 5:
          combinedBoost = ( ((uint16_t)configPage9.boostByGear5 * (uint16_t)get3DTableValue(&boostTable, (currentStatus.TPS * 2), currentStatus.RPM)) / 100 ) << 2;
          if( combinedBoost <= 511 ){ currentStatus.boostTarget = combinedBoost; }
          else{ currentStatus.boostTarget = 511; }
          break;
        case 6:
          combinedBoost = ( ((uint16_t)configPage9.boostByGear6 * (uint16_t)get3DTableValue(&boostTable, (currentStatus.TPS * 2), currentStatus.RPM)) / 100 ) << 2;
          if( combinedBoost <= 511 ){ currentStatus.boostTarget = combinedBoost; }
          else{ currentStatus.boostTarget = 511; }
          break;
        default:
          break;
      }
    }
    else if( configPage9.boostByGearEnabled == 2 ) 
    {
      switch (currentStatus.gear)
      {
        case 1:
          currentStatus.boostTarget = (configPage9.boostByGear1 << 1);
          break;
        case 2:
          currentStatus.boostTarget = (configPage9.boostByGear2 << 1);
          break;
        case 3:
          currentStatus.boostTarget = (configPage9.boostByGear3 << 1);
          break;
        case 4:
          currentStatus.boostTarget = (configPage9.boostByGear4 << 1);
          break;
        case 5:
          currentStatus.boostTarget = (configPage9.boostByGear5 << 1);
          break;
        case 6:
          currentStatus.boostTarget = (configPage9.boostByGear6 << 1);
          break;
        default:
          break;
      }
    }
  }
}

void boostControl()
{
  if( configPage6.boostEnabled==1 )
  {
    if(configPage4.boostType == OPEN_LOOP_BOOST)
    {
      //Open loop
      if ( (configPage9.boostByGearEnabled > 0) && (configPage2.vssMode > 1) ){ boostByGear(); }
      else{ currentStatus.boostDuty = get3DTableValue(&boostTable, (currentStatus.TPS * 2), currentStatus.RPM) * 2 * 100; }

      if(currentStatus.boostDuty > 10000) { currentStatus.boostDuty = 10000; } //Safety check
      if(currentStatus.boostDuty == 0) { DISABLE_BOOST_TIMER(); BOOST_PIN_LOW(); } //If boost duty is 0, shut everything down
      else
      {
        boost_pwm_target_value = ((unsigned long)(currentStatus.boostDuty) * boost_pwm_max_count) / 10000; //Convert boost duty (Which is a % multiplied by 100) to a pwm count
      }
    }
    else if (configPage4.boostType == CLOSED_LOOP_BOOST)
    {
      if( (boostCounter & 7) == 1) 
      { 
        if ( (configPage9.boostByGearEnabled > 0) && (configPage2.vssMode > 1) ){ boostByGear(); }
        else{ currentStatus.boostTarget = get3DTableValue(&boostTable, (currentStatus.TPS * 2), currentStatus.RPM) << 1; } //Boost target table is in kpa and divided by 2
      } 
      if(((configPage15.boostControlEnable == EN_BOOST_CONTROL_BARO) & (currentStatus.MAP >= currentStatus.baro)) | ((configPage15.boostControlEnable == EN_BOOST_CONTROL_FIXED) & (currentStatus.MAP >= configPage15.boostControlEnableThreshold))) //Only enables boost control above baro pressure or above user defined threshold (User defined level is usually set to boost with wastegate actuator only boost level)
      {
        //If flex fuel is enabled, there can be an adder to the boost target based on ethanol content
        if( configPage2.flexEnabled == 1 )
        {
          currentStatus.boostTarget += table2D_getValue(&flexBoostTable, currentStatus.ethanolPct);;
        }
        else
        {
          currentStatus.flexBoostCorrection = 0;
        }

        if(currentStatus.boostTarget > 0)
        {
          //This only needs to be run very infrequently, once every 16 calls to boostControl(). This is approx. once per second
          if( (boostCounter & 15) == 1)
          {
            boostPID.SetOutputLimits(configPage2.boostMinDuty, configPage2.boostMaxDuty);

            if(configPage6.boostMode == BOOST_MODE_SIMPLE) { boostPID.SetTunings(SIMPLE_BOOST_P, SIMPLE_BOOST_I, SIMPLE_BOOST_D); }
            else { boostPID.SetTunings(configPage6.boostKP, configPage6.boostKI, configPage6.boostKD); }
          }

          bool PIDcomputed = boostPID.Compute(get3DTableValue(&boostTableLookupDuty, currentStatus.boostTarget, currentStatus.RPM) * 100/2); //Compute() returns false if the required interval has not yet passed.
          if(currentStatus.boostDuty == 0) { DISABLE_BOOST_TIMER(); BOOST_PIN_LOW(); } //If boost duty is 0, shut everything down
          else
          {
            if(PIDcomputed == true)
            {
              boost_pwm_target_value = ((unsigned long)(currentStatus.boostDuty) * boost_pwm_max_count) / 10000; //Convert boost duty (Which is a % multiplied by 100) to a pwm count
            }
          }
        }
        else
        {
          //If boost target is 0, turn everything off
          boostDisable();
        }
      }
      else
      {
        boostPID.Initialize(); //This resets the ITerm value to prevent rubber banding
        //Boost control needs to have a high duty cycle if control is below threshold (baro or fixed value). This ensures the waste gate is closed as much as possible, this build boost as fast as possible.
        currentStatus.boostDuty = configPage15.boostDCWhenDisabled*100;
        boost_pwm_target_value = ((unsigned long)(currentStatus.boostDuty) * boost_pwm_max_count) / 10000; //Convert boost duty (Which is a % multiplied by 100) to a pwm count
        ENABLE_BOOST_TIMER(); //Turn on the compare unit (ie turn on the interrupt) if boost duty >0
        if(currentStatus.boostDuty==0){
          //If boost control does nothing disable PWM completely
          boostDisable();
        }     
      } //MAP above boost + hyster
    } //Open / Cloosed loop

    //Check for 100% duty cycle
    if(currentStatus.boostDuty >= 10000)
    {
      DISABLE_BOOST_TIMER(); //Turn off the compare unit (ie turn off the interrupt) if boost duty is 100%
      BOOST_PIN_HIGH(); //Turn on boost pin if duty is 100%
    }
    else if(currentStatus.boostDuty > 0)
    {
      ENABLE_BOOST_TIMER(); //Turn on the compare unit (ie turn on the interrupt) if boost duty is > 0
    }
    
  }
  else { // Disable timer channel and zero the flex boost correction status
    DISABLE_BOOST_TIMER();
    currentStatus.flexBoostCorrection = 0;
  }

  boostCounter++;
}

void vvtControl()
{
  if( (configPage6.vvtEnabled == 1) && (currentStatus.coolant >= (int)(configPage4.vvtMinClt - CALIBRATION_TEMPERATURE_OFFSET)) && (BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN)))
  {
    if(vvtTimeHold == false) 
    {
      vvtWarmTime = runSecsX10;
      vvtTimeHold = true;
    }

    //Calculate the current cam angle for miata trigger
    if( configPage4.TrigPattern == 9 ) { currentStatus.vvt1Angle = getCamAngle_Miata9905(); }

    if( (vvtIsHot == true) || ((runSecsX10 - vvtWarmTime) >= (configPage4.vvtDelay * VVT_TIME_DELAY_MULTIPLIER)) ) 
    {
      vvtIsHot = true;

      if( (configPage6.vvtMode == VVT_MODE_OPEN_LOOP) || (configPage6.vvtMode == VVT_MODE_ONOFF) )
      {
        //Lookup VVT duty based on either MAP or TPS
        if(configPage6.vvtLoadSource == VVT_LOAD_TPS) { currentStatus.vvt1Duty = get3DTableValue(&vvtTable, (currentStatus.TPS * 2), currentStatus.RPM); }
        else { currentStatus.vvt1Duty = get3DTableValue(&vvtTable, (currentStatus.MAP), currentStatus.RPM); }

        //VVT table can be used for controlling on/off switching. If this is turned on, then disregard any interpolation or non-binary values
        if( (configPage6.vvtMode == VVT_MODE_ONOFF) && (currentStatus.vvt1Duty < 200) ) { currentStatus.vvt1Duty = 0; }

        vvt1_pwm_value = halfPercentage(currentStatus.vvt1Duty, vvt_pwm_max_count);

        if (configPage10.vvt2Enabled == 1) // same for VVT2 if it's enabled
        {
          //Lookup VVT duty based on either MAP or TPS
          if(configPage6.vvtLoadSource == VVT_LOAD_TPS) { currentStatus.vvt2Duty = get3DTableValue(&vvt2Table, (currentStatus.TPS * 2), currentStatus.RPM); }
          else { currentStatus.vvt2Duty = get3DTableValue(&vvt2Table, (currentStatus.MAP), currentStatus.RPM); }

          //VVT table can be used for controlling on/off switching. If this is turned on, then disregard any interpolation or non-binary values
          if( (configPage6.vvtMode == VVT_MODE_ONOFF) && (currentStatus.vvt2Duty < 200) ) { currentStatus.vvt2Duty = 0; }

          vvt2_pwm_value = halfPercentage(currentStatus.vvt2Duty, vvt_pwm_max_count);
        }

      } //Open loop
      else if( (configPage6.vvtMode == VVT_MODE_CLOSED_LOOP) )
      {
        //Lookup VVT duty based on either MAP or TPS
        if(configPage6.vvtLoadSource == VVT_LOAD_TPS) { currentStatus.vvt1TargetAngle = get3DTableValue(&vvtTable, (currentStatus.TPS * 2), currentStatus.RPM); }
        else { currentStatus.vvt1TargetAngle = get3DTableValue(&vvtTable, currentStatus.MAP, currentStatus.RPM); }

        if( (vvtCounter & 31) == 1) { vvtPID.SetTunings(configPage10.vvtCLKP, configPage10.vvtCLKI, configPage10.vvtCLKD);  //This only needs to be run very infrequently, once every 32 calls to vvtControl(). This is approx. once per second
        vvtPID.SetControllerDirection(configPage6.vvtPWMdir); }

        // safety check that the cam angles are ok. The engine will be totally undriveable if the cam sensor is faulty and giving wrong cam angles, so if that happens, default to 0 duty.
        // This also prevents using zero or negative current angle values for PID adjustment, because those don't work in integer PID.
        if ( currentStatus.vvt1Angle <=  configPage10.vvtCLMinAng || currentStatus.vvt1Angle > configPage10.vvtCLMaxAng )
        {
          currentStatus.vvt1Duty = 0;
          vvt1_pwm_value = halfPercentage(currentStatus.vvt1Duty, vvt_pwm_max_count);
          BIT_SET(currentStatus.status4, BIT_STATUS4_VVT1_ERROR);
        }
        //Check that we're not already at the angle we want to be
        else if((configPage6.vvtCLUseHold > 0) && (currentStatus.vvt1TargetAngle == currentStatus.vvt1Angle) )
        {
          currentStatus.vvt1Duty = configPage10.vvtCLholdDuty;
          vvt1_pwm_value = halfPercentage(currentStatus.vvt1Duty, vvt_pwm_max_count);
          vvtPID.Initialize();
          BIT_CLEAR(currentStatus.status4, BIT_STATUS4_VVT1_ERROR);
        }
        else
        {
          //This is dumb, but need to convert the current angle into a long pointer.
          vvt_pid_target_angle = (unsigned long)currentStatus.vvt1TargetAngle;
          vvt_pid_current_angle = (long)currentStatus.vvt1Angle;

          //If not already at target angle, calculate new value from PID
          bool PID_compute = vvtPID.Compute(true);
          //vvtPID.Compute2(currentStatus.vvt1TargetAngle, currentStatus.vvt1Angle, false);
          //vvt_pwm_target_value = percentage(40, vvt_pwm_max_count);
          //if (currentStatus.vvt1Angle > currentStatus.vvt1TargetAngle) { vvt_pwm_target_value = 0; }
          if(PID_compute == true) { vvt1_pwm_value = halfPercentage(currentStatus.vvt1Duty, vvt_pwm_max_count); }
          BIT_CLEAR(currentStatus.status4, BIT_STATUS4_VVT1_ERROR);
        }

        if (configPage10.vvt2Enabled == 1) // same for VVT2 if it's enabled
        {
          if(configPage6.vvtLoadSource == VVT_LOAD_TPS) { currentStatus.vvt2TargetAngle = get3DTableValue(&vvt2Table, (currentStatus.TPS * 2), currentStatus.RPM); }
          else { currentStatus.vvt2TargetAngle = get3DTableValue(&vvt2Table, currentStatus.MAP, currentStatus.RPM); }

          if( vvtCounter == 30) { vvt2PID.SetTunings(configPage10.vvtCLKP, configPage10.vvtCLKI, configPage10.vvtCLKD);  //This only needs to be run very infrequently, once every 32 calls to vvtControl(). This is approx. once per second
          vvt2PID.SetControllerDirection(configPage4.vvt2PWMdir); }

          // safety check that the cam angles are ok. The engine will be totally undriveable if the cam sensor is faulty and giving wrong cam angles, so if that happens, default to 0 duty.
          // This also prevents using zero or negative current angle values for PID adjustment, because those don't work in integer PID.
          if ( currentStatus.vvt2Angle <= configPage10.vvtCLMinAng || currentStatus.vvt2Angle > configPage10.vvtCLMaxAng )
          {
            currentStatus.vvt2Duty = 0;
            vvt2_pwm_value = halfPercentage(currentStatus.vvt2Duty, vvt_pwm_max_count);
            BIT_SET(currentStatus.status4, BIT_STATUS4_VVT2_ERROR);
          }
          //Check that we're not already at the angle we want to be
          else if((configPage6.vvtCLUseHold > 0) && (currentStatus.vvt2TargetAngle == currentStatus.vvt2Angle) )
          {
            currentStatus.vvt2Duty = configPage10.vvtCLholdDuty;
            vvt2_pwm_value = halfPercentage(currentStatus.vvt2Duty, vvt_pwm_max_count);
            vvt2PID.Initialize();
            BIT_CLEAR(currentStatus.status4, BIT_STATUS4_VVT2_ERROR);
          }
          else
          {
            //This is dumb, but need to convert the current angle into a long pointer.
            vvt2_pid_target_angle = (unsigned long)currentStatus.vvt2TargetAngle;
            vvt2_pid_current_angle = (long)currentStatus.vvt2Angle;
            //If not already at target angle, calculate new value from PID
            bool PID_compute = vvt2PID.Compute(true);
            if(PID_compute == true) { vvt2_pwm_value = halfPercentage(currentStatus.vvt2Duty, vvt_pwm_max_count); }
            BIT_CLEAR(currentStatus.status4, BIT_STATUS4_VVT2_ERROR);
          }
        }
        vvtCounter++;
      }

      //Set the PWM state based on the above lookups
      if( (currentStatus.vvt1Duty == 0) && (currentStatus.vvt2Duty == 0) )
      {
        //Make sure solenoid is off (0% duty)
        VVT1_PIN_OFF();
        VVT2_PIN_OFF();
        vvt1_pwm_state = false;
        vvt1_max_pwm = false;
        vvt2_pwm_state = false;
        vvt2_max_pwm = false;
        DISABLE_VVT_TIMER();
      }
      else if( (currentStatus.vvt1Duty >= 200) && (currentStatus.vvt2Duty >= 200) )
      {
        //Make sure solenoid is on (100% duty)
        VVT1_PIN_ON();
        VVT2_PIN_ON();
        vvt1_pwm_state = true;
        vvt1_max_pwm = true;
        vvt2_pwm_state = true;
        vvt2_max_pwm = true;
        DISABLE_VVT_TIMER();
      }
      else
      {
        //Duty cycle is between 0 and 100. Make sure the timer is enabled
        ENABLE_VVT_TIMER();
        if(currentStatus.vvt1Duty < 200) { vvt1_max_pwm = false; }
        if(currentStatus.vvt2Duty < 200) { vvt2_max_pwm = false; }
      }
 
    }
  }
  else 
  { 
    // Disable timer channel
    DISABLE_VVT_TIMER(); 
    currentStatus.vvt1Duty = 0;
    vvt1_pwm_value = 0;
    currentStatus.vvt2Duty = 0;
    vvt2_pwm_value = 0;
    vvt1_pwm_state = false;
    vvt1_max_pwm = false;
    vvt2_pwm_state = false;
    vvt2_max_pwm = false;
    vvtTimeHold=false;
  } 
}

void nitrousControl()
{
  bool nitrousOn = false; //This tracks whether the control gets turned on at any point. 
  if(configPage10.n2o_enable > 0)
  {
    bool isArmed = READ_N2O_ARM_PIN();
    if (configPage10.n2o_pin_polarity == 1) { isArmed = !isArmed; } //If nitrous is active when pin is low, flip the reading (n2o_pin_polarity = 0 = active when High)

    //Perform the main checks to see if nitrous is ready
    if( (isArmed == true) && (currentStatus.coolant > (configPage10.n2o_minCLT - CALIBRATION_TEMPERATURE_OFFSET)) && (currentStatus.TPS > configPage10.n2o_minTPS) && (currentStatus.O2 < configPage10.n2o_maxAFR) && (currentStatus.MAP < (uint16_t)(configPage10.n2o_maxMAP * 2)) )
    {
      //Config page values are divided by 100 to fit within a byte. Multiply them back out to real values. 
      uint16_t realStage1MinRPM = (uint16_t)configPage10.n2o_stage1_minRPM * 100;
      uint16_t realStage1MaxRPM = (uint16_t)configPage10.n2o_stage1_maxRPM * 100;
      uint16_t realStage2MinRPM = (uint16_t)configPage10.n2o_stage2_minRPM * 100;
      uint16_t realStage2MaxRPM = (uint16_t)configPage10.n2o_stage2_maxRPM * 100;

      //The nitrous state is set to 0 and then the subsequent stages are added
      // OFF    = 0
      // STAGE1 = 1
      // STAGE2 = 2
      // BOTH   = 3 (ie STAGE1 + STAGE2 = BOTH)
      currentStatus.nitrous_status = NITROUS_OFF; //Reset the current state
      if( (currentStatus.RPM > realStage1MinRPM) && (currentStatus.RPM < realStage1MaxRPM) )
      {
        currentStatus.nitrous_status += NITROUS_STAGE1;
        BIT_SET(currentStatus.status3, BIT_STATUS3_NITROUS);
        N2O_STAGE1_PIN_HIGH();
        nitrousOn = true;
      }
      if(configPage10.n2o_enable == NITROUS_STAGE2) //This is really just a sanity check
      {
        if( (currentStatus.RPM > realStage2MinRPM) && (currentStatus.RPM < realStage2MaxRPM) )
        {
          currentStatus.nitrous_status += NITROUS_STAGE2;
          BIT_SET(currentStatus.status3, BIT_STATUS3_NITROUS);
          N2O_STAGE2_PIN_HIGH();
          nitrousOn = true;
        }
      }
    }
  }

  if (nitrousOn == false)
  {
    currentStatus.nitrous_status = NITROUS_OFF;
    BIT_CLEAR(currentStatus.status3, BIT_STATUS3_NITROUS);

    if(configPage10.n2o_enable > 0)
    {
      N2O_STAGE1_PIN_LOW();
      N2O_STAGE2_PIN_LOW();
    }
  }
}

// Water methanol injection control
void wmiControl()
{
  int wmiPW = 0;
  
  // wmi can only work when vvt is disabled 
  if( (configPage6.vvtEnabled == 0) && (configPage10.wmiEnabled >= 1) )
  {
    if( WMI_TANK_IS_EMPTY() )
    {
    BIT_CLEAR(currentStatus.status4, BIT_STATUS4_WMI_EMPTY);
      if( (currentStatus.TPS >= configPage10.wmiTPS) && (currentStatus.RPMdiv100 >= configPage10.wmiRPM) && ( (currentStatus.MAP / 2) >= configPage10.wmiMAP) && ( (currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET) >= configPage10.wmiIAT) )
      {
        switch(configPage10.wmiMode)
        {
        case WMI_MODE_SIMPLE:
          // Simple mode - Output is turned on when preset boost level is reached
          wmiPW = 100;
          break;
        case WMI_MODE_PROPORTIONAL:
          // Proportional Mode - Output PWM is proportionally controlled between two MAP values - MAP Value 1 = PWM:0% / MAP Value 2 = PWM:100%
          wmiPW = map(currentStatus.MAP/2, configPage10.wmiMAP, configPage10.wmiMAP2, 0, 100);
          break;
        case WMI_MODE_OPENLOOP:
          //  Mapped open loop - Output PWM follows 2D map value (RPM vs MAP) Cell value contains desired PWM% [range 0-100%]
          wmiPW = get3DTableValue(&wmiTable, currentStatus.MAP, currentStatus.RPM);
          break;
        case WMI_MODE_CLOSEDLOOP:
          // Mapped closed loop - Output PWM follows injector duty cycle with 2D correction map applied (RPM vs MAP). Cell value contains correction value% [nom 100%] 
          wmiPW = max(0, ((int)currentStatus.PW1 + configPage10.wmiOffset)) * get3DTableValue(&wmiTable, currentStatus.MAP, currentStatus.RPM) / 100;
          break;
        default:
          // Wrong mode
          wmiPW = 0;
          break;
        }
        if (wmiPW > 100) { wmiPW = 100; } //without this the duty can get beyond 100%
      }
    }
    else { BIT_SET(currentStatus.status4, BIT_STATUS4_WMI_EMPTY); }

    currentStatus.wmiPW = wmiPW;
    vvt1_pwm_value = percentage(currentStatus.wmiPW, vvt_pwm_max_count);

    if(wmiPW == 0)
    {
      // Make sure water pump is off
      VVT1_PIN_LOW();
      DISABLE_VVT_TIMER();
      digitalWrite(pinWMIEnabled, LOW);
    }
    else
    {
      digitalWrite(pinWMIEnabled, HIGH);
      if (wmiPW >= 100)
      {
        // Make sure water pump is on (100% duty)
        VVT1_PIN_HIGH();
        DISABLE_VVT_TIMER();
      }
      else
      {
        ENABLE_VVT_TIMER();
      }
    }
  }
}

void boostDisable()
{
  boostPID.Initialize(); //This resets the ITerm value to prevent rubber banding
  currentStatus.boostDuty = 0;
  DISABLE_BOOST_TIMER(); //Turn off timer
  BOOST_PIN_LOW(); //Make sure solenoid is off (0% duty)
}

//The interrupt to control the Boost PWM
#if defined(CORE_AVR)
  ISR(TIMER1_COMPA_vect)
#else
  void boostInterrupt() //Most ARM chips can simply call a function
#endif
{
  if (boost_pwm_state == true)
  {
    BOOST_PIN_LOW();  // Switch pin to low
    SET_COMPARE(BOOST_TIMER_COMPARE, BOOST_TIMER_COUNTER + (boost_pwm_max_count - boost_pwm_cur_value) );
    boost_pwm_state = false;
  }
  else
  {
    BOOST_PIN_HIGH();  // Switch pin high
    SET_COMPARE(BOOST_TIMER_COMPARE, BOOST_TIMER_COUNTER + boost_pwm_target_value);
    boost_pwm_cur_value = boost_pwm_target_value;
    boost_pwm_state = true;
  }
}

//The interrupt to control the VVT PWM
#if defined(CORE_AVR)
  ISR(TIMER1_COMPB_vect)
#else
  void vvtInterrupt() //Most ARM chips can simply call a function
#endif
{
  if ( ((vvt1_pwm_state == false) || (vvt1_max_pwm == true)) && ((vvt2_pwm_state == false) || (vvt2_max_pwm == true)) )
  {
    if( (vvt1_pwm_value > 0) && (vvt1_max_pwm == false) ) //Don't toggle if at 0%
    {
      VVT1_PIN_ON();
      vvt1_pwm_state = true;
    }
    if( (vvt2_pwm_value > 0) && (vvt2_max_pwm == false) ) //Don't toggle if at 0%
    {
      VVT2_PIN_ON();
      vvt2_pwm_state = true;
    }

    if( (vvt1_pwm_state == true) && ((vvt1_pwm_value <= vvt2_pwm_value) || (vvt2_pwm_state == false)) )
    {
      SET_COMPARE(VVT_TIMER_COMPARE, VVT_TIMER_COUNTER + vvt1_pwm_value);
      vvt1_pwm_cur_value = vvt1_pwm_value;
      vvt2_pwm_cur_value = vvt2_pwm_value;
      if (vvt1_pwm_value == vvt2_pwm_value) { nextVVT = 2; } //Next event is for both PWM
      else { nextVVT = 0; } //Next event is for PWM0
    }
    else if( vvt2_pwm_state == true )
    {
      SET_COMPARE(VVT_TIMER_COMPARE, VVT_TIMER_COUNTER + vvt2_pwm_value);
      vvt1_pwm_cur_value = vvt1_pwm_value;
      vvt2_pwm_cur_value = vvt2_pwm_value;
      nextVVT = 1; //Next event is for PWM1
    }
    else { SET_COMPARE(VVT_TIMER_COMPARE, VVT_TIMER_COUNTER + vvt_pwm_max_count); } //Shouldn't ever get here
  }
  else
  {
    if(nextVVT == 0)
    {
      if(vvt1_pwm_value < (long)vvt_pwm_max_count) //Don't toggle if at 100%
      {
        VVT1_PIN_OFF();
        vvt1_pwm_state = false;
        vvt1_max_pwm = false;
      }
      else { vvt1_max_pwm = true; }
      nextVVT = 1; //Next event is for PWM1
      if(vvt2_pwm_state == true){ SET_COMPARE(VVT_TIMER_COMPARE, VVT_TIMER_COUNTER + (vvt2_pwm_cur_value - vvt1_pwm_cur_value) ); }
      else
      { 
        SET_COMPARE(VVT_TIMER_COMPARE, VVT_TIMER_COUNTER + (vvt_pwm_max_count - vvt1_pwm_cur_value) );
        nextVVT = 2; //Next event is for both PWM
      }
    }
    else if (nextVVT == 1)
    {
      if(vvt2_pwm_value < (long)vvt_pwm_max_count) //Don't toggle if at 100%
      {
        VVT2_PIN_OFF();
        vvt2_pwm_state = false;
        vvt2_max_pwm = false;
      }
      else { vvt2_max_pwm = true; }
      nextVVT = 0; //Next event is for PWM0
      if(vvt1_pwm_state == true) { SET_COMPARE(VVT_TIMER_COMPARE, VVT_TIMER_COUNTER + (vvt1_pwm_cur_value - vvt2_pwm_cur_value) ); }
      else
      { 
        SET_COMPARE(VVT_TIMER_COMPARE, VVT_TIMER_COUNTER + (vvt_pwm_max_count - vvt2_pwm_cur_value) );
        nextVVT = 2; //Next event is for both PWM
      }
    }
    else
    {
      if(vvt1_pwm_value < (long)vvt_pwm_max_count) //Don't toggle if at 100%
      {
        VVT1_PIN_OFF();
        vvt1_pwm_state = false;
        vvt1_max_pwm = false;
        SET_COMPARE(VVT_TIMER_COMPARE, VVT_TIMER_COUNTER + (vvt_pwm_max_count - vvt1_pwm_cur_value) );
      }
      else { vvt1_max_pwm = true; }
      if(vvt2_pwm_value < (long)vvt_pwm_max_count) //Don't toggle if at 100%
      {
        VVT2_PIN_OFF();
        vvt2_pwm_state = false;
        vvt2_max_pwm = false;
        SET_COMPARE(VVT_TIMER_COMPARE, VVT_TIMER_COUNTER + (vvt_pwm_max_count - vvt2_pwm_cur_value) );
      }
      else { vvt2_max_pwm = true; }
    }
  }
}

#if defined(PWM_FAN_AVAILABLE)
//The interrupt to control the FAN PWM. Mega2560 doesn't have enough timers, so this is only for the ARM chip ones
  void fanInterrupt()
{
  if (fan_pwm_state == true)
  {
    FAN_OFF();
    FAN_TIMER_COMPARE = FAN_TIMER_COUNTER + (fan_pwm_max_count - fan_pwm_cur_value);
    fan_pwm_state = false;
  }
  else
  {
    FAN_ON();
    FAN_TIMER_COMPARE = FAN_TIMER_COUNTER + fan_pwm_value;
    fan_pwm_cur_value = fan_pwm_value;
    fan_pwm_state = true;
  }
}
#endif
