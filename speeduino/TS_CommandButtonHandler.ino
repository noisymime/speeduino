
/** \file
 * Header file for the TunerStudio command handler
 * The command handler manages all the inputs FROM TS which are issued when a command button is clicked by the user
 */

#include "TS_CommandButtonHandler.h"
#include "globals.h"
#include "utilities.h"
#include "scheduledIO.h"
#include "sensors.h"
#include "storage.h"
#include "SD_logger.h"
#ifdef USE_MC33810
  #include "acc_mc33810.h"
#endif

/**
 * @brief 
 * 
 * @param buttonCommand The command number of the button that was clicked. See TS_CommendButtonHandler.h for a list of button IDs
 */
void TS_CommandButtonsHandler(uint16_t buttonCommand)
{
  switch (buttonCommand)
  {
    case TS_CMD_TEST_DSBL: // cmd is stop
      BIT_CLEAR(currentStatus.testOutputs, 1);
      endCoil1Charge();
      endCoil2Charge();
      endCoil3Charge();
      endCoil4Charge();
      #if IGN_CHANNELS >= 5
      endCoil5Charge();
      #endif
      #if IGN_CHANNELS >= 6
      endCoil6Charge();
      #endif
      #if IGN_CHANNELS >= 7
      endCoil7Charge();
      #endif
      #if IGN_CHANNELS >= 8
      endCoil8Charge();
      #endif


      closeInjector1();
      closeInjector2();
      closeInjector3();
      closeInjector4();
      #if INJ_CHANNELS >= 5
      closeInjector5();
      #endif
      #if INJ_CHANNELS >= 6
      closeInjector6();
      #endif
      #if INJ_CHANNELS >= 7
      closeInjector7();
      #endif
      #if INJ_CHANNELS >= 8
      closeInjector8();
      #endif

      HWTest_INJ_50pc = 0;
      HWTest_IGN_50pc = 0;
      break;

    case TS_CMD_TEST_ENBL: // cmd is enable
      // currentStatus.testactive = 1;
      BIT_SET(currentStatus.testOutputs, 1);
      break;

    case TS_CMD_INJ1_ON: // cmd group is for injector1 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector1(); }
      break;

    case TS_CMD_INJ1_OFF: // cmd group is for injector1 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector1(); BIT_CLEAR(HWTest_INJ_50pc, INJ1_CMD_BIT); }
      break;

    case TS_CMD_INJ1_50PC: // cmd group is for injector1 50% dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_INJ_50pc, INJ1_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_50pc, INJ1_CMD_BIT)) { closeInjector1(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ2_ON: // cmd group is for injector2 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector2(); }
      break;

    case TS_CMD_INJ2_OFF: // cmd group is for injector2 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector2(); BIT_CLEAR(HWTest_INJ_50pc, INJ2_CMD_BIT); }
      break;

    case TS_CMD_INJ2_50PC: // cmd group is for injector2 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_INJ_50pc, INJ2_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_50pc, INJ2_CMD_BIT)) { closeInjector2(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ3_ON: // cmd group is for injector3 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector3(); }
      break;

    case TS_CMD_INJ3_OFF: // cmd group is for injector3 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector3(); BIT_CLEAR(HWTest_INJ_50pc, INJ3_CMD_BIT); }
      break;

    case TS_CMD_INJ3_50PC: // cmd group is for injector3 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_INJ_50pc, INJ3_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_50pc, INJ3_CMD_BIT)) { closeInjector3(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ4_ON: // cmd group is for injector4 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector4(); }
      break;

    case TS_CMD_INJ4_OFF: // cmd group is for injector4 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector4(); BIT_CLEAR(HWTest_INJ_50pc, INJ4_CMD_BIT); }
      break;

    case TS_CMD_INJ4_50PC: // cmd group is for injector4 50% dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_INJ_50pc, INJ4_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_50pc, INJ4_CMD_BIT)) { closeInjector4(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ5_ON: // cmd group is for injector5 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector5(); }
      break;

    case TS_CMD_INJ5_OFF: // cmd group is for injector5 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector5(); BIT_CLEAR(HWTest_INJ_50pc, INJ5_CMD_BIT); }
      break;

    case TS_CMD_INJ5_50PC: // cmd group is for injector5 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_INJ_50pc, INJ5_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_50pc, INJ5_CMD_BIT)) { closeInjector5(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ6_ON: // cmd group is for injector6 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector6(); }
      break;

    case TS_CMD_INJ6_OFF: // cmd group is for injector6 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector6(); BIT_CLEAR(HWTest_INJ_50pc, INJ6_CMD_BIT); }
      break;

    case TS_CMD_INJ6_50PC: // cmd group is for injector6 50% dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_INJ_50pc, INJ6_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_50pc, INJ6_CMD_BIT)) { closeInjector6(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ7_ON: // cmd group is for injector7 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector7(); }
      break;

    case TS_CMD_INJ7_OFF: // cmd group is for injector7 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector7(); BIT_CLEAR(HWTest_INJ_50pc, INJ7_CMD_BIT); }
      break;

    case TS_CMD_INJ7_50PC: // cmd group is for injector7 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_INJ_50pc, INJ7_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_50pc, INJ7_CMD_BIT)) { closeInjector7(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_INJ8_ON: // cmd group is for injector8 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ openInjector8(); }
      break;

    case TS_CMD_INJ8_OFF: // cmd group is for injector8 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ closeInjector8(); BIT_CLEAR(HWTest_INJ_50pc, INJ8_CMD_BIT); }
      break;

    case TS_CMD_INJ8_50PC: // cmd group is for injector8 50% dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_INJ_50pc, INJ8_CMD_BIT); }
      if(!BIT_CHECK(HWTest_INJ_50pc, INJ8_CMD_BIT)) { closeInjector8(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN1_ON: // cmd group is for spark1 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ){ beginCoil1Charge(); }
      break;

    case TS_CMD_IGN1_OFF: // cmd group is for spark1 off actions
        if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil1Charge(); BIT_CLEAR(HWTest_IGN_50pc, IGN1_CMD_BIT); }
      break;

    case TS_CMD_IGN1_50PC: // cmd group is for spark1 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_IGN_50pc, IGN1_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_50pc, IGN1_CMD_BIT)) { endCoil1Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN2_ON: // cmd group is for spark2 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { beginCoil2Charge(); }
      break;

    case TS_CMD_IGN2_OFF: // cmd group is for spark2 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil2Charge(); BIT_CLEAR(HWTest_IGN_50pc, IGN2_CMD_BIT); }
      break;

    case TS_CMD_IGN2_50PC: // cmd group is for spark2 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_IGN_50pc, IGN2_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_50pc, IGN2_CMD_BIT)) { endCoil2Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN3_ON: // cmd group is for spark3 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { beginCoil3Charge(); }
      break;

    case TS_CMD_IGN3_OFF: // cmd group is for spark3 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil3Charge(); BIT_CLEAR(HWTest_IGN_50pc, IGN3_CMD_BIT); }
      break;

    case TS_CMD_IGN3_50PC: // cmd group is for spark3 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_IGN_50pc, IGN3_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_50pc, IGN3_CMD_BIT)) { endCoil3Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN4_ON: // cmd group is for spark4 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { beginCoil4Charge(); }
      break;

    case TS_CMD_IGN4_OFF: // cmd group is for spark4 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil4Charge(); BIT_CLEAR(HWTest_IGN_50pc, IGN4_CMD_BIT); }
      break;

    case TS_CMD_IGN4_50PC: // cmd group is for spark4 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_IGN_50pc, IGN4_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_50pc, IGN4_CMD_BIT)) { endCoil4Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN5_ON: // cmd group is for spark5 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { beginCoil5Charge(); }
      break;

    case TS_CMD_IGN5_OFF: // cmd group is for spark5 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil5Charge(); BIT_CLEAR(HWTest_IGN_50pc, IGN5_CMD_BIT); }
      break;

    case TS_CMD_IGN5_50PC: // cmd group is for spark4 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_IGN_50pc, IGN5_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_50pc, IGN5_CMD_BIT)) { endCoil5Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN6_ON: // cmd group is for spark6 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { beginCoil6Charge(); }
      break;

    case TS_CMD_IGN6_OFF: // cmd group is for spark6 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil6Charge(); BIT_CLEAR(HWTest_IGN_50pc, IGN6_CMD_BIT); }
      break;

    case TS_CMD_IGN6_50PC: // cmd group is for spark6 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_IGN_50pc, IGN6_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_50pc, IGN6_CMD_BIT)) { endCoil6Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN7_ON: // cmd group is for spark7 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { beginCoil7Charge(); }
      break;

    case TS_CMD_IGN7_OFF: // cmd group is for spark7 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil7Charge(); BIT_CLEAR(HWTest_IGN_50pc, IGN7_CMD_BIT); }
      break;

    case TS_CMD_IGN7_50PC: // cmd group is for spark7 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_IGN_50pc, IGN7_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_50pc, IGN7_CMD_BIT)) { endCoil7Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    case TS_CMD_IGN8_ON: // cmd group is for spark8 on actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { beginCoil8Charge(); }
      break;

    case TS_CMD_IGN8_OFF: // cmd group is for spark8 off actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { endCoil8Charge(); BIT_CLEAR(HWTest_IGN_50pc, IGN8_CMD_BIT); }
      break;

    case TS_CMD_IGN8_50PC: // cmd group is for spark8 50%dc actions
      if( BIT_CHECK(currentStatus.testOutputs, 1) ) { BIT_TOGGLE(HWTest_IGN_50pc, IGN8_CMD_BIT); }
      if(!BIT_CHECK(HWTest_IGN_50pc, IGN8_CMD_BIT)) { endCoil8Charge(); } //Ensure this output is turned off (Otherwise the output may stay on permanently)
      break;

    //VSS Calibration routines
    case TS_CMD_VSS_60KMH:
      {
        //Calibrate the actual pulses per distance
        uint32_t calibrationGap = vssGetPulseGap(0);
        if( calibrationGap > 0 )
        {
          configPage2.vssPulsesPerKm = 60000000UL / calibrationGap;
          writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
          BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
        }
      }
      break;

    //Calculate the RPM to speed ratio for each gear
    case TS_CMD_VSS_RATIO1:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio1 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    case TS_CMD_VSS_RATIO2:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio2 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    case TS_CMD_VSS_RATIO3:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio3 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    case TS_CMD_VSS_RATIO4: 
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio4 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    case TS_CMD_VSS_RATIO5:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio5 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    case TS_CMD_VSS_RATIO6:
      if(currentStatus.vss > 0)
      {
        configPage2.vssRatio6 = (currentStatus.vss * 10000UL) / currentStatus.RPM;
        writeConfig(1); // Need to manually save the new config value as it will not trigger a burn in tunerStudio due to use of ControllerPriority
        BIT_SET(currentStatus.status3, BIT_STATUS3_VSS_REFRESH); //Set the flag to trigger the UI reset
      }
      break;

    //STM32 Commands
    case TS_CMD_STM32_REBOOT: //
      doSystemReset();
      break;

    case TS_CMD_STM32_BOOTLOADER: //
      jumpToBootloader();
      break;

#ifdef SD_LOGGING
    case TS_CMD_SD_FORMAT: //Format SD card
      formatExFat();
      break;
#endif

    default:
      break;
  }
}
