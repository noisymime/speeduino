/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
/** @file
 * Process Incoming and outgoing serial communications.
 */
#include "globals.h"
#include "comms.h"
#include "cancomms.h"
#include "storage.h"
#include "maths.h"
#include "utilities.h"
#include "decoders.h"
#include "TS_CommandButtonHandler.h"
#include "errors.h"
#include "pages.h"
#include "page_crc.h"
#include "logger.h"
#ifdef RTC_ENABLED
  #include "rtc_common.h"
#endif

byte currentPage = 1;//Not the same as the speeduino config page numbers
bool isMap = true; /**< Whether or not the currentPage contains only a 3D map that would require translation */
unsigned long requestCount = 0; /**< The number of times the A command has been issued. This is used to track whether a reset has recently been performed on the controller */
byte currentCommand; /**< The serial command that is currently being processed. This is only useful when cmdPending=True */
bool cmdPending = false; /**< Whether or not a serial request has only been partially received. This occurs when a command character has been received in the serial buffer, but not all of its arguments have yet been received. If true, the active command will be stored in the currentCommand variable */
bool chunkPending = false; /**< Whether or not the current chunk write is complete or not */
uint16_t chunkComplete = 0; /**< The number of bytes in a chunk write that have been written so far */
uint16_t chunkSize = 0; /**< The complete size of the requested chunk write */
int valueOffset; /**< The memory offset within a given page for a value to be read from or written to. Note that we cannot use 'offset' as a variable name, it is a reserved word for several teensy libraries */
byte tsCanId = 0;     // current tscanid requested
byte inProgressOffset;
byte inProgressLength;
uint32_t inProgressCompositeTime;
bool serialInProgress = false;
bool toothLogSendInProgress = false;
bool compositeLogSendInProgress = false;
bool legacySerial = false;

/** Processes the incoming data on the serial buffer based on the command sent.
Can be either data for a new command or a continuation of data for command that is already in progress:
- cmdPending = If a command has started but is wairing on further data to complete
- chunkPending = Specifically for the new receive value method where TS will send a known number of contiguous bytes to be written to a table

Comands are single byte (letter symbol) commands.
*/
void command()
{
  if ( (cmdPending == false) && (legacySerial == false) ) { currentCommand = Serial.read(); }

  switch (currentCommand)
  {

    case 'a':
      cmdPending = true;

      if (Serial.available() >= 2)
      {
        Serial.read(); //Ignore the first value, it's always 0
        Serial.read(); //Ignore the second value, it's always 6
        sendValuesLegacy();
        cmdPending = false;
      }
      break;

    case 'A': // send x bytes of realtime values
      sendValues(0, LOG_ENTRY_SIZE, 0x31, 0);   //send values to serial0
      break;


    case 'B': // Burn current values to eeprom
      writeAllConfig();
      break;

    case 'b': // New EEPROM burn command to only burn a single page at a time
      cmdPending = true;

      if (Serial.available() >= 2)
      {
        Serial.read(); //Ignore the first table value, it's always 0
        writeConfig(Serial.read());
        cmdPending = false;
      }
      break;

    case 'C': // test communications. This is used by Tunerstudio to see whether there is an ECU on a given serial port
      testComm();
      break;

    case 'c': //Send the current loops/sec value
      Serial.write(lowByte(currentStatus.loopsPerSecond));
      Serial.write(highByte(currentStatus.loopsPerSecond));
      break;

    case 'd': // Send a CRC32 hash of a given page
      cmdPending = true;

      if (Serial.available() >= 2)
      {
        Serial.read(); //Ignore the first byte value, it's always 0
        uint32_t CRC32_val = calculatePageCRC32( Serial.read() );
        
        //Split the 4 bytes of the CRC32 value into individual bytes and send
        Serial.write( ((CRC32_val >> 24) & 255) );
        Serial.write( ((CRC32_val >> 16) & 255) );
        Serial.write( ((CRC32_val >> 8) & 255) );
        Serial.write( (CRC32_val & 255) );
        
        cmdPending = false;
      }
      break;

    case 'E': // receive command button commands
      cmdPending = true;

      if(Serial.available() >= 2)
      {
        byte cmdGroup = Serial.read();
        byte cmdValue = Serial.read();
        uint16_t cmdCombined = word(cmdGroup, cmdValue);

        if ( ((cmdCombined >= TS_CMD_INJ1_ON) && (cmdCombined <= TS_CMD_IGN8_50PC)) || (cmdCombined == TS_CMD_TEST_ENBL) || (cmdCombined == TS_CMD_TEST_DSBL) )
        {
          //Hardware test buttons
          if (currentStatus.RPM == 0) { TS_CommandButtonsHandler(cmdCombined); }
          cmdPending = false;
        }
        else if( (cmdCombined >= TS_CMD_VSS_60KMH) && (cmdCombined <= TS_CMD_VSS_RATIO6) )
        {
          //VSS Calibration commands
          TS_CommandButtonsHandler(cmdCombined);
          cmdPending = false;
        }
        else if( (cmdCombined >= TS_CMD_STM32_REBOOT) && (cmdCombined <= TS_CMD_STM32_BOOTLOADER) )
        {
          //STM32 DFU mode button
          TS_CommandButtonsHandler(cmdCombined);
          cmdPending = false;
        }
      }
      break;

    case 'F': // send serial protocol version
      Serial.print(F("001"));
      break;

    case 'H': //Start the tooth logger
      currentStatus.toothLogEnabled = true;
      currentStatus.compositeLogEnabled = false; //Safety first (Should never be required)
      BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
      toothHistoryIndex = 0;

      //Disconnect the standard interrupt and add the logger version
      detachInterrupt( digitalPinToInterrupt(pinTrigger) );
      attachInterrupt( digitalPinToInterrupt(pinTrigger), loggerPrimaryISR, CHANGE );

      detachInterrupt( digitalPinToInterrupt(pinTrigger2) );
      attachInterrupt( digitalPinToInterrupt(pinTrigger2), loggerSecondaryISR, CHANGE );

      Serial.write(1); //TS needs an acknowledgement that this was received. I don't know if this is the correct response, but it seems to work
      break;

    case 'h': //Stop the tooth logger
      currentStatus.toothLogEnabled = false;

      //Disconnect the logger interrupts and attach the normal ones
      detachInterrupt( digitalPinToInterrupt(pinTrigger) );
      attachInterrupt( digitalPinToInterrupt(pinTrigger), triggerHandler, primaryTriggerEdge );

      detachInterrupt( digitalPinToInterrupt(pinTrigger2) );
      attachInterrupt( digitalPinToInterrupt(pinTrigger2), triggerSecondaryHandler, secondaryTriggerEdge );
      break;

    case 'J': //Start the composite logger
      currentStatus.compositeLogEnabled = true;
      currentStatus.toothLogEnabled = false; //Safety first (Should never be required)
      BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
      toothHistoryIndex = 0;

      //Disconnect the standard interrupt and add the logger version
      detachInterrupt( digitalPinToInterrupt(pinTrigger) );
      attachInterrupt( digitalPinToInterrupt(pinTrigger), loggerPrimaryISR, CHANGE );

      detachInterrupt( digitalPinToInterrupt(pinTrigger2) );
      attachInterrupt( digitalPinToInterrupt(pinTrigger2), loggerSecondaryISR, CHANGE );

      Serial.write(1); //TS needs an acknowledgement that this was received. I don't know if this is the correct response, but it seems to work
      break;

    case 'j': //Stop the composite logger
      currentStatus.compositeLogEnabled = false;

      //Disconnect the logger interrupts and attach the normal ones
      detachInterrupt( digitalPinToInterrupt(pinTrigger) );
      attachInterrupt( digitalPinToInterrupt(pinTrigger), triggerHandler, primaryTriggerEdge );

      detachInterrupt( digitalPinToInterrupt(pinTrigger2) );
      attachInterrupt( digitalPinToInterrupt(pinTrigger2), triggerSecondaryHandler, secondaryTriggerEdge );
      break;

    case 'L': // List the contents of current page in human readable form
      #ifndef SMALL_FLASH_MODE
      sendPageASCII();
      #endif
      break;

    case 'm': //Send the current free memory
      currentStatus.freeRAM = freeRam();
      Serial.write(lowByte(currentStatus.freeRAM));
      Serial.write(highByte(currentStatus.freeRAM));
      break;

    case 'N': // Displays a new line.  Like pushing enter in a text editor
      Serial.println();
      break;

    case 'P': // set the current page
      //This is a legacy function and is no longer used by TunerStudio. It is maintained for compatibility with other systems
      //A 2nd byte of data is required after the 'P' specifying the new page number.
      cmdPending = true;

      if (Serial.available() > 0)
      {
        currentPage = Serial.read();
        //This converts the ascii number char into binary. Note that this will break everyything if there are ever more than 48 pages (48 = asci code for '0')
        if ((currentPage >= '0') && (currentPage <= '9')) // 0 - 9
        {
          currentPage -= 48;
        }
        else if ((currentPage >= 'a') && (currentPage <= 'f')) // 10 - 15
        {
          currentPage -= 87;
        }
        else if ((currentPage >= 'A') && (currentPage <= 'F'))
        {
          currentPage -= 55;
        }
        
        // Detecting if the current page is a table/map
        if ( (currentPage == veMapPage) || (currentPage == ignMapPage) || (currentPage == afrMapPage) || (currentPage == fuelMap2Page) || (currentPage == ignMap2Page) ) { isMap = true; }
        else { isMap = false; }
        cmdPending = false;
      }
      break;

    /*
    * New method for sending page values
    */
    case 'p':
      cmdPending = true;

      //6 bytes required:
      //2 - Page identifier
      //2 - offset
      //2 - Length
      if(Serial.available() >= 6)
      {
        byte offset1, offset2, length1, length2;
        int length;
        byte tempPage;

        Serial.read(); // First byte of the page identifier can be ignored. It's always 0
        tempPage = Serial.read();
        //currentPage = 1;
        offset1 = Serial.read();
        offset2 = Serial.read();
        valueOffset = word(offset2, offset1);
        length1 = Serial.read();
        length2 = Serial.read();
        length = word(length2, length1);
        for(int i = 0; i < length; i++)
        {
          Serial.write( getPageValue(tempPage, valueOffset + i) );
        }

        cmdPending = false;
      }
      break;

    case 'Q': // send code version
      Serial.print(F("speeduino 202201"));
      break;

    case 'r': //New format for the optimised OutputChannels
      cmdPending = true;
      byte cmd;
      if (Serial.available() >= 6)
      {
        tsCanId = Serial.read(); //Read the $tsCanId
        cmd = Serial.read(); // read the command

        uint16_t offset, length;
        byte tmp;
        tmp = Serial.read();
        offset = word(Serial.read(), tmp);
        tmp = Serial.read();
        length = word(Serial.read(), tmp);


        if(cmd == 0x30) //Send output channels command 0x30 is 48dec
        {
          sendValues(offset, length, cmd, 0);
        }
        else
        {
          //No other r/ commands are supported in legacy mode
        }
        cmdPending = false;
      }
      break;

    case 'S': // send code version
      Serial.print(F("Speeduino 2022.01"));
      currentStatus.secl = 0; //This is required in TS3 due to its stricter timings
      break;

    case 'T': //Send 256 tooth log entries to Tuner Studios tooth logger
      //6 bytes required:
      //2 - Page identifier
      //2 - offset
      //2 - Length
      cmdPending = true;
      if(Serial.available() >= 6)
      {
        Serial.read(); // First byte of the page identifier can be ignored. It's always 0
        Serial.read(); // First byte of the page identifier can be ignored. It's always 0
        Serial.read(); // First byte of the page identifier can be ignored. It's always 0
        Serial.read(); // First byte of the page identifier can be ignored. It's always 0
        Serial.read(); // First byte of the page identifier can be ignored. It's always 0
        Serial.read(); // First byte of the page identifier can be ignored. It's always 0

        if(currentStatus.toothLogEnabled == true) { sendToothLog_old(0); } //Sends tooth log values as ints
        else if (currentStatus.compositeLogEnabled == true) { sendCompositeLog_old(0); }

        cmdPending = false;
      }

      

      break;

    case 't': // receive new Calibration info. Command structure: "t", <tble_idx> <data array>.
      byte tableID;
      //byte canID;

      //The first 2 bytes sent represent the canID and tableID
      while (Serial.available() == 0) { }
      tableID = Serial.read(); //Not currently used for anything

      receiveCalibration(tableID); //Receive new values and store in memory
      writeCalibration(); //Store received values in EEPROM

      break;

    case 'U': //User wants to reset the Arduino (probably for FW update)
      if (resetControl != RESET_CONTROL_DISABLED)
      {
      #ifndef SMALL_FLASH_MODE
        if (!cmdPending) { Serial.println(F("Comms halted. Next byte will reset the Arduino.")); }
      #endif

        while (Serial.available() == 0) { }
        digitalWrite(pinResetControl, LOW);
      }
      else
      {
      #ifndef SMALL_FLASH_MODE
        if (!cmdPending) { Serial.println(F("Reset control is currently disabled.")); }
      #endif
      }
      break;

    case 'V': // send VE table and constants in binary
      sendPage();
      break;

    case 'W': // receive new VE obr constant at 'W'+<offset>+<newbyte>
      cmdPending = true;

      if (isMap)
      {
        if(Serial.available() >= 3) // 1 additional byte is required on the MAP pages which are larger than 255 bytes
        {
          byte offset1, offset2;
          offset1 = Serial.read();
          offset2 = Serial.read();
          valueOffset = word(offset2, offset1);
          setPageValue(currentPage, valueOffset, Serial.read());
          cmdPending = false;
        }
      }
      else
      {
        if(Serial.available() >= 2)
        {
          valueOffset = Serial.read();
          setPageValue(currentPage, valueOffset, Serial.read());
          cmdPending = false;
        }
      }

      break;

    case 'M':
      cmdPending = true;

      if(chunkPending == false)
      {
        //This means it's a new request
        //7 bytes required:
        //2 - Page identifier
        //2 - offset
        //2 - Length
        //1 - 1st New value
        if(Serial.available() >= 7)
        {
          byte offset1, offset2, length1, length2;

          Serial.read(); // First byte of the page identifier can be ignored. It's always 0
          currentPage = Serial.read();
          //currentPage = 1;
          offset1 = Serial.read();
          offset2 = Serial.read();
          valueOffset = word(offset2, offset1);
          length1 = Serial.read();
          length2 = Serial.read();
          chunkSize = word(length2, length1);

          //Regular page data
          chunkPending = true;
          chunkComplete = 0;
        }
      }
      //This CANNOT be an else of the above if statement as chunkPending gets set to true above
      if(chunkPending == true)
      { 
        while( (Serial.available() > 0) && (chunkComplete < chunkSize) )
        {
          setPageValue(currentPage, (valueOffset + chunkComplete), Serial.read());
          chunkComplete++;
        }
        if(chunkComplete >= chunkSize) { cmdPending = false; chunkPending = false; }
      }
      break;

    case 'w':
      //No w commands are supported in legacy mode. This should never be called
      if(Serial.available() >= 7)
      {
        byte offset1, offset2, length1, length2;

        Serial.read(); // First byte of the page identifier can be ignored. It's always 0
        currentPage = Serial.read();
        //currentPage = 1;
        offset1 = Serial.read();
        offset2 = Serial.read();
        valueOffset = word(offset2, offset1);
        length1 = Serial.read();
        length2 = Serial.read();
        chunkSize = word(length2, length1);
      }
      break;

    case 'Z': //Totally non-standard testing function. Will be removed once calibration testing is completed. This function takes 1.5kb of program space! :S
    #ifndef SMALL_FLASH_MODE
      Serial.println(F("Coolant"));
      for (int x = 0; x < 32; x++)
      {
        Serial.print(cltCalibration_bins[x]);
        Serial.print(", ");
        Serial.println(cltCalibration_values[x]);
      }
      Serial.println(F("Inlet temp"));
      for (int x = 0; x < 32; x++)
      {
        Serial.print(iatCalibration_bins[x]);
        Serial.print(", ");
        Serial.println(iatCalibration_values[x]);
      }
      Serial.println(F("O2"));
      for (int x = 0; x < 32; x++)
      {
        Serial.print(o2Calibration_bins[x]);
        Serial.print(", ");
        Serial.println(o2Calibration_values[x]);
      }
      Serial.println(F("WUE"));
      for (int x = 0; x < 10; x++)
      {
        Serial.print(configPage4.wueBins[x]);
        Serial.print(F(", "));
        Serial.println(configPage2.wueValues[x]);
      }
      Serial.flush();
    #endif
      break;

    case 'z': //Send 256 tooth log entries to a terminal emulator
      sendToothLog_old(0); //Sends tooth log values as chars
      break;

    case '`': //Custom 16u2 firmware is making its presence known
      cmdPending = true;

      if (Serial.available() >= 1) {
        configPage4.bootloaderCaps = Serial.read();
        cmdPending = false;
      }
      break;


    case '?':
    #ifndef SMALL_FLASH_MODE
      Serial.println
      (F(
         "\n"
         "===Command Help===\n\n"
         "All commands are single character and are concatenated with their parameters \n"
         "without spaces."
         "Syntax:  <command>+<parameter1>+<parameter2>+<parameterN>\n\n"
         "===List of Commands===\n\n"
         "A - Displays 31 bytes of currentStatus values in binary (live data)\n"
         "B - Burn current map and configPage values to eeprom\n"
         "C - Test COM port.  Used by Tunerstudio to see whether an ECU is on a given serial \n"
         "    port. Returns a binary number.\n"
         "N - Print new line.\n"
         "P - Set current page.  Syntax:  P+<pageNumber>\n"
         "R - Same as A command\n"
         "S - Display signature number\n"
         "Q - Same as S command\n"
         "V - Display map or configPage values in binary\n"
         "W - Set one byte in map or configPage.  Expects binary parameters. \n"
         "    Syntax:  W+<offset>+<newbyte>\n"
         "t - Set calibration values.  Expects binary parameters.  Table index is either 0, \n"
         "    1, or 2.  Syntax:  t+<tble_idx>+<newValue1>+<newValue2>+<newValueN>\n"
         "Z - Display calibration values\n"
         "T - Displays 256 tooth log entries in binary\n"
         "r - Displays 256 tooth log entries\n"
         "U - Prepare for firmware update. The next byte received will cause the Arduino to reset.\n"
         "? - Displays this help page"
       ));
     #endif

      break;

    default:
      Serial.println(F("Err: Unknown cmd"));
      cmdPending = false;
      break;
  }
}

/** Send a status record back to tuning/logging SW.
 * This will "live" information from @ref currentStatus struct.
 * @param offset - Start field number
 * @param packetLength - Length of actual message (after possible ack/confirm headers)
 * @param cmd - ??? - Will be used as some kind of ack on CANSerial
 * @param portNum - Port number (0=Serial, 3=CANSerial)
 * E.g. tuning sw command 'A' (Send all values) will send data from field number 0, LOG_ENTRY_SIZE fields.
 * @return the current values of a fixed group of variables
 */
//void sendValues(int packetlength, byte portNum)
void sendValues(uint16_t offset, uint16_t packetLength, byte cmd, byte portNum)
{  
  if (portNum == 3)
  {
    //CAN serial
    #if defined(USE_SERIAL3)
      if (cmd == 30)
      {
        CANSerial.write("r");         //confirm cmd type
        CANSerial.write(cmd);
      }
      else if (cmd == 31) { CANSerial.write("A"); }        //confirm cmd type
    #else
      UNUSED(cmd);
    #endif
  }
  else
  {
    if(requestCount == 0) { currentStatus.secl = 0; }
    requestCount++;
  }

  currentStatus.spark ^= (-currentStatus.hasSync ^ currentStatus.spark) & (1U << BIT_SPARK_SYNC); //Set the sync bit of the Spark variable to match the hasSync variable

  for(byte x=0; x<packetLength; x++)
  {
    if (portNum == 0) { Serial.write(getTSLogEntry(offset+x)); }
    #if defined(CANSerial_AVAILABLE)
      else if (portNum == 3){ CANSerial.write(getTSLogEntry(offset+x)); }
    #endif

    //Check whether the tx buffer still has space
    if(Serial.availableForWrite() < 1) 
    { 
      //tx buffer is full. Store the current state so it can be resumed later
      inProgressOffset = offset + x + 1;
      inProgressLength = packetLength - x - 1;
      serialInProgress = true;
      return;
    }
    
  }
  serialInProgress = false;
  // Reset any flags that are being used to trigger page refreshes
  BIT_CLEAR(currentStatus.status3, BIT_STATUS3_VSS_REFRESH);

}

void sendValuesLegacy()
{
  uint16_t temp;
  int bytestosend = 114;

  bytestosend -= Serial.write(currentStatus.secl>>8);
  bytestosend -= Serial.write(currentStatus.secl);
  bytestosend -= Serial.write(currentStatus.PW1>>8);
  bytestosend -= Serial.write(currentStatus.PW1);
  bytestosend -= Serial.write(currentStatus.PW2>>8);
  bytestosend -= Serial.write(currentStatus.PW2);
  bytestosend -= Serial.write(currentStatus.RPM>>8);
  bytestosend -= Serial.write(currentStatus.RPM);

  temp = currentStatus.advance * 10;
  bytestosend -= Serial.write(temp>>8);
  bytestosend -= Serial.write(temp);

  bytestosend -= Serial.write(currentStatus.nSquirts);
  bytestosend -= Serial.write(currentStatus.engine);
  bytestosend -= Serial.write(currentStatus.afrTarget);
  bytestosend -= Serial.write(currentStatus.afrTarget); // send twice so afrtgt1 == afrtgt2
  bytestosend -= Serial.write(99); // send dummy data as we don't have wbo2_en1
  bytestosend -= Serial.write(99); // send dummy data as we don't have wbo2_en2

  temp = currentStatus.baro * 10;
  bytestosend -= Serial.write(temp>>8);
  bytestosend -= Serial.write(temp);

  temp = currentStatus.MAP * 10;
  bytestosend -= Serial.write(temp>>8);
  bytestosend -= Serial.write(temp);

  temp = currentStatus.IAT * 10;
  bytestosend -= Serial.write(temp>>8);
  bytestosend -= Serial.write(temp);

  temp = currentStatus.coolant * 10;
  bytestosend -= Serial.write(temp>>8);
  bytestosend -= Serial.write(temp);

  temp = currentStatus.TPS * 10;
  bytestosend -= Serial.write(temp>>8);
  bytestosend -= Serial.write(temp);

  bytestosend -= Serial.write(currentStatus.battery10>>8);
  bytestosend -= Serial.write(currentStatus.battery10);
  bytestosend -= Serial.write(currentStatus.O2>>8);
  bytestosend -= Serial.write(currentStatus.O2);
  bytestosend -= Serial.write(currentStatus.O2_2>>8);
  bytestosend -= Serial.write(currentStatus.O2_2);

  bytestosend -= Serial.write(99); // knock
  bytestosend -= Serial.write(99); // knock

  temp = currentStatus.egoCorrection * 10;
  bytestosend -= Serial.write(temp>>8); // egocor1
  bytestosend -= Serial.write(temp); // egocor1
  bytestosend -= Serial.write(temp>>8); // egocor2
  bytestosend -= Serial.write(temp); // egocor2

  temp = currentStatus.iatCorrection * 10;
  bytestosend -= Serial.write(temp>>8); // aircor
  bytestosend -= Serial.write(temp); // aircor

  temp = currentStatus.wueCorrection * 10;
  bytestosend -= Serial.write(temp>>8); // warmcor
  bytestosend -= Serial.write(temp); // warmcor

  bytestosend -= Serial.write(99); // accelEnrich
  bytestosend -= Serial.write(99); // accelEnrich
  bytestosend -= Serial.write(99); // tpsFuelCut
  bytestosend -= Serial.write(99); // tpsFuelCut
  bytestosend -= Serial.write(99); // baroCorrection
  bytestosend -= Serial.write(99); // baroCorrection

  temp = currentStatus.corrections * 10;
  bytestosend -= Serial.write(temp>>8); // gammaEnrich
  bytestosend -= Serial.write(temp); // gammaEnrich

  temp = currentStatus.VE * 10;
  bytestosend -= Serial.write(temp>>8); // ve1
  bytestosend -= Serial.write(temp); // ve1
  temp = currentStatus.VE2 * 10;
  bytestosend -= Serial.write(temp>>8); // ve2
  bytestosend -= Serial.write(temp); // ve2

  bytestosend -= Serial.write(99); // iacstep
  bytestosend -= Serial.write(99); // iacstep
  bytestosend -= Serial.write(99); // cold_adv_deg
  bytestosend -= Serial.write(99); // cold_adv_deg

  temp = currentStatus.tpsDOT * 10;
  bytestosend -= Serial.write(temp>>8); // TPSdot
  bytestosend -= Serial.write(temp); // TPSdot

  temp = currentStatus.mapDOT * 10;
  bytestosend -= Serial.write(temp >> 8); // MAPdot
  bytestosend -= Serial.write(temp); // MAPdot

  temp = currentStatus.dwell * 10;
  bytestosend -= Serial.write(temp>>8); // dwell
  bytestosend -= Serial.write(temp); // dwell

  bytestosend -= Serial.write(99); // MAF
  bytestosend -= Serial.write(99); // MAF
  bytestosend -= Serial.write(currentStatus.fuelLoad*10); // fuelload
  bytestosend -= Serial.write(99); // fuelcor
  bytestosend -= Serial.write(99); // fuelcor
  bytestosend -= Serial.write(99); // portStatus

  temp = currentStatus.advance1 * 10;
  bytestosend -= Serial.write(temp>>8);
  bytestosend -= Serial.write(temp);
  temp = currentStatus.advance2 * 10;
  bytestosend -= Serial.write(temp>>8);
  bytestosend -= Serial.write(temp);

  for(int i = 0; i < bytestosend; i++)
  {
    // send dummy data to fill remote's buffer
    Serial.write(99);
  }
}

namespace {

  void send_raw_entity(const page_iterator_t &entity)
  {
    Serial.write((byte *)entity.pData, entity.size);
  }

  inline void send_table_values(table_value_iterator it)
  {
    while (!it.at_end())
    {
      auto row = *it;
      Serial.write(&*row, row.size());
      ++it;
    }
  }

  inline void send_table_axis(table_axis_iterator it)
  {
    while (!it.at_end())
    {
      Serial.write((byte)*it);
      ++it;
    }
  }

  void send_table_entity(const page_iterator_t &entity)
  {
    send_table_values(rows_begin(entity));
    send_table_axis(x_begin(entity));
    send_table_axis(y_begin(entity));
  }

  void send_entity(const page_iterator_t &entity)
  {
    switch (entity.type)
    {
    case Raw:
      return send_raw_entity(entity);
      break;

    case Table:
      return send_table_entity(entity);
      break;
    
    case NoEntity:
      // No-op
      break;

    default:
      abort();
      break;
    }
  }
}

/** Pack the data within the current page (As set with the 'P' command) into a buffer and send it.
 * 
 * Creates a page iterator by @ref page_begin() (See: pages.cpp). Sends page given in @ref currentPage.
 * 
 * Note that some translation of the data is required to lay it out in the way Megasqurit / TunerStudio expect it.
 * Data is sent in binary format, as defined by in each page in the speeduino.ini.
 */
void sendPage()
{
  page_iterator_t entity = page_begin(currentPage);

  while (entity.type!=End)
  {
    send_entity(entity);
    entity = advance(entity);
  }
}

namespace {

  /// Prints each element in the memory byte range (*first, *last).
  void serial_println_range(const byte *first, const byte *last)
  {
    while (first!=last)
    {
      Serial.println(*first);
      ++first;
    }
  }
  void serial_println_range(const uint16_t *first, const uint16_t *last)
  {
    while (first!=last)
    {
      Serial.println(*first);
      ++first;
    }
  }

  void serial_print_space_delimited(const byte *first, const byte *last)
  {
    while (first!=last)
    {
      Serial.print(*first);// This displays the values horizantially on the screen
      Serial.print(F(" "));
      ++first;
    }
    Serial.println();
  }
  #define serial_print_space_delimited_array(array) serial_print_space_delimited(array, _end_range_address(array))

  void serial_print_prepadding(byte value)
  {
    if (value < 100)
    {
      Serial.print(F(" "));
      if (value < 10)
      {
        Serial.print(F(" "));
      }
    }
  }

  void serial_print_prepadded_value(byte value)
  {
      serial_print_prepadding(value);
      Serial.print(value);
      Serial.print(F(" "));
  }

  void print_row(const table_axis_iterator &y_it, table_row_iterator row)
  {
    serial_print_prepadded_value((byte)*y_it);

    while (!row.at_end())
    {
      serial_print_prepadded_value(*row);
      ++row;
    }
    Serial.println();
  }

  void print_x_axis(const void *pTable, table_type_t key)
  {
    Serial.print(F("    "));

    auto x_it = x_begin(pTable, key);
    while(!x_it.at_end())
    {
      serial_print_prepadded_value((byte)*x_it);
      ++x_it;
    }
  }

  void serial_print_3dtable(const void *pTable, table_type_t key)
  {
    auto y_it = y_begin(pTable, key);
    auto row_it = rows_begin(pTable, key);

    while (!row_it.at_end())
    {
      print_row(y_it, *row_it);
      ++y_it;
      ++row_it;
    }

    print_x_axis(pTable, key);
    Serial.println();
  }
}

/** Send page as ASCII for debugging purposes.
 * Similar to sendPage(), however data is sent in human readable format. Sends page given in @ref currentPage.
 * 
 * This is used for testing only (Not used by TunerStudio) in order to see current map and config data without the need for TunerStudio. 
 */
void sendPageASCII()
{
  switch (currentPage)
  {
    case veMapPage:
      Serial.println(F("\nVE Map"));
      serial_print_3dtable(&fuelTable, fuelTable.type_key);
      break;

    case veSetPage:
      Serial.println(F("\nPg 2 Cfg"));
      // The following loop displays in human readable form of all byte values in config page 1 up to but not including the first array.
      serial_println_range((byte *)&configPage2, configPage2.wueValues);
      serial_print_space_delimited_array(configPage2.wueValues);
      // This displays all the byte values between the last array up to but not including the first unsigned int on config page 1
      serial_println_range(_end_range_byte_address(configPage2.wueValues), (byte*)&configPage2.injAng);
      // The following loop displays four unsigned ints
      serial_println_range(configPage2.injAng, configPage2.injAng + _countof(configPage2.injAng));
      // Following loop displays byte values between the unsigned ints
      serial_println_range(_end_range_byte_address(configPage2.injAng), (byte*)&configPage2.mapMax);
      Serial.println(configPage2.mapMax);
      // Following loop displays remaining byte values of the page
      serial_println_range(&configPage2.fpPrime, (byte *)&configPage2 + sizeof(configPage2));
      break;

    case ignMapPage:
      Serial.println(F("\nIgnition Map"));
      serial_print_3dtable(&ignitionTable, ignitionTable.type_key);
      break;

    case ignSetPage:
      Serial.println(F("\nPg 4 Cfg"));
      Serial.println(configPage4.triggerAngle);// configPage4.triggerAngle is an int so just display it without complication
      // Following loop displays byte values after that first int up to but not including the first array in config page 2
      serial_println_range((byte*)&configPage4.FixAng, configPage4.taeBins);
      serial_print_space_delimited_array(configPage4.taeBins);
      serial_print_space_delimited_array(configPage4.taeValues);
      serial_print_space_delimited_array(configPage4.wueBins);
      Serial.println(configPage4.dwellLimit);// Little lonely byte stuck between two arrays. No complications just display it.
      serial_print_space_delimited_array(configPage4.dwellCorrectionValues);
      serial_println_range(_end_range_byte_address(configPage4.dwellCorrectionValues), (byte *)&configPage4 + sizeof(configPage4));
      break;

    case afrMapPage:
      Serial.println(F("\nAFR Map"));
      serial_print_3dtable(&afrTable, afrTable.type_key);
      break;

    case afrSetPage:
      Serial.println(F("\nPg 6 Config"));
      serial_println_range((byte *)&configPage6, configPage6.voltageCorrectionBins);
      serial_print_space_delimited_array(configPage6.voltageCorrectionBins);
      serial_print_space_delimited_array(configPage6.injVoltageCorrectionValues);
      serial_print_space_delimited_array(configPage6.airDenBins);
      serial_print_space_delimited_array(configPage6.airDenRates);
      serial_println_range(_end_range_byte_address(configPage6.airDenRates), configPage6.iacCLValues);
      serial_print_space_delimited_array(configPage6.iacCLValues);
      serial_print_space_delimited_array(configPage6.iacOLStepVal);
      serial_print_space_delimited_array(configPage6.iacOLPWMVal);
      serial_print_space_delimited_array(configPage6.iacBins);
      serial_print_space_delimited_array(configPage6.iacCrankSteps);
      serial_print_space_delimited_array(configPage6.iacCrankDuty);
      serial_print_space_delimited_array(configPage6.iacCrankBins);
      // Following loop is for remaining byte value of page
      serial_println_range(_end_range_byte_address(configPage6.iacCrankBins), (byte *)&configPage6 + sizeof(configPage6));
      break;

    case boostvvtPage:
      Serial.println(F("\nBoost Map"));
      serial_print_3dtable(&boostTable, boostTable.type_key);
      Serial.println(F("\nVVT Map"));
      serial_print_3dtable(&vvtTable, vvtTable.type_key);
      break;

    case seqFuelPage:
      Serial.println(F("\nTrim 1 Table"));
      serial_print_3dtable(&trim1Table, trim1Table.type_key);
      break;

    case canbusPage:
      Serial.println(F("\nPage 9 Cfg"));
      serial_println_range((byte *)&configPage9, (byte *)&configPage9 + sizeof(configPage9));
      break;

    case fuelMap2Page:
      Serial.println(F("\n2nd Fuel Map"));
      serial_print_3dtable(&fuelTable2, fuelTable2.type_key);
      break;
   
    case ignMap2Page:
      Serial.println(F("\n2nd Ignition Map"));
      serial_print_3dtable(&ignitionTable2, ignitionTable2.type_key);
      break;

    case warmupPage:
    case progOutsPage:
    default:
    #ifndef SMALL_FLASH_MODE
        Serial.println(F("\nPage has not been implemented yet"));
    #endif
        break;
  }
}


/** Processes an incoming stream of calibration data (for CLT, IAT or O2) from TunerStudio.
 * Result is store in EEPROM and memory.
 * 
 * @param tableID - calibration table to process. 0 = Coolant Sensor. 1 = IAT Sensor. 2 = O2 Sensor.
 */
void receiveCalibration(byte tableID)
{
  void* pnt_TargetTable_values; //Pointer that will be used to point to the required target table values
  uint16_t* pnt_TargetTable_bins;   //Pointer that will be used to point to the required target table bins
  int OFFSET, DIVISION_FACTOR;

  switch (tableID)
  {
    case 0:
      //coolant table
      pnt_TargetTable_values = (uint16_t *)&cltCalibration_values;
      pnt_TargetTable_bins = (uint16_t *)&cltCalibration_bins;
      OFFSET = CALIBRATION_TEMPERATURE_OFFSET; //
      DIVISION_FACTOR = 10;
      break;
    case 1:
      //Inlet air temp table
      pnt_TargetTable_values = (uint16_t *)&iatCalibration_values;
      pnt_TargetTable_bins = (uint16_t *)&iatCalibration_bins;
      OFFSET = CALIBRATION_TEMPERATURE_OFFSET;
      DIVISION_FACTOR = 10;
      break;
    case 2:
      //O2 table
      //pnt_TargetTable = (byte *)&o2CalibrationTable;
      pnt_TargetTable_values = (uint8_t *)&o2Calibration_values;
      pnt_TargetTable_bins = (uint16_t *)&o2Calibration_bins;
      OFFSET = 0;
      DIVISION_FACTOR = 1;
      break;

    default:
      OFFSET = 0;
      pnt_TargetTable_values = (uint16_t *)&iatCalibration_values;
      pnt_TargetTable_bins = (uint16_t *)&iatCalibration_bins;
      DIVISION_FACTOR = 10;
      break; //Should never get here, but if we do, just fail back to main loop
  }

  int16_t tempValue;
  byte tempBuffer[2];

  if(tableID == 2)
  {
    //O2 calibration. Comes through as 1024 8-bit values of which we use every 32nd
    for (int x = 0; x < 1024; x++)
    {
      while ( Serial.available() < 1 ) {}
      tempValue = Serial.read();

      if( (x % 32) == 0)
      {
        ((uint8_t*)pnt_TargetTable_values)[(x/32)] = (byte)tempValue; //O2 table stores 8 bit values
        pnt_TargetTable_bins[(x/32)] = (x);
      }
      
    }
  }
  else
  {
    //Temperature calibrations are sent as 32 16-bit values
    for (uint16_t x = 0; x < 32; x++)
    {
      while ( Serial.available() < 2 ) {}
      tempBuffer[0] = Serial.read();
      tempBuffer[1] = Serial.read();

      tempValue = (int16_t)(word(tempBuffer[1], tempBuffer[0])); //Combine the 2 bytes into a single, signed 16-bit value
      tempValue = div(tempValue, DIVISION_FACTOR).quot; //TS sends values multipled by 10 so divide back to whole degrees. 
      tempValue = ((tempValue - 32) * 5) / 9; //Convert from F to C
      
      //Apply the temp offset and check that it results in all values being positive
      tempValue = tempValue + OFFSET;
      if (tempValue < 0) { tempValue = 0; }

      
      ((uint16_t*)pnt_TargetTable_values)[x] = tempValue; //Both temp tables have 16-bit values
      pnt_TargetTable_bins[x] = (x * 32U);
      writeCalibration();
    }
  }

  writeCalibration();
}

/** Send 256 tooth log entries to serial.
 * if useChar is true, the values are sent as chars to be printed out by a terminal emulator
 * if useChar is false, the values are sent as a 2 byte integer which is readable by TunerStudios tooth logger
*/
void sendToothLog_old(byte startOffset)
{
  //We need TOOTH_LOG_SIZE number of records to send to TunerStudio. If there aren't that many in the buffer then we just return and wait for the next call
  if (BIT_CHECK(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY)) //Sanity check. Flagging system means this should always be true
  {
      for (int x = startOffset; x < TOOTH_LOG_SIZE; x++)
      {
        //Check whether the tx buffer still has space
        if(Serial.availableForWrite() < 4) 
        { 
          //tx buffer is full. Store the current state so it can be resumed later
          inProgressOffset = x;
          toothLogSendInProgress = true;
          return;
        }


        Serial.write(toothHistory[x] >> 24);
        Serial.write(toothHistory[x] >> 16);
        Serial.write(toothHistory[x] >> 8);
        Serial.write(toothHistory[x]);
      }
      BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
      cmdPending = false;
      toothLogSendInProgress = false;
      toothHistoryIndex = 0;
  }
  else 
  { 
    //TunerStudio has timed out, send a LOG of all 0s
    for(int x = 0; x < (4*TOOTH_LOG_SIZE); x++)
    {
      Serial.write(static_cast<byte>(0x00)); //GCC9 fix
    }
    cmdPending = false; 
  } 
}

void sendCompositeLog_old(byte startOffset)
{
  if (BIT_CHECK(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY)) //Sanity check. Flagging system means this should always be true
  {
      if(startOffset == 0) { inProgressCompositeTime = 0; }
      for (int x = startOffset; x < TOOTH_LOG_SIZE; x++)
      {
        //Check whether the tx buffer still has space
        if(Serial.availableForWrite() < 4) 
        { 
          //tx buffer is full. Store the current state so it can be resumed later
          inProgressOffset = x;
          compositeLogSendInProgress = true;
          return;
        }

        inProgressCompositeTime = toothHistory[x]; //This combined runtime (in us) that the log was going for by this record)
        
        Serial.write(inProgressCompositeTime >> 24);
        Serial.write(inProgressCompositeTime >> 16);
        Serial.write(inProgressCompositeTime >> 8);
        Serial.write(inProgressCompositeTime);

        Serial.write(compositeLogHistory[x]); //The status byte (Indicates the trigger edge, whether it was a pri/sec pulse, the sync status)
      }
      BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
      toothHistoryIndex = 0;
      cmdPending = false;
      compositeLogSendInProgress = false;
      inProgressCompositeTime = 0;
  }
  else 
  { 
    //TunerStudio has timed out, send a LOG of all 0s
    for(int x = 0; x < (5*TOOTH_LOG_SIZE); x++)
    {
      Serial.write(static_cast<byte>(0x00)); //GCC9 fix
    }
    cmdPending = false; 
  } 
}

void testComm()
{
  Serial.write(1);
  return;
}
