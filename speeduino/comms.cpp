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
#include "table_iterator.h"
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

/** Processes the incoming data on the serial buffer based on the command sent.
Can be either data for a new command or a continuation of data for command that is already in progress:
- cmdPending = If a command has started but is wairing on further data to complete
- chunkPending = Specifically for the new receive value method where TS will send a known number of contiguous bytes to be written to a table

Comands are single byte (letter symbol) commands.
*/
void command()
{
  if (cmdPending == false) { currentCommand = Serial.read(); }

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
        uint32_t CRC32_val = calculateCRC32( Serial.read() );
        
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
      toothHistorySerialIndex = 0;

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
      toothHistorySerialIndex = 0;
      compositeLastToothTime = 0;

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
      Serial.print(F("speeduino 202104-dev"));
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
#ifdef RTC_ENABLED
        else if(cmd == SD_RTC_PAGE) //Request to read SD card RTC
        {
          /*
          uint16_t packetSize = 2 + 1 + length + 4;
          packetSize = 15;
          Serial.write(highByte(packetSize));
          Serial.write(lowByte(packetSize));
          byte packet[length+1];

          packet[0] = 0;
          packet[1] = length;
          packet[2] = 0;
          packet[3] = 0;
          packet[4] = 0;
          packet[5] = 0;
          packet[6] = 0;
          packet[7] = 0;
          packet[8] = 0;
          Serial.write(packet, 9);

          FastCRC32 CRC32;
          uint32_t CRC32_val = CRC32.crc32((byte *)packet, sizeof(packet) );;
      
          //Split the 4 bytes of the CRC32 value into individual bytes and send
          Serial.write( ((CRC32_val >> 24) & 255) );
          Serial.write( ((CRC32_val >> 16) & 255) );
          Serial.write( ((CRC32_val >> 8) & 255) );
          Serial.write( (CRC32_val & 255) );
          */
          Serial.write(rtc_getSecond()); //Seconds
          Serial.write(rtc_getMinute()); //Minutes
          Serial.write(rtc_getHour()); //Hours
          Serial.write(rtc_getDOW()); //Day of Week
          Serial.write(rtc_getDay()); //Date
          Serial.write(rtc_getMonth()); //Month
          Serial.write(lowByte(rtc_getYear())); //Year - NOTE 2 bytes
          Serial.write(highByte(rtc_getYear())); //Year

        }
        else if(cmd == SD_READWRITE_PAGE) //Request SD card extended parameters
        {
          //SD read commands use the offset and length fields to indicate the request type
          if((offset == SD_READ_STAT_OFFSET) && (length == SD_READ_STAT_LENGTH))
          {
            //Read the status of the SD card
            
            //Serial.write(0);


            //Serial.write(currentStatus.TS_SD_Status);
            Serial.write((uint8_t)5);
            Serial.write((uint8_t)0);

            //All other values are 2 bytes          
            Serial.write((uint8_t)2); //Sector size
            Serial.write((uint8_t)0); //Sector size

            //Max blocks (4 bytes)
            Serial.write((uint8_t)0);
            Serial.write((uint8_t)0x20); //1gb dummy card
            Serial.write((uint8_t)0);
            Serial.write((uint8_t)0);

            //Max roots (Number of files)
            Serial.write((uint8_t)0);
            Serial.write((uint8_t)1);

            //Dir Start (4 bytes)
            Serial.write((uint8_t)0); //Dir start lower 2 bytes
            Serial.write((uint8_t)0); //Dir start lower 2 bytes
            Serial.write((uint8_t)0); //Dir start lower 2 bytes
            Serial.write((uint8_t)0); //Dir start lower 2 bytes

            //Unkown purpose for last 2 bytes
            Serial.write((uint8_t)0); //Dir start lower 2 bytes
            Serial.write((uint8_t)0); //Dir start lower 2 bytes
            
            /*
            Serial.write(lowByte(23));
            Serial.write(highByte(23));

            byte packet[17];
            packet[0] = 0;
            packet[1] = 5;
            packet[2] = 0;

            packet[3] = 2;
            packet[4] = 0;

            packet[5] = 0;
            packet[6] = 0x20;
            packet[7] = 0;
            packet[8] = 0;

            packet[9] = 0;
            packet[10] = 1;

            packet[11] = 0;
            packet[12] = 0;
            packet[13] = 0;
            packet[14] = 0;

            packet[15] = 0;
            packet[16] = 0;

            Serial.write(packet, 17);
            FastCRC32 CRC32;
            uint32_t CRC32_val = CRC32.crc32((byte *)packet, sizeof(packet) );;
        
            //Split the 4 bytes of the CRC32 value into individual bytes and send
            Serial.write( ((CRC32_val >> 24) & 255) );
            Serial.write( ((CRC32_val >> 16) & 255) );
            Serial.write( ((CRC32_val >> 8) & 255) );
            Serial.write( (CRC32_val & 255) );
            */

          }
          //else if(length == 0x202)
          {
            //File info
          }
        }
        else if(cmd == 0x14)
        {
          //Fetch data from file
        }
#endif
        else
        {
          //No other r/ commands should be called
        }
        cmdPending = false;
      }
      break;

    case 'S': // send code version
      Serial.print(F("Speeduino 2021.04-dev"));
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

        if(currentStatus.toothLogEnabled == true) { sendToothLog(0); } //Sends tooth log values as ints
        else if (currentStatus.compositeLogEnabled == true) { sendCompositeLog(0); }

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
#ifdef RTC_ENABLED
      if(currentPage == SD_READWRITE_PAGE)
        { 
          cmdPending = false;

          //Reserved for the SD card settings. Appears to be hardcoded into TS. Flush the final byte in the buffer as its not used for now
          Serial.read(); 
          if((valueOffset == SD_WRITE_DO_OFFSET) && (chunkSize == SD_WRITE_DO_LENGTH))
          {
            /*
            SD DO command. Single byte of data where the commands are:
            0 Reset
            1 Reset
            2 Stop logging
            3 Start logging
            4 Load status variable
            5 Init SD card
            */
            Serial.read();
          }
          else if((valueOffset == SD_WRITE_SEC_OFFSET) && (chunkSize == SD_WRITE_SEC_LENGTH))
          {
            //SD write sector command
          }
          else if((valueOffset == SD_ERASEFILE_OFFSET) && (chunkSize == SD_ERASEFILE_LENGTH))
          {
            //Erase file command
            //First 4 bytes are the log number in ASCII
            /*
            char log1 = Serial.read();
            char log2 = Serial.read();
            char log3 = Serial.read();
            char log4 = Serial.read();
            */

            //Next 2 bytes are the directory block no
            Serial.read();
            Serial.read();
          }
          else if((valueOffset == SD_SPD_TEST_OFFSET) && (chunkSize == SD_SPD_TEST_LENGTH))
          {
            //Perform a speed test on the SD card
            //First 4 bytes are the sector number to write to
            Serial.read();
            Serial.read();
            Serial.read();
            Serial.read();

            //Last 4 bytes are the number of sectors to test
            Serial.read();
            Serial.read();
            Serial.read();
            Serial.read();
          }
        }
        else if(currentPage == SD_RTC_PAGE)
        {
          cmdPending = false;
          //Used for setting RTC settings
          if((valueOffset == SD_RTC_WRITE_OFFSET) && (chunkSize == SD_RTC_WRITE_LENGTH))
          {
            //Set the RTC date/time
            //Need to ensure there are 9 more bytes with the new values
            while(Serial.available() < 9) {} //Terrible hack, but RTC values should not be set with the engine running anyway
            byte second = Serial.read();
            byte minute = Serial.read();
            byte hour = Serial.read();
            //byte dow = Serial.read();
            Serial.read(); // This is the day of week value, which is currently unused
            byte day = Serial.read();
            byte month = Serial.read();
            uint16_t year = Serial.read();
            year = word(Serial.read(), year);
            Serial.read(); //Final byte is unused (Always has value 0x5a)
            rtc_setTime(second, minute, hour, day, month, year);
          }
        }
#endif
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
      sendToothLog(0); //Sends tooth log values as chars
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
      break;
  }
}
/** Send a numbered byte-field (partial field in case of mul;ti-byte fields) from "current status" structure.
 * Notes on fields:
 * - Numbered field will be fields from @ref currentStatus, but not at all in the internal order of strct (e.g. field RPM value, number 14 will be
 *   2nd field in struct)
 * - The fields stored in multi-byte types will be accessed lowbyte and highbyte separately (e.g. PW1 will be broken into numbered byte-fields 75,76)
 * @param byteNum - byte-Field number
 * @return Field value in 1 byte size struct fields or 1 byte partial value (chunk) on multibyte fields.
 */
byte getStatusEntry(uint16_t byteNum)
{
  byte statusValue = 0;

  switch(byteNum)
  {
    case 0: statusValue = currentStatus.secl; break; //secl is simply a counter that increments each second. Used to track unexpected resets (Which will reset this count to 0)
    case 1: statusValue = currentStatus.status1; break; //status1 Bitfield
    case 2: statusValue = currentStatus.engine; break; //Engine Status Bitfield
    case 3: statusValue = currentStatus.syncLossCounter; break;
    case 4: statusValue = lowByte(currentStatus.MAP); break; //2 bytes for MAP
    case 5: statusValue = highByte(currentStatus.MAP); break;
    case 6: statusValue = (byte)(currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET); break; //mat
    case 7: statusValue = (byte)(currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); break; //Coolant ADC
    case 8: statusValue = currentStatus.batCorrection; break; //Battery voltage correction (%)
    case 9: statusValue = currentStatus.battery10; break; //battery voltage
    case 10: statusValue = currentStatus.O2; break; //O2
    case 11: statusValue = currentStatus.egoCorrection; break; //Exhaust gas correction (%)
    case 12: statusValue = currentStatus.iatCorrection; break; //Air temperature Correction (%)
    case 13: statusValue = currentStatus.wueCorrection; break; //Warmup enrichment (%)
    case 14: statusValue = lowByte(currentStatus.RPM); break; //rpm HB
    case 15: statusValue = highByte(currentStatus.RPM); break; //rpm LB
    case 16: statusValue = (byte)(currentStatus.AEamount >> 1); break; //TPS acceleration enrichment (%) divided by 2 (Can exceed 255)
    case 17: statusValue = lowByte(currentStatus.corrections); break; //Total GammaE (%)
    case 18: statusValue = highByte(currentStatus.corrections); break; //Total GammaE (%)
    case 19: statusValue = currentStatus.VE1; break; //VE 1 (%)
    case 20: statusValue = currentStatus.VE2; break; //VE 2 (%)
    case 21: statusValue = currentStatus.afrTarget; break;
    case 22: statusValue = currentStatus.tpsDOT; break; //TPS DOT
    case 23: statusValue = scaleCrankAngleDown(currentStatus.advance); break;
    case 24: statusValue = currentStatus.TPS; break; // TPS (0% to 100%)
    
    case 25: 
      if(currentStatus.loopsPerSecond > 60000) { currentStatus.loopsPerSecond = 60000;}
      statusValue = lowByte(currentStatus.loopsPerSecond); 
      break;
    case 26: 
      if(currentStatus.loopsPerSecond > 60000) { currentStatus.loopsPerSecond = 60000;}
      statusValue = highByte(currentStatus.loopsPerSecond); 
      break;
    
    case 27: 
      currentStatus.freeRAM = freeRam();
      statusValue = lowByte(currentStatus.freeRAM); //(byte)((currentStatus.loopsPerSecond >> 8) & 0xFF);
      break; 
    case 28: 
      currentStatus.freeRAM = freeRam();
      statusValue = highByte(currentStatus.freeRAM); 
      break;

    case 29: statusValue = (byte)(currentStatus.boostTarget >> 1); break; //Divide boost target by 2 to fit in a byte
    case 30: statusValue = (byte)(currentStatus.boostDuty / 100); break;
    case 31: statusValue = currentStatus.spark; break; //Spark related bitfield

    //rpmDOT must be sent as a signed integer
    case 32: statusValue = lowByte(currentStatus.rpmDOT); break;
    case 33: statusValue = highByte(currentStatus.rpmDOT); break;

    case 34: statusValue = currentStatus.ethanolPct; break; //Flex sensor value (or 0 if not used)
    case 35: statusValue = currentStatus.flexCorrection; break; //Flex fuel correction (% above or below 100)
    case 36: statusValue = currentStatus.flexIgnCorrection; break; //Ignition correction (Increased degrees of advance) for flex fuel

    case 37: statusValue = currentStatus.idleLoad; break;
    case 38: statusValue = currentStatus.testOutputs; break;

    case 39: statusValue = currentStatus.O2_2; break; //O2
    case 40: statusValue = currentStatus.baro; break; //Barometer value

    case 41: statusValue = lowByte(currentStatus.canin[0]); break;
    case 42: statusValue = highByte(currentStatus.canin[0]); break;
    case 43: statusValue = lowByte(currentStatus.canin[1]); break;
    case 44: statusValue = highByte(currentStatus.canin[1]); break;
    case 45: statusValue = lowByte(currentStatus.canin[2]); break;
    case 46: statusValue = highByte(currentStatus.canin[2]); break;
    case 47: statusValue = lowByte(currentStatus.canin[3]); break;
    case 48: statusValue = highByte(currentStatus.canin[3]); break;
    case 49: statusValue = lowByte(currentStatus.canin[4]); break;
    case 50: statusValue = highByte(currentStatus.canin[4]); break;
    case 51: statusValue = lowByte(currentStatus.canin[5]); break;
    case 52: statusValue = highByte(currentStatus.canin[5]); break;
    case 53: statusValue = lowByte(currentStatus.canin[6]); break;
    case 54: statusValue = highByte(currentStatus.canin[6]); break;
    case 55: statusValue = lowByte(currentStatus.canin[7]); break;
    case 56: statusValue = highByte(currentStatus.canin[7]); break;
    case 57: statusValue = lowByte(currentStatus.canin[8]); break;
    case 58: statusValue = highByte(currentStatus.canin[8]); break;
    case 59: statusValue = lowByte(currentStatus.canin[9]); break;
    case 60: statusValue = highByte(currentStatus.canin[9]); break;
    case 61: statusValue = lowByte(currentStatus.canin[10]); break;
    case 62: statusValue = highByte(currentStatus.canin[10]); break;
    case 63: statusValue = lowByte(currentStatus.canin[11]); break;
    case 64: statusValue = highByte(currentStatus.canin[11]); break;
    case 65: statusValue = lowByte(currentStatus.canin[12]); break;
    case 66: statusValue = highByte(currentStatus.canin[12]); break;
    case 67: statusValue = lowByte(currentStatus.canin[13]); break;
    case 68: statusValue = highByte(currentStatus.canin[13]); break;
    case 69: statusValue = lowByte(currentStatus.canin[14]); break;
    case 70: statusValue = highByte(currentStatus.canin[14]); break;
    case 71: statusValue = lowByte(currentStatus.canin[15]); break;
    case 72: statusValue = highByte(currentStatus.canin[15]); break;

    case 73: statusValue = currentStatus.tpsADC; break;
    case 74: statusValue = getNextError(); break;

    case 75: statusValue = lowByte(currentStatus.PW1); break; //Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
    case 76: statusValue = highByte(currentStatus.PW1); break; //Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
    case 77: statusValue = lowByte(currentStatus.PW2); break; //Pulsewidth 2 multiplied by 10 in ms. Have to convert from uS to mS.
    case 78: statusValue = highByte(currentStatus.PW2); break; //Pulsewidth 2 multiplied by 10 in ms. Have to convert from uS to mS.
    case 79: statusValue = lowByte(currentStatus.PW3); break; //Pulsewidth 3 multiplied by 10 in ms. Have to convert from uS to mS.
    case 80: statusValue = highByte(currentStatus.PW3); break; //Pulsewidth 3 multiplied by 10 in ms. Have to convert from uS to mS.
    case 81: statusValue = lowByte(currentStatus.PW4); break; //Pulsewidth 4 multiplied by 10 in ms. Have to convert from uS to mS.
    case 82: statusValue = highByte(currentStatus.PW4); break; //Pulsewidth 4 multiplied by 10 in ms. Have to convert from uS to mS.

    case 83: statusValue = currentStatus.status3; break;
    case 84: statusValue = currentStatus.engineProtectStatus; break;
    case 85: statusValue = lowByte(currentStatus.fuelLoad); break;
    case 86: statusValue = highByte(currentStatus.fuelLoad); break;
    case 87: statusValue = lowByte(currentStatus.ignLoad); break;
    case 88: statusValue = highByte(currentStatus.ignLoad); break;
    case 89: statusValue = lowByte(currentStatus.dwell); break;
    case 90: statusValue = highByte(currentStatus.dwell); break;
    case 91: statusValue = currentStatus.CLIdleTarget; break;
    case 92: statusValue = currentStatus.mapDOT; break;
    case 93: statusValue = lowByte(scaleCrankAngleDown(currentStatus.vvt1Angle)); break; //2 bytes for vvt1Angle
    case 94: statusValue = highByte(scaleCrankAngleDown(currentStatus.vvt1Angle)); break;
    case 95: statusValue = scaleCrankAngleDown(currentStatus.vvt1TargetAngle); break;
    case 96: statusValue = (byte)(currentStatus.vvt1Duty); break;
    case 97: statusValue = lowByte(currentStatus.flexBoostCorrection); break;
    case 98: statusValue = highByte(currentStatus.flexBoostCorrection); break;
    case 99: statusValue = currentStatus.baroCorrection; break;
    case 100: statusValue = currentStatus.VE; break; //Current VE (%). Can be equal to VE1 or VE2 or a calculated value from both of them
    case 101: statusValue = currentStatus.ASEValue; break; //Current ASE (%)
    case 102: statusValue = lowByte(currentStatus.vss); break;
    case 103: statusValue = highByte(currentStatus.vss); break;
    case 104: statusValue = currentStatus.gear; break;
    case 105: statusValue = currentStatus.fuelPressure; break;
    case 106: statusValue = currentStatus.oilPressure; break;
    case 107: statusValue = currentStatus.wmiPW; break;
    case 108: statusValue = currentStatus.status4; break;
    case 109: statusValue = lowByte(scaleCrankAngleDown(currentStatus.vvt2Angle)); break; //2 bytes for vvt2Angle
    case 110: statusValue = highByte(scaleCrankAngleDown(currentStatus.vvt2Angle)); break;
    case 111: statusValue = scaleCrankAngleDown(currentStatus.vvt2TargetAngle); break;
    case 112: statusValue = (byte)(currentStatus.vvt2Duty); break;
    case 113: statusValue = currentStatus.outputsStatus; break;
    case 114: statusValue = (byte)(currentStatus.fuelTemp + CALIBRATION_TEMPERATURE_OFFSET); break; //Fuel temperature from flex sensor
    case 115: statusValue = currentStatus.fuelTempCorrection; break; //Fuel temperature Correction (%)
    case 116: statusValue = scaleCrankAngleDown(currentStatus.advance1); break; //advance 1 (%)
    case 117: statusValue = scaleCrankAngleDown(currentStatus.advance2); break; //advance 2 (%)
    case 118: statusValue = currentStatus.TS_SD_Status; break; //SD card status
    case 119: statusValue = lowByte(currentStatus.EMAP); break; //2 bytes for EMAP
    case 120: statusValue = highByte(currentStatus.EMAP); break;
  }

  return statusValue;

  //Each new inclusion here need to be added on speeduino.ini@L78, only list first byte of an integer and second byte as "INVALID"
  //Every 2-byte integer added here should have it's lowByte index added to fsIntIndex array on globals.ino@L116
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
    if (portNum == 0) { Serial.write(getStatusEntry(offset+x)); }
    #if defined(CANSerial_AVAILABLE)
      else if (portNum == 3){ CANSerial.write(getStatusEntry(offset+x)); }
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

  inline void send_table_values(table_row_iterator_t it)
  {
    while (!at_end(it))
    {
      auto row = get_row(it);
      Serial.write(row.pValue, row.pEnd-row.pValue);
      advance_row(it);
    }
  }

  inline void send_table_axis(table_axis_iterator_t it)
  {
    while (!at_end(it))
    {
      Serial.write(get_value(it));
      it = advance_axis(it);
    }
  }

  void send_table_entity(table3D *pTable)
  {
    send_table_values(rows_begin(pTable));
    send_table_axis(x_begin(pTable));
    send_table_axis(y_begin(pTable));
  }

  void send_entity(const page_iterator_t &entity)
  {
    switch (entity.type)
    {
    case Raw:
      return send_raw_entity(entity);
      break;

    case Table:
      return send_table_entity(entity.pTable);
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

  void print_row(const table_axis_iterator_t &y_it, table_row_t row)
  {
    serial_print_prepadded_value(get_value(y_it));

    while (!at_end(row))
    {
      serial_print_prepadded_value(*row.pValue++);
    }
    Serial.println();
  }

  void print_x_axis(const table3D &currentTable)
  {
    Serial.print(F("    "));

    auto x_it = x_begin(&currentTable);
    while(!at_end(x_it))
    {
      serial_print_prepadded_value(get_value(x_it));
      advance_axis(x_it);
    }
  }

  void serial_print_3dtable(const table3D &currentTable)
  {
    auto y_it = y_begin(&currentTable);
    auto row_it = rows_begin(&currentTable);

    while (!at_end(row_it))
    {
      print_row(y_it, get_row(row_it));
      advance_axis(y_it);
      advance_row(row_it);
    }

    print_x_axis(currentTable);
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
      serial_print_3dtable(fuelTable);
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
      serial_print_3dtable(ignitionTable);
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
      serial_print_3dtable(afrTable);
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
      serial_print_3dtable(boostTable);
      Serial.println(F("\nVVT Map"));
      serial_print_3dtable(vvtTable);
      break;

    case seqFuelPage:
      Serial.println(F("\nTrim 1 Table"));
      serial_print_3dtable(trim1Table);
      break;

    case canbusPage:
      Serial.println(F("\nPage 9 Cfg"));
      serial_println_range((byte *)&configPage9, (byte *)&configPage9 + sizeof(configPage9));
      break;

    case fuelMap2Page:
      Serial.println(F("\n2nd Fuel Map"));
      serial_print_3dtable(fuelTable2);
      break;
   
    case ignMap2Page:
      Serial.println(F("\n2nd Ignition Map"));
      serial_print_3dtable(ignitionTable2);
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
void sendToothLog(byte startOffset)
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
        //Serial.write(highByte(toothHistory[toothHistorySerialIndex]));
        //Serial.write(lowByte(toothHistory[toothHistorySerialIndex]));
        Serial.write(toothHistory[toothHistorySerialIndex] >> 24);
        Serial.write(toothHistory[toothHistorySerialIndex] >> 16);
        Serial.write(toothHistory[toothHistorySerialIndex] >> 8);
        Serial.write(toothHistory[toothHistorySerialIndex]);

        if(toothHistorySerialIndex == (TOOTH_LOG_BUFFER-1)) { toothHistorySerialIndex = 0; }
        else { toothHistorySerialIndex++; }
      }
      BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
      cmdPending = false;
      toothLogSendInProgress = false;
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

void sendCompositeLog(byte startOffset)
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

        inProgressCompositeTime += toothHistory[toothHistorySerialIndex]; //This combined runtime (in us) that the log was going for by this record)
        
        Serial.write(inProgressCompositeTime >> 24);
        Serial.write(inProgressCompositeTime >> 16);
        Serial.write(inProgressCompositeTime >> 8);
        Serial.write(inProgressCompositeTime);

        Serial.write(compositeLogHistory[toothHistorySerialIndex]); //The status byte (Indicates the trigger edge, whether it was a pri/sec pulse, the sync status)

        if(toothHistorySerialIndex == (TOOTH_LOG_BUFFER-1)) { toothHistorySerialIndex = 0; }
        else { toothHistorySerialIndex++; }
      }
      BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
      toothHistoryIndex = 0;
      toothHistorySerialIndex = 0;
      compositeLastToothTime = 0;
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
