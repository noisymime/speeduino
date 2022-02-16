/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
/** @file
 * Process Incoming and outgoing serial communications.
 */
#include "globals.h"
#include "newComms.h"
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
#include "comms.h"
#include "src/FastCRC/FastCRC.h"
#ifdef RTC_ENABLED
  #include "rtc_common.h"
#endif
#ifdef SD_LOGGING
  #include "SD_logger.h"
#endif

uint16_t serialPayloadLength = 0;
bool serialReceivePending = false; /**< Whether or not a serial request has only been partially received. This occurs when a the length has been received in the serial buffer, but not all of the payload or CRC has yet been received. */
uint16_t serialBytesReceived = 0; /**< The number of bytes received in the serial buffer during the current command. */
uint32_t serialCRC = 0; 
bool serialWriteInProgress = false;
uint16_t serialBytesTransmitted = 0;
uint32_t serialReceiveStartTime = 0; /**< The time at which the serial receive started. Used for calculating whether a timeout has occurred */
FastCRC32 CRC32_serial; //This instance of CRC32 is exclusively used on the comms envelope CRC validations. It is separate to those used for page or calibration calculations to prevent update calls clashing with one another
#ifdef RTC_ENABLED
  uint8_t serialPayload[SD_FILE_TRANSMIT_BUFFER_SIZE]; /**< Serial payload buffer must be significantly larger for boards that support SD logging. Large enough to contain 4 sectors + overhead */
  uint16_t SDcurrentDirChunk;
  uint32_t SDreadStartSector;
  uint32_t SDreadNumSectors;
  uint32_t SDreadCompletedSectors = 0;
#else
  uint8_t serialPayload[SERIAL_BUFFER_SIZE]; /**< Serial payload buffer. */
#endif

/** Processes the incoming data on the serial buffer based on the command sent.
Can be either data for a new command or a continuation of data for command that is already in progress:
- cmdPending = If a command has started but is wairing on further data to complete
- chunkPending = Specifically for the new receive value method where TS will send a known number of contiguous bytes to be written to a table

Comands are single byte (letter symbol) commands.
*/
void parseSerial()
{

  //Check for an existing legacy command in progress
  if(cmdPending == true)
  {
    command();
    return;
  }

  if (serialReceivePending == false)
  { 
    serialBytesReceived = 0; //Reset the number of bytes received as we're starting a new command

    //New command received
    //Need at least 2 bytes to read the length of the command
    serialReceivePending = true; //Flag the serial receive as being in progress
    byte lowByte = Serial.read();

    //Check if the command is legacy using the call/response mechanism
    if((lowByte >= 'A') && (lowByte <= 'z') )
    {
      //Handle legacy cases here
      serialReceivePending = false; //Make sure new serial handling does not interfere with legacy handling
      legacySerial = true;
      currentCommand = lowByte;
      command();
    }
    else
    {
      while(Serial.available() == 0) { } //Wait for the 2nd byte to be received (This will almost never happen)

      byte highByte = Serial.read();
      serialPayloadLength = word(lowByte, highByte);
      serialBytesReceived = 2;
      cmdPending = false; // Make sure legacy handling does not interfere with new serial handling
      serialReceiveStartTime = millis();
    }
  }

  //If there is a serial receive in progress, read as much from the buffer as possible or until we receive all bytes
  while( (Serial.available() > 0) && (serialReceivePending == true) )
  {
    if (serialBytesReceived < (serialPayloadLength + SERIAL_LEN_SIZE) )
    {
      serialPayload[(serialBytesReceived - SERIAL_LEN_SIZE)] = Serial.read();
      serialBytesReceived++;
    }
    else if (Serial.available() >= SERIAL_CRC_LENGTH)
    {
      uint32_t crc1 = Serial.read();
      uint32_t crc2 = Serial.read();
      uint32_t crc3 = Serial.read();
      uint32_t crc4 = Serial.read();
      serialCRC = (crc1<<24) | (crc2<<16) | (crc3<<8) | crc4;
      
      serialReceivePending = false; //The serial receive is now complete

      //Test the CRC
      uint32_t receivedCRC = CRC32_serial.crc32(serialPayload, serialPayloadLength);
      //receivedCRC++;
      if(serialCRC != receivedCRC)
      {
        //CRC Error. Need to send an error message
        sendSerialReturnCode(SERIAL_RC_CRC_ERR);
      }
      else
      {
        //CRC is correct. Process the command
        processSerialCommand();
      } //CRC match
    } //CRC received in full

    //Check for a timeout
    if( (millis() - serialReceiveStartTime) > SERIAL_TIMEOUT)
    {
      //Timeout occurred
      serialReceivePending = false; //Reset the serial receive

      //Flush the serial buffer
      while(Serial.available() > 0)
      {
        Serial.read();
      }
      sendSerialReturnCode(SERIAL_RC_TIMEOUT);
    } //Timeout
  } //Data in serial buffer and serial receive in progress
}

void sendSerialReturnCode(byte returnCode)
{
  Serial.write((uint8_t)0);
  Serial.write((uint8_t)1); //Size is always 1

  Serial.write(returnCode);

  //Calculate and send CRC
  uint32_t CRC32_val = CRC32_serial.crc32(&returnCode, 1);
  Serial.write( ((CRC32_val >> 24) & 255) );
  Serial.write( ((CRC32_val >> 16) & 255) );
  Serial.write( ((CRC32_val >> 8) & 255) );
  Serial.write( (CRC32_val & 255) );
}

void sendSerialPayload(void *payload, uint16_t payloadLength)
{
  //Start new transmission session
  serialBytesTransmitted = 0; 
  serialWriteInProgress = false;

  uint16_t totalPayloadLength = payloadLength;
  Serial.write(totalPayloadLength >> 8);
  Serial.write(totalPayloadLength);

  //Need to handle serial buffer being full. This is just for testing
  serialPayloadLength = payloadLength; //Save the payload length incase we need to transmit in multiple steps
  for(uint16_t i = 0; i < payloadLength; i++)
  {
    Serial.write(((uint8_t*)payload)[i]);
    serialBytesTransmitted++;

    if(Serial.availableForWrite() == 0)
    {
      //Serial buffer is full. Need to wait for it to be free
      serialWriteInProgress = true;
      break;
    }
  }

  if(serialWriteInProgress == false)
  {
    //All data transmitted. Send the CRC
    uint32_t CRC32_val = CRC32_serial.crc32((uint8_t*)payload, payloadLength);
    Serial.write( ((CRC32_val >> 24) & 255) );
    Serial.write( ((CRC32_val >> 16) & 255) );
    Serial.write( ((CRC32_val >> 8) & 255) );
    Serial.write( (CRC32_val & 255) );
  }
}

void continueSerialTransmission()
{
  if(serialWriteInProgress == true)
  {
    serialWriteInProgress = false; //Assume we will reach the end of the serial buffer. If we run out of buffer, this will be set to true below
    //Serial buffer is free. Continue sending the data
    for(uint16_t i = serialBytesTransmitted; i < serialPayloadLength; i++)
    {
      Serial.write(serialPayload[i]);
      serialBytesTransmitted++;

      if(Serial.availableForWrite() == 0)
      {
        //Serial buffer is full. Need to wait for it to be free
        serialWriteInProgress = true;
        break;
      }
    }

    if(serialWriteInProgress == false)
    {
      //All data transmitted. Send the CRC
      uint32_t CRC32_val = CRC32_serial.crc32(serialPayload, serialPayloadLength);
      Serial.write( ((CRC32_val >> 24) & 255) );
      Serial.write( ((CRC32_val >> 16) & 255) );
      Serial.write( ((CRC32_val >> 8) & 255) );
      Serial.write( (CRC32_val & 255) );
    }
  }
}

void processSerialCommand()
{
  currentCommand = serialPayload[0];

  switch (currentCommand)
  {

    case 'A': // send x bytes of realtime values
      //sendValues(0, LOG_ENTRY_SIZE, 0x31, 0);   //send values to serial0
      generateLiveValues(0, LOG_ENTRY_SIZE); 
      break;

    case 'b': // New EEPROM burn command to only burn a single page at a time 
      writeConfig(serialPayload[2]); //Read the table number and perform burn. Note that byte 1 in the array is unused
      sendSerialReturnCode(SERIAL_RC_BURN_OK);
      break;

    case 'C': // test communications. This is used by Tunerstudio to see whether there is an ECU on a given serial port
    {
      uint8_t tempPayload[] = {SERIAL_RC_OK, currentStatus.secl};
      sendSerialPayload(&tempPayload, 2);
      break;
    }

    case 'd': // Send a CRC32 hash of a given page
    {
      uint32_t CRC32_val = calculatePageCRC32( serialPayload[2] );
      uint8_t payloadCRC32[5];
      
      //First byte is the flag
      payloadCRC32[0] = SERIAL_RC_OK;

      //Split the 4 bytes of the CRC32 value into individual bytes and send
      payloadCRC32[1] =  ((CRC32_val >> 24) & 255);
      payloadCRC32[2] = ((CRC32_val >> 16) & 255);
      payloadCRC32[3] = ((CRC32_val >> 8) & 255);
      payloadCRC32[4] = (CRC32_val & 255);
      
      sendSerialPayload( &payloadCRC32, 5);

      break;
    }

    case 'E': // receive command button commands
    {
      uint16_t cmdCombined = word(serialPayload[1], serialPayload[2]);

      if ( ((cmdCombined >= TS_CMD_INJ1_ON) && (cmdCombined <= TS_CMD_IGN8_50PC)) || (cmdCombined == TS_CMD_TEST_ENBL) || (cmdCombined == TS_CMD_TEST_DSBL) )
      {
        //Hardware test buttons
        if (currentStatus.RPM == 0) { TS_CommandButtonsHandler(cmdCombined); }
      }
      else if( (cmdCombined >= TS_CMD_VSS_60KMH) && (cmdCombined <= TS_CMD_VSS_RATIO6) )
      {
        //VSS Calibration commands
        TS_CommandButtonsHandler(cmdCombined);
      }
      else if( (cmdCombined >= TS_CMD_STM32_REBOOT) && (cmdCombined <= TS_CMD_STM32_BOOTLOADER) )
      {
        //STM32 DFU mode button
        TS_CommandButtonsHandler(cmdCombined);
      }
      else if( (cmdCombined >= TS_CMD_SD_FORMAT) && (cmdCombined <= TS_CMD_SD_FORMAT) )
      {
        //SD Commands
        TS_CommandButtonsHandler(cmdCombined);
      }
      sendSerialReturnCode(SERIAL_RC_OK);
      break;
    }

    case 'F': // send serial protocol version
    {
      byte serialVersion[] = {SERIAL_RC_OK, '0', '0', '2'};
      sendSerialPayload(&serialVersion, 4);
      break;
    }

    case 'H': //Start the tooth logger
      currentStatus.toothLogEnabled = true;
      currentStatus.compositeLogEnabled = false; //Safety first (Should never be required)
      toothLogSendInProgress = false;
      BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
      toothHistoryIndex = 0;

      //Disconnect the standard interrupt and add the logger version
      detachInterrupt( digitalPinToInterrupt(pinTrigger) );
      attachInterrupt( digitalPinToInterrupt(pinTrigger), loggerPrimaryISR, CHANGE );

      detachInterrupt( digitalPinToInterrupt(pinTrigger2) );
      attachInterrupt( digitalPinToInterrupt(pinTrigger2), loggerSecondaryISR, CHANGE );

      sendSerialReturnCode(SERIAL_RC_OK);
      break;

    case 'h': //Stop the tooth logger
      currentStatus.toothLogEnabled = false;

      //Disconnect the logger interrupts and attach the normal ones
      detachInterrupt( digitalPinToInterrupt(pinTrigger) );
      attachInterrupt( digitalPinToInterrupt(pinTrigger), triggerHandler, primaryTriggerEdge );

      detachInterrupt( digitalPinToInterrupt(pinTrigger2) );
      attachInterrupt( digitalPinToInterrupt(pinTrigger2), triggerSecondaryHandler, secondaryTriggerEdge );
      sendSerialReturnCode(SERIAL_RC_OK);
      break;

    case 'I': // send CAN ID
    {
      byte serialVersion[] = {SERIAL_RC_OK, 0};
      sendSerialPayload(&serialVersion, 2);
      break;
    }

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

      sendSerialReturnCode(SERIAL_RC_OK);
      break;

    case 'j': //Stop the composite logger
      currentStatus.compositeLogEnabled = false;

      //Disconnect the logger interrupts and attach the normal ones
      detachInterrupt( digitalPinToInterrupt(pinTrigger) );
      attachInterrupt( digitalPinToInterrupt(pinTrigger), triggerHandler, primaryTriggerEdge );

      detachInterrupt( digitalPinToInterrupt(pinTrigger2) );
      attachInterrupt( digitalPinToInterrupt(pinTrigger2), triggerSecondaryHandler, secondaryTriggerEdge );
      sendSerialReturnCode(SERIAL_RC_OK);
      break;

    case 'k': //Send CRC values for the calibration pages
    {
      uint32_t CRC32_val = readCalibrationCRC32(serialPayload[2]); //Get the CRC for the requested page

      serialPayload[0] = SERIAL_RC_OK;
      serialPayload[1] = ((CRC32_val >> 24) & 255);
      serialPayload[2] = ((CRC32_val >> 16) & 255);
      serialPayload[3] = ((CRC32_val >> 8) & 255);
      serialPayload[4] = (CRC32_val & 255);
      sendSerialPayload( &serialPayload, 5);

      break;
    }

    case 'M':
    {
      //New write command
      //7 bytes required:
      //2 - Page identifier
      //2 - offset
      //2 - Length
      //1 - 1st New value
      byte offset1, offset2, length1, length2;

      uint8_t currentPage = serialPayload[2]; //Page ID is 2 bytes, but as the first byte is always 0 it can be ignored
      offset1 = serialPayload[3];
      offset2 = serialPayload[4];
      uint16_t valueOffset = word(offset2, offset1);
      length1 = serialPayload[5];
      length2 = serialPayload[6];
      uint16_t chunkSize = word(length2, length1);

      if( (valueOffset + chunkSize) > getPageSize(currentPage))
      {
        //This should never happen, but just incase
        sendSerialReturnCode(SERIAL_RC_RANGE_ERR);
        break;
      }

      for(uint16_t i = 0; i < chunkSize; i++)
      {
        setPageValue(currentPage, (valueOffset + i), serialPayload[7 + i]);
      }
      
      deferEEPROMWrites = true;
      
      sendSerialReturnCode(SERIAL_RC_OK);
      
      break;
    }  

    /*
    * New method for sending page values (MS command equivalent is 'r')
    */
    case 'p':
    {
      //6 bytes required:
      //2 - Page identifier
      //2 - offset
      //2 - Length
      byte offset1, offset2, length1, length2;
      int length;
      byte tempPage;

      tempPage = serialPayload[2];
      offset1 = serialPayload[3];
      offset2 = serialPayload[4];
      valueOffset = word(offset2, offset1);
      length1 = serialPayload[5];
      length2 = serialPayload[6];
      length = word(length2, length1);

      //Setup the transmit buffer
      serialPayload[0] = SERIAL_RC_OK;
      for(int i = 0; i < length; i++)
      {
        serialPayload[i+1] = getPageValue(tempPage, valueOffset + i);
      }
      sendSerialPayload(&serialPayload, (length + 1));
      break;
    }

    case 'Q': // send code version
    {
      char productString[] = { SERIAL_RC_OK, 's','p','e','e','d','u','i','n','o',' ','2','0','2','2','0','4','-','d','e','v'} ; //Note no null terminator in array and statu variable at the start
      //char productString[] = { SERIAL_RC_OK, 's','p','e','e','d','u','i','n','o',' ','2','0','2','2','0','4'} ; //Note no null terminator in array and statu variable at the start
      sendSerialPayload(&productString, sizeof(productString));
      break;
    }

    case 'r': //New format for the optimised OutputChannels
    {
      uint8_t cmd = serialPayload[2];
      uint16_t offset = word(serialPayload[4], serialPayload[3]);
      uint16_t length = word(serialPayload[6], serialPayload[5]);
#ifdef RTC_ENABLED      
      uint16_t SD_arg1 = word(serialPayload[3], serialPayload[4]);
      uint16_t SD_arg2 = word(serialPayload[5], serialPayload[6]);
#endif

      if(cmd == 0x30) //Send output channels command 0x30 is 48dec
      {
        generateLiveValues(offset, length);
        sendSerialPayload(&serialPayload, (length + 1));
      }
#ifdef RTC_ENABLED
      else if(cmd == SD_RTC_PAGE) //Request to read SD card RTC
      {
        serialPayload[0] = SERIAL_RC_OK;
        serialPayload[1] = rtc_getSecond(); //Seconds
        serialPayload[2] = rtc_getMinute(); //Minutes
        serialPayload[3] = rtc_getHour(); //Hours
        serialPayload[4] = rtc_getDOW(); //Day of week
        serialPayload[5] = rtc_getDay(); //Day of month
        serialPayload[6] = rtc_getMonth(); //Month
        serialPayload[7] = highByte(rtc_getYear()); //Year
        serialPayload[8] = lowByte(rtc_getYear()); //Year
        sendSerialPayload(&serialPayload, 9);

      }
      else if(cmd == SD_READWRITE_PAGE) //Request SD card extended parameters
      {
        //SD read commands use the offset and length fields to indicate the request type
        if((SD_arg1 == SD_READ_STAT_ARG1) && (SD_arg2 == SD_READ_STAT_ARG2))
        {
          //Read the status of the SD card
          
          serialPayload[0] = SERIAL_RC_OK;

          serialPayload[1] = currentStatus.TS_SD_Status;
          serialPayload[2] = 0; //Error code
 
          //Sector size = 512
          serialPayload[3] = 2;
          serialPayload[4] = 0;

          //Max blocks (4 bytes)
          uint32_t sectors = sectorCount();
          serialPayload[5] = ((sectors >> 24) & 255);
          serialPayload[6] = ((sectors >> 16) & 255);
          serialPayload[7] = ((sectors >> 8) & 255);
          serialPayload[8] = (sectors & 255);
          /*
          serialPayload[5] = 0;
          serialPayload[6] = 0x20; //1gb dummy card
          serialPayload[7] = 0;
          serialPayload[8] = 0;
          */

          //Max roots (Number of files)
          uint16_t numLogFiles = getNextSDLogFileNumber() - 2; // -1 because this returns the NEXT file name not the current one and -1 because TS expects a 0 based index
          serialPayload[9] = highByte(numLogFiles);
          serialPayload[10] = lowByte(numLogFiles);

          //Dir Start (4 bytes)
          serialPayload[11] = 0;
          serialPayload[12] = 0;
          serialPayload[13] = 0;
          serialPayload[14] = 0;

          //Unkown purpose for last 2 bytes
          serialPayload[15] = 0;
          serialPayload[16] = 0;

          sendSerialPayload(&serialPayload, 17);

        }
        else if((SD_arg1 == SD_READ_DIR_ARG1) && (SD_arg2 == SD_READ_DIR_ARG2))
        {
          //Send file details
          serialPayload[0] = SERIAL_RC_OK;

          uint16_t logFileNumber = (SDcurrentDirChunk * 16) + 1;
          uint8_t filesInCurrentChunk = 0;
          uint16_t payloadIndex = 1;
          while((filesInCurrentChunk < 16) && (getSDLogFileDetails(&serialPayload[payloadIndex], logFileNumber) == true))
          {
            logFileNumber++;
            filesInCurrentChunk++;
            payloadIndex += 32;
          }
          serialPayload[payloadIndex] = lowByte(SDcurrentDirChunk);
          serialPayload[payloadIndex + 1] = highByte(SDcurrentDirChunk);
          //Serial.print("Index:");
          //Serial.print(payloadIndex);

          sendSerialPayload(&serialPayload, (payloadIndex + 2));

        }

      }
      else if(cmd == SD_READFILE_PAGE)
      {
        //Fetch data from file
        if(SD_arg2 == SD_READ_COMP_ARG2)
        {
          //arg1 is the block number to return
          serialPayload[0] = SERIAL_RC_OK;
          serialPayload[1] = highByte(SD_arg1);
          serialPayload[2] = lowByte(SD_arg1);

          uint32_t currentSector = SDreadStartSector + (SD_arg1 * 4);
          
          int32_t numSectorsToSend = 0;
          if(SDreadNumSectors > SDreadCompletedSectors)
          {
            numSectorsToSend = SDreadNumSectors - SDreadCompletedSectors;
            if(numSectorsToSend > 4) //Maximum of 4 sectors at a time
            {
              numSectorsToSend = 4;
            }
          }
          SDreadCompletedSectors += numSectorsToSend;
          
          if(numSectorsToSend <= 0) { sendSerialReturnCode(SERIAL_RC_OK); }
          else
          {
            readSDSectors(&serialPayload[3], currentSector, numSectorsToSend); 
            sendSerialPayload(&serialPayload, (numSectorsToSend * SD_SECTOR_SIZE + 3));
          }
        }
      }
#endif
      else
      {
        //No other r/ commands should be called
      }
      cmdPending = false;
    
      break;
    }

    case 'S': // send code version
    {
      byte productString[] = { SERIAL_RC_OK, 'S', 'p', 'e', 'e', 'd', 'u', 'i', 'n', 'o', ' ', '2', '0', '2', '2', '.', '0', '4', '-', 'd', 'e', 'v'};
      //byte productString[] = { SERIAL_RC_OK, 'S', 'p', 'e', 'e', 'd', 'u', 'i', 'n', 'o', ' ', '2', '0', '2', '2', '0', '2'};
      sendSerialPayload(&productString, sizeof(productString));
      currentStatus.secl = 0; //This is required in TS3 due to its stricter timings
      break;
    }
      

    case 'T': //Send 256 tooth log entries to Tuner Studios tooth logger
      if(currentStatus.toothLogEnabled == true) { sendToothLog(0); } //Sends tooth log values as ints
      else if (currentStatus.compositeLogEnabled == true) { sendCompositeLog(0); }

      break;

    case 't': // receive new Calibration info. Command structure: "t", <tble_idx> <data array>.
    {
      uint8_t cmd = serialPayload[2];
      uint16_t valueOffset = word(serialPayload[3], serialPayload[4]);
      uint16_t calibrationLength = word(serialPayload[5], serialPayload[6]); // Should be 256
      uint32_t calibrationCRC = 0;

      if(cmd == O2_CALIBRATION_PAGE)
      {
        //TS sends a total of 1024 bytes of calibration data, broken up into 256 byte chunks
        //As we're using an interpolated 2D table, we only need to store 32 values out of this 1024
        void* pnt_TargetTable_values = (uint8_t *)&o2Calibration_values; //Pointer that will be used to point to the required target table values
        uint16_t* pnt_TargetTable_bins = (uint16_t *)&o2Calibration_bins; //Pointer that will be used to point to the required target table bins

        //Read through the current chunk (Should be 256 bytes long)
        for(uint16_t x = 0; x < calibrationLength; x++)
        {
          uint16_t totalOffset = valueOffset + x;
          //Only apply every 32nd value
          if( (x % 32) == 0 )
          {
            ((uint8_t*)pnt_TargetTable_values)[(totalOffset/32)] = serialPayload[x+7]; //O2 table stores 8 bit values
            pnt_TargetTable_bins[(totalOffset/32)] = (totalOffset);

          }

          //Update the CRC
          if(totalOffset == 0)
          {
            calibrationCRC = CRC32.crc32(&serialPayload[7], 1, false);
          }
          else
          {
            calibrationCRC = CRC32.crc32_upd(&serialPayload[x+7], 1, false);
          }
          //Check if CRC is finished
          if(totalOffset == 1023) 
          {
            //apply CRC reflection
            calibrationCRC = ~calibrationCRC;
            storeCalibrationCRC32(O2_CALIBRATION_PAGE, calibrationCRC);
          }
        }
        sendSerialReturnCode(SERIAL_RC_OK);
        Serial.flush(); //This is safe because engine is assumed to not be running during calibration

        //Check if this is the final chunk of calibration data
        #ifdef CORE_STM32
          //STM32 requires TS to send 16 x 64 bytes chunk rather than 4 x 256 bytes. 
          if(valueOffset == (64*15)) { writeCalibrationPage(cmd); } //Store received values in EEPROM if this is the final chunk of calibration
        #else
          if(valueOffset == (256*3)) { writeCalibrationPage(cmd); } //Store received values in EEPROM if this is the final chunk of calibration
        #endif
      }
      else if(cmd == IAT_CALIBRATION_PAGE)
      {
        void* pnt_TargetTable_values = (uint16_t *)&iatCalibration_values;
        uint16_t* pnt_TargetTable_bins = (uint16_t *)&iatCalibration_bins;

        //Temperature calibrations are sent as 32 16-bit values (ie 64 bytes total)
        if(calibrationLength == 64)
        {
          for (uint16_t x = 0; x < 32; x++)
          {
            int16_t tempValue = (int16_t)(word(serialPayload[((2 * x) + 8)], serialPayload[((2 * x) + 7)])); //Combine the 2 bytes into a single, signed 16-bit value
            tempValue = div(tempValue, 10).quot; //TS sends values multipled by 10 so divide back to whole degrees. 
            tempValue = ((tempValue - 32) * 5) / 9; //Convert from F to C
            
            //Apply the temp offset and check that it results in all values being positive
            tempValue = tempValue + CALIBRATION_TEMPERATURE_OFFSET;
            if (tempValue < 0) { tempValue = 0; }

            
            ((uint16_t*)pnt_TargetTable_values)[x] = tempValue; //Both temp tables have 16-bit values
            pnt_TargetTable_bins[x] = (x * 32U);
          }
          //Update the CRC
          calibrationCRC = CRC32.crc32(&serialPayload[7], 64);
          storeCalibrationCRC32(IAT_CALIBRATION_PAGE, calibrationCRC);

          writeCalibration();
          sendSerialReturnCode(SERIAL_RC_OK);
        }
        else { sendSerialReturnCode(SERIAL_RC_RANGE_ERR); }
        
      }
      else if(cmd == CLT_CALIBRATION_PAGE)
      {
        void* pnt_TargetTable_values = (uint16_t *)&cltCalibration_values;
        uint16_t* pnt_TargetTable_bins = (uint16_t *)&cltCalibration_bins;

        //Temperature calibrations are sent as 32 16-bit values
        if(calibrationLength == 64)
        {
          for (uint16_t x = 0; x < 32; x++)
          {
            int16_t tempValue = (int16_t)(word(serialPayload[((2 * x) + 8)], serialPayload[((2 * x) + 7)])); //Combine the 2 bytes into a single, signed 16-bit value
            tempValue = div(tempValue, 10).quot; //TS sends values multipled by 10 so divide back to whole degrees. 
            tempValue = ((tempValue - 32) * 5) / 9; //Convert from F to C
            
            //Apply the temp offset and check that it results in all values being positive
            tempValue = tempValue + CALIBRATION_TEMPERATURE_OFFSET;
            if (tempValue < 0) { tempValue = 0; }

            
            ((uint16_t*)pnt_TargetTable_values)[x] = tempValue; //Both temp tables have 16-bit values
            pnt_TargetTable_bins[x] = (x * 32U);
          }
          //Update the CRC
          calibrationCRC = CRC32.crc32(&serialPayload[7], 64);
          storeCalibrationCRC32(CLT_CALIBRATION_PAGE, calibrationCRC);

          writeCalibration();
          sendSerialReturnCode(SERIAL_RC_OK);
        }
        else { sendSerialReturnCode(SERIAL_RC_RANGE_ERR); }
      }
      else
      {
        sendSerialReturnCode(SERIAL_RC_RANGE_ERR);
      }
      break;
    }

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

    case 'w':
    {
#ifdef RTC_ENABLED
      uint8_t cmd = serialPayload[2];
      uint16_t SD_arg1 = word(serialPayload[3], serialPayload[4]);
      uint16_t SD_arg2 = word(serialPayload[5], serialPayload[6]);
      if(cmd == SD_READWRITE_PAGE)
        { 
          if((SD_arg1 == SD_WRITE_DO_ARG1) && (SD_arg2 == SD_WRITE_DO_ARG2))
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
            uint8_t command = serialPayload[7];
            if(command == 2) { endSDLogging(); manualLogActive = false; }
            else if(command == 3) { beginSDLogging(); manualLogActive = true; }
            else if(command == 4) { setTS_SD_status(); }
            //else if(command == 5) { initSD(); }
            
            sendSerialReturnCode(SERIAL_RC_OK);
          }
          else if((SD_arg1 == SD_WRITE_DIR_ARG1) && (SD_arg2 == SD_WRITE_DIR_ARG2))
          {
            //Begin SD directory read. Value in payload represents the directory chunk to read
            //Directory chunks are each 16 files long
            SDcurrentDirChunk = word(serialPayload[7], serialPayload[8]);
            sendSerialReturnCode(SERIAL_RC_OK);
          }
          else if((SD_arg1 == SD_WRITE_SEC_ARG1) && (SD_arg2 == SD_WRITE_SEC_ARG2))
          {
            //SD write sector command
          }
          else if((SD_arg1 == SD_ERASEFILE_ARG1) && (SD_arg2 == SD_ERASEFILE_ARG2))
          {
            //Erase file command
            //We just need the 4 ASCII characters of the file name
            char log1 = serialPayload[7];
            char log2 = serialPayload[8];
            char log3 = serialPayload[9];
            char log4 = serialPayload[10];

            deleteLogFile(log1, log2, log3, log4);
            sendSerialReturnCode(SERIAL_RC_OK);
          }
          else if((SD_arg1 == SD_SPD_TEST_ARG1) && (SD_arg2 == SD_SPD_TEST_ARG2))
          {
            //Perform a speed test on the SD card
            //First 4 bytes are the sector number to write to
            uint32_t sector;
            uint8_t sector1 = serialPayload[7];
            uint8_t sector2 = serialPayload[8];
            uint8_t sector3 = serialPayload[9];
            uint8_t sector4 = serialPayload[10];
            sector = (sector1 << 24) | (sector2 << 16) | (sector3 << 8) | sector4;


            //Last 4 bytes are the number of sectors to test
            uint32_t testSize;
            uint8_t testSize1 = serialPayload[11];
            uint8_t testSize2 = serialPayload[12];
            uint8_t testSize3 = serialPayload[13];
            uint8_t testSize4 = serialPayload[14];
            testSize = (testSize1 << 24) | (testSize2 << 16) | (testSize3 << 8) | testSize4; 

            sendSerialReturnCode(SERIAL_RC_OK);

          }
          else if((SD_arg1 == SD_WRITE_COMP_ARG1) && (SD_arg2 == SD_WRITE_COMP_ARG2))
          {
            //Prepare to read a 2024 byte chunk of data from the SD card
            uint8_t sector1 = serialPayload[7];
            uint8_t sector2 = serialPayload[8];
            uint8_t sector3 = serialPayload[9];
            uint8_t sector4 = serialPayload[10];
            //SDreadStartSector = (sector1 << 24) | (sector2 << 16) | (sector3 << 8) | sector4;
            SDreadStartSector = (sector4 << 24) | (sector3 << 16) | (sector2 << 8) | sector1;
            //SDreadStartSector = sector4 | (sector3 << 8) | (sector2 << 16) | (sector1 << 24);

            //Next 4 bytes are the number of sectors to write
            uint8_t sectorCount1 = serialPayload[11];
            uint8_t sectorCount2 = serialPayload[12];
            uint8_t sectorCount3 = serialPayload[13];
            uint8_t sectorCount4 = serialPayload[14];
            SDreadNumSectors = (sectorCount1 << 24) | (sectorCount2 << 16) | (sectorCount3 << 8) | sectorCount4;

            //Reset the sector counter
            SDreadCompletedSectors = 0;

            sendSerialReturnCode(SERIAL_RC_OK);
          }
        }
        else if(cmd == SD_RTC_PAGE)
        {
          cmdPending = false;
          //Used for setting RTC settings
          if((SD_arg1 == SD_RTC_WRITE_ARG1) && (SD_arg2 == SD_RTC_WRITE_ARG2))
          {
            //Set the RTC date/time
            byte second = serialPayload[7];
            byte minute = serialPayload[8];
            byte hour = serialPayload[9];
            //byte dow = serialPayload[10]; //Not used
            byte day = serialPayload[11];
            byte month = serialPayload[12];
            uint16_t year = word(serialPayload[13], serialPayload[14]);
            rtc_setTime(second, minute, hour, day, month, year);
            sendSerialReturnCode(SERIAL_RC_OK);
          }
        }
#endif
      break;
    }

    default:
      //Unknown command
      sendSerialReturnCode(SERIAL_RC_UKWN_ERR);
      break;
  }
}

/** Send a status record back to tuning/logging SW.
 * This will "live" information from @ref currentStatus struct.
 * @param offset - Start field number
 * @param packetLength - Length of actual message (after possible ack/confirm headers)
 * E.g. tuning sw command 'A' (Send all values) will send data from field number 0, LOG_ENTRY_SIZE fields.
 */
//void sendValues(int packetlength, byte portNum)
void generateLiveValues(uint16_t offset, uint16_t packetLength)
{  
  if(requestCount == 0) { currentStatus.secl = 0; }
  requestCount++;

  currentStatus.spark ^= (-currentStatus.hasSync ^ currentStatus.spark) & (1U << BIT_SPARK_SYNC); //Set the sync bit of the Spark variable to match the hasSync variable

  serialPayload[0] = SERIAL_RC_OK;
  for(byte x=0; x<packetLength; x++)
  {
    serialPayload[x+1] = getTSLogEntry(offset+x); 
  }
  // Reset any flags that are being used to trigger page refreshes
  BIT_CLEAR(currentStatus.status3, BIT_STATUS3_VSS_REFRESH);

}

namespace 
{

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

}

/** 
 * 
*/
void sendToothLog(byte startOffset)
{
  //We need TOOTH_LOG_SIZE number of records to send to TunerStudio. If there aren't that many in the buffer then we just return and wait for the next call
  if (BIT_CHECK(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY)) //Sanity check. Flagging system means this should always be true
  {
    uint32_t CRC32_val = 0;
    if(startOffset == 0)
    {
      //Transmit the size of the packet
      uint16_t totalPayloadLength = (TOOTH_LOG_SIZE * 4) + 1; //Size of the tooth log (uint32_t values) plus the return code
      Serial.write(totalPayloadLength >> 8);
      Serial.write(totalPayloadLength);

      //Begin new CRC hash
      const uint8_t returnCode = SERIAL_RC_OK;
      CRC32_val = CRC32_serial.crc32(&returnCode, 1, false);

      //Send the return code
      Serial.write(returnCode);
    }
    
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

      //Transmit the tooth time
      uint32_t tempToothHistory = toothHistory[x];
      uint8_t toothHistory_1 = ((tempToothHistory >> 24) & 255);
      uint8_t toothHistory_2 = ((tempToothHistory >> 16) & 255);
      uint8_t toothHistory_3 = ((tempToothHistory >> 8) & 255);
      uint8_t toothHistory_4 = ((tempToothHistory) & 255);
      Serial.write(toothHistory_1);
      Serial.write(toothHistory_2);
      Serial.write(toothHistory_3);
      Serial.write(toothHistory_4);

      //Update the CRC
      CRC32_val = CRC32_serial.crc32_upd(&toothHistory_1, 1, false);
      CRC32_val = CRC32_serial.crc32_upd(&toothHistory_2, 1, false);
      CRC32_val = CRC32_serial.crc32_upd(&toothHistory_3, 1, false);
      CRC32_val = CRC32_serial.crc32_upd(&toothHistory_4, 1, false);
    }
    BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
    cmdPending = false;
    toothLogSendInProgress = false;
    toothHistoryIndex = 0;

    //Apply the CRC reflection
    CRC32_val = ~CRC32_val;

    //Send the CRC
    Serial.write( ((CRC32_val >> 24) & 255) );
    Serial.write( ((CRC32_val >> 16) & 255) );
    Serial.write( ((CRC32_val >> 8) & 255) );
    Serial.write( (CRC32_val & 255) );
  }
  else 
  { 
    sendSerialReturnCode(SERIAL_RC_BUSY_ERR);
    cmdPending = false; 
    toothLogSendInProgress = false;
  } 
}

void sendCompositeLog(byte startOffset)
{
  if ( (BIT_CHECK(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY)) || (compositeLogSendInProgress == true) ) //Sanity check. Flagging system means this should always be true
  {
    BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
    uint32_t CRC32_val = 0;
    if(startOffset == 0)
    { 
      inProgressCompositeTime = 0; 

      //Transmit the size of the packet
      uint16_t totalPayloadLength = (TOOTH_LOG_SIZE * 5) + 1; //Size of the tooth log (1x uint32_t + 1x uint8_t values) plus the return code
      Serial.write(totalPayloadLength >> 8);
      Serial.write(totalPayloadLength);

      //Begin new CRC hash
      const uint8_t returnCode = SERIAL_RC_OK;
      CRC32_val = CRC32_serial.crc32(&returnCode, 1, false);

      //Send the return code
      Serial.write(returnCode);
    }
    for (int x = startOffset; x < TOOTH_LOG_SIZE; x++)
    {
      //Check whether the tx buffer still has space
      if(Serial.availableForWrite() < 5) 
      { 
        //tx buffer is full. Store the current state so it can be resumed later
        inProgressOffset = x;
        compositeLogSendInProgress = true;
        
        return;
      }

      inProgressCompositeTime = toothHistory[x]; //This combined runtime (in us) that the log was going for by this record
      uint8_t inProgressCompositeTime_1 = (inProgressCompositeTime >> 24) & 255;
      uint8_t inProgressCompositeTime_2 = (inProgressCompositeTime >> 16) & 255;
      uint8_t inProgressCompositeTime_3 = (inProgressCompositeTime >> 8) & 255;
      uint8_t inProgressCompositeTime_4 = (inProgressCompositeTime) & 255;

      //Transmit the tooth time
      Serial.write(inProgressCompositeTime_1);
      Serial.write(inProgressCompositeTime_2);
      Serial.write(inProgressCompositeTime_3);
      Serial.write(inProgressCompositeTime_4);

      //Update the CRC
      CRC32_val = CRC32_serial.crc32_upd(&inProgressCompositeTime_1, 1, false);
      CRC32_val = CRC32_serial.crc32_upd(&inProgressCompositeTime_2, 1, false);
      CRC32_val = CRC32_serial.crc32_upd(&inProgressCompositeTime_3, 1, false);
      CRC32_val = CRC32_serial.crc32_upd(&inProgressCompositeTime_4, 1, false);

      //The status byte (Indicates the trigger edge, whether it was a pri/sec pulse, the sync status)
      uint8_t statusByte = compositeLogHistory[x];
      Serial.write(statusByte);

      //Update the CRC with the status byte
      CRC32_val = CRC32_serial.crc32_upd(&statusByte, 1, false);
    }
    BIT_CLEAR(currentStatus.status1, BIT_STATUS1_TOOTHLOG1READY);
    toothHistoryIndex = 0;
    cmdPending = false;
    compositeLogSendInProgress = false;
    inProgressCompositeTime = 0;

    //Apply the CRC reflection
    CRC32_val = ~CRC32_val;

    //Send the CRC
    Serial.write( ((CRC32_val >> 24) & 255) );
    Serial.write( ((CRC32_val >> 16) & 255) );
    Serial.write( ((CRC32_val >> 8) & 255) );
    Serial.write( (CRC32_val & 255) );
  }
  else 
  { 
    sendSerialReturnCode(SERIAL_RC_BUSY_ERR);
    cmdPending = false; 
    compositeLogSendInProgress = false;
  } 
}