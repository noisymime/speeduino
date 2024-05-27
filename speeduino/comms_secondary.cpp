/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
can_comms was originally contributed by Darren Siepka
*/

/*
secondserial_command is called when a command is received from the secondary serial port
It parses the command and calls the relevant function.

can_command is called when a command is received by the onboard/attached canbus module
It parses the command and calls the relevant function.

sendcancommand is called when a command is to be sent either to serial3 
,to the external Can interface, or to the onboard/attached can interface
*/
#include "globals.h"
#include "comms_secondary.h"
#include "comms_CAN.h"
#include "maths.h"
#include "errors.h"
#include "utilities.h"
#include "comms_legacy.h"
#include "logger.h"
#include "page_crc.h"
#include "board_definition.h"

uint8_t currentSecondaryCommand;
SECONDARY_SERIAL_T* pSecondarySerial;

void secondserial_Command(void)
{
  #if defined(secondarySerial_AVAILABLE)
  if ( serialSecondaryStatusFlag == SERIAL_INACTIVE )  { currentSecondaryCommand = secondarySerial.read(); }

  switch (currentSecondaryCommand)
  {
    case 'A': 
      // sends a fixed 75 bytes of data. Used by Real Dash (Among others)
      if(configPage9.secondarySerialProtocol == SECONDARY_SERIAL_PROTO_GENERIC_FIXED) { sendValues(0, CAN_PACKET_SIZE, 0x31, secondarySerial, serialSecondaryStatusFlag, &getLegacySecondarySerialLogEntry); } // Send values using the legacy fixed byte order
      else { sendValues(0, CAN_PACKET_SIZE, 0x31, secondarySerial, serialSecondaryStatusFlag); } //send values to serial3 using the order in the ini file
      break;

    case 'b': // New EEPROM burn command to only burn a single page at a time
      legacySerialHandler(currentSecondaryCommand, secondarySerial, serialSecondaryStatusFlag);
      break;

    case 'B': // AS above but for the serial compatibility mode. 
      BIT_SET(currentStatus.status4, BIT_STATUS4_COMMS_COMPAT); //Force the compat mode
      legacySerialHandler(currentSecondaryCommand, secondarySerial, serialSecondaryStatusFlag);
      break;

    case 'd': // Send a CRC32 hash of a given page
      legacySerialHandler(currentSecondaryCommand, secondarySerial, serialSecondaryStatusFlag);
      break;

    case 'G': // this is the reply command sent by the Can interface
      serialSecondaryStatusFlag = SERIAL_COMMAND_INPROGRESS_LEGACY;
      byte destcaninchannel;
      if (secondarySerial.available() >= 9)
      {
        serialSecondaryStatusFlag = SERIAL_INACTIVE;
        uint8_t cmdSuccessful = secondarySerial.read();        //0 == fail,  1 == good.
        destcaninchannel = secondarySerial.read();  // the input channel that requested the data value
        if (cmdSuccessful != 0)
        {                                 // read all 8 bytes of data.
          uint8_t Gdata[9];
          uint8_t Glow, Ghigh;

          for (byte Gx = 0; Gx < 8; Gx++) // first two are the can address the data is from. next two are the can address the data is for.then next 1 or two bytes of data
          {
            Gdata[Gx] = secondarySerial.read();
          }
          Glow = Gdata[(configPage9.caninput_source_start_byte[destcaninchannel]&7)];
          if ((BIT_CHECK(configPage9.caninput_source_num_bytes,destcaninchannel) > 0))  //if true then num bytes is 2
          {
            if ((configPage9.caninput_source_start_byte[destcaninchannel]&7) < 8)   //you can't have a 2 byte value starting at byte 7(8 on the list)
            {
              Ghigh = Gdata[((configPage9.caninput_source_start_byte[destcaninchannel]&7)+1)];
            }
            else { Ghigh = 0; }
          }
        else
        {
          Ghigh = 0;
        }

        currentStatus.canin[destcaninchannel] = (Ghigh<<8) | Glow;
      }

        else{}  //continue as command request failed and/or data/device was not available

      }
      break;

    case 'k':   //placeholder for new can interface (toucan etc) commands

        break;
        
    case 'L':
    {
      //uint8_t Llength;
      while (secondarySerial.available() == 0) { }
      uint8_t canListen = secondarySerial.read();

      if (canListen == 0)
      {
        //command request failed and/or data/device was not available
        break;
      }

      while (secondarySerial.available() == 0) { }
      /*
      Unclear what the below is trying to achieve. Commenting out for now to avoid compiler warnings for unused variables
      Llength = secondarySerial.read();             // next the number of bytes expected value
      uint8_t Lbuffer[8];                     //8 byte buffer to store incoming can data

      for (uint8_t Lcount = 0; Lcount <Llength ;Lcount++)
      {
        while (secondarySerial.available() == 0){}
        // receive all x bytes into "Lbuffer"
        Lbuffer[Lcount] = secondarySerial.read();
      }
      */
      break;
    }

    case 'M':
      legacySerialHandler(currentSecondaryCommand, secondarySerial, serialSecondaryStatusFlag);
      break;
      
    case 'n': // sends the bytes of realtime values from the NEW CAN list
      //sendValues(0, NEW_CAN_PACKET_SIZE, 0x32, secondarySerial, serialSecondaryStatusFlag); //send values to serial3
      if(configPage9.secondarySerialProtocol == SECONDARY_SERIAL_PROTO_GENERIC_FIXED) { sendValues(0, NEW_CAN_PACKET_SIZE, 0x32, secondarySerial, serialSecondaryStatusFlag, &getLegacySecondarySerialLogEntry); } // Send values using the legacy fixed byte order
      else { sendValues(0, NEW_CAN_PACKET_SIZE, 0x32, secondarySerial, serialSecondaryStatusFlag); } //send values to serial3 using the order in the ini file
      break;

    case 'p':
      legacySerialHandler(currentSecondaryCommand, secondarySerial, serialSecondaryStatusFlag);
      break;

    case 'Q': // send code version
      legacySerialHandler(currentSecondaryCommand, secondarySerial, serialSecondaryStatusFlag);
       break;

    case 'r': //New format for the optimised OutputChannels over CAN
      legacySerialHandler(currentSecondaryCommand, secondarySerial, serialSecondaryStatusFlag);
      break;

    case 's': // send the "a" stream code version
      secondarySerial.print(F("Speeduino csx02019.8"));
      break;

    case 'S': // send code version
      if(configPage9.secondarySerialProtocol == SECONDARY_SERIAL_PROTO_MSDROID) { legacySerialHandler('Q', secondarySerial, serialSecondaryStatusFlag); } //Note 'Q', this is a workaround for msDroid
      else { legacySerialHandler(currentSecondaryCommand, secondarySerial, serialSecondaryStatusFlag); }
      
      break;

    case 'Z': //dev use
       break;

    default:
       break;
  }
  #endif
} 
    
// this routine sends a request(either "0" for a "G" , "1" for a "L" , "2" for a "R" to the Can interface or "3" sends the request via the actual local canbus
void sendCancommand(uint8_t cmdtype, uint16_t canaddress, uint8_t candata1, uint8_t candata2, uint16_t sourcecanAddress)
{
#if defined(secondarySerial_AVAILABLE)
    switch (cmdtype)
    {
      case 0:
        secondarySerial.print("G");
        secondarySerial.write(canaddress);  //tscanid of speeduino device
        secondarySerial.write(candata1);    // table id
        secondarySerial.write(candata2);    //table memory offset
        break;

      case 1:                      //send request to listen for a can message
        secondarySerial.print("L");
        secondarySerial.write(canaddress);  //11 bit canaddress of device to listen for
        break;

     case 2:                                          // requests via serial3
        secondarySerial.print("R");                         //send "R" to request data from the sourcecanAddress whose value is sent next
        secondarySerial.write(candata1);                    //the currentStatus.current_caninchannel
        secondarySerial.write(lowByte(sourcecanAddress) );       //send lsb first
        secondarySerial.write(highByte(sourcecanAddress) );
        break;

     case 3:
        //send to truecan send routine
        //canaddress == speeduino canid, candata1 == canin channel dest, paramgroup == can address  to request from
        //This section is to be moved to the correct can output routine later
        #if defined(NATIVE_CAN_AVAILABLE)
        outMsg.id = (canaddress);
        outMsg.len = 8;
        outMsg.buf[0] = 0x0B ;  //11;   
        outMsg.buf[1] = 0x15;
        outMsg.buf[2] = candata1;
        outMsg.buf[3] = 0x24;
        outMsg.buf[4] = 0x7F;
        outMsg.buf[5] = 0x70;
        outMsg.buf[6] = 0x9E;
        outMsg.buf[7] = 0x4D;
        CAN_write();
        #endif
        break;

     default:
        break;
    }
#else
  UNUSED(cmdtype);
  UNUSED(canaddress);
  UNUSED(candata1);
  UNUSED(candata2);
  UNUSED(sourcecanAddress);
#endif
}
