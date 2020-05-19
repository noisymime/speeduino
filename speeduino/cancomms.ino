/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
can_comms was originally contributed by Darren Siepka
*/

/*
secondserial_command is called when a command is received from the secondary serial port
It parses the command and calls the relevant function.

can_command is called when a command is recieved by the onboard/attached canbus module
It parses the command and calls the relevant function.

sendcancommand is called when a command is to be sent either to serial3 
,to the external Can interface, or to the onboard/attached can interface
*/
#include "globals.h"
#include "cancomms.h"
#include "maths.h"
#include "errors.h"
#include "utils.h"

void secondserial_Command()
{
  if (! canCmdPending) {  currentsecondserialCommand = CANSerial.read();  }

  switch (currentsecondserialCommand)
  {
    case 'A': // sends the bytes of realtime values from the OLD CAN list
        sendcanValues(0, CAN_PACKET_SIZE, 0x31, 1); //send values to serial3
        break;

    case 'G': // this is the reply command sent by the Can interface
       byte destcaninchannel;
      if (CANSerial.available() >= 9)
      {
        canCmdPending = false;
        cancmdfail = CANSerial.read();        //0 == fail,  1 == good.
        destcaninchannel = CANSerial.read();  // the input channel that requested the data value
        if (cancmdfail != 0)
           {                                 // read all 8 bytes of data.
            for (byte Gx = 0; Gx < 8; Gx++) // first two are the can address the data is from. next two are the can address the data is for.then next 1 or two bytes of data
              {
                Gdata[Gx] = CANSerial.read();
              }
            Glow = Gdata[(configPage9.caninput_source_start_byte[destcaninchannel]&7)];
            if ((BIT_CHECK(configPage9.caninput_source_num_bytes,destcaninchannel) > 0))  //if true then num bytes is 2
               {
                if ((configPage9.caninput_source_start_byte[destcaninchannel]&7) < 8)   //you cant have a 2 byte value starting at byte 7(8 on the list)
                   {
                    Ghigh = Gdata[((configPage9.caninput_source_start_byte[destcaninchannel]&7)+1)];
                   }
            else{Ghigh = 0;}
               }
          else
               {
                 Ghigh = 0;
               }

          currentStatus.canin[destcaninchannel] = (Ghigh<<8) | Glow;
        }

        else{}  //continue as command request failed and/or data/device was not available

      }
      else
      {
        canCmdPending = true;
      }
      
        break;

    case 'k':   //placeholder for new can interface (toucan etc) commands

        break;
        
    case 'L':
        uint8_t Llength;
        while (CANSerial.available() == 0) { }
        canlisten = CANSerial.read();

        if (canlisten == 0)
         {
          //command request failed and/or data/device was not available
          break;
         }

         while (CANSerial.available() == 0) { }
         Llength= CANSerial.read();              // next the number of bytes expected value

         for (uint8_t Lcount = 0; Lcount <Llength ;Lcount++)
         {
           while (CANSerial.available() == 0){}
           // receive all x bytes into "Lbuffer"
           Lbuffer[Lcount] = CANSerial.read();
         }
         break;

    case 'n': // sends the bytes of realtime values from the NEW CAN list
        sendcanValues(0, NEW_CAN_PACKET_SIZE, 0x32, 1); //send values to serial3
        break;

    case 'p':
      //6 bytes required:
      //2 - Page identifier
      //2 - offset
      //2 - Length
      if(CANSerial.available() >= 6)
      {
        byte offset1, offset2, length1, length2;
        int length;
        byte tempPage;

        CANSerial.read(); // First byte of the page identifier can be ignored. It's always 0
        tempPage = CANSerial.read();
        offset1 = CANSerial.read();
        offset2 = CANSerial.read();
        valueOffset = word(offset2, offset1);
        length1 = CANSerial.read();
        length2 = CANSerial.read();
        length = word(length2, length1);

        // confirm command type
        CANSerial.write("p");

        for(int i = 0; i < length; i++)
        {
          CANSerial.write(getPageValue(tempPage, valueOffset + i));
        }
      }
      break;

    case 'r': //New format for the optimised OutputChannels
      byte Cmd;
      if (CANSerial.available() >= 6)
      {
        CANSerial.read(); //Read the $tsCanId
        Cmd = CANSerial.read();

        uint16_t offset, length;
        if( (Cmd == 0x30) || ( (Cmd >= 0x40) && (Cmd <0x50) ) ) //Send output channels command 0x30 is 48dec, 0x40(64dec)-0x4F(79dec) are external can request
        {
          byte tmp;
          tmp = CANSerial.read();
          offset = word(CANSerial.read(), tmp);
          tmp = CANSerial.read();
          length = word(CANSerial.read(), tmp);
          sendcanValues(offset, length,Cmd, 1);
          canCmdPending = false;
//Serial.print(Cmd);
        }
        else
        {
          //No other r/ commands should be called
        }
      }
      else
      {
        canCmdPending = true;
      }
      
      break;

    case 's': // send the "a" stream code version
      CANSerial.print(F("Speeduino csx02019.8"));
       break;

    case 'S': // send code version
      CANSerial.print(F("Speeduino 2019.08-ser"));
      break;
      
    case 'Q': // send code version
       //for (unsigned int revn = 0; revn < sizeof( TSfirmwareVersion) - 1; revn++)
       for (unsigned int revn = 0; revn < 10 - 1; revn++)
       {
         CANSerial.write( TSfirmwareVersion[revn]);
       }
       //Serial3.print("speeduino 201609-dev");
       break;

    case 'Z': //dev use
       break;

    default:
       break;
  }
}
void sendcanValues(uint16_t offset, uint16_t packetLength, byte cmd, byte portType)
{
  byte fullStatus[NEW_CAN_PACKET_SIZE];    // this must be set to the maximum number of data fullstatus must read in

    //CAN serial
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)|| defined(CORE_STM32) || defined (CORE_TEENSY) //ATmega2561 does not have Serial3
      if (cmd == 0x30) 
          {
           CANSerial.write("r");         //confirm cmd type
           CANSerial.write(cmd);
          }
        else if (cmd == 0x31)
          {
           CANSerial.write("A");         // confirm command type   
          }
        else if (cmd == 0x32)
          {
           CANSerial.write("n");                       // confirm command type
           CANSerial.write(cmd);                       // send command type  , 0x32 (dec50) is ascii '0'
           CANSerial.write(NEW_CAN_PACKET_SIZE);       // send the packet size the receiving device should expect.
          }
    #endif

  currentStatus.spark ^= (-currentStatus.hasSync ^ currentStatus.spark) & (1U << BIT_SPARK_SYNC); //Set the sync bit of the Spark variable to match the hasSync variable

  fullStatus[0] = currentStatus.secl; //secl is simply a counter that increments each second. Used to track unexpected resets (Which will reset this count to 0)
  fullStatus[1] = currentStatus.status1; //status1 Bitfield
  fullStatus[2] = currentStatus.engine; //Engine Status Bitfield
  fullStatus[3] = (byte)(divu100(currentStatus.dwell)); //Dwell in ms * 10
  fullStatus[4] = lowByte(currentStatus.MAP); //2 bytes for MAP
  fullStatus[5] = highByte(currentStatus.MAP);
  fullStatus[6] = (byte)(currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET); //mat
  fullStatus[7] = (byte)(currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET); //Coolant ADC
  fullStatus[8] = currentStatus.batCorrection; //Battery voltage correction (%)
  fullStatus[9] = currentStatus.battery10; //battery voltage
  fullStatus[10] = currentStatus.O2; //O2
  fullStatus[11] = currentStatus.egoCorrection; //Exhaust gas correction (%)
  fullStatus[12] = currentStatus.iatCorrection; //Air temperature Correction (%)
  fullStatus[13] = currentStatus.wueCorrection; //Warmup enrichment (%)
  fullStatus[14] = lowByte(currentStatus.RPM); //rpm HB
  fullStatus[15] = highByte(currentStatus.RPM); //rpm LB
  fullStatus[16] = currentStatus.AEamount; //acceleration enrichment (%)
  fullStatus[17] = currentStatus.corrections; //Total GammaE (%)
  fullStatus[18] = currentStatus.VE; //Current VE 1 (%)
  fullStatus[19] = currentStatus.afrTarget;
  fullStatus[20] = lowByte(currentStatus.PW1); //Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[21] = highByte(currentStatus.PW1); //Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[22] = currentStatus.tpsDOT; //TPS DOT
  fullStatus[23] = currentStatus.advance;
  fullStatus[24] = currentStatus.TPS; // TPS (0% to 100%)
  //Need to split the int loopsPerSecond value into 2 bytes
  fullStatus[25] = lowByte(currentStatus.loopsPerSecond);
  fullStatus[26] = highByte(currentStatus.loopsPerSecond);

  //The following can be used to show the amount of free memory
  currentStatus.freeRAM = freeRam();
  fullStatus[27] = lowByte(currentStatus.freeRAM); //(byte)((currentStatus.loopsPerSecond >> 8) & 0xFF);
  fullStatus[28] = highByte(currentStatus.freeRAM);

  fullStatus[29] = (byte)(currentStatus.boostTarget >> 1); //Divide boost target by 2 to fit in a byte
  fullStatus[30] = (byte)(currentStatus.boostDuty / 100);
  fullStatus[31] = currentStatus.spark; //Spark related bitfield

  //rpmDOT must be sent as a signed integer
  fullStatus[32] = lowByte(currentStatus.rpmDOT);
  fullStatus[33] = highByte(currentStatus.rpmDOT);

  fullStatus[34] = currentStatus.ethanolPct; //Flex sensor value (or 0 if not used)
  fullStatus[35] = currentStatus.flexCorrection; //Flex fuel correction (% above or below 100)
  fullStatus[36] = currentStatus.flexIgnCorrection; //Ignition correction (Increased degrees of advance) for flex fuel

  fullStatus[37] = currentStatus.idleLoad;
  fullStatus[38] = currentStatus.testOutputs;

  fullStatus[39] = currentStatus.O2_2; //O2
  fullStatus[40] = currentStatus.baro; //Barometer value

  fullStatus[41] = lowByte(currentStatus.canin[0]);
  fullStatus[42] = highByte(currentStatus.canin[0]);
  fullStatus[43] = lowByte(currentStatus.canin[1]);
  fullStatus[44] = highByte(currentStatus.canin[1]);
  fullStatus[45] = lowByte(currentStatus.canin[2]);
  fullStatus[46] = highByte(currentStatus.canin[2]);
  fullStatus[47] = lowByte(currentStatus.canin[3]);
  fullStatus[48] = highByte(currentStatus.canin[3]);
  fullStatus[49] = lowByte(currentStatus.canin[4]);
  fullStatus[50] = highByte(currentStatus.canin[4]);
  fullStatus[51] = lowByte(currentStatus.canin[5]);
  fullStatus[52] = highByte(currentStatus.canin[5]);
  fullStatus[53] = lowByte(currentStatus.canin[6]);
  fullStatus[54] = highByte(currentStatus.canin[6]);
  fullStatus[55] = lowByte(currentStatus.canin[7]);
  fullStatus[56] = highByte(currentStatus.canin[7]);
  fullStatus[57] = lowByte(currentStatus.canin[8]);
  fullStatus[58] = highByte(currentStatus.canin[8]);
  fullStatus[59] = lowByte(currentStatus.canin[9]);
  fullStatus[60] = highByte(currentStatus.canin[9]);
  fullStatus[61] = lowByte(currentStatus.canin[10]);
  fullStatus[62] = highByte(currentStatus.canin[10]);
  fullStatus[63] = lowByte(currentStatus.canin[11]);
  fullStatus[64] = highByte(currentStatus.canin[11]);
  fullStatus[65] = lowByte(currentStatus.canin[12]);
  fullStatus[66] = highByte(currentStatus.canin[12]);
  fullStatus[67] = lowByte(currentStatus.canin[13]);
  fullStatus[68] = highByte(currentStatus.canin[13]);
  fullStatus[69] = lowByte(currentStatus.canin[14]);
  fullStatus[70] = highByte(currentStatus.canin[14]);
  fullStatus[71] = lowByte(currentStatus.canin[15]);
  fullStatus[72] = highByte(currentStatus.canin[15]);

  fullStatus[73] = currentStatus.tpsADC;
  fullStatus[74] = getNextError();

  for(byte x=0; x<packetLength; x++)
  {
    if (portType == 1){ CANSerial.write(fullStatus[offset+x]); }
    else if (portType == 2)
            {
              //sendto canbus transmit routine
            }
  }

}

void can_Command()
{
 //int currentcanCommand = inMsg.id;
 #if defined(CORE_TEENSY)
       currentStatus.canin[12] = (inMsg.id);
 if ((inMsg.id == configPage9.obd_address)  || (inMsg.id == 0x7DF))      
  {
    // The address is the speeduino specific ecu canbus address 
    // or the 0x7df(2015 dec) broadcast address
    if (inMsg.buf[1] == 0x01)
      {
        // PID mode 0 , realtime data stream
        obd_response(inMsg.buf[1], inMsg.buf[2], 0);     // get the obd response based on the data in byte2
        outMsg.id = (0x7E8);//(configPage9.obd_address+8);
        Can0.write(outMsg);       // send the 8 bytes of obd data   
      }
    if (inMsg.buf[1] == 0x22)
      {
        // PID mode 22h , custom mode , non standard data
        obd_response(inMsg.buf[1], inMsg.buf[2], inMsg.buf[3]);     // get the obd response based on the data in byte2
        outMsg.id = (0x7E8); //configPage9.obd_address+8);
        Can0.write(outMsg);       // send the 8 bytes of obd data
      }
  }
 if (inMsg.id == configPage9.obd_address)      
  {
    // The address is only the speeduino specific ecu canbus address    
    if (inMsg.buf[1] == 0x09)
      {
       // PID mode 9 , vehicle information request
       if (inMsg.buf[2] == 02)
         {
          //send the VIN number , 17 char long VIN sent in 5 messages.
         }
      else if (inMsg.buf[2] == 0x0A)
         {
          //code 20: send 20 ascii characters with ECU name , "ECU -SpeeduinoXXXXXX" , change the XXXXXX ONLY as required.  
         }
      }
  }
#endif  
}  
    
// this routine sends a request(either "0" for a "G" , "1" for a "L" , "2" for a "R" to the Can interface or "3" sends the request via the actual local canbus
void sendCancommand(uint8_t cmdtype, uint16_t canaddress, uint8_t candata1, uint8_t candata2, uint16_t sourcecanAddress)
{
    switch (cmdtype)
    {
      case 0:
        CANSerial.print("G");
        CANSerial.write(canaddress);  //tscanid of speeduino device
        CANSerial.write(candata1);    // table id
        CANSerial.write(candata2);    //table memory offset
        break;

      case 1:                      //send request to listen for a can message
        CANSerial.print("L");
        CANSerial.write(canaddress);  //11 bit canaddress of device to listen for
        break;

     case 2:                                          // requests via serial3
        CANSerial.print("R");                         //send "R" to request data from the sourcecanAddress whos value is sent next
        CANSerial.write(candata1);                    //the currentStatus.current_caninchannel
        CANSerial.write(lowByte(sourcecanAddress) );       //send lsb first
        CANSerial.write(highByte(sourcecanAddress) );
        break;

     case 3:
        //send to truecan send routine
        //canaddress == speeduino canid, candata1 == canin channel dest, paramgroup == can address  to request from
        //This section is to be moved to the correct can output routine later
        #if defined(CORE_TEENSY) //Scope guarding this for now, but this needs a bit of a rethink for how it can be handled better across multiple archs
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
        Can0.write(outMsg);
        #endif
        break;

     default:
        break;
    }
}

// This routine builds the realtime data into packets that the obd requesting device can understand. This is only used by teensy and stm32 with onboard canbus
void obd_response(uint8_t thePIDmode, uint8_t therequestedPIDlow, uint8_t therequestedPIDhigh)
{

// #if defined(CORE_STM32) || defined(CORE_TEENSY)
#if defined(CORE_TEENSY)
//only build the PID if the mcu has onboard/attached can 

  uint16_t obdcalcA;    //used in obd calcs
  uint16_t obdcalcB;    //used in obd calcs 
  uint16_t obdcalcC;    //used in obd calcs 
  uint16_t obdcalcD;    //used in obd calcs
  uint32_t obdcalcE32;    //used in calcs 
  uint32_t obdcalcF32;    //used in calcs 
  uint16_t obdcalcG16;    //used in calcs
  uint16_t obdcalcH16;    //used in calcs  

  outMsg.len = 8;

#endif
}
