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
#include "utilities.h"

uint8_t currentsecondserialCommand;
uint8_t currentCanPage = 1;//Not the same as the speeduino config page numbers
uint8_t nCanretry = 0;      //no of retrys
uint8_t cancmdfail = 0;     //command fail yes/no
uint8_t canlisten = 0;
uint8_t Lbuffer[8];         //8 byte buffer to store incomng can data
uint8_t Gdata[9];
uint8_t Glow, Ghigh;
bool canCmdPending = false;

#if ( defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) )
  #define CANSerial_AVAILABLE
  HardwareSerial &CANSerial = Serial3;
#elif defined(CORE_STM32)
  #define CANSerial_AVAILABLE
  #ifndef Serial2
    #define Serial2 Serial1
  #endif
  #if defined(STM32GENERIC) // STM32GENERIC core
    SerialUART &CANSerial = Serial2;
  #else //libmaple core aka STM32DUINO
    HardwareSerial &CANSerial = Serial2;
  #endif
#elif defined(CORE_TEENSY)
  #define CANSerial_AVAILABLE
  HardwareSerial &CANSerial = Serial2;
#endif

void secondserial_Command()
{
  #if defined(CANSerial_AVAILABLE)
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

    case 'r': //New format for the optimised OutputChannels over CAN
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
  #endif
}
void sendcanValues(uint16_t offset, uint16_t packetLength, byte cmd, byte portType)
{
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

#if defined(CANSerial_AVAILABLE)
  byte fullStatus[NEW_CAN_PACKET_SIZE];    // this must be set to the maximum number of data fullstatus must read in
  fullStatus[0] = currentStatus.secl; //secl is simply a counter that increments each second. Used to track unexpected resets (Which will reset this count to 0)
  fullStatus[1] = currentStatus.status1; //status1 Bitfield, inj1Status(0), inj2Status(1), inj3Status(2), inj4Status(3), DFCOOn(4), boostCutFuel(5), toothLog1Ready(6), toothLog2Ready(7)
  fullStatus[2] = currentStatus.engine; //Engine Status Bitfield, running(0), crank(1), ase(2), warmup(3), tpsaccaen(4), tpsacden(5), mapaccaen(6), mapaccden(7)
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
  fullStatus[31] = currentStatus.spark; //Spark related bitfield, launchHard(0), launchSoft(1), hardLimitOn(2), softLimitOn(3), boostCutSpark(4), error(5), idleControlOn(6), sync(7)

  //rpmDOT must be sent as a signed integer
  fullStatus[32] = lowByte(currentStatus.rpmDOT);
  fullStatus[33] = highByte(currentStatus.rpmDOT);

  fullStatus[34] = currentStatus.ethanolPct; //Flex sensor value (or 0 if not used)
  fullStatus[35] = currentStatus.flexCorrection; //Flex fuel correction (% above or below 100)
  fullStatus[36] = currentStatus.flexIgnCorrection; //Ignition correction (Increased degrees of advance) for flex fuel

  fullStatus[37] = currentStatus.idleLoad;
  fullStatus[38] = currentStatus.testOutputs; // testEnabled(0), testActive(1)

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
  fullStatus[74] = getNextError(); // errorNum (0:1), currentError(2:7)

  fullStatus[75] = currentStatus.launchCorrection;
  fullStatus[76] = lowByte(currentStatus.PW2); //Pulsewidth 2 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[77] = highByte(currentStatus.PW2); //Pulsewidth 2 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[78] = lowByte(currentStatus.PW3); //Pulsewidth 3 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[79] = highByte(currentStatus.PW3); //Pulsewidth 3 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[80] = lowByte(currentStatus.PW4); //Pulsewidth 4 multiplied by 10 in ms. Have to convert from uS to mS.
  fullStatus[81] = highByte(currentStatus.PW4); //Pulsewidth 4 multiplied by 10 in ms. Have to convert from uS to mS.

  fullStatus[82] = currentStatus.status3; // resentLockOn(0), nitrousOn(1), fuel2Active(2), vssRefresh(3), halfSync(4), nSquirts(6:7)
  fullStatus[83] = currentStatus.engineProtectStatus; //RPM(0), MAP(1), OIL(2), AFR(3), Unused(4:7)
  fullStatus[84] = lowByte(currentStatus.fuelLoad);
  fullStatus[85] = highByte(currentStatus.fuelLoad);
  fullStatus[86] = lowByte(currentStatus.ignLoad);
  fullStatus[87] = highByte(currentStatus.ignLoad);
  fullStatus[88] = lowByte(currentStatus.injAngle); 
  fullStatus[89] = highByte(currentStatus.injAngle); 
  fullStatus[90] = currentStatus.idleDuty;
  fullStatus[91] = currentStatus.CLIdleTarget; //closed loop idle target
  fullStatus[92] = currentStatus.mapDOT; //rate of change of the map 
  fullStatus[93] = (int8_t)currentStatus.vvt1Angle;
  fullStatus[94] = currentStatus.vvt1TargetAngle;
  fullStatus[95] = currentStatus.vvt1Duty;
  fullStatus[96] = lowByte(currentStatus.flexBoostCorrection);
  fullStatus[97] = highByte(currentStatus.flexBoostCorrection);
  fullStatus[98] = currentStatus.baroCorrection;
  fullStatus[99] = currentStatus.ASEValue; //Current ASE (%)
  fullStatus[100] = lowByte(currentStatus.vss); //speed reading from the speed sensor
  fullStatus[101] = highByte(currentStatus.vss);
  fullStatus[102] = currentStatus.gear; 
  fullStatus[103] = currentStatus.fuelPressure;
  fullStatus[104] = currentStatus.oilPressure;
  fullStatus[105] = currentStatus.wmiPW;
  fullStatus[106] = currentStatus.status4; // wmiEmptyBit(0), vvt1Error(1), vvt2Error(2), UnusedBits(3:7)
  fullStatus[107] = (int8_t)currentStatus.vvt2Angle;
  fullStatus[108] = currentStatus.vvt2TargetAngle;
  fullStatus[109] = currentStatus.vvt2Duty;
  fullStatus[110] = currentStatus.outputsStatus;
  fullStatus[111] = (byte)(currentStatus.fuelTemp + CALIBRATION_TEMPERATURE_OFFSET); //Fuel temperature from flex sensor
  fullStatus[112] = currentStatus.fuelTempCorrection; //Fuel temperature Correction (%)
  fullStatus[113] = currentStatus.VE1; //VE 1 (%)
  fullStatus[114] = currentStatus.VE2; //VE 2 (%)
  fullStatus[115] = currentStatus.advance1; //advance 1 
  fullStatus[116] = currentStatus.advance2; //advance 2 
  fullStatus[117] = currentStatus.nitrous_status;
  fullStatus[118] = currentStatus.TS_SD_Status; //SD card status

  for(byte x=0; x<packetLength; x++)
  {
      if (portType == 1){ CANSerial.write(fullStatus[offset+x]); }
      else if (portType == 2)
      {
        //sendto canbus transmit routine
      }
  }
#else 
  UNUSED(offset);
  UNUSED(packetLength);
  UNUSED(cmd);
  UNUSED(portType);
#endif

}

void can_Command()
{
 //int currentcanCommand = inMsg.id;
 #if defined (NATIVE_CAN_AVAILABLE)
      // currentStatus.canin[12] = (inMsg.id);
 if ( (inMsg.id == uint16_t(configPage9.obd_address + 0x100))  || (inMsg.id == 0x7DF))      
  {
    // The address is the speeduino specific ecu canbus address 
    // or the 0x7df(2015 dec) broadcast address
    if (inMsg.buf[1] == 0x01)
      {
        // PID mode 0 , realtime data stream
        obd_response(inMsg.buf[1], inMsg.buf[2], 0);     // get the obd response based on the data in byte2
        outMsg.id = (0x7E8);       //((configPage9.obd_address + 0x100)+ 8);  
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
 if (inMsg.id == uint16_t(configPage9.obd_address + 0x100))      
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
#if defined(CANSerial_AVAILABLE)
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
        #if defined(CORE_TEENSY) || defined(STM32F407xx) || defined(STM32F103xB) || defined(STM32F405xx)  //Scope guarding this for now, but this needs a bit of a rethink for how it can be handled better across multiple archs
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
#else
  UNUSED(cmdtype);
  UNUSED(canaddress);
  UNUSED(candata1);
  UNUSED(candata2);
  UNUSED(sourcecanAddress);
#endif
}

#if defined(NATIVE_CAN_AVAILABLE)
// This routine builds the realtime data into packets that the obd requesting device can understand. This is only used by teensy and stm32 with onboard canbus
void obd_response(uint8_t PIDmode, uint8_t requestedPIDlow, uint8_t requestedPIDhigh)
{ 
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
  
if (PIDmode == 0x01)
  {
     //currentStatus.canin[13] = therequestedPIDlow; 
   switch (requestedPIDlow)
         {
          case 0:       //PID-0x00 PIDs supported 01-20  
            outMsg.buf[0] =  0x06;    // sending 6 bytes
            outMsg.buf[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            outMsg.buf[2] =  0x00;    // PID code
            outMsg.buf[3] =  0x08;   //B0000 1000   1-8
            outMsg.buf[4] =  B00111110;   //9-16
            outMsg.buf[5] =  B10100000;   //17-24
            outMsg.buf[6] =  B00010001;   //17-32
            outMsg.buf[7] =  B00000000;   
          break;

          case 5:      //PID-0x05 Engine coolant temperature , range is -40 to 215 deg C , formula == A-40
            outMsg.buf[0] =  0x03;                 // sending 3 bytes
            outMsg.buf[1] =  0x41;                 // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            outMsg.buf[2] =  0x05;                 // pid code
            outMsg.buf[3] =  (byte)(currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);   //the data value A
            outMsg.buf[4] =  0x00;                 //the data value B which is 0 as unused
            outMsg.buf[5] =  0x00; 
            outMsg.buf[6] =  0x00; 
            outMsg.buf[7] =  0x00;
          break;
         
          case 11:        // PID-0x0B , MAP , range is 0 to 255 kPa , Formula == A
            uint16_t temp_engineMap;
            temp_engineMap = (highByte(currentStatus.MAP - currentStatus.baro)<<8) | lowByte(currentStatus.MAP - currentStatus.baro);
            outMsg.buf[0] =  0x03;    // sending 3 byte
            outMsg.buf[1] =  0x41;    // 
            outMsg.buf[2] =  0x0B;    // pid code
            outMsg.buf[3] =  temp_engineMap;    // absolute map is map gauge value - baro , baro is 100ish 
            outMsg.buf[4] =  0x00;
            outMsg.buf[5] =  0x00; 
            outMsg.buf[6] =  0x00; 
            outMsg.buf[7] =  0x00;
          break;

          case 12:        // PID-0x0C , RPM  , range is 0 to 16383.75 rpm , Formula == 256A+B / 4
            uint16_t temp_revs; 
            temp_revs = currentStatus.RPM << 2 ;      //
            outMsg.buf[0] = 0x04;                        // sending 4 byte
            outMsg.buf[1] = 0x41;                        // 
            outMsg.buf[2] = 0x0C;                        // pid code
            outMsg.buf[3] = highByte(temp_revs);         //obdcalcB; A
            outMsg.buf[4] = lowByte(temp_revs);          //obdcalcD; B
            outMsg.buf[5] = 0x00; 
            outMsg.buf[6] = 0x00; 
            outMsg.buf[7] = 0x00;
          break;

          case 13:        //PID-0x0D , Vehicle speed , range is 0 to 255 km/h , formula == A 
            uint8_t temp_vehiclespeed;
            temp_vehiclespeed = 120;               // TEST VALUE !!!!!   
            outMsg.buf[0] =  0x03;                    // sending 3 bytes
            outMsg.buf[1] =  0x41;                    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            outMsg.buf[2] =  0x0D;                    // pid code
            outMsg.buf[3] =  temp_vehiclespeed;       // A
            outMsg.buf[4] =  0x00;                    // B
            outMsg.buf[5] =  0x00; 
            outMsg.buf[6] =  0x00; 
            outMsg.buf[7] =  0x00;
          break;

          case 14:      //PID-0x0E , Ignition Timing advance, range is -64 to 63.5 BTDC , formula == A/2 - 64 
            int8_t temp_timingadvance;
            temp_timingadvance = ((currentStatus.advance + 64) << 1);
            //obdcalcA = ((timingadvance + 64) <<1) ; //((timingadvance + 64) *2)
            outMsg.buf[0] =  0x03;                     // sending 3 bytes
            outMsg.buf[1] =  0x41;                     // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            outMsg.buf[2] =  0x0E;                     // pid code
            outMsg.buf[3] =  temp_timingadvance;       // A
            outMsg.buf[4] =  0x00;                     // B
            outMsg.buf[5] =  0x00; 
            outMsg.buf[6] =  0x00; 
            outMsg.buf[7] =  0x00;
          break;

          case 15:      //PID-0x0F , Inlet air temperature , range is -40 to 215 deg C, formula == A-40 
            outMsg.buf[0] =  0x03;                                                         // sending 3 bytes
            outMsg.buf[1] =  0x41;                                                         // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
            outMsg.buf[2] =  0x0F;                                                         // pid code
            outMsg.buf[3] =  (byte)(currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET);   // A
            outMsg.buf[4] =  0x00;                                                         // B
            outMsg.buf[5] =  0x00; 
            outMsg.buf[6] =  0x00; 
            outMsg.buf[7] =  0x00;
         break;

         case 17:  // PID-0x11 , 
           // TPS percentage, range is 0 to 100 percent, formula == 100/256 A 
           uint16_t temp_tpsPC;
           temp_tpsPC = currentStatus.TPS;
           obdcalcA = (temp_tpsPC <<8) / 100;     // (tpsPC *256) /100;
           if (obdcalcA > 255){ obdcalcA = 255;}
           outMsg.buf[0] =  0x03;                    // sending 3 bytes
           outMsg.buf[1] =  0x41;                    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
           outMsg.buf[2] =  0x11;                    // pid code
           outMsg.buf[3] =  obdcalcA;                // A
           outMsg.buf[4] =  0x00;                    // B
           outMsg.buf[5] =  0x00; 
           outMsg.buf[6] =  0x00; 
           outMsg.buf[7] =  0x00;
         break;
  
         case 19:      //PID-0x13 , oxygen sensors present, A0-A3 == bank1 , A4-A7 == bank2 , 
           uint16_t O2present;
           O2present = B00000011 ;       //realtimebufferA[24];         TEST VALUE !!!!!
           outMsg.buf[0] =  0x03;           // sending 3 bytes
           outMsg.buf[1] =  0x41;           // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
           outMsg.buf[2] =  0x13;           // pid code
           outMsg.buf[3] =  O2present ;     // A
           outMsg.buf[4] =  0x00;           // B
           outMsg.buf[5] =  0x00; 
           outMsg.buf[6] =  0x00; 
           outMsg.buf[7] =  0x00;
         break;

         case 28:      // PID-0x1C obd standard
           uint16_t obdstandard;
           obdstandard = 7;              // This is OBD2 / EOBD
           outMsg.buf[0] =  0x03;           // sending 3 bytes
           outMsg.buf[1] =  0x41;           // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
           outMsg.buf[2] =  0x1C;           // pid code
           outMsg.buf[3] =  obdstandard;    // A
           outMsg.buf[4] =  0x00;           // B
           outMsg.buf[5] =  0x00; 
           outMsg.buf[6] =  0x00; 
           outMsg.buf[7] =  0x00;
         break;
  
        case 32:      // PID-0x20 PIDs supported [21-40]
          outMsg.buf[0] =  0x06;          // sending 4 bytes
          outMsg.buf[1] =  0x41;          // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
          outMsg.buf[2] =  0x20;          // pid code
          outMsg.buf[3] =  B01011000;     // 33-40
          outMsg.buf[4] =  B00000000;     //41 - 48
          outMsg.buf[5] =  B00100000;     //49-56
          outMsg.buf[6] =  B00000001;     //57-64
          outMsg.buf[7] = 0x00;
        break;

        case 34:      // PID-0x22 Fuel /Pressure (Relative to manifold vacuum) , range is 0 to 5177.265 kPa , formula == 0.079(256A+B)
          int16_t temp_fuelpressure;
          temp_fuelpressure = 3165;                 // test value !!!!!! currentStatus.fuelPressure ;           
          obdcalcG16 = temp_fuelpressure ;        //  this needs converting to kpa !
          obdcalcA = highByte(obdcalcG16);
          obdcalcB = lowByte(obdcalcG16);      
          outMsg.buf[0] =  0x03;                 // sending 3 bytes
          outMsg.buf[1] =  0x41;                 // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
          outMsg.buf[2] =  0x22;                 // pid code
          outMsg.buf[3] =  obdcalcA;             // A
          outMsg.buf[4] =  obdcalcB;             // B 
          outMsg.buf[5] =  0x00; 
          outMsg.buf[6] =  0x00; 
          outMsg.buf[7] =  0x00;
        break;
   
        case 36:      // PID-0x24 O2 sensor2, AB: fuel/air equivalence ratio, CD: voltage ,  Formula == (2/65536)(256A +B) , 8/65536(256C+D) , Range is 0 to <2 and 0 to >8V 
          //uint16_t O2_1e ;
          //int16_t O2_1v ; 
          obdcalcH16 = configPage2.stoich/10 ;            // configPage2.stoich(is *10 so 14.7 is 147)
          obdcalcE32 = currentStatus.O2/10;            // afr(is *10 so 25.5 is 255) , needs a 32bit else will overflow
          obdcalcF32 = (obdcalcE32<<8) / obdcalcH16;      //this is same as (obdcalcE32/256) / obdcalcH16 . this calculates the ratio      
          obdcalcG16 = (obdcalcF32 *32768)>>8;          
          obdcalcA = highByte(obdcalcG16);
          obdcalcB = lowByte(obdcalcG16);       

          obdcalcF32 = currentStatus.O2ADC ;             //o2ADC is wideband volts to send *100    
          obdcalcG16 = (obdcalcF32 *20971)>>8;          
          obdcalcC = highByte(obdcalcG16);
          obdcalcD = lowByte(obdcalcG16);
    
          outMsg.buf[0] =  0x06;    // sending 4 bytes
          outMsg.buf[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
          outMsg.buf[2] =  0x24;    // pid code
          outMsg.buf[3] =  obdcalcA;   // A
          outMsg.buf[4] =  obdcalcB;   // B
          outMsg.buf[5] =  obdcalcC;   // C
          outMsg.buf[6] =  obdcalcD;   // D
          outMsg.buf[7] =  0x00;
        break;

        case 37:      //O2 sensor2, AB fuel/air equivalence ratio, CD voltage ,  2/65536(256A +B) ,8/65536(256C+D) , range is 0 to <2 and 0 to >8V
          //uint16_t O2_2e ;
          //int16_t O2_2V ; 
          obdcalcH16 = configPage2.stoich/10 ;            // configPage2.stoich(is *10 so 14.7 is 147)
          obdcalcE32 = currentStatus.O2_2/10;            // afr(is *10 so 25.5 is 255) , needs a 32bit else will overflow
          obdcalcF32 = (obdcalcE32<<8) / obdcalcH16;      //this is same as (obdcalcE32/256) / obdcalcH16 . this calculates the ratio      
          obdcalcG16 = (obdcalcF32 *32768)>>8;          
          obdcalcA = highByte(obdcalcG16);
          obdcalcB = lowByte(obdcalcG16);       

          obdcalcF32 = currentStatus.O2_2ADC ;             //o2_2ADC is wideband volts to send *100    
          obdcalcG16 = (obdcalcF32 *20971)>>8;          
          obdcalcC = highByte(obdcalcG16);
          obdcalcD = lowByte(obdcalcG16);
    
          outMsg.buf[0] =  0x06;    // sending 4 bytes
          outMsg.buf[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
          outMsg.buf[2] =  0x25;    // pid code
          outMsg.buf[3] =  obdcalcA;   // A
          outMsg.buf[4] =  obdcalcB;   // B
          outMsg.buf[5] =  obdcalcC;   // C
          outMsg.buf[6] =  obdcalcD;   // D 
          outMsg.buf[7] =  0x00;
        break;

        case 51:      //PID-0x33 Absolute Barometric pressure , range is 0 to 255 kPa , formula == A
          outMsg.buf[0] =  0x03;                  // sending 3 bytes
          outMsg.buf[1] =  0x41;                  // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
          outMsg.buf[2] =  0x33;                  // pid code
          outMsg.buf[3] =  currentStatus.baro ;   // A
          outMsg.buf[4] =  0x00;                  // B which is 0 as unused
          outMsg.buf[5] =  0x00; 
          outMsg.buf[6] =  0x00; 
          outMsg.buf[7] =  0x00;
        break;
   
        case 64:      // PIDs supported [41-60]  
          outMsg.buf[0] =  0x06;    // sending 4 bytes
          outMsg.buf[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
          outMsg.buf[2] =  0x40;    // pid code
          outMsg.buf[3] =  B01000100;    // 65-72dec
          outMsg.buf[4] =  B00000000;    // 73-80
          outMsg.buf[5] =  B01000000;   //  81-88
          outMsg.buf[6] =  B00010000;   //  89-96
          outMsg.buf[7] =  0x00;
        break;

        case 66:      //control module voltage, 256A+B / 1000 , range is 0 to 65.535v
          uint16_t temp_ecuBatt;
          temp_ecuBatt = currentStatus.battery10;   // create a 16bit temp variable to do the math
          obdcalcA = temp_ecuBatt*100;              // should be *1000 but ecuBatt is already *10
          outMsg.buf[0] =  0x04;                       // sending 4 bytes
          outMsg.buf[1] =  0x41;                       // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
          outMsg.buf[2] =  0x42;                       // pid code
          outMsg.buf[3] =  highByte(obdcalcA) ;        // A
          outMsg.buf[4] =  lowByte(obdcalcA) ;         // B
          outMsg.buf[5] =  0x00; 
          outMsg.buf[6] =  0x00; 
          outMsg.buf[7] =  0x00;
        break;

        case 70:        //PID-0x46 Ambient Air Temperature , range is -40 to 215 deg C , formula == A-40
          uint16_t temp_ambientair;
          temp_ambientair = 11;              // TEST VALUE !!!!!!!!!!
          obdcalcA = temp_ambientair + 40 ;    // maybe later will be (byte)(currentStatus.AAT + CALIBRATION_TEMPERATURE_OFFSET)
          outMsg.buf[0] =  0x03;             // sending 3 byte
          outMsg.buf[1] =  0x41;             // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
          outMsg.buf[2] =  0x46;             // pid code
          outMsg.buf[3] =  obdcalcA;         // A 
          outMsg.buf[4] =  0x00;
          outMsg.buf[5] =  0x00; 
          outMsg.buf[6] =  0x00; 
          outMsg.buf[7] =  0x00;
        break;

        case 82:        //PID-0x52 Ethanol fuel % , range is 0 to 100% , formula == (100/255)A
          outMsg.buf[0] =  0x03;                       // sending 3 byte
          outMsg.buf[1] =  0x41;                       // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc. 
          outMsg.buf[2] =  0x52;                       // pid code
          outMsg.buf[3] =  currentStatus.ethanolPct;   // A
          outMsg.buf[4] =  0x00;
          outMsg.buf[5] =  0x00; 
          outMsg.buf[6] =  0x00; 
          outMsg.buf[7] =  0x00;
        break;

        case 92:        //PID-0x5C Engine oil temperature , range is -40 to 210 deg C , formula == A-40
          uint16_t temp_engineoiltemp;
          temp_engineoiltemp = 40;              // TEST VALUE !!!!!!!!!! 
          obdcalcA = temp_engineoiltemp+40 ;    // maybe later will be (byte)(currentStatus.EOT + CALIBRATION_TEMPERATURE_OFFSET)
          outMsg.buf[0] =  0x03;                // sending 3 byte
          outMsg.buf[1] =  0x41;                // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc. 
          outMsg.buf[2] =  0x5C;                // pid code
          outMsg.buf[3] =  obdcalcA ;           // A
          outMsg.buf[4] =  0x00;
          outMsg.buf[5] =  0x00; 
          outMsg.buf[6] =  0x00; 
          outMsg.buf[7] =  0x00;
        break;

        case 96:       //PIDs supported [61-80]  
          outMsg.buf[0] =  0x06;    // sending 4 bytes
          outMsg.buf[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
          outMsg.buf[2] =  0x60;    // pid code
          outMsg.buf[3] =  0x00;    // B0000 0000
          outMsg.buf[4] =  0x00;    // B0000 0000
          outMsg.buf[5] =  0x00;    // B0000 0000
          outMsg.buf[6] =  0x00;    // B0000 0000
          outMsg.buf[7] =  0x00;
        break;

        default:
        break;
     }
    } 
  else if (PIDmode == 0x22)
    {
     // these are custom PID  not listed in the SAE std .
     if (requestedPIDhigh == 0x77)
       {
        if ((requestedPIDlow >= 0x01) && (requestedPIDlow <= 0x10))
             {   
                 // PID 0x01 (1 dec) to 0x10 (16 dec)
                 // Aux data / can data IN Channel 1 - 16  
                 outMsg.buf[0] =  0x06;                                               // sending 8 bytes
                 outMsg.buf[1] =  0x62;                                               // Same as query, except that 40h is added to the mode value. So:62h = custom mode
                 outMsg.buf[2] =  requestedPIDlow;                                 // PID code
                 outMsg.buf[3] =  0x77;                                               // PID code
                 outMsg.buf[4] =  lowByte(currentStatus.canin[requestedPIDlow]);   // A
                 outMsg.buf[5] =  highByte(currentStatus.canin[requestedPIDlow]);  // B
                 outMsg.buf[6] =  0x00;                                               // C
                 outMsg.buf[7] =  0x00;                                               // D
            }
       }     
    }
}
#endif