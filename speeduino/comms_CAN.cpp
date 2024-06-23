/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

/*
This is for handling the data broadcasted to various CAN dashes and instrument clusters.
*/
#include "globals.h"

#if defined(NATIVE_CAN_AVAILABLE)
#include "comms_CAN.h"
#include "utilities.h"

CAN_message_t inMsg;
CAN_message_t outMsg;

//These are declared locally for Teensy due to this issue: https://github.com/tonton81/FlexCAN_T4/issues/67
#if defined(CORE_TEENSY35) || defined(CORE_TEENSY36)        // use for Teensy 3.5/3.6 only 
  FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
#elif defined(CORE_TEENSY41)         // use for Teensy 4.1 only
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0; 
#endif

// Forward declare
void DashMessage(uint16_t DashMessageID);


void initCAN()
{
  #if defined (NATIVE_CAN_AVAILABLE)
    configPage9.intcan_available = 1;   // device has internal canbus
    //Teensy uses the Flexcan_T4 library to use the internal canbus
    //enable local can interface
    //setup can interface to 500k   
    Can0.begin();
    Can0.setBaudRate(500000);
    Can0.enableFIFO();
    /* Note: This must come after the call to setPinMapping() or else pins 29 and 30 will become unusable as outputs.
     * Workaround for: https://github.com/tonton81/FlexCAN_T4/issues/14 */
    
    #if defined(CORE_TEENSY35)         // use for Teensy 3.5 only 
      Can0.setRX(DEF);
      Can0.setTX(DEF);
    #endif
  #endif
}

int CAN_read()
{
  return Can0.read(inMsg);
}

void CAN_write()
{
  Can0.write(outMsg);
}

void sendBMWCluster()
{
  outMsg.flags.extended = 0; //Make sure to set this to standard
  DashMessage(CAN_BMW_DME1);
  Can0.write(outMsg);
  DashMessage(CAN_BMW_DME2);
  Can0.write(outMsg);
  DashMessage(CAN_BMW_DME4);
  Can0.write(outMsg);
}

void sendVAGCluster()
{
  outMsg.flags.extended = 0; //Make sure to set this to standard
  DashMessage(CAN_VAG_RPM);
  Can0.write(outMsg);
  DashMessage(CAN_VAG_VSS);
  Can0.write(outMsg);
}
void receiveCANwbo() 
{
  // Currently only RusEFI CAN Wideband supported: https://github.com/mck1117/wideband
  if(configPage2.canWBO == CAN_WBO_RUSEFI)
  {
    outMsg.id = 0xEF50000;
    outMsg.flags.extended = 1;
    outMsg.len = 2;
    outMsg.buf[0] = currentStatus.battery10; // We don't do any conversion since factor is 0.1 and speeduino value is x10
    outMsg.buf[1] = BIT_CHECK(currentStatus.engine, BIT_ENGINE_RUN) ? 0x1 : 0x0; // Enable heater once engine is running (ie. above cranking rpm), this condition can be changed to CLT above certain temp and so on.
    Can0.write(outMsg);
    outMsg.flags.extended = 0; //Make sure to set this back to standard to avoid future problems
    if ((inMsg.id == 0x190 || inMsg.id == 0x192))
    {
      uint32_t inLambda;
      inLambda = (word(inMsg.buf[3], inMsg.buf[2])); // Combining 2 bytes of data into single variable factor is 0.0001 so lambda 1 comes in as 10K
      if(inMsg.buf[1] == 0x1) // Checking if lambda is valid
      {
        inLambda = (inLambda * configPage2.stoich) / 10000; // Multiplying lambda by stoich ratio to get AFR and dividing it by 10000 to get correct value
        switch(inMsg.id)
        {
          case 0x190:
          if (inLambda > 250) { currentStatus.O2 = 250; } //Check if we don't overflow the 8bit O2 variable
          else { currentStatus.O2 = inLambda & 0xFF; }
          break;

          case 0x192:
          if (inLambda > 250) { currentStatus.O2_2 = 250; } //Check if we don't overflow the 8bit O2 variable
          else { currentStatus.O2_2 = inLambda & 0xFF; }
          break;

          default:
          break;
        }
      }
    }
  }
}
// switch case for gathering all data to message based on CAN Id.
void DashMessage(uint16_t DashMessageID)
{
  switch (DashMessageID)
  {
    case CAN_BMW_DME1:
      uint32_t temp_RPM;
      temp_RPM = currentStatus.RPM * 64;  //RPM conversion is currentStatus.RPM * 6.4, but this does it without floats.
      temp_RPM = temp_RPM / 10;
      outMsg.id = DashMessageID;
      outMsg.len = 8;
      outMsg.buf[0] = 0x05;  //bitfield, Bit0 = 1 = terminal 15 on detected, Bit2 = 1 = the ASC message ASC1 was received within the last 500 ms and contains no plausibility errors
      outMsg.buf[1] = 0x0C;  //Indexed Engine Torque in % of C_TQ_STND TBD do torque calculation.
      outMsg.buf[2] = lowByte(uint16_t(temp_RPM));  //lsb RPM
      outMsg.buf[3] = highByte(uint16_t(temp_RPM)); //msb RPM
      outMsg.buf[4] = 0x0C;  //Indicated Engine Torque in % of C_TQ_STND TBD do torque calculation!! Use same as for byte 1
      outMsg.buf[5] = 0x15;  //Engine Torque Loss (due to engine friction, AC compressor and electrical power consumption)
      outMsg.buf[6] = 0x00;  //not used
      outMsg.buf[7] = 0x35;  //Theorethical Engine Torque in % of C_TQ_STND after charge intervention
    break;

    case CAN_BMW_DME2:
      uint8_t temp_TPS;
      uint8_t temp_BARO;
      uint16_t temp_CLT;
      temp_TPS = map(currentStatus.TPS, 0, 200, 1, 254);//TPS value conversion (from 0x01 to 0xFE)
      temp_CLT = (((currentStatus.coolant - CALIBRATION_TEMPERATURE_OFFSET) + 48)*4/3); //CLT conversion (actual value to add is 48.373, but close enough)
      if (temp_CLT > 255) { temp_CLT = 255; } //CLT conversion can yield to higher values than what fits to byte, so limit the maximum value to 255.
      temp_BARO = currentStatus.baro;

      outMsg.id = DashMessageID;
      outMsg.len = 7;
      outMsg.buf[0] = 0x11;  //Multiplexed Information
      outMsg.buf[1] = temp_CLT;
      outMsg.buf[2] = temp_BARO;
      outMsg.buf[3] = 0x08;  //bitfield, Bit0 = 0 = Clutch released, Bit 3 = 1 = engine running
      outMsg.buf[4] = 0x00;  //TPS_VIRT_CRU_CAN (Not used)
      outMsg.buf[5] = temp_TPS;
      outMsg.buf[6] = 0x00;  //bitfield, Bit0 = 0 = brake not actuated, Bit1 = 0 = brake switch system OK etc...
      outMsg.buf[7] = 0x00;  //not used, but set to zero just in case.
    break;

    case 0x545:       //fuel consumption and CEl light for BMW e46/e39/e38 instrument cluster
                      //fuel consumption calculation not implemented yet. But this still needs to be sent to get rid of the CEL and EML fault lights on the dash.
      outMsg.id = DashMessageID;
      outMsg.len = 5;
      outMsg.buf[0] = 0x00;  //Check engine light (binary 10), Cruise light (binary 1000), EML (binary 10000).
      outMsg.buf[1] = 0x00;  //LSB Fuel consumption
      outMsg.buf[2] = 0x00;  //MSB Fuel Consumption
      if (currentStatus.coolant > 159) { outMsg.buf[3] = 0x08; } //Turn on overheat light if coolant temp hits 120 degrees celsius.
      else { outMsg.buf[3] = 0x00; } //Overheat light off at normal engine temps.
      outMsg.buf[4] = 0x7E; //this is oil temp
    break;

    case 0x280:       //RPM for VW instrument cluster
      temp_RPM =  currentStatus.RPM * 4; //RPM conversion
      outMsg.id = DashMessageID;
      outMsg.len = 8;
      outMsg.buf[0] = 0x49;
      outMsg.buf[1] = 0x0E;
      outMsg.buf[2] = lowByte(uint16_t(temp_RPM));  //lsb RPM
      outMsg.buf[3] = highByte(uint16_t(temp_RPM)); //msb RPM
      outMsg.buf[4] = 0x0E;
      outMsg.buf[5] = 0x00;
      outMsg.buf[6] = 0x1B;
      outMsg.buf[7] = 0x0E;
    break;

    case 0x5A0:       //VSS for VW instrument cluster
      uint16_t temp_VSS;
      temp_VSS =  currentStatus.vss * 133; //VSS conversion
      outMsg.id = DashMessageID;
      outMsg.len = 8;
      outMsg.buf[0] = 0xFF;
      outMsg.buf[1] = lowByte(temp_VSS);
      outMsg.buf[2] = highByte(temp_VSS);
      outMsg.buf[3] = 0x00;
      outMsg.buf[4] = 0x00;
      outMsg.buf[5] = 0x00;
      outMsg.buf[6] = 0x00;
      outMsg.buf[7] = 0xAD;
    break;

    default:
    break;
  }
}

void can_Command(void)
{
  if ( (inMsg.id == uint16_t(configPage9.obd_address + TS_CAN_OFFSET))  || (inMsg.id == 0x7DF))      
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
  if (inMsg.id == uint16_t(configPage9.obd_address + TS_CAN_OFFSET))      
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
} 

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
  outMsg.flags.extended = 0; //Make sure to set this to standard
  
  if (PIDmode == 0x01)
  {
    switch (requestedPIDlow)
    {
      case 0:       //PID-0x00 PIDs supported 01-20  
        outMsg.buf[0] =  0x06;    // sending 6 bytes
        outMsg.buf[1] =  0x41;    // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
        outMsg.buf[2] =  0x00;    // PID code
        outMsg.buf[3] =  0x08;   //B0000 1000   1-8
        outMsg.buf[4] =  B01111110;   //9-16
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

      case 10:        // PID-0x0A , Fuel Pressure (Gauge) , range is 0 to 765 kPa , formula == A / 3)
        uint16_t temp_fuelpressure;
        // Fuel pressure is in PSI. PSI to kPa is 6.89475729, but that needs to be divided by 3 for OBD2 formula. So 2.298.... 2.3 is close enough, so that in fraction.
        temp_fuelpressure = (currentStatus.fuelPressure * 23) / 10;
        outMsg.buf[0] =  0x03;    // sending 3 byte
        outMsg.buf[1] =  0x41;    // 
        outMsg.buf[2] =  0x0A;    // pid code
        outMsg.buf[3] =  lowByte(temp_fuelpressure);
        outMsg.buf[4] =  0x00;
        outMsg.buf[5] =  0x00; 
        outMsg.buf[6] =  0x00; 
        outMsg.buf[7] =  0x00;
      break;

      case 11:        // PID-0x0B , MAP , range is 0 to 255 kPa , Formula == A
        outMsg.buf[0] =  0x03;    // sending 3 byte
        outMsg.buf[1] =  0x41;    // 
        outMsg.buf[2] =  0x0B;    // pid code
        outMsg.buf[3] =  lowByte(currentStatus.MAP);    // absolute map
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
        outMsg.buf[0] =  0x03;                       // sending 3 bytes
        outMsg.buf[1] =  0x41;                       // Same as query, except that 40h is added to the mode value. So:41h = show current data ,42h = freeze frame ,etc.
        outMsg.buf[2] =  0x0D;                       // pid code
        outMsg.buf[3] =  lowByte(currentStatus.vss); // A
        outMsg.buf[4] =  0x00;                       // B
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
        obdcalcA = (temp_tpsPC <<8) / 200;     // (tpsPC *256) /200;
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
        outMsg.buf[3] =  B00011000;     // 33-40
        outMsg.buf[4] =  B00000000;     //41 - 48
        outMsg.buf[5] =  B00100000;     //49-56
        outMsg.buf[6] =  B00000001;     //57-64
        outMsg.buf[7] = 0x00;
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
          outMsg.buf[4] =  lowByte(currentStatus.canin[requestedPIDlow-1]);   // A
          outMsg.buf[5] =  highByte(currentStatus.canin[requestedPIDlow-1]);  // B
          outMsg.buf[6] =  0x00;                                               // C
          outMsg.buf[7] =  0x00;                                               // D
      }
    }
    // this allows to get any value out of current status array.
    else if (requestedPIDhigh == 0x78)
    {
      int16_t tempValue;
      tempValue = ProgrammableIOGetData(requestedPIDlow);
      outMsg.buf[0] =  0x06;                 // sending 6 bytes
      outMsg.buf[1] =  0x62;                 // Same as query, except that 40h is added to the mode value. So:62h = custom mode
      outMsg.buf[2] =  requestedPIDlow;      // PID code
      outMsg.buf[3] =  0x78;                 // PID code
      outMsg.buf[4] =  lowByte(tempValue);   // A
      outMsg.buf[5] =  highByte(tempValue);  // B
      outMsg.buf[6] =  0x00; 
      outMsg.buf[7] =  0x00;
    }
  }
}

void readAuxCanBus()
{
  for (int i = 0; i < 16; i++)
  {
    uint16_t channelAddress = (configPage9.caninput_source_can_address[i] + TS_CAN_OFFSET);
    if (inMsg.id == channelAddress ) //Filters frame ID
    {

      if (!BIT_CHECK(configPage9.caninput_source_num_bytes, i))
      {
        // Gets the one-byte value from the Data Field.
        currentStatus.canin[i] = inMsg.buf[configPage9.caninput_source_start_byte[i]];
      }
      else
      {
        if (configPage9.caninputEndianess == 1)
        {
          //Gets the two-byte value from the Data Field in Litlle Endian.
          currentStatus.canin[i] = ((inMsg.buf[configPage9.caninput_source_start_byte[i]]) | (inMsg.buf[configPage9.caninput_source_start_byte[i] + 1] << 8));
        }
        else
        {
          //Gets the two-byte value from the Data Field in Big Endian.
          currentStatus.canin[i] = ((inMsg.buf[configPage9.caninput_source_start_byte[i]] << 8) | (inMsg.buf[configPage9.caninput_source_start_byte[i] + 1]));
        }
      }
    }
  } 
}
#endif