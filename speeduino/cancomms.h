#ifndef CANCOMMS_H
#define CANCOMMS_H

#define NEW_CAN_PACKET_SIZE   119
#define CAN_PACKET_SIZE   75

#if ( defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) )
  #define CANSerial_AVAILABLE
  extern HardwareSerial &CANSerial;
#elif defined(CORE_STM32)
  #define CANSerial_AVAILABLE
  #ifndef Serial2
    #define Serial2 Serial1
  #endif
  #if defined(STM32GENERIC) // STM32GENERIC core
    extern SerialUART &CANSerial;
  #else //libmaple core aka STM32DUINO
    extern HardwareSerial &CANSerial;
  #endif
#elif defined(CORE_TEENSY)
  #define CANSerial_AVAILABLE
  extern HardwareSerial &CANSerial;
#endif

void secondserial_Command();//This is the heart of the Command Line Interpeter.  All that needed to be done was to make it human readable.
void sendcanValues(uint16_t offset, uint16_t packetLength, byte cmd, byte portNum);
void can_Command();
void sendCancommand(uint8_t cmdtype , uint16_t canadddress, uint8_t candata1, uint8_t candata2, uint16_t sourcecanAddress);
void obd_response(uint8_t therequestedPID , uint8_t therequestedPIDlow, uint8_t therequestedPIDhigh);

#endif // CANCOMMS_H
