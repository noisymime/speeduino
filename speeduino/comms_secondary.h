#ifndef COMMS_SECONDARY_H
#define COMMS_SECONDARY_H

#include <Arduino.h>

#define NEW_CAN_PACKET_SIZE   123
#define CAN_PACKET_SIZE   75

#define SECONDARY_SERIAL_PROTO_GENERIC_FIXED  0
#define SECONDARY_SERIAL_PROTO_GENERIC_INI    1
#define SECONDARY_SERIAL_PROTO_CAN            2
#define SECONDARY_SERIAL_PROTO_MSDROID        3
#define SECONDARY_SERIAL_PROTO_REALDASH       4

extern SECONDARY_SERIAL_T *pSecondarySerial;
#define secondarySerial (*pSecondarySerial)

void secondserial_Command(void);//This is the heart of the Command Line Interpreter.  All that needed to be done was to make it human readable.
void sendCancommand(uint8_t cmdtype , uint16_t canadddress, uint8_t candata1, uint8_t candata2, uint16_t sourcecanAddress);

#endif // COMMS_SECONDARY_H
