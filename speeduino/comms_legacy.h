/** \file comms.h
 * @brief File for handling all serial requests 
 * @author Josh Stewart
 * 
 * This file contains all the functions associated with serial comms.
 * This includes sending of live data, sending/receiving current page data, sending CRC values of pages, receiving sensor calibration data etc
 * 
 */

#ifndef COMMS_H
#define COMMS_H

/** \enum SerialStatus
 * @brief The current state of serial communication
 * */
enum SerialStatus {
  /** No serial comms is in progress */
  SERIAL_INACTIVE, 
  /** A partial write is in progress. */
  SERIAL_TRANSMIT_INPROGRESS, 
  /** A partial write is in progress (legacy send). */
  SERIAL_TRANSMIT_INPROGRESS_LEGACY, 
  /** We are part way through transmitting the tooth log */
  SERIAL_TRANSMIT_TOOTH_INPROGRESS, 
  /** We are part way through transmitting the tooth log (legacy send) */
  SERIAL_TRANSMIT_TOOTH_INPROGRESS_LEGACY, 
  /** We are part way through transmitting the composite log */
  SERIAL_TRANSMIT_COMPOSITE_INPROGRESS,
  /** We are part way through transmitting the composite log (legacy send) */
  SERIAL_TRANSMIT_COMPOSITE_INPROGRESS_LEGACY,
  /** Whether or not a serial request has only been partially received.
   * This occurs when a the length has been received in the serial buffer,
   * but not all of the payload or CRC has yet been received. 
   * 
   * Expectation is that ::serialReceive is called  until the status reverts 
   * to SERIAL_INACTIVE
  */
  SERIAL_RECEIVE_INPROGRESS,
  /** We are part way through processing a legacy serial commang: call ::serialReceive */
  SERIAL_COMMAND_INPROGRESS_LEGACY,
};
/** @brief Current status of serial comms. */
extern SerialStatus serialStatusFlag;

/**
 * @brief Is a serial write in progress?
 * 
 * Expectation is that ::serialTransmit is called until this
 * returns false
 */
inline bool serialTransmitInProgress(void) {
    return serialStatusFlag==SERIAL_TRANSMIT_INPROGRESS
    || serialStatusFlag==SERIAL_TRANSMIT_INPROGRESS_LEGACY
    || serialStatusFlag==SERIAL_TRANSMIT_TOOTH_INPROGRESS
    || serialStatusFlag==SERIAL_TRANSMIT_TOOTH_INPROGRESS_LEGACY
    || serialStatusFlag==SERIAL_TRANSMIT_COMPOSITE_INPROGRESS
    || serialStatusFlag==SERIAL_TRANSMIT_COMPOSITE_INPROGRESS_LEGACY;
}

/**
 * @brief Is a non-blocking serial receive operation in progress?
 * 
 * Expectation is the ::serialReceive is called until this
 * returns false.
 */
inline bool serialRecieveInProgress(void) {
  return serialStatusFlag==SERIAL_RECEIVE_INPROGRESS
  || serialStatusFlag==SERIAL_COMMAND_INPROGRESS_LEGACY;
}

extern bool firstCommsRequest; /**< The number of times the A command has been issued. This is used to track whether a reset has recently been performed on the controller */
extern byte logItemsTransmitted;
extern byte inProgressLength;

void legacySerialCommand(void);//This is the heart of the Command Line Interpreter.  All that needed to be done was to make it human readable.
void sendValues(uint16_t offset, uint16_t packetLength, byte cmd, byte portNum);
void sendValuesLegacy(void);
void sendPage(void);
void sendPageASCII(void);
void receiveCalibration(byte tableID);
void testComm(void);
void sendToothLog_legacy(byte startOffset);
void sendCompositeLog_legacy(byte startOffset);

#endif // COMMS_H
