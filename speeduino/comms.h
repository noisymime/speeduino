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


extern byte currentPage;//Not the same as the speeduino config page numbers
extern bool isMap; /**< Whether or not the currentPage contains only a 3D map that would require translation */
extern unsigned long requestCount; /**< The number of times the A command has been issued. This is used to track whether a reset has recently been performed on the controller */
extern byte currentCommand; /**< The serial command that is currently being processed. This is only useful when cmdPending=True */
extern bool cmdPending; /**< Whether or not a serial request has only been partially received. This occurs when a command character has been received in the serial buffer, but not all of its arguments have yet been received. If true, the active command will be stored in the currentCommand variable */
extern bool chunkPending; /**< Whether or not the current chucnk write is complete or not */
extern uint16_t chunkComplete; /**< The number of bytes in a chunk write that have been written so far */
extern uint16_t chunkSize; /**< The complete size of the requested chunk write */
extern int valueOffset; /**< THe memory offset within a given page for a value to be read from or written to. Note that we cannot use 'offset' as a variable name, it is a reserved word for several teensy libraries */
extern byte tsCanId;     // current tscanid requested
extern byte inProgressOffset;
extern byte inProgressLength;
extern bool legacySerial;
extern uint32_t inProgressCompositeTime;
extern bool serialInProgress;
extern bool toothLogSendInProgress;
extern bool compositeLogSendInProgress;

void command();//This is the heart of the Command Line Interpeter.  All that needed to be done was to make it human readable.
void sendValues(uint16_t, uint16_t,byte, byte);
void sendValuesLegacy();
void saveConfig();
void sendPage();
void sendPageASCII();
void receiveCalibration(byte);
void sendToothLog_old(uint8_t);
void testComm();
void commandButtons(int16_t);
void sendCompositeLog_old(uint8_t);

#endif // COMMS_H
