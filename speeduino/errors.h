#ifndef ERRORS_H
#define ERRORS_H

/*
 * Up to 64 different error codes may be defined (6 bits)
 */
#define ERR_NONE        0U //No error
#define ERR_UNKNOWN     1U //Unknown error
#define ERR_IAT_SHORT   2U //Inlet sensor shorted
#define ERR_IAT_GND     3U //Inlet sensor grounded
#define ERR_CLT_SHORT   4U //Coolant sensor shorted
#define ERR_CLT_GND     5U //Coolant Sensor grounded
#define ERR_O2_SHORT    6U //O2 sensor shorted
#define ERR_O2_GND      7U //O2 sensor grounded
#define ERR_TPS_SHORT   8U //TPS shorted (Is potentially valid)
#define ERR_TPS_GND     9U //TPS grounded (Is potentially valid)
#define ERR_BAT_HIGH    10U //Battery voltage is too high
#define ERR_BAT_LOW     11U //Battery voltage is too low
#define ERR_MAP_HIGH    12U //MAP output is too high
#define ERR_MAP_LOW     13U //MAP output is too low

#define ERR_DEFAULT_IAT_SHORT   80 //Note that the default is 40C. 80 is used due to the -40 offset
#define ERR_DEFAULT_IAT_GND     80 //Note that the default is 40C. 80 is used due to the -40 offset
#define ERR_DEFAULT_CKT_SHORT   80 //Note that the default is 40C. 80 is used due to the -40 offset
#define ERR_DEFAULT_CLT_GND     80 //Note that the default is 40C. 80 is used due to the -40 offset
#define ERR_DEFAULT_O2_SHORT    147 //14.7
#define ERR_DEFAULT_O2_GND      147 //14.7
#define ERR_DEFAULT_TPS_SHORT   50 //50%
#define ERR_DEFAULT_TPS_GND     50 //50%
#define ERR_DEFAULT_BAT_HIGH    130 //13v
#define ERR_DEFAULT_BAT_LOW     130 //13v
#define ERR_DEFAULT_MAP_HIGH    240
#define ERR_DEFAULT_MAP_LOW     80


#define MAX_ERRORS  4 //The number of errors the system can hold simultaneously. Should be a power of 2

/*
 * This struct is a single byte in length and is sent to TS
 * The first 2 bits are used to define the current error (0-3)
 * The remaining 6 bits are used to give the error number
 */
struct packedError
{
  byte errorNum : 2;
  byte errorID  : 6;
};

byte getNextError(void);
byte setError(byte errorID);
void clearError(byte errorID);

extern byte errorCount;

#endif
