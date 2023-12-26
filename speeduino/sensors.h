#ifndef SENSORS_H
#define SENSORS_H

#include "globals.h"

// The following are alpha values for the ADC filters.
// Their values are from 0 to 240, with 0 being no filtering and 240 being maximum
#define ADCFILTER_TPS_DEFAULT   50
#define ADCFILTER_CLT_DEFAULT  180
#define ADCFILTER_IAT_DEFAULT  180
#define ADCFILTER_O2_DEFAULT   128
#define ADCFILTER_BAT_DEFAULT  128
#define ADCFILTER_MAP_DEFAULT   20 //This is only used on Instantaneous MAP readings and is intentionally very weak to allow for faster response
#define ADCFILTER_BARO_DEFAULT  64

#define ADCFILTER_PSI_DEFAULT  150 //not currently configurable at runtime, used for misc pressure sensors, oil, fuel, etc.

#define FILTER_FLEX_DEFAULT     75

#define BARO_MIN      65
#define BARO_MAX      108

#define KNOCK_MODE_DIGITAL  1
#define KNOCK_MODE_ANALOG   2

#define VSS_GEAR_HYSTERESIS 10
#define VSS_SAMPLES         4 //Must be a power of 2 and smaller than 255

#define TPS_READ_FREQUENCY  30 //ONLY VALID VALUES ARE 15 or 30!!!

extern volatile byte flexCounter;
extern volatile unsigned long flexStartTime;
extern volatile unsigned long flexPulseWidth;

#if defined(CORE_AVR)
  #define READ_FLEX() ((*flex_pin_port & flex_pin_mask) ? true : false)
#else
  #define READ_FLEX() digitalRead(pinFlex)
#endif

extern volatile byte knockCounter;

extern unsigned int MAPcount; //Number of samples taken in the current MAP cycle
extern uint32_t MAPcurRev; //Tracks which revolution we're sampling on
extern bool auxIsEnabled;
extern uint16_t MAPlast; /**< The previous MAP reading */
extern unsigned long MAP_time; //The time the MAP sample was taken
extern unsigned long MAPlast_time; //The time the previous MAP sample was taken

/**
 * @brief Simple low pass IIR filter macro for the analog inputs
 * This is effectively implementing the smooth filter from playground.arduino.cc/Main/Smooth
 * But removes the use of floats and uses 8 bits of fixed precision.
 */
#define ADC_FILTER(input, alpha, prior) (((long)input * (256 - alpha) + ((long)prior * alpha))) >> 8

void initialiseADC(void);
void readTPS(bool useFilter=true); //Allows the option to override the use of the filter
void readO2_2(void);
void flexPulse(void);
uint32_t vssGetPulseGap(byte toothHistoryIndex);
void vssPulse(void);
uint16_t getSpeed(void);
byte getGear(void);
byte getFuelPressure(void);
byte getOilPressure(void);
uint16_t readAuxanalog(uint8_t analogPin);
uint16_t readAuxdigital(uint8_t digitalPin);
void readCLT(bool useFilter=true); //Allows the option to override the use of the filter
void readIAT(void);
void readO2(void);
void readBat(void);
void readBaro(void);
void readMAP(void);
void instanteneousMAPReading(void);

#endif // SENSORS_H
