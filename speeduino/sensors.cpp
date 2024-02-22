/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
/** @file
 * Read sensors with appropriate timing / scheduling.
 */
#include <SimplyAtomic.h>
#include "sensors.h"
#include "crankMaths.h"
#include "globals.h"
#include "maths.h"
#include "storage.h"
#include "comms.h"
#include "idle.h"
#include "errors.h"
#include "corrections.h"
#include "pages.h"
#include "decoders.h"
#include "auxiliaries.h"
#include "utilities.h"

/**
 * @brief A specialist function to map a value in the range [0, 1023] (I.e. 10-bit) to a different range.
 * 
 * Mostly used for analog input voltage level to real world value conversions.
 * 
 * @details
 * analogRead returns a number in the range [0, 1023], representing the pin input 
 * voltage from min to max (typically 0V - 5V)
 * We need to convert that value to the real world value the sensor is reading (pressure, temperature etc.)
 * If:
 *    * rangeMin is the real world value when the sensor is reading 0V
 *    * rangeMax is the real world measurement when the sensor is reading 5V
 *    * There is a linear relationship between voltage output and the real world value.
 * 
 * then this function will return the real world measurement (kPa, °C etc)
 * 
 * @param value Value to map (should be in range [0, 1023])
 * @param rangeMin Minimum of the output range
 * @param rangeMax Maximum of the output range
 * @return int16_t 
 */
static inline int16_t fastMap10Bit(uint16_t value, int16_t rangeMin, int16_t rangeMax) {
  uint16_t range = rangeMax-rangeMin; // Must be positive (assuming rangeMax>=rangeMin)
  uint16_t fromStartOfRange = (uint16_t)rshift<10>((uint32_t)value * range);
  return rangeMin + (int16_t)fromStartOfRange;
}

//
static inline uint16_t readAnalogPin(uint8_t pin) {
  // Why do we read twice? Who knows.....
  analogRead(pin);
  // According to the docs, analogRead result should be in range 0-1023
  // Clip the result to zero minimum to prevent rollover just in case
  int tmp = analogRead(pin);
  // max is a macro on some platforms - DO NOT place the call to analogRead as an inline parameter:
  // (you might end up calling it twice)
  return max(0, tmp);
}

bool auxIsEnabled;

static volatile uint32_t vssTimes[VSS_SAMPLES] = {0};
static volatile uint8_t vssIndex = 0U;

volatile uint8_t flexCounter = 0U;
static volatile uint32_t flexStartTime = 0UL;
volatile uint32_t flexPulseWidth = 0U;

#if defined(ANALOG_ISR)
static volatile uint16_t AnChannel[16];
static inline uint16_t readAnalogSensor(uint8_t pin) {
  return AnChannel[pin-A0];
}
static inline uint16_t readMAPSensor(uint8_t pin) {
#if defined(ANALOG_ISR_MAP)
  return AnChannel[pin-A0];
#else
  return readAnalogPin(pin);
#endif
}
#define ADMUX_DEFAULT_CONFIG  0x40 //AVCC reference, ADC0 input, right adjusted, ADC enabled

ISR(ADC_vect)
{
  byte nChannel = (ADMUX & 0x07);

  byte result_low = ADCL;
  byte result_high = ADCH;

  #if defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__)
    if (nChannel == 7U) { ADMUX = 0x40; }
  #elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    if( BIT_CHECK(ADCSRB, MUX5) ) { nChannel += 8; }  //8 to 15
    if(nChannel == 15U)
    {
      ADMUX = ADMUX_DEFAULT_CONFIG; //channel 0
      ADCSRB = 0x00; //clear MUX5 bit

      BIT_CLEAR(ADCSRA,ADIE); //Disable interrupt as we're at the end of a full ADC cycle. This will be re-enabled in the main loop
    }
    else if (nChannel == 7U) //channel 7
    {
      ADMUX = ADMUX_DEFAULT_CONFIG;
      ADCSRB = 0x08; //Set MUX5 bit
    }
  #endif
    else { ADMUX++; }

  //ADMUX always appears to be one ahead of the actual channel value that is in ADCL/ADCH. Subtract 1 from it to get the correct channel number
  if(nChannel == 0U) { nChannel = 16;} 
  AnChannel[nChannel-1] = (result_high << 8) | result_low;
}
#else
static inline uint16_t readAnalogSensor(uint8_t pin) {
  return readAnalogPin(pin);
}
static inline uint16_t readMAPSensor(uint8_t pin) {
  return readAnalogPin(pin);
}
#endif

/** Init all ADC conversions by setting resolutions, etc.
 */
void initialiseADC(void)
{
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this

  #if defined(ANALOG_ISR)
    noInterrupts(); //Interrupts should be turned off when playing with any of these registers

    ADCSRB = 0x00; //ADC Auto Trigger Source is in Free Running mode

    ADMUX = ADMUX_DEFAULT_CONFIG;  //Select AVCC as reference, ADC Right Adjust Result, Starting at channel 0

    //All of the below is the longhand version of: ADCSRA = 0xEE;
    #ifndef ADFR
      #define ADFR 5 //Looks like this is now defined. Retain this for compatibility with earlier versions of Arduino IDE that did not have this.
    #endif
    BIT_SET(ADCSRA,ADFR); //Set free running mode
    BIT_SET(ADCSRA,ADIE); //Set ADC interrupt enabled
    BIT_CLEAR(ADCSRA,ADIF); //Clear interrupt flag

    // Set ADC clock to 125KHz (Prescaler = 128)
    BIT_SET(ADCSRA,ADPS2);
    BIT_SET(ADCSRA,ADPS1);
    BIT_SET(ADCSRA,ADPS0);

    BIT_SET(ADCSRA,ADEN); //Enable ADC

    interrupts();
    BIT_SET(ADCSRA,ADSC); //Start conversion

  #else
    //This sets the ADC (Analog to Digital Converter) to run at 1Mhz, greatly reducing analog read times (MAP/TPS) when using the standard analogRead() function
    //1Mhz is the fastest speed permitted by the CPU without affecting accuracy
    //Please see chapter 11 of 'Practical Arduino' (books.google.com.au/books?id=HsTxON1L6D4C&printsec=frontcover#v=onepage&q&f=false) for more detail
     BIT_SET(ADCSRA,ADPS2);
     BIT_CLEAR(ADCSRA,ADPS1);
     BIT_CLEAR(ADCSRA,ADPS0);
  #endif
#elif defined(ARDUINO_ARCH_STM32) //STM32GENERIC core and ST STM32duino core, change analog read to 12 bit
  analogReadResolution(10); //use 10bits for analog reading on STM32 boards
#endif

  //The following checks the aux inputs and initialises pins if required
  auxIsEnabled = false;
  for (uint8_t AuxinChan = 0U; AuxinChan <16U ; AuxinChan++)
  {
    currentStatus.current_caninchannel = AuxinChan;                   
    if (((configPage9.caninput_sel[currentStatus.current_caninchannel]&12U) == 4U)
    && ((configPage9.enable_secondarySerial == 1U) || ((configPage9.enable_intcan == 1U) && (configPage9.intcan_available == 1U))))
    { //if current input channel is enabled as external input in caninput_selxb(bits 2:3) and secondary serial or internal canbus is enabled(and is mcu supported)                 
      //currentStatus.canin[14] = 22;  Dev test use only!
      auxIsEnabled = true;     
    }
    else if ((((configPage9.enable_secondarySerial == 1U) || ((configPage9.enable_intcan == 1U) && (configPage9.intcan_available == 1U))) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&12U) == 8U)
            || (((configPage9.enable_secondarySerial == 0U) && ( (configPage9.enable_intcan == 1U) && (configPage9.intcan_available == 0U) )) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&3U) == 2U)  
            || (((configPage9.enable_secondarySerial == 0U) && (configPage9.enable_intcan == 0U)) && ((configPage9.caninput_sel[currentStatus.current_caninchannel]&3U) == 2U)))  
    {  //if current input channel is enabled as analog local pin check caninput_selxb(bits 2:3) with &12 and caninput_selxa(bits 0:1) with &3
      uint8_t pinNumber = pinTranslateAnalog(configPage9.Auxinpina[currentStatus.current_caninchannel]&63U);
      if( pinIsUsed(pinNumber) )
      {
        //Do nothing here as the pin is already in use.
        BIT_SET(currentStatus.engineProtectStatus, PROTECT_IO_ERROR); //Tell user that there is problem by lighting up the I/O error indicator
      }
      else
      {
        //Channel is active and analog
        pinMode( pinNumber, INPUT);
        //currentStatus.canin[14] = 33;  Dev test use only!
        auxIsEnabled = true;
      }  
    }
    else if ((((configPage9.enable_secondarySerial == 1U) || ((configPage9.enable_intcan == 1U) && (configPage9.intcan_available == 1U))) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&12U) == 12U)
            || (((configPage9.enable_secondarySerial == 0U) && ( (configPage9.enable_intcan == 1U) && (configPage9.intcan_available == 0U) )) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&3U) == 3U)
            || (((configPage9.enable_secondarySerial == 0U) && (configPage9.enable_intcan == 0U)) && ((configPage9.caninput_sel[currentStatus.current_caninchannel]&3U) == 3U)))
    {  //if current input channel is enabled as digital local pin check caninput_selxb(bits 2:3) with &12 and caninput_selxa(bits 0:1) with &3
       uint8_t pinNumber = (configPage9.Auxinpinb[currentStatus.current_caninchannel]&63U) + 1U;
       if( pinIsUsed(pinNumber) )
       {
         //Do nothing here as the pin is already in use.
        BIT_SET(currentStatus.engineProtectStatus, PROTECT_IO_ERROR); //Tell user that there is problem by lighting up the I/O error indicator
       }
       else
       {
         //Channel is active and digital
         pinMode( pinNumber, INPUT);
         //currentStatus.canin[14] = 44;  Dev test use only!
         auxIsEnabled = true;
       }  
    }
    else {
      //  Do nothing. Keep MISRA checker happy
    }
  } //For loop iterating through aux in lines
  

  //Sanity checks to ensure none of the filter values are set above 240 (Which would include the 255 value which is the default on a new arduino)
  //If an invalid value is detected, it's reset to the default the value and burned to EEPROM. 
  //Each sensor has it's own default value
  if(configPage4.ADCFILTER_TPS  > 240U) { configPage4.ADCFILTER_TPS   = ADCFILTER_TPS_DEFAULT;   writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_CLT  > 240U) { configPage4.ADCFILTER_CLT   = ADCFILTER_CLT_DEFAULT;   writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_IAT  > 240U) { configPage4.ADCFILTER_IAT   = ADCFILTER_IAT_DEFAULT;   writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_O2   > 240U) { configPage4.ADCFILTER_O2    = ADCFILTER_O2_DEFAULT;    writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_BAT  > 240U) { configPage4.ADCFILTER_BAT   = ADCFILTER_BAT_DEFAULT;   writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_MAP  > 240U) { configPage4.ADCFILTER_MAP   = ADCFILTER_MAP_DEFAULT;   writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_BARO > 240U) { configPage4.ADCFILTER_BARO  = ADCFILTER_BARO_DEFAULT;  writeConfig(ignSetPage); }
  if(configPage4.FILTER_FLEX    > 240U) { configPage4.FILTER_FLEX     = FILTER_FLEX_DEFAULT;     writeConfig(ignSetPage); }

  flexStartTime = micros();

  vssIndex = 0;
}


// ==========================================  MAP ==========================================


static constexpr uint16_t VALID_MAP_MAX=1022U; //The largest ADC value that is valid for the MAP sensor
static constexpr uint16_t VALID_MAP_MIN=2U; //The smallest ADC value that is valid for the MAP sensor

static inline void validateMAP(void)
{
  //Error checks
  if((uint16_t)currentStatus.MAP < VALID_MAP_MIN)
  {
    currentStatus.MAP = ERR_DEFAULT_MAP_LOW;
    (void)setError(ERR_MAP_LOW);
  }
  else if((uint16_t)currentStatus.MAP > VALID_MAP_MAX)
  {
    currentStatus.MAP = ERR_DEFAULT_MAP_HIGH;
    (void)setError(ERR_MAP_HIGH);
  }
  else
  {
    if(errorCount > 0U)
    {
      clearError(ERR_MAP_HIGH);
      clearError(ERR_MAP_LOW);
    }
  }
}

static inline bool isValidMapSensorReading(uint16_t reading) {
  return (reading < VALID_MAP_MAX) && (reading > VALID_MAP_MIN);  
}

static inline uint16_t readFilteredMapADC(uint8_t pin, uint8_t alpha, uint16_t prior) {
  uint16_t tempReading = readMAPSensor(pin);

  //Error check
  if (isValidMapSensorReading(tempReading)) { 
    return LOW_PASS_FILTER(tempReading, alpha, prior);
  }
  return prior;
}

static inline uint16_t mapADCToMAP(uint16_t mapADC, int8_t mapMin, uint16_t mapMax) {
  int16_t mapped = fastMap10Bit(mapADC, mapMin, mapMax); //Get the current MAP value
  return max((int16_t)0, mapped);  //Sanity check
}



static inline void resetMAPLast(map_last_read_t &lastRead) {
  //Update the calculation times and last value. These are used by the MAP based Accel enrich
  lastRead.lastMAPValue = currentStatus.MAP;
  lastRead.previousReadingTime = lastRead.lastReadingTime;
  lastRead.lastReadingTime = micros();
}

static inline void instanteneousMAPReading(map_last_read_t &lastReading)
{
  resetMAPLast(lastReading);

  currentStatus.mapADC = readFilteredMapADC(pinMAP, configPage4.ADCFILTER_MAP, currentStatus.mapADC);
  currentStatus.MAP = mapADCToMAP(currentStatus.mapADC, configPage2.mapMin, configPage2.mapMax); //Get the current MAP value
  validateMAP();
  
  //Repeat for EMAP if it's enabled
  if(configPage6.useEMAP == true)
  {
    currentStatus.EMAPADC = readFilteredMapADC(pinEMAP, configPage4.ADCFILTER_MAP, currentStatus.EMAPADC);
    currentStatus.EMAP = mapADCToMAP(currentStatus.EMAPADC, configPage2.EMAPMin, configPage2.EMAPMax);
  }
}

struct map_cycle_average_t {
  uint32_t curRev;
  uint32_t mapAdcRunningTotal;
  uint32_t emapAdcRunningTotal;
  uint16_t sampleCount;
};

static inline void cycleAverageMAPReadingAccumulate(map_cycle_average_t &cycle_average) {
  cycle_average.sampleCount++;

  currentStatus.mapADC = readFilteredMapADC(pinMAP, configPage4.ADCFILTER_MAP, currentStatus.mapADC);
  cycle_average.mapAdcRunningTotal += currentStatus.mapADC; //Add the current reading onto the total

  //Repeat for EMAP if it's enabled
  if(configPage6.useEMAP == true)
  {
    currentStatus.EMAPADC = readFilteredMapADC(pinEMAP, configPage4.ADCFILTER_MAP, currentStatus.EMAPADC);
    cycle_average.emapAdcRunningTotal += currentStatus.EMAPADC; //Add the current reading onto the totak
  }
}

static inline void cycleAverageEndCycle(map_cycle_average_t &cycle_average, map_last_read_t &lastReading) {
  if( (cycle_average.mapAdcRunningTotal != 0UL) && (cycle_average.sampleCount != 0U) )
  {
    resetMAPLast(lastReading);

    currentStatus.mapADC = udiv_32_16(cycle_average.mapAdcRunningTotal, cycle_average.sampleCount);
    currentStatus.MAP = mapADCToMAP(currentStatus.mapADC, configPage2.mapMin, configPage2.mapMax); //Get the current MAP value
    validateMAP();

    //If EMAP is enabled, the process is identical to the above
    if(configPage6.useEMAP == true)
    {
      currentStatus.EMAPADC = udiv_32_16(cycle_average.emapAdcRunningTotal, cycle_average.sampleCount); //Note that the MAP count can be reused here as it will always be the same count.
      currentStatus.EMAP = mapADCToMAP(currentStatus.EMAPADC, configPage2.EMAPMin, configPage2.EMAPMax);
    }
  } else { 
    instanteneousMAPReading(lastReading); 
  }

  // Reset for next cycle.
  cycle_average.sampleCount = 0U;
  cycle_average.mapAdcRunningTotal = 0U;
  cycle_average.emapAdcRunningTotal = 0U;
  cycle_average.curRev = currentStatus.startRevolutions;
}

static inline bool isCycleCurrent(uint32_t curRev) {
  ATOMIC() {
    return (curRev == currentStatus.startRevolutions) || ((curRev+1U) == currentStatus.startRevolutions);
  }
  return false; // Just here to avoid compiler warning.
}

static inline bool isCycleCurrent(const map_cycle_average_t &cycle_avg) {
  return isCycleCurrent(cycle_avg.curRev);
}

static inline void cycleAverageMAPReading(map_cycle_average_t &cycle_average, map_last_read_t &lastReading) {
  //If the engine isn't running and RPM below switch point, fall back to instantaneous reads
  if ( (currentStatus.RPMdiv100 > configPage2.mapSwitchPoint) && HasAnySync(currentStatus) && (currentStatus.startRevolutions > 1U) )
  {
    //2 revolutions are looked at for 4 stroke. 2 stroke not currently catered for.
    if( isCycleCurrent(cycle_average) ) {
      cycleAverageMAPReadingAccumulate(cycle_average);
    } else {
      //Reaching here means that the last cycle has completed and the MAP value should be calculated
      cycleAverageEndCycle(cycle_average, lastReading);
    }
  }
  else 
  {
    instanteneousMAPReading(lastReading);
    cycle_average.mapAdcRunningTotal = currentStatus.mapADC; //Keep updating the MAPrunningValue to give it head start when switching to cycle average.
    if(configPage6.useEMAP == true)
    {
      cycle_average.emapAdcRunningTotal = currentStatus.EMAPADC;
    }
    cycle_average.sampleCount = 1U;
  }
}

struct map_cycle_min_t {
  uint32_t curRev;
  uint16_t mapMinimum;
};

static inline void cycleMinimumAccumulate(map_cycle_min_t &cycle_min) {
  uint16_t tempReading = readMAPSensor(pinMAP);
  //Error check
  if( isValidMapSensorReading(tempReading) ) {
    //Check whether the current reading is lower than the running minimum
    cycle_min.mapMinimum = min(tempReading, cycle_min.mapMinimum); 
  }
}

static inline void cycleMinimumEndCycle(map_cycle_min_t &cycle_min, map_last_read_t &lastReading) {
  resetMAPLast(lastReading);

  currentStatus.mapADC = cycle_min.mapMinimum;
  currentStatus.MAP = mapADCToMAP(currentStatus.mapADC, configPage2.mapMin, configPage2.mapMax); //Get the current MAP value

  validateMAP();

  cycle_min.curRev = currentStatus.startRevolutions; //Reset the current rev count
  cycle_min.mapMinimum = VALID_MAP_MAX; //Reset the latest value so the next reading will always be lower
}

static inline bool isCycleCurrent(const map_cycle_min_t &cycle_min) {
  return isCycleCurrent(cycle_min.curRev);
}

static inline void cycleMinimumMAPReading(map_cycle_min_t &cycle_min, map_last_read_t &lastReading) {
  //If the engine isn't running and RPM below switch point, fall back to instantaneous reads
  if (currentStatus.RPMdiv100 > configPage2.mapSwitchPoint) 
  {
    //2 revolutions are looked at for 4 stroke. 2 stroke not currently catered for.
    if ( isCycleCurrent(cycle_min) ) {
      cycleMinimumAccumulate(cycle_min);
    } else {
      //Reaching here means that the last cycle has completed and the MAP value should be calculated
      cycleMinimumEndCycle(cycle_min, lastReading);
    }
  }
  else 
  {
    instanteneousMAPReading(lastReading);
    cycle_min.mapMinimum = currentStatus.mapADC;  //Keep updating the MAPrunningValue to give it head start when switching to cycle minimum.
  }
}

struct map_event_average_t {
  uint32_t mapAdcRunningTotal;
  uint16_t sampleCount;
  uint32_t currentIgnitionEventIndex;
};

static inline void eventAverageAccumulate(map_event_average_t &eventAverage) {
  currentStatus.mapADC = readFilteredMapADC(pinMAP, configPage4.ADCFILTER_MAP, currentStatus.mapADC);
  eventAverage.mapAdcRunningTotal += currentStatus.mapADC; //Add the current reading onto the total
  eventAverage.sampleCount++;
}

static inline bool isIgnitionEventValid(const map_event_average_t &eventAverage) {
  ATOMIC() {
    return (eventAverage.currentIgnitionEventIndex < ignitionCount);
  }
  return false; // Just here to avoid compiler warning.
}

static inline void eventAverageEndEvent(map_event_average_t &eventAverage, map_last_read_t &lastReading) {
  //Reaching here means that the next ignition event has occurred and the MAP value should be calculated
  //Sanity check
  if( (eventAverage.mapAdcRunningTotal != 0U) && (eventAverage.sampleCount != 0U) && isIgnitionEventValid(eventAverage) )
  {
    resetMAPLast(lastReading);

    currentStatus.mapADC = udiv_32_16(eventAverage.mapAdcRunningTotal, eventAverage.sampleCount);
    currentStatus.MAP = mapADCToMAP(currentStatus.mapADC, configPage2.mapMin, configPage2.mapMax); //Get the current MAP value
    validateMAP();
  }
  else { 
    instanteneousMAPReading(lastReading); 
  }

  // Reset for next cycle.
  eventAverage.mapAdcRunningTotal = 0U;
  eventAverage.sampleCount = 0U;
  eventAverage.currentIgnitionEventIndex = ignitionCount;
}

static inline bool isIgnitionEventCurrent(const map_event_average_t &eventAverage) {
  ATOMIC() {
    return (eventAverage.currentIgnitionEventIndex == ignitionCount);
  }
  return false; // Just here to avoid compiler warning.
}

static inline void eventAverageMAPReading(map_event_average_t &eventAverage, map_last_read_t &lastReading) {
  //Average of an ignition event
  if ( (currentStatus.RPMdiv100 > configPage2.mapSwitchPoint) && HasAnySync(currentStatus) && (currentStatus.startRevolutions > 1U) && (! currentStatus.engineProtectStatus) ) //If the engine isn't running, fall back to instantaneous reads
  {
    if( isIgnitionEventCurrent(eventAverage) ) { //Watch for a change in the ignition counter to determine whether we're still on the same event
      eventAverageAccumulate(eventAverage);
    } else {
      eventAverageEndEvent(eventAverage, lastReading);
    }
  } else {
    instanteneousMAPReading(lastReading);
    eventAverage.mapAdcRunningTotal = currentStatus.mapADC; //Keep updating the MAPrunningValue to give it head start when switching to ignition event average.
    eventAverage.sampleCount = 1;
  }
}

struct map_algorithm_t {
  map_last_read_t lastReading;

  union {
    map_cycle_average_t cycle_average;
    map_cycle_min_t cycle_min;
    map_event_average_t event_average;
  };
};
static map_algorithm_t mapAlgorithmState;

#if defined(UNIT_TEST)
map_last_read_t& getMapLast(void){
  return mapAlgorithmState.lastReading;
}
#endif

static void initialiseMAP(void) {
  (void)memset(&mapAlgorithmState, 0, sizeof(mapAlgorithmState));
}

void readMAP(void)
{
  //MAP Sampling system
  switch(configPage2.mapSample)
  {
    case 1:
      //Average of a cycle
      cycleAverageMAPReading(mapAlgorithmState.cycle_average, mapAlgorithmState.lastReading);
      break;

    case 2:
      //Minimum reading in a cycle
      cycleMinimumMAPReading(mapAlgorithmState.cycle_min, mapAlgorithmState.lastReading);
      break;

    case 3:
      //Average of an ignition event
      eventAverageMAPReading(mapAlgorithmState.event_average, mapAlgorithmState.lastReading);
      break; 

    case 0:
    default:
      instanteneousMAPReading(mapAlgorithmState.lastReading);
    break;
  }
}

/** @brief Get the MAP change between the last 2 readings */
int16_t getMAPDelta(void) {
  return (int16_t)currentStatus.MAP - (int16_t)mapAlgorithmState.lastReading.lastMAPValue;
}

/** @brief Get the time in µS between the last 2 MAP readings */
uint32_t getMAPDeltaTime(void) {
  return mapAlgorithmState.lastReading.lastReadingTime - mapAlgorithmState.lastReading.previousReadingTime;
}

// ========================================== TPS ==========================================

void readTPS(bool useFilter)
{
  currentStatus.TPSlast = currentStatus.TPS;
  uint8_t tempTPS = (uint8_t)fastMap10Bit(readAnalogSensor(pinTPS), 0U, 255U); //Get the current raw TPS ADC value and map it into a uint8_t

  //The use of the filter can be overridden if required. This is used on startup to disable priming pulse if flood clear is wanted
  if(useFilter == true) { currentStatus.tpsADC = (uint8_t)LOW_PASS_FILTER((uint16_t)tempTPS, configPage4.ADCFILTER_TPS, (uint16_t)currentStatus.tpsADC); }
  else { currentStatus.tpsADC = tempTPS; }
  uint8_t tempADC = currentStatus.tpsADC; //The tempADC value is used in order to allow TunerStudio to recover and redo the TPS calibration if this somehow gets corrupted

  if(configPage2.tpsMax > configPage2.tpsMin)
  {
    //Check that the ADC values fall within the min and max ranges (Should always be the case, but noise can cause these to fluctuate outside the defined range).
    tempADC = clamp(tempADC, configPage2.tpsMin, configPage2.tpsMax);
    currentStatus.TPS = map(tempADC, configPage2.tpsMin, configPage2.tpsMax, 0, 200); //Take the raw TPS ADC value and convert it into a TPS% based on the calibrated values
  }
  else
  {
    //This case occurs when the TPS +5v and gnd are wired backwards, but the user wishes to retain this configuration.
    //In such a case, tpsMin will be greater then tpsMax and hence checks and mapping needs to be reversed

    tempADC = 255U - tempADC; //Reverse the ADC values
    uint8_t tempTPSMax = 255U - configPage2.tpsMax;
    uint8_t tempTPSMin = 255U - configPage2.tpsMin;

    //All checks below are reversed from the standard case above
    tempADC = clamp(tempADC, tempTPSMin, tempTPSMax);
    currentStatus.TPS = map(tempADC, tempTPSMin, tempTPSMax, 0, 200);
  }

  //Check whether the closed throttle position sensor is active
  if(configPage2.CTPSEnabled == true)
  {
    if(configPage2.CTPSPolarity == 0U) { currentStatus.CTPSActive = !digitalRead(pinCTPS); } //Normal mode (ground switched)
    else { currentStatus.CTPSActive = digitalRead(pinCTPS); } //Inverted mode (5v activates closed throttle position sensor)
  }
  else { currentStatus.CTPSActive = 0; }
}

void readCLT(bool useFilter)
{
  uint16_t tempReading = readAnalogSensor(pinCLT);
  //The use of the filter can be overridden if required. This is used on startup so there can be an immediately accurate coolant value for priming
  if(useFilter == true) { currentStatus.cltADC = LOW_PASS_FILTER(tempReading, configPage4.ADCFILTER_CLT, currentStatus.cltADC); }
  else { currentStatus.cltADC = tempReading; }
  
  currentStatus.coolant = table2D_getValue(&cltCalibrationTable, currentStatus.cltADC) - CALIBRATION_TEMPERATURE_OFFSET; //Temperature calibration values are stored as positive bytes. We subtract 40 from them to allow for negative temperatures
}

void readIAT(void)
{
  currentStatus.iatADC = LOW_PASS_FILTER(readAnalogSensor(pinIAT), configPage4.ADCFILTER_IAT, currentStatus.iatADC);
  currentStatus.IAT = table2D_getValue(&iatCalibrationTable, currentStatus.iatADC) - CALIBRATION_TEMPERATURE_OFFSET;
}

// ========================================== Baro ==========================================

/* 
* The highest sea-level pressure on Earth occurs in Siberia, where the Siberian High often attains a sea-level pressure above 105 kPa;
* with record highs close to 108.5 kPa.
* The lowest possible baro reading is based on an altitude of 3500m above sea level.
*/
static inline bool isValidBaro(uint8_t baro) {
static constexpr uint16_t BARO_MIN = 65U;
static constexpr uint16_t BARO_MAX = 108U;

  return (baro >= BARO_MIN) && (baro <= BARO_MAX);
}

static inline void setBaroFromSensorReading(uint16_t sensorReading) {
  currentStatus.baroADC = sensorReading;
  int16_t tempValue = fastMap10Bit(currentStatus.baroADC, configPage2.baroMin, configPage2.baroMax);
  currentStatus.baro = (uint8_t)max((int16_t)0, tempValue);
}

// Should only be called when the engine isn't running.
static inline void setBaroFromMAP(void) {
  uint16_t tempReading = mapADCToMAP(readMAPSensor(pinMAP), configPage2.mapMin, configPage2.mapMax);
  if (isValidBaro(tempReading)) //Safety check to ensure the baro reading is within the physical limits
  {
    currentStatus.baro = tempReading;
    storeLastBaro(currentStatus.baro);
  }
}

void readBaro(void)
{
  if ( configPage6.useExtBaro != 0U  ) {
    // readings
    setBaroFromSensorReading(LOW_PASS_FILTER(readMAPSensor(pinBaro), configPage4.ADCFILTER_BARO, currentStatus.baroADC)); //Very weak filter
  // If no dedicated baro sensor is available, attempt to get a reading from the MAP sensor. This can only be done if the engine is not running. 
  } else if ((currentStatus.RPM == 0U) && !engineIsRunning(micros()-MICROS_PER_SEC)) {
    setBaroFromMAP();
  } else {
    // Do nothing - baro remains at last read value & MISRA checker is kept happy.
  }
}

static inline void initialiseBaro(void) {
  if ( configPage6.useExtBaro != 0U  )
  {
    // Use raw unfiltered value initially
    setBaroFromSensorReading(readMAPSensor(pinBaro));
  }
  else
  {
    //Attempt to use the last known good baro reading from EEPROM as a starting point
    uint8_t lastBaro = readLastBaro();
    // Make sure it's not invalid (Possible on first run etc)
    currentStatus.baro = isValidBaro(lastBaro) ? lastBaro : 100U;
    // We assume external callers already made sure the engine isn't running
    setBaroFromMAP();
  }
}

void initialiseMAPBaro(void) {
  initialiseMAP();
  initialiseBaro();
}

// ========================================== O2 ==========================================

void readO2(void)
{
  //An O2 read is only performed if an O2 sensor type is selected. This is to prevent potentially dangerous use of the O2 readings prior to proper setup/calibration
  if(configPage6.egoType > 0U)
  {
    currentStatus.O2ADC = LOW_PASS_FILTER(readAnalogSensor(pinO2), configPage4.ADCFILTER_O2, currentStatus.O2ADC);
    currentStatus.O2 = table2D_getValue(&o2CalibrationTable, currentStatus.O2ADC);
  }
  else
  {
    currentStatus.O2ADC = 0U;
    currentStatus.O2 = 0U;
  }
  
}

void readO2_2(void)
{
  //Second O2 currently disabled as its not being used
  //Get the current O2 value.
  currentStatus.O2_2ADC = LOW_PASS_FILTER(readAnalogSensor(pinO2_2), configPage4.ADCFILTER_O2, currentStatus.O2_2ADC);
  currentStatus.O2_2 = table2D_getValue(&o2CalibrationTable, currentStatus.O2_2ADC);
}

void readBat(void)
{
  int16_t tempReading = fastMap10Bit(readAnalogSensor(pinBat), 0, 245); //Get the current raw Battery value. Permissible values are from 0v to 24.5v (245)

  //Apply the offset calibration value to the reading
  tempReading += configPage4.batVoltCorrect;
  if(tempReading < 0){
    tempReading=0;
  }  //with negative overflow prevention


  //The following is a check for if the voltage has jumped up from under 5.5v to over 7v.
  //If this occurs, it's very likely that the system has gone from being powered by USB to being powered from the 12v power source.
  //Should that happen, we re-trigger the fuel pump priming and idle homing (If using a stepper)
  if( (currentStatus.battery10 < 55U) && (tempReading > 70) && (currentStatus.RPM == 0U) )
  {
    //Re-prime the fuel pump
    fpPrimeTime = currentStatus.secl;
    currentStatus.fpPrimed = false;
    FUEL_PUMP_ON();

    //Redo the stepper homing
    if( (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_CL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OL) )
    {
      initialiseIdle(true);
    }
  }

  currentStatus.battery10 = LOW_PASS_FILTER(tempReading, configPage4.ADCFILTER_BAT, currentStatus.battery10);
}

/**
 * @brief Returns the VSS pulse gap for a given history point
 * 
 * @param historyIndex The gap number that is wanted. EG:
 * historyIndex = 0 = Latest entry
 * historyIndex = 1 = 2nd entry entry
 */
uint32_t vssGetPulseGap(uint8_t historyIndex)
{
  uint32_t tempGap = 0;
  
  noInterrupts();
  int8_t tempIndex = vssIndex - historyIndex;
  if(tempIndex < 0) { tempIndex += (int8_t)VSS_SAMPLES; }

  if(tempIndex > 0) { tempGap = vssTimes[tempIndex] - vssTimes[tempIndex - 1]; }
  else { tempGap = vssTimes[0] - vssTimes[(VSS_SAMPLES-1U)]; }
  interrupts();

  return tempGap;
}

uint16_t getSpeed(void)
{
  uint16_t tempSpeed = 0;
  // Get VSS from CAN, Serial or Analog by using Aux input channels.
  if(configPage2.vssMode == 1U)
  {
    // Direct reading from Aux channel
    if (configPage2.vssPulsesPerKm == 0U)
    {
      tempSpeed = currentStatus.canin[configPage2.vssAuxCh];
    }
    // Adjust the reading by dividing it by set amount.
    else
    {
      tempSpeed = (currentStatus.canin[configPage2.vssAuxCh] / configPage2.vssPulsesPerKm);
    }
    tempSpeed = LOW_PASS_FILTER(tempSpeed, configPage2.vssSmoothing, currentStatus.vss); //Apply speed smoothing factor
  }
  // Interrupt driven mode
  else if(configPage2.vssMode > 1U)
  {
    uint32_t pulseTime = 0U;
    uint32_t vssTotalTime = 0U;

    //Add up the time between the teeth. Note that the total number of gaps is equal to the number of samples minus 1
    for(uint8_t x = 0U; x<(VSS_SAMPLES-1U); x++)
    {
      vssTotalTime += vssGetPulseGap(x);
    }

    pulseTime = vssTotalTime / ((uint32_t)VSS_SAMPLES - 1UL);
    if ( (micros() - vssTimes[vssIndex]) > MICROS_PER_SEC ) { tempSpeed = 0; } // Check that the car hasn't come to a stop. Is true if last pulse was more than 1 second ago
    else 
    {
      tempSpeed = MICROS_PER_HOUR / (pulseTime * configPage2.vssPulsesPerKm); //Convert the pulse gap into km/h
      tempSpeed = LOW_PASS_FILTER(tempSpeed, configPage2.vssSmoothing, currentStatus.vss); //Apply speed smoothing factor
    }
    if(tempSpeed > 1000U) { tempSpeed = currentStatus.vss; } //Safety check. This usually occurs when there is a hardware issue
  } else {
    // Do nothing
  }
  return tempSpeed;
}

byte getGear(void)
{
  byte tempGear = 0U; //Unknown gear
  if(currentStatus.vss > 0U)
  {
    //If the speed is non-zero, default to the last calculated gear
    tempGear = currentStatus.gear;

    uint16_t pulsesPer1000rpm = udiv_32_16(currentStatus.vss * 10000UL, currentStatus.RPM); //Gives the current pulses per 1000RPM, multiplied by 10 (10x is the multiplication factor for the ratios in TS)
    //Begin gear detection
    if( (pulsesPer1000rpm > (configPage2.vssRatio1 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio1 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 1; }
    else if( (pulsesPer1000rpm > (configPage2.vssRatio2 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio2 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 2; }
    else if( (pulsesPer1000rpm > (configPage2.vssRatio3 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio3 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 3; }
    else if( (pulsesPer1000rpm > (configPage2.vssRatio4 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio4 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 4; }
    else if( (pulsesPer1000rpm > (configPage2.vssRatio5 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio5 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 5; }
    else if( (pulsesPer1000rpm > (configPage2.vssRatio6 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio6 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 6; }
    else {
      // Do nothing 
    }
  }
  
  return tempGear;
}

byte getFuelPressure(void)
{
  int16_t tempFuelPressure = 0;

  if(configPage10.fuelPressureEnable > 0U)
  {
    tempFuelPressure = fastMap10Bit(readAnalogSensor(pinFuelPressure), configPage10.fuelPressureMin, configPage10.fuelPressureMax);
    tempFuelPressure = LOW_PASS_FILTER(tempFuelPressure, ADCFILTER_PSI_DEFAULT, currentStatus.fuelPressure); //Apply smoothing factor
    //Sanity checks
    tempFuelPressure = clamp(tempFuelPressure, (int16_t)0, (int16_t)configPage10.fuelPressureMax);
  }

  return (byte)tempFuelPressure;
}

byte getOilPressure(void)
{
  int16_t tempOilPressure = 0;

  if(configPage10.oilPressureEnable > 0U)
  {
    //Perform ADC read
    tempOilPressure = fastMap10Bit(readAnalogSensor(pinOilPressure), configPage10.oilPressureMin, configPage10.oilPressureMax);
    tempOilPressure = LOW_PASS_FILTER(tempOilPressure, ADCFILTER_PSI_DEFAULT, currentStatus.oilPressure); //Apply smoothing factor
    //Sanity check
    tempOilPressure = clamp(tempOilPressure, (int16_t)0, (int16_t)configPage10.oilPressureMax);
  }


  return (byte)tempOilPressure;
}

uint8_t getAnalogKnock(void)
{
  uint8_t pinKnock = A15; //Default value incase the user has not selected an analog pin in TunerStudio
  if(configPage10.knock_pin >=47U)
  {
    pinKnock = pinTranslateAnalog(configPage10.knock_pin - 47U); //The knock_pin varialbe has both digital and analog pins listed. A0 is at position 47
  }

  //Perform ADC read
  return (uint8_t)fastMap10Bit(readAnalogSensor(pinKnock), 0U, 255U);
}

/*
 * The interrupt function for reading the flex sensor frequency and pulse width
 * flexCounter value is incremented with every pulse and reset back to 0 once per second
 */
void flexPulse(void)
{
  if(READ_FLEX() == true)
  {
    uint16_t tempPW = clamp(micros() - flexStartTime, 0UL, (unsigned long)UINT16_MAX); //Calculate the pulse width
    flexPulseWidth = LOW_PASS_FILTER(tempPW, configPage4.FILTER_FLEX, flexPulseWidth);
    ++flexCounter;
  }
  else
  {
    flexStartTime = micros(); //Start pulse width measurement.
  }
}

/*
 * The interrupt function for pulses from a knock conditioner / controller
 * 
 */
void knockPulse(void)
{
  if( (currentStatus.MAP < (configPage10.knock_maxMAP*2U)) && (currentStatus.RPMdiv100 < configPage10.knock_maxRPM) )
  {
    if(!BIT_CHECK(currentStatus.status5, BIT_STATUS5_KNOCK_ACTIVE)) { currentStatus.knockCount++; } //If knock is not currently active we count every pulse. If knock is already active then additional pulses will be counted in correctionKnockTiming()
    BIT_SET(currentStatus.status5, BIT_STATUS5_KNOCK_PULSE);
  }
}

/**
 * @brief The ISR function for VSS pulses
 * 
 */
void vssPulse(void)
{
  //TODO: Add basic filtering here
  vssIndex++;
  if(vssIndex == VSS_SAMPLES) { vssIndex = 0U; }

  vssTimes[vssIndex] = micros();
}

// Read the Aux analog value for pin set by analogPin 
uint16_t readAuxanalog(uint8_t analogPin)
{
  return readAnalogSensor(analogPin); // readAnalogSensor is inlined within this CPP file.
} 

uint16_t readAuxdigital(uint8_t digitalPin)
{
  //read the Aux digital value for pin set by digitalPin 
  unsigned int tempReading;
  tempReading = digitalRead(digitalPin); 
  return tempReading;
} 
