/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
/** @file
 * Read sensors with appropriate timing / scheduling.
 */
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

/** Init all ADC conversions by setting resolutions, etc.
 */
void initialiseADC()
{
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) //AVR chips use the ISR for this

  #if defined(ANALOG_ISR)
    //This sets the ADC (Analog to Digitial Converter) to run at 250KHz, greatly reducing analog read times (MAP/TPS)
    //the code on ISR run each conversion every 25 ADC clock, conversion run about 100KHz effectively
    //making a 6250 conversions/s on 16 channels and 12500 on 8 channels devices.
    noInterrupts(); //Interrupts should be turned off when playing with any of these registers

    ADCSRB = 0x00; //ADC Auto Trigger Source is in Free Running mode
    ADMUX = 0x40;  //Select AREF as reference, ADC Left Adjust Result, Starting at channel 0

    //All of the below is the longhand version of: ADCSRA = 0xEE;
    #define ADFR 5 //Why the HELL isn't this defined in the same place as everything else (wiring.h)?!?!
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
    //This sets the ADC (Analog to Digitial Converter) to run at 1Mhz, greatly reducing analog read times (MAP/TPS) when using the standard analogRead() function
    //1Mhz is the fastest speed permitted by the CPU without affecting accuracy
    //Please see chapter 11 of 'Practical Arduino' (books.google.com.au/books?id=HsTxON1L6D4C&printsec=frontcover#v=onepage&q&f=false) for more detail
     BIT_SET(ADCSRA,ADPS2);
     BIT_CLEAR(ADCSRA,ADPS1);
     BIT_CLEAR(ADCSRA,ADPS0);
  #endif
#elif defined(ARDUINO_ARCH_STM32) //STM32GENERIC core and ST STM32duino core, change analog read to 12 bit
  analogReadResolution(10); //use 10bits for analog reading on STM32 boards
#endif
  MAPcurRev = 0;
  MAPcount = 0;
  MAPrunningValue = 0;

  //The following checks the aux inputs and initialises pins if required
  auxIsEnabled = false;
  for (byte AuxinChan = 0; AuxinChan <16 ; AuxinChan++)
  {
    currentStatus.current_caninchannel = AuxinChan;                   
    if (((configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 4)
    && ((configPage9.enable_secondarySerial == 1) || ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1))))
    { //if current input channel is enabled as external input in caninput_selxb(bits 2:3) and secondary serial or internal canbus is enabled(and is mcu supported)                 
      //currentStatus.canin[14] = 22;  Dev test use only!
      auxIsEnabled = true;     
    }
    else if ((((configPage9.enable_secondarySerial == 1) || ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1))) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 8)
            || (((configPage9.enable_secondarySerial == 0) && ( (configPage9.enable_intcan == 1) && (configPage9.intcan_available == 0) )) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 2)  
            || (((configPage9.enable_secondarySerial == 0) && (configPage9.enable_intcan == 0)) && ((configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 2)))  
    {  //if current input channel is enabled as analog local pin check caninput_selxb(bits 2:3) with &12 and caninput_selxa(bits 0:1) with &3
      byte pinNumber = (configPage9.Auxinpina[currentStatus.current_caninchannel]&127);
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
    else if ((((configPage9.enable_secondarySerial == 1) || ((configPage9.enable_intcan == 1) && (configPage9.intcan_available == 1))) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&12) == 12)
            || (((configPage9.enable_secondarySerial == 0) && ( (configPage9.enable_intcan == 1) && (configPage9.intcan_available == 0) )) && (configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 3)
            || (((configPage9.enable_secondarySerial == 0) && (configPage9.enable_intcan == 0)) && ((configPage9.caninput_sel[currentStatus.current_caninchannel]&3) == 3)))
    {  //if current input channel is enabled as digital local pin check caninput_selxb(bits 2:3) wih &12 and caninput_selxa(bits 0:1) with &3
       byte pinNumber = (configPage9.Auxinpinb[currentStatus.current_caninchannel]&63) + 1;
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
  } //For loop iterating through aux in lines

  //Sanity checks to ensure none of the filter values are set above 240 (Which would include the 255 value which is the default on a new arduino)
  //If an invalid value is detected, it's reset to the default the value and burned to EEPROM. 
  //Each sensor has it's own default value
  if(configPage4.ADCFILTER_TPS  > 240) { configPage4.ADCFILTER_TPS   = ADCFILTER_TPS_DEFAULT;   writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_CLT  > 240) { configPage4.ADCFILTER_CLT   = ADCFILTER_CLT_DEFAULT;   writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_IAT  > 240) { configPage4.ADCFILTER_IAT   = ADCFILTER_IAT_DEFAULT;   writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_O2   > 240) { configPage4.ADCFILTER_O2    = ADCFILTER_O2_DEFAULT;    writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_BAT  > 240) { configPage4.ADCFILTER_BAT   = ADCFILTER_BAT_DEFAULT;   writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_MAP  > 240) { configPage4.ADCFILTER_MAP   = ADCFILTER_MAP_DEFAULT;   writeConfig(ignSetPage); }
  if(configPage4.ADCFILTER_BARO > 240) { configPage4.ADCFILTER_BARO  = ADCFILTER_BARO_DEFAULT;  writeConfig(ignSetPage); }
  if(configPage4.FILTER_FLEX    > 240) { configPage4.FILTER_FLEX     = FILTER_FLEX_DEFAULT;     writeConfig(ignSetPage); }

  flexStartTime = micros();

  vssIndex = 0;
}

static inline void validateMAP()
{
  //Error checks
  if(currentStatus.MAP < VALID_MAP_MIN)
  {
    currentStatus.MAP = ERR_DEFAULT_MAP_LOW;
    mapErrorCount += 1;
    setError(ERR_MAP_LOW);
  }
  else if(currentStatus.MAP > VALID_MAP_MAX)
  {
    currentStatus.MAP = ERR_DEFAULT_MAP_HIGH;
    mapErrorCount += 1;
    setError(ERR_MAP_HIGH);
  }
  else
  {
    if(errorCount > 0)
    {
      clearError(ERR_MAP_HIGH);
      clearError(ERR_MAP_LOW);
    }
    mapErrorCount = 0;
  }
}

static inline void instanteneousMAPReading()
{
  //Update the calculation times and last value. These are used by the MAP based Accel enrich
  MAPlast = currentStatus.MAP;
  MAPlast_time = MAP_time;
  MAP_time = micros();

  unsigned int tempReading;
  //Instantaneous MAP readings
  #if defined(ANALOG_ISR_MAP)
    tempReading = AnChannel[pinMAP-A0];
  #else
    tempReading = analogRead(pinMAP);
    tempReading = analogRead(pinMAP);
  #endif
  //Error checking
  if( (tempReading >= VALID_MAP_MAX) || (tempReading <= VALID_MAP_MIN) ) { mapErrorCount += 1; }
  else { mapErrorCount = 0; }

  //During startup a call is made here to get the baro reading. In this case, we can't apply the ADC filter
  if(initialisationComplete == true) { currentStatus.mapADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_MAP, currentStatus.mapADC); } //Very weak filter
  else { currentStatus.mapADC = tempReading; } //Baro reading (No filter)

  currentStatus.MAP = fastMap10Bit(currentStatus.mapADC, configPage2.mapMin, configPage2.mapMax); //Get the current MAP value
  if(currentStatus.MAP < 0) { currentStatus.MAP = 0; } //Sanity check
  
  //Repeat for EMAP if it's enabled
  if(configPage6.useEMAP == true)
  {
    tempReading = analogRead(pinEMAP);
    tempReading = analogRead(pinEMAP);

    //Error check
    if( (tempReading < VALID_MAP_MAX) && (tempReading > VALID_MAP_MIN) )
      {
        currentStatus.EMAPADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_MAP, currentStatus.EMAPADC);
      }
    else { mapErrorCount += 1; }
    currentStatus.EMAP = fastMap10Bit(currentStatus.EMAPADC, configPage2.EMAPMin, configPage2.EMAPMax);
    if(currentStatus.EMAP < 0) { currentStatus.EMAP = 0; } //Sanity check
  }

}

static inline void readMAP()
{
  unsigned int tempReading;
  //MAP Sampling system
  switch(configPage2.mapSample)
  {
    case 0:
      //Instantaneous MAP readings
      instanteneousMAPReading();
      break;

    case 1:
      //Average of a cycle

      if ( (currentStatus.RPMdiv100 > configPage2.mapSwitchPoint) && (currentStatus.hasSync == true) && (currentStatus.startRevolutions > 1) ) //If the engine isn't running and RPM below switch point, fall back to instantaneous reads
      {
        if( (MAPcurRev == currentStatus.startRevolutions) || ( (MAPcurRev+1) == currentStatus.startRevolutions) ) //2 revolutions are looked at for 4 stroke. 2 stroke not currently catered for.
        {
          #if defined(ANALOG_ISR_MAP)
            tempReading = AnChannel[pinMAP-A0];
          #else
            tempReading = analogRead(pinMAP);
            tempReading = analogRead(pinMAP);
          #endif

          //Error check
          if( (tempReading < VALID_MAP_MAX) && (tempReading > VALID_MAP_MIN) )
          {
            currentStatus.mapADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_MAP, currentStatus.mapADC);
            MAPrunningValue += currentStatus.mapADC; //Add the current reading onto the total
            MAPcount++;
          }
          else { mapErrorCount += 1; }

          //Repeat for EMAP if it's enabled
          if(configPage6.useEMAP == true)
          {
            tempReading = analogRead(pinEMAP);
            tempReading = analogRead(pinEMAP);

            //Error check
            if( (tempReading < VALID_MAP_MAX) && (tempReading > VALID_MAP_MIN) )
            {
              currentStatus.EMAPADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_MAP, currentStatus.EMAPADC);
              EMAPrunningValue += currentStatus.EMAPADC; //Add the current reading onto the total
            }
            else { mapErrorCount += 1; }
          }
        }
        else
        {
          //Reaching here means that the last cylce has completed and the MAP value should be calculated
          //Sanity check
          if( (MAPrunningValue != 0) && (MAPcount != 0) )
          {
            //Update the calculation times and last value. These are used by the MAP based Accel enrich
            MAPlast = currentStatus.MAP;
            MAPlast_time = MAP_time;
            MAP_time = micros();

            currentStatus.mapADC = ldiv(MAPrunningValue, MAPcount).quot;
            currentStatus.MAP = fastMap10Bit(currentStatus.mapADC, configPage2.mapMin, configPage2.mapMax); //Get the current MAP value
            validateMAP();

            //If EMAP is enabled, the process is identical to the above
            if(configPage6.useEMAP == true)
            {
              currentStatus.EMAPADC = ldiv(EMAPrunningValue, MAPcount).quot; //Note that the MAP count can be reused here as it will always be the same count.
              currentStatus.EMAP = fastMap10Bit(currentStatus.EMAPADC, configPage2.EMAPMin, configPage2.EMAPMax);
              if(currentStatus.EMAP < 0) { currentStatus.EMAP = 0; } //Sanity check
            }
          }
          else { instanteneousMAPReading(); }

          MAPcurRev = currentStatus.startRevolutions; //Reset the current rev count
          MAPrunningValue = 0;
          EMAPrunningValue = 0; //Can reset this even if EMAP not used
          MAPcount = 0;
        }
      }
      else 
      {
        instanteneousMAPReading();
        MAPrunningValue = currentStatus.mapADC; //Keep updating the MAPrunningValue to give it head start when switching to cycle average.
        if(configPage6.useEMAP == true)
        {
          EMAPrunningValue = currentStatus.EMAPADC;
        }
        MAPcount = 1;
      }
      break;

    case 2:
      //Minimum reading in a cycle
      if (currentStatus.RPMdiv100 > configPage2.mapSwitchPoint) //If the engine isn't running and RPM below switch point, fall back to instantaneous reads
      {
        if( (MAPcurRev == currentStatus.startRevolutions) || ((MAPcurRev+1) == currentStatus.startRevolutions) ) //2 revolutions are looked at for 4 stroke. 2 stroke not currently catered for.
        {
          #if defined(ANALOG_ISR_MAP)
            tempReading = AnChannel[pinMAP-A0];
          #else
            tempReading = analogRead(pinMAP);
            tempReading = analogRead(pinMAP);
          #endif
          //Error check
          if( (tempReading < VALID_MAP_MAX) && (tempReading > VALID_MAP_MIN) )
          {
            if( (unsigned long)tempReading < MAPrunningValue ) { MAPrunningValue = (unsigned long)tempReading; } //Check whether the current reading is lower than the running minimum
          }
          else { mapErrorCount += 1; }
        }
        else
        {
          //Reaching here means that the last cylce has completed and the MAP value should be calculated

          //Update the calculation times and last value. These are used by the MAP based Accel enrich
          MAPlast = currentStatus.MAP;
          MAPlast_time = MAP_time;
          MAP_time = micros();

          currentStatus.mapADC = MAPrunningValue;
          currentStatus.MAP = fastMap10Bit(currentStatus.mapADC, configPage2.mapMin, configPage2.mapMax); //Get the current MAP value
          MAPcurRev = currentStatus.startRevolutions; //Reset the current rev count
          MAPrunningValue = 1023; //Reset the latest value so the next reading will always be lower

          validateMAP();
        }
      }
      else 
      {
        instanteneousMAPReading();
        MAPrunningValue = currentStatus.mapADC;  //Keep updating the MAPrunningValue to give it head start when switching to cycle minimum.
      }
      break;

    case 3:
      //Average of an ignition event
      if ( (currentStatus.RPMdiv100 > configPage2.mapSwitchPoint) && (currentStatus.hasSync == true) && (currentStatus.startRevolutions > 1) && (! currentStatus.engineProtectStatus) ) //If the engine isn't running, fall back to instantaneous reads
      {
        if( (MAPcurRev == ignitionCount) ) //Watch for a change in the ignition counter to determine whether we're still on the same event
        {
          #if defined(ANALOG_ISR_MAP)
            tempReading = AnChannel[pinMAP-A0];
          #else
            tempReading = analogRead(pinMAP);
            tempReading = analogRead(pinMAP);
          #endif

          //Error check
          if( (tempReading < VALID_MAP_MAX) && (tempReading > VALID_MAP_MIN) )
          {
            currentStatus.mapADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_MAP, currentStatus.mapADC);
            MAPrunningValue += currentStatus.mapADC; //Add the current reading onto the total
            MAPcount++;
          }
          else { mapErrorCount += 1; }
        }
        else
        {
          //Reaching here means that the  next ignition event has occured and the MAP value should be calculated
          //Sanity check
          if( (MAPrunningValue != 0) && (MAPcount != 0) && (MAPcurRev < ignitionCount) )
          {
            //Update the calculation times and last value. These are used by the MAP based Accel enrich
            MAPlast = currentStatus.MAP;
            MAPlast_time = MAP_time;
            MAP_time = micros();

            currentStatus.mapADC = ldiv(MAPrunningValue, MAPcount).quot;
            currentStatus.MAP = fastMap10Bit(currentStatus.mapADC, configPage2.mapMin, configPage2.mapMax); //Get the current MAP value
            validateMAP();
          }
          else { instanteneousMAPReading(); }

          MAPcurRev = ignitionCount; //Reset the current event count
          MAPrunningValue = 0;
          MAPcount = 0;
        }
      }
      else 
      {
        instanteneousMAPReading();
        MAPrunningValue = currentStatus.mapADC; //Keep updating the MAPrunningValue to give it head start when switching to ignition event average.
        MAPcount = 1;
      }
      break; 

    default:
    //Instantaneous MAP readings (Just in case)
    instanteneousMAPReading();
    break;
  }
}

void readTPS(bool useFilter)
{
  TPSlast = currentStatus.TPS;
  TPSlast_time = TPS_time;
  #if defined(ANALOG_ISR)
    byte tempTPS = fastMap1023toX(AnChannel[pinTPS-A0], 255); //Get the current raw TPS ADC value and map it into a byte
  #else
    analogRead(pinTPS);
    byte tempTPS = fastMap1023toX(analogRead(pinTPS), 255); //Get the current raw TPS ADC value and map it into a byte
  #endif
  //The use of the filter can be overridden if required. This is used on startup to disable priming pulse if flood clear is wanted
  if(useFilter == true) { currentStatus.tpsADC = ADC_FILTER(tempTPS, configPage4.ADCFILTER_TPS, currentStatus.tpsADC); }
  else { currentStatus.tpsADC = tempTPS; }
  byte tempADC = currentStatus.tpsADC; //The tempADC value is used in order to allow TunerStudio to recover and redo the TPS calibration if this somehow gets corrupted

  if(configPage2.tpsMax > configPage2.tpsMin)
  {
    //Check that the ADC values fall within the min and max ranges (Should always be the case, but noise can cause these to fluctuate outside the defined range).
    if (currentStatus.tpsADC < configPage2.tpsMin) { tempADC = configPage2.tpsMin; }
    else if(currentStatus.tpsADC > configPage2.tpsMax) { tempADC = configPage2.tpsMax; }
    currentStatus.TPS = map(tempADC, configPage2.tpsMin, configPage2.tpsMax, 0, 200); //Take the raw TPS ADC value and convert it into a TPS% based on the calibrated values
  }
  else
  {
    //This case occurs when the TPS +5v and gnd are wired backwards, but the user wishes to retain this configuration.
    //In such a case, tpsMin will be greater then tpsMax and hence checks and mapping needs to be reversed

    tempADC = 255 - currentStatus.tpsADC; //Reverse the ADC values
    uint16_t tempTPSMax = 255 - configPage2.tpsMax;
    uint16_t tempTPSMin = 255 - configPage2.tpsMin;

    //All checks below are reversed from the standard case above
    if (tempADC > tempTPSMax) { tempADC = tempTPSMax; }
    else if(tempADC < tempTPSMin) { tempADC = tempTPSMin; }
    currentStatus.TPS = map(tempADC, tempTPSMin, tempTPSMax, 0, 200);
  }

  //Check whether the closed throttle position sensor is active
  if(configPage2.CTPSEnabled == true)
  {
    if(configPage2.CTPSPolarity == 0) { currentStatus.CTPSActive = !digitalRead(pinCTPS); } //Normal mode (ground switched)
    else { currentStatus.CTPSActive = digitalRead(pinCTPS); } //Inverted mode (5v activates closed throttle position sensor)
  }
  else { currentStatus.CTPSActive = 0; }
  TPS_time = micros();
}

void readCLT(bool useFilter)
{
  unsigned int tempReading;
  #if defined(ANALOG_ISR)
    tempReading = fastMap1023toX(AnChannel[pinCLT-A0], 511); //Get the current raw CLT value
  #else
    tempReading = analogRead(pinCLT);
    tempReading = analogRead(pinCLT);
    //tempReading = fastMap1023toX(analogRead(pinCLT), 511); //Get the current raw CLT value
  #endif
  //The use of the filter can be overridden if required. This is used on startup so there can be an immediately accurate coolant value for priming
  if(useFilter == true) { currentStatus.cltADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_CLT, currentStatus.cltADC); }
  else { currentStatus.cltADC = tempReading; }
  
  currentStatus.coolant = table2D_getValue(&cltCalibrationTable, currentStatus.cltADC) - CALIBRATION_TEMPERATURE_OFFSET; //Temperature calibration values are stored as positive bytes. We subtract 40 from them to allow for negative temperatures
}

void readIAT()
{
  unsigned int tempReading;
  #if defined(ANALOG_ISR)
    tempReading = fastMap1023toX(AnChannel[pinIAT-A0], 511); //Get the current raw IAT value
  #else
    tempReading = analogRead(pinIAT);
    tempReading = analogRead(pinIAT);
    //tempReading = fastMap1023toX(analogRead(pinIAT), 511); //Get the current raw IAT value
  #endif
  currentStatus.iatADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_IAT, currentStatus.iatADC);
  currentStatus.IAT = table2D_getValue(&iatCalibrationTable, currentStatus.iatADC) - CALIBRATION_TEMPERATURE_OFFSET;
}

void readBaro()
{
  if ( configPage6.useExtBaro != 0 )
  {
    int tempReading;
    // readings
    #if defined(ANALOG_ISR_MAP)
      tempReading = AnChannel[pinBaro-A0];
    #else
      tempReading = analogRead(pinBaro);
      tempReading = analogRead(pinBaro);
    #endif

    if(initialisationComplete == true) { currentStatus.baroADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_BARO, currentStatus.baroADC); }//Very weak filter
    else { currentStatus.baroADC = tempReading; } //Baro reading (No filter)

    currentStatus.baro = fastMap10Bit(currentStatus.baroADC, configPage2.baroMin, configPage2.baroMax); //Get the current MAP value
  }
  else
  {
    /*
    * If no dedicated baro sensor is available, attempt to get a reading from the MAP sensor. This can only be done if the engine is not running. 
    * 1. Verify that the engine is not running
    * 2. Verify that the reading from the MAP sensor is within the possible physical limits
    */

    //Attempt to use the last known good baro reading from EEPROM as a starting point
    byte lastBaro = readLastBaro();
    if ((lastBaro >= BARO_MIN) && (lastBaro <= BARO_MAX)) //Make sure it's not invalid (Possible on first run etc)
    { currentStatus.baro = lastBaro; } //last baro correction
    else { currentStatus.baro = 100; } //Fall back position.

    //Verify the engine isn't running by confirming RPM is 0 and it has been at least 1 second since the last tooth was detected
    unsigned long timeToLastTooth = (micros() - toothLastToothTime);
    if((currentStatus.RPM == 0) && (timeToLastTooth > 1000000UL))
    {
      instanteneousMAPReading(); //Get the current MAP value
      /* 
      * The highest sea-level pressure on Earth occurs in Siberia, where the Siberian High often attains a sea-level pressure above 105 kPa;
      * with record highs close to 108.5 kPa.
      * The lowest possible baro reading is based on an altitude of 3500m above sea level.
      */
      if ((currentStatus.MAP >= BARO_MIN) && (currentStatus.MAP <= BARO_MAX)) //Safety check to ensure the baro reading is within the physical limits
      {
        currentStatus.baro = currentStatus.MAP;
        storeLastBaro(currentStatus.baro);
      }
    }
  }
}

void readO2()
{
  //An O2 read is only performed if an O2 sensor type is selected. This is to prevent potentially dangerous use of the O2 readings prior to proper setup/calibration
  if(configPage6.egoType > 0)
  {
    unsigned int tempReading;
    #if defined(ANALOG_ISR)
      tempReading = fastMap1023toX(AnChannel[pinO2-A0], 511); //Get the current O2 value.
    #else
      tempReading = analogRead(pinO2);
      tempReading = analogRead(pinO2);
      //tempReading = fastMap1023toX(analogRead(pinO2), 511); //Get the current O2 value.
    #endif
    currentStatus.O2ADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_O2, currentStatus.O2ADC);
    //currentStatus.O2 = o2CalibrationTable[currentStatus.O2ADC];
    currentStatus.O2 = table2D_getValue(&o2CalibrationTable, currentStatus.O2ADC);
  }
  else
  {
    currentStatus.O2ADC = 0;
    currentStatus.O2 = 0;
  }
  
}

void readO2_2()
{
  //Second O2 currently disabled as its not being used
  //Get the current O2 value.
  unsigned int tempReading;
  #if defined(ANALOG_ISR)
    tempReading = fastMap1023toX(AnChannel[pinO2_2-A0], 511); //Get the current O2 value.
  #else
    tempReading = analogRead(pinO2_2);
    tempReading = analogRead(pinO2_2);
    //tempReading = fastMap1023toX(analogRead(pinO2_2), 511); //Get the current O2 value.
  #endif
  currentStatus.O2_2ADC = ADC_FILTER(tempReading, configPage4.ADCFILTER_O2, currentStatus.O2_2ADC);
  currentStatus.O2_2 = table2D_getValue(&o2CalibrationTable, currentStatus.O2_2ADC);
}

void readBat()
{
  int tempReading;
  #if defined(ANALOG_ISR)
    tempReading = fastMap1023toX(AnChannel[pinBat-A0], 245); //Get the current raw Battery value. Permissible values are from 0v to 24.5v (245)
  #else
    tempReading = analogRead(pinBat);
    tempReading = fastMap1023toX(analogRead(pinBat), 245); //Get the current raw Battery value. Permissible values are from 0v to 24.5v (245)
  #endif

  //Apply the offset calibration value to the reading
  tempReading += configPage4.batVoltCorrect;
  if(tempReading < 0){
    tempReading=0;
  }  //with negative overflow prevention


  //The following is a check for if the voltage has jumped up from under 5.5v to over 7v.
  //If this occurs, it's very likely that the system has gone from being powered by USB to being powered from the 12v power source.
  //Should that happen, we retrigger the fuel pump priming and idle homing (If using a stepper)
  if( (currentStatus.battery10 < 55) && (tempReading > 70) && (currentStatus.RPM == 0) )
  {
    //Reprime the fuel pump
    fpPrimeTime = currentStatus.secl;
    fpPrimed = false;
    FUEL_PUMP_ON();

    //Redo the stepper homing
    if( (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_CL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_STEP_OL) )
    {
      initialiseIdle();
    }
  }

  currentStatus.battery10 = ADC_FILTER(tempReading, configPage4.ADCFILTER_BAT, currentStatus.battery10);
}

/**
 * @brief Returns the VSS pulse gap for a given history point
 * 
 * @param historyIndex The gap number that is wanted. EG:
 * historyIndex = 0 = Latest entry
 * historyIndex = 1 = 2nd entry entry
 */
uint32_t vssGetPulseGap(byte historyIndex)
{
  uint32_t tempGap = 0;
  
  noInterrupts();
  int8_t tempIndex = vssIndex - historyIndex;
  if(tempIndex < 0) { tempIndex += VSS_SAMPLES; }

  if(tempIndex > 0) { tempGap = vssTimes[tempIndex] - vssTimes[tempIndex - 1]; }
  else { tempGap = vssTimes[0] - vssTimes[(VSS_SAMPLES-1)]; }
  interrupts();

  return tempGap;
}

uint16_t getSpeed()
{
  uint16_t tempSpeed = 0;

  if(configPage2.vssMode == 1)
  {
    //VSS mode 1 is (Will be) CAN
  }
  // Interrupt driven mode
  else if(configPage2.vssMode > 1)
  {
    uint32_t pulseTime = 0;
    uint32_t vssTotalTime = 0;

    //Add up the time between the teeth. Note that the total number of gaps is equal to the number of samples minus 1
    for(byte x = 0; x<(VSS_SAMPLES-1); x++)
    {
      vssTotalTime += vssGetPulseGap(x);
    }

    pulseTime = vssTotalTime / (VSS_SAMPLES - 1);
    if ( (micros() - vssTimes[0]) > 1000000UL ) { tempSpeed = 0; } // Check that the car hasn't come to a stop
    else 
      {
        tempSpeed = 3600000000UL / (pulseTime * configPage2.vssPulsesPerKm); //Convert the pulse gap into km/h
        tempSpeed = ADC_FILTER(tempSpeed, configPage2.vssSmoothing, currentStatus.vss); //Apply speed smoothing factor
      }
    if(tempSpeed > 1000) { tempSpeed = currentStatus.vss; } //Safety check. This usually occurs when there is a hardware issue

  }
  return tempSpeed;
}

byte getGear()
{
  byte tempGear = 0; //Unknown gear
  if(currentStatus.vss > 0)
  {
    //If the speed is non-zero, default to the last calculated gear
    tempGear = currentStatus.gear;

    uint16_t pulsesPer1000rpm = (currentStatus.vss * 10000UL) / currentStatus.RPM; //Gives the current pulses per 1000RPM, multipled by 10 (10x is the multiplication factor for the ratios in TS)
    //Begin gear detection
    if( (pulsesPer1000rpm > (configPage2.vssRatio1 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio1 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 1; }
    else if( (pulsesPer1000rpm > (configPage2.vssRatio2 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio2 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 2; }
    else if( (pulsesPer1000rpm > (configPage2.vssRatio3 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio3 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 3; }
    else if( (pulsesPer1000rpm > (configPage2.vssRatio4 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio4 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 4; }
    else if( (pulsesPer1000rpm > (configPage2.vssRatio5 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio5 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 5; }
    else if( (pulsesPer1000rpm > (configPage2.vssRatio6 - VSS_GEAR_HYSTERESIS)) && (pulsesPer1000rpm < (configPage2.vssRatio6 + VSS_GEAR_HYSTERESIS)) ) { tempGear = 6; }
  }
  
  return tempGear;
}

byte getFuelPressure()
{
  int16_t tempFuelPressure = 0;
  uint16_t tempReading;

  if(configPage10.fuelPressureEnable > 0)
  {
    //Perform ADC read
    tempReading = analogRead(pinFuelPressure);
    tempReading = analogRead(pinFuelPressure);

    tempFuelPressure = fastMap10Bit(tempReading, configPage10.fuelPressureMin, configPage10.fuelPressureMax);
    tempFuelPressure = ADC_FILTER(tempFuelPressure, ADCFILTER_PSI_DEFAULT, currentStatus.fuelPressure); //Apply smoothing factor
    //Sanity checks
    if(tempFuelPressure > configPage10.fuelPressureMax) { tempFuelPressure = configPage10.fuelPressureMax; }
    if(tempFuelPressure < 0 ) { tempFuelPressure = 0; } //prevent negative values, which will cause problems later when the values aren't signed.
  }

  return (byte)tempFuelPressure;
}

byte getOilPressure()
{
  int16_t tempOilPressure = 0;
  uint16_t tempReading;

  if(configPage10.oilPressureEnable > 0)
  {
    //Perform ADC read
    tempReading = analogRead(pinOilPressure);
    tempReading = analogRead(pinOilPressure);


    tempOilPressure = fastMap10Bit(tempReading, configPage10.oilPressureMin, configPage10.oilPressureMax);
    tempOilPressure = ADC_FILTER(tempOilPressure, ADCFILTER_PSI_DEFAULT, currentStatus.oilPressure); //Apply smoothing factor
    //Sanity check
    if(tempOilPressure > configPage10.oilPressureMax) { tempOilPressure = configPage10.oilPressureMax; }
    if(tempOilPressure < 0 ) { tempOilPressure = 0; } //prevent negative values, which will cause problems later when the values aren't signed.
  }


  return (byte)tempOilPressure;
}

/*
 * The interrupt function for reading the flex sensor frequency and pulse width
 * flexCounter value is incremented with every pulse and reset back to 0 once per second
 */
void flexPulse()
{
  if(READ_FLEX() == true)
  {
    unsigned long tempPW = (micros() - flexStartTime); //Calculate the pulse width
    flexPulseWidth = ADC_FILTER(tempPW, configPage4.FILTER_FLEX, flexPulseWidth);
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
void knockPulse()
{
  //Check if this the start of a knock. 
  if(knockCounter == 0)
  {
    //knockAngle = crankAngle + fastTimeToAngle( (micros() - lastCrankAngleCalc) ); 
    knockStartTime = micros();
    knockCounter = 1;
  }
  else { ++knockCounter; } //Knock has already started, so just increment the counter for this

}

/**
 * @brief The ISR function for VSS pulses
 * 
 */
void vssPulse()
{
  //TODO: Add basic filtering here
  vssIndex++;
  if(vssIndex == VSS_SAMPLES) { vssIndex = 0; }

  vssTimes[vssIndex] = micros();
}

uint16_t readAuxanalog(uint8_t analogPin)
{
  //read the Aux analog value for pin set by analogPin 
  unsigned int tempReading;
  #if defined(ANALOG_ISR)
    tempReading = fastMap1023toX(AnChannel[analogPin-A0], 1023); //Get the current raw Auxanalog value
  #else
    tempReading = analogRead(analogPin);
    tempReading = analogRead(analogPin);
    //tempReading = fastMap1023toX(analogRead(analogPin), 511); Get the current raw Auxanalog value
  #endif
  return tempReading;
} 

uint16_t readAuxdigital(uint8_t digitalPin)
{
  //read the Aux digital value for pin set by digitalPin 
  unsigned int tempReading;
  tempReading = digitalRead(digitalPin); 
  return tempReading;
} 
