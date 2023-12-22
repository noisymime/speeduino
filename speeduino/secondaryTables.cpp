#include "globals.h"
#include "secondaryTables.h"
#include "corrections.h"
#include "load_source.h"

/**
 * @brief Looks up and returns the VE value from the secondary fuel table
 * 
 * This performs largely the same operations as getVE() however the lookup is of the secondary fuel table and uses the secondary load source
 * @return byte 
 */
static inline uint8_t getVE2(void)
{
  currentStatus.fuelLoad2 = getLoad(configPage10.fuel2Algorithm, currentStatus);
  return get3DTableValue(&fuelTable2, currentStatus.fuelLoad2, currentStatus.RPM); //Perform lookup into fuel map for RPM vs MAP value
}

static inline bool fuelModeCondSwitchRpmActive(void) {
  return (configPage10.fuel2SwitchVariable == FUEL2_CONDITION_RPM)
      && (currentStatus.RPM > configPage10.fuel2SwitchValue);
}

static inline bool fuelModeCondSwitchMapActive(void) {
  return (configPage10.fuel2SwitchVariable == FUEL2_CONDITION_MAP)
      && ((uint16_t)(int16_t)currentStatus.MAP > configPage10.fuel2SwitchValue);
}

static inline bool fuelModeCondSwitchTpsActive(void) {
  return (configPage10.fuel2SwitchVariable == FUEL2_CONDITION_TPS)
      && (currentStatus.TPS > configPage10.fuel2SwitchValue);
}

static inline bool fuelModeCondSwitchEthanolActive(void) {
  return (configPage10.fuel2SwitchVariable == FUEL2_CONDITION_ETH)
      && (currentStatus.ethanolPct > configPage10.fuel2SwitchValue);
}

static inline bool fuelModeCondSwitchActive(void) {
  return (configPage10.fuel2Mode == FUEL2_MODE_CONDITIONAL_SWITCH)
      && ( fuelModeCondSwitchRpmActive()
        || fuelModeCondSwitchMapActive() 
        || fuelModeCondSwitchTpsActive()
        || fuelModeCondSwitchEthanolActive());
}

static inline bool fuelModeInputSwitchActive(void) {
  return (configPage10.fuel2Mode == FUEL2_MODE_INPUT_SWITCH)
      && (digitalRead(pinFuel2Input) == configPage10.fuel2InputPolarity);
}

void calculateSecondaryFuel(void)
{
  //If the secondary fuel table is in use, also get the VE value from there
  if(configPage10.fuel2Mode == FUEL2_MODE_MULTIPLY)
  {
    currentStatus.VE2 = getVE2();
    //Fuel 2 table is treated as a % value. Table 1 and 2 are multiplied together and divided by 100
    uint16_t combinedVE = ((uint16_t)currentStatus.VE1 * (uint16_t)currentStatus.VE2) / 100U;
    currentStatus.VE = min(UINT8_MAX, combinedVE);
  }
  else if(configPage10.fuel2Mode == FUEL2_MODE_ADD)
  {
    currentStatus.VE2 = getVE2();
    //Fuel tables are added together, but a check is made to make sure this won't overflow the 8-bit VE value
    uint16_t combinedVE = (uint16_t)currentStatus.VE1 + (uint16_t)currentStatus.VE2;
    currentStatus.VE = min(UINT8_MAX, combinedVE);
  }
  else if(fuelModeCondSwitchActive() || fuelModeInputSwitchActive())
  {
    currentStatus.VE2 = getVE2();
    BIT_SET(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Set the bit indicating that the 2nd fuel table is in use. 
    currentStatus.VE = currentStatus.VE2;
  }
  else
  {
    // Unknown mode or mode not activated
    BIT_CLEAR(currentStatus.status3, BIT_STATUS3_FUEL2_ACTIVE); //Clear the bit indicating that the 2nd fuel table is in use. 
  }
}

/**
 * @brief Performs a lookup of the second ignition advance table. The values used to look this up will be RPM and whatever load source the user has configured
 * 
 * @return byte The current target advance value in degrees
 */
static inline int8_t getAdvance2(void)
{
  currentStatus.ignLoad2 = getLoad(configPage10.spark2Algorithm, currentStatus);
  int8_t tempAdvance = (int16_t)get3DTableValue(&ignitionTable2, currentStatus.ignLoad2, currentStatus.RPM) - INT16_C(OFFSET_IGNITION); //As above, but for ignition advance

  //Perform the corrections calculation on the secondary advance value, only if it uses a switched mode
  if( (configPage10.spark2SwitchVariable == SPARK2_MODE_CONDITIONAL_SWITCH) || (configPage10.spark2SwitchVariable == SPARK2_MODE_INPUT_SWITCH) ) { 
    tempAdvance = correctionsIgn(tempAdvance);
  } 

  return tempAdvance;
}

static inline bool sparkModeCondSwitchRpmActive(void) {
  return (configPage10.spark2SwitchVariable == SPARK2_CONDITION_RPM)
      && (currentStatus.RPM > configPage10.spark2SwitchValue);
}

static inline bool sparkModeCondSwitchMapActive(void) {
  return (configPage10.spark2SwitchVariable == SPARK2_CONDITION_MAP)
      && ((uint16_t)(int16_t)currentStatus.MAP > configPage10.spark2SwitchValue);
}

static inline bool sparkModeCondSwitchTpsActive(void) {
  return (configPage10.spark2SwitchVariable == SPARK2_CONDITION_TPS)
      && (currentStatus.TPS > configPage10.spark2SwitchValue);
}

static inline bool sparkModeCondSwitchEthanolActive(void) {
return (configPage10.spark2SwitchVariable == SPARK2_CONDITION_ETH)
    && (currentStatus.ethanolPct > configPage10.spark2SwitchValue);
}

static inline bool sparkModeCondSwitchActive(void) {
  return (configPage10.spark2Mode == SPARK2_MODE_CONDITIONAL_SWITCH)
      && ( sparkModeCondSwitchRpmActive()
        || sparkModeCondSwitchMapActive() 
        || sparkModeCondSwitchTpsActive()
        || sparkModeCondSwitchEthanolActive());
}

static inline bool sparkModeInputSwitchActive(void) {
  return (configPage10.spark2Mode == SPARK2_MODE_INPUT_SWITCH)
      && (digitalRead(pinSpark2Input) == configPage10.spark2InputPolarity);
}

void calculateSecondarySpark(void)
{
  //Same as above but for the secondary ignition table
  if(configPage10.spark2Mode == SPARK2_MODE_MULTIPLY)
  {
    BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE);
    //make sure we don't have a negative value in the multiplier table (sharing a signed 8 bit table)
    currentStatus.advance2 = max(0, getAdvance2());
    //Spark 2 table is treated as a % value. Table 1 and 2 are multiplied together and divided by 100
    int16_t combinedAdvance = ((int16_t)currentStatus.advance1 * (int16_t)currentStatus.advance2) / 100;
    //make sure we don't overflow and accidentally set negative timing, currentStatus.advance can only hold a signed 8 bit value
    currentStatus.advance = min(INT8_MAX, combinedAdvance);
  }
  else if(configPage10.spark2Mode == SPARK2_MODE_ADD)
  {
    BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); //Set the bit indicating that the 2nd spark table is in use. 
    currentStatus.advance2 = getAdvance2();
    //Spark tables are added together, but a check is made to make sure this won't overflow the 8-bit VE value
    int16_t combinedAdvance = (int16_t)currentStatus.advance1 + (int16_t)currentStatus.advance2;
    currentStatus.advance = min(INT8_MAX, combinedAdvance);
  }
  else if(sparkModeCondSwitchActive() || sparkModeInputSwitchActive())
  {
    BIT_SET(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); //Set the bit indicating that the 2nd spark table is in use. 
    currentStatus.advance2 = getAdvance2();
    currentStatus.advance = currentStatus.advance2;
  }
  else
  {
    // Unknown mode or mode not activated
    BIT_CLEAR(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE); //Clear the bit indicating that the 2nd spark table is in use. 
  }

  //Apply the fixed timing correction manually. This has to be done again here if any of the above conditions are met to prevent any of the seconadary calculations applying instead of fixec timing
  if (BIT_CHECK(currentStatus.status5, BIT_STATUS5_SPARK2_ACTIVE)) {
    currentStatus.advance = correctionFixedTiming(currentStatus.advance);
    currentStatus.advance = correctionCrankingFixedTiming(currentStatus.advance); //This overrides the regular fixed timing, must come last
  }
}
