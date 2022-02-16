/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
/** @file
 * Lower level ConfigPage*, Table2D, Table3D and EEPROM storage operations.
 */

#include "globals.h"
#include EEPROM_LIB_H //This is defined in the board .h files
#include "storage.h"
#include "pages.h"

//The maximum number of write operations that will be performed in one go. If we try to write to the EEPROM too fast (Each write takes ~3ms) then the rest of the system can hang)
#if defined(CORE_STM32) || defined(CORE_TEENSY) & !defined(USE_SPI_EEPROM)
#define EEPROM_MAX_WRITE_BLOCK 64
#else
#define EEPROM_MAX_WRITE_BLOCK 12
//#define EEPROM_MAX_WRITE_BLOCK 8
#endif

#define EEPROM_DATA_VERSION   0

// Calibration data is stored at the end of the EEPROM (This is in case any further calibration tables are needed as they are large blocks)
#define STORAGE_END 0xFFF       // Should be E2END?
#define EEPROM_CALIBRATION_CLT_VALUES (STORAGE_END-sizeof(cltCalibration_values))
#define EEPROM_CALIBRATION_CLT_BINS   (EEPROM_CALIBRATION_CLT_VALUES-sizeof(cltCalibration_bins))
#define EEPROM_CALIBRATION_IAT_VALUES (EEPROM_CALIBRATION_CLT_BINS-sizeof(iatCalibration_values))
#define EEPROM_CALIBRATION_IAT_BINS   (EEPROM_CALIBRATION_IAT_VALUES-sizeof(iatCalibration_bins))
#define EEPROM_CALIBRATION_O2_VALUES  (EEPROM_CALIBRATION_IAT_BINS-sizeof(o2Calibration_values))
#define EEPROM_CALIBRATION_O2_BINS    (EEPROM_CALIBRATION_O2_VALUES-sizeof(o2Calibration_bins))
#define EEPROM_LAST_BARO              (EEPROM_CALIBRATION_O2_BINS-1)

static bool eepromWritesPending = false;
static bool forceBurn = false;
bool deferEEPROMWrites = false;

bool isEepromWritePending()
{
  return eepromWritesPending;
}

/** Write all config pages to EEPROM.
 */
void writeAllConfig()
{
  uint8_t pageCount = getPageCount();
  uint8_t page = 1U;
  writeConfig(page++);
  while (page<pageCount && ( !eepromWritesPending || forceBurn ) )
  {
    writeConfig(page++);
  }
}

void enableForceBurn() { forceBurn = true; }
void disableForceBurn() { forceBurn = false; }


//  ================================= Internal write support ===============================
struct write_location {
  eeprom_address_t address;
  uint16_t counter;

  /** Update byte to EEPROM by first comparing content and the need to write it.
  We only ever write to the EEPROM where the new value is different from the currently stored byte
  This is due to the limited write life of the EEPROM (Approximately 100,000 writes)
  */
  void update(uint8_t value)
  {
    if (EEPROM.read(address)!=value)
    {
      EEPROM.write(address, value);
      ++counter;
    }
  }

  write_location& operator++()
  {
    ++address;
    return *this;
  }

  bool can_write() const
  {
    return (counter<=EEPROM_MAX_WRITE_BLOCK);
  }
};

static inline write_location write_range(const byte *pStart, const byte *pEnd, write_location location)
{
  while ( (location.can_write() || forceBurn) && pStart!=pEnd)
  {
    location.update(*pStart);
    ++pStart; 
    ++location;
  }
  return location;
}

static inline write_location write(const table_row_iterator &row, write_location location)
{
  return write_range(&*row, row.end(), location);
}

static inline write_location write(table_value_iterator it, write_location location)
{
  while ((location.can_write() || forceBurn) && !it.at_end())
  {
    location = write(*it, location);
    ++it;
  }
  return location;
}

static inline write_location write(table_axis_iterator it, write_location location)
{
  while ((location.can_write() || forceBurn) && !it.at_end())
  {
    location.update((byte)*it);
    ++location;
    ++it;
  }
  return location;
}


static inline write_location writeTable(const void *pTable, table_type_t key, write_location location)
{
  return write(y_begin(pTable, key).reverse(), 
                write(x_begin(pTable, key), 
                  write(rows_begin(pTable, key), location)));
}

//Simply an alias for EEPROM.update()
void EEPROMWriteRaw(uint16_t address, uint8_t data) { EEPROM.update(address, data); }
uint8_t EEPROMReadRaw(uint16_t address) { return EEPROM.read(address); }

//  ================================= End write support ===============================

/** Write a table or map to EEPROM storage.
Takes the current configuration (config pages and maps)
and writes them to EEPROM as per the layout defined in storage.h.
*/
void writeConfig(uint8_t pageNum)
{
  write_location result = { 0, 0 };

  if(deferEEPROMWrites == true) { result.counter = (EEPROM_MAX_WRITE_BLOCK + 1); } //If we are deferring writes then we don't want to write anything. This will force can_write() to return false and the write will be skipped.

  switch(pageNum)
  {
    case veMapPage:
      /*---------------------------------------------------
      | Fuel table (See storage.h for data layout) - Page 1
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&fuelTable, fuelTable.type_key, { EEPROM_CONFIG1_MAP, 0 });
      break;

    case veSetPage:
      /*---------------------------------------------------
      | Config page 2 (See storage.h for data layout)
      | 64 byte long config table
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage2, (byte *)&configPage2+sizeof(configPage2), { EEPROM_CONFIG2_START, 0 });
      break;

    case ignMapPage:
      /*---------------------------------------------------
      | Ignition table (See storage.h for data layout) - Page 1
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&ignitionTable, ignitionTable.type_key, { EEPROM_CONFIG3_MAP, 0 });
      break;

    case ignSetPage:
      /*---------------------------------------------------
      | Config page 2 (See storage.h for data layout)
      | 64 byte long config table
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage4, (byte *)&configPage4+sizeof(configPage4), { EEPROM_CONFIG4_START, 0 });
      break;

    case afrMapPage:
      /*---------------------------------------------------
      | AFR table (See storage.h for data layout) - Page 5
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&afrTable, afrTable.type_key, {  EEPROM_CONFIG5_MAP, 0 });
      break;

    case afrSetPage:
      /*---------------------------------------------------
      | Config page 3 (See storage.h for data layout)
      | 64 byte long config table
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage6, (byte *)&configPage6+sizeof(configPage6), { EEPROM_CONFIG6_START, 0 });
      break;

    case boostvvtPage:
      /*---------------------------------------------------
      | Boost and vvt tables (See storage.h for data layout) - Page 8
      | 8x8 table itself + the 8 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&boostTable, boostTable.type_key, { EEPROM_CONFIG7_MAP1, 0 });
      result = writeTable(&vvtTable, vvtTable.type_key, { EEPROM_CONFIG7_MAP2, result.counter });
      result = writeTable(&stagingTable, stagingTable.type_key, { EEPROM_CONFIG7_MAP3, result.counter });
      break;

    case seqFuelPage:
      /*---------------------------------------------------
      | Fuel trim tables (See storage.h for data layout) - Page 9
      | 6x6 tables itself + the 6 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&trim1Table, trim1Table.type_key, { EEPROM_CONFIG8_MAP1, 0 });
      result = writeTable(&trim2Table, trim2Table.type_key, { EEPROM_CONFIG8_MAP2, result.counter });
      result = writeTable(&trim3Table, trim3Table.type_key, { EEPROM_CONFIG8_MAP3, result.counter });
      result = writeTable(&trim4Table, trim4Table.type_key, { EEPROM_CONFIG8_MAP4, result.counter });
      result = writeTable(&trim5Table, trim5Table.type_key, { EEPROM_CONFIG8_MAP5, result.counter });
      result = writeTable(&trim6Table, trim6Table.type_key, { EEPROM_CONFIG8_MAP6, result.counter });
      result = writeTable(&trim7Table, trim7Table.type_key, { EEPROM_CONFIG8_MAP7, result.counter });
      result = writeTable(&trim8Table, trim8Table.type_key, { EEPROM_CONFIG8_MAP8, result.counter });
      break;

    case canbusPage:
      /*---------------------------------------------------
      | Config page 10 (See storage.h for data layout)
      | 192 byte long config table
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage9, (byte *)&configPage9+sizeof(configPage9), { EEPROM_CONFIG9_START, 0 });
      break;

    case warmupPage:
      /*---------------------------------------------------
      | Config page 11 (See storage.h for data layout)
      | 192 byte long config table
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage10, (byte *)&configPage10+sizeof(configPage10), { EEPROM_CONFIG10_START, 0});
      break;

    case fuelMap2Page:
      /*---------------------------------------------------
      | Fuel table 2 (See storage.h for data layout)
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&fuelTable2, fuelTable2.type_key, { EEPROM_CONFIG11_MAP, 0 });
      break;

    case wmiMapPage:
      /*---------------------------------------------------
      | WMI and Dwell tables (See storage.h for data layout) - Page 12
      | 8x8 WMI table itself + the 8 values along each of the axis
      | 8x8 VVT2 table + the 8 values along each of the axis
      | 4x4 Dwell table itself + the 4 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&wmiTable, wmiTable.type_key, { EEPROM_CONFIG12_MAP, 0 });
      result = writeTable(&vvt2Table, vvt2Table.type_key, { EEPROM_CONFIG12_MAP2, result.counter });
      result = writeTable(&dwellTable, dwellTable.type_key, { EEPROM_CONFIG12_MAP3, result.counter });
      break;
      
  case progOutsPage:
      /*---------------------------------------------------
      | Config page 13 (See storage.h for data layout)
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage13, (byte *)&configPage13+sizeof(configPage13), { EEPROM_CONFIG13_START, 0});
      break;
    
    case ignMap2Page:
      /*---------------------------------------------------
      | Ignition table (See storage.h for data layout) - Page 1
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&ignitionTable2, ignitionTable2.type_key, { EEPROM_CONFIG14_MAP, 0 });
      break;

    default:
      break;
  }

  eepromWritesPending = !result.can_write();
}

/** Reset all configPage* structs (2,4,6,9,10,13) and write them full of null-bytes.
 */
void resetConfigPages()
{
  for (uint8_t page=1; page<getPageCount(); ++page)
  {
    page_iterator_t entity = page_begin(page);
    while (entity.type!=End)
    {
      if (entity.type==Raw)
      {
        memset(entity.pData, 0, entity.size);
      }
      entity = advance(entity);
    }
  }
}

//  ================================= Internal read support ===============================

/** Load range of bytes form EEPROM offset to memory.
 * @param address - start offset in EEPROM
 * @param pFirst - Start memory address
 * @param pLast - End memory address
 */
static inline eeprom_address_t load_range(eeprom_address_t address, byte *pFirst, const byte *pLast)
{
#if defined(CORE_AVR)
  // The generic code in the #else branch works but this provides a 45% speed up on AVR
  size_t size = pLast-pFirst;
  eeprom_read_block(pFirst, (void*)address, size);
  return address+size;
#else
  for (; pFirst != pLast; ++address, (void)++pFirst)
  {
    *pFirst = EEPROM.read(address);
  }
  return address;
#endif
}

static inline eeprom_address_t load(table_row_iterator row, eeprom_address_t address)
{
  return load_range(address, &*row, row.end());
}

static inline eeprom_address_t load(table_value_iterator it, eeprom_address_t address)
{
  while (!it.at_end())
  {
    address = load(*it, address);
    ++it;
  }
  return address; 
}

static inline eeprom_address_t load(table_axis_iterator it, eeprom_address_t address)
{
  while (!it.at_end())
  {
    *it = EEPROM.read(address);
    ++address;
    ++it;
  }
  return address;    
}


static inline eeprom_address_t loadTable(void *pTable, table_type_t key, eeprom_address_t address)
{
  return load(y_begin(pTable, key).reverse(),
                load(x_begin(pTable, key), 
                  load(rows_begin(pTable, key), address)));
}

//  ================================= End internal read support ===============================


/** Load all config tables from storage.
 */
void loadConfig()
{
  loadTable(&fuelTable, fuelTable.type_key, EEPROM_CONFIG1_MAP);
  load_range(EEPROM_CONFIG2_START, (byte *)&configPage2, (byte *)&configPage2+sizeof(configPage2));
  
  //*********************************************************************************************************************************************************************************
  //IGNITION CONFIG PAGE (2)

  loadTable(&ignitionTable, ignitionTable.type_key, EEPROM_CONFIG3_MAP);
  load_range(EEPROM_CONFIG4_START, (byte *)&configPage4, (byte *)&configPage4+sizeof(configPage4));

  //*********************************************************************************************************************************************************************************
  //AFR TARGET CONFIG PAGE (3)

  loadTable(&afrTable, afrTable.type_key, EEPROM_CONFIG5_MAP);
  load_range(EEPROM_CONFIG6_START, (byte *)&configPage6, (byte *)&configPage6+sizeof(configPage6));

  //*********************************************************************************************************************************************************************************
  // Boost and vvt tables load
  loadTable(&boostTable, boostTable.type_key, EEPROM_CONFIG7_MAP1);
  loadTable(&vvtTable, vvtTable.type_key,  EEPROM_CONFIG7_MAP2);
  loadTable(&stagingTable, stagingTable.type_key, EEPROM_CONFIG7_MAP3);

  //*********************************************************************************************************************************************************************************
  // Fuel trim tables load
  loadTable(&trim1Table, trim1Table.type_key, EEPROM_CONFIG8_MAP1);
  loadTable(&trim2Table, trim2Table.type_key, EEPROM_CONFIG8_MAP2);
  loadTable(&trim3Table, trim3Table.type_key, EEPROM_CONFIG8_MAP3);
  loadTable(&trim4Table, trim4Table.type_key, EEPROM_CONFIG8_MAP4);
  loadTable(&trim5Table, trim5Table.type_key, EEPROM_CONFIG8_MAP5);
  loadTable(&trim6Table, trim6Table.type_key, EEPROM_CONFIG8_MAP6);
  loadTable(&trim7Table, trim7Table.type_key, EEPROM_CONFIG8_MAP7);
  loadTable(&trim8Table, trim8Table.type_key, EEPROM_CONFIG8_MAP8);

  //*********************************************************************************************************************************************************************************
  //canbus control page load
  load_range(EEPROM_CONFIG9_START, (byte *)&configPage9, (byte *)&configPage9+sizeof(configPage9));

  //*********************************************************************************************************************************************************************************

  //CONFIG PAGE (10)
  load_range(EEPROM_CONFIG10_START, (byte *)&configPage10, (byte *)&configPage10+sizeof(configPage10));

  //*********************************************************************************************************************************************************************************
  //Fuel table 2 (See storage.h for data layout)
  loadTable(&fuelTable2, fuelTable2.type_key, EEPROM_CONFIG11_MAP);

  //*********************************************************************************************************************************************************************************
  // WMI, VVT2 and Dwell table load
  loadTable(&wmiTable, wmiTable.type_key, EEPROM_CONFIG12_MAP);
  loadTable(&vvt2Table, vvt2Table.type_key, EEPROM_CONFIG12_MAP2);
  loadTable(&dwellTable, dwellTable.type_key, EEPROM_CONFIG12_MAP3);

  //*********************************************************************************************************************************************************************************
  //CONFIG PAGE (13)
  load_range(EEPROM_CONFIG13_START, (byte *)&configPage13, (byte *)&configPage13+sizeof(configPage13));

  //*********************************************************************************************************************************************************************************
  //SECOND IGNITION CONFIG PAGE (14)

  loadTable(&ignitionTable2, ignitionTable2.type_key, EEPROM_CONFIG14_MAP);

  //*********************************************************************************************************************************************************************************
}

/** Read the calibration information from EEPROM.
This is separate from the config load as the calibrations do not exist as pages within the ini file for Tuner Studio.
*/
void loadCalibration()
{
  // If you modify this function be sure to also modify writeCalibration();
  // it should be a mirror image of this function.

  EEPROM.get(EEPROM_CALIBRATION_O2_BINS, o2Calibration_bins);
  EEPROM.get(EEPROM_CALIBRATION_O2_VALUES, o2Calibration_values);
  
  EEPROM.get(EEPROM_CALIBRATION_IAT_BINS, iatCalibration_bins);
  EEPROM.get(EEPROM_CALIBRATION_IAT_VALUES, iatCalibration_values);

  EEPROM.get(EEPROM_CALIBRATION_CLT_BINS, cltCalibration_bins);
  EEPROM.get(EEPROM_CALIBRATION_CLT_VALUES, cltCalibration_values);
}

/** Write calibration tables to EEPROM.
This takes the values in the 3 calibration tables (Coolant, Inlet temp and O2)
and saves them to the EEPROM.
*/
void writeCalibration()
{
  // If you modify this function be sure to also modify loadCalibration();
  // it should be a mirror image of this function.

  EEPROM.put(EEPROM_CALIBRATION_O2_BINS, o2Calibration_bins);
  EEPROM.put(EEPROM_CALIBRATION_O2_VALUES, o2Calibration_values);
  
  EEPROM.put(EEPROM_CALIBRATION_IAT_BINS, iatCalibration_bins);
  EEPROM.put(EEPROM_CALIBRATION_IAT_VALUES, iatCalibration_values);

  EEPROM.put(EEPROM_CALIBRATION_CLT_BINS, cltCalibration_bins);
  EEPROM.put(EEPROM_CALIBRATION_CLT_VALUES, cltCalibration_values);
}

void writeCalibrationPage(uint8_t pageNum)
{
  if(pageNum == O2_CALIBRATION_PAGE)
  {
    EEPROM.put(EEPROM_CALIBRATION_O2_BINS, o2Calibration_bins);
    EEPROM.put(EEPROM_CALIBRATION_O2_VALUES, o2Calibration_values);
  }
  else if(pageNum == IAT_CALIBRATION_PAGE)
  {
    EEPROM.put(EEPROM_CALIBRATION_IAT_BINS, iatCalibration_bins);
    EEPROM.put(EEPROM_CALIBRATION_IAT_VALUES, iatCalibration_values);
  }
  else if(pageNum == CLT_CALIBRATION_PAGE)
  {
    EEPROM.put(EEPROM_CALIBRATION_CLT_BINS, cltCalibration_bins);
    EEPROM.put(EEPROM_CALIBRATION_CLT_VALUES, cltCalibration_values);
  }
}

static eeprom_address_t compute_crc_address(uint8_t pageNum)
{
  return EEPROM_LAST_BARO-((getPageCount() - pageNum)*sizeof(uint32_t));
}

/** Write CRC32 checksum to EEPROM.
Takes a page number and CRC32 value then stores it in the relevant place in EEPROM
@param pageNum - Config page number
@param crcValue - CRC32 checksum
*/
void storePageCRC32(uint8_t pageNum, uint32_t crcValue)
{
  EEPROM.put(compute_crc_address(pageNum), crcValue);
}

/** Retrieves and returns the 4 byte CRC32 checksum for a given page from EEPROM.
@param pageNum - Config page number
*/
uint32_t readPageCRC32(uint8_t pageNum)
{
  uint32_t crc32_val;
  return EEPROM.get(compute_crc_address(pageNum), crc32_val);
}

/** Same as above, but writes the CRC32 for the calibration page rather than tune data
@param pageNum - Calibration page number
@param crcValue - CRC32 checksum
*/
void storeCalibrationCRC32(uint8_t calibrationPageNum, uint32_t calibrationCRC)
{
  uint16_t targetAddress;
  switch(calibrationPageNum)
  {
    case O2_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_O2_CRC;
      break;
    case IAT_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_IAT_CRC;
      break;
    case CLT_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_CLT_CRC;
      break;
    default:
      targetAddress = EEPROM_CALIBRATION_CLT_CRC; //Obviously should never happen
      break;
  }

  EEPROM.put(targetAddress, calibrationCRC);
}

/** Retrieves and returns the 4 byte CRC32 checksum for a given calibration page from EEPROM.
@param pageNum - Config page number
*/
uint32_t readCalibrationCRC32(uint8_t calibrationPageNum)
{
  uint32_t crc32_val;
  uint16_t targetAddress;
  switch(calibrationPageNum)
  {
    case O2_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_O2_CRC;
      break;
    case IAT_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_IAT_CRC;
      break;
    case CLT_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_CLT_CRC;
      break;
    default:
      targetAddress = EEPROM_CALIBRATION_CLT_CRC; //Obviously should never happen
      break;
  }

  EEPROM.get(targetAddress, crc32_val);
  return crc32_val;
}

uint16_t getEEPROMSize()
{
  return EEPROM.length();
}

// Utility functions.
// By having these in this file, it prevents other files from calling EEPROM functions directly. This is useful due to differences in the EEPROM libraries on different devces
/// Read last stored barometer reading from EEPROM.
byte readLastBaro() { return EEPROM.read(EEPROM_LAST_BARO); }
/// Write last acquired arometer reading to EEPROM.
void storeLastBaro(byte newValue) { EEPROM.update(EEPROM_LAST_BARO, newValue); }
/// Read EEPROM current data format version (from offset EEPROM_DATA_VERSION).
byte readEEPROMVersion() { return EEPROM.read(EEPROM_DATA_VERSION); }
/// Store EEPROM current data format version (to offset EEPROM_DATA_VERSION).
void storeEEPROMVersion(byte newVersion) { EEPROM.update(EEPROM_DATA_VERSION, newVersion); }
