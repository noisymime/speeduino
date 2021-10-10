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
#include "table3d_iterator.h"
#include "pages.h"

//The maximum number of write operations that will be performed in one go. If we try to write to the EEPROM too fast (Each write takes ~3ms) then the rest of the system can hang)
#if defined(CORE_STM32) || defined(CORE_TEENSY) & !defined(USE_SPI_EEPROM)
#define EEPROM_MAX_WRITE_BLOCK 64
#else
#define EEPROM_MAX_WRITE_BLOCK 30
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
  while (page<pageCount && !eepromWritesPending)
  {
    writeConfig(page++);
  }
}


//  ================================= Internal write support ===============================

typedef struct write_location {
  eeprom_address_t address;
  uint8_t counter;
} write_location;


inline bool can_write(const write_location &location)
{
  return location.counter<=EEPROM_MAX_WRITE_BLOCK;
}

/** Update byte to EEPROM by first comparing content and the need to write it.
We only ever write to the EEPROM where the new value is different from the currently stored byte
This is due to the limited write life of the EEPROM (Approximately 100,000 writes)
*/
static inline write_location update(uint8_t value, const write_location &location)
{
  if (EEPROM.read(location.address)!=value)
  {
    EEPROM.write(location.address, value);
    return { location.address+1, (uint8_t)(location.counter+1U) };
  }
  return { location.address+1, location.counter };
}

static inline write_location write_range(const byte *pStart, const byte *pEnd, write_location location)
{
  while (can_write(location) && pStart!=pEnd)
  {
    location = update(*pStart, location);
    ++pStart; 
  }
  return location;
}

static inline write_location write(const table_row_t &row, write_location location)
{
  return write_range(row.pValue, row.pEnd, location);
}

static inline write_location write(table_row_iterator_t it, write_location location)
{
  while (can_write(location) && !at_end(it))
  {
    location = write(get_row(it), location);
    it = advance_row(it);
  }
  return location;
}

static inline write_location write(table_axis_iterator_t it, write_location location)
{
  while (can_write(location) && !at_end(it))
  {
    location = update(get_value(it), location);
    it = advance_axis(it);
  }
  return location;
}


static inline write_location writeTable(const void *pTable, table_type_t key, write_location location)
{
  return write(y_rbegin(pTable, key), 
                write(x_begin(pTable, key), 
                  write(rows_begin(pTable, key), location)));
}

//  ================================= End write support ===============================

/** Write a table or map to EEPROM storage.
Takes the current configuration (config pages and maps)
and writes them to EEPROM as per the layout defined in storage.h.
*/
void writeConfig(uint8_t pageNum)
{
  write_location result = { 0, 0 };

  switch(pageNum)
  {
    case veMapPage:
      /*---------------------------------------------------
      | Fuel table (See storage.h for data layout) - Page 1
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&fuelTable, fuelTable._metadata.type_key, { EEPROM_CONFIG1_MAP, 0 });
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
      result = writeTable(&ignitionTable, ignitionTable._metadata.type_key, { EEPROM_CONFIG3_MAP, 0 });
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
      result = writeTable(&afrTable, afrTable._metadata.type_key, {  EEPROM_CONFIG5_MAP, 0 });
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
      result = writeTable(&boostTable, boostTable._metadata.type_key, { EEPROM_CONFIG7_MAP1, 0 });
      result = writeTable(&vvtTable, vvtTable._metadata.type_key, { EEPROM_CONFIG7_MAP2, result.counter });
      result = writeTable(&stagingTable, stagingTable._metadata.type_key, { EEPROM_CONFIG7_MAP3, result.counter });
      break;

    case seqFuelPage:
      /*---------------------------------------------------
      | Fuel trim tables (See storage.h for data layout) - Page 9
      | 6x6 tables itself + the 6 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&trim1Table, trim1Table._metadata.type_key, { EEPROM_CONFIG8_MAP1, 0 });
      result = writeTable(&trim2Table, trim2Table._metadata.type_key, { EEPROM_CONFIG8_MAP2, result.counter });
      result = writeTable(&trim3Table, trim3Table._metadata.type_key, { EEPROM_CONFIG8_MAP3, result.counter });
      result = writeTable(&trim4Table, trim4Table._metadata.type_key, { EEPROM_CONFIG8_MAP4, result.counter });
      result = writeTable(&trim5Table, trim5Table._metadata.type_key, { EEPROM_CONFIG8_MAP5, result.counter });
      result = writeTable(&trim6Table, trim6Table._metadata.type_key, { EEPROM_CONFIG8_MAP6, result.counter });
      result = writeTable(&trim7Table, trim7Table._metadata.type_key, { EEPROM_CONFIG8_MAP7, result.counter });
      result = writeTable(&trim8Table, trim8Table._metadata.type_key, { EEPROM_CONFIG8_MAP8, result.counter });
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
      result = writeTable(&fuelTable2, fuelTable2._metadata.type_key, { EEPROM_CONFIG11_MAP, 0 });
      break;

    case wmiMapPage:
      /*---------------------------------------------------
      | WMI and Dwell tables (See storage.h for data layout) - Page 12
      | 8x8 WMI table itself + the 8 values along each of the axis
      | 8x8 VVT2 table + the 8 values along each of the axis
      | 4x4 Dwell table itself + the 4 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&wmiTable, wmiTable._metadata.type_key, { EEPROM_CONFIG12_MAP, 0 });
      result = writeTable(&vvt2Table, vvt2Table._metadata.type_key, { EEPROM_CONFIG12_MAP2, result.counter });
      result = writeTable(&dwellTable, dwellTable._metadata.type_key, { EEPROM_CONFIG12_MAP3, result.counter });
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
      result = writeTable(&ignitionTable2, ignitionTable2._metadata.type_key, { EEPROM_CONFIG14_MAP, 0 });
      break;

    default:
      break;
  }

  eepromWritesPending = !can_write(result);
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
  for (; pFirst != pLast; ++address, (void)++pFirst)
  {
    *pFirst = EEPROM.read(address);
  }
  return address;
}

static inline eeprom_address_t load(table_row_t row, eeprom_address_t address)
{
  return load_range(address, row.pValue, row.pEnd);
}

static inline eeprom_address_t load(table_row_iterator_t it, eeprom_address_t address)
{
  while (!at_end(it))
  {
    address = load(get_row(it), address);
    it = advance_row(it);
  }
  return address; 
}

static inline eeprom_address_t load(table_axis_iterator_t it, eeprom_address_t address)
{
  while (!at_end(it))
  {
    set_value(it, EEPROM.read(address));
    ++address;
    it = advance_axis(it);
  }
  return address;    
}


static inline eeprom_address_t loadTable(void *pTable, table_type_t key, eeprom_address_t address)
{
  return load(y_rbegin(pTable, key),
                load(x_begin(pTable, key), 
                  load(rows_begin(pTable, key), address)));
}

//  ================================= End internal read support ===============================


/** Load all config tables from storage.
 */
void loadConfig()
{
  loadTable(&fuelTable, fuelTable._metadata.type_key, EEPROM_CONFIG1_MAP);
  load_range(EEPROM_CONFIG2_START, (byte *)&configPage2, (byte *)&configPage2+sizeof(configPage2));
  
  //*********************************************************************************************************************************************************************************
  //IGNITION CONFIG PAGE (2)

  loadTable(&ignitionTable, ignitionTable._metadata.type_key, EEPROM_CONFIG3_MAP);
  load_range(EEPROM_CONFIG4_START, (byte *)&configPage4, (byte *)&configPage4+sizeof(configPage4));

  //*********************************************************************************************************************************************************************************
  //AFR TARGET CONFIG PAGE (3)

  loadTable(&afrTable, afrTable._metadata.type_key, EEPROM_CONFIG5_MAP);
  load_range(EEPROM_CONFIG6_START, (byte *)&configPage6, (byte *)&configPage6+sizeof(configPage6));

  //*********************************************************************************************************************************************************************************
  // Boost and vvt tables load
  loadTable(&boostTable, boostTable._metadata.type_key, EEPROM_CONFIG7_MAP1);
  loadTable(&vvtTable, vvtTable._metadata.type_key,  EEPROM_CONFIG7_MAP2);
  loadTable(&stagingTable, stagingTable._metadata.type_key, EEPROM_CONFIG7_MAP3);

  //*********************************************************************************************************************************************************************************
  // Fuel trim tables load
  loadTable(&trim1Table, trim1Table._metadata.type_key, EEPROM_CONFIG8_MAP1);
  loadTable(&trim2Table, trim2Table._metadata.type_key, EEPROM_CONFIG8_MAP2);
  loadTable(&trim3Table, trim3Table._metadata.type_key, EEPROM_CONFIG8_MAP3);
  loadTable(&trim4Table, trim4Table._metadata.type_key, EEPROM_CONFIG8_MAP4);
  loadTable(&trim5Table, trim5Table._metadata.type_key, EEPROM_CONFIG8_MAP5);
  loadTable(&trim6Table, trim6Table._metadata.type_key, EEPROM_CONFIG8_MAP6);
  loadTable(&trim7Table, trim7Table._metadata.type_key, EEPROM_CONFIG8_MAP7);
  loadTable(&trim8Table, trim8Table._metadata.type_key, EEPROM_CONFIG8_MAP8);

  //*********************************************************************************************************************************************************************************
  //canbus control page load
  load_range(EEPROM_CONFIG9_START, (byte *)&configPage9, (byte *)&configPage9+sizeof(configPage9));

  //*********************************************************************************************************************************************************************************

  //CONFIG PAGE (10)
  load_range(EEPROM_CONFIG10_START, (byte *)&configPage10, (byte *)&configPage10+sizeof(configPage10));

  //*********************************************************************************************************************************************************************************
  //Fuel table 2 (See storage.h for data layout)
  loadTable(&fuelTable2, fuelTable2._metadata.type_key, EEPROM_CONFIG11_MAP);

  //*********************************************************************************************************************************************************************************
  // WMI, VVT2 and Dwell table load
  loadTable(&wmiTable, wmiTable._metadata.type_key, EEPROM_CONFIG12_MAP);
  loadTable(&vvt2Table, vvt2Table._metadata.type_key, EEPROM_CONFIG12_MAP2);
  loadTable(&dwellTable, dwellTable._metadata.type_key, EEPROM_CONFIG12_MAP3);

  //*********************************************************************************************************************************************************************************
  //CONFIG PAGE (13)
  load_range(EEPROM_CONFIG13_START, (byte *)&configPage13, (byte *)&configPage13+sizeof(configPage13));

  //*********************************************************************************************************************************************************************************
  //SECOND IGNITION CONFIG PAGE (14)

  loadTable(&ignitionTable2, ignitionTable2._metadata.type_key, EEPROM_CONFIG14_MAP);

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
