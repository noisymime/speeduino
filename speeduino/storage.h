#ifndef STORAGE_H
#define STORAGE_H

/** @file storage.h
 * @brief Functions for reading and writing user settings to/from EEPROM
 *
 * Current layout of EEPROM is as follows (Version 18):
 *
 * |Offset (Dec)|Size (Bytes)| Description                          | Reference                          |
 * | ---------: | :--------: | :----------------------------------: | :--------------------------------- |
 * | 0          |1           | EEPROM version                       | @ref EEPROM_DATA_VERSION           |
 * | 1          |2           | X and Y sizes for fuel table         |                                    |
 * | 3          |256         | Fuel table (16x16)                   | @ref EEPROM_CONFIG1_MAP            |
 * | 259        |16          | Fuel table (X axis) (RPM)            |                                    |
 * | 275        |16          | Fuel table (Y axis) (MAP/TPS)        |                                    |
 * | 291        |128         | Page 2 settings                      | @ref EEPROM_CONFIG2_START          |
 * | 419        |2           | X and Y sizes for ignition table     |                                    |
 * | 421        |256         | Ignition table (16x16)               | @ref EEPROM_CONFIG3_MAP            |
 * | 677        |16          | Ignition table (X axis) (RPM)        |                                    |
 * | 693        |16          | Ignition table (Y axis) (MAP/TPS)    |                                    |
 * | 709        |128         | Page 4 settings                      | @ref EEPROM_CONFIG4_START          |
 * | 837        |2           | X and Y sizes for AFR target table   |                                    |
 * | 839        |256         | AFR target table (16x16)             | @ref EEPROM_CONFIG5_MAP            |
 * | 1095       |16          | AFR target table (X axis) (RPM)      |                                    |
 * | 1111       |16          | AFR target table (Y axis) (MAP/TPS)  |                                    |
 * | 1127       |128         | Page 6 settings                      | @ref EEPROM_CONFIG6_START          |
 * | 1255       |2           | X and Y sizes for boost table        |                                    |
 * | 1257       |64          | Boost table (8x8)                    | @ref EEPROM_CONFIG7_MAP1           |
 * | 1321       |8           | Boost table (X axis) (RPM)           |                                    |
 * | 1329       |8           | Boost table (Y axis) (TPS)           |                                    |
 * | 1337       |2           | X and Y sizes for vvt table          |                                    |
 * | 1339       |64          | VVT table (8x8)                      | @ref EEPROM_CONFIG7_MAP2           |
 * | 1403       |8           | VVT table (X axis) (RPM)             |                                    |
 * | 1411       |8           | VVT table (Y axis) (MAP)             |                                    |
 * | 1419       |2           | X and Y sizes for staging table      |                                    |
 * | 1421       |64          | Staging table (8x8)                  | @ref EEPROM_CONFIG7_MAP3           |
 * | 1485       |8           | Staging table (X axis) (RPM)         |                                    |
 * | 1493       |8           | Staging table (Y axis) (MAP)         |                                    |
 * | 1501       |2           | X and Y sizes for trim1 table        |                                    |
 * | 1503       |36          | Trim1 table (6x6)                    | @ref EEPROM_CONFIG8_MAP1           |
 * | 1539       |6           | Trim1 table (X axis) (RPM)           |                                    |
 * | 1545       |6           | Trim1 table (Y axis) (MAP)           |                                    |
 * | 1551       |2           | X and Y sizes for trim2 table        |                                    |
 * | 1553       |36          | Trim2 table (6x6)                    | @ref EEPROM_CONFIG8_MAP2           |
 * | 1589       |6           | Trim2 table (X axis) (RPM)           |                                    |
 * | 1595       |6           | Trim2 table (Y axis) (MAP)           |                                    |
 * | 1601       |2           | X and Y sizes for trim3 table        |                                    |
 * | 1603       |36          | Trim3 table (6x6)                    | @ref EEPROM_CONFIG8_MAP3           |
 * | 1639       |6           | Trim3 table (X axis) (RPM)           |                                    |
 * | 1545       |6           | Trim3 table (Y axis) (MAP)           |                                    |
 * | 1651       |2           | X and Y sizes for trim4 table        |                                    |
 * | 1653       |36          | Trim4 table (6x6)                    | @ref EEPROM_CONFIG8_MAP4           |
 * | 1689       |6           | Trim4 table (X axis) (RPM)           |                                    |
 * | 1595       |6           | Trim4 table (Y axis) (MAP)           |                                    |
 * | 1701       |9           | HOLE ??                              |                                    |
 * | 1710       |192         | Page 9 settings                      | @ref EEPROM_CONFIG9_START          |
 * | 1902       |192         | Page 10 settings                     | @ref EEPROM_CONFIG10_START         |
 * | 2094       |2           | X and Y sizes for fuel2 table        |                                    |
 * | 2096       |256         | Fuel2 table (16x16)                  | @ref EEPROM_CONFIG11_MAP           |
 * | 2352       |16          | Fuel2 table (X axis) (RPM)           |                                    |
 * | 2368       |16          | Fuel2 table (Y axis) (MAP/TPS)       |                                    |
 * | 2384       |1           | HOLE ??                              |                                    |
 * | 2385       |2           | X and Y sizes for WMI table          |                                    |
 * | 2387       |64          | WMI table (8x8)                      | @ref EEPROM_CONFIG12_MAP           |
 * | 2451       |8           | WMI table (X axis) (RPM)             |                                    |
 * | 2459       |8           | WMI table (Y axis) (MAP)             |                                    |
 * | 2467       |2           | X and Y sizes VVT2 table             |                                    |
 * | 2469       |64          | VVT2 table (8x8)                     | @ref EEPROM_CONFIG12_MAP2          |
 * | 2553       |8           | VVT2 table (X axis) (RPM)            |                                    |
 * | 2541       |8           | VVT2 table (Y axis) (MAP)            |                                    |
 * | 2549       |2           | X and Y sizes dwell table            |                                    |
 * | 2551       |16          | Dwell table (4x4)                    | @ref EEPROM_CONFIG12_MAP3          |
 * | 2567       |4           | Dwell table (X axis) (RPM)           |                                    |
 * | 2571       |4           | Dwell table (Y axis) (MAP)           |                                    |
 * | 2575       |5           | HOLE ??                              |                                    |
 * | 2580       |128         | Page 13 settings                     | @ref EEPROM_CONFIG13_START         |
 * | 2708       |2           | X and Y sizes for ignition2 table    |                                    |
 * | 2710       |256         | Ignition2 table (16x16)              | @ref EEPROM_CONFIG14_MAP           |
 * | 2966       |16          | Ignition2 table (X axis) (RPM)       |                                    |
 * | 2982       |16          | Ignition2 table (Y axis) (MAP/TPS)   |                                    |
 * | 2998       |1           | HOLE ??                              |                                    |
 * | 2999       |2           | X and Y sizes for trim5 table        |                                    |
 * | 3001       |36          | Trim5 table (6x6)                    | @ref EEPROM_CONFIG8_MAP5           |
 * | 3037       |6           | Trim5 table (X axis) (RPM)           |                                    |
 * | 3043       |6           | Trim5 table (Y axis) (MAP)           |                                    |
 * | 3049       |2           | X and Y sizes for trim6 table        |                                    |
 * | 3051       |36          | Trim6 table (6x6)                    | @ref EEPROM_CONFIG8_MAP6           |
 * | 3087       |6           | Trim6 table (X axis) (RPM)           |                                    |
 * | 3093       |6           | Trim6 table (Y axis) (MAP)           |                                    |
 * | 3099       |2           | X and Y sizes for trim7 table        |                                    |
 * | 3101       |36          | Trim7 table (6x6)                    | @ref EEPROM_CONFIG8_MAP7           |
 * | 3137       |6           | Trim7 table (X axis) (RPM)           |                                    |
 * | 3143       |6           | Trim7 table (Y axis) (MAP)           |                                    |
 * | 3149       |2           | X and Y sizes for trim8 table        |                                    |
 * | 3151       |36          | Trim8 table (6x6)                    | @ref EEPROM_CONFIG8_MAP8           |
 * | 3187       |6           | Trim8 table (X axis) (RPM)           |                                    |
 * | 3193       |6           | Trim8 table (Y axis) (MAP)           |                                    |
 * | 3199       |487         | EMPTY                                |                                    |
 * | 3686       |56          | Page CRC32 sums (4x14)               | Last first, 14 -> 1                |
 * | 3742       |1           | Baro value saved at init             | @ref EEPROM_LAST_BARO              |
 * | 3743       |64          | O2 Calibration Bins                  | @ref EEPROM_CALIBRATION_O2_BINS    |
 * | 3807       |32          | O2 Calibration Values                | @ref EEPROM_CALIBRATION_O2_VALUES  |
 * | 3839       |64          | IAT Calibration Bins                 | @ref EEPROM_CALIBRATION_IAT_BINS   |
 * | 3903       |64          | IAT Calibration Values               | @ref EEPROM_CALIBRATION_IAT_VALUES |
 * | 3967       |64          | CLT Calibration Bins                 | @ref EEPROM_CALIBRATION_CLT_BINS   |
 * | 4031       |64          | CLT Calibration Values               | @ref EEPROM_CALIBRATION_CLT_VALUES |
 * | 4095       |            | END                                  |                                    |
 *
 */

void writeAllConfig();
void writeConfig(uint8_t pageNum);
void loadConfig();
void loadCalibration();
void writeCalibration();
void resetConfigPages();
void enableForceBurn();
void disableForceBurn();

//These are utility functions that prevent other files from having to use EEPROM.h directly
byte readLastBaro();
void storeLastBaro(byte);
uint8_t readEEPROMVersion();
void storeEEPROMVersion(uint8_t);
void storePageCRC32(uint8_t pageNum, uint32_t crcValue);
uint32_t readPageCRC32(uint8_t pageNum);

bool isEepromWritePending();

#define EEPROM_CONFIG1_MAP    3
#define EEPROM_CONFIG2_START  291
#define EEPROM_CONFIG2_END    419
#define EEPROM_CONFIG3_MAP    421
#define EEPROM_CONFIG4_START  709
#define EEPROM_CONFIG4_END    837
#define EEPROM_CONFIG5_MAP    839
#define EEPROM_CONFIG6_START  1127
#define EEPROM_CONFIG6_END    1255
#define EEPROM_CONFIG7_MAP1   1257
#define EEPROM_CONFIG7_MAP2   1339
#define EEPROM_CONFIG7_MAP3   1421
#define EEPROM_CONFIG7_END    1501
#define EEPROM_CONFIG8_MAP1   1503
#define EEPROM_CONFIG8_MAP2   1553
#define EEPROM_CONFIG8_MAP3   1603
#define EEPROM_CONFIG8_MAP4   1653
#define EEPROM_CONFIG9_START  1710
#define EEPROM_CONFIG9_END    1902
#define EEPROM_CONFIG10_START 1902
#define EEPROM_CONFIG10_END   2094
#define EEPROM_CONFIG11_MAP   2096
#define EEPROM_CONFIG11_END   2385
#define EEPROM_CONFIG12_MAP   2387
#define EEPROM_CONFIG12_MAP2  2469
#define EEPROM_CONFIG12_MAP3  2551
#define EEPROM_CONFIG12_END   2575
#define EEPROM_CONFIG13_START 2580
#define EEPROM_CONFIG13_END   2708
#define EEPROM_CONFIG14_MAP   2710
#define EEPROM_CONFIG14_END   2998
//This is OUT OF ORDER as Page 8 was expanded to add fuel trim tables 5-8. The EEPROM for them is simply added here so as not to impact existing tunes
#define EEPROM_CONFIG8_MAP5   3001
#define EEPROM_CONFIG8_MAP6   3051
#define EEPROM_CONFIG8_MAP7   3101
#define EEPROM_CONFIG8_MAP8   3151

//These were the values used previously when all calibration tables were 512 long. They need to be retained so the update process (202005 -> 202008) can work
#define EEPROM_CALIBRATION_O2_OLD   2559
#define EEPROM_CALIBRATION_IAT_OLD  3071
#define EEPROM_CALIBRATION_CLT_OLD  3583

#endif // STORAGE_H
