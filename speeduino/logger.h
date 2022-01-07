/** \file logger.h
 * @brief File for generating log files and meta data
 * @author Josh Stewart
 * 
 * This file contains functions for creating a log file for use eith by TunerStudio directly or to be written to an SD card
 * 
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <assert.h>

#ifndef UNIT_TEST // Scope guard for unit testing
  #define LOG_ENTRY_SIZE      122 /**< The size of the live data packet. This MUST match ochBlockSize setting in the ini file */
  #define SD_LOG_ENTRY_SIZE   122 /**< The size of the live data packet used by the SD card.*/
#else
  #define LOG_ENTRY_SIZE      1 /**< The size of the live data packet. This MUST match ochBlockSize setting in the ini file */
  #define SD_LOG_ENTRY_SIZE   1 /**< The size of the live data packet used by the SD card.*/
#endif

#define SD_LOG_NUM_FIELDS   89 /**< The number of fields that are in the log. This is always smaller than the entry size due to some fields being 2 bytes */

byte getTSLogEntry(uint16_t);
int16_t getReadableLogEntry(uint16_t);
bool is2ByteEntry(uint8_t);

// This array indicates which index values from the log are 2 byte values
// This array MUST remain in ascending order
// !!!! WARNING: If any value above 255 is required in this array, changes MUST be made to is2ByteEntry() function !!!!
const byte PROGMEM fsIntIndex[] = {4, 14, 17, 25, 27, 32, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61, 63, 65, 67, 69, 71, 75, 77, 79, 81, 85, 87, 89, 93, 97, 102, 109, 119 };

//List of logger field names. This must be in the same order and length as logger_updateLogdataCSV()
const char header_0[] PROGMEM = "secl";
const char header_1[] PROGMEM = "status1";
const char header_2[] PROGMEM = "engine";
const char header_3[] PROGMEM = "Sync Loss #";
const char header_4[] PROGMEM = "MAP";
const char header_5[] PROGMEM = "IAT(C)";
const char header_6[] PROGMEM = "CLT(C)";
const char header_7[] PROGMEM = "Battery Correction";
const char header_8[] PROGMEM = "Battery V";
const char header_9[] PROGMEM = "AFR";
const char header_10[] PROGMEM = "EGO Correction";
const char header_11[] PROGMEM = "IAT Correction";
const char header_12[] PROGMEM = "WUE Correction";
const char header_13[] PROGMEM = "RPM";
const char header_14[] PROGMEM = "Accel. Correction";
const char header_15[] PROGMEM = "Gamma Correction";
const char header_16[] PROGMEM = "VE1";
const char header_17[] PROGMEM = "VE2";
const char header_18[] PROGMEM = "AFR Target";
const char header_19[] PROGMEM = "TPSdot";
const char header_20[] PROGMEM = "Advance Current";
const char header_21[] PROGMEM = "TPS";
const char header_22[] PROGMEM = "Loops/S";
const char header_23[] PROGMEM = "Free RAM";
const char header_24[] PROGMEM = "Boost Target";
const char header_25[] PROGMEM = "Boost Duty";
const char header_26[] PROGMEM = "status2";
const char header_27[] PROGMEM = "rpmDOT";
const char header_28[] PROGMEM = "Eth%";
const char header_29[] PROGMEM = "Flex Fuel Correction";
const char header_30[] PROGMEM = "Flex Adv Correction";
const char header_31[] PROGMEM = "IAC Steps/Duty";
const char header_32[] PROGMEM = "testoutputs";
const char header_33[] PROGMEM = "AFR2";
const char header_34[] PROGMEM = "Baro";
const char header_35[] PROGMEM = "AUX_IN 0";
const char header_36[] PROGMEM = "AUX_IN 1";
const char header_37[] PROGMEM = "AUX_IN 2";
const char header_38[] PROGMEM = "AUX_IN 3";
const char header_39[] PROGMEM = "AUX_IN 4";
const char header_40[] PROGMEM = "AUX_IN 5";
const char header_41[] PROGMEM = "AUX_IN 6";
const char header_42[] PROGMEM = "AUX_IN 7";
const char header_43[] PROGMEM = "AUX_IN 8";
const char header_44[] PROGMEM = "AUX_IN 9";
const char header_45[] PROGMEM = "AUX_IN 10";
const char header_46[] PROGMEM = "AUX_IN 11";
const char header_47[] PROGMEM = "AUX_IN 12";
const char header_48[] PROGMEM = "AUX_IN 13";
const char header_49[] PROGMEM = "AUX_IN 14";
const char header_50[] PROGMEM = "AUX_IN 15";
const char header_51[] PROGMEM = "TPS ADC";
const char header_52[] PROGMEM = "Errors";
const char header_53[] PROGMEM = "PW";
const char header_54[] PROGMEM = "PW2";
const char header_55[] PROGMEM = "PW3";
const char header_56[] PROGMEM = "PW4";
const char header_57[] PROGMEM = "status3";
const char header_58[] PROGMEM = "Engine Protect";
const char header_59[] PROGMEM = "";
const char header_60[] PROGMEM = "Fuel Load";
const char header_61[] PROGMEM = "Ign Load";
const char header_62[] PROGMEM = "Dwell";
const char header_63[] PROGMEM = "Idle Target (RPM)";
const char header_64[] PROGMEM = "MAP DOT";
const char header_65[] PROGMEM = "VVT1 Angle";
const char header_66[] PROGMEM = "VVT1 Target";
const char header_67[] PROGMEM = "VVT1 Duty";
const char header_68[] PROGMEM = "Flex Boost Adj";
const char header_69[] PROGMEM = "Baro Correction";
const char header_70[] PROGMEM = "VE Current";
const char header_71[] PROGMEM = "ASE Correction";
const char header_72[] PROGMEM = "Vehicle Speed";
const char header_73[] PROGMEM = "Gear";
const char header_74[] PROGMEM = "Fuel Pressure";
const char header_75[] PROGMEM = "Oil Pressure";
const char header_76[] PROGMEM = "WMI PW";
const char header_77[] PROGMEM = "status4";
const char header_78[] PROGMEM = "VVT2 Angle";
const char header_79[] PROGMEM = "VVT2 Target";
const char header_80[] PROGMEM = "VVT2 Duty";
const char header_81[] PROGMEM = "outputs";
const char header_82[] PROGMEM = "Fuel Temp";
const char header_83[] PROGMEM = "Fuel Temp Correction";
const char header_84[] PROGMEM = "Advance 1";
const char header_85[] PROGMEM = "Advance 2";
const char header_86[] PROGMEM = "SD Status";
const char header_87[] PROGMEM = "EMAP";
const char header_88[] PROGMEM = "Fan Duty";
/*
const char header_89[] PROGMEM = "";
const char header_90[] PROGMEM = "";
const char header_91[] PROGMEM = "";
const char header_92[] PROGMEM = "";
const char header_93[] PROGMEM = "";
const char header_94[] PROGMEM = "";
const char header_95[] PROGMEM = "";
const char header_96[] PROGMEM = "";
const char header_97[] PROGMEM = "";
const char header_98[] PROGMEM = "";
const char header_99[] PROGMEM = "";
const char header_100[] PROGMEM = "";
const char header_101[] PROGMEM = "";
const char header_102[] PROGMEM = "";
const char header_103[] PROGMEM = "";
const char header_104[] PROGMEM = "";
const char header_105[] PROGMEM = "";
const char header_106[] PROGMEM = "";
const char header_107[] PROGMEM = "";
const char header_108[] PROGMEM = "";
const char header_109[] PROGMEM = "";
const char header_110[] PROGMEM = "";
const char header_111[] PROGMEM = "";
const char header_112[] PROGMEM = "";
const char header_113[] PROGMEM = "";
const char header_114[] PROGMEM = "";
const char header_115[] PROGMEM = "";
const char header_116[] PROGMEM = "";
const char header_117[] PROGMEM = "";
const char header_118[] PROGMEM = "";
const char header_119[] PROGMEM = "";
const char header_120[] PROGMEM = "";
const char header_121[] PROGMEM = "";
*/

const char* const header_table[] PROGMEM = {  header_0,\
                                              header_1,\
                                              header_2,\
                                              header_3,\
                                              header_4,\
                                              header_5,\
                                              header_6,\
                                              header_7,\
                                              header_8,\
                                              header_9,\
                                              header_10,\
                                              header_11,\
                                              header_12,\
                                              header_13,\
                                              header_14,\
                                              header_15,\
                                              header_16,\
                                              header_17,\
                                              header_18,\
                                              header_19,\
                                              header_20,\
                                              header_21,\
                                              header_22,\
                                              header_23,\
                                              header_24,\
                                              header_25,\
                                              header_26,\
                                              header_27,\
                                              header_28,\
                                              header_29,\
                                              header_30,\
                                              header_31,\
                                              header_32,\
                                              header_33,\
                                              header_34,\
                                              header_35,\
                                              header_36,\
                                              header_37,\
                                              header_38,\
                                              header_39,\
                                              header_40,\
                                              header_41,\
                                              header_42,\
                                              header_43,\
                                              header_44,\
                                              header_45,\
                                              header_46,\
                                              header_47,\
                                              header_48,\
                                              header_49,\
                                              header_50,\
                                              header_51,\
                                              header_52,\
                                              header_53,\
                                              header_54,\
                                              header_55,\
                                              header_56,\
                                              header_57,\
                                              header_58,\
                                              header_59,\
                                              header_60,\
                                              header_61,\
                                              header_62,\
                                              header_63,\
                                              header_64,\
                                              header_65,\
                                              header_66,\
                                              header_67,\
                                              header_68,\
                                              header_69,\
                                              header_70,\
                                              header_71,\
                                              header_72,\
                                              header_73,\
                                              header_74,\
                                              header_75,\
                                              header_76,\
                                              header_77,\
                                              header_78,\
                                              header_79,\
                                              header_80,\
                                              header_81,\
                                              header_82,\
                                              header_83,\
                                              header_84,\
                                              header_85,\
                                              header_86,\
                                              header_87,\
                                              header_88,\
                                              /*
                                              header_89,\
                                              header_90,\
                                              header_91,\
                                              header_92,\
                                              header_93,\
                                              header_94,\
                                              header_95,\
                                              header_96,\
                                              header_97,\
                                              header_98,\
                                              header_99,\
                                              header_100,\
                                              header_101,\
                                              header_102,\
                                              header_103,\
                                              header_104,\
                                              header_105,\
                                              header_106,\
                                              header_107,\
                                              header_108,\
                                              header_109,\
                                              header_110,\
                                              header_111,\
                                              header_112,\
                                              header_113,\
                                              header_114,\
                                              header_115,\
                                              header_116,\
                                              header_117,\
                                              header_118,\
                                              header_119,\
                                              header_120,\
                                              header_121,\
                                              */
                                            };

static_assert(sizeof(header_table) == (sizeof(char*) * SD_LOG_NUM_FIELDS), "Number of header table titles must match number of log fields");

#endif
