#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#ifdef SD_LOGGING

#ifdef __SD_H__
  #include <SD.h>
#else
  #include "SdFat.h"
#endif
//#include <SdSpiCard.h>
#include "RingBuf.h"


#define SD_STATUS_OFF               0 /**< SD system is inactive. FS and file remain closed */
#define SD_STATUS_READY             1 /**< File has been openeed and preallocated, but a log session has not commenced */
#define SD_STATUS_ACTIVE            2 /**< Log session commenced */
#define SD_STATUS_ERROR_NO_CARD     3 /**< No SD card found when attempting to open file */
#define SD_STATUS_ERROR_NO_FS       4 /**< No filesystem found when attempting to open file */
#define SD_STATUS_ERROR_NO_WRITE    5 /**< Card and filesystem found, however file creation failed due to no write access */
#define SD_STATUS_ERROR_NO_SPACE    6 /**< File could not be preallocated as there is not enough space on card */
#define SD_STATUS_ERROR_WRITE_FAIL  7 /**< Log file created and opened, but a sector write failed during logging */
#define SD_STATUS_ERROR_FORMAT_FAIL 8 /**< Attempted formatting of SD card failed */

#define SD_STATUS_CARD_PRESENT      0 //0=no card, 1=card present
#define SD_STATUS_CARD_TYPE         1 //0=SD, 1=SDHC
#define SD_STATUS_CARD_READY        2 //0=not ready, 1=ready
#define SD_STATUS_CARD_LOGGING      3 //0=not logging, 1=logging
#define SD_STATUS_CARD_ERROR        4 //0=no error, 1=error
#define SD_STATUS_CARD_VERSION      5 //0=1.x, 1=2.x
#define SD_STATUS_CARD_FS           6 //0=no FAT16, 1=FAT32
#define SD_STATUS_CARD_UNUSED       7 //0=normal, 1=unused


#define SD_SECTOR_SIZE              512 // Standard SD sector size

#if defined CORE_TEENSY
    #define SD_CS_PIN BUILTIN_SDCARD
#elif defined CORE_STM32
    #define SD_CS_PIN PD2  //CS pin can be pretty much anything, but PD2 is one of the ones left unused from SDIO pins.
#else
    #define SD_CS_PIN 10 //This is a made up value for now
#endif

//Test values only
#define SD_LOG_FILE_SIZE  10000000 //Default 10mb file size
#define MAX_LOG_FILES     10000
#define LOG_FILE_PREFIX "SPD_"
#define LOG_FILE_EXTENSION "csv"
#define RING_BUF_CAPACITY SD_LOG_ENTRY_SIZE * 10 //Allow for 10 entries in the ringbuffer. Will need tuning

/*
Standard FAT16/32
SdFs sd; 
FsFile logFile;
RingBuf<FsFile, RING_BUF_CAPACITY> rb;
*/
//ExFat
extern SdExFat sd;
extern ExFile logFile;
extern RingBuf<ExFile, RING_BUF_CAPACITY> rb;

extern uint8_t SD_status;
extern uint16_t currentLogFileNumber;
extern bool manualLogActive;

void initSD();
void writeSDLogEntry();
void writetSDLogHeader();
void beginSDLogging();
void endSDLogging();
void setTS_SD_status();
void formatExFat();
void deleteLogFile(char, char, char, char);
bool createLogFile();
void dateTime(uint16_t*, uint16_t*, uint8_t*); //Used for timestamping with RTC
uint16_t getNextSDLogFileNumber();
bool getSDLogFileDetails(uint8_t* , uint16_t);
void readSDSectors(uint8_t*, uint32_t, uint16_t);
uint32_t sectorCount();



#endif //SD_LOGGING
#endif //SD_LOGGER_H
