#pragma once


//Hardcoded TunerStudio addresses/commands for various SD/RTC commands
#define SD_READWRITE_PAGE   0x11
#define SD_READFILE_PAGE    0x14
#define SD_RTC_PAGE         0x07

#define SD_READ_STAT_ARG1   0x0000
#define SD_READ_STAT_ARG2   0x0010
#define SD_READ_DIR_ARG1    0x0000
#define SD_READ_DIR_ARG2    0x0202
#define SD_READ_SEC_ARG1    0x0002
#define SD_READ_SEC_ARG2    0x0004
#define SD_READ_STRM_ARG1   0x0004
#define SD_READ_STRM_ARG2   0x0001
#define SD_READ_COMP_ARG1   0x0000 //Not used for anything
#define SD_READ_COMP_ARG2   0x0800
#define SD_RTC_READ_ARG1    0x024D
#define SD_RTC_READ_ARG2    0x0008

#define SD_WRITE_DO_ARG1    0x0000
#define SD_WRITE_DO_ARG2    0x0001
#define SD_WRITE_DIR_ARG1   0x0001
#define SD_WRITE_DIR_ARG2   0x0002
#define SD_WRITE_SEC_ARG1   0x0003
#define SD_WRITE_SEC_ARG2   0x0204
#define SD_WRITE_COMP_ARG1  0x0005
#define SD_WRITE_COMP_ARG2  0x0008
#define SD_ERASEFILE_ARG1   0x0006
#define SD_ERASEFILE_ARG2   0x0006
#define SD_SPD_TEST_ARG1    0x0007
#define SD_SPD_TEST_ARG2    0x0004
#define SD_RTC_WRITE_ARG1   0x027E
#define SD_RTC_WRITE_ARG2   0x0009