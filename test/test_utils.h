
#pragma once

#include <stdint.h>
#include <unity.h>
#include <Arduino.h>
#include "table2d.h"

// Unity macro to reduce memory usage (RAM, .bss)
//
// Unity supplied RUN_TEST captures the function name
// using #func directly in the call to UnityDefaultTestRun.
// This is a raw string that is placed in the data segment,
// which consumes RAM.
//
// So instead, place the function name in flash memory and
// load it at run time.
#define RUN_TEST_P(func) \
  { \
    char funcName[128]; \
    strcpy_P(funcName, PSTR(#func)); \
    UnityDefaultTestRun(func, funcName, __LINE__); \
  }

// ============================ SET_UNITY_FILENAME ============================ 

static inline uint8_t ufname_set(const char *newFName)
{
    Unity.TestFile = newFName;
    return 1;
}

static inline void ufname_szrestore(char** __s)
{
    Unity.TestFile = *__s;
    __asm__ volatile ("" ::: "memory");
}


#define UNITY_FILENAME_RESTORE char* _ufname_saved                           \
    __attribute__((__cleanup__(ufname_szrestore))) = (char*)Unity.TestFile

#define SET_UNITY_FILENAME()                                                        \
for ( UNITY_FILENAME_RESTORE, _ufname_done = ufname_set(__FILE__);                  \
    _ufname_done; _ufname_done = 0 )

// ============================ end SET_UNITY_FILENAME ============================ 


// Store test data in flash, if feasible.
#if defined(PROGMEM)
#define TEST_DATA_P static constexpr PROGMEM
#else
#define TEST_DATA_P static constexpr
#endif

// Populate a 3d table (from PROGMEM if available)
// You wuld typically declare the 3 source arrays usin TEST_DATA_P
template <typename table3d_t>
static inline void populate_table_P(table3d_t &table, 
                                  const table3d_axis_t *pXValues,   // PROGMEM if available
                                  const table3d_axis_t *pYValues,   // PROGMEM if available
                                  const table3d_value_t *pZValues)  // PROGMEM if available
{
  {
    table_axis_iterator itX = table.axisX.begin();
    while (!itX.at_end())
    {
#if defined(PROGMEM)
      *itX = (table3d_axis_t)pgm_read_word(pXValues);
#else
      *itX = *pXValues;
#endif      
      ++pXValues;
      ++itX;
    }
  }  
  {
    table_axis_iterator itY = table.axisY.begin();
    while (!itY.at_end())
    {
#if defined(PROGMEM)
      *itY = (table3d_axis_t)pgm_read_word(pYValues);
#else
      *itY = *pYValues;
#endif      
      ++pYValues;
      ++itY;
    }
  }
  {
    table_value_iterator itZ = table.values.begin();
    while (!itZ.at_end())
    {
      table_row_iterator itRow = *itZ;
      while (!itRow.at_end())
      {
#if defined(PROGMEM)
        *itRow = pgm_read_byte(pZValues);
#else
        *itRow = *pZValues;
#endif
        ++pZValues;
        ++itRow;
      }
      ++itZ;
    }
  }
}

static inline void populate_2dtable(table2D *pTable, uint8_t value, uint8_t bin) {
  for (uint8_t index=0; index<pTable->xSize; ++index) {
    ((uint8_t*)pTable->values)[index] = value;
    ((uint8_t*)pTable->axisX)[index] = bin;
  }
  pTable->cacheTime = UINT8_MAX;
}
