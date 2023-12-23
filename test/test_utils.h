
#pragma once
#include "table3d.h"

#include <stdint.h>
#include <unity.h>

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

static __inline__ uint8_t ufname_set(const char *newFName)
{
    Unity.TestFile = newFName;
    return 1;
}
static __inline__ void ufname_szrestore(char** __s)
{
    Unity.TestFile = *__s;
    __asm__ volatile ("" ::: "memory");
}

#define UNITY_FILENAME_RESTORE char* _ufname_saved                           \
    __attribute__((__cleanup__(ufname_szrestore))) = (char*)Unity.TestFile

#define SET_UNITY_FILENAME()                                                        \
for ( UNITY_FILENAME_RESTORE, _ufname_done = ufname_set(__FILE__);                  \
    _ufname_done; _ufname_done = 0 )


void populateTable(table3d16RpmLoad &table, const table3d_value_t values[], const table3d_axis_t xAxis[], const table3d_axis_t yAxis[]);
