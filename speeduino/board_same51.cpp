#include "board_definition.h"

#if defined(CORE_SAME51)

void initBoard()
{
    /*
    ***********************************************************************************************************
    * General
    */

    /*
    ***********************************************************************************************************
    * Timers
    */

    /*
    ***********************************************************************************************************
    * Auxiliaries
    */

    /*
    ***********************************************************************************************************
    * Idle
    */

    /*
    ***********************************************************************************************************
    * Schedules
    */
}

uint16_t freeRam()
{
  return 0;
}

void doSystemReset() { return; }
void jumpToBootloader() { return; }

#endif
