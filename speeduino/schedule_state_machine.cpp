#include "schedule_state_machine.h"
#include "timers.h"

void defaultPendingToRunning(Schedule *schedule) {
  schedule->pStartCallback();
  schedule->Status = RUNNING; //Set the status to be in progress (ie The start callback has been called, but not the end callback)
  schedule->_compare = schedule->_counter + schedule->Duration;
}

void defaultRunningToOff(Schedule *schedule) {
  schedule->pEndCallback();
  schedule->Status = OFF;
}

void defaultRunningToPending(Schedule *schedule) {
  schedule->pEndCallback();
  schedule->_compare = schedule->nextStartCompare;
  schedule->Status = PENDING;
}

static inline bool hasNextSchedule(const Schedule &schedule) {
  return schedule.Status==RUNNING_WITHNEXT;
}

void movetoNextState(Schedule &schedule, 
                    scheduleStateTranstionFunc pendingToRunning, 
                    scheduleStateTranstionFunc runningToOff,
                    scheduleStateTranstionFunc runningToPending)
{
  if (schedule.Status == PENDING) //Check to see if this schedule is turn on
  {
    pendingToRunning(&schedule);
  }
  else if (isRunning(schedule))
  {
    //If there is a next schedule queued up, activate it
    if(hasNextSchedule(schedule)) {
      runningToPending(&schedule);
    } else {
      runningToOff(&schedule);
    }
  } else {
    // Nothing to do but keep MISRA checker happy
  }
}