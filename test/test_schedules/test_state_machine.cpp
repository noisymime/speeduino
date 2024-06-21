
#include <Arduino.h>
#include <unity.h>
#include "../test_utils.h"
#include "schedule_state_machine.h"

static int16_t startCount = 0;
static void startCallback(void) { ++startCount; }

static int16_t endCount = 0;
static void endCallback(void) { ++endCount; }

static constexpr COMPARE_TYPE TIMER_VARIANCE = 5;

static void test_defaultPendingToRunning(void) {
    Schedule schedule(IGN1_COUNTER, IGN1_COMPARE);
    setCallbacks(schedule, startCallback, endCallback);

    schedule.Status = PENDING;
    startCount = 0;
    endCount = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        IGN1_TIMER_DISABLE();
        schedule.Duration = uS_TO_TIMER_COMPARE(2048); 
#if defined(CORE_AVR)        
        COUNTER_TYPE counterPreAction = schedule._counter;
#endif
       
        defaultPendingToRunning(&schedule);

        TEST_ASSERT_EQUAL(RUNNING, schedule.Status);
#if defined(CORE_AVR)        
        TEST_ASSERT_UINT32_WITHIN(TIMER_VARIANCE, counterPreAction+schedule.Duration, schedule._compare);
#endif
        TEST_ASSERT_EQUAL(1, startCount);
        TEST_ASSERT_EQUAL(0, endCount);
    }
}

static void test_defaultRunningToOff(void) {
    Schedule schedule(IGN1_COUNTER, IGN1_COMPARE);
    setCallbacks(schedule, startCallback, endCallback);

    schedule.Status = RUNNING;
    startCount = 0;
    endCount = 0;
    
    defaultRunningToOff(&schedule);

    TEST_ASSERT_EQUAL(OFF, schedule.Status);
    TEST_ASSERT_EQUAL(0, startCount);
    TEST_ASSERT_EQUAL(1, endCount);

    schedule.Status = RUNNING_WITHNEXT;
    startCount = 0;
    endCount = 0;
    
    defaultRunningToOff(&schedule);

    TEST_ASSERT_EQUAL(OFF, schedule.Status);
    TEST_ASSERT_EQUAL(0, startCount);
    TEST_ASSERT_EQUAL(1, endCount);
}

static void test_defaultRunningToPending(void) {
    Schedule schedule(IGN1_COUNTER, IGN1_COMPARE);
    setCallbacks(schedule, startCallback, endCallback);

    schedule.Status = RUNNING;
    startCount = 0;
    endCount = 0;
   
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        IGN1_TIMER_DISABLE();
        schedule.nextStartCompare = schedule._counter + uS_TO_TIMER_COMPARE(2048); 

        defaultRunningToPending(&schedule);

        TEST_ASSERT_EQUAL(PENDING, schedule.Status);
#if defined(CORE_AVR)        
        TEST_ASSERT_UINT32_WITHIN(TIMER_VARIANCE, schedule.nextStartCompare, schedule._compare);
#endif
        TEST_ASSERT_EQUAL(0, startCount);
        TEST_ASSERT_EQUAL(1, endCount);
    }
}

static uint8_t pendingToRunningCount;
static Schedule *pPendingToRunningSchedule;
static void pendingToRunning(Schedule *pSchedule) {
    ++pendingToRunningCount;
    pPendingToRunningSchedule = pSchedule;
}

static uint8_t runningToOffCount;
static Schedule *pRunningToOffSchedule;
static void runningToOff(Schedule *pSchedule) {
    ++runningToOffCount;
    pRunningToOffSchedule = pSchedule;
}

static uint8_t runningToPendingCount;
static Schedule *pRunningToPendingSchedule;
static void runningToPending(Schedule *pSchedule) {
    ++runningToPendingCount;
    pRunningToPendingSchedule = pSchedule;
}

static void test_movetoNextState_pendingToRunning(void) {
    Schedule schedule(IGN1_COUNTER, IGN1_COMPARE);
    setCallbacks(schedule, startCallback, endCallback);

    schedule.Status = PENDING;
    pendingToRunningCount = 0;
    pPendingToRunningSchedule = NULL;
    runningToOffCount = 0;
    pRunningToOffSchedule = NULL;
    runningToPendingCount = 0;
    pRunningToPendingSchedule = NULL;
   
    movetoNextState(schedule, &pendingToRunning, &runningToOff, &runningToPending);

    TEST_ASSERT_EQUAL(1, pendingToRunningCount);
    TEST_ASSERT_EQUAL(&schedule, pPendingToRunningSchedule);
    TEST_ASSERT_EQUAL(0, runningToOffCount);
    TEST_ASSERT_NULL(pRunningToOffSchedule);
    TEST_ASSERT_EQUAL(0, runningToPendingCount);
    TEST_ASSERT_NULL(pRunningToPendingSchedule);
}

static void test_movetoNextState_runningToOff(void) {
    Schedule schedule(IGN1_COUNTER, IGN1_COMPARE);
    setCallbacks(schedule, startCallback, endCallback);

    schedule.Status = RUNNING;
    pendingToRunningCount = 0;
    pPendingToRunningSchedule = NULL;
    runningToOffCount = 0;
    pRunningToOffSchedule = NULL;
    runningToPendingCount = 0;
    pRunningToPendingSchedule = NULL;
   
    movetoNextState(schedule, &pendingToRunning, &runningToOff, &runningToPending);

    TEST_ASSERT_EQUAL(0, pendingToRunningCount);
    TEST_ASSERT_NULL(pPendingToRunningSchedule);
    TEST_ASSERT_EQUAL(1, runningToOffCount);
    TEST_ASSERT_EQUAL(&schedule, pRunningToOffSchedule);
    TEST_ASSERT_EQUAL(0, runningToPendingCount);
    TEST_ASSERT_NULL(pRunningToPendingSchedule);
}

static void test_movetoNextState_runningToPending(void) {
    Schedule schedule(IGN1_COUNTER, IGN1_COMPARE);
    setCallbacks(schedule, startCallback, endCallback);

    schedule.Status = RUNNING_WITHNEXT;
    pendingToRunningCount = 0;
    pPendingToRunningSchedule = NULL;
    runningToOffCount = 0;
    pRunningToOffSchedule = NULL;
    runningToPendingCount = 0;
    pRunningToPendingSchedule = NULL;
   
    movetoNextState(schedule, &pendingToRunning, &runningToOff, &runningToPending);

    TEST_ASSERT_EQUAL(0, pendingToRunningCount);
    TEST_ASSERT_NULL(pPendingToRunningSchedule);
    TEST_ASSERT_EQUAL(0, runningToOffCount);
    TEST_ASSERT_NULL(pRunningToOffSchedule);
    TEST_ASSERT_EQUAL(1, runningToPendingCount);
    TEST_ASSERT_EQUAL(&schedule, pRunningToPendingSchedule);
}

static void test_movetoNextState_off(void) {
    Schedule schedule(IGN1_COUNTER, IGN1_COMPARE);
    setCallbacks(schedule, startCallback, endCallback);

    schedule.Status = OFF;
    pendingToRunningCount = 0;
    pPendingToRunningSchedule = NULL;
    runningToOffCount = 0;
    pRunningToOffSchedule = NULL;
    runningToPendingCount = 0;
    pRunningToPendingSchedule = NULL;
   
    movetoNextState(schedule, &defaultPendingToRunning, &defaultRunningToOff, &defaultRunningToPending);
    TEST_ASSERT_EQUAL(0, pendingToRunningCount);
    TEST_ASSERT_NULL(pPendingToRunningSchedule);
    TEST_ASSERT_EQUAL(0, runningToOffCount);
    TEST_ASSERT_NULL(pRunningToOffSchedule);
    TEST_ASSERT_EQUAL(0, runningToPendingCount);
    TEST_ASSERT_NULL(pRunningToPendingSchedule);
   
    movetoNextState(schedule, &defaultPendingToRunning, &defaultRunningToOff, &defaultRunningToPending);
    TEST_ASSERT_EQUAL(0, pendingToRunningCount);
    TEST_ASSERT_NULL(pPendingToRunningSchedule);
    TEST_ASSERT_EQUAL(0, runningToOffCount);
    TEST_ASSERT_NULL(pRunningToOffSchedule);
    TEST_ASSERT_EQUAL(0, runningToPendingCount);
    TEST_ASSERT_NULL(pRunningToPendingSchedule);
}

void testScheduleStateMachine(void)
{
  SET_UNITY_FILENAME() {
    RUN_TEST_P(test_defaultPendingToRunning);
    RUN_TEST_P(test_defaultRunningToOff);
    RUN_TEST_P(test_defaultRunningToPending);
    RUN_TEST_P(test_movetoNextState_pendingToRunning);
    RUN_TEST_P(test_movetoNextState_runningToOff);
    RUN_TEST_P(test_movetoNextState_runningToPending);
    RUN_TEST_P(test_movetoNextState_off);
  }
}
