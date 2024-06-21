#pragma once

#include "scheduler.h"

/**
 * @defgroup schedule-state-machine Schedule finite state machine 
 * 
 * @brief All Schedule instances move through the same set of states (status)
 * in the same sequence: they are a Finite State Machine (FSM). These methods
 * enforce this ordering for the timer driven state transitions.
 * 
 * The states transitions are (roughly):
 * @verbatim 
 *  PENDING <--
 *     |      |
 *     V      |
 *  RUNNING ---
 *     |
 *     V
 *    OFF
 * @endverbatim
 * 
 * The OFF to PENDING transition is handled by setFuelSchedule & setIgnitionSchedule
 *  
 * @{
 */

/** 
 * @brief The type of function that is called during a state transition. 
 * Captures the actions to take as a result of the state transition.
 */
typedef void (*scheduleStateTranstionFunc)(Schedule *);

/**
 * @brief The engine/pump that moves a schedule through it's various timer driven states.
 * 
 * @note: expectation is that this is called by the same timer interrupt that the schedule
 * is linked to via it's compare & counter members. 
 * 
 * @param schedule The schedule to change the state of
 * @param pendingToRunning Function to call if the schedule is moving from PENDING to RUNNING
 * @param runningToOff Function to call if the schedule is moving from RUNNING to OFF
 * @param runningToPending Function to call if the schedule is moving from RUNNING to PENDING
 */
void movetoNextState(Schedule &schedule, 
                    scheduleStateTranstionFunc pendingToRunning, 
                    scheduleStateTranstionFunc runningToOff,
                    scheduleStateTranstionFunc runningToPending);
                                                                  
/**
 * @brief Default action for PENDING to RUNNING state transition.
 * 
 * This should be called by another function that handles PENDING to RUNNING
 * 
 * @param schedule The schedule that is/will be moving from PENDING to RUNNING
 */
void defaultPendingToRunning(Schedule *schedule);
/**
 * @brief Default action for RUNNING to OFF state transition.
 * 
 * This should be called by another function that handles RUNNING to OFF
 * 
 * @param schedule The schedule that is/will be moving from RUNNING to OFF
 */
void defaultRunningToOff(Schedule *schedule);

/**
 * @brief Default action for RUNNING to PENDING state transition.
 * 
 * This should be called by another function that handles RUNNING to PENDING
 * 
 * @param schedule The schedule that is/will be moving from RUNNING to PENDING
 */
void defaultRunningToPending(Schedule *schedule);

///@}