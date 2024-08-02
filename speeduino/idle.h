#ifndef IDLE_H
#define IDLE_H

#include "globals.h"
#include "table2d.h"
#include "board_definition.h"

#define IAC_ALGORITHM_NONE    0
#define IAC_ALGORITHM_ONOFF   1
#define IAC_ALGORITHM_PWM_OL  2
#define IAC_ALGORITHM_PWM_CL  3
#define IAC_ALGORITHM_STEP_OL 4
#define IAC_ALGORITHM_STEP_CL 5
#define IAC_ALGORITHM_PWM_OLCL  6 //Openloop plus closedloop IAC control
#define IAC_ALGORITHM_STEP_OLCL  7 //Openloop plus closedloop IAC control

#define IDLE_PIN_LOW()  *idle_pin_port &= ~(idle_pin_mask)
#define IDLE_PIN_HIGH() *idle_pin_port |= (idle_pin_mask)
#define IDLE2_PIN_LOW()  *idle2_pin_port &= ~(idle2_pin_mask)
#define IDLE2_PIN_HIGH() *idle2_pin_port |= (idle2_pin_mask)

#define STEPPER_FORWARD 0
#define STEPPER_BACKWARD 1
#define STEPPER_POWER_WHEN_ACTIVE 0
#define IDLE_TABLE_SIZE 10

enum StepperStatus {SOFF, STEPPING, COOLING}; //The 2 statuses that a stepper can have. STEPPING means that a high pulse is currently being sent and will need to be turned off at some point.

struct StepperIdle
{
  int curIdleStep; //Tracks the current location of the stepper
  int targetIdleStep; //What the targeted step is
  volatile StepperStatus stepperStatus;
  volatile unsigned long stepStartTime;
};

extern uint16_t idle_pwm_max_count; //Used for variable PWM frequency
extern long FeedForwardTerm;

void initialiseIdle(bool forcehoming);
void idleControl(void);
void initialiseIdleUpOutput(void);
void disableIdle(void);
void idleInterrupt(void);

#endif
