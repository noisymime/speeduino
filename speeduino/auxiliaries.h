#ifndef AUX_H
#define AUX_H

#include BOARD_H //Note that this is not a real file, it is defined in globals.h. 

void initialiseAuxPWM();
void boostControl();
void boostDisable();
void idleControl();
void vvtControl();
void initialiseFan();
void initialiseAirCon();
void nitrousControl();
void fanControl();
void airConControl();
void wmiControl();

bool READ_AIRCON_REQUEST();

#define SIMPLE_BOOST_P  1
#define SIMPLE_BOOST_I  1
#define SIMPLE_BOOST_D  1

#define BOOST_PIN_LOW()  *boost_pin_port &= ~(boost_pin_mask)
#define BOOST_PIN_HIGH() *boost_pin_port |= (boost_pin_mask)
#define VVT1_PIN_LOW()    *vvt1_pin_port &= ~(vvt1_pin_mask)
#define VVT1_PIN_HIGH()   *vvt1_pin_port |= (vvt1_pin_mask)
#define VVT2_PIN_LOW()    *vvt2_pin_port &= ~(vvt2_pin_mask)
#define VVT2_PIN_HIGH()   *vvt2_pin_port |= (vvt2_pin_mask)
#define VVT1_PIN_ON()     VVT1_PIN_HIGH();
#define VVT1_PIN_OFF()    VVT1_PIN_LOW();
#define VVT2_PIN_ON()     VVT2_PIN_HIGH();
#define VVT2_PIN_OFF()    VVT2_PIN_LOW();
#define FAN_PIN_LOW()    *fan_pin_port &= ~(fan_pin_mask)
#define FAN_PIN_HIGH()   *fan_pin_port |= (fan_pin_mask)
#define AIRCON_ON()      {(((configPage13.airConCompPol&1==1)) ? AIRCON_PIN_LOW() : AIRCON_PIN_HIGH()); BIT_SET(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR);}
#define AIRCON_OFF()      {(((configPage13.airConCompPol&1==1)) ? AIRCON_PIN_HIGH() : AIRCON_PIN_LOW()); BIT_CLEAR(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR);}
#define AIRCON_PIN_LOW()    *aircon_comp_pin_port &= ~(aircon_comp_pin_mask)
#define AIRCON_PIN_HIGH()   *aircon_comp_pin_port |= (aircon_comp_pin_mask)
#define N2O_STAGE1_PIN_LOW()  *n2o_stage1_pin_port &= ~(n2o_stage1_pin_mask)
#define N2O_STAGE1_PIN_HIGH() *n2o_stage1_pin_port |= (n2o_stage1_pin_mask)
#define N2O_STAGE2_PIN_LOW()  *n2o_stage2_pin_port &= ~(n2o_stage2_pin_mask)
#define N2O_STAGE2_PIN_HIGH() *n2o_stage2_pin_port |= (n2o_stage2_pin_mask)
#define READ_N2O_ARM_PIN()    ((*n2o_arming_pin_port & n2o_arming_pin_mask) ? true : false)

#define VVT_TIME_DELAY_MULTIPLIER  50

#define FAN_ON()         ((configPage6.fanInv) ? FAN_PIN_LOW() : FAN_PIN_HIGH())
#define FAN_OFF()        ((configPage6.fanInv) ? FAN_PIN_HIGH() : FAN_PIN_LOW())

#define WMI_TANK_IS_EMPTY() ((configPage10.wmiEmptyEnabled) ? ((configPage10.wmiEmptyPolarity) ? digitalRead(pinWMIEmpty) : !digitalRead(pinWMIEmpty)) : 0)

volatile PORT_TYPE *boost_pin_port;
volatile PINMASK_TYPE boost_pin_mask;
volatile PORT_TYPE *vvt1_pin_port;
volatile PINMASK_TYPE vvt1_pin_mask;
volatile PORT_TYPE *vvt2_pin_port;
volatile PINMASK_TYPE vvt2_pin_mask;
volatile PORT_TYPE *fan_pin_port;
volatile PINMASK_TYPE fan_pin_mask;
volatile PORT_TYPE *aircon_comp_pin_port;
volatile PINMASK_TYPE aircon_comp_pin_mask;
volatile PORT_TYPE *aircon_req_pin_port;
volatile PINMASK_TYPE aircon_req_pin_mask;
volatile PORT_TYPE *n2o_stage1_pin_port;
volatile PINMASK_TYPE n2o_stage1_pin_mask;
volatile PORT_TYPE *n2o_stage2_pin_port;
volatile PINMASK_TYPE n2o_stage2_pin_mask;
volatile PORT_TYPE *n2o_arming_pin_port;
volatile PINMASK_TYPE n2o_arming_pin_mask;


volatile bool boost_pwm_state;
unsigned int boost_pwm_max_count; //Used for variable PWM frequency
volatile unsigned int boost_pwm_cur_value;
long boost_pwm_target_value;
long boost_cl_target_boost;
byte boostCounter;
byte vvtCounter;
uint32_t vvtWarmTime;
bool vvtIsHot;
bool vvtTimeHold;

byte fanHIGH = HIGH;             // Used to invert the cooling fan output
byte fanLOW = LOW;               // Used to invert the cooling fan output

uint8_t acStartDelay;
uint8_t acTPSLockoutDelay;
uint8_t engineRunSeconds;
bool waitedAfterCranking; // This starts false and prevents the A/C from running until a few seconds after cranking

volatile bool vvt1_pwm_state;
volatile bool vvt2_pwm_state;
volatile bool vvt1_max_pwm;
volatile bool vvt2_max_pwm;
volatile char nextVVT;
unsigned int vvt_pwm_max_count; //Used for variable PWM frequency
volatile unsigned int vvt1_pwm_cur_value;
volatile unsigned int vvt2_pwm_cur_value;
long vvt1_pwm_value;
long vvt2_pwm_value;
long vvt_pid_target_angle;
long vvt2_pid_target_angle;
long vvt_pid_current_angle;
long vvt2_pid_current_angle;

void boostInterrupt();
void vvtInterrupt();


#endif
