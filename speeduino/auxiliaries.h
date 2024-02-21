#ifndef AUX_H
#define AUX_H

#include BOARD_H //Note that this is not a real file, it is defined in globals.h. 

#include <SimplyAtomic.h>

void initialiseAuxPWM(void);
void boostControl(void);
void boostDisable(void);
void boostByGear(void);
void vvtControl(void);
void initialiseFan(void);
void initialiseAirCon(void);
void nitrousControl(void);
void fanControl(void);
void airConControl(void);
bool READ_AIRCON_REQUEST(void);
void wmiControl(void);

#define SIMPLE_BOOST_P  1
#define SIMPLE_BOOST_I  1
#define SIMPLE_BOOST_D  1

#if(defined(CORE_TEENSY) || defined(CORE_STM32))
#define BOOST_PIN_LOW()         (digitalWrite(pinBoost, LOW))
#define BOOST_PIN_HIGH()        (digitalWrite(pinBoost, HIGH))
#define VVT1_PIN_LOW()          (digitalWrite(pinVVT_1, LOW))
#define VVT1_PIN_HIGH()         (digitalWrite(pinVVT_1, HIGH))
#define VVT2_PIN_LOW()          (digitalWrite(pinVVT_2, LOW))
#define VVT2_PIN_HIGH()         (digitalWrite(pinVVT_2, HIGH))
#define FAN_PIN_LOW()           (digitalWrite(pinFan, LOW))
#define FAN_PIN_HIGH()          (digitalWrite(pinFan, HIGH))
#define N2O_STAGE1_PIN_LOW()    (digitalWrite(configPage10.n2o_stage1_pin, LOW))
#define N2O_STAGE1_PIN_HIGH()   (digitalWrite(configPage10.n2o_stage1_pin, HIGH))
#define N2O_STAGE2_PIN_LOW()    (digitalWrite(configPage10.n2o_stage2_pin, LOW))
#define N2O_STAGE2_PIN_HIGH()   (digitalWrite(configPage10.n2o_stage2_pin, HIGH))
#define AIRCON_PIN_LOW()        (digitalWrite(pinAirConComp, LOW))
#define AIRCON_PIN_HIGH()       (digitalWrite(pinAirConComp, HIGH))
#define AIRCON_FAN_PIN_LOW()    (digitalWrite(pinAirConFan, LOW))
#define AIRCON_FAN_PIN_HIGH()   (digitalWrite(pinAirConFan, HIGH))
#define FUEL_PUMP_ON()          (digitalWrite(pinFuelPump, HIGH))
#define FUEL_PUMP_OFF()         (digitalWrite(pinFuelPump, LOW))

#else

#define BOOST_PIN_LOW()         ATOMIC() { *boost_pin_port &= ~(boost_pin_mask); }
#define BOOST_PIN_HIGH()        ATOMIC() { *boost_pin_port |= (boost_pin_mask);  }
#define VVT1_PIN_LOW()          ATOMIC() { *vvt1_pin_port &= ~(vvt1_pin_mask);   }
#define VVT1_PIN_HIGH()         ATOMIC() { *vvt1_pin_port |= (vvt1_pin_mask);    }
#define VVT2_PIN_LOW()          ATOMIC() { *vvt2_pin_port &= ~(vvt2_pin_mask);   }
#define VVT2_PIN_HIGH()         ATOMIC() { *vvt2_pin_port |= (vvt2_pin_mask);    }
#define N2O_STAGE1_PIN_LOW()    ATOMIC() { *n2o_stage1_pin_port &= ~(n2o_stage1_pin_mask);  }
#define N2O_STAGE1_PIN_HIGH()   ATOMIC() { *n2o_stage1_pin_port |= (n2o_stage1_pin_mask);   }
#define N2O_STAGE2_PIN_LOW()    ATOMIC() { *n2o_stage2_pin_port &= ~(n2o_stage2_pin_mask);  }
#define N2O_STAGE2_PIN_HIGH()   ATOMIC() { *n2o_stage2_pin_port |= (n2o_stage2_pin_mask);   }
#define FUEL_PUMP_ON()          ATOMIC() { *pump_pin_port |= (pump_pin_mask);     }
#define FUEL_PUMP_OFF()         ATOMIC() { *pump_pin_port &= ~(pump_pin_mask);    }

//Note the below macros cannot use ATOMIC() as they are called from within ternary operators. The ATOMIC is instead placed around the ternary call below
#define FAN_PIN_LOW()           *fan_pin_port &= ~(fan_pin_mask)
#define FAN_PIN_HIGH()          *fan_pin_port |= (fan_pin_mask)
#define AIRCON_PIN_LOW()        *aircon_comp_pin_port &= ~(aircon_comp_pin_mask)
#define AIRCON_PIN_HIGH()       *aircon_comp_pin_port |= (aircon_comp_pin_mask)
#define AIRCON_FAN_PIN_LOW()    *aircon_fan_pin_port &= ~(aircon_fan_pin_mask)
#define AIRCON_FAN_PIN_HIGH()   *aircon_fan_pin_port |= (aircon_fan_pin_mask)

#endif

#define AIRCON_ON()             ATOMIC() { ((((configPage15.airConCompPol)==1)) ? AIRCON_PIN_LOW() : AIRCON_PIN_HIGH()); BIT_SET(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); }
#define AIRCON_OFF()            ATOMIC() { ((((configPage15.airConCompPol)==1)) ? AIRCON_PIN_HIGH() : AIRCON_PIN_LOW()); BIT_CLEAR(currentStatus.airConStatus, BIT_AIRCON_COMPRESSOR); }
#define AIRCON_FAN_ON()         ATOMIC() { ((((configPage15.airConFanPol)==1)) ? AIRCON_FAN_PIN_LOW() : AIRCON_FAN_PIN_HIGH()); BIT_SET(currentStatus.airConStatus, BIT_AIRCON_FAN); }
#define AIRCON_FAN_OFF()        ATOMIC() { ((((configPage15.airConFanPol)==1)) ? AIRCON_FAN_PIN_HIGH() : AIRCON_FAN_PIN_LOW()); BIT_CLEAR(currentStatus.airConStatus, BIT_AIRCON_FAN); }

#define FAN_ON()                ATOMIC() { ((configPage6.fanInv) ? FAN_PIN_LOW() : FAN_PIN_HIGH()); }
#define FAN_OFF()               ATOMIC() { ((configPage6.fanInv) ? FAN_PIN_HIGH() : FAN_PIN_LOW()); }

#define READ_N2O_ARM_PIN()    ((*n2o_arming_pin_port & n2o_arming_pin_mask) ? true : false)

#define VVT1_PIN_ON()     VVT1_PIN_HIGH();
#define VVT1_PIN_OFF()    VVT1_PIN_LOW();
#define VVT2_PIN_ON()     VVT2_PIN_HIGH();
#define VVT2_PIN_OFF()    VVT2_PIN_LOW();
#define VVT_TIME_DELAY_MULTIPLIER  50

#define WMI_TANK_IS_EMPTY() ((configPage10.wmiEmptyEnabled) ? ((configPage10.wmiEmptyPolarity) ? digitalRead(pinWMIEmpty) : !digitalRead(pinWMIEmpty)) : 1)

extern volatile PORT_TYPE *vvt1_pin_port;
extern volatile PINMASK_TYPE vvt1_pin_mask;
extern volatile PORT_TYPE *vvt2_pin_port;
extern volatile PINMASK_TYPE vvt2_pin_mask;
extern volatile PORT_TYPE *fan_pin_port;
extern volatile PINMASK_TYPE fan_pin_mask;

#if defined(PWM_FAN_AVAILABLE)//PWM fan not available on Arduino MEGA
extern uint16_t fan_pwm_max_count; //Used for variable PWM frequency
void fanInterrupt(void);
#endif

extern uint16_t vvt_pwm_max_count; //Used for variable PWM frequency
extern uint16_t boost_pwm_max_count; //Used for variable PWM frequency

void boostInterrupt(void);
void vvtInterrupt(void);

#endif