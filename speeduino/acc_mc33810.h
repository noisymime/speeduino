#ifndef MC33810_H
#define MC33810_H

#include <SPI.h>

volatile PORT_TYPE *mc33810_1_pin_port;
volatile PINMASK_TYPE mc33810_1_pin_mask;
volatile PORT_TYPE *mc33810_2_pin_port;
volatile PINMASK_TYPE mc33810_2_pin_mask;

//#define MC33810_ONOFF_CMD   3
static uint8_t MC33810_ONOFF_CMD = 0x30; //48 in decimal
volatile uint8_t mc33810_1_requestedState; //Current binary state of the 1st ICs IGN and INJ values
volatile uint8_t mc33810_2_requestedState; //Current binary state of the 2nd ICs IGN and INJ values
volatile uint8_t mc33810_1_returnState; //Current binary state of the 1st ICs IGN and INJ values
volatile uint8_t mc33810_2_returnState; //Current binary state of the 2nd ICs IGN and INJ values

void initMC33810(void);

#define MC33810_1_ACTIVE() (*mc33810_1_pin_port &= ~(mc33810_1_pin_mask))
#define MC33810_1_INACTIVE() (*mc33810_1_pin_port |= (mc33810_1_pin_mask))
#define MC33810_2_ACTIVE() (*mc33810_2_pin_port &= ~(mc33810_2_pin_mask))
#define MC33810_2_INACTIVE() (*mc33810_2_pin_port |= (mc33810_2_pin_mask))

//These are default values for which injector is attached to which output on the IC. 
//They may (Probably will) be changed during init by the board specific config in init.ino
uint8_t MC33810_BIT_INJ1 = 1;
uint8_t MC33810_BIT_INJ2 = 2;
uint8_t MC33810_BIT_INJ3 = 3;
uint8_t MC33810_BIT_INJ4 = 4;
uint8_t MC33810_BIT_INJ5 = 5;
uint8_t MC33810_BIT_INJ6 = 6;
uint8_t MC33810_BIT_INJ7 = 7;
uint8_t MC33810_BIT_INJ8 = 8;

uint8_t MC33810_BIT_IGN1 = 1;
uint8_t MC33810_BIT_IGN2 = 2;
uint8_t MC33810_BIT_IGN3 = 3;
uint8_t MC33810_BIT_IGN4 = 4;
uint8_t MC33810_BIT_IGN5 = 5;
uint8_t MC33810_BIT_IGN6 = 6;
uint8_t MC33810_BIT_IGN7 = 7;
uint8_t MC33810_BIT_IGN8 = 8;

#define openInjector1_MC33810() MC33810_1_ACTIVE(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_INJ1); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define openInjector2_MC33810() MC33810_1_ACTIVE(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_INJ2); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define openInjector3_MC33810() MC33810_1_ACTIVE(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_INJ3); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define openInjector4_MC33810() MC33810_1_ACTIVE(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_INJ4); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define openInjector5_MC33810() MC33810_2_ACTIVE(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_INJ5); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define openInjector6_MC33810() MC33810_2_ACTIVE(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_INJ6); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define openInjector7_MC33810() MC33810_2_ACTIVE(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_INJ7); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define openInjector8_MC33810() MC33810_2_ACTIVE(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_INJ8); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()

#define closeInjector1_MC33810() MC33810_1_ACTIVE(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_INJ1); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define closeInjector2_MC33810() MC33810_1_ACTIVE(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_INJ2); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define closeInjector3_MC33810() MC33810_1_ACTIVE(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_INJ3); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define closeInjector4_MC33810() MC33810_1_ACTIVE(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_INJ4); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define closeInjector5_MC33810() MC33810_2_ACTIVE(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_INJ5); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define closeInjector6_MC33810() MC33810_2_ACTIVE(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_INJ6); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define closeInjector7_MC33810() MC33810_2_ACTIVE(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_INJ7); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define closeInjector8_MC33810() MC33810_2_ACTIVE(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_INJ8); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()

#define injector1Toggle_MC33810() MC33810_1_ACTIVE(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_INJ1); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define injector2Toggle_MC33810() MC33810_1_ACTIVE(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_INJ2); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define injector3Toggle_MC33810() MC33810_1_ACTIVE(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_INJ3); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define injector4Toggle_MC33810() MC33810_1_ACTIVE(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_INJ4); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define injector5Toggle_MC33810() MC33810_2_ACTIVE(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_INJ5); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define injector6Toggle_MC33810() MC33810_2_ACTIVE(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_INJ6); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define injector7Toggle_MC33810() MC33810_2_ACTIVE(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_INJ7); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define injector8Toggle_MC33810() MC33810_2_ACTIVE(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_INJ8); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()

#define coil1High_MC33810() MC33810_1_ACTIVE(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_IGN1); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define coil2High_MC33810() MC33810_1_ACTIVE(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_IGN2); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
//#define coil1High_MC33810() MC33810_1_ACTIVE(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_IGN1); MC33810_1_INACTIVE()
//#define coil2High_MC33810() MC33810_1_ACTIVE(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_IGN2); MC33810_1_INACTIVE()
#define coil3High_MC33810() MC33810_1_ACTIVE(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_IGN3); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define coil4High_MC33810() MC33810_1_ACTIVE(); BIT_SET(mc33810_1_requestedState, MC33810_BIT_IGN4); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define coil5High_MC33810() MC33810_2_ACTIVE(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_IGN5); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define coil6High_MC33810() MC33810_2_ACTIVE(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_IGN6); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define coil7High_MC33810() MC33810_2_ACTIVE(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_IGN7); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define coil8High_MC33810() MC33810_2_ACTIVE(); BIT_SET(mc33810_2_requestedState, MC33810_BIT_IGN8); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()

#define coil1Low_MC33810() MC33810_1_ACTIVE(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_IGN1); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define coil2Low_MC33810() MC33810_1_ACTIVE(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_IGN2); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define coil3Low_MC33810() MC33810_1_ACTIVE(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_IGN3); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define coil4Low_MC33810() MC33810_1_ACTIVE(); BIT_CLEAR(mc33810_1_requestedState, MC33810_BIT_IGN4); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define coil5Low_MC33810() MC33810_2_ACTIVE(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_IGN5); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define coil6Low_MC33810() MC33810_2_ACTIVE(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_IGN6); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define coil7Low_MC33810() MC33810_2_ACTIVE(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_IGN7); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define coil8Low_MC33810() MC33810_2_ACTIVE(); BIT_CLEAR(mc33810_2_requestedState, MC33810_BIT_IGN8); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()

#define coil1Toggle_MC33810() MC33810_1_ACTIVE(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_IGN1); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define coil2Toggle_MC33810() MC33810_1_ACTIVE(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_IGN2); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define coil3Toggle_MC33810() MC33810_1_ACTIVE(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_IGN3); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define coil4Toggle_MC33810() MC33810_1_ACTIVE(); BIT_TOGGLE(mc33810_1_requestedState, MC33810_BIT_IGN4); mc33810_1_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_1_requestedState)); MC33810_1_INACTIVE()
#define coil5Toggle_MC33810() MC33810_2_ACTIVE(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_IGN5); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define coil6Toggle_MC33810() MC33810_2_ACTIVE(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_IGN6); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define coil7Toggle_MC33810() MC33810_2_ACTIVE(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_IGN7); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()
#define coil8Toggle_MC33810() MC33810_2_ACTIVE(); BIT_TOGGLE(mc33810_2_requestedState, MC33810_BIT_IGN8); mc33810_2_returnState = SPI.transfer16(word(MC33810_ONOFF_CMD, mc33810_2_requestedState)); MC33810_2_INACTIVE()

#endif