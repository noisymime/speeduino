#include "acc_mc33810.h"
#include "globals.h"
#include <SPI.h>

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

volatile PORT_TYPE *mc33810_1_pin_port;
volatile PINMASK_TYPE mc33810_1_pin_mask;
volatile PORT_TYPE *mc33810_2_pin_port;
volatile PINMASK_TYPE mc33810_2_pin_mask;

void initMC33810(void)
{
    //Set pin port/masks
    mc33810_1_pin_port = portOutputRegister(digitalPinToPort(pinMC33810_1_CS));
    mc33810_1_pin_mask = digitalPinToBitMask(pinMC33810_1_CS);
    mc33810_2_pin_port = portOutputRegister(digitalPinToPort(pinMC33810_2_CS));
    mc33810_2_pin_mask = digitalPinToBitMask(pinMC33810_2_CS);

    //Set the output states of both ICs to be off to fuel and ignition
    mc33810_1_requestedState = 0;
    mc33810_2_requestedState = 0;
    mc33810_1_returnState = 0;
    mc33810_2_returnState = 0;

    pinMode(pinMC33810_1_CS, OUTPUT);
    pinMode(pinMC33810_2_CS, OUTPUT);

    SPI.begin();
    //These are the SPI settings per the datasheet
	SPI.beginTransaction(SPISettings(6000000, MSBFIRST, SPI_MODE0)); 

    //Set the ignition outputs to GPGD mode
    /*
    0001 = Mode select command
    1111 = Set all 1 GD[0...3] outputs to use GPGD mode
    00000000 = All remaining values are unused (For us)
    */
    //uint16_t cmd = 0b000111110000;
    uint16_t cmd = 0b0001111100000000;
    //IC1
    MC33810_1_ACTIVE();
    SPI.transfer16(cmd);
    MC33810_1_INACTIVE();
    //IC2
    MC33810_2_ACTIVE();
    SPI.transfer16(cmd);
    MC33810_2_INACTIVE();

    //Disable the Open Load pull-down current sync (See page 31 of MC33810 DS)
    /*
    0010 = LSD Fault Command
    1000 = LSD Fault operation is Shutdown (Default)
    1111 = Open load detection fault when active (Default)
    0000 = Disable open load detection when off (Changed from 1111 to 0000)
    */
    cmd = 0b0010100011110000;
    //IC1
    MC33810_1_ACTIVE();
    SPI.transfer16(cmd);
    MC33810_1_INACTIVE();
    //IC2
    MC33810_2_ACTIVE();
    SPI.transfer16(cmd);
    MC33810_2_INACTIVE();
    
}