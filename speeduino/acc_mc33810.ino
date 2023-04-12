#include "acc_mc33810.h"
#include "globals.h"
#include <SPI.h>

void initMC33810(void)
{
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
    
}