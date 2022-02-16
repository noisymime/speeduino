#if defined(CORE_TEENSY) && defined(CORE_TEENSY35)
#include "board_teensy35.h"
#include "globals.h"
#include "auxiliaries.h"
#include "idle.h"
#include "scheduler.h"

#if defined(__MK64FX512__)         // use for Teensy 3.5 only 
  FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
#elif defined(__MK66FX1M0__)         // use for Teensy 3.6 only
  FlexCAN_T4<CAN0, RX_SIZE_256, TX_SIZE_16> Can0;
  FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1; 
#endif

void initBoard()
{
    /*
    ***********************************************************************************************************
    * General
    */

    /*
    ***********************************************************************************************************
    * Idle
    */
    if ((configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_OL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_CL))
    {
        //FlexTimer 2, compare channel 0 is used for idle
        FTM2_MODE |= FTM_MODE_WPDIS; // Write Protection Disable
        FTM2_MODE |= FTM_MODE_FTMEN; //Flex Timer module enable
        FTM2_MODE |= FTM_MODE_INIT;

        FTM2_SC = 0x00;      // Set this to zero before changing the modulus
        FTM2_CNTIN = 0x0000; //Shouldn't be needed, but just in case
        FTM2_CNT = 0x0000;   // Reset the count to zero
        FTM2_MOD = 0xFFFF;   // max modulus = 65535

        /*
        * Enable the clock for FTM0/1
        * 00 No clock selected. Disables the FTM counter.
        * 01 System clock
        * 10 Fixed frequency clock (32kHz)
        * 11 External clock
        */
        FTM2_SC |= FTM_SC_CLKS(0b10);

        /*
        * Trim the slow clock from 32kHz down to 31.25kHz (The slowest it will go)
        * This is somewhat imprecise and documentation is not good.
        * I poked the chip until I figured out the values associated with 31.25kHz
        */
        MCG_C3 = 0x9B;

        /*
        * Set Prescaler
        * This is the slowest that the timer can be clocked . It results in ticks of 32uS on the teensy 3.5:
        * 31250 Hz = Slow_clock
        * 1 * 1000000uS / Slow_clock = 32uS
        *
        * 000 = Divide by 1
        * 001 Divide by 2
        * 010 Divide by 4
        * 011 Divide by 8
        * 100 Divide by 16
        * 101 Divide by 32
        * 110 Divide by 64
        * 111 Divide by 128
        */
        FTM2_SC |= FTM_SC_PS(0b0); //No prescaler

        //Setup the channels (See Pg 1014 of K64 DS).
        FTM2_C0SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
        FTM2_C0SC |= FTM_CSC_MSA;  //Enable Compare mode
        //The below enables channel compare interrupt, but this is done in idleControl()
        //FTM2_C0SC |= FTM_CSC_CHIE;

        FTM2_C1SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
        FTM2_C1SC |= FTM_CSC_MSA;  //Enable Compare mode
        //Enable channel compare interrupt (This is currently disabled as not in use)
        //FTM2_C1SC |= FTM_CSC_CHIE;

        //Enable IRQ Interrupt
        NVIC_ENABLE_IRQ(IRQ_FTM2);
    }

    /*
    ***********************************************************************************************************
    * Auxilliaries
    */

    /*
    ***********************************************************************************************************
    * BOOST and VVT
    */
    if (configPage6.boostEnabled == 1 || configPage6.vvtEnabled == 1)
    {
        //FlexTimer 2, compare channel 0 is used for idle
        FTM1_MODE |= FTM_MODE_WPDIS; // Write Protection Disable
        FTM1_MODE |= FTM_MODE_FTMEN; //Flex Timer module enable
        FTM1_MODE |= FTM_MODE_INIT;

        FTM1_SC = 0x00;      // Set this to zero before changing the modulus
        FTM1_CNTIN = 0x0000; //Shouldn't be needed, but just in case
        FTM1_CNT = 0x0000;   // Reset the count to zero
        FTM1_MOD = 0xFFFF;   // max modulus = 65535

        /*
        * Enable the clock for FTM0/1
        * 00 No clock selected. Disables the FTM counter.
        * 01 System clock
        * 10 Fixed frequency clock (32kHz)
        * 11 External clock
        */
        FTM1_SC |= FTM_SC_CLKS(0b10);

        /*
        * Trim the slow clock from 32kHz down to 31.25kHz (The slowest it will go)
        * This is somewhat imprecise and documentation is not good.
        * I poked the chip until I figured out the values associated with 31.25kHz
        */
        MCG_C3 = 0x9B;

        /*
        * Set Prescaler
        * This is the slowest that the timer can be clocked . It results in ticks of 32uS on the teensy 3.5:
        * 31250 Hz = Slow_clock
        * 1 * 1000000uS / Slow_clock = 32uS
        *
        * 000 = Divide by 1
        * 001 Divide by 2
        * 010 Divide by 4
        * 011 Divide by 8
        * 100 Divide by 16
        * 101 Divide by 32
        * 110 Divide by 64
        * 111 Divide by 128
        */
        FTM1_SC |= FTM_SC_PS(0b0); //No prescaler

        //Setup the channels (See Pg 1014 of K64 DS).
        FTM1_C0SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
        FTM1_C0SC |= FTM_CSC_MSA;  //Enable Compare mode
        //The below enables channel compare interrupt, but this is done in idleControl()
        //FTM1_C0SC |= FTM_CSC_CHIE;

        FTM1_C1SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
        FTM1_C1SC |= FTM_CSC_MSA;  //Enable Compare mode
        //Enable channel compare interrupt (This is currently disabled as not in use)
        //FTM1_C1SC |= FTM_CSC_CHIE;

        FTM2_C1SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
        FTM2_C1SC |= FTM_CSC_MSA;  //Enable Compare mode
        //Enable channel compare interrupt (This is currently disabled as not in use)
        //FTM1_C2SC |= FTM_CSC_CHIE;

        //Enable IRQ Interrupt
        NVIC_ENABLE_IRQ(IRQ_FTM1);

        boost_pwm_max_count = 1000000L / (32 * configPage6.boostFreq * 2); //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
        vvt_pwm_max_count = 1000000L / (32 * configPage6.vvtFreq * 2);     //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz
        fan_pwm_max_count = 1000000L / (32 * configPage6.fanFreq * 2);     //Converts the frequency in Hz to the number of ticks (at 16uS) it takes to complete 1 cycle. Note that the frequency is divided by 2 coming from TS to allow for up to 512hz


    }

    /*
    ***********************************************************************************************************
    * Timers
    */
    //Uses the PIT timer on Teensy.
    lowResTimer.begin(oneMSInterval, 1000);

    /*
    ***********************************************************************************************************
    * Schedules
    */

    //FlexTimer 0 is used for 4 ignition and 4 injection schedules. There are 8 channels on this module, so no other timers are needed
    FTM0_MODE |= FTM_MODE_WPDIS; //Write Protection Disable
    FTM0_MODE |= FTM_MODE_FTMEN; //Flex Timer module enable
    FTM0_MODE |= FTM_MODE_INIT;

    FTM0_SC = 0x00;      // Set this to zero before changing the modulus
    FTM0_CNTIN = 0x0000; //Shouldn't be needed, but just in case
    FTM0_CNT = 0x0000;   //Reset the count to zero
    FTM0_MOD = 0xFFFF;   //max modulus = 65535

    //FlexTimer 3 is used for schedules on channel 5+. Currently only channel 5 is used, but will likely be expanded later
    FTM3_MODE |= FTM_MODE_WPDIS; //Write Protection Disable
    FTM3_MODE |= FTM_MODE_FTMEN; //Flex Timer module enable
    FTM3_MODE |= FTM_MODE_INIT;

    FTM3_SC = 0x00;      // Set this to zero before changing the modulus
    FTM3_CNTIN = 0x0000; //Shouldn't be needed, but just in case
    FTM3_CNT = 0x0000;   //Reset the count to zero
    FTM3_MOD = 0xFFFF;   //max modulus = 65535

    /*
    * Enable the clock for FTM0/1
    * 00 No clock selected. Disables the FTM counter.
    * 01 System clock
    * 10 Fixed frequency clock
    * 11 External clock
    */
    FTM0_SC |= FTM_SC_CLKS(0b1);
    FTM3_SC |= FTM_SC_CLKS(0b1);

    /*
    * Set Prescaler
    * This is the slowest that the timer can be clocked (Without used the slow timer, which is too slow). It results in ticks of 2.13333uS on the teensy 3.5:
    * 60000000 Hz = F_BUS
    * 128 * 1000000uS / F_BUS = 2.133uS
    *
    * 000 = Divide by 1
    * 001 Divide by 2
    * 010 Divide by 4
    * 011 Divide by 8
    * 100 Divide by 16
    * 101 Divide by 32
    * 110 Divide by 64
    * 111 Divide by 128
    */
    FTM0_SC |= FTM_SC_PS(0b111);
    FTM3_SC |= FTM_SC_PS(0b111);

    //Setup the channels (See Pg 1014 of K64 DS).
    //The are probably not needed as power on state should be 0
    //FTM0_C0SC &= ~FTM_CSC_ELSB;
    //FTM0_C0SC &= ~FTM_CSC_ELSA;
    //FTM0_C0SC &= ~FTM_CSC_DMA;
    FTM0_C0SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C0SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C0SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM0_C1SC &= ~FTM_CSC_MSB; //According to Pg 965 of the datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C1SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C1SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM0_C2SC &= ~FTM_CSC_MSB; //According to Pg 965 of the datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C2SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C2SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM0_C3SC &= ~FTM_CSC_MSB; //According to Pg 965 of the datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C3SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C3SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM0_C4SC &= ~FTM_CSC_MSB; //According to Pg 965 of the datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C4SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C4SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM0_C5SC &= ~FTM_CSC_MSB; //According to Pg 965 of the datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C5SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C5SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM0_C6SC &= ~FTM_CSC_MSB; //According to Pg 965 of the datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C6SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C6SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM0_C7SC &= ~FTM_CSC_MSB; //According to Pg 965 of the datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM0_C7SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM0_C7SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    //Do the same, but on flex timer 3 (Used for channels 5-8)
    FTM3_C0SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C0SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C0SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM3_C1SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C1SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C1SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM3_C2SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C2SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C2SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM3_C3SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C3SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C3SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM3_C4SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C4SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C4SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM3_C5SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C5SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C5SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM3_C6SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C6SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C6SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    FTM3_C7SC &= ~FTM_CSC_MSB; //According to Pg 965 of the K64 datasheet, this should not be needed as MSB is reset to 0 upon reset, but the channel interrupt fails to fire without it
    FTM3_C7SC |= FTM_CSC_MSA;  //Enable Compare mode
    FTM3_C7SC |= FTM_CSC_CHIE; //Enable channel compare interrupt

    // enable IRQ Interrupt
    NVIC_ENABLE_IRQ(IRQ_FTM0);
    NVIC_ENABLE_IRQ(IRQ_FTM3);
}

void ftm0_isr(void)
{
  //Use separate variables for each test to ensure conversion to bool
  bool interrupt1 = (FTM0_C0SC & FTM_CSC_CHF);
  bool interrupt2 = (FTM0_C1SC & FTM_CSC_CHF);
  bool interrupt3 = (FTM0_C2SC & FTM_CSC_CHF);
  bool interrupt4 = (FTM0_C3SC & FTM_CSC_CHF);
  bool interrupt5 = (FTM0_C4SC & FTM_CSC_CHF);
  bool interrupt6 = (FTM0_C5SC & FTM_CSC_CHF);
  bool interrupt7 = (FTM0_C6SC & FTM_CSC_CHF);
  bool interrupt8 = (FTM0_C7SC & FTM_CSC_CHF);

  if(interrupt1) { FTM0_C0SC &= ~FTM_CSC_CHF; fuelSchedule1Interrupt(); }
  else if(interrupt2) { FTM0_C1SC &= ~FTM_CSC_CHF; fuelSchedule2Interrupt(); }
  else if(interrupt3) { FTM0_C2SC &= ~FTM_CSC_CHF; fuelSchedule3Interrupt(); }
  else if(interrupt4) { FTM0_C3SC &= ~FTM_CSC_CHF; fuelSchedule4Interrupt(); }
  else if(interrupt5) { FTM0_C4SC &= ~FTM_CSC_CHF; ignitionSchedule1Interrupt(); }
  else if(interrupt6) { FTM0_C5SC &= ~FTM_CSC_CHF; ignitionSchedule2Interrupt(); }
  else if(interrupt7) { FTM0_C6SC &= ~FTM_CSC_CHF; ignitionSchedule3Interrupt(); }
  else if(interrupt8) { FTM0_C7SC &= ~FTM_CSC_CHF; ignitionSchedule4Interrupt(); }

}
void ftm3_isr(void)
{

#if (INJ_CHANNELS >= 5)
  bool interrupt1 = (FTM3_C0SC & FTM_CSC_CHF);
  if(interrupt1) { FTM3_C0SC &= ~FTM_CSC_CHF; fuelSchedule5Interrupt(); }
#endif
#if (INJ_CHANNELS >= 6)
  bool interrupt2 = (FTM3_C1SC & FTM_CSC_CHF);
  if(interrupt2) { FTM3_C1SC &= ~FTM_CSC_CHF; fuelSchedule6Interrupt(); }
#endif
#if (INJ_CHANNELS >= 7)
  bool interrupt3 = (FTM3_C2SC & FTM_CSC_CHF);
  if(interrupt3) { FTM3_C2SC &= ~FTM_CSC_CHF; fuelSchedule7Interrupt(); }
#endif
#if (INJ_CHANNELS >= 8)
  bool interrupt4 = (FTM3_C3SC & FTM_CSC_CHF);
  if(interrupt4) { FTM3_C3SC &= ~FTM_CSC_CHF; fuelSchedule8Interrupt(); }
#endif
#if (IGN_CHANNELS >= 5)
  bool interrupt5 = (FTM3_C4SC & FTM_CSC_CHF);
  if(interrupt5) { FTM3_C4SC &= ~FTM_CSC_CHF; ignitionSchedule5Interrupt(); }
#endif
#if (IGN_CHANNELS >= 6)
  bool interrupt6 = (FTM3_C5SC & FTM_CSC_CHF);
  if(interrupt6) { FTM3_C5SC &= ~FTM_CSC_CHF; ignitionSchedule6Interrupt(); }
#endif
#if (IGN_CHANNELS >= 7)
  bool interrupt7 = (FTM3_C6SC & FTM_CSC_CHF);
  if(interrupt7) { FTM3_C6SC &= ~FTM_CSC_CHF; ignitionSchedule7Interrupt(); }
#endif
#if (IGN_CHANNELS >= 8)
  bool interrupt8 = (FTM3_C7SC & FTM_CSC_CHF);
  if(interrupt8) { FTM3_C7SC &= ~FTM_CSC_CHF; ignitionSchedule8Interrupt(); }
#endif

}

//Boost and VVT handler
void ftm1_isr(void)
{
  //FTM1 only has 2 compare channels (Is this correct?)
  //Use separate variables for each test to ensure conversion to bool
  bool interrupt1 = (FTM1_C0SC & FTM_CSC_CHF);
  bool interrupt2 = (FTM1_C1SC & FTM_CSC_CHF);

  if(interrupt1) { FTM1_C0SC &= ~FTM_CSC_CHF; boostInterrupt(); }
  else if(interrupt2) { FTM1_C1SC &= ~FTM_CSC_CHF; vvtInterrupt(); }

}

//Idle and spare handler
void ftm2_isr(void)
{ 
  //FTM2 only has 2 compare channels
  //Use separate variables for each test to ensure conversion to bool
  bool interrupt1 = (FTM2_C0SC & FTM_CSC_CHF);
  bool interrupt2 = (FTM2_C1SC & FTM_CSC_CHF); //For PWM Fan

  if(interrupt1) { FTM2_C0SC &= ~FTM_CSC_CHF; idleInterrupt(); }
  else if(interrupt2) { FTM2_C1SC &= ~FTM_CSC_CHF; fanInterrupt(); } //For PWM Fan
}

uint16_t freeRam()
{
    uint32_t freeRam;
    uint32_t stackTop;
    uint32_t heapTop;

    // current position of the stack.
    stackTop = (uint32_t)&stackTop;

    // current position of heap.
    void *hTop = malloc(1);
    heapTop = (uint32_t)hTop;
    free(hTop);
    freeRam = stackTop - heapTop;

    if(freeRam>0xFFFF){return 0xFFFF;}
    else{return freeRam;}
}

//This function is used for attempting to set the RTC time during compile
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void doSystemReset() { return; }
void jumpToBootloader() { return; }

#endif
