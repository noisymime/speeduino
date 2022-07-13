#if defined(CORE_TEENSY) && defined(__IMXRT1062__)
#include "board_teensy41.h"
#include "globals.h"
#include "auxiliaries.h"
#include "idle.h"
#include "scheduler.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can2;

void initBoard()
{
    /*
    ***********************************************************************************************************
    * General
    */

    /*
    Idle + Boost + VVT use the PIT timer. THIS IS ALSO USED BY THE INTERVAL TIMER THAT CALLS THE 1MS LOW RES TIMER!
    This has 4 channels that don't have compare registers, but will run for a period of time and then fire an interrupt
    The clock for these is set to 24Mhz and a prescale of 48 is used to give a 2uS tick time
    Set Prescaler
      * This is ideally too fast, but appears to be the slowest that the PIT can be set
      * 24Mhz = PER_clock
      * 48 * 1000000uS / PER_clock = 2uS
    */
    CCM_CSCMR1 |= CCM_CSCMR1_PERCLK_CLK_SEL; // 24MHz
    CCM_CSCMR1 |= CCM_CSCMR1_PERCLK_PODF(0b110000); //Prescale to 48

    attachInterruptVector(IRQ_PIT, PIT_isr);
    NVIC_ENABLE_IRQ(IRQ_PIT);

    /*
    ***********************************************************************************************************
    * Idle
    */
    if( (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_OL) || (configPage6.iacAlgorithm == IAC_ALGORITHM_PWM_CL) )
    {
      PIT_TCTRL0 = 0;
      PIT_TCTRL0 |= PIT_TCTRL_TIE; // enable Timer 1 interrupts
      PIT_TCTRL0 |= PIT_TCTRL_TEN; // start Timer 1
    }

    /*
    ***********************************************************************************************************
    * Timers
    */
    //Uses the PIT timer channel 4 on Teensy 4.1.
    //lowResTimer.begin(oneMSInterval, 1000);
    PIT_TCTRL3 = 0;
    PIT_TCTRL3 |= PIT_TCTRL_TIE; // enable Timer 2 interrupts
    PIT_TCTRL3 |= PIT_TCTRL_TEN; // start Timer 2
    PIT_LDVAL3 = 500; //500 * 2uS = 1ms

    //TODO: Configure timers here

    /*
    ***********************************************************************************************************
    * Auxiliaries
    */

    //2uS resolution Min 8Hz, Max 5KHz
    boost_pwm_max_count = 1000000L / (2 * configPage6.boostFreq * 2); //Converts the frequency in Hz to the number of ticks (at 2uS) it takes to complete 1 cycle. The x2 is there because the frequency is stored at half value (in a byte) to allow frequencies up to 511Hz
    vvt_pwm_max_count = 1000000L / (2 * configPage6.vvtFreq * 2); //Converts the frequency in Hz to the number of ticks (at 2uS) it takes to complete 1 cycle
    fan_pwm_max_count = 1000000L / (2 * configPage6.vvtFreq * 2); //Converts the frequency in Hz to the number of ticks (at 2uS) it takes to complete 1 cycle

    //TODO: Configure timers here

    /*
    ***********************************************************************************************************
    * Schedules
    */
    //Use the Quad timer, 60Mhz IP_bus_clk divided by 128 for a 2.133333uS period
    //TMR1 - Fuel 1-4
    //0
    TMR1_CTRL0 = 0;
    TMR1_CSCTRL0 = 0;
    TMR1_LOAD0 = 0; /* Reset load register */
    TMR1_CTRL0 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR1_CTRL0 |= TMR_CTRL_CM(1); //Start the timer
    //1
    TMR1_CTRL1 = 0;
    TMR1_CSCTRL1 = 0;
    TMR1_LOAD1 = 0; /* Reset load register */
    TMR1_CTRL1 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR1_CTRL1 |= TMR_CTRL_CM(1); //Start the timer
    //2
    TMR1_CTRL2 = 0;
    TMR1_CSCTRL2 = 0;
    TMR1_LOAD2 = 0; /* Reset load register */
    TMR1_CTRL2 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR1_CTRL2 |= TMR_CTRL_CM(1); //Start the timer
    //3
    TMR1_CTRL3 = 0;
    TMR1_CSCTRL3 = 0;
    TMR1_LOAD3 = 0; /* Reset load register */
    TMR1_CTRL3 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR1_CTRL3 |= TMR_CTRL_CM(1); //Start the timer
    //TMR2 - Ign 1-4
    //0
    TMR2_CTRL0 = 0;
    TMR2_CSCTRL0 = 0;
    TMR2_LOAD0 = 0; /* Reset load register */
    TMR2_CTRL0 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR2_CTRL0 |= TMR_CTRL_CM(1); //Start the timer
    //1
    TMR2_CTRL1 = 0;
    TMR2_CSCTRL1 = 0;
    TMR2_LOAD1 = 0; /* Reset load register */
    TMR2_CTRL1 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR2_CTRL1 |= TMR_CTRL_CM(1); //Start the timer
    //2
    TMR2_CTRL2 = 0;
    TMR2_CSCTRL2 = 0;
    TMR2_LOAD2 = 0; /* Reset load register */
    TMR2_CTRL2 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR2_CTRL2 |= TMR_CTRL_CM(1); //Start the timer
    //3
    TMR2_CTRL3 = 0;
    TMR2_CSCTRL3 = 0;
    TMR2_LOAD3 = 0; /* Reset load register */
    TMR2_CTRL3 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR2_CTRL3 |= TMR_CTRL_CM(1); //Start the timer
    //TMR3 - Fuel 5-8
    //0
    TMR3_CTRL0 = 0;
    TMR3_CSCTRL0 = 0;
    TMR3_LOAD0 = 0; /* Reset load register */
    TMR3_CTRL0 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR3_CTRL0 |= TMR_CTRL_CM(1); //Start the timer
    //1
    TMR3_CTRL1 = 0;
    TMR3_CSCTRL1 = 0;
    TMR3_LOAD1 = 0; /* Reset load register */
    TMR3_CTRL1 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR3_CTRL1 |= TMR_CTRL_CM(1); //Start the timer
    //2
    TMR3_CTRL2 = 0;
    TMR3_CSCTRL2 = 0;
    TMR3_LOAD2 = 0; /* Reset load register */
    TMR3_CTRL2 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR3_CTRL2 |= TMR_CTRL_CM(1); //Start the timer
    //3
    TMR3_CTRL3 = 0;
    TMR3_CSCTRL3 = 0;
    TMR3_LOAD3 = 0; /* Reset load register */
    TMR3_CTRL3 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR3_CTRL3 |= TMR_CTRL_CM(1); //Start the timer
    //TMR4 - IGN 5-8
    //0
    TMR4_CTRL0 = 0;
    TMR4_CSCTRL0 = 0;
    TMR4_LOAD0 = 0; /* Reset load register */
    TMR4_CTRL0 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR4_CTRL0 |= TMR_CTRL_CM(1); //Start the timer
    //1
    TMR4_CTRL1 = 0;
    TMR4_CSCTRL1 = 0;
    TMR4_LOAD1 = 0; /* Reset load register */
    TMR4_CTRL1 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR4_CTRL1 |= TMR_CTRL_CM(1); //Start the timer
    //2
    TMR4_CTRL2 = 0;
    TMR4_CSCTRL2 = 0;
    TMR4_LOAD2 = 0; /* Reset load register */
    TMR4_CTRL2 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR4_CTRL2 |= TMR_CTRL_CM(1); //Start the timer
    //3
    TMR4_CTRL3 = 0;
    TMR4_CSCTRL3 = 0;
    TMR4_LOAD3 = 0; /* Reset load register */
    TMR4_CTRL3 |= TMR_CTRL_PCS(0b1111); //Set the prescaler to 128
    TMR4_CTRL3 |= TMR_CTRL_CM(1); //Start the timer

    attachInterruptVector(IRQ_QTIMER1, TMR1_isr);
    NVIC_ENABLE_IRQ(IRQ_QTIMER1);
    attachInterruptVector(IRQ_QTIMER2, TMR2_isr);
    NVIC_ENABLE_IRQ(IRQ_QTIMER2);
    attachInterruptVector(IRQ_QTIMER3, TMR3_isr);
    NVIC_ENABLE_IRQ(IRQ_QTIMER3);
    attachInterruptVector(IRQ_QTIMER4, TMR4_isr);
    NVIC_ENABLE_IRQ(IRQ_QTIMER4);
}

void PIT_isr()
{
  bool interrupt1 = (PIT_TFLG0 & PIT_TFLG_TIF);
  bool interrupt2 = (PIT_TFLG1 & PIT_TFLG_TIF);
  bool interrupt3 = (PIT_TFLG2 & PIT_TFLG_TIF);
  bool interrupt4 = (PIT_TFLG3 & PIT_TFLG_TIF);

  if(interrupt1)      { PIT_TFLG0 = 1;}
  else if(interrupt2) { PIT_TFLG1 = 1;}
  else if(interrupt3) { PIT_TFLG2 = 1;}
  else if(interrupt4) { oneMSInterval(); PIT_TFLG3 = 1;}
}

void TMR1_isr(void)
{
  //TMR1 is fuel channels 1-4
  bool interrupt1 = (TMR1_CSCTRL0 & TMR_CSCTRL_TCF1);
  bool interrupt2 = (TMR1_CSCTRL1 & TMR_CSCTRL_TCF1);
  bool interrupt3 = (TMR1_CSCTRL2 & TMR_CSCTRL_TCF1);
  bool interrupt4 = (TMR1_CSCTRL3 & TMR_CSCTRL_TCF1);

  if(interrupt1)      { TMR1_CSCTRL0 &= ~TMR_CSCTRL_TCF1; fuelSchedule1Interrupt(); }
  else if(interrupt2) { TMR1_CSCTRL1 &= ~TMR_CSCTRL_TCF1; fuelSchedule2Interrupt(); }
  else if(interrupt3) { TMR1_CSCTRL2 &= ~TMR_CSCTRL_TCF1; fuelSchedule3Interrupt(); }
  else if(interrupt4) { TMR1_CSCTRL3 &= ~TMR_CSCTRL_TCF1; fuelSchedule4Interrupt(); }
}
void TMR2_isr(void)
{
  //TMR2 is IGN channels 1-4
  bool interrupt1 = (TMR2_CSCTRL0 & TMR_CSCTRL_TCF1);
  bool interrupt2 = (TMR2_CSCTRL1 & TMR_CSCTRL_TCF1);
  bool interrupt3 = (TMR2_CSCTRL2 & TMR_CSCTRL_TCF1);
  bool interrupt4 = (TMR2_CSCTRL3 & TMR_CSCTRL_TCF1);

  if(interrupt1)      { TMR2_CSCTRL0 &= ~TMR_CSCTRL_TCF1; ignitionSchedule1Interrupt(); }
  else if(interrupt2) { TMR2_CSCTRL1 &= ~TMR_CSCTRL_TCF1; ignitionSchedule2Interrupt(); }
  else if(interrupt3) { TMR2_CSCTRL2 &= ~TMR_CSCTRL_TCF1; ignitionSchedule3Interrupt(); }
  else if(interrupt4) { TMR2_CSCTRL3 &= ~TMR_CSCTRL_TCF1; ignitionSchedule4Interrupt(); }
}
void TMR3_isr(void)
{
  //TMR3 is Fuel  channels 5-8
  bool interrupt1 = (TMR3_CSCTRL0 & TMR_CSCTRL_TCF1);
  bool interrupt2 = (TMR3_CSCTRL1 & TMR_CSCTRL_TCF1);
  bool interrupt3 = (TMR3_CSCTRL2 & TMR_CSCTRL_TCF1);
  bool interrupt4 = (TMR3_CSCTRL3 & TMR_CSCTRL_TCF1);

  if(interrupt1)      { TMR3_CSCTRL0 &= ~TMR_CSCTRL_TCF1; fuelSchedule5Interrupt(); }
  else if(interrupt2) { TMR3_CSCTRL1 &= ~TMR_CSCTRL_TCF1; fuelSchedule6Interrupt(); }
  else if(interrupt3) { TMR3_CSCTRL2 &= ~TMR_CSCTRL_TCF1; fuelSchedule7Interrupt(); }
  else if(interrupt4) { TMR3_CSCTRL3 &= ~TMR_CSCTRL_TCF1; fuelSchedule8Interrupt(); }
}
void TMR4_isr(void)
{
  //TMR4 is IGN channels 5-8
  bool interrupt1 = (TMR4_CSCTRL0 & TMR_CSCTRL_TCF1);
  bool interrupt2 = (TMR4_CSCTRL1 & TMR_CSCTRL_TCF1);
  bool interrupt3 = (TMR4_CSCTRL2 & TMR_CSCTRL_TCF1);
  bool interrupt4 = (TMR4_CSCTRL3 & TMR_CSCTRL_TCF1);

  if(interrupt1)      { TMR4_CSCTRL0 &= ~TMR_CSCTRL_TCF1; ignitionSchedule5Interrupt(); }
  else if(interrupt2) { TMR4_CSCTRL1 &= ~TMR_CSCTRL_TCF1; ignitionSchedule6Interrupt(); }
  else if(interrupt3) { TMR4_CSCTRL2 &= ~TMR_CSCTRL_TCF1; ignitionSchedule7Interrupt(); }
  else if(interrupt4) { TMR4_CSCTRL3 &= ~TMR_CSCTRL_TCF1; ignitionSchedule8Interrupt(); }
}

uint16_t freeRam()
{
    uint32_t stackTop;
    uint32_t heapTop;

    // current position of the stack.
    stackTop = (uint32_t) &stackTop;

    // current position of heap.
    void* hTop = malloc(1);
    heapTop = (uint32_t) hTop;
    free(hTop);

    // The difference is the free, available ram.
    return (uint16_t)stackTop - heapTop;
}

//This function is used for attempting to set the RTC time during compile
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void doSystemReset() { return; }
void jumpToBootloader() { return; }

#endif
