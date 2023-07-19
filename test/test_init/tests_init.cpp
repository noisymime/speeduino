#include <unity.h>
#include "globals.h"
#include "init.h"
#include "../test_utils.h"

#define UNKNOWN_PIN 0xFF

#if !defined(NOT_A_PIN)
#define NOT_A_PIN 0
#endif

uint8_t getPinMode(uint8_t pin)
{
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);

  // I don't see an option for mega to return this, but whatever...
  if (NOT_A_PIN == port) return UNKNOWN_PIN;

  // Is there a bit we can check?
  if (0 == bit) return UNKNOWN_PIN;

  // Is there only a single bit set?
  if (bit & (bit - 1)) return UNKNOWN_PIN;

  volatile uint8_t *reg, *out;
  reg = portModeRegister(port);
  out = portOutputRegister(port);

  if (*reg & bit)
    return OUTPUT;
  else if (*out & bit)
    return INPUT_PULLUP;
  else
    return INPUT;
}

void test_initialisation_complete(void)
{
  initialiseAll(); //Run the main initialise function
  TEST_ASSERT_EQUAL(true, initialisationComplete);
}

void test_initialisation_ports(void)
{
  //Test that all the port values have been set
  initialiseAll(); //Run the main initialise function
  TEST_ASSERT_NOT_EQUAL(0, inj1_pin_port);
  TEST_ASSERT_NOT_EQUAL(0, inj2_pin_port);
  TEST_ASSERT_NOT_EQUAL(0, inj3_pin_port);
  TEST_ASSERT_NOT_EQUAL(0, inj4_pin_port);
  TEST_ASSERT_NOT_EQUAL(0, ign1_pin_port);
  TEST_ASSERT_NOT_EQUAL(0, ign2_pin_port);
  TEST_ASSERT_NOT_EQUAL(0, ign3_pin_port);
  TEST_ASSERT_NOT_EQUAL(0, ign4_pin_port);
}

//Test that all mandatory output pins have their mode correctly set to output
void test_initialisation_outputs_V03(void)
{
  configPage2.pinMapping = 2; //Set the board number to test
  initialiseAll(); //Run the main initialise function

  char msg[32];
  strcpy_P(msg, PSTR("Coil1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil1), msg);
  strcpy_P(msg, PSTR("Coil2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil2), msg);
  strcpy_P(msg, PSTR("Coil3"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil3), msg);
  strcpy_P(msg, PSTR("Coil4"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil4), msg);
  strcpy_P(msg, PSTR("Injector 1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector1), msg);
  strcpy_P(msg, PSTR("Injector 2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector2), msg);
  strcpy_P(msg, PSTR("Injector 3"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector3), msg);
  strcpy_P(msg, PSTR("Injector 4"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector4), msg);
  strcpy_P(msg, PSTR("Tacho Out"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinTachOut), msg);
  strcpy_P(msg, PSTR("Fuel Pump"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinFuelPump), msg);
  strcpy_P(msg, PSTR("Fan"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinFan), msg);
}

//Test that all mandatory output pins have their mode correctly set to output
void test_initialisation_outputs_V04(void)
{
  configPage2.pinMapping = 3; //Set the board number to test

  initialiseAll(); //Run the main initialise function

  char msg[32];
  strcpy_P(msg, PSTR("Coil1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil1), msg);
  strcpy_P(msg, PSTR("Coil2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil2), msg);
  strcpy_P(msg, PSTR("Coil3"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil3), msg);
  strcpy_P(msg, PSTR("Coil4"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil4), msg);
  strcpy_P(msg, PSTR("Injector 1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector1), msg);
  strcpy_P(msg, PSTR("Injector 2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector2), msg);
  strcpy_P(msg, PSTR("Injector 3"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector3), msg);
  strcpy_P(msg, PSTR("Injector 4"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector4), msg);
  strcpy_P(msg, PSTR("Tacho Out"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinTachOut), msg);
  strcpy_P(msg, PSTR("Fuel Pump"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinFuelPump), msg);
  strcpy_P(msg, PSTR("Fan"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinFan), msg);
  /*
  if(isIdlePWM) 
  {
    TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinIdle1), "Idle 1");
    TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinIdle2), "Idle 2");
  }
  else if (isIdleStepper)
  {
    TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinStepperDir), "Stepper Dir");
    TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinStepperStep), "Stepper Step");
    TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinStepperEnable), "Stepper Enable");
  }
  
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinFan), "Fan");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinBoost), "Boost");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinVVT_1), "VVT1");
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinVVT_2), "VVT2");
  */

}

//Test that all mandatory output pins have their mode correctly set to output
void test_initialisation_outputs_MX5_8995(void)
{
  configPage2.pinMapping = 9; //Set the board number to test
  initialiseAll(); //Run the main initialise function

  char msg[32];
  strcpy_P(msg, PSTR("Coil1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil1), msg);
  strcpy_P(msg, PSTR("Coil2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil2), msg);
  strcpy_P(msg, PSTR("Coil3"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil3), msg);
  strcpy_P(msg, PSTR("Coil4"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinCoil4), msg);
  strcpy_P(msg, PSTR("Injector 1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector1), msg);
  strcpy_P(msg, PSTR("Injector 2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector2), msg);
  strcpy_P(msg, PSTR("Injector 3"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector3), msg);
  strcpy_P(msg, PSTR("Injector 4"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinInjector4), msg);
  strcpy_P(msg, PSTR("Tacho Out"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinTachOut), msg);
  strcpy_P(msg, PSTR("Fuel Pump"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinFuelPump), msg);
  strcpy_P(msg, PSTR("Fan"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinFan), msg);
}

void test_initialisation_outputs_PWM_idle(void)
{
  configPage2.pinMapping = 3; //Set the board number to test (v0.4)
  //Force 2 channel PWM idle
  configPage6.iacChannels = 1;
  configPage6.iacAlgorithm = 2;

  initialiseAll(); //Run the main initialise function

  bool isIdlePWM = (configPage6.iacAlgorithm > 0) && ((configPage6.iacAlgorithm <= 3) || (configPage6.iacAlgorithm == 6));

  char msg[32];
  strcpy_P(msg, PSTR("Is PWM Idle"));
  TEST_ASSERT_TRUE_MESSAGE(isIdlePWM, msg);
  strcpy_P(msg, PSTR("Idle 1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinIdle1), msg);
  strcpy_P(msg, PSTR("Idle 2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinIdle2), msg);
}

void test_initialisation_outputs_stepper_idle(void)
{
  bool isIdleStepper = (configPage6.iacAlgorithm > 3) && (configPage6.iacAlgorithm != 6);

  char msg[32];
  strcpy_P(msg, PSTR("Is Stepper Idle"));
  TEST_ASSERT_TRUE_MESSAGE(isIdleStepper, msg);
  strcpy_P(msg, PSTR("Stepper Dir"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinStepperDir), msg);
  strcpy_P(msg, PSTR("Stepper Step"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinStepperStep), msg);
  strcpy_P(msg, PSTR("Stepper Enable"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinStepperEnable), msg);
}

void test_initialisation_outputs_boost(void)
{
  char msg[32];
  strcpy_P(msg, PSTR("Boost"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinBoost), msg);
}

void test_initialisation_outputs_VVT(void)
{
  char msg[32];
  strcpy_P(msg, PSTR("VVT1"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinVVT_1), msg);
  strcpy_P(msg, PSTR("VVT2"));
  TEST_ASSERT_EQUAL_MESSAGE(OUTPUT, getPinMode(pinVVT_2), msg);
}

void testInitialisation()
{
  RUN_TEST_P(test_initialisation_complete);
  RUN_TEST_P(test_initialisation_ports);
  RUN_TEST_P(test_initialisation_outputs_V03);
  RUN_TEST_P(test_initialisation_outputs_V04);
  RUN_TEST_P(test_initialisation_outputs_MX5_8995);
  RUN_TEST_P(test_initialisation_outputs_PWM_idle);
  RUN_TEST_P(test_initialisation_outputs_boost);
  RUN_TEST_P(test_initialisation_outputs_VVT);
}