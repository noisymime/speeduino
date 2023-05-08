#include <Arduino.h>
#include <unity.h>
#include "globals.h"
#include "init.h"
#include "schedule_calcs.h"
#include "scheduledIO.h"
#include "utilities.h"
#include "..\test_utils.h"

extern uint16_t req_fuel_uS;

static constexpr uint16_t reqFuel = 86; // ms * 10

static void __attribute__((noinline)) assert_fuel_channel(bool enabled, uint16_t angle, uint8_t cmdBit, int channelInjDegrees, voidVoidCallback startFunction, voidVoidCallback endFunction)
{
  char msg[32];

  sprintf_P(msg, PSTR("channel%" PRIu8 "InjDegrees.isEnabled"), cmdBit+1);
  TEST_ASSERT_EQUAL_MESSAGE(enabled, BIT_CHECK(channelInjEnabled, cmdBit), msg);
  sprintf_P(msg, PSTR("channe%" PRIu8 "InjDegrees"), cmdBit+1);
  TEST_ASSERT_EQUAL_MESSAGE(angle, channelInjDegrees, msg);
  sprintf_P(msg, PSTR("inj%" PRIu8 "StartFunction"), cmdBit+1);
  TEST_ASSERT_TRUE_MESSAGE(!enabled || (startFunction!=nullCallback), msg);
  sprintf_P(msg, PSTR("inj%" PRIu8 "EndFunction"), cmdBit+1);
  TEST_ASSERT_TRUE_MESSAGE(!enabled || (endFunction!=nullCallback), msg);
}

static void __attribute__((noinline)) assert_fuel_schedules(uint16_t crankAngle, uint16_t reqFuel, const bool enabled[], const uint16_t angle[])
{
  char msg[32];

  strcpy_P(msg, PSTR("CRANK_ANGLE_MAX_INJ"));
  TEST_ASSERT_EQUAL_INT16_MESSAGE(crankAngle, CRANK_ANGLE_MAX_INJ, msg);
  strcpy_P(msg, PSTR("req_fuel_uS"));
  TEST_ASSERT_EQUAL_UINT16_MESSAGE(reqFuel, req_fuel_uS, msg);

  assert_fuel_channel(enabled[0], angle[0], INJ1_CMD_BIT, channel1InjDegrees, inj1StartFunction, inj1EndFunction);
  assert_fuel_channel(enabled[1], angle[1], INJ2_CMD_BIT, channel2InjDegrees, inj2StartFunction, inj2EndFunction);
  assert_fuel_channel(enabled[2], angle[2], INJ3_CMD_BIT, channel3InjDegrees, inj3StartFunction, inj3EndFunction);
  assert_fuel_channel(enabled[3], angle[3], INJ4_CMD_BIT, channel4InjDegrees, inj4StartFunction, inj4EndFunction);

#if INJ_CHANNELS>=5
  assert_fuel_channel(enabled[4], angle[4], INJ5_CMD_BIT, channel5InjDegrees, inj5StartFunction, inj5EndFunction);
#endif

#if INJ_CHANNELS>=6
  assert_fuel_channel(enabled[5], angle[5], INJ6_CMD_BIT, channel6InjDegrees, inj6StartFunction, inj6EndFunction);
#endif

#if INJ_CHANNELS>=7
  assert_fuel_channel(enabled[6], angle[6], INJ7_CMD_BIT, channel7InjDegrees, inj7StartFunction, inj7EndFunction);
#endif

#if INJ_CHANNELS>=8
  assert_fuel_channel(enabled[7], angle[7], INJ8_CMD_BIT, channel8InjDegrees, inj8StartFunction, inj8EndFunction);
#endif 
}

static void cylinder1_stroke4_seq_no_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, false, false, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
}

static void cylinder1_stroke4_semiseq_no_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, false, false, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
}

static void cylinder1_stroke4_seq_with_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
  }

static void cylinder1_stroke4_semiseq_with_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
}

static void run_1_cylinder_4stroke_tests(void)
{
  configPage2.nCylinders = 1;
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel; 
  configPage2.divider = 1;

  RUN_TEST_P(cylinder1_stroke4_seq_no_stage);
  RUN_TEST_P(cylinder1_stroke4_semiseq_no_stage);
  RUN_TEST_P(cylinder1_stroke4_seq_with_stage);
  RUN_TEST_P(cylinder1_stroke4_semiseq_with_stage);
}

static void cylinder1_stroke2_seq_no_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
  const bool enabled[] = {true, false, false, false, false, false, false, false};
  const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 100U, enabled, angle);  
  }

static void cylinder1_stroke2_semiseq_no_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, false, false, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 100U, enabled, angle);
  }

static void cylinder1_stroke2_seq_with_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 100U, enabled, angle);
  }

static void cylinder1_stroke2_semiseq_with_stage(void)
  {
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
    assert_fuel_schedules(360U, reqFuel * 100U, enabled, angle);  
}

static void run_1_cylinder_2stroke_tests(void)
{
  configPage2.nCylinders = 1;
  configPage2.strokes = TWO_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 1;

  RUN_TEST_P(cylinder1_stroke2_seq_no_stage);
  RUN_TEST_P(cylinder1_stroke2_semiseq_no_stage);
  RUN_TEST_P(cylinder1_stroke2_seq_with_stage);
  RUN_TEST_P(cylinder1_stroke2_semiseq_with_stage);
}

static void cylinder2_stroke4_seq_no_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
  }

static void cylinder2_stroke4_semiseq_no_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 50U, enabled, angle);
  }

static void cylinder2_stroke4_seq_with_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,0,180,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
  }

static void cylinder2_stroke4_semiseq_with_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,0,180,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 50U, enabled, angle);
}

static void run_2_cylinder_4stroke_tests(void)
{
  configPage2.nCylinders = 2;
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 1;

  RUN_TEST_P(cylinder2_stroke4_seq_no_stage);
  RUN_TEST_P(cylinder2_stroke4_semiseq_no_stage);
  RUN_TEST_P(cylinder2_stroke4_seq_with_stage);
  RUN_TEST_P(cylinder2_stroke4_semiseq_with_stage);
}


static void cylinder2_stroke2_seq_no_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
  }

static void cylinder2_stroke2_semiseq_no_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
  }

static void cylinder2_stroke2_seq_with_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,0,180,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
  }

static void cylinder2_stroke2_semiseq_with_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,0,180,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
}

static void run_2_cylinder_2stroke_tests(void)
{
  configPage2.nCylinders = 2;
  configPage2.strokes = TWO_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 1;

  RUN_TEST_P(cylinder2_stroke2_seq_no_stage);
  RUN_TEST_P(cylinder2_stroke2_semiseq_no_stage);
  RUN_TEST_P(cylinder2_stroke2_seq_with_stage);
  RUN_TEST_P(cylinder2_stroke2_semiseq_with_stage);
}

static void cylinder3_stroke4_seq_no_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,240,480,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
  }

static void cylinder3_stroke4_semiseq_no_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,80,160,0,0,0,0,0};
  assert_fuel_schedules(240U, reqFuel * 50U, enabled, angle);
  }

static void cylinder3_stroke4_seq_with_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS>=6
	const bool enabled[] = {true, true, true, true, true, true, false, false};
	const uint16_t angle[] = {0,240,480,0,240,480,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,240,480,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#endif
  }

static void cylinder3_stroke4_semiseq_with_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS>=6
	const bool enabled[] = {true, true, true, true, true, true, false, false};
	const uint16_t angle[] = {0,80,160,0,80,160,0,0};
  assert_fuel_schedules(240U, reqFuel * 50U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,80,160,0,0,0,0,0};
  assert_fuel_schedules(240U, reqFuel * 50U, enabled, angle); 
#endif
}

static void run_3_cylinder_4stroke_tests(void)
{
  configPage2.nCylinders = 3;
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 1;

  RUN_TEST_P(cylinder3_stroke4_seq_no_stage);
  RUN_TEST_P(cylinder3_stroke4_semiseq_no_stage);
  RUN_TEST_P(cylinder3_stroke4_seq_with_stage);
  RUN_TEST_P(cylinder3_stroke4_semiseq_with_stage);
}

static void cylinder3_stroke2_seq_no_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,80,160,0,0,0,0,0};
  assert_fuel_schedules(120U, reqFuel * 100U, enabled, angle);
  }

static void cylinder3_stroke2_semiseq_no_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,80,160,0,0,0,0,0};
  assert_fuel_schedules(120U, reqFuel * 100U, enabled, angle);
  }

static void cylinder3_stroke2_seq_with_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS>=6
	const bool enabled[] = {true, true, true, true, true, true, false, false};
	const uint16_t angle[] = {0,80,160,0,80,160,0,0};
  assert_fuel_schedules(120U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,80,160,0,0,0,0,0};
  assert_fuel_schedules(120U, reqFuel * 100U, enabled, angle);
#endif
  }

static void cylinder3_stroke2_semiseq_with_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS>=6
	const bool enabled[] = {true, true, true, true, true, true, false, false};
	const uint16_t angle[] = {0,80,160,0,80,160,0,0};
  assert_fuel_schedules(120U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,80,160,0,0,0,0,0};
  assert_fuel_schedules(120U, reqFuel * 100U, enabled, angle);
#endif
}

static void run_3_cylinder_2stroke_tests(void)
{
  configPage2.nCylinders = 3;
  configPage2.strokes = TWO_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 1;
 
  RUN_TEST_P(cylinder3_stroke2_seq_no_stage);
  RUN_TEST_P(cylinder3_stroke2_semiseq_no_stage);
  RUN_TEST_P(cylinder3_stroke2_seq_with_stage);
  RUN_TEST_P(cylinder3_stroke2_semiseq_with_stage);
}

static void cylinder4_stroke4_seq_no_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,360,540,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
  }

static void cylinder4_stroke4_semiseq_no_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 50U, enabled, angle);
  }

static void cylinder4_stroke4_seq_with_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS>=8
	const bool enabled[] = {true, true, true, true, true, true, true, true;
	static const uint16_t angles[] PROGMEM = e[] = {0,180,360,540,0,180,36;
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#elif INJ_CHANNELS >= 5
	const bool enabled[] = {true, true, true, true, true, false, false, false};
	const uint16_t angle[] = {0,180,360,540,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,360,540,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#endif
  }

static void cylinder4_stroke4_semiseq_with_stage(void)  
{
  configPage2.injLayout = INJ_PAIRED;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,0,180,0,0,0,0};
  assert_fuel_schedules(360U, reqFuel * 50U, enabled, angle);
}

void run_4_cylinder_4stroke_tests(void)
{
  configPage2.nCylinders = 4;
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 2;

  RUN_TEST_P(cylinder4_stroke4_seq_no_stage);
  RUN_TEST_P(cylinder4_stroke4_semiseq_no_stage);
  RUN_TEST_P(cylinder4_stroke4_seq_with_stage);
  RUN_TEST_P(cylinder4_stroke4_semiseq_with_stage);  
}

static void cylinder4_stroke2_seq_no_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
  }

static void cylinder4_stroke2_semiseq_no_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, false, false, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
  }

static void cylinder4_stroke2_seq_with_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS>=8
	const bool enabled[] = {true, true, true, true, true, true, true, true;
	const uint16_t angle[] = {0,180,0,0,0,180,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
#elif INJ_CHANNELS >= 5
	const bool enabled[] = {true, true, true, true, true, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,0,0,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
#endif
  }

static void cylinder4_stroke2_semiseq_with_stage(void)
{
  configPage2.injLayout = INJ_PAIRED;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,180,0,180,0,0,0,0};
  assert_fuel_schedules(180U, reqFuel * 100U, enabled, angle);
}

void run_4_cylinder_2stroke_tests(void)
{
  configPage2.nCylinders = 4;
  configPage2.strokes = TWO_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 2;

  RUN_TEST_P(cylinder4_stroke2_seq_no_stage);
  RUN_TEST_P(cylinder4_stroke2_semiseq_no_stage);
  RUN_TEST_P(cylinder4_stroke2_seq_with_stage);
  RUN_TEST_P(cylinder4_stroke2_semiseq_with_stage);  
}

static void cylinder5_stroke4_seq_no_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS >= 5
	const bool enabled[] = {true, true, true, true, true, false, false, false};
	const uint16_t angle[] = {0,144,288,432,576,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#endif
  }


static void cylinder5_stroke4_semiseq_no_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,72,144,216,288,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
  }

static void cylinder5_stroke4_seq_with_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS >= 6
	const bool enabled[] = {true, true, true, true, true, true, false, false};
	const uint16_t angle[] = {0,144,288,432,576,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#endif
  }

static void cylinder5_stroke4_semiseq_with_stage(void) 
{
  configPage2.injLayout = INJ_PAIRED;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS >= 5
	const bool enabled[] = {true, true, true, true, true, false, false, false};
	const uint16_t angle[] = {0,72,144,216,288,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,72,144,216,288,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#endif
}

void run_5_cylinder_4stroke_tests(void)
{
  configPage2.nCylinders = 5;
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 5;

  RUN_TEST_P(cylinder5_stroke4_seq_no_stage);
  RUN_TEST_P(cylinder5_stroke4_semiseq_no_stage);
  RUN_TEST_P(cylinder5_stroke4_seq_with_stage);
  RUN_TEST_P(cylinder5_stroke4_semiseq_with_stage); 
}

static void cylinder6_stroke4_seq_no_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS >= 6
	const bool enabled[] = {true, true, true, true, true, true, false, false};
	static const uint16_t angles[] PROGMEM = = {0,120,240,360,480,600,0,;
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#endif
  }

static void cylinder6_stroke4_semiseq_no_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,120,240,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
  }

static void cylinder6_stroke4_seq_with_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS >= 8
	const bool enabled[] = {true, true, true, true, true, true, false, false};
	static const uint16_t angles[] PROGMEM = = {0,120,240,360,480,600,0,;
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#endif
  }


static void cylinder6_stroke4_semiseq_with_stage(void)
{
  configPage2.injLayout = INJ_SEMISEQUENTIAL;
  configPage10.stagingEnabled = true;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS >= 8
	const bool enabled[] = {true, true, true, true, true, true, true, true;
	static const uint16_t angles[] PROGMEM = = {0,120,240,0,0,120,240,;
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, false, false, false, false, false};
	const uint16_t angle[] = {0,120,240,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#endif
}

void run_6_cylinder_4stroke_tests(void)
{
  configPage2.nCylinders = 6;
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 6;

  RUN_TEST_P(cylinder6_stroke4_seq_no_stage);
  RUN_TEST_P(cylinder6_stroke4_semiseq_no_stage);
  RUN_TEST_P(cylinder6_stroke4_seq_with_stage);
  RUN_TEST_P(cylinder6_stroke4_semiseq_with_stage); 
}

static void cylinder8_stroke4_seq_no_stage(void)
{
  configPage2.injLayout = INJ_SEQUENTIAL;
  configPage10.stagingEnabled = false;
  initialiseAll(); //Run the main initialise function
#if INJ_CHANNELS >= 8
	const bool enabled[] = {true, true, true, true, true, true, true, true;
	static const uint16_t angles[] PROGMEM = e[] = {0,90,180,270,360,450,5;
  assert_fuel_schedules(720U, reqFuel * 100U, enabled, angle);
#else
	const bool enabled[] = {true, true, true, true, false, false, false, false};
	const uint16_t angle[] = {0,0,0,0,0,0,0,0};
  assert_fuel_schedules(720U, reqFuel * 50U, enabled, angle);
#endif
  }

void run_8_cylinder_4stroke_tests(void)
{
  configPage2.nCylinders = 8;
  configPage2.strokes = FOUR_STROKE;
  configPage2.engineType = EVEN_FIRE;
  configPage2.injTiming = true;
  configPage2.reqFuel = reqFuel;
  configPage2.divider = 8;

  // Staging not supported on 8 cylinders

  RUN_TEST_P(cylinder8_stroke4_seq_no_stage);
}


void testFuelScheduleInit()
{
  run_1_cylinder_4stroke_tests();
  run_1_cylinder_2stroke_tests();
  run_2_cylinder_4stroke_tests();
  run_2_cylinder_2stroke_tests();
  run_3_cylinder_4stroke_tests();
  run_3_cylinder_2stroke_tests();
  run_4_cylinder_4stroke_tests();
  run_4_cylinder_2stroke_tests();
  run_5_cylinder_4stroke_tests();
  run_6_cylinder_4stroke_tests();
  run_8_cylinder_4stroke_tests();
}