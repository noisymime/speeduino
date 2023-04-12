#include <Arduino.h>
#include <unity.h>
#include <init.h>

#include "test_schedules.h"

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  delay(2000);

  UNITY_BEGIN(); // start unit testing

  initialiseAll(); //Run the main initialise function
  test_status_initial_off();
  //test_status_off_to_pending();
  //test_status_pending_to_running();
  //test_status_running_to_pending();
  //test_status_running_to_off();
  test_accuracy_timeout();
  test_accuracy_duration();
  
  UNITY_END(); // stop unit testing

}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
}