
#include <Arduino.h>
#include <globals.h>
#include <unity.h>

#include "missing_tooth/missing_tooth.h"
#include "dual_wheel/dual_wheel.h"
#include "renix/renix.h"
#include "Nissan360/Nissan360.h"
#include "FordST170/FordST170.h"
#include "NGC/test_ngc.h"
#include "ThirtySixMinus21/ThirtySixMinus21.h"

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);

    UNITY_BEGIN();    // IMPORTANT LINE!

    testMissingTooth();
    testDualWheel();
    testRenix();
    testNissan360();
    testFordST170();
    testNGC();
    testThirtySixMinus21();

    UNITY_END(); // stop unit testing
}

void loop()
{
    // Blink to indicate end of test
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
    delay(250);
}