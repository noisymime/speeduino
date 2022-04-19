/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

#ifdef USE_DISPLAY

#include <SPI.h>
#include <Wire.h>
#include "src/Adafruit_SSD1306/Adafruit_GFX.h"
#include "src/Adafruit_SSD1306/Adafruit_SSD1306.h"

Adafruit_SSD1306 display(pinDisplayReset);

void initialiseDisplay()
{
  //Protection against older pin mappings where the crank/cam signals were on the SDA and SCL pins. This will cause the Arduino to lock hard if you try to initialise i2c devices when a crank signal is coming in
  if(pinTrigger == 20 || pinTrigger == 21 || pinTrigger2 == 20 || pinTrigger2 == 21) { return; }

   switch(configPage1.displayType)
   {
     case 1:
       display.SSD1306_SETCOMPINS_V = 0x02;
      break;
    case 2:
       display.SSD1306_SETCOMPINS_V = 0x12;
      break;
     case 3:
       display.SSD1306_SETCOMPINS_V = 0x12;
      break;
     case 4:
       display.SSD1306_SETCOMPINS_V = 0x12;
      break;
   }

   display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialise with the I2C addr 0x3C (for the 128x32)
   display.clearDisplay();
   display.setTextSize(1);
   display.setTextColor(WHITE);
   display.setCursor(0,0);
   display.print("RPM: ");
   display.setCursor(0,16);
   display.print("CPU: ");
}

void updateDisplay()
{
  display.clearDisplay();
  display.setCursor(0,0);
  switch(configPage1.display1)
  {
    case 0:
      display.print("RPM: ");
      display.setCursor(28,0);
      display.print(currentStatus.RPM);
      break;
    case 1:
      display.print("PW: ");
      display.setCursor(28,0);
      display.print(currentStatus.PW1);
      break;
    case 2:
      display.print("Adv: ");
      display.setCursor(28,0);
      display.print(currentStatus.advance);
      break;
    case 3:
      display.print("VE: ");
      display.setCursor(28,0);
      display.print(currentStatus.VE);
      break;
    case 4:
      display.print("GamE: ");
      display.setCursor(28,0);
      display.print(currentStatus.corrections);
      break;
    case 5:
      display.print("TPS: ");
      display.setCursor(28,0);
      display.print(currentStatus.TPS);
      break;
    case 6:
      display.print("IAT: ");
      display.setCursor(28,0);
      display.print(currentStatus.IAT);
      break;
    case 7:
      display.print("CLT: ");
      display.setCursor(28,0);
      display.print(currentStatus.coolant);
      break;
  }

  display.setCursor(0,11);
  switch(configPage1.display3)
  {
    case 0:
      display.print("RPM: ");
      display.setCursor(28,11);
      display.print(currentStatus.RPM);
      break;
    case 1:
      display.print("PW: ");
      display.setCursor(28,11);
      display.print(currentStatus.PW1);
      break;
    case 2:
      display.print("Adv: ");
      display.setCursor(28,11);
      display.print(currentStatus.advance);
      break;
    case 3:
      display.print("VE: ");
      display.setCursor(28,11);
      display.print(currentStatus.VE);
      break;
    case 4:
      display.print("GamE: ");
      display.setCursor(28,11);
      display.print(currentStatus.corrections);
      break;
    case 5:
      display.print("TPS: ");
      display.setCursor(28,11);
      display.print(currentStatus.TPS);
      break;
    case 6:
      display.print("IAT: ");
      display.setCursor(28,11);
      display.print(currentStatus.IAT);
      break;
    case 7:
      display.print("CLT: ");
      display.setCursor(28,11);
      display.print(currentStatus.coolant);
      break;
  }

  display.setCursor(64,0);
  switch(configPage1.display2)
  {
    case 0:
      display.print("O2: ");
      display.setCursor(92,0);
      display.print(currentStatus.O2);
      break;
    case 1:
      display.print("Vdc: ");
      display.setCursor(92,0);
      display.print(currentStatus.battery10);
      break;
    case 2:
      display.print("CPU: ");
      display.setCursor(92,0);
      display.print(currentStatus.loopsPerSecond);
      break;
    case 3:
      display.print("Mem: ");
      display.setCursor(92,0);
      display.print(currentStatus.freeRAM);
      break;
  }

  display.setCursor(64,11);
  switch(configPage1.display4)
  {
    case 0:
      display.print("O2: ");
      display.setCursor(92,11);
      display.print(currentStatus.O2);
      break;
    case 1:
      display.print("Vdc: ");
      display.setCursor(92,11);
      display.print(currentStatus.battery10);
      break;
    case 2:
      display.print("CPU: ");
      display.setCursor(92,11);
      display.print(currentStatus.loopsPerSecond);
      break;
    case 3:
      display.print("Mem: ");
      display.setCursor(92,11);
      display.print(currentStatus.freeRAM);
      break;
  }

  int barWidth = ldiv(((unsigned long)currentStatus.RPM * 128), 9000).quot;
  //int barWidth = map(currentStatus.RPM, 0, 9000, 0, 128);
  display.fillRect(0, 20, barWidth, 10, 1);

  display.display();
}
#endif
