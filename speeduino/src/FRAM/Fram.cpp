//  Basic read/write functions for the MB85RS64A SPI FRAM chip
//  Copyright (C) 2017  Industruino <connect@industruino.com>
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
//
//  Developed by Claudio Indellicati <bitron.it@gmail.com>
//
//  Mod by Vitor_Boss on 01/2019
//    work with STM32
//    added option to use any SPI port
//    added software version of SPI with configurable speed

#include "Fram.h"
#ifdef SPI_HAS_TRANSACTION
  SPISettings FRAMSettings(FRAM_DEFAULT_CLOCK, MSBFIRST, SPI_MODE0);
#endif

void FramClass::assertCS(void)
{ 
#ifdef SPI_HAS_TRANSACTION
  spi->beginTransaction(FRAMSettings); 
#endif
  *csPort &= ~(csMask); 
}

void FramClass::deassertCS(void)
{
#ifdef SPI_HAS_TRANSACTION
  spi->endTransaction();
#endif
  *csPort |= (csMask);
}

/*-----------------------------------------------------------------------------*/

FramClass::FramClass(void)
{
  clkPin = mosiPin = misoPin = NC;
  csPin = FRAM_DEFAULT_CS_PIN;
  csPinInit();
  begin(csPin);
}

/*-----------------------------------------------------------------------------*/

#if defined(ARDUINO_ARCH_STM32) || defined(__IMXRT1062__)
FramClass::FramClass (uint32_t ssel, SPIClass &_spi)
#else
FramClass::FramClass (uint8_t ssel, SPIClass &_spi)
#endif
{
  clkPin = mosiPin = misoPin = NC;
  begin(ssel, _spi);
}

/*-----------------------------------------------------------------------------*/

#if defined(ARDUINO_ARCH_STM32) || defined(__IMXRT1062__)
FramClass::FramClass (uint32_t mosi, uint32_t miso, uint32_t sclk, uint32_t ssel, uint32_t clockspeed)
#else
FramClass::FramClass (uint8_t mosi, uint8_t miso, uint8_t sclk, uint8_t ssel, uint32_t clockspeed)
#endif
{
  csPin = ssel;
  clkPin = sclk;
  misoPin = miso;
  mosiPin = mosi;
  setClock(clockspeed);
  csPinInit();

  if (clkPin != NC)
  {
    pinMode(clkPin, OUTPUT);
    clkPort = portOutputRegister(digitalPinToPort(clkPin));
    clkMask = digitalPinToBitMask(clkPin);
  }
  if (mosiPin != NC)
  {
    pinMode(mosiPin, OUTPUT);
    mosiPort = portOutputRegister(digitalPinToPort(mosiPin));
    mosiMask = digitalPinToBitMask(mosiPin);
  }
  // Set CS pin HIGH and configure it as an output
  pinMode(csPin, OUTPUT);
  pinMode(misoPin, INPUT_PULLUP);
  deassertCS();
}

/*-----------------------------------------------------------------------------*/

void FramClass::enableWrite (uint8_t state)
{
  assertCS();
  if (state){ spiSend(FRAM_CMD_WREN); }
  else { spiSend(FRAM_CMD_WRDI); }
  deassertCS();
}

/*-----------------------------------------------------------------------------*/

void FramClass::setClock(uint32_t clockSpeed) {
  spiSpeed = 1000000 / (clockSpeed * 2);
  #ifdef SPI_HAS_TRANSACTION
  FRAMSettings = SPISettings(clockSpeed, MSBFIRST, SPI_MODE0);
  #if defined(ARDUINO_ARCH_STM32)
  spi->beginTransaction(csPin, FRAMSettings);
  #else
  spi->beginTransaction(FRAMSettings);
  #endif
  #endif
}

/*-----------------------------------------------------------------------------*/

uint8_t FramClass::isDeviceActive(void) {
  uint8_t result;
  enableWrite(1); //Best way of detecting a device
  uint8_t SR = readSR();
  result = (SR!=0) && (SR!=255);
  enableWrite(0);
  return result;
}

/*-----------------------------------------------------------------------------*/

#if defined(ARDUINO_ARCH_STM32) || defined(__IMXRT1062__)
void FramClass::begin (uint32_t ssel, SPIClass &_spi)
#else
void FramClass::begin (uint8_t ssel, SPIClass &_spi)
#endif
{
  clkPin = mosiPin = misoPin = NC;
  csPin = ssel;
  spi = &_spi;
  
  // Set CS pin HIGH and configure it as an output
  csPinInit();
  deassertCS();
  setClock(FRAM_DEFAULT_CLOCK);
  #ifdef SPI_HAS_TRANSACTION
    spi->begin();
    spi->beginTransaction(FRAMSettings);
  #else
  #if defined(STM32F2)
    spi->setClockDivider (SPI_CLOCK_DIV4); // SPI @ 15MHz
  #elif defined(STM32F4)
    spi->setClockDivider (SPI_CLOCK_DIV16);
  #else
    spi->setClockDivider (SPI_CLOCK_DIV2); // 8 MHz
  #endif
    spi->setDataMode(SPI_MODE0);
    spi->begin();
  #endif
  delayMicroseconds(15);//>3us
}

/*-----------------------------------------------------------------------------*/

uint8_t FramClass::write (uint32_t addr, uint8_t data)
{
  enableWrite(1);
  assertCS();
  spiSend(FRAM_CMD_WRITE);
  sendAddr(addr);
  spiSend(data);
  deassertCS();
  #if defined(ARDUINO_ARCH_STM32)
  delayMicroseconds(5);
  #else
  SOFT_DELAY(5);
  #endif
  enableWrite(0);

  return 1U;
}

/*-----------------------------------------------------------------------------*/

uint8_t FramClass::write (uint32_t addr, uint8_t *data, uint16_t count)
{
  if (addr + count > FRAM_SIZE)
    return 0U;

  if (count == 0U)
    return 255;

  enableWrite(1);
  assertCS();
  spiSend(FRAM_CMD_WRITE);
  sendAddr(addr);
  for (uint16_t i = 0; i < count; ++i)
    spiSend(data[i]);
  deassertCS();
  #if defined(ARDUINO_ARCH_STM32)
  delayMicroseconds(5);
  #else
  SOFT_DELAY(5);
  #endif
  enableWrite(0);

  return 1U;
}

/*-----------------------------------------------------------------------------*/

uint8_t FramClass::read (uint32_t addr, uint8_t *dataBuffer, uint16_t count)
{
  if (addr + count > FRAM_SIZE)
    return 0U;

  if (count == 0U)
    return 255;

  assertCS();
  spiSend(FRAM_CMD_READ);
  sendAddr(addr);
  for (uint16_t i=0; i < count; ++i)
    dataBuffer[i] = spiSend(DUMMYBYTE);
  deassertCS();

  return 1U;
}

/*-----------------------------------------------------------------------------*/

uint8_t FramClass::read (uint32_t addr)
{
  uint8_t dataBuffer;

  assertCS();
  spiSend(FRAM_CMD_READ);
  sendAddr(addr);
  dataBuffer = spiSend(DUMMYBYTE);
  deassertCS();

  return dataBuffer;
}

/*-----------------------------------------------------------------------------*/

uint8_t FramClass::update (uint32_t addr, uint8_t data)
{
  if(read(addr) != data)
    write(addr, data);
  return 1U;
}

uint8_t FramClass::clear(void)
{
  enableWrite(1);
  assertCS();
  spiSend(FRAM_CMD_WRITE);
  sendAddr(0x0000);
  for (uint32_t i = 0; i < FRAM_SIZE; ++i)
    spiSend(0x00);
  deassertCS();
  #if defined(ARDUINO_ARCH_STM32)
  delayMicroseconds(5);
  #else
  SOFT_DELAY(5);
  #endif
  enableWrite(0);

  return 1U;
}
/*-----------------------------------------------------------------------------*/

uint8_t FramClass::readSR(void)
{
  uint8_t dataBuffer;

  assertCS();
  spiSend(FRAM_CMD_RDSR);
  dataBuffer = spiSend(DUMMYBYTE);
  deassertCS();

  return dataBuffer;
}

/*-----------------------------------------------------------------------------*/
void FramClass::sendAddr(uint32_t addr)
{
  spiSend16(addr & 0xFFFF);
#if((FRAM_SIZE > 0xFFFF) && (FRAM_SIZE <= 0x00FFFFFF))
  spiSend((addr>>16) & 0xFF);
#elif(FRAM_SIZE >= 0x00FFFFFF)
  spiSend16((addr>>16) & 0xFFFF);
#endif
}

uint32_t FramClass::length(void)
{
  return FRAM_SIZE;
}

uint8_t FramClass::spiSend(uint8_t data) 
{
  uint8_t reply = 0;
  if(clkPin != NC)
  {
    for (int i=7; i>=0; i--)
    {
      reply <<= 1;
      setClockPin(LOW);
      fastWrite(mosiPort, mosiMask, (data & ((uint8_t)1<<i)));
      setClockPin(HIGH);
      reply |= digitalRead(misoPin);
    }
    fastWrite(clkPort, clkMask, LOW);
  }
#if defined(ARDUINO_ARCH_STM32)
  else { reply = spi->transfer(csPin, data, SPI_CONTINUE); }
#else
  else { reply = spi->transfer(data); }
#endif
  return reply;
}

/*-----------------------------------------------------------------------------*/

uint16_t FramClass::spiSend16(uint16_t data) 
{
  uint16_t reply = 0;
  if(clkPin != NC)
  {
    for (int i=15; i>=0; i--)
    {
      reply <<= 1;
      setClockPin(LOW);
      fastWrite(mosiPort, mosiMask, (data & ((uint16_t)1<<i)));
      setClockPin(HIGH);
      reply |= digitalRead(misoPin);
    }
    fastWrite(clkPort, clkMask, LOW);
  }
#if defined(ARDUINO_ARCH_STM32)
  else { reply = spi->transfer16(csPin, data, SPI_CONTINUE); }
#else
  else { reply = spi->transfer16(data); }
#endif
  return reply;
}

/*-----------------------------------------------------------------------------*/


//FramClass Fram;

