#include "globals.h"
#include RTC_LIB_H //Defined in each boards .h file
#ifdef RTC_ENABLED
#include "rtc_common.h"


void initRTC()
{

  #if defined(CORE_TEENSY35) || defined(CORE_TEENSY36)|| defined(CORE_TEENSY41)
    setSyncProvider(getTeensy3Time);
  #elif defined(CORE_STM32)
  
  #endif

}

uint8_t rtc_getSecond()
{
  uint8_t tempSecond = 0;
#ifdef RTC_ENABLED
  #if defined(CORE_TEENSY)
    tempSecond = second();
  #elif defined(CORE_STM32)
    tempSecond = rtc.getSeconds();
  #endif
#endif
  return tempSecond;
}

uint8_t rtc_getMinute()
{
  uint8_t tempMinute = 0;
#ifdef RTC_ENABLED
  #if defined(CORE_TEENSY)
    tempMinute = minute();
  #elif defined(CORE_STM32)
    tempMinute = rtc.getMinutes();
  #endif
#endif
  return tempMinute;
}

uint8_t rtc_getHour()
{
  uint8_t tempHour = 0;
#ifdef RTC_ENABLED
  #if defined(CORE_TEENSY)
    tempHour = hour();
  #elif defined(CORE_STM32)
    tempHour = rtc.getHours();
  #endif
#endif
  return tempHour;
}

uint8_t rtc_getDay()
{
  uint8_t tempDay = 0;
#ifdef RTC_ENABLED
  #if defined(CORE_TEENSY)
    tempDay = day();
  #elif defined(CORE_STM32)
    tempDay = rtc.getDay();
  #endif
#endif
  return tempDay;
}

uint8_t rtc_getDOW()
{
  uint8_t dow = 0;
#ifdef RTC_ENABLED
  #if defined(CORE_TEENSY)
    dow = weekday();
  #elif defined(CORE_STM32)
    dow = rtc.getWeekDay();
  #endif
#endif
  return dow;
}

uint8_t rtc_getMonth()
{
  uint8_t tempMonth = 0;
#ifdef RTC_ENABLED
  #if defined(CORE_TEENSY)
    tempMonth = month();
  #elif defined(CORE_STM32)
    tempMonth = rtc.getMonth();
  #endif
#endif
  return tempMonth;
}

uint16_t rtc_getYear()
{
  uint16_t tempYear = 0;
#ifdef RTC_ENABLED
  #if defined(CORE_TEENSY)
    tempYear = year();
  #elif defined(CORE_STM32)
    //year in stm32 rtc is a byte. So add year 2000 to make it correct
    tempYear = (2000+rtc.getYear());
  #endif
#endif
  return tempYear;
}

void rtc_setTime(byte second, byte minute, byte hour, byte day, byte month, uint16_t year)
{
#ifdef RTC_ENABLED
  #if defined(CORE_TEENSY)
    setTime(hour, minute, second, day, month, year);
    Teensy3Clock.set(now());
  #elif defined(CORE_STM32)
    //If RTC time has not been set earlier (no battery etc.) we need to stop the RTC and restart it with LSE_CLOCK to have accurate RTC.
    if (!rtc.isTimeSet()) {
      rtc.end();
      rtc.setClockSource(STM32RTC::LSE_CLOCK);
      rtc.begin();
    }
    rtc.setTime(hour, minute, second);
    //year in stm32 rtc is a byte. so subtract year 2000 to fit
    rtc.setDate(day, month, (year-2000));
  #endif
#endif
}
#endif
