/*
 Liam - DIY Robot Lawn Mower

 Clock Library
 Changelog:
 2024-09-11 - modding to suite STM32 built in RTC using STM32RTC library [https://github.com/stm32duino/STM32RTC]

 ======================
  Licensed under GPLv3
 ======================
*/
#include <STM32RTC.h>
#include "Clock.h"
#include "Definition.h"
#include <Wire.h>

/* Check for null pointer references */
if (&rtc == nullptr) {
  Serial.println("RTC is NULL!");
} 
else {
  /* Get the rtc object */
  STM32RTC& rtc = STM32RTC::getInstance();
}

/* Change these values to set the current initial time */
uint8_t seconds = 0;
uint8_t minutes = 0;
uint8_t hours = 16;

/* Change these values to set the current initial date */
/* Monday 15th June 2015 */
uint8_t weekDay = 1;
uint8_t day = 15;
uint8_t month = 6;
uint8_t year = 15;

/*
 * Constructor for the CLOCK class
 */
CLOCK::CLOCK(uint8_t outHour, uint8_t outMinute, uint8_t inHour, uint8_t inMinute) {
  outTimeHour = outHour;
  outTimeMinutes = outMinute;
  inTimeHour = outHour;
  inTimeMinutes = outMinute;

  /* Check for unhandled exceptions */
  try {
    Wire.begin();
    RTC.begin();

    if (! RTC.isrunning()) {
      Serial.println("RTC is NOT running!");
    }
  } catch (const std::exception& e) {
    Serial.println("Exception occurred: " + String(e.what()));
  }
}

/*
 * Set the time on the RTC
 */
void CLOCK::rtc.setTime(uint8_t hours, uint8_t minutes, uint8_t seconds) {
  try {
    RTC.adjust(Time(hours, minutes, seconds));
  } catch (const std::exception& e) {
    Serial.println("Exception occurred: " + String(e.what()));
  }
}

/*
 * Set the date on the RTC
 */
void CLOCK::rtc.setDate(uint8_t weekDay,uint8_t day,uint8_t month,uint8_t year) {
  try {
    RTC.adjust(Date(day, month, year));
  } catch (const std::exception& e) {
    Serial.println("Exception occurred: " + String(e.what()));
  }
}

/*
 * Set the alarm time
 */
void CLOCK::setGoOutTime(uint8_t Hour, uint8_t Minutes) {
  try {
    rtc.attachInterrupt(alarmMatch);
    rtc.setAlarmTime(16, 0, 10, 123); //day, hour, minutes, seconds
    rtc.enableAlarm(rtc.MATCH_DHHMMSS);
    outTimeHour = Hour;
    outTimeMinutes = Minutes;
  } catch (const std::exception& e) {
    Serial.println("Exception occurred: " + String(e.what()));
  }
}

/*
 * Set the alarm time
 */
void CLOCK::setGoHomeTime(uint8_t Hour, uint8_t Minutes) {
  try {
    rtc.attachInterrupt(alarmMatch);
    rtc.setAlarmTime(16, 0, 10, 123); //day, hour, minutes, seconds
    rtc.enableAlarm(rtc.MATCH_DHHMMSS);
    inTimeHour = Hour;
    inTimeMinutes = Minutes;
  } catch (const std::exception& e) {
    Serial.println("Exception occurred: " + String(e.what()));
  }
}

/*
 * Check if it is time to cut the grass
 */
bool CLOCK::timeToCut() {
  try {
    if ((int)RTC.now().hour() * 60 + (int)RTC.now().minute() > (int)outTimeHour * 60 + (int)outTimeMinutes &&
        (int)RTC.now().hour() * 60 + (int)RTC.now().minute() < (int)inTimeHour * 60 + (int)inTimeMinutes)
      return true;
  } catch (const std::exception& e) {
    Serial.println("Exception occurred: " + String(e.what()));
  }

  return false;
}

/*
 * Print the current time
 */
void CLOCK::printTime() {
  try {
    DateTime now = RTC.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
  } catch (const std::exception& e) {
    Serial.println("Exception occurred: " + String(e.what()));
  }
}

