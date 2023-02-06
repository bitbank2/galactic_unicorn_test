#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"

#include "pico_graphics.hpp"
#include "galactic_unicorn.hpp"
#include "okcolor.hpp"
#include <SCD41.h>
#include <rtc_eeprom.h>

#define SDA_PIN 4
#define SCL_PIN 5

using namespace pimoroni;

PicoGraphics_PenRGB888 graphics(53, 11, nullptr);
GalacticUnicorn galactic_unicorn;
SCD41 mySensor;
uint8_t u8Brightness;

void setup()
{
  //stdio_init_all();
  delay(250);
  rtcInit(RTC_DS3231, SDA_PIN, SCL_PIN, 0);

  galactic_unicorn.init();
  //galactic_unicorn.set_brightness(0.5f);
  // Start periodic measurements
  if (mySensor.init(SDA_PIN, SCL_PIN, true, 100000) != SCD41_SUCCESS)
  {
    Serial.println("CO2 Sensor Failure!");
    while (1) {};
  }
  mySensor.start();
} /* setup() */

void loop()
{
  struct tm myTime;

  while(true) {
    char szTemp[32];

    rtcGetTime(&myTime);
    mySensor.getSample();

    if(galactic_unicorn.is_pressed(galactic_unicorn.SWITCH_BRIGHTNESS_UP)) {
      galactic_unicorn.adjust_brightness(+0.01f);
    }
    if(galactic_unicorn.is_pressed(galactic_unicorn.SWITCH_BRIGHTNESS_DOWN)) {
      galactic_unicorn.adjust_brightness(-0.01f);
    }

    graphics.set_pen(0, 0, 0);
    graphics.clear();
    graphics.set_font("bitmap6");
    // Set brightness based on the time of day
    u8Brightness = 2; // assume super dim for night
    if (myTime.tm_hour >= 10 && myTime.tm_hour < 23)
       u8Brightness = 32; // daytime, use a brighter setting
    graphics.set_pen(0, u8Brightness, 0); // time in green
    sprintf(szTemp, "%02d:%02d", myTime.tm_hour, myTime.tm_min);
    graphics.text(szTemp, Point(0, -1), -1, 1);
    sprintf(szTemp, "%d ", mySensor.temperature());
    graphics.set_pen(u8Brightness, u8Brightness, 0); // temp in yellow
    graphics.text(szTemp, Point(26, -1), -1, 1);
    sprintf(szTemp, "%d ", mySensor.humidity());
    graphics.set_pen(u8Brightness, 0, u8Brightness); // humidity in magenta
    graphics.text(szTemp, Point(40, -1), -1, 1);
    sprintf(szTemp, "%d ppm", mySensor.co2());
    graphics.set_pen(u8Brightness, 0, 0); // CO2 in red
    graphics.text(szTemp, Point(0, 5), -1, 1);
    galactic_unicorn.update(&graphics);
    delay(5000);
  } // while
} /* loop() */

