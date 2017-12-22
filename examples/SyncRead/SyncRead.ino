/*
    SyncRead.ino

    Created by Grégory Marti <greg.marti@gmail.com>
    Copyright 2017 Grégory Marti

    This file is part of the AsyncTSL2591 library.

    AsyncTSL2591 library is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <AsyncTSL2591.h>


AsyncTSL2591 sensor;


const TSL2591_CONTROL_AGAIN_t again = TSL2591_CONTROL_AGAIN_MED;
const TSL2591_CONTROL_ATIME_t atime = TSL2591_CONTROL_ATIME_400MS;


void setup() {
  Serial.begin(9600);
  if (sensor.begin(again, atime)) {
    Serial.print("Sensor found !");
  } else {
    Serial.print("Sensor not found !");
  }
}


void loop() {

  unsigned long startTime;
  unsigned long duration;

  Serial.println("Luminosity measurement");
  startTime = millis();

  AsyncTSL2591Data data = sensor.syncLuminosityMeasurement();

  duration = millis() - startTime;
  Serial.print(duration);
  
  Serial.print("ms (following datasheet, should be between ");Serial.print(95 * (atime +1));Serial.print(" and ");Serial.print(108 * (atime +1)); Serial.println("ms");

  Serial.print(F("Full: ")); Serial.println(data.c0);
  Serial.print(F("IR: ")); Serial.println(data.c1);
  float lux = data.calculateLux();
  Serial.print("Luminosity : ");
  Serial.print(lux, 2);
  Serial.println("lux");
  delay(2000);
}
