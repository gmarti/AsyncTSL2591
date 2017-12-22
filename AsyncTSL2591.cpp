/*
    AsyncTSL2591.cpp

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

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include "AsyncTSL2591.h"

AsyncTSL2591Data::AsyncTSL2591Data(const uint16_t c0, const uint16_t c1, const TSL2591_CONTROL_AGAIN_t again, const TSL2591_CONTROL_ATIME_t atime)
  : c0(c0), c1(c1), again(again), atime(atime) {}

float AsyncTSL2591Data::calculateLux() {
  const float atimeValue = (atime + 1) * 100.0;

  float againC0;
  float againC1;

  if (again == TSL2591_CONTROL_AGAIN_LOW) {
    againC0 = TSL2591_CONTROL_AGAIN_LOW_C0;
    againC1 = TSL2591_CONTROL_AGAIN_LOW_C1;
  }
  else if (again == TSL2591_CONTROL_AGAIN_MED) {
    againC0 = TSL2591_CONTROL_AGAIN_MED_C0;
    againC1 = TSL2591_CONTROL_AGAIN_MED_C1;
  }
  else if (again == TSL2591_CONTROL_AGAIN_HIGH) {
    againC0 = TSL2591_CONTROL_AGAIN_HIGH_C0;
    againC1 = TSL2591_CONTROL_AGAIN_HIGH_C1;
  }
  else if (again == TSL2591_CONTROL_AGAIN_MAX) {
    againC0 = TSL2591_CONTROL_AGAIN_MAX_C0;
    againC1 = TSL2591_CONTROL_AGAIN_MAX_C1;
  }

#define MILLIW_PER_CM2_TO_LUX_550n  6.83
#define MILLIW_PER_CM2_TO_LUX_SUN ((1/0.0079)/100)
#define MILLIW_PER_CM2_TO_LUX_MY 4.1359

//Because TSL2591_CONTROL_AGAIN_HIGH_C0 == TSL2591_CONTROL_AGAIN_HIGH_C1
  float constPart =  (TSL2591_CONTROL_AGAIN_HIGH_C0 * 100.0 * MILLIW_PER_CM2_TO_LUX_MY) / atimeValue;
  float luxWhite = (((float)c0 / (264.1 * againC0)) - ((float)c1 / (34.9 * againC1))) * constPart;
  float lux850nm = (((float)c0 / (257.5 * againC0)) - ((float)c1 / (154.1 * againC1))) * constPart;

  return max(lux850nm, max(luxWhite, 0));

}

AsyncTSL2591::AsyncTSL2591() {}

bool AsyncTSL2591::begin(TSL2591_CONTROL_AGAIN_t again = TSL2591_CONTROL_AGAIN_LOW, TSL2591_CONTROL_ATIME_t atime = TSL2591_CONTROL_ATIME_200MS) {
  Wire.begin();
  uint8_t id = _readRegister(TSL2591_CMD | TSL2591_ID_REGISTER);


  if (id != TSL2591_DEVICE_IDENTIFICATION) {
    return false;
  }
  _again = again;
  _atime = atime;
  _updateAlsGainAndTime();
  _powerOff();
  return true;
}

void AsyncTSL2591::setAlsGain(const TSL2591_CONTROL_AGAIN_t again) {
  _again = again;
  _updateAlsGainAndTime();
}

void AsyncTSL2591::setAlsTime(const TSL2591_CONTROL_ATIME_t atime) {
  _atime = atime;
  _updateAlsGainAndTime();
}

TSL2591_CONTROL_AGAIN_t AsyncTSL2591::getAlsGain() {
  return _again;
}

TSL2591_CONTROL_ATIME_t AsyncTSL2591::getAlsTime() {
  return _atime;
}

void AsyncTSL2591::_updateAlsGainAndTime() {
  _writeValue(TSL2591_CMD | TSL2591_CONTROL_REGISTER, _atime | _again);
}

void AsyncTSL2591::_powerOn(uint8_t reg = 0) {
  _writeValue(TSL2591_CMD | TSL2591_ENABLE_REGISTER, TSL2591_ENABLE_PON | reg);
}

void AsyncTSL2591::_powerOff() {
  _writeValue(TSL2591_CMD | TSL2591_ENABLE_REGISTER, TSL2591_ENABLE_POF);
}

AsyncTSL2591Data AsyncTSL2591::syncLuminosityMeasurement(const bool clearDoubleBuffer = true) {
  startLuminosityMeasurement();
  while (!isMeasurementReady()) {}
  return getLuminosityMeasurement(clearDoubleBuffer);
}

bool AsyncTSL2591::startLuminosityMeasurement() {
  _powerOn(TSL2591_ENABLE_AEN);
}

AsyncTSL2591Data AsyncTSL2591::getLuminosityMeasurement(const bool clearDoubleBuffer = true) {
  _powerOff();

  //c0 visible
  //c1 IR
  uint16_t c0 ;
  uint16_t c1 ;
  c1 = _readRegister16(TSL2591_CMD | TSL2591_ALSDATA_C1DATAL);
  c0 = _readRegister16(TSL2591_CMD | TSL2591_ALSDATA_C0DATAL);
  if (clearDoubleBuffer) {
    c1 = _readRegister16(TSL2591_CMD | TSL2591_ALSDATA_C1DATAL);
    c0 = _readRegister16(TSL2591_CMD | TSL2591_ALSDATA_C0DATAL);
  }

  return AsyncTSL2591Data(c0, c1, _again, _atime);
}


bool AsyncTSL2591::isMeasurementReady() {
  int reg = _readRegister(TSL2591_CMD | TSL2591_STATUS_REGISTER);
  return reg & TSL2591_STATUS_AVALID;
}


uint8_t AsyncTSL2591::_readRegister(uint8_t reg) {
  _write(reg);
  return _read();
}
uint16_t AsyncTSL2591::_readRegister16(uint8_t reg) {
  _write(reg);
  return _read16();
}

uint8_t AsyncTSL2591::_read() {
  Wire.requestFrom(TSL2591_I2C_ADDRESS, 1);
  return Wire.read();
}

uint16_t AsyncTSL2591::_read16() {
  Wire.requestFrom(TSL2591_I2C_ADDRESS, 2);
  uint16_t result;
  result = Wire.read();
  result |= Wire.read() << 8;
  return result;
}

void AsyncTSL2591::_write(uint8_t reg, bool i2cStopMessage  = true) {
  Wire.beginTransmission(TSL2591_I2C_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(i2cStopMessage);
}

void AsyncTSL2591::_writeValue(uint8_t reg, uint8_t value, bool i2cStopMessage  = true) {
  Wire.beginTransmission(TSL2591_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission(i2cStopMessage);
}
