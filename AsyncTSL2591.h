/*
    AsyncTSL2591.h

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



#ifndef AsyncTSL2591_H
#define AsyncTSL2591_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define TSL2591_I2C_ADDRESS 0x29

//Command register
#define TSL2591_CMD_REGISTER 0x80
#define TSL2591_CMD_NORMAL_OPERATION 0x20
#define TSL2591_CMD_SPECIAL_FUNCTION 0x30
#define TSL2591_CMD_SP_SET_INTERRUPT 0x04
#define TSL2591_CMD_SP_CLEAR_ALS_INT 0x06
#define TSL2591_CMD_SP_CLEAR_ALS_NO_PERSIST_INT 0x07
#define TSL2591_CMD_SP_CLEAR_NO_PERSIST_INT 0x0A
#define TSL2591_CMD 0xA0 //TSL2591_CMD_REGISTER | TSL2591_CMD_NORMAL_OPERATION

//Enable register
#define TSL2591_ENABLE_REGISTER 0x00
#define TSL2591_ENABLE_NPIEN 0x80
#define TSL2591_ENABLE_SAI 0x40
#define TSL2591_ENABLE_AIEN 0x10
#define TSL2591_ENABLE_AEN 0x02
#define TSL2591_ENABLE_PON 0x01
#define TSL2591_ENABLE_POF 0x00

//Control register
#define TSL2591_CONTROL_REGISTER 0x01
#define TSL2591_CONTROL_SRESET 0x80

typedef enum
{
  TSL2591_CONTROL_AGAIN_LOW = 0x00,
  TSL2591_CONTROL_AGAIN_MED = 0x10,
  TSL2591_CONTROL_AGAIN_HIGH = 0x20,
  TSL2591_CONTROL_AGAIN_MAX = 0x30
}
TSL2591_CONTROL_AGAIN_t;

#define TSL2591_CONTROL_AGAIN_LOW_C0  1
#define TSL2591_CONTROL_AGAIN_LOW_C1  1
#define TSL2591_CONTROL_AGAIN_MED_C0  24.5
#define TSL2591_CONTROL_AGAIN_MED_C1  24.5
#define TSL2591_CONTROL_AGAIN_HIGH_C0 400
#define TSL2591_CONTROL_AGAIN_HIGH_C1 400
#define TSL2591_CONTROL_AGAIN_MAX_C0 9200
#define TSL2591_CONTROL_AGAIN_MAX_C1 9900


typedef enum
{
  TSL2591_CONTROL_ATIME_100MS = 0x00,
  TSL2591_CONTROL_ATIME_200MS = 0x01,
  TSL2591_CONTROL_ATIME_300MS = 0x02,
  TSL2591_CONTROL_ATIME_400MS = 0x03,
  TSL2591_CONTROL_ATIME_500MS = 0x04,
  TSL2591_CONTROL_ATIME_600MS = 0x05
}
TSL2591_CONTROL_ATIME_t;

//ALS interrupt threshold register
#define TSL2591_ALS_INT_TRESHOLD_AILTL 0x04
#define TSL2591_ALS_INT_TRESHOLD_AILTH 0x05
#define TSL2591_ALS_INT_TRESHOLD_AIHTL 0x06
#define TSL2591_ALS_INT_TRESHOLD_AIHTH 0x07
#define TSL2591_ALS_INT_TRESHOLD_NPAILTL 0x08
#define TSL2591_ALS_INT_TRESHOLD_NPAILTH 0x09
#define TSL2591_ALS_INT_TRESHOLD_NPAIHTL 0x0A
#define TSL2591_ALS_INT_TRESHOLD_NPAIHTH 0x0B

//Persist register
#define TSL2591_PERSIST_REGISTER 0x0C

//PID register
#define TSL2591_PID_REGISTER 0x11
#define TSL2591_PID_PACKAGEID 0x30

//ID register
#define TSL2591_ID_REGISTER 0x12
#define TSL2591_DEVICE_IDENTIFICATION 0x50

//Status register
#define TSL2591_STATUS_REGISTER 0x13
#define TSL2591_STATUS_NPINTR 0x20
#define TSL2591_STATUS_AINT 0x10
#define TSL2591_STATUS_AVALID 0x01

//ALS data register
#define TSL2591_ALSDATA_C0DATAL 0x14
#define TSL2591_ALSDATA_C0DATAH 0x15
#define TSL2591_ALSDATA_C1DATAL 0x16
#define TSL2591_ALSDATA_C1DATAH 0x17


class AsyncTSL2591Data {
public:
  AsyncTSL2591Data(const uint16_t c0, const uint16_t c1, const TSL2591_CONTROL_AGAIN_t again, const TSL2591_CONTROL_ATIME_t atime);

  const uint16_t c0;
  const uint16_t c1;
  const TSL2591_CONTROL_AGAIN_t again;
  const TSL2591_CONTROL_ATIME_t atime;

  float calculateLux();
};

class AsyncTSL2591 {
public:
  AsyncTSL2591();

  bool begin(TSL2591_CONTROL_AGAIN_t again = TSL2591_CONTROL_AGAIN_LOW, TSL2591_CONTROL_ATIME_t atime = TSL2591_CONTROL_ATIME_200MS);

  AsyncTSL2591Data syncLuminosityMeasurement(const bool clearDoubleBuffer = true);

  bool startLuminosityMeasurement();

  bool isMeasurementReady();

  AsyncTSL2591Data getLuminosityMeasurement(const bool clearDoubleBuffer = true);

  
  void setAlsGain(const TSL2591_CONTROL_AGAIN_t again);
  void setAlsTime(const TSL2591_CONTROL_ATIME_t atime);

  TSL2591_CONTROL_AGAIN_t getAlsGain();
  TSL2591_CONTROL_ATIME_t getAlsTime();

private:


  TSL2591_CONTROL_AGAIN_t _again;
  TSL2591_CONTROL_ATIME_t _atime;

  void _updateAlsGainAndTime();

  void _powerOn(uint8_t reg = 0);
  void _powerOff();

  uint8_t _read();
  uint16_t _read16();
  void _write(uint8_t reg, bool i2cStopMessage   = true);
  void _writeValue(uint8_t reg, uint8_t value, bool i2cStopMessage  = true);

  uint8_t _readRegister(uint8_t reg);
  uint16_t _readRegister16(uint8_t reg);
};

#endif // AsyncTSL2591_H







