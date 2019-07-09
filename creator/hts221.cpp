/*
 * Copyright 2016 <Admobilize>
 * MATRIX Labs  [http://creator.matrix.one]
 * This file is part of MATRIX Creator firmware for MCU
 * Author: Andrés Calderón [andres.calderon@admobilize.com]
 *
 * MATRIX Creator firmware for MCU is free software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.

 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This code was based in:
 *
 https://github.com/ControlEverythingCommunity/HTS221/blob/master/Java/HTS221.java
 */

#include "./hts221.h"
#include "./common_data.h"
#include "ch.h"

namespace creator {

HTS221::HTS221(I2C* i2c, uint8_t address) : i2c_(i2c), address_(address) {}

bool HTS221::Begin() {
  /*
    Select average configuration registerTemperature average samples = 16,
    humidity average samples = 32
  */
  Write(0x10, 0x1B);

  /*
  Select control register1
  Power on, block data update, data rate o/p = 1 Hz
  */
  Write(0x20, 0x85);

  /*
  Read Calibration values from the non-volatile memory of the device
  Humidity Calibration values. Variables follow datasheet names.
  */
  H0_rH_x2 = Read(0x30);
  H1_rH_x2 = Read(0x31);
  H0_T0_OUT = ((Read(0x37) & 0xFF) << 8) | (Read(0x36) & 0xFF);
  H1_T0_OUT = ((Read(0x3B) & 0xFF) << 8) | (Read(0x3A) & 0xFF);

  T0_degC_x8 = (Read(0x32) & 0xFF);
  T1_degC_x8 = (Read(0x33) & 0xFF);

  int8_t T_msd = (Read(0x35) & 0x0F);

  T0_degC_x8 = ((T_msd & 0x03) << 8) | T0_degC_x8;
  T1_degC_x8 = ((T_msd & 0x0C) << 6) | T1_degC_x8;

  T1_OUT = ((Read(0x3F) & 0xFF) << 8) + (Read(0x3E) & 0xFF);
  T0_OUT = ((Read(0x3D) & 0xFF) << 8) + (Read(0x3C) & 0xFF);
  if (T1_OUT > 32767) T1_OUT -= 65536;
  if (T0_OUT > 32767) T0_OUT -= 65536;

  /*
  Values to make Interpolation in GetData faster
  */
  H_RATIO = (float)(H1_rH_x2 - H0_rH_x2) / (H1_T0_OUT - H0_T0_OUT) / 2;
  T_RATIO = (float)(T1_degC_x8 - T0_degC_x8) / (T1_OUT - T0_OUT) / 8;

  // Apply factor scale
  H_RATIO *= factor_scale;
  T_RATIO *= factor_scale;

  H_OFFSET = factor_scale * H0_rH_x2 / 2;
  T_OFFSET = factor_scale * T0_degC_x8 / 8;

  return true;
}

void HTS221::GetData(int& humidity, int& temperature) {
  uint8_t data[4];

  // Reading sensor registers
  Read(0x28 | 0x80, data, 4);

  // Fomating the data from hum and temp registers
  int hum = ((data[1] & 0xFF) * 256) + (data[0] & 0xFF);
  int temp = ((data[3] & 0xFF) * 256) + (data[2] & 0xFF);

  // 2's complement conversion
  if (temp > 32767) temp -= 65536;

  // Interpolation using calibration data
  humidity = (int)((hum - H0_T0_OUT) * H_RATIO + H_OFFSET);
  temperature = (int)((temp - T0_OUT) * T_RATIO + T_OFFSET);
}

uint8_t HTS221::Read(uint8_t a) { return i2c_->ReadByte(address_, a); }

void HTS221::Write(uint8_t a, uint8_t d) { i2c_->WriteByte(address_, a, d); }

uint8_t HTS221::Read(uint8_t a, uint8_t* data, uint8_t size) {
  return i2c_->ReadBytes(address_, a, data, size);
}
};  // namespace creator
