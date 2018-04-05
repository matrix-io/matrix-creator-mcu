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
 */

#include "./mpl3115a2.h"
#include "./common_data.h"
#include "ch.h"
#include "hal.h"

namespace creator {

const uint8_t MPL3115A2_CTRL_REG1 = 0x26;
const uint8_t MPL3115A2_PT_DATA_CFG = 0x13;
const uint8_t MPL3115A2_REGISTER_STATUS = 0x00;
const uint8_t MPL3115A2_WHOAMI = 0x0C;
const uint8_t MPL3115A2_REGISTER_PRESSURE_MSB = 0x01;
const uint8_t MPL3115A2_REGISTER_TEMP_MSB = 0x04;
const int MPL3115A2_RETRIES = 10;

MPL3115A2::MPL3115A2(I2C* i2c, uint8_t address) : i2c_(i2c), address_(address) {
  CTRL_REG1_.data = 0;
  PT_DATA_CFG_.data = 0;
  DR_STATUS_.data = 0;
}

bool MPL3115A2::Begin() {
  uint8_t whoami = Read(MPL3115A2_WHOAMI);
  if (whoami != 0xC4) {
    return false;
  }

  CTRL_REG1_.data = Read(MPL3115A2_CTRL_REG1);

  CTRL_REG1_.fields.OS = 7;
  Write(MPL3115A2_CTRL_REG1, CTRL_REG1_.data);

  PT_DATA_CFG_.fields.DREM = 1;
  PT_DATA_CFG_.fields.PDEFE = 1;
  PT_DATA_CFG_.fields.TDEFE = 1;

  Write(MPL3115A2_PT_DATA_CFG, PT_DATA_CFG_.data);

  return true;
}

/* Gets pressure level in kPa */
int MPL3115A2::GetPressure() {
  uint32_t pressure;
  int count = 0;

  CTRL_REG1_.data = Read(MPL3115A2_CTRL_REG1);
  CTRL_REG1_.fields.OST = 0;
  CTRL_REG1_.fields.ALT = 0;
  Write(MPL3115A2_CTRL_REG1, CTRL_REG1_.data);

  CTRL_REG1_.data = Read(MPL3115A2_CTRL_REG1);
  CTRL_REG1_.fields.OST = 1;
  Write(MPL3115A2_CTRL_REG1, CTRL_REG1_.data);

  // Read DR_STATUS PDR if Data is Available
  while (true) {
    DR_STATUS_.data = Read(MPL3115A2_REGISTER_STATUS);
    if (DR_STATUS_.fields.PDR) break;
    if (count == MPL3115A2_RETRIES) break;
    chThdSleepMilliseconds(50);
    count++;
  }
  uint8_t data[3];
  i2c_->ReadBytes(address_, MPL3115A2_REGISTER_PRESSURE_MSB, (uint8_t*)data,
                  sizeof(data));

  pressure = ((data[0] << 16) | (data[1] << 8) | data[2]) >> 4;

  return pressure * factor_scale / 4.0;
}

int MPL3115A2::GetAltitude() {
  int32_t altitude;
  int count = 0;

  CTRL_REG1_.data = Read(MPL3115A2_CTRL_REG1);
  CTRL_REG1_.fields.OST = 0;
  CTRL_REG1_.fields.ALT = 1;
  Write(MPL3115A2_CTRL_REG1, CTRL_REG1_.data);

  CTRL_REG1_.data = Read(MPL3115A2_CTRL_REG1);
  CTRL_REG1_.fields.OST = 1;
  Write(MPL3115A2_CTRL_REG1, CTRL_REG1_.data);

  // Read DR_STATUS PDR if Data is Available
  while (true) {
    DR_STATUS_.data = Read(MPL3115A2_REGISTER_STATUS);
    if (DR_STATUS_.fields.PDR) break;
    if (count == MPL3115A2_RETRIES) break;
    chThdSleepMilliseconds(50);
    count++;
  }
  uint8_t data[3];
  i2c_->ReadBytes(address_, MPL3115A2_REGISTER_PRESSURE_MSB, (uint8_t*)data,
                  sizeof(data));

  altitude = ((data[0] << 16) | (data[1] << 8) | data[2]) >> 4;
  if (altitude & 0x80000) altitude |= 0xFFF00000;
  return altitude * factor_scale / 16.0;
}

/* Gets the temperature in °C */
int MPL3115A2::GetTemperature() {
  int count = 0;

  CTRL_REG1_.data = Read(MPL3115A2_CTRL_REG1);
  CTRL_REG1_.fields.OST = 0;
  Write(MPL3115A2_CTRL_REG1, CTRL_REG1_.data);

  CTRL_REG1_.data = Read(MPL3115A2_CTRL_REG1);
  CTRL_REG1_.fields.OST = 1;
  Write(MPL3115A2_CTRL_REG1, CTRL_REG1_.data);

  // Read DR_STATUS PDR if Data is Available
  while (true) {
    DR_STATUS_.data = Read(MPL3115A2_REGISTER_STATUS);
    if (DR_STATUS_.fields.TDR) break;
    if (count == MPL3115A2_RETRIES) break;
    chThdSleepMilliseconds(50);
    count++;
  }
  uint8_t data[2];
  i2c_->ReadBytes(address_, MPL3115A2_REGISTER_TEMP_MSB, (uint8_t*)data,
                  sizeof(data));

  return (((data[0] << 8) | data[1]) >> 4) * factor_scale / 16.0;
}

uint8_t MPL3115A2::Read(uint8_t a) { return i2c_->ReadByte(address_, a); }

void MPL3115A2::Write(uint8_t a, uint8_t d) { i2c_->WriteByte(address_, a, d); }
};  // namespace creator
