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

#include "ch.h"
#include "./mpl3115a2.h"

namespace creator {

const uint8_t MPL3115A2_REGISTER_STATUS = 0x00;
const uint8_t MPL3115A2_REGISTER_STATUS_TDR = 0x02;
const uint8_t MPL3115A2_REGISTER_STATUS_PDR = 0x04;
const uint8_t MPL3115A2_REGISTER_STATUS_PTDR = 0x08;

const uint8_t MPL3115A2_CTRL_REG1 = 0x26;
const uint8_t MPL3115A2_CTRL_REG1_SBYB = 0x01;
const uint8_t MPL3115A2_CTRL_REG1_OST = 0x02;
const uint8_t MPL3115A2_CTRL_REG1_RST = 0x04;
const uint8_t MPL3115A2_CTRL_REG1_OS1 = 0x00;
const uint8_t MPL3115A2_CTRL_REG1_OS2 = 0x08;
const uint8_t MPL3115A2_CTRL_REG1_OS4 = 0x10;
const uint8_t MPL3115A2_CTRL_REG1_OS8 = 0x18;
const uint8_t MPL3115A2_CTRL_REG1_OS16 = 0x20;
const uint8_t MPL3115A2_CTRL_REG1_OS32 = 0x28;
const uint8_t MPL3115A2_CTRL_REG1_OS64 = 0x30;
const uint8_t MPL3115A2_CTRL_REG1_OS128 = 0x38;
const uint8_t MPL3115A2_CTRL_REG1_RAW = 0x40;
const uint8_t MPL3115A2_CTRL_REG1_ALT = 0x80;
const uint8_t MPL3115A2_CTRL_REG1_BAR = 0x00;

const uint8_t MPL3115A2_WHOAMI = 0x0C;

const uint8_t MPL3115A2_PT_DATA_CFG = 0x13;
const uint8_t MPL3115A2_PT_DATA_CFG_TDEFE = 0x01;
const uint8_t MPL3115A2_PT_DATA_CFG_PDEFE = 0x02;
const uint8_t MPL3115A2_PT_DATA_CFG_DREM = 0x04;

const uint8_t MPL3115A2_REGISTER_PRESSURE_MSB = 0x01;
const uint8_t MPL3115A2_REGISTER_TEMP_MSB = 0x04;

MPL3115A2::MPL3115A2(I2C* i2c, uint8_t address)
    : i2c_(i2c), address_(address) {}

bool MPL3115A2::Begin() {
  uint8_t whoami = Read(MPL3115A2_WHOAMI);
  if (whoami != 0xC4) {
    return false;
  }

  Write(MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_SBYB |
                                 MPL3115A2_CTRL_REG1_OS128 |
                                 MPL3115A2_CTRL_REG1_ALT);

  Write(MPL3115A2_PT_DATA_CFG, MPL3115A2_PT_DATA_CFG_TDEFE |
                                   MPL3115A2_PT_DATA_CFG_PDEFE |
                                   MPL3115A2_PT_DATA_CFG_DREM);

  return true;
}

/* Gets pressure level in kPa */
float MPL3115A2::GetPressure() {
  uint32_t pressure;

  Write(MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_SBYB |
                                 MPL3115A2_CTRL_REG1_OS128 |
                                 MPL3115A2_CTRL_REG1_BAR);

  uint8_t sta = 0;
  while (!(sta & MPL3115A2_REGISTER_STATUS_PDR)) {
    sta = Read(MPL3115A2_REGISTER_STATUS);
    chThdSleepMilliseconds(10);
  }

  int8_t data[3];
  i2c_->ReadBytes(address_, MPL3115A2_REGISTER_PRESSURE_MSB, (uint8_t*)data,
                  sizeof(data));

  pressure = ((data[0] << 16) | (data[1] << 8) | data[2]) >> 4;

  return float(pressure) / 4.0;
}

float MPL3115A2::GetAltitude() {
  int32_t altitude;

  Write(MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_SBYB |
                                 MPL3115A2_CTRL_REG1_OS128 |
                                 MPL3115A2_CTRL_REG1_ALT);

  uint8_t sta = 0;
  while (!(sta & MPL3115A2_REGISTER_STATUS_PDR)) {
    sta = Read(MPL3115A2_REGISTER_STATUS);
    chThdSleepMilliseconds(10);
  }

  int8_t data[3];
  i2c_->ReadBytes(address_, MPL3115A2_REGISTER_PRESSURE_MSB, (uint8_t*)data,
                  sizeof(data));

  altitude = ((data[0] << 16) | (data[1] << 8) | data[2]) >> 4;

  if (altitude & 0x80000) altitude |= 0xFFF00000;

  return float(altitude) / 16.0;
}

/* Gets the temperature in °C */
float MPL3115A2::GetTemperature() {
  int32_t temp;
  uint8_t sta = 0;
  while (!(sta & MPL3115A2_REGISTER_STATUS_TDR)) {
    sta = Read(MPL3115A2_REGISTER_STATUS);
    chThdSleepMilliseconds(10);
  }

  int8_t data[2];
  i2c_->ReadBytes(address_, MPL3115A2_REGISTER_TEMP_MSB, (uint8_t*)data,
                  sizeof(data));

  temp = ((data[0] << 8) | data[1]) >> 4;

  return float(temp) / 16.0;
}

uint8_t MPL3115A2::Read(uint8_t a) { return i2c_->ReadByte(address_, a); }

void MPL3115A2::Write(uint8_t a, uint8_t d) { i2c_->WriteByte(address_, a, d); }
};
