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

#ifndef CPP_CREATOR_MPL3115A2_H_
#define CPP_CREATOR_MPL3115A2_H_

#include "atmel_twid.h"
#include "chtypes.h"

#include "./i2c.h"

namespace creator {

union mpl3115a2_CTRL_REG1 {
  uint8_t data;
  struct fields_t {
    uint8_t SBYB : 1;
    uint8_t OST : 1;
    uint8_t RST : 1;
    uint8_t OS : 3;
    uint8_t RAW : 1;
    uint8_t ALT : 1;
  };

  fields_t fields;
};

union mpl3115a2_PT_DATA_CFG {
  uint8_t data;
  struct fields_t {
    uint8_t TDEFE : 1;
    uint8_t PDEFE : 1;
    uint8_t DREM : 1;
  };
  fields_t fields;
};

union mpl3115a2_DR_STATUS {
  uint8_t data;
  struct fields_t {
    uint8_t reserved0 : 1;
    uint8_t TDR : 1;
    uint8_t PDR : 1;
    uint8_t PTDR : 1;
    uint8_t reserved1 : 1;
    uint8_t TOW : 1;
    uint8_t POW : 1;
    uint8_t PTOW : 1;
  };
  fields_t fields;
};

class MPL3115A2 {
 public:
  MPL3115A2(I2C* i2c, uint8_t address = 0x60);
  bool Begin();
  int GetPressure();
  int GetAltitude();
  int GetTemperature();

 private:
  void Write(uint8_t a, uint8_t d);
  uint8_t Read(uint8_t a);
  I2C* i2c_;
  uint8_t address_;
  mpl3115a2_CTRL_REG1 CTRL_REG1_;
  mpl3115a2_PT_DATA_CFG PT_DATA_CFG_;
  mpl3115a2_DR_STATUS DR_STATUS_;
};

};      // namespace creator
#endif  // CPP_CREATOR_I2C_H_
