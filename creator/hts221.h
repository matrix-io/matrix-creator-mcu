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

#ifndef CPP_CREATOR_HTS221_H_
#define CPP_CREATOR_HTS221_H_

#include "atmel_twid.h"
#include "chtypes.h"

#include "./i2c.h"

namespace creator {

class HTS221 {
 public:
  HTS221(I2C* i2c, uint8_t address = 0x5F);
  bool Begin();
  void GetData(int& humidity, int& temperature);

 private:
  void Write(uint8_t a, uint8_t d);
  uint8_t Read(uint8_t a);
  uint8_t Read(uint8_t a, uint8_t* data, uint8_t size);

  uint8_t mode_;
  I2C* i2c_;
  uint8_t address_;
  float H_RATIO, T_RATIO, H_OFFSET, T_OFFSET;
  int32_t H0_rH_x2, H0_T0_OUT, T0_degC_x8, T0_OUT;
  int32_t H1_rH_x2, H1_T0_OUT, T1_degC_x8, T1_OUT;
};

};      // namespace creator
#endif  // CPP_CREATOR_I2C_H_
