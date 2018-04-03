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

#ifndef CPP_CREATOR_VEML6020_H_
#define CPP_CREATOR_VEML6020_H_

#include "chtypes.h"

#include "./i2c.h"

namespace creator {

union veml6070_CFG_REG {
  uint8_t data;
  struct fields_t {
    uint8_t SD : 1;
    uint8_t reserved0 : 1;
    uint8_t IT : 2;
    uint8_t ACK_THD : 1;
    uint8_t ACK : 1;
    uint8_t reserbed1 : 1;
  };
  fields_t fields;
};

class VEML6070 {
 public:
  VEML6070(I2C* i2c);
  bool Begin();
  int GetUV();

 private:
  I2C* i2c_;
  veml6070_CFG_REG CFG_REG_;
};

};      // namespace creator
#endif  // CPP_CREATOR_VEML6020_H_
