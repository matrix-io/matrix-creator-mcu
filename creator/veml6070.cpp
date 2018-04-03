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

#include "./veml6070.h"
#include "ch.h"
#include "hal.h"

const uint8_t VEML6070_ADDR_1 = 0x38;
const uint8_t VEML6070_ADDR_2 = 0x39;

namespace creator {

VEML6070::VEML6070(I2C* i2c) : i2c_(i2c) { CFG_REG_.data = 0x00; }

bool VEML6070::Begin() {
  CFG_REG_.fields.reserved0 = 1;
  CFG_REG_.fields.ACK = 0;
  CFG_REG_.fields.IT = 3; /* IT=4T */
  i2c_->WriteByte(VEML6070_ADDR_1, CFG_REG_.data);
  return true;
}

int VEML6070::GetUV() {
  /*
    RSET = 270 k ohms, IT=4T
    Pag 5: http://www.vishay.com/docs/84310/designingveml6070.pdf
  */

  return float((i2c_->ReadByte(VEML6070_ADDR_2) << 8) |
               i2c_->ReadByte(VEML6070_ADDR_1));
}
};  // namespace creator
