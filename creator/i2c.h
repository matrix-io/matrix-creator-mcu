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

#ifndef CPP_CREATOR_I2C_H_
#define CPP_CREATOR_I2C_H_

#include "atmel_twid.h"
#include "ch.h"
#include "chtypes.h"

namespace creator {

class I2C {
 public:
  void Init();

  void WriteByte(uint8_t address, uint8_t subAddress, uint8_t data);

  void WriteByte(uint8_t address, uint8_t data);

  uint8_t ReadByte(uint8_t address, uint8_t subAddress);
  uint8_t ReadByte(uint8_t address);
  uint8_t ReadBytes(uint8_t address, uint8_t subAddress, uint8_t* dest,
                    uint8_t count);

 private:
  Twid twid_;
};
};      // namespace creator
#endif  // CPP_CREATOR_I2C_H_
