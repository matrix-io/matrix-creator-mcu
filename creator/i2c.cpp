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

#include "atmel_twi.h"
#include "pio.h"

#include "./i2c.h"

namespace creator {

/** TWI clock frequency in Hz. */
#define TWCK 400000

#define BOARD_MCK 64000000

static const Pin pins[] = {PIN_TWD0, PIN_TWCK0};

void I2C::Init() {
  /* Configure TWI */
  chSysLock();
  PIO_Configure(pins, PIO_LISTSIZE(pins));
  PMC->PMC_WPMR = 0x504D4300; /* Disable write protect */
  PMC->PMC_PCER0 = 1 << ID_TWI0;
  PMC->PMC_WPMR = 0x504D4301; /* Enable write protect */
  TWI_ConfigureMaster(TWI0, TWCK, BOARD_MCK);
  TWID_Initialize(&twid_, TWI0);
  chSysUnlock();
}

void I2C::WriteByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  // TODO (andres.calderon): Handle timeouts, handle errors
  chSysLock();
  TWID_Write(&twid_, address, subAddress, 1, &data, 1, 0);
  chSysUnlock();
}

void I2C::WriteByte(uint8_t address, uint8_t data) {
  // TODO (andres.calderon): Handle timeouts, handle errors
  chSysLock();
  TWID_Write(&twid_, address, 0, 0, &data, 1, 0);
  chSysUnlock();
}

uint8_t I2C::ReadByte(uint8_t address, uint8_t subAddress) {
  // TODO (andres.calderon): Handle timeouts, handle errors
  chSysLock();
  uint8_t ret;
  uint8_t data;

  if (TWID_Read(&twid_, address, subAddress, 1, &data, 1, 0) == 0) ret = data;
  chSysUnlock();
  return ret;
}

uint8_t I2C::ReadByte(uint8_t address) {
  // TODO (andres.calderon): Handle timeouts, handle errors
  chSysLock();
  uint8_t ret;
  uint8_t data;

  if (TWID_Read(&twid_, address, 0, 0, &data, 1, 0) == 0) ret = data;
  chSysUnlock();
  return ret;
}

uint8_t I2C::ReadBytes(uint8_t address, uint8_t subAddress, uint8_t* dest,
                       uint8_t count) {
  chSysLock();
  uint8_t ret = 0;
  // TODO (andres.calderon): Handle timeouts, handle errors
  if (TWID_Read(&twid_, address, subAddress, 1, dest, count, 0) == 0) {
    ret = count;
  }
  chSysUnlock();
  return ret;
}

};  // namespace creator
