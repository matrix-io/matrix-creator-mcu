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
#include "hal.h"
#include "board.h"
#include "lsm9ds1.h"
#include "madgwick_ahrs.h"

#include <string.h>
#include <mcuconf.h>

extern "C" {
#include "atmel_psram.h"
}

static WORKING_AREA(waBlinkThread, 128);
static msg_t BlinkThread(void *arg) {
  (void)arg;
  while (TRUE) {
    palClearPad(IOPORT3, 17);
    chThdSleepMilliseconds(500);
    palSetPad(IOPORT3, 17);
    chThdSleepMilliseconds(1);
  }
  return (0);
}

static void writeFloatToPSRAM(char *ram, int addr, float data) {
  char *p = (char *)&data;

  ram[addr] = p[1];
  ram[addr + 1] = p[0];
  ram[addr + 2] = p[3];
  ram[addr + 3] = p[2];  // swapping bytes...
}

static WORKING_AREA(waIMUThread, 1024);
static msg_t IMUThread(void *arg) {
  (void)arg;

  float sampleFrequency = 5;

  char *psram = (char *)PSRAM_BASE_ADDRESS;

  /* Configure EBI I/O for psram connection*/
  PIO_Configure(pinPsram, PIO_LISTSIZE(pinPsram));

  /* complete SMC configuration between PSRAM and SMC waveforms.*/
  BOARD_ConfigurePSRAM(SMC);

  LSM9DS1 imu(IMU_MODE_I2C, 0x6A, 0x1C);

  imu.begin();

  Madgwick imu_filer_;
  imu_filer_.begin(sampleFrequency);

  /*
    http://www.chibios.org/dokuwiki/doku.php?id=chibios:kb:timing
  */

  systime_t time = chTimeNow();  // T0

  while (TRUE) {
    time += MS2ST(200);  // Next deadline

    chThdSleepMicroseconds(1);
    imu.readGyro();

    chThdSleepMicroseconds(1);
    imu.readMag();

    chThdSleepMicroseconds(1);
    imu.readAccel();

    imu_filer_.update(
        imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz),
        imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az),
        imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));

    writeFloatToPSRAM(psram, 0, imu_filer_.getRoll());
    writeFloatToPSRAM(psram, 4, imu_filer_.getYaw());
    writeFloatToPSRAM(psram, 8, imu_filer_.getPitch());

    chThdSleepUntil(time);
  }
  return (0);
}

/*
 * Application entry point.
 */
int main(void) {

  halInit();

  chSysInit();

  /* Creates the blinker thread. */
  chThdCreateStatic(waBlinkThread, sizeof(waBlinkThread), NORMALPRIO,
                    BlinkThread, NULL);

  /* Creates the imu thread. */
  chThdCreateStatic(waIMUThread, sizeof(waIMUThread), NORMALPRIO, IMUThread,
                    NULL);

  return (0);
}
