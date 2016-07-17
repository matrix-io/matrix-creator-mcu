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
#include "sensors_data.h"

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

static void Write4bytesToPSRAM(const char *src, char *ram) {
  ram[0] = src[1];
  ram[1] = src[0];
  ram[2] = src[3];
  ram[3] = src[2];  // swapping bytes...
}

static void WriteToPSRAM(const char *src, char *ram, int len) {
  for (int offset = 0; offset < len; offset = offset + 4) {
    Write4bytesToPSRAM((const char *)(&src[offset]), (char *)(&ram[offset]));
  }
}


static WORKING_AREA(waIMUThread, 1024);
static msg_t IMUThread(void *arg) {
  (void)arg;

  float sampleFrequency =
      5;  // TODO (andres.calderon): Improve the sampleFrequency

  register char *psram = (char *)PSRAM_BASE_ADDRESS;

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

  IMUData data;
  while (TRUE) {
    time += MS2ST(200);  // Next deadline

    chThdSleepMicroseconds(1);
    imu.readGyro();
    data.gyro_x = imu.calcGyro(imu.gx);
    data.gyro_y = imu.calcGyro(imu.gy);
    data.gyro_z = imu.calcGyro(imu.gz);

    chThdSleepMicroseconds(1);
    imu.readMag();
    data.mag_x = imu.calcMag(imu.mx);
    data.mag_y = imu.calcMag(imu.my);
    data.mag_z = imu.calcMag(imu.mz);

    chThdSleepMicroseconds(1);
    imu.readAccel();
    data.accel_x = imu.calcMag(imu.ax);
    data.accel_y = imu.calcMag(imu.ay);
    data.accel_z = imu.calcMag(imu.az);

    imu_filer_.update(data.gyro_x, data.gyro_y, data.gyro_z,
                      data.accel_x, data.accel_y, data.accel_z,
                      data.mag_x, data.mag_y, data.mag_z);

    data.yaw = imu_filer_.getYaw();
    data.pitch = imu_filer_.getPitch();
    data.roll = imu_filer_.getRoll();

    WriteToPSRAM((const char *)&data, &psram[mem_offset_imu], sizeof(data));

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
