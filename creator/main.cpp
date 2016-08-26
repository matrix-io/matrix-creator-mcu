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

#include <math.h>
#include <string.h>
#include <mcuconf.h>

#include "./i2c.h"
#include "./sensors_data.h"
#include "./mpl3115a2.h"
#include "./lsm9ds1.h"
#include "./hts221.h"
#include "./veml6070.h"

extern "C" {
#include "atmel_psram.h"
}

/* Global objects */
creator::I2C i2c;  // TODO(andres.calderon@admobilize.com): avoid global objects

void psram_copy(uint8_t mem_offset, char *data, uint8_t len) {
  register char *psram = (char *)PSRAM_BASE_ADDRESS;

  for (int i = 0; i < len; i++) {
    psram[mem_offset + i] = data[i];
  }
}

static WORKING_AREA(waEnvThread, 256);
static msg_t EnvThread(void *arg) {
  (void)arg;

  creator::HTS221 hts221(&i2c);
  creator::MPL3115A2 mpl3115a2(&i2c);
  creator::VEML6070 veml6070(&i2c);

  hts221.Begin();
  mpl3115a2.Begin();
  veml6070.Begin();

  HumidityData hum;
  PressureData press;
  UVData uv;

  while (true) {
    palSetPad(IOPORT3, 17);
    chThdSleepMilliseconds(1);
    palClearPad(IOPORT3, 17);

    hts221.GetData(hum.humidity, hum.temperature);
    psram_copy(mem_offset_humidity, (char *)&hum, sizeof(hum));

    press.altitude = mpl3115a2.GetAltitude();
    press.pressure = mpl3115a2.GetPressure();
    press.temperature = mpl3115a2.GetTemperature();
    psram_copy(mem_offset_press, (char *)&press, sizeof(press));

    uv.UV = veml6070.GetUV();
    psram_copy(mem_offset_uv, (char *)&uv, sizeof(uv));
  }
  return (0);
}

static WORKING_AREA(waIMUThread, 512);
static msg_t IMUThread(void *arg) {
  (void)arg;

  LSM9DS1 imu(&i2c, IMU_MODE_I2C, 0x6A, 0x1C);

  imu.begin();

  IMUData data;

  while (true) {
    imu.readGyro();
    data.gyro_x = imu.calcGyro(imu.gx);
    data.gyro_y = imu.calcGyro(imu.gy);
    data.gyro_z = imu.calcGyro(imu.gz);

    imu.readMag();
    data.mag_x = imu.calcMag(imu.mx);
    data.mag_y = imu.calcMag(imu.my);
    data.mag_z = imu.calcMag(imu.mz);

    imu.readAccel();
    data.accel_x = imu.calcAccel(imu.ax);
    data.accel_y = imu.calcAccel(imu.ay);
    data.accel_z = imu.calcAccel(imu.az);

    data.yaw = atan2(data.mag_y, -data.mag_x) * 180.0 / M_PI;
    data.roll = atan2(data.accel_y, data.accel_z) * 180.0 / M_PI;
    data.pitch = atan2(-data.accel_x, sqrt(data.accel_y * data.accel_y +
                                           data.accel_z * data.accel_z)) *
                 180.0 / M_PI;

    psram_copy(mem_offset_imu, (char *)&data, sizeof(data));

    chThdSleepMilliseconds(20);
  }
  return (0);
}

/*
 * Application entry point.
 */
int main(void) {
  halInit();

  chSysInit();
  sdStart(&SD1, NULL);
  /* Configure EBI I/O for psram connection*/
  PIO_Configure(pinPsram, PIO_LISTSIZE(pinPsram));

  /* complete SMC configuration between PSRAM and SMC waveforms.*/
  BOARD_ConfigurePSRAM(SMC);

  i2c.Init();
  /* Creates the imu thread. */
  chThdCreateStatic(waIMUThread, sizeof(waIMUThread), NORMALPRIO, IMUThread,
                    NULL);
  /* Creates the Env thread. */
  chThdCreateStatic(waEnvThread, sizeof(waEnvThread), NORMALPRIO, EnvThread,
                    NULL);

  return (0);
}
