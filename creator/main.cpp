#include "ch.h"
#include "hal.h"
#include "test.h"
#include "chprintf.h"
#include "atmel_adc.h"
#include "pio.h"
#include "pmc.h"
#include "board.h"
#include "lsm9ds1.h"
#include <mcuconf.h>

#define PLL_A 0 /* PLL A */
#define PLL_B 1 /* PLL B */

/** Pin PCK2 (PA31 Peripheral B) */
const Pin pinPCK[] = PIN_PCK2;

static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {
  (void)arg;
  while (TRUE) {
    palClearPad(IOPORT3, 17);
    chThdSleepMilliseconds(1000);
    palSetPad(IOPORT3, 17);
    chThdSleepMilliseconds(5);
  }
  return (0);
}

static WORKING_AREA(waThread2, 1024);
static msg_t Thread2(void *arg) {
  (void)arg;
  LSM9DS1 imu(IMU_MODE_I2C, 0x6A, 0x1C);

  imu.begin();
  while (TRUE) {
    chThdSleepMilliseconds(1);
    imu.readGyro();
    
    chThdSleepMilliseconds(1);
    imu.readMag();

    chThdSleepMilliseconds(1);
    imu.readAccel();

    chprintf((BaseChannel *)&SD1,
             "gyro: %d %d %d,  accel: %d %d %d,  mag: %d %d %d\n\r", imu.gx,
             imu.gy, imu.gz, imu.ax, imu.ay, imu.az, imu.mx, imu.my, imu.mz);
  }
  return (0);
}

/*
 * Application entry point.
 */
int main(void) {
  halInit();
  chSysInit();
  sdStart(&SD1, NULL); /* Activates the serial driver 1 */

  // Configure PCK2 as FPGA clock
  PIO_Configure(pinPCK, 1);
  PmcConfigurePllClock(PLL_A, (32 - 1), 3);

  ConfigurePck(PMC_PCK_CSS_PLLA_CLK, PMC_PCK_PRES_CLK_2);

  /* Creates the blinker thread. */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /* Creates the imu thread. */
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

  return (0);
}
