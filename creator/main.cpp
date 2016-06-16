#include "ch.h"
#include "hal.h"
#include "test.h"
#include "chvt.h"
#include "chprintf.h"
#include "atmel_adc.h"

#include "pio.h"
#include "pmc.h"
#include "board.h"
#include "lsm9ds1.h"
#include "madgwick_ahrs.h"
#include "ch.hpp"
#include <mcuconf.h>

extern "C" {
#include "atmel_psram.h"
}

#define PLL_A 0 /* PLL A */
#define PLL_B 1 /* PLL B */

/** Pin PCK2 (PA31 Peripheral B) */
const Pin pinPCK[] = PIN_PCK2;

static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {
  (void)arg;
  while (TRUE) {
    palClearPad(IOPORT3, 17);
    chThdSleepMilliseconds(500);
    palSetPad(IOPORT3, 17);
    chThdSleepMilliseconds(1);
  }
  return (0);
}

static WORKING_AREA(waThread2, 1024);
static msg_t Thread2(void *arg) {
  (void)arg;

  float sampleFrequency = 2;

  LSM9DS1 imu(IMU_MODE_I2C, 0x6A, 0x1C);

  imu.begin();

  Madgwick imu_filer_;
  imu_filer_.begin(sampleFrequency);

  /*
    http://www.chibios.org/dokuwiki/doku.php?id=chibios:kb:timing
  */

  systime_t time = chTimeNow();  // T0
  int counter = 0;
  while (TRUE) {
    time += MS2ST(500);  // Next deadline

    chThdSleepMicroseconds(500);
    imu.readGyro();

    chThdSleepMicroseconds(500);
    imu.readMag();

    chThdSleepMicroseconds(500);
    imu.readAccel();
    /*
        imu_filer_.update(
            imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz),
            imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az),
            imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
    */
    chprintf((BaseChannel *)&SD1, "counter = %d\r", counter);
    counter++;

    chprintf((BaseChannel *)&SD1,
             "gyro: %d %d %d,  accel: %d %d %d,  mag: %d %d %d\n\r", imu.gx,
             imu.gy, imu.gz, imu.ax, imu.ay, imu.az, imu.mx, imu.my, imu.mz);

    ///    chThdSleepUntil(time);
  }
  return (0);
}

static WORKING_AREA(waThread3, 1024);
static msg_t Thread3(void *arg) {
  (void)arg;
  uint8_t *ptr = (uint8_t *)PSRAM_BASE_ADDRESS;
  uint32_t i, j;

  /* Configure EBI I/O for psram connection*/
  PIO_Configure(pinPsram, PIO_LISTSIZE(pinPsram));
  /* complete SMC configuration between PSRAM and SMC waveforms.*/
  BOARD_ConfigurePSRAM(SMC);

  for (i = 0; i < 140; i++) {
    ptr[i] = 0x01;
  }

  while (TRUE) {
    for (j = 0; j < 4; j++) {
      ptr[j] = 0x20;
      chThdSleepMilliseconds(100);
      for (i = 0; i < 35; i++) {
        chThdSleepMilliseconds(100);
        ptr[i * 4 + j] = 0x00;
        ptr[(i + 1) * 4 + j] = 0x20;
      }
      for (i = 0; i < 140; i++) {
        ptr[i] = 0x0;
      }
    }
  }
  return (0);
}

void fpgaOscInit() {
  PIO_Configure(pinPCK, 1);
  ConfigurePck(PMC_PCK_CSS_PLLA_CLK, PMC_PCK_PRES_CLK_2);
}

/*
 * Application entry point.
 */
int main(void) {
  halInit();
  chSysInit();
  fpgaOscInit();
  sdStart(&SD1, NULL); /* Activates the serial driver 1 */

  /* Creates the blinker thread. */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  /* Creates the imu thread. */
  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

  /* Creates the imu thread. */
  chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO, Thread3, NULL);

  return (0);
}
