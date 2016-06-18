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
#include <string.h>
#include <mcuconf.h>

extern "C" {
#include "atmel_psram.h"
}

#define PLL_A 0 /* PLL A */
#define PLL_B 1 /* PLL B */

/** Pin PCK2 (PA31 Peripheral B) */
const Pin pinPCK[] = PIN_PCK2;

static int needle_south = 0;
static int needle_north = 0;
unsigned char data_rx;

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

  float sampleFrequency = 5;

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

    //chprintf((BaseChannel *)&SD1, "counter = %d\r", counter);
    counter++;

    if (imu_filer_.getRoll() < -20 || imu_filer_.getRoll() > 20) {
      needle_south = (int)(imu_filer_.getPitch() / 10);
      needle_north = needle_south > 18 ? needle_south - 18 : needle_south + 17;
    } else {
      needle_south = (int)(imu_filer_.getYaw() / 10);
      needle_north = needle_south > 18 ? needle_south - 18 : needle_south + 17;
    }

    if (0)
      chprintf((BaseChannel *)&SD1,
               "gyro: %d %d %d,  accel: %d %d %d,  mag: %d %d %d\n\r", imu.gx,
               imu.gy, imu.gz, imu.ax, imu.ay, imu.az, imu.mx, imu.my, imu.mz);

    needle_south = (int)(imu_filer_.getYaw() / 10);
    needle_north = needle_south > 18 ? needle_south - 18 : needle_south + 17;

    if (0)
      chprintf((BaseChannel *)&SD1, "roll: %d,  pitch: %d,  yaw: %d\n\r",
               (int)imu_filer_.getRoll(), (int)imu_filer_.getPitch(),
               (int)imu_filer_.getYaw());

    chThdSleepUntil(time);
  }
  return (0);
}

typedef struct LedRGBW {
  uint8_t g;
  uint8_t r;
  uint8_t b;
  uint8_t w;
} LedRGBW_;

typedef struct RGB332 {
  uint8_t r:3;
  uint8_t g:3;
  uint8_t b:2;
} RGB332_;


static WORKING_AREA(waThread3, 1024);
static msg_t Thread3(void *arg) {
  (void)arg;
  LedRGBW *leds = (LedRGBW *)PSRAM_BASE_ADDRESS;
  int led;

  /* Configure EBI I/O for psram connection*/
  PIO_Configure(pinPsram, PIO_LISTSIZE(pinPsram));
  /* complete SMC configuration between PSRAM and SMC waveforms.*/
  BOARD_ConfigurePSRAM(SMC);

  systime_t time = chTimeNow();
  
  RGB332* rgb332 = (RGB332*)&data_rx;
  while (TRUE) {
    time += MS2ST(200);  // Next deadline

    for (led = 0; led < 35; led++) {
      leds[led].r = rgb332->r<<2;
      leds[led].g = rgb332->g<<2;
      leds[led].b = rgb332->b<<2;
      leds[led].w = 0;
    }

    chThdSleepUntil(time);
  }

  return (0);
}

static WORKING_AREA(waThread4, 128);
static msg_t Thread4(void *arg) {
  (void)arg;
  while (TRUE) {
    data_rx = chIOGet(&SD1);
    chprintf((BaseChannel *)&SD1, "data:%d",data_rx);
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
  /* Creates Serial get thread. */
  chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO, Thread4, NULL);

  return (0);
}
