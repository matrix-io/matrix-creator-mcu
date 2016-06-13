#include "ch.h"
#include "hal.h"
#include "test.h"
#include "chprintf.h"
#include "atmel_adc.h"
#include "atmel_twi.h"
#include "atmel_twid.h"
#include "pio.h"
#include "pmc.h"
#include "board.h"
#include <mcuconf.h>


/** TWI clock frequency in Hz. */
#define TWCK            10000
#define BOARD_MCK       48000000



/** Slave address of Temperature Sensor. */
#define HTS221_ADDRESS   0x5F

/** Internal register within MCP9800 */
#define WHOAMI_REG       0x0F    // 0xBC
#define AV_CONF_REG      0x10    // 0x1B



#define PLL_A            0           /* PLL A */
#define PLL_B            1           /* PLL B */
/** Pin PCK2 (PA31 Peripheral B) */
const Pin pinPCK[] = PIN_PCK2;

static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {
  (void)arg;
  while (TRUE) {
    palClearPad(IOPORT3, 17);
    chThdSleepMilliseconds(50);
    palSetPad(IOPORT3, 17);
    chThdSleepMilliseconds(50);
  }
  return(0);
}

static const Pin pins[] = {
	PIN_TWD0,
	PIN_TWCK0
};

static Twid twid;


/*
 * Application entry point.
 */
int main(void) {
   int ADC_Val[4];
   unsigned short temp;
   unsigned char Tdata[2];
   halInit();
   chSysInit();
   sdStart(&SD1, NULL);  /* Activates the serial driver 1 */
   sdStart(&SD2, NULL);  /* Activates the serial driver 2 */

   // Configure PCK2 as FPGA clock


// #define PMC_MCKR_Val    0x00000001 //0x00000022      // 0x00000001   // 01 OSC_CLOCK  02 PLL_CLK  system_SAM3.c
  
   PIO_Configure(pinPCK, 1);
   PmcConfigurePllClock( PLL_A, (32 - 1), 3  ) ;
   /* If a new value for CSS field corresponds to PLL Clock, Program the PRES field first*/
//   PmcMasterClockSelection( PMC_MCKR_CSS_MAIN_CLK, PMC_MCKR_PRES_CLK_2);
   /* Then program the CSS field. */
//   PmcMasterClockSelection( PMC_MCKR_CSS_PLLA_CLK, PMC_MCKR_PRES_CLK_2 ) ;
   ConfigurePck( PMC_PCK_CSS_PLLA_CLK, PMC_PCK_PRES_CLK_2 ) ;
 
   /* ADC configuration*/
   ADC_Initialize( ADC);
   /* startup = 15:    640 periods of ADCClock
    * for prescal = 11
    *     prescal: ADCClock = MCK / ( (PRESCAL+1) * 2 ) => 48MHz / ((11+1)*2) = 2MHz
    *     ADC clock = 2 MHz
    */
   ADC_cfgFrequency( ADC, 15, 11 ); //
   ADC_check( ADC, 48000000 ); // Board Clock 48000000
   ADC->ADC_CHER = 0x00000033;  // Enable Channels 0, 1, 4, 5 
   ADC->ADC_MR |= 0x80;
   ADC_StartConversion(ADC); /* Start conversion */

    /* Configure TWI */
    PIO_Configure(pins, PIO_LISTSIZE(pins));
    PMC->PMC_WPMR = 0x504D4300; /* Disable write protect */
    PMC->PMC_PCER0 = 1 << ID_TWI0;
    PMC->PMC_WPMR = 0x504D4301; /* Enable write protect */
    TWI_ConfigureMaster(TWI0, TWCK, BOARD_MCK);
    TWID_Initialize(&twid, TWI0);



   /* Creates the blinker thread. */
   chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

   while (TRUE) {
    while( !( (ADC->ADC_ISR & ADC_ISR_EOC0) && (ADC->ADC_ISR & ADC_ISR_EOC1) && (ADC->ADC_ISR & ADC_ISR_EOC4) && (ADC->ADC_ISR & ADC_ISR_EOC5) ) );
    chThdSleepMilliseconds(500);
    ADC_Val[0] = ADC->ADC_CDR[0];
    ADC_Val[1] = ADC->ADC_CDR[1];
    ADC_Val[2] = ADC->ADC_CDR[4];
    ADC_Val[3] = ADC->ADC_CDR[5];

      TWID_Read(&twid, HTS221_ADDRESS, WHOAMI_REG, 0x01, Tdata, 0x02, 0);
      chprintf((BaseChannel *)&SD1, "I2C value:0x%X, 0x%X\n\r",Tdata[1], Tdata[0]);
      TWID_Read(&twid, HTS221_ADDRESS, AV_CONF_REG, 0x01, Tdata, 0x02, 0);
      chprintf((BaseChannel *)&SD1, "I2C value:0x%X, 0x%X\n\r",Tdata[1], Tdata[0]);
//    chprintf((BaseChannel *)&SD1, "%d, %d, %d, %d \r\n", ADC_Val[0]*3300/4096, ADC_Val[1]*3300/4096, ADC_Val[2]*3300/4096, ADC_Val[3]*3300/4096 ) ;


/*
uint8_t TWID_Read(
    Twid *pTwid,         // Pointer to a Twid instance.
    uint8_t address,     // TWI slave address.
    uint32_t iaddress,   // Optional slave internal address.
    uint8_t isize,       // Internal address size in bytes.
    uint8_t *pData,      // Data buffer for storing received bytes.
    uint32_t num,        // Number of bytes to read.
    Async *pAsync)       // Asynchronous transfer descriptor.
{
 \return 0 if the transfer has been started; otherwise returns a TWI error code.
*/


   }
   return(0);
}
