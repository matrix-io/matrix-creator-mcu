#ifndef _PSRAM_H_
#define _PSRAM_H_

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/
/** Base address of chip select */
#define PSRAM_BASE_ADDRESS         (0x63000000)      //NCS3

/** 8-bits Data bus on EBI */
#define PIN_DATA_BUS_ON_EBI         {0x000000ff, PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}
/** Read Signal (output) */
#define PIN_NRD_ON_EBI              {1 << 11, PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}
/** Write Enable Signal (output) */
#define PIN_NWE_ON_EBI              {1 << 8,  PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}
/** Static Memory Controller Chip Select Lines */
#define PIN_NCS3_ON_EBI             {1 << 12, PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}
/** Address Bus A0-A13 */
#define PIN_ADDR_BUS_ON_EBI         {0xfffc0000, PIOC, ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** PIOs configuration for EBI connect to Psram. */
const Pin pinPsram[] = {PIN_DATA_BUS_ON_EBI, \
                        PIN_NRD_ON_EBI, \
                        PIN_NWE_ON_EBI, \
                        PIN_NCS3_ON_EBI,\
                        PIN_ADDR_BUS_ON_EBI
                        };



extern void BOARD_ConfigurePSRAM( Smc* pSmc );


#endif /* #ifndef _PSRAM_ */
