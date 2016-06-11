/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,2011 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

#include "ch.h"
#include "hal.h"

/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
#if HAL_USE_PAL || defined(__DOXYGEN__)
const PALConfig pal_default_config =
{
   {VAL_PIOA_ODSR, VAL_PIOA_OSR, VAL_PIOA_PUSR},
   {VAL_PIOB_ODSR, VAL_PIOB_OSR, VAL_PIOB_PUSR},
   {VAL_PIOC_ODSR, VAL_PIOC_OSR, VAL_PIOC_PUSR}
};
#endif

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init (void)
{
   /* Watchdog disabled.*/
   WDT->WDT_MR = WDT_MR_WDDIS;;
  
   at91sam3_clock_init();
}

/*
 * Board-specific initialization code.
 */
void boardInit (void)
{
  /*
   * LED pins setup.
   */
  palSetPad(IOPORT1, PIOA_LED1);
  palSetPadMode(IOPORT1, PIOA_LED1, PAL_MODE_OUTPUT_PUSHPULL);
  
  palSetPad(IOPORT2, PIOB_LED2);
  palSetPadMode(IOPORT2, PIOB_LED2, PAL_MODE_OUTPUT_PUSHPULL);
  
  palSetPad(IOPORT1, PIOA_LED3);
  palSetPadMode(IOPORT1, PIOA_LED3, PAL_MODE_OUTPUT_PUSHPULL);
  
  palClearPad(IOPORT1, PIOA_LED4);
  palSetPadMode(IOPORT1, PIOA_LED4, PAL_MODE_OUTPUT_PUSHPULL);
  
  /*
   * Buttons setup.
   */
  palSetGroupMode(IOPORT1, PIOA_SW1_MASK | PIOA_SW2_MASK, 0, PAL_MODE_INPUT_PULLUP);
}
