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

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for the Atmel SAM3U-EK development board.
 */

/*
 * Board identifier.
 */
#define BOARD_ATMEL_SAM3U_EK
#define BOARD_NAME      "Atmel SAM3U-EK (SAM3U4E)"

/*
 * Select your platform by modifying the following line.
 */
#if !defined(SAM3_PLATFORM)
#define SAM3_PLATFORM   SAM3U4E
#endif

#include "at91sam3.h"

/*
 * Initial I/O setup.
 */
#define VAL_PIOA_ODSR   0x00000000      /* Output data. */
#define VAL_PIOA_OSR    0x00000000      /* Direction. */
#define VAL_PIOA_PUSR   0xFFFFFFFF      /* Pull-up. */

#define VAL_PIOB_ODSR   0x00000000      /* Output data. */
#define VAL_PIOB_OSR    0x00000000      /* Direction. */
#define VAL_PIOB_PUSR   0xFFFFFFFF      /* Pull-up. */

#define VAL_PIOC_ODSR   0x00000000      /* Output data. */
#define VAL_PIOC_OSR    0x00000000      /* Direction. */
#define VAL_PIOC_PUSR   0xFFFFFFFF      /* Pull-up. */

/*
 * I/O definitions.
 */
#define PIOB_LED2       0
#define PIOB_LED2_MASK  (1 << PIOB_LED2_MASK)
#define PIOB_LED3       1
#define PIOB_LED3_MASK  (1 << PIOB_LED3_MASK)
#define PIOB_LED4       2
#define PIOB_LED4_MASK  (1 << PIOB_LED4_MASK)

#define PIOA_SW1        18
#define PIOA_SW1_MASK   (1 << PIOA_SW1)
#define PIOA_SW2        19
#define PIOA_SW2_MASK   (1 << PIOA_SW2)

/*
 * UART0 definitions.
 */
#define SAM3_UART0_RX   PIO_PA11
#define SAM3_UART0_TX   PIO_PA12



#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
