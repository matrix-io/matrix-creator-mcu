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

/**
 * @file    AT91SAM3/pal_lld.c
 * @brief   AT91SAM3 PIO low level driver code.
 *
 * @addtogroup PAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)


#define PAL_MODE_MASK                   0x1F

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   AT91SAM3 I/O ports configuration.
 * @details PIO registers initialization.
 *
 * @param[in] config    the AT91SAM3 ports configuration
 *
 * @notapi
 */
void _pal_lld_init (const PALConfig *config) {
  uint32_t ports;
  
  ports = PIOA_CLOCK ||  /* Parallel I/O Controller A */
          PIOB_CLOCK ||  /* Parallel I/O Controller B */
          PIOC_CLOCK;    /* Parallel I/O Controller C */

#if defined(SAM3U_PLATFORM)
  PMC->PMC_PCER = ports;
#else
  PMC->PMC_WPMR = 0x504D4300; /* Disable write protect */
  PMC->PMC_PCER = ports;
  PMC->PMC_WPMR = 0x504D4301; /* Enable write protect */
#endif
  

  /*
   * PIOA setup.
   */
  PIOA->PIO_PUER      = config->P0Data.pusr;   /* Pull-up as spec.*/
  PIOA->PIO_PUDR      = ~config->P0Data.pusr;
  PIOA->PIO_PER       = 0xFFFFFFFF;            /* PIO enabled.*/
  PIOA->PIO_ODSR      = config->P0Data.odsr;   /* Data as specified.*/
  PIOA->PIO_OER       = config->P0Data.osr;    /* Dir. as specified.*/
  PIOA->PIO_ODR       = ~config->P0Data.osr;
  PIOA->PIO_IFDR      = 0xFFFFFFFF;            /* Filter disabled.*/
  PIOA->PIO_IDR       = 0xFFFFFFFF;            /* Int. disabled.*/
  PIOA->PIO_MDDR      = 0xFFFFFFFF;            /* Push Pull drive.*/
#if defined(SAM3U_PLATFORM)
  PIOA->PIO_ABSR      = 0x00000000;            /* Peripheral A.*/
#else  
  PIOA->PIO_ABCDSR[0] = 0x00000000;            /* Peripheral A.*/
  PIOA->PIO_ABCDSR[1] = 0x00000000;            /* Peripheral A.*/
#endif  
  PIOA->PIO_OWER      = 0xFFFFFFFF;            /* Write enabled.*/

  /*
   * PIOB setup.
   */
  PIOB->PIO_PUER      = config->P1Data.pusr;   /* Pull-up as spec.*/
  PIOB->PIO_PUDR      = ~config->P1Data.pusr;
  PIOB->PIO_PER       = 0xFFFFFFFF;            /* PIO enabled.*/
  PIOB->PIO_ODSR      = config->P1Data.odsr;   /* Data as specified.*/
  PIOB->PIO_OER       = config->P1Data.osr;    /* Dir. as specified.*/
  PIOB->PIO_ODR       = ~config->P1Data.osr;
  PIOB->PIO_IFDR      = 0xFFFFFFFF;            /* Filter disabled.*/
  PIOB->PIO_IDR       = 0xFFFFFFFF;            /* Int. disabled.*/
  PIOB->PIO_MDDR      = 0xFFFFFFFF;            /* Push Pull drive.*/
#if defined(SAM3U_PLATFORM)
  PIOB->PIO_ABSR      = 0x00000000;            /* Peripheral A.*/
#else  
  PIOB->PIO_ABCDSR[0] = 0x00000000;            /* Peripheral A.*/
  PIOB->PIO_ABCDSR[1] = 0x00000000;            /* Peripheral A.*/
#endif  
  PIOB->PIO_OWER      = 0xFFFFFFFF;            /* Write enabled.*/

  /*
   * PIOC setup.
   */
  PIOC->PIO_PUER      = config->P2Data.pusr;   /* Pull-up as spec.*/
  PIOC->PIO_PUDR      = ~config->P2Data.pusr;
  PIOC->PIO_PER       = 0xFFFFFFFF;            /* PIO enabled.*/
  PIOC->PIO_ODSR      = config->P2Data.odsr;   /* Data as specified.*/
  PIOC->PIO_OER       = config->P2Data.osr;    /* Dir. as specified.*/
  PIOC->PIO_ODR       = ~config->P2Data.osr;
  PIOC->PIO_IFDR      = 0xFFFFFFFF;            /* Filter disabled.*/
  PIOC->PIO_IDR       = 0xFFFFFFFF;            /* Int. disabled.*/
  PIOC->PIO_MDDR      = 0xFFFFFFFF;            /* Push Pull drive.*/
#if defined(SAM3U_PLATFORM)
  PIOC->PIO_ABSR      = 0x00000000;            /* Peripheral A.*/
#else  
  PIOC->PIO_ABCDSR[0] = 0x00000000;            /* Peripheral A.*/
  PIOC->PIO_ABCDSR[1] = 0x00000000;            /* Peripheral A.*/
#endif  
  PIOC->PIO_OWER      = 0xFFFFFFFF;            /* Write enabled.*/
}

/**
 * @brief   Pads mode setup.
 * @details This function programs a pads group belonging to the same port
 *          with the specified mode.
 * @note    This function is not meant to be invoked directly from the
 *          application code.
 * @note    @p PAL_MODE_RESET is implemented as input with pull-up.
 * @note    @p PAL_MODE_UNCONNECTED is implemented as push pull output with
 *          high state.
 * @note    @p PAL_MODE_OUTPUT_OPENDRAIN also enables the pull-up resistor.
 *
 * @param[in] port      the port identifier
 * @param[in] mask      the group mask
 * @param[in] mode      the mode
 *
 * @notapi
 */
void _pal_lld_setgroupmode(ioportid_t port,
                           ioportmask_t mask,
                           uint_fast8_t mode) {

  switch (mode & PAL_MODE_MASK) {
  case PAL_MODE_RESET:
  case PAL_MODE_INPUT_PULLUP:
    port->PIO_PUER = mask;
    port->PIO_ODR = mask;
    break;
  case PAL_MODE_INPUT:
  case PAL_MODE_INPUT_ANALOG:
    port->PIO_PUDR = mask;
    port->PIO_ODR = mask;
    break;
  case PAL_MODE_UNCONNECTED:
    port->PIO_SODR = mask;
    /* Falls in */
  case PAL_MODE_OUTPUT_PUSHPULL:
    port->PIO_PUDR = mask;
    port->PIO_OER = mask;
    port->PIO_MDDR = mask;
    break;
  case PAL_MODE_OUTPUT_OPENDRAIN:
    port->PIO_PUER = mask;
    port->PIO_OER = mask;
    port->PIO_MDER = mask;
  }
}

#endif /* HAL_USE_PAL */

/** @} */
