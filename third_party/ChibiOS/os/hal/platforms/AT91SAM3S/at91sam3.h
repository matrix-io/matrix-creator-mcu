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

#ifndef _AT91SAM3_H_
#define _AT91SAM3_H_

/*
 * Supported platforms.
 */
#define SAM3S1A         0
#define SAM3S1B         1
#define SAM3S1C         2
#define SAM3S2A         3
#define SAM3S2B         4
#define SAM3S2C         5
#define SAM3S4A         6
#define SAM3S4B         7
#define SAM3S4C         8
 
#define SAM3U1C         9
#define SAM3U1E         10
#define SAM3U2C         11
#define SAM3U2E         12
#define SAM3U4C         13
#define SAM3U4E         14

#define SAM3N1A         15
#define SAM3N1B         16
#define SAM3N1C         17
#define SAM3N2A         18
#define SAM3N2B         19
#define SAM3N2C         20
#define SAM3N4A         21
#define SAM3N4B         22
#define SAM3N4C         23



#ifndef SAM3_PLATFORM
#error "SAM3 platform not defined"
#endif

#if   (SAM3_PLATFORM == SAM3N4C)
#define sam3n4
#include "at91lib/SAM3N.h"
#elif (SAM3_PLATFORM == SAM3S4C)
#define sam3s4
#include "at91lib/SAM3S.h"
#elif (SAM3_PLATFORM == SAM3U4E)
#include "at91lib/SAM3U.h"
#else
#error "SAM3 platform not supported"
#endif


/*
 * SAM3N devices
 */
#if ((SAM3_PLATFORM == SAM3N1A) || (SAM3_PLATFORM == SAM3N1B) || (SAM3_PLATFORM == SAM3N1C) || \
     (SAM3_PLATFORM == SAM3N2A) || (SAM3_PLATFORM == SAM3N2B) || (SAM3_PLATFORM == SAM3N2C) || \
     (SAM3_PLATFORM == SAM3N4A) || (SAM3_PLATFORM == SAM3N4B) || (SAM3_PLATFORM == SAM3N4C) )
     
#define SAM3N_PLATFORM

#define UART0_CLOCK        (1<<ID_UART0)
#define UART1_CLOCK        (1<<ID_UART1)
#define PIOA_CLOCK         (1<<ID_PIOA)
#define PIOB_CLOCK         (1<<ID_PIOB)
#define PIOC_CLOCK         (1<<ID_PIOC)
#define ADC_CLOCK          (1<<ID_ADC)
#define PWM_CLOCK          (1<<ID_PWM)
#define USART0_CLOCK       (1<<ID_USART0)
#define TWI0_CLOCK         (1<<ID_TWI0)

#endif  

/*
 * SAM3S devices
 */
#if ((SAM3_PLATFORM == SAM3S1A) || (SAM3_PLATFORM == SAM3S1B) || (SAM3_PLATFORM == SAM3S1C) || \
     (SAM3_PLATFORM == SAM3S2A) || (SAM3_PLATFORM == SAM3S2B) || (SAM3_PLATFORM == SAM3S2C) || \
     (SAM3_PLATFORM == SAM3S4A) || (SAM3_PLATFORM == SAM3S4B) || (SAM3_PLATFORM == SAM3S4C) )
     
#define SAM3S_PLATFORM

#define CKGR_PLLR          CKGR_PLLAR
#define PMC_PCER           PMC_PCER0
#define PMC_PCDR           PMC_PCDR0

#define UART0_CLOCK        (1<<ID_UART0)
#define PIOA_CLOCK         (1<<ID_PIOA)
#define PIOB_CLOCK         (1<<ID_PIOB)
#define PIOC_CLOCK         (1<<ID_PIOC)
#define ADC_CLOCK          (1<<ID_ADC)
#define UART1_CLOCK        (1<<ID_UART1)

#endif 

/*
 * SAM3U devices
 */
#if ((SAM3_PLATFORM == SAM3U1C) || (SAM3_PLATFORM == SAM3U1E) || \
     (SAM3_PLATFORM == SAM3U2C) || (SAM3_PLATFORM == SAM3U2E) || \
     (SAM3_PLATFORM == SAM3U4C) || (SAM3_PLATFORM == SAM3U4E) )
     
#define SAM3U_PLATFORM

#define CKGR_MOR              PMC_MOR
#define CKGR_PLLR             PMC_PLLAR
#define EFC                   EFC0

#define UART0_CLOCK           (1<<ID_UART)
#define PIOA_CLOCK            (1<<ID_PIOA)
#define PIOB_CLOCK            (1<<ID_PIOB)
#define PIOC_CLOCK            (1<<ID_PIOC)

#define US_MR_CHMODE_NORMAL   UART_MR_CHMODE_NORMAL
#define US_MR_PAR_NO          UART_MR_PAR_NO
     
#endif 


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */
#include "core_cm3.h"         /* Cortex-M3 processor and core peripherals */
#include "system_SAM3.h"     /* System Header                            */
   
#endif /* _AT91SAM3_H_ */
