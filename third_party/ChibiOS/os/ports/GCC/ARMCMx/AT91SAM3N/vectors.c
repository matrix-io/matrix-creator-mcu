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
 * @file    GCC/ARMCMx/AT91SAM3N/vectors.c
 * @brief   Interrupt vectors for the AT91SAM3N family.
 *
 * @defgroup ARMCMx_AT91SAM3N_VECTORS AT91SAM3N Interrupt Vectors
 * @ingroup ARMCMx_SPECIFIC
 * @details Interrupt vectors for the AT91SAM3N family.
 * @{
 */

#include "ch.h"

#if !defined(__DOXYGEN__)
extern void __ram_end__(void);
extern void ResetHandler(void);
extern void NMIVector(void);
extern void HardFaultVector(void);
extern void MemManageVector(void);
extern void BusFaultVector(void);
extern void UsageFaultVector(void);
extern void Vector1C(void);
extern void Vector20(void);
extern void Vector24(void);
extern void Vector28(void);
extern void SVCallVector(void);
extern void DebugMonitorVector(void);
extern void Vector34(void);
extern void PendSVVector(void);
extern void SysTickVector(void);
extern void Vector40(void);      /* 0  Supply Controller */
extern void Vector44(void);      /* 1  Reset Controller */
extern void Vector48(void);      /* 2  Real Time Clock */
extern void Vector4C(void);      /* 3  Real Time Timer */
extern void Vector50(void);      /* 4  Watchdog Timer */
extern void Vector54(void);      /* 5  PMC */
extern void Vector58(void);      /* 6  EEFC */
extern void Vector5C(void);      /* 7  Reserved */
extern void Vector60(void);      /* 8  UART0 */
extern void Vector64(void);      /* 9  UART1 */
extern void Vector68(void);      /* 10 Reserved */
extern void Vector6C(void);      /* 11 Parallel IO Controller A */
extern void Vector70(void);      /* 12 Parallel IO Controller B */
extern void Vector74(void);      /* 13 Parallel IO Controller C */
extern void Vector78(void);      /* 14 USART 0 */
extern void Vector7C(void);      /* 15 USART 1 */
extern void Vector80(void);      /* 16 Reserved */
extern void Vector84(void);      /* 17 Reserved */
extern void Vector88(void);      /* 18 Reserved */
extern void Vector8C(void);      /* 19 TWI 0 */
extern void Vector90(void);      /* 20 TWI 1 */
extern void Vector94(void);      /* 21 SPI */
extern void Vector98(void);      /* 22 Reserved */
extern void Vector9C(void);      /* 23 Timer Counter 0 */
extern void VectorA0(void);      /* 24 Timer Counter 1 */
extern void VectorA4(void);      /* 25 Timer Counter 2 */
extern void VectorA8(void);      /* 26 Timer Counter 3 */
extern void VectorAC(void);      /* 27 Timer Counter 4 */
extern void VectorB0(void);      /* 28 Timer Counter 5 */
extern void VectorB4(void);      /* 29 ADC controller */
extern void VectorB8(void);      /* 30 DAC controller */
extern void VectorBC(void);      /* 31 PWM */
#endif

/**
 * @brief   AT91SAM3N vectors table.
 */
#if !defined(__DOXYGEN__)
__attribute__ ((section("vectors")))
#endif
void  (*_vectors[])(void) = {
  __ram_end__,        ResetHandler,       NMIVector,          HardFaultVector,
  MemManageVector,    BusFaultVector,     UsageFaultVector,   Vector1C,
  Vector20,           Vector24,           Vector28,           SVCallVector,
  DebugMonitorVector, Vector34,           PendSVVector,       SysTickVector,
  Vector40,           Vector44,           Vector48,           Vector4C,
  Vector50,           Vector54,           Vector58,           Vector5C,
  Vector60,           Vector64,           Vector68,           Vector6C,
  Vector70,           Vector74,           Vector78,           Vector7C,
  Vector80,           Vector84,           Vector88,           Vector8C,
  Vector90,           Vector94,           Vector98,           Vector9C,
  VectorA0,           VectorA4,           VectorA8,           VectorAC,
  VectorB0,           VectorB4,           VectorB8,           VectorBC
};

/**
 * @brief   Unhandled exceptions handler.
 * @details Any undefined exception vector points to this function by default.
 *          This function simply stops the system into an infinite loop.
 *
 * @notapi
 */
#if !defined(__DOXYGEN__)
__attribute__ ((naked))
#endif
void _unhandled_exception(void) {

  asm volatile (
    ".weak NMIVector            \nNMIVector:                \n\t"
    ".weak HardFaultVector      \nHardFaultVector:          \n\t"
    ".weak MemManageVector      \nMemManageVector:          \n\t"
    ".weak BusFaultVector       \nBusFaultVector:           \n\t"
    ".weak UsageFaultVector     \nUsageFaultVector:         \n\t"
    ".weak Vector1C             \nVector1C:                 \n\t"
    ".weak Vector20             \nVector20:                 \n\t"
    ".weak Vector24             \nVector24:                 \n\t"
    ".weak Vector28             \nVector28:                 \n\t"
    ".weak SVCallVector         \nSVCallVector:             \n\t"
    ".weak DebugMonitorVector   \nDebugMonitorVector:       \n\t"
    ".weak Vector34             \nVector34:                 \n\t"
    ".weak PendSVVector         \nPendSVVector:             \n\t"
    ".weak SysTickVector        \nSysTickVector:            \n\t"
    ".weak Vector40             \nVector40:                 \n\t"
    ".weak Vector44             \nVector44:                 \n\t"
    ".weak Vector48             \nVector48:                 \n\t"
    ".weak Vector4C             \nVector4C:                 \n\t"
    ".weak Vector50             \nVector50:                 \n\t"
    ".weak Vector54             \nVector54:                 \n\t"
    ".weak Vector58             \nVector58:                 \n\t"
    ".weak Vector5C             \nVector5C:                 \n\t"
    ".weak Vector60             \nVector60:                 \n\t"
    ".weak Vector64             \nVector64:                 \n\t"
    ".weak Vector68             \nVector68:                 \n\t"
    ".weak Vector6C             \nVector6C:                 \n\t"
    ".weak Vector70             \nVector70:                 \n\t"
    ".weak Vector74             \nVector74:                 \n\t"
    ".weak Vector78             \nVector78:                 \n\t"
    ".weak Vector7C             \nVector7C:                 \n\t"
    ".weak Vector80             \nVector80:                 \n\t"
    ".weak Vector84             \nVector84:                 \n\t"
    ".weak Vector88             \nVector88:                 \n\t"
    ".weak Vector8C             \nVector8C:                 \n\t"
    ".weak Vector90             \nVector90:                 \n\t"
    ".weak Vector94             \nVector94:                 \n\t"
    ".weak Vector98             \nVector98:                 \n\t"
    ".weak Vector9C             \nVector9C:                 \n\t"
    ".weak VectorA0             \nVectorA0:                 \n\t"
    ".weak VectorA4             \nVectorA4:                 \n\t"
    ".weak VectorA8             \nVectorA8:                 \n\t"
    ".weak VectorAC             \nVectorAC:                 \n\t"
    ".weak VectorB0             \nVectorB0:                 \n\t"
    ".weak VectorB4             \nVectorB4:                 \n\t"
    ".weak VectorB8             \nVectorB8:                 \n\t"
    ".weak VectorBC             \nVectorBC:                 \n\t"
  );

  while (TRUE)
    ;
}

/** @} */
