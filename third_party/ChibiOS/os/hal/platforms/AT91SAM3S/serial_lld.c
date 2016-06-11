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
 * @file    AT91SAM3/serial_lld.c
 * @brief   AT91SAM3 low level serial driver code.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "ch.h"
#include "hal.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief UART0 serial driver identifier.*/
#if USE_SAM3_UART0 || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

#if USE_SAM3_UART1 || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {
  SERIAL_DEFAULT_BITRATE,
  US_MR_CHMODE_NORMAL | US_MR_PAR_NO | US_MR_NBSTOP_2_BIT
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   UART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void uart_init(SerialDriver *sdp, const SerialConfig *config) {
  Uart* u = sdp->uart;
  uint32_t CPUFrequency;
  
  /* Update SystemCoreClock */
  SystemCoreClockUpdate();
  CPUFrequency = SystemCoreClock;

  /* Disables IRQ sources and stop operations.*/
  u->UART_IDR = 0xFFFFFFFF;
  u->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RSTSTA;

  /* New parameters setup.*/
  u->UART_BRGR = CPUFrequency / (config->sc_speed * 16);
  u->UART_MR = config->sc_mr;

  /* Enables operations and IRQ sources.*/
  u->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
  u->UART_IER = UART_IER_RXRDY | UART_IER_TXRDY | UART_IER_OVRE | UART_IER_FRAME | UART_IER_PARE;
}

/**
 * @brief   UART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] u         pointer to an UART I/O block
 */
static void uart_deinit(Uart *u) {

  /* Disables IRQ sources and stop operations.*/
  u->UART_IDR = 0xFFFFFFFF;
  u->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX | UART_CR_RSTSTA;
  u->UART_MR = 0;
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] sr        UART SR register value
 */
static void set_error(SerialDriver *sdp, uint16_t sr) {
  ioflags_t sts = 0;

  if (sr & UART_SR_OVRE)
    sts |= SD_OVERRUN_ERROR;
  if (sr & UART_SR_PARE)
    sts |= SD_PARITY_ERROR;
  if (sr & UART_SR_FRAME)
    sts |= SD_FRAMING_ERROR;
  chSysLockFromIsr();
  chIOAddFlagsI(sdp, sts);
  chSysUnlockFromIsr();
}

/**
 * @brief   Common IRQ handler.
 *
 * @param[in] sdp       communication channel associated to the UART
 */
static void serve_interrupt(SerialDriver *sdp) {
  uint32_t sr;
  Uart* u = sdp->uart;

  /* get status */
  sr = u->UART_SR;

  /* Handle RX character if available */
  if (sr & UART_SR_RXRDY) {
    chSysLockFromIsr();
    sdIncomingDataI(sdp, u->UART_RHR);
    chSysUnlockFromIsr();
  }
  
  /* Check if a character must be send */   
  if ((u->UART_IMR & UART_IMR_TXRDY) && (sr & UART_SR_TXRDY)) {
    msg_t b;

    chSysLockFromIsr();
    b = chOQGetI(&sdp->oqueue);
    if (b < Q_OK) {
      chIOAddFlagsI(sdp, IO_OUTPUT_EMPTY);
      u->UART_IDR = UART_SR_TXRDY;
    }
    else
      u->UART_THR = b;
    chSysUnlockFromIsr();
  }

  /* Handle error if available */  
  sr &= (UART_SR_OVRE | UART_SR_FRAME | UART_SR_PARE);
  if (sr != 0) {
    set_error(sdp, sr);
    u->UART_CR = UART_CR_RSTSTA; /* Reset Status Bits */
  }
}

#if USE_SAM3_UART0 || defined(__DOXYGEN__)
static void notify1(GenericQueue *qp) {

  (void)qp;
  UART0->UART_IER = UART_SR_TXRDY;
}
#endif


#if USE_SAM3_UART1 || defined(__DOXYGEN__)
static void notify2(GenericQueue *qp) {

  (void)qp;
  UART1->UART_IER = UART_SR_TXRDY;
}
#endif


/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if USE_SAM3_UART0 || defined(__DOXYGEN__)
/**
 * @brief   UART0 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(UART0_IRQHandler) {

  CH_IRQ_PROLOGUE();

  serve_interrupt(&SD1);

  CH_IRQ_EPILOGUE();
}
#endif


#if USE_SAM3_UART1 || defined(__DOXYGEN__)
/**
 * @brief   UART1 interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(UART1_IRQHandler) {

  CH_IRQ_PROLOGUE();

  serve_interrupt(&SD2);

  CH_IRQ_EPILOGUE();
}
#endif


/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {

#if USE_SAM3_UART0
  sdObjectInit(&SD1, NULL, notify1);
  SD1.uart = UART0;
  PIOA->PIO_PDR  = SAM3_UART0_RX | SAM3_UART0_TX;
  PIOA->PIO_PUDR = SAM3_UART0_RX | SAM3_UART0_TX;
#endif

#if USE_SAM3_UART1
  sdObjectInit(&SD2, NULL, notify2);
  SD2.uart = UART1;
  PIOB->PIO_PDR  = SAM3_UART1_RX | SAM3_UART1_TX;
  PIOB->PIO_PUDR = SAM3_UART1_RX | SAM3_UART1_TX;
#endif

}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL)
    config = &default_config;

  if (sdp->state == SD_STOP) {
#if USE_SAM3_UART0
    if (&SD1 == sdp) {
      /* Starts the clock. */
#if defined(SAM3U_PLATFORM)
      PMC->PMC_PCER = UART0_CLOCK;
#else      
      PMC->PMC_WPMR = 0x504D4300; /* Disable write protect */
      PMC->PMC_PCER = UART0_CLOCK;
      PMC->PMC_WPMR = 0x504D4301; /* Enable write protect */
#endif      
      
      /* Enables associated interrupt vector.*/
      nvicEnableVector(UART0_IRQn, CORTEX_PRIORITY_MASK(SAM3_UART0_PRIORITY));
    }
#endif

#if USE_SAM3_UART1
    if (&SD2 == sdp) {
      /* Starts the clock. */
#if defined(SAM3U_PLATFORM)
      PMC->PMC_PCER = UART1_CLOCK;
#else      
      PMC->PMC_WPMR = 0x504D4300; /* Disable write protect */
      PMC->PMC_PCER = UART1_CLOCK;
      PMC->PMC_WPMR = 0x504D4301; /* Enable write protect */
#endif      
      
      /* Enables associated interrupt vector.*/
      nvicEnableVector(UART1_IRQn, CORTEX_PRIORITY_MASK(SAM3_UART1_PRIORITY));
    }
#endif

  }
  uart_init(sdp, config);
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the UART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
    uart_deinit(sdp->uart);
#if USE_SAM3_UART0
    if (&SD1 == sdp) {
      /* Stop the clock. */
#if defined(SAM3U_PLATFORM)
      PMC->PMC_PCDR = UART0_CLOCK;
#else
      PMC->PMC_WPMR = 0x504D4300; /* Disable write protect */
      PMC->PMC_PCDR = UART0_CLOCK;
      PMC->PMC_WPMR = 0x504D4301; /* Enable write protect */
#endif      
      nvicDisableVector(UART0_IRQn);
      return;
    }
#endif
  }

  if (sdp->state == SD_READY) {
    uart_deinit(sdp->uart);
#if USE_SAM3_UART1
    if (&SD2 == sdp) {
      /* Stop the clock. */
#if defined(SAM3U_PLATFORM)
      PMC->PMC_PCDR = UART1_CLOCK;
#else
      PMC->PMC_WPMR = 0x504D4300; /* Disable write protect */
      PMC->PMC_PCDR = UART1_CLOCK;
      PMC->PMC_WPMR = 0x504D4301; /* Enable write protect */
#endif      
      nvicDisableVector(UART1_IRQn);
      return;
    }
#endif
  }

}

#endif /* HAL_USE_SERIAL */

/** @} */
