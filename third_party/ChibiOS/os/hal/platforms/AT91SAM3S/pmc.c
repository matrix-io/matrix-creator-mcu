/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2009, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/

#include "at91lib/SAM3S.h"

#include <assert.h>

/*----------------------------------------------------------------------------
 *        Local definitions
 *----------------------------------------------------------------------------*/

#define MASK_STATUS0 0xFFFFFFFC
#define MASK_STATUS1 0xFFFFFFFF

#define PLL_A            0           /* PLL A */
#define PLL_B            1           /* PLL B */


/*----------------------------------------------------------------------------
 *        Exported functions
 *----------------------------------------------------------------------------*/

/**
 * \brief Enables the clock of a peripheral. The peripheral ID is used
 * to identify which peripheral is targetted.
 *
 * \note The ID must NOT be shifted (i.e. 1 << ID_xxx).
 *
 * \param id  Peripheral ID (ID_xxx).
 */
extern void PMC_EnablePeripheral( uint32_t dwId )
{
    assert( dwId < 35 ) ;

    if ( dwId < 32 )
    {
        if ( (PMC->PMC_PCSR0 & ((uint32_t)1 << dwId)) == ((uint32_t)1 << dwId) )
        {
            //TRACE_DEBUG( "PMC_EnablePeripheral: clock of peripheral"  " %u is already enabled\n\r", dwId ) ;
        }
        else
        {
            PMC->PMC_PCER0 = 1 << dwId ;
        }
    }
    else
    {
        dwId -= 32;
        if ((PMC->PMC_PCSR1 & ((uint32_t)1 << dwId)) == ((uint32_t)1 << dwId))
        {
            //TRACE_DEBUG( "PMC_EnablePeripheral: clock of peripheral"  " %u is already enabled\n\r", dwId + 32 ) ;
        }
        else
        {
            PMC->PMC_PCER1 = 1 << dwId ;
        }
    }
}

/**
 * \brief Disables the clock of a peripheral. The peripheral ID is used
 * to identify which peripheral is targetted.
 *
 * \note The ID must NOT be shifted (i.e. 1 << ID_xxx).
 *
 * \param id  Peripheral ID (ID_xxx).
 */
extern void PMC_DisablePeripheral( uint32_t dwId )
{
    assert( dwId < 35 ) ;

    if ( dwId < 32 )
    {
        if ( (PMC->PMC_PCSR0 & ((uint32_t)1 << dwId)) != ((uint32_t)1 << dwId) )
        {
            //TRACE_DEBUG("PMC_DisablePeripheral: clock of peripheral" " %u is not enabled\n\r", dwId ) ;
        }
        else
        {
            PMC->PMC_PCDR0 = 1 << dwId ;
        }
    }
    else
    {
        dwId -= 32 ;
        if ( (PMC->PMC_PCSR1 & ((uint32_t)1 << dwId)) != ((uint32_t)1 << dwId) )
        {
            //TRACE_DEBUG( "PMC_DisablePeripheral: clock of peripheral" " %u is not enabled\n\r", dwId + 32 ) ;
        }
        else
        {
            PMC->PMC_PCDR1 = 1 << dwId ;
        }
    }
}

/**
 * \brief Enable all the periph clock via PMC.
 */
extern void PMC_EnableAllPeripherals( void )
{
    PMC->PMC_PCER0 = MASK_STATUS0 ;
    while ( (PMC->PMC_PCSR0 & MASK_STATUS0) != MASK_STATUS0 ) ;

    PMC->PMC_PCER1 = MASK_STATUS1 ;
    while ( (PMC->PMC_PCSR1 & MASK_STATUS1) != MASK_STATUS1 ) ;

    //TRACE_DEBUG( "Enable all periph clocks\n\r" ) ;
}

/**
 * \brief Disable all the periph clock via PMC.
 */
extern void PMC_DisableAllPeripherals( void )
{
    PMC->PMC_PCDR0 = MASK_STATUS0 ;
    while ( (PMC->PMC_PCSR0 & MASK_STATUS0) != 0 ) ;

    PMC->PMC_PCDR1 = MASK_STATUS1 ;
    while ( (PMC->PMC_PCSR1 & MASK_STATUS1) != 0 ) ;

    //TRACE_DEBUG( "Disable all periph clocks\n\r" ) ;
}

/**
 * \brief Get Periph Status for the given peripheral ID.
 *
 * \param id  Peripheral ID (ID_xxx).
 */
extern uint32_t PMC_IsPeriphEnabled( uint32_t dwId )
{
    assert( dwId < 35 ) ;

    if ( dwId < 32 )
    {
        return ( PMC->PMC_PCSR0 & (1 << dwId) ) ;
    }
    else {
        return ( PMC->PMC_PCSR1 & (1 << (dwId - 32)) ) ;
    }
}


extern void PmcMasterClockSelection( uint32_t dwClockSource, uint32_t dwPrescaler )
{
    uint32_t dwTimeout = 0 ;

    switch ( dwClockSource )
    {
        case PMC_MCKR_CSS_SLOW_CLK :
        case PMC_MCKR_CSS_MAIN_CLK :
            /* The Master Clock selection is made by writing the CSS field in PMC_MCKR programs the prescaler.*/
            PMC->PMC_MCKR = (PMC->PMC_MCKR & ~PMC_MCKR_CSS_Msk) | dwClockSource ;

            /* Once the PMC_MCKR register has been written, the user must wait for the MCKRDY bit to be set in the PMC_SR register.*/
            while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) && (dwTimeout++ <  0xFFFFFFFF) ) ;

            /* The Master Clock selection is made by writing the PRES field in PMC_MCKR programs the prescaler.*/
            PMC->PMC_MCKR = (PMC->PMC_MCKR & ~PMC_MCKR_PRES_Msk) | dwPrescaler ;

            /* Once the PMC_MCKR register has been written, the user must wait for the MCKRDY bit to be set in the PMC_SR register.*/
            while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) && (dwTimeout++ <  0xFFFFFFFF) ) ;
        break ;

        case PMC_MCKR_CSS_PLLA_CLK :
        case PMC_MCKR_CSS_PLLB_CLK :
            /* The Master Clock selection is made by writing the PRES field in PMC_MCKR programs the prescaler.*/
            PMC->PMC_MCKR = (PMC->PMC_MCKR & ~PMC_MCKR_PRES_Msk) | dwPrescaler ;

            /* Once the PMC_MCKR register has been written, the user must wait for the MCKRDY bit to be set in the PMC_SR register.*/
            while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) && (dwTimeout++ <  0xFFFFFFFF) ) ;

            /* The Master Clock selection is made by writing the CSS field in PMC_MCKR programs the prescaler.*/
            PMC->PMC_MCKR = (PMC->PMC_MCKR & ~PMC_MCKR_CSS_Msk) | dwClockSource ;

            /* Once the PMC_MCKR register has been written, the user must wait for the MCKRDY bit to be set in the PMC_SR register.*/
            while ( !(PMC->PMC_SR & PMC_SR_MCKRDY) && (dwTimeout++ <  0xFFFFFFFF) ) ;
        break ;
    }
}

extern void ConfigurePck( uint32_t dwClockSource, uint32_t dwPrescaler)
{

    PMC->PMC_WPMR = PMC_WPMR_WPKEY(0x504D43);
    /* Programmable clock 1 output disabled */
    PMC->PMC_SCDR = PMC_SCER_PCK2;
    /* Configure PMC Programmable Clock */
    PMC->PMC_PCK[2] = dwClockSource | dwPrescaler ;
    /* Enable PCK2 output */
    PMC->PMC_SCER = PMC_SCER_PCK2;
    /* Wait for the PCKRDY2 bit to be set in the PMC_SR register */
    while ( (PMC->PMC_SR & PMC_SR_PCKRDY2) == 0 ) ;
}


extern void PmcMainClockSwitchMainOsc( void )
{
    uint32_t dwTimeout ;

    /* Enable Main XTAL Oscillator */
    PMC->CKGR_MOR = CKGR_MOR_KEY( 0x37 ) | CKGR_MOR_MOSCXTST( 0x8 ) | CKGR_MOR_MOSCSEL | CKGR_MOR_MOSCRCEN | CKGR_MOR_MOSCXTEN ;

   /* Wait for Main Oscillator Selection Status bit MOSCSELS */
    for ( dwTimeout=0 ; !(PMC->PMC_SR & PMC_SR_MOSCSELS) && (dwTimeout++ < 0xFFFFFFFF) ; ) ;
}


extern void PmcConfigurePllClock( uint8_t pllab, uint32_t dwMul, uint32_t dwDiv )
{
    uint32_t dwTimeout=0 ;

    if ( pllab == PLL_A )
    {
        /* Configure PLLA and the divider */
        PMC->CKGR_PLLAR = CKGR_PLLAR_STUCKTO1 | CKGR_PLLAR_PLLACOUNT( 0x1f ) | CKGR_PLLAR_MULA( dwMul ) | dwDiv ;

        /* If PLL activated, wait locking */
        if ( dwMul != 0 )
        {
            /* wait for the LOCKA bit to be set */
            while ( !(PMC->PMC_SR & PMC_SR_LOCKA) && (dwTimeout++ < 0xFFFFFFFF) ) ;
        }
    }

    if ( pllab == PLL_B )
    {
        /* Configure PLLB and the divider */
        PMC->CKGR_PLLBR = CKGR_PLLAR_STUCKTO1 | CKGR_PLLAR_PLLACOUNT( 0x1f ) | CKGR_PLLAR_MULA( dwMul ) | dwDiv ;

        /* If PLL activated, wait locking */
        if ( dwMul != 0 )
        {
            /* wait for the LOCKB bit to be set */
            while ( !(PMC->PMC_SR & PMC_SR_LOCKB) && (dwTimeout++ < 0xFFFFFFFF) ) ;
        }
    }
}




