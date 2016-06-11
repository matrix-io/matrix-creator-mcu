/****************************************************************************
*  Copyright (c) 2011 by Michael Fischer. All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*  
*  1. Redistributions of source code must retain the above copyright 
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the 
*     documentation and/or other materials provided with the distribution.
*  3. Neither the name of the author nor the names of its contributors may 
*     be used to endorse or promote products derived from this software 
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
*  SUCH DAMAGE.
*
****************************************************************************
*  History:
*
*  25.05.2011  mifi  First Version, this is not an original Atmel file.
*                    I could not find a SAM3U.h like it is available for
*                    the SAM3S and SAM3N. Therefore I decide to build an
*                    own file based on AT91SAM3U4.h. Only parts which are
*                    needed for MY application was added.
*                    Therefore partial copyright by Atmel Corporation too.
****************************************************************************/

#ifndef SAM3U_H
#define SAM3U_H

/** \addtogroup SAM3U_definitions SAM3U definitions
  This file defines all structures and symbols for SAM3U:
    - registers and bitfields
    - peripheral base address
    - peripheral ID
    - PIO definitions
*/
/*@{*/

#ifdef __cplusplus
 extern "C" {
#endif

#ifndef __ASSEMBLY__
#include <stdint.h>
#ifndef __cplusplus
typedef volatile const uint32_t RoReg;    /**< Read only 32-bit register (volatile const unsigned int) */
#else
typedef volatile       uint32_t RoReg;    /**< Read only 32-bit register (volatile const unsigned int) */
#endif
typedef volatile       uint32_t WoReg;    /**< Write only 32-bit register (volatile unsigned int) */
typedef volatile       uint32_t RwReg;    /**< Read-Write 32-bit register (volatile unsigned int) */
typedef volatile       uint32_t AT91_REG; /**< Read-Write 32-bit register (volatile unsigned int) */
#define CAST(type, value) ((type *) value)
#define REG_ACCESS(type, address) (*(type*)address) /**< C code: Register value */
#else
#define CAST(type, value) (value)
#define REG_ACCESS(type, address) (address) /**< Assembly code: Register address */
#endif

/* ************************************************************************** */
/*   CMSIS DEFINITIONS FOR SAM3U */
/* ************************************************************************** */
/** \addtogroup SAM3U_cmsis CMSIS Definitions */
/*@{*/

/**< Interrupt Number Definition */
typedef enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ******************************/
  NonMaskableInt_IRQn   = -14, /**<  2 Non Maskable Interrupt                */
  MemoryManagement_IRQn = -12, /**<  4 Cortex-M3 Memory Management Interrupt */
  BusFault_IRQn         = -11, /**<  5 Cortex-M3 Bus Fault Interrupt         */
  UsageFault_IRQn       = -10, /**<  6 Cortex-M3 Usage Fault Interrupt       */
  SVCall_IRQn           = -5,  /**< 11 Cortex-M3 SV Call Interrupt           */
  DebugMonitor_IRQn     = -4,  /**< 12 Cortex-M3 Debug Monitor Interrupt     */
  PendSV_IRQn           = -2,  /**< 14 Cortex-M3 Pend SV Interrupt           */
  SysTick_IRQn          = -1,  /**< 15 Cortex-M3 System Tick Interrupt       */
/******  SAM3U specific Interrupt Numbers *********************************/

  SUPC_IRQn            =  0, /**<  0 SAM3U Supply Controller (SUPC) */
  RSTC_IRQn            =  1, /**<  1 SAM3U Reset Controller (RSTC) */
  RTC_IRQn             =  2, /**<  2 SAM3U Real Time Clock (RTC) */
  RTT_IRQn             =  3, /**<  3 SAM3U Real Time Timer (RTT) */
  WDT_IRQn             =  4, /**<  4 SAM3U Watchdog Timer (WDT) */
  PMC_IRQn             =  5, /**<  5 SAM3U Power Management Controller (PMC) */
  EFC0_IRQn            =  6, /**<  6 SAM3U Enhanced Embedded Flash Controller 0 (EFC0) */
  EFC1_IRQn            =  7, /**<  7 SAM3U Enhanced Embedded Flash Controller 1 (EFC1) */
  UART0_IRQn           =  8, /**<  8 SAM3U UART 0 (UART0) */
  SMC_IRQn             =  9, /**<  9 SAM3U Static Memory Controller */
  PIOA_IRQn            = 10, /**< 10 SAM3U Parallel I/O Controller A (PIOA) */
  PIOB_IRQn            = 11, /**< 11 SAM3U Parallel I/O Controller B (PIOB) */
  PIOC_IRQn            = 12, /**< 12 SAM3U Parallel I/O Controller C (PIOC) */
  USART0_IRQn          = 13, /**< 13 SAM3U USART 0 (USART0) */
  USART1_IRQn          = 14, /**< 14 SAM3U USART 1 (USART1) */
  USART2_IRQn          = 15, /**< 15 SAM3U USART 2 (USART2) */
  USART3_IRQn          = 16, /**< 16 SAM3U USART 3 (USART3) */
  HSMCI_IRQn           = 17, /**< 17 SAM3U Multimedia Card Interface (HSMCI) */  
  TWI0_IRQn            = 18, /**< 18 SAM3U Two Wire Interface 0 (TWI0) */
  TWI1_IRQn            = 19, /**< 19 SAM3U Two Wire Interface 1 (TWI1) */
  SPI_IRQn             = 20, /**< 20 SAM3U Serial Peripheral Interface (SPI) */
  SSC_IRQn             = 21, /**< 21 SAM3U Synchronous Serial Controler (SSC) */
  TC0_IRQn             = 22, /**< 22 SAM3U Timer/Counter 0 (TC0) */
  TC1_IRQn             = 23, /**< 23 SAM3U Timer/Counter 1 (TC1) */
  TC2_IRQn             = 24, /**< 24 SAM3U Timer/Counter 2 (TC2) */
  PWM_IRQn             = 25, /**< 25 SAM3U Pulse Width Modulation (PWM) */
  ADC12B_IRQn          = 26, /**< 26 SAM3U Analog To Digital Converter (ADC12B) */
  ADC_IRQn             = 27, /**< 27 SAM3U Analog To Digital Converter (ADC) */
  DMAC_IRQn            = 28, /**< 28 SAM3U DMA Controller (DMAC) */
  UDPHS_IRQn           = 29  /**< 29 SAM3U USB Device High Speed (UDPHS) */
} IRQn_Type;

/**
 * \brief Configuration of the Cortex-M3 Processor and Core Peripherals
 */

#define __MPU_PRESENT          1 /**< $product does provide a MPU */
#define __NVIC_PRIO_BITS       4 /**< $product uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig 0 /**< Set to 1 if different SysTick Config is used */

/*@}*/

/* ************************************************************************** */
/**  SOFTWARE PERIPHERAL API DEFINITION FOR SAM3U */
/* ************************************************************************** */
/** \addtogroup SAM3U_api Peripheral Software API */
/*@{*/


/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Embedded Flash Controller */
/* ============================================================================= */
/** \addtogroup SAM3U_EFC Embedded Flash Controller */
/*@{*/

#ifndef __ASSEMBLY__
/** \brief Efc hardware registers */
typedef struct {
  RwReg EEFC_FMR;    // EFC Flash Mode Register
  WoReg EEFC_FCR;    // EFC Flash Command Register
  RoReg EEFC_FSR;    // EFC Flash Status Register
  RoReg EEFC_FRR;    // EFC Flash Result Register
} Efc;
#endif /* __ASSEMBLY__ */
/* -------- EEFC_FMR : (EFC Offset: 0x00) EEFC Flash Mode Register -------- */
#define EEFC_FMR_FRDY (0x1u << 0) /**< \brief (EEFC_FMR) Ready Interrupt Enable */
#define EEFC_FMR_FWS_Pos 8
#define EEFC_FMR_FWS_Msk (0xfu << EEFC_FMR_FWS_Pos) /**< \brief (EEFC_FMR) Flash Wait State */
#define EEFC_FMR_FWS(value) ((EEFC_FMR_FWS_Msk & ((value) << EEFC_FMR_FWS_Pos)))
#define EEFC_FMR_FAM (0x1u << 24) /**< \brief (EEFC_FMR) Flash Access Mode */
/* -------- EEFC_FCR : (EFC Offset: 0x04) EEFC Flash Command Register -------- */
#define EEFC_FCR_FCMD_Pos 0
#define EEFC_FCR_FCMD_Msk (0xffu << EEFC_FCR_FCMD_Pos) /**< \brief (EEFC_FCR) Flash Command */
#define EEFC_FCR_FCMD(value) ((EEFC_FCR_FCMD_Msk & ((value) << EEFC_FCR_FCMD_Pos)))
#define EEFC_FCR_FARG_Pos 8
#define EEFC_FCR_FARG_Msk (0xffffu << EEFC_FCR_FARG_Pos) /**< \brief (EEFC_FCR) Flash Command Argument */
#define EEFC_FCR_FARG(value) ((EEFC_FCR_FARG_Msk & ((value) << EEFC_FCR_FARG_Pos)))
#define EEFC_FCR_FKEY_Pos 24
#define EEFC_FCR_FKEY_Msk (0xffu << EEFC_FCR_FKEY_Pos) /**< \brief (EEFC_FCR) Flash Writing Protection Key */
#define EEFC_FCR_FKEY(value) ((EEFC_FCR_FKEY_Msk & ((value) << EEFC_FCR_FKEY_Pos)))
/* -------- EEFC_FSR : (EFC Offset: 0x08) EEFC Flash Status Register -------- */
#define EEFC_FSR_FRDY (0x1u << 0) /**< \brief (EEFC_FSR) Flash Ready Status */
#define EEFC_FSR_FCMDE (0x1u << 1) /**< \brief (EEFC_FSR) Flash Command Error Status */
#define EEFC_FSR_FLOCKE (0x1u << 2) /**< \brief (EEFC_FSR) Flash Lock Error Status */
/* -------- EEFC_FRR : (EFC Offset: 0x0C) EEFC Flash Result Register -------- */
#define EEFC_FRR_FVALUE_Pos 0
#define EEFC_FRR_FVALUE_Msk (0xffffffffu << EEFC_FRR_FVALUE_Pos) /**< \brief (EEFC_FRR) Flash Result Value */

/*@}*/


/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Parallel Input/Output */
/* ============================================================================= */
/** \addtogroup SAM3U_PIO Parallel Input/Output */
/*@{*/

#ifndef __ASSEMBLY__
/** \brief Pio hardware registers */
typedef struct {
   AT91_REG  PIO_PER;   // PIO Enable Register
   AT91_REG  PIO_PDR;   // PIO Disable Register
   AT91_REG  PIO_PSR;   // PIO Status Register
   AT91_REG  Reserved0[1];    // 
   AT91_REG  PIO_OER;   // Output Enable Register
   AT91_REG  PIO_ODR;   // Output Disable Registerr
   AT91_REG  PIO_OSR;   // Output Status Register
   AT91_REG  Reserved1[1];    // 
   AT91_REG  PIO_IFER;  // Input Filter Enable Register
   AT91_REG  PIO_IFDR;  // Input Filter Disable Register
   AT91_REG  PIO_IFSR;  // Input Filter Status Register
   AT91_REG  Reserved2[1];    // 
   AT91_REG  PIO_SODR;  // Set Output Data Register
   AT91_REG  PIO_CODR;  // Clear Output Data Register
   AT91_REG  PIO_ODSR;  // Output Data Status Register
   AT91_REG  PIO_PDSR;  // Pin Data Status Register
   AT91_REG  PIO_IER;   // Interrupt Enable Register
   AT91_REG  PIO_IDR;   // Interrupt Disable Register
   AT91_REG  PIO_IMR;   // Interrupt Mask Register
   AT91_REG  PIO_ISR;   // Interrupt Status Register
   AT91_REG  PIO_MDER;  // Multi-driver Enable Register
   AT91_REG  PIO_MDDR;  // Multi-driver Disable Register
   AT91_REG  PIO_MDSR;  // Multi-driver Status Register
   AT91_REG  Reserved3[1];    // 
   AT91_REG  PIO_PUDR;  // Pull-up Disable Register
   AT91_REG  PIO_PUER;  // Pull-up Enable Register
   AT91_REG  PIO_PUSR;  // Pull-up Status Register
   AT91_REG  Reserved4[1];    // 
   AT91_REG  PIO_ABSR;  // Peripheral AB Select Register
   AT91_REG  Reserved5[3];    // 
   AT91_REG  PIO_SCIFSR;   // System Clock Glitch Input Filter Select Register
   AT91_REG  PIO_DIFSR;    // Debouncing Input Filter Select Register
   AT91_REG  PIO_IFDGSR;   // Glitch or Debouncing Input Filter Clock Selection Status Register
   AT91_REG  PIO_SCDR;  // Slow Clock Divider Debouncing Register
   AT91_REG  Reserved6[4];    // 
   AT91_REG  PIO_OWER;  // Output Write Enable Register
   AT91_REG  PIO_OWDR;  // Output Write Disable Register
   AT91_REG  PIO_OWSR;  // Output Write Status Register
   AT91_REG  Reserved7[1];    // 
   AT91_REG  PIO_AIMER;    // Additional Interrupt Modes Enable Register
   AT91_REG  PIO_AIMDR;    // Additional Interrupt Modes Disables Register
   AT91_REG  PIO_AIMMR;    // Additional Interrupt Modes Mask Register
   AT91_REG  Reserved8[1];    // 
   AT91_REG  PIO_ESR;   // Edge Select Register
   AT91_REG  PIO_LSR;   // Level Select Register
   AT91_REG  PIO_ELSR;  // Edge/Level Status Register
   AT91_REG  Reserved9[1];    // 
   AT91_REG  PIO_FELLSR;   // Falling Edge/Low Level Select Register
   AT91_REG  PIO_REHLSR;   // Rising Edge/ High Level Select Register
   AT91_REG  PIO_FRLHSR;   // Fall/Rise - Low/High Status Register
   AT91_REG  Reserved10[1];   // 
   AT91_REG  PIO_LOCKSR;   // Lock Status Register
   AT91_REG  Reserved11[6];   // 
   AT91_REG  PIO_VER;   // PIO VERSION REGISTER 
   AT91_REG  Reserved12[8];   // 
   AT91_REG  PIO_KER;   // Keypad Controller Enable Register
   AT91_REG  PIO_KRCR;  // Keypad Controller Row Column Register
   AT91_REG  PIO_KDR;   // Keypad Controller Debouncing Register
   AT91_REG  Reserved13[1];   // 
   AT91_REG  PIO_KIER;  // Keypad Controller Interrupt Enable Register
   AT91_REG  PIO_KIDR;  // Keypad Controller Interrupt Disable Register
   AT91_REG  PIO_KIMR;  // Keypad Controller Interrupt Mask Register
   AT91_REG  PIO_KSR;   // Keypad Controller Status Register
   AT91_REG  PIO_KKPR;  // Keypad Controller Key Press Register
   AT91_REG  PIO_KKRR;  // Keypad Controller Key Release Register
} Pio;
#endif /* __ASSEMBLY__ */
/* -------- PIO_PER : (PIO Offset: 0x0000) PIO Enable Register -------- */
#define PIO_PER_P0 (0x1u << 0) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P1 (0x1u << 1) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P2 (0x1u << 2) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P3 (0x1u << 3) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P4 (0x1u << 4) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P5 (0x1u << 5) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P6 (0x1u << 6) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P7 (0x1u << 7) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P8 (0x1u << 8) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P9 (0x1u << 9) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P10 (0x1u << 10) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P11 (0x1u << 11) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P12 (0x1u << 12) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P13 (0x1u << 13) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P14 (0x1u << 14) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P15 (0x1u << 15) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P16 (0x1u << 16) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P17 (0x1u << 17) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P18 (0x1u << 18) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P19 (0x1u << 19) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P20 (0x1u << 20) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P21 (0x1u << 21) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P22 (0x1u << 22) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P23 (0x1u << 23) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P24 (0x1u << 24) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P25 (0x1u << 25) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P26 (0x1u << 26) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P27 (0x1u << 27) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P28 (0x1u << 28) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P29 (0x1u << 29) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P30 (0x1u << 30) /**< \brief (PIO_PER) PIO Enable */
#define PIO_PER_P31 (0x1u << 31) /**< \brief (PIO_PER) PIO Enable */
/* -------- PIO_PDR : (PIO Offset: 0x0004) PIO Disable Register -------- */
#define PIO_PDR_P0 (0x1u << 0) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P1 (0x1u << 1) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P2 (0x1u << 2) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P3 (0x1u << 3) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P4 (0x1u << 4) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P5 (0x1u << 5) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P6 (0x1u << 6) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P7 (0x1u << 7) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P8 (0x1u << 8) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P9 (0x1u << 9) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P10 (0x1u << 10) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P11 (0x1u << 11) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P12 (0x1u << 12) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P13 (0x1u << 13) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P14 (0x1u << 14) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P15 (0x1u << 15) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P16 (0x1u << 16) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P17 (0x1u << 17) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P18 (0x1u << 18) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P19 (0x1u << 19) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P20 (0x1u << 20) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P21 (0x1u << 21) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P22 (0x1u << 22) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P23 (0x1u << 23) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P24 (0x1u << 24) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P25 (0x1u << 25) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P26 (0x1u << 26) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P27 (0x1u << 27) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P28 (0x1u << 28) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P29 (0x1u << 29) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P30 (0x1u << 30) /**< \brief (PIO_PDR) PIO Disable */
#define PIO_PDR_P31 (0x1u << 31) /**< \brief (PIO_PDR) PIO Disable */
/* -------- PIO_PSR : (PIO Offset: 0x0008) PIO Status Register -------- */
#define PIO_PSR_P0 (0x1u << 0) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P1 (0x1u << 1) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P2 (0x1u << 2) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P3 (0x1u << 3) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P4 (0x1u << 4) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P5 (0x1u << 5) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P6 (0x1u << 6) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P7 (0x1u << 7) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P8 (0x1u << 8) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P9 (0x1u << 9) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P10 (0x1u << 10) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P11 (0x1u << 11) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P12 (0x1u << 12) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P13 (0x1u << 13) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P14 (0x1u << 14) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P15 (0x1u << 15) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P16 (0x1u << 16) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P17 (0x1u << 17) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P18 (0x1u << 18) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P19 (0x1u << 19) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P20 (0x1u << 20) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P21 (0x1u << 21) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P22 (0x1u << 22) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P23 (0x1u << 23) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P24 (0x1u << 24) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P25 (0x1u << 25) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P26 (0x1u << 26) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P27 (0x1u << 27) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P28 (0x1u << 28) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P29 (0x1u << 29) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P30 (0x1u << 30) /**< \brief (PIO_PSR) PIO Status */
#define PIO_PSR_P31 (0x1u << 31) /**< \brief (PIO_PSR) PIO Status */
/* -------- PIO_OER : (PIO Offset: 0x0010) Output Enable Register -------- */
#define PIO_OER_P0 (0x1u << 0) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P1 (0x1u << 1) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P2 (0x1u << 2) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P3 (0x1u << 3) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P4 (0x1u << 4) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P5 (0x1u << 5) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P6 (0x1u << 6) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P7 (0x1u << 7) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P8 (0x1u << 8) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P9 (0x1u << 9) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P10 (0x1u << 10) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P11 (0x1u << 11) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P12 (0x1u << 12) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P13 (0x1u << 13) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P14 (0x1u << 14) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P15 (0x1u << 15) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P16 (0x1u << 16) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P17 (0x1u << 17) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P18 (0x1u << 18) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P19 (0x1u << 19) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P20 (0x1u << 20) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P21 (0x1u << 21) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P22 (0x1u << 22) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P23 (0x1u << 23) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P24 (0x1u << 24) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P25 (0x1u << 25) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P26 (0x1u << 26) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P27 (0x1u << 27) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P28 (0x1u << 28) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P29 (0x1u << 29) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P30 (0x1u << 30) /**< \brief (PIO_OER) Output Enable */
#define PIO_OER_P31 (0x1u << 31) /**< \brief (PIO_OER) Output Enable */
/* -------- PIO_ODR : (PIO Offset: 0x0014) Output Disable Register -------- */
#define PIO_ODR_P0 (0x1u << 0) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P1 (0x1u << 1) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P2 (0x1u << 2) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P3 (0x1u << 3) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P4 (0x1u << 4) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P5 (0x1u << 5) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P6 (0x1u << 6) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P7 (0x1u << 7) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P8 (0x1u << 8) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P9 (0x1u << 9) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P10 (0x1u << 10) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P11 (0x1u << 11) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P12 (0x1u << 12) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P13 (0x1u << 13) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P14 (0x1u << 14) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P15 (0x1u << 15) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P16 (0x1u << 16) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P17 (0x1u << 17) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P18 (0x1u << 18) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P19 (0x1u << 19) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P20 (0x1u << 20) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P21 (0x1u << 21) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P22 (0x1u << 22) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P23 (0x1u << 23) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P24 (0x1u << 24) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P25 (0x1u << 25) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P26 (0x1u << 26) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P27 (0x1u << 27) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P28 (0x1u << 28) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P29 (0x1u << 29) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P30 (0x1u << 30) /**< \brief (PIO_ODR) Output Disable */
#define PIO_ODR_P31 (0x1u << 31) /**< \brief (PIO_ODR) Output Disable */
/* -------- PIO_OSR : (PIO Offset: 0x0018) Output Status Register -------- */
#define PIO_OSR_P0 (0x1u << 0) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P1 (0x1u << 1) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P2 (0x1u << 2) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P3 (0x1u << 3) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P4 (0x1u << 4) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P5 (0x1u << 5) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P6 (0x1u << 6) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P7 (0x1u << 7) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P8 (0x1u << 8) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P9 (0x1u << 9) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P10 (0x1u << 10) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P11 (0x1u << 11) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P12 (0x1u << 12) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P13 (0x1u << 13) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P14 (0x1u << 14) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P15 (0x1u << 15) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P16 (0x1u << 16) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P17 (0x1u << 17) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P18 (0x1u << 18) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P19 (0x1u << 19) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P20 (0x1u << 20) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P21 (0x1u << 21) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P22 (0x1u << 22) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P23 (0x1u << 23) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P24 (0x1u << 24) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P25 (0x1u << 25) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P26 (0x1u << 26) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P27 (0x1u << 27) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P28 (0x1u << 28) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P29 (0x1u << 29) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P30 (0x1u << 30) /**< \brief (PIO_OSR) Output Status */
#define PIO_OSR_P31 (0x1u << 31) /**< \brief (PIO_OSR) Output Status */
/* -------- PIO_IFER : (PIO Offset: 0x0020) Glitch Input Filter Enable Register -------- */
#define PIO_IFER_P0 (0x1u << 0) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P1 (0x1u << 1) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P2 (0x1u << 2) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P3 (0x1u << 3) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P4 (0x1u << 4) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P5 (0x1u << 5) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P6 (0x1u << 6) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P7 (0x1u << 7) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P8 (0x1u << 8) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P9 (0x1u << 9) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P10 (0x1u << 10) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P11 (0x1u << 11) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P12 (0x1u << 12) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P13 (0x1u << 13) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P14 (0x1u << 14) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P15 (0x1u << 15) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P16 (0x1u << 16) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P17 (0x1u << 17) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P18 (0x1u << 18) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P19 (0x1u << 19) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P20 (0x1u << 20) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P21 (0x1u << 21) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P22 (0x1u << 22) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P23 (0x1u << 23) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P24 (0x1u << 24) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P25 (0x1u << 25) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P26 (0x1u << 26) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P27 (0x1u << 27) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P28 (0x1u << 28) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P29 (0x1u << 29) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P30 (0x1u << 30) /**< \brief (PIO_IFER) Input Filter Enable */
#define PIO_IFER_P31 (0x1u << 31) /**< \brief (PIO_IFER) Input Filter Enable */
/* -------- PIO_IFDR : (PIO Offset: 0x0024) Glitch Input Filter Disable Register -------- */
#define PIO_IFDR_P0 (0x1u << 0) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P1 (0x1u << 1) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P2 (0x1u << 2) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P3 (0x1u << 3) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P4 (0x1u << 4) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P5 (0x1u << 5) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P6 (0x1u << 6) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P7 (0x1u << 7) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P8 (0x1u << 8) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P9 (0x1u << 9) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P10 (0x1u << 10) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P11 (0x1u << 11) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P12 (0x1u << 12) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P13 (0x1u << 13) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P14 (0x1u << 14) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P15 (0x1u << 15) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P16 (0x1u << 16) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P17 (0x1u << 17) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P18 (0x1u << 18) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P19 (0x1u << 19) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P20 (0x1u << 20) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P21 (0x1u << 21) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P22 (0x1u << 22) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P23 (0x1u << 23) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P24 (0x1u << 24) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P25 (0x1u << 25) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P26 (0x1u << 26) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P27 (0x1u << 27) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P28 (0x1u << 28) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P29 (0x1u << 29) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P30 (0x1u << 30) /**< \brief (PIO_IFDR) Input Filter Disable */
#define PIO_IFDR_P31 (0x1u << 31) /**< \brief (PIO_IFDR) Input Filter Disable */
/* -------- PIO_IFSR : (PIO Offset: 0x0028) Glitch Input Filter Status Register -------- */
#define PIO_IFSR_P0 (0x1u << 0) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P1 (0x1u << 1) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P2 (0x1u << 2) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P3 (0x1u << 3) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P4 (0x1u << 4) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P5 (0x1u << 5) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P6 (0x1u << 6) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P7 (0x1u << 7) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P8 (0x1u << 8) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P9 (0x1u << 9) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P10 (0x1u << 10) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P11 (0x1u << 11) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P12 (0x1u << 12) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P13 (0x1u << 13) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P14 (0x1u << 14) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P15 (0x1u << 15) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P16 (0x1u << 16) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P17 (0x1u << 17) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P18 (0x1u << 18) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P19 (0x1u << 19) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P20 (0x1u << 20) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P21 (0x1u << 21) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P22 (0x1u << 22) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P23 (0x1u << 23) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P24 (0x1u << 24) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P25 (0x1u << 25) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P26 (0x1u << 26) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P27 (0x1u << 27) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P28 (0x1u << 28) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P29 (0x1u << 29) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P30 (0x1u << 30) /**< \brief (PIO_IFSR) Input Filer Status */
#define PIO_IFSR_P31 (0x1u << 31) /**< \brief (PIO_IFSR) Input Filer Status */
/* -------- PIO_SODR : (PIO Offset: 0x0030) Set Output Data Register -------- */
#define PIO_SODR_P0 (0x1u << 0) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P1 (0x1u << 1) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P2 (0x1u << 2) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P3 (0x1u << 3) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P4 (0x1u << 4) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P5 (0x1u << 5) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P6 (0x1u << 6) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P7 (0x1u << 7) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P8 (0x1u << 8) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P9 (0x1u << 9) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P10 (0x1u << 10) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P11 (0x1u << 11) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P12 (0x1u << 12) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P13 (0x1u << 13) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P14 (0x1u << 14) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P15 (0x1u << 15) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P16 (0x1u << 16) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P17 (0x1u << 17) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P18 (0x1u << 18) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P19 (0x1u << 19) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P20 (0x1u << 20) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P21 (0x1u << 21) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P22 (0x1u << 22) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P23 (0x1u << 23) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P24 (0x1u << 24) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P25 (0x1u << 25) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P26 (0x1u << 26) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P27 (0x1u << 27) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P28 (0x1u << 28) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P29 (0x1u << 29) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P30 (0x1u << 30) /**< \brief (PIO_SODR) Set Output Data */
#define PIO_SODR_P31 (0x1u << 31) /**< \brief (PIO_SODR) Set Output Data */
/* -------- PIO_CODR : (PIO Offset: 0x0034) Clear Output Data Register -------- */
#define PIO_CODR_P0 (0x1u << 0) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P1 (0x1u << 1) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P2 (0x1u << 2) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P3 (0x1u << 3) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P4 (0x1u << 4) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P5 (0x1u << 5) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P6 (0x1u << 6) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P7 (0x1u << 7) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P8 (0x1u << 8) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P9 (0x1u << 9) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P10 (0x1u << 10) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P11 (0x1u << 11) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P12 (0x1u << 12) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P13 (0x1u << 13) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P14 (0x1u << 14) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P15 (0x1u << 15) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P16 (0x1u << 16) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P17 (0x1u << 17) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P18 (0x1u << 18) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P19 (0x1u << 19) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P20 (0x1u << 20) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P21 (0x1u << 21) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P22 (0x1u << 22) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P23 (0x1u << 23) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P24 (0x1u << 24) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P25 (0x1u << 25) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P26 (0x1u << 26) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P27 (0x1u << 27) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P28 (0x1u << 28) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P29 (0x1u << 29) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P30 (0x1u << 30) /**< \brief (PIO_CODR) Clear Output Data */
#define PIO_CODR_P31 (0x1u << 31) /**< \brief (PIO_CODR) Clear Output Data */
/* -------- PIO_ODSR : (PIO Offset: 0x0038) Output Data Status Register -------- */
#define PIO_ODSR_P0 (0x1u << 0) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P1 (0x1u << 1) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P2 (0x1u << 2) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P3 (0x1u << 3) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P4 (0x1u << 4) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P5 (0x1u << 5) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P6 (0x1u << 6) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P7 (0x1u << 7) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P8 (0x1u << 8) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P9 (0x1u << 9) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P10 (0x1u << 10) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P11 (0x1u << 11) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P12 (0x1u << 12) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P13 (0x1u << 13) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P14 (0x1u << 14) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P15 (0x1u << 15) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P16 (0x1u << 16) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P17 (0x1u << 17) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P18 (0x1u << 18) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P19 (0x1u << 19) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P20 (0x1u << 20) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P21 (0x1u << 21) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P22 (0x1u << 22) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P23 (0x1u << 23) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P24 (0x1u << 24) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P25 (0x1u << 25) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P26 (0x1u << 26) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P27 (0x1u << 27) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P28 (0x1u << 28) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P29 (0x1u << 29) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P30 (0x1u << 30) /**< \brief (PIO_ODSR) Output Data Status */
#define PIO_ODSR_P31 (0x1u << 31) /**< \brief (PIO_ODSR) Output Data Status */
/* -------- PIO_PDSR : (PIO Offset: 0x003C) Pin Data Status Register -------- */
#define PIO_PDSR_P0 (0x1u << 0) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P1 (0x1u << 1) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P2 (0x1u << 2) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P3 (0x1u << 3) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P4 (0x1u << 4) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P5 (0x1u << 5) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P6 (0x1u << 6) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P7 (0x1u << 7) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P8 (0x1u << 8) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P9 (0x1u << 9) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P10 (0x1u << 10) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P11 (0x1u << 11) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P12 (0x1u << 12) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P13 (0x1u << 13) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P14 (0x1u << 14) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P15 (0x1u << 15) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P16 (0x1u << 16) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P17 (0x1u << 17) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P18 (0x1u << 18) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P19 (0x1u << 19) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P20 (0x1u << 20) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P21 (0x1u << 21) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P22 (0x1u << 22) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P23 (0x1u << 23) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P24 (0x1u << 24) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P25 (0x1u << 25) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P26 (0x1u << 26) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P27 (0x1u << 27) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P28 (0x1u << 28) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P29 (0x1u << 29) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P30 (0x1u << 30) /**< \brief (PIO_PDSR) Output Data Status */
#define PIO_PDSR_P31 (0x1u << 31) /**< \brief (PIO_PDSR) Output Data Status */
/* -------- PIO_IER : (PIO Offset: 0x0040) Interrupt Enable Register -------- */
#define PIO_IER_P0 (0x1u << 0) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P1 (0x1u << 1) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P2 (0x1u << 2) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P3 (0x1u << 3) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P4 (0x1u << 4) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P5 (0x1u << 5) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P6 (0x1u << 6) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P7 (0x1u << 7) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P8 (0x1u << 8) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P9 (0x1u << 9) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P10 (0x1u << 10) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P11 (0x1u << 11) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P12 (0x1u << 12) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P13 (0x1u << 13) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P14 (0x1u << 14) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P15 (0x1u << 15) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P16 (0x1u << 16) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P17 (0x1u << 17) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P18 (0x1u << 18) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P19 (0x1u << 19) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P20 (0x1u << 20) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P21 (0x1u << 21) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P22 (0x1u << 22) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P23 (0x1u << 23) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P24 (0x1u << 24) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P25 (0x1u << 25) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P26 (0x1u << 26) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P27 (0x1u << 27) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P28 (0x1u << 28) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P29 (0x1u << 29) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P30 (0x1u << 30) /**< \brief (PIO_IER) Input Change Interrupt Enable */
#define PIO_IER_P31 (0x1u << 31) /**< \brief (PIO_IER) Input Change Interrupt Enable */
/* -------- PIO_IDR : (PIO Offset: 0x0044) Interrupt Disable Register -------- */
#define PIO_IDR_P0 (0x1u << 0) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P1 (0x1u << 1) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P2 (0x1u << 2) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P3 (0x1u << 3) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P4 (0x1u << 4) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P5 (0x1u << 5) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P6 (0x1u << 6) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P7 (0x1u << 7) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P8 (0x1u << 8) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P9 (0x1u << 9) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P10 (0x1u << 10) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P11 (0x1u << 11) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P12 (0x1u << 12) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P13 (0x1u << 13) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P14 (0x1u << 14) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P15 (0x1u << 15) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P16 (0x1u << 16) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P17 (0x1u << 17) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P18 (0x1u << 18) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P19 (0x1u << 19) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P20 (0x1u << 20) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P21 (0x1u << 21) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P22 (0x1u << 22) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P23 (0x1u << 23) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P24 (0x1u << 24) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P25 (0x1u << 25) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P26 (0x1u << 26) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P27 (0x1u << 27) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P28 (0x1u << 28) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P29 (0x1u << 29) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P30 (0x1u << 30) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
#define PIO_IDR_P31 (0x1u << 31) /**< \brief (PIO_IDR) Input Change Interrupt Disable */
/* -------- PIO_IMR : (PIO Offset: 0x0048) Interrupt Mask Register -------- */
#define PIO_IMR_P0 (0x1u << 0) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P1 (0x1u << 1) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P2 (0x1u << 2) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P3 (0x1u << 3) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P4 (0x1u << 4) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P5 (0x1u << 5) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P6 (0x1u << 6) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P7 (0x1u << 7) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P8 (0x1u << 8) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P9 (0x1u << 9) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P10 (0x1u << 10) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P11 (0x1u << 11) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P12 (0x1u << 12) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P13 (0x1u << 13) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P14 (0x1u << 14) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P15 (0x1u << 15) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P16 (0x1u << 16) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P17 (0x1u << 17) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P18 (0x1u << 18) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P19 (0x1u << 19) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P20 (0x1u << 20) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P21 (0x1u << 21) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P22 (0x1u << 22) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P23 (0x1u << 23) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P24 (0x1u << 24) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P25 (0x1u << 25) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P26 (0x1u << 26) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P27 (0x1u << 27) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P28 (0x1u << 28) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P29 (0x1u << 29) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P30 (0x1u << 30) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
#define PIO_IMR_P31 (0x1u << 31) /**< \brief (PIO_IMR) Input Change Interrupt Mask */
/* -------- PIO_ISR : (PIO Offset: 0x004C) Interrupt Status Register -------- */
#define PIO_ISR_P0 (0x1u << 0) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P1 (0x1u << 1) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P2 (0x1u << 2) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P3 (0x1u << 3) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P4 (0x1u << 4) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P5 (0x1u << 5) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P6 (0x1u << 6) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P7 (0x1u << 7) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P8 (0x1u << 8) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P9 (0x1u << 9) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P10 (0x1u << 10) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P11 (0x1u << 11) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P12 (0x1u << 12) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P13 (0x1u << 13) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P14 (0x1u << 14) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P15 (0x1u << 15) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P16 (0x1u << 16) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P17 (0x1u << 17) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P18 (0x1u << 18) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P19 (0x1u << 19) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P20 (0x1u << 20) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P21 (0x1u << 21) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P22 (0x1u << 22) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P23 (0x1u << 23) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P24 (0x1u << 24) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P25 (0x1u << 25) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P26 (0x1u << 26) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P27 (0x1u << 27) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P28 (0x1u << 28) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P29 (0x1u << 29) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P30 (0x1u << 30) /**< \brief (PIO_ISR) Input Change Interrupt Status */
#define PIO_ISR_P31 (0x1u << 31) /**< \brief (PIO_ISR) Input Change Interrupt Status */
/* -------- PIO_MDER : (PIO Offset: 0x0050) Multi-driver Enable Register -------- */
#define PIO_MDER_P0 (0x1u << 0) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P1 (0x1u << 1) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P2 (0x1u << 2) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P3 (0x1u << 3) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P4 (0x1u << 4) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P5 (0x1u << 5) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P6 (0x1u << 6) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P7 (0x1u << 7) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P8 (0x1u << 8) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P9 (0x1u << 9) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P10 (0x1u << 10) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P11 (0x1u << 11) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P12 (0x1u << 12) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P13 (0x1u << 13) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P14 (0x1u << 14) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P15 (0x1u << 15) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P16 (0x1u << 16) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P17 (0x1u << 17) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P18 (0x1u << 18) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P19 (0x1u << 19) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P20 (0x1u << 20) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P21 (0x1u << 21) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P22 (0x1u << 22) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P23 (0x1u << 23) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P24 (0x1u << 24) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P25 (0x1u << 25) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P26 (0x1u << 26) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P27 (0x1u << 27) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P28 (0x1u << 28) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P29 (0x1u << 29) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P30 (0x1u << 30) /**< \brief (PIO_MDER) Multi Drive Enable. */
#define PIO_MDER_P31 (0x1u << 31) /**< \brief (PIO_MDER) Multi Drive Enable. */
/* -------- PIO_MDDR : (PIO Offset: 0x0054) Multi-driver Disable Register -------- */
#define PIO_MDDR_P0 (0x1u << 0) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P1 (0x1u << 1) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P2 (0x1u << 2) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P3 (0x1u << 3) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P4 (0x1u << 4) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P5 (0x1u << 5) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P6 (0x1u << 6) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P7 (0x1u << 7) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P8 (0x1u << 8) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P9 (0x1u << 9) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P10 (0x1u << 10) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P11 (0x1u << 11) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P12 (0x1u << 12) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P13 (0x1u << 13) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P14 (0x1u << 14) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P15 (0x1u << 15) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P16 (0x1u << 16) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P17 (0x1u << 17) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P18 (0x1u << 18) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P19 (0x1u << 19) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P20 (0x1u << 20) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P21 (0x1u << 21) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P22 (0x1u << 22) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P23 (0x1u << 23) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P24 (0x1u << 24) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P25 (0x1u << 25) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P26 (0x1u << 26) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P27 (0x1u << 27) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P28 (0x1u << 28) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P29 (0x1u << 29) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P30 (0x1u << 30) /**< \brief (PIO_MDDR) Multi Drive Disable. */
#define PIO_MDDR_P31 (0x1u << 31) /**< \brief (PIO_MDDR) Multi Drive Disable. */
/* -------- PIO_MDSR : (PIO Offset: 0x0058) Multi-driver Status Register -------- */
#define PIO_MDSR_P0 (0x1u << 0) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P1 (0x1u << 1) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P2 (0x1u << 2) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P3 (0x1u << 3) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P4 (0x1u << 4) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P5 (0x1u << 5) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P6 (0x1u << 6) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P7 (0x1u << 7) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P8 (0x1u << 8) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P9 (0x1u << 9) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P10 (0x1u << 10) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P11 (0x1u << 11) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P12 (0x1u << 12) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P13 (0x1u << 13) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P14 (0x1u << 14) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P15 (0x1u << 15) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P16 (0x1u << 16) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P17 (0x1u << 17) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P18 (0x1u << 18) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P19 (0x1u << 19) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P20 (0x1u << 20) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P21 (0x1u << 21) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P22 (0x1u << 22) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P23 (0x1u << 23) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P24 (0x1u << 24) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P25 (0x1u << 25) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P26 (0x1u << 26) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P27 (0x1u << 27) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P28 (0x1u << 28) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P29 (0x1u << 29) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P30 (0x1u << 30) /**< \brief (PIO_MDSR) Multi Drive Status. */
#define PIO_MDSR_P31 (0x1u << 31) /**< \brief (PIO_MDSR) Multi Drive Status. */
/* -------- PIO_PUDR : (PIO Offset: 0x0060) Pull-up Disable Register -------- */
#define PIO_PUDR_P0 (0x1u << 0) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P1 (0x1u << 1) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P2 (0x1u << 2) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P3 (0x1u << 3) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P4 (0x1u << 4) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P5 (0x1u << 5) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P6 (0x1u << 6) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P7 (0x1u << 7) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P8 (0x1u << 8) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P9 (0x1u << 9) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P10 (0x1u << 10) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P11 (0x1u << 11) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P12 (0x1u << 12) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P13 (0x1u << 13) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P14 (0x1u << 14) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P15 (0x1u << 15) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P16 (0x1u << 16) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P17 (0x1u << 17) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P18 (0x1u << 18) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P19 (0x1u << 19) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P20 (0x1u << 20) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P21 (0x1u << 21) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P22 (0x1u << 22) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P23 (0x1u << 23) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P24 (0x1u << 24) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P25 (0x1u << 25) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P26 (0x1u << 26) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P27 (0x1u << 27) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P28 (0x1u << 28) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P29 (0x1u << 29) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P30 (0x1u << 30) /**< \brief (PIO_PUDR) Pull Up Disable. */
#define PIO_PUDR_P31 (0x1u << 31) /**< \brief (PIO_PUDR) Pull Up Disable. */
/* -------- PIO_PUER : (PIO Offset: 0x0064) Pull-up Enable Register -------- */
#define PIO_PUER_P0 (0x1u << 0) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P1 (0x1u << 1) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P2 (0x1u << 2) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P3 (0x1u << 3) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P4 (0x1u << 4) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P5 (0x1u << 5) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P6 (0x1u << 6) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P7 (0x1u << 7) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P8 (0x1u << 8) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P9 (0x1u << 9) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P10 (0x1u << 10) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P11 (0x1u << 11) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P12 (0x1u << 12) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P13 (0x1u << 13) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P14 (0x1u << 14) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P15 (0x1u << 15) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P16 (0x1u << 16) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P17 (0x1u << 17) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P18 (0x1u << 18) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P19 (0x1u << 19) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P20 (0x1u << 20) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P21 (0x1u << 21) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P22 (0x1u << 22) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P23 (0x1u << 23) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P24 (0x1u << 24) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P25 (0x1u << 25) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P26 (0x1u << 26) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P27 (0x1u << 27) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P28 (0x1u << 28) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P29 (0x1u << 29) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P30 (0x1u << 30) /**< \brief (PIO_PUER) Pull Up Enable. */
#define PIO_PUER_P31 (0x1u << 31) /**< \brief (PIO_PUER) Pull Up Enable. */
/* -------- PIO_PUSR : (PIO Offset: 0x0068) Pad Pull-up Status Register -------- */
#define PIO_PUSR_P0 (0x1u << 0) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P1 (0x1u << 1) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P2 (0x1u << 2) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P3 (0x1u << 3) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P4 (0x1u << 4) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P5 (0x1u << 5) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P6 (0x1u << 6) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P7 (0x1u << 7) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P8 (0x1u << 8) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P9 (0x1u << 9) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P10 (0x1u << 10) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P11 (0x1u << 11) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P12 (0x1u << 12) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P13 (0x1u << 13) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P14 (0x1u << 14) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P15 (0x1u << 15) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P16 (0x1u << 16) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P17 (0x1u << 17) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P18 (0x1u << 18) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P19 (0x1u << 19) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P20 (0x1u << 20) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P21 (0x1u << 21) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P22 (0x1u << 22) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P23 (0x1u << 23) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P24 (0x1u << 24) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P25 (0x1u << 25) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P26 (0x1u << 26) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P27 (0x1u << 27) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P28 (0x1u << 28) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P29 (0x1u << 29) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P30 (0x1u << 30) /**< \brief (PIO_PUSR) Pull Up Status. */
#define PIO_PUSR_P31 (0x1u << 31) /**< \brief (PIO_PUSR) Pull Up Status. */
/* -------- PIO_ABSR : (PIO Offset: 0x0070) Peripheral Select Register -------- */
#define PIO_ABSR_P0 (0x1u << 0) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P1 (0x1u << 1) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P2 (0x1u << 2) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P3 (0x1u << 3) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P4 (0x1u << 4) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P5 (0x1u << 5) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P6 (0x1u << 6) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P7 (0x1u << 7) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P8 (0x1u << 8) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P9 (0x1u << 9) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P10 (0x1u << 10) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P11 (0x1u << 11) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P12 (0x1u << 12) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P13 (0x1u << 13) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P14 (0x1u << 14) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P15 (0x1u << 15) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P16 (0x1u << 16) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P17 (0x1u << 17) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P18 (0x1u << 18) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P19 (0x1u << 19) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P20 (0x1u << 20) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P21 (0x1u << 21) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P22 (0x1u << 22) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P23 (0x1u << 23) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P24 (0x1u << 24) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P25 (0x1u << 25) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P26 (0x1u << 26) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P27 (0x1u << 27) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P28 (0x1u << 28) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P29 (0x1u << 29) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P30 (0x1u << 30) /**< \brief (PIO_ABSR) Peripheral Select. */
#define PIO_ABSR_P31 (0x1u << 31) /**< \brief (PIO_ABSR) Peripheral Select. */
/* -------- PIO_IFSCDR : (PIO Offset: 0x0080) Input Filter Slow Clock Disable Register -------- */
#define PIO_IFSCDR_P0 (0x1u << 0) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P1 (0x1u << 1) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P2 (0x1u << 2) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P3 (0x1u << 3) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P4 (0x1u << 4) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P5 (0x1u << 5) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P6 (0x1u << 6) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P7 (0x1u << 7) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P8 (0x1u << 8) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P9 (0x1u << 9) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P10 (0x1u << 10) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P11 (0x1u << 11) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P12 (0x1u << 12) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P13 (0x1u << 13) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P14 (0x1u << 14) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P15 (0x1u << 15) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P16 (0x1u << 16) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P17 (0x1u << 17) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P18 (0x1u << 18) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P19 (0x1u << 19) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P20 (0x1u << 20) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P21 (0x1u << 21) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P22 (0x1u << 22) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P23 (0x1u << 23) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P24 (0x1u << 24) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P25 (0x1u << 25) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P26 (0x1u << 26) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P27 (0x1u << 27) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P28 (0x1u << 28) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P29 (0x1u << 29) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P30 (0x1u << 30) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
#define PIO_IFSCDR_P31 (0x1u << 31) /**< \brief (PIO_IFSCDR) PIO Clock Glitch Filtering Select. */
/* -------- PIO_IFSCER : (PIO Offset: 0x0084) Input Filter Slow Clock Enable Register -------- */
#define PIO_IFSCER_P0 (0x1u << 0) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P1 (0x1u << 1) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P2 (0x1u << 2) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P3 (0x1u << 3) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P4 (0x1u << 4) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P5 (0x1u << 5) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P6 (0x1u << 6) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P7 (0x1u << 7) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P8 (0x1u << 8) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P9 (0x1u << 9) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P10 (0x1u << 10) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P11 (0x1u << 11) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P12 (0x1u << 12) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P13 (0x1u << 13) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P14 (0x1u << 14) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P15 (0x1u << 15) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P16 (0x1u << 16) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P17 (0x1u << 17) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P18 (0x1u << 18) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P19 (0x1u << 19) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P20 (0x1u << 20) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P21 (0x1u << 21) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P22 (0x1u << 22) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P23 (0x1u << 23) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P24 (0x1u << 24) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P25 (0x1u << 25) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P26 (0x1u << 26) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P27 (0x1u << 27) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P28 (0x1u << 28) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P29 (0x1u << 29) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P30 (0x1u << 30) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
#define PIO_IFSCER_P31 (0x1u << 31) /**< \brief (PIO_IFSCER) Debouncing Filtering Select. */
/* -------- PIO_IFSCSR : (PIO Offset: 0x0088) Input Filter Slow Clock Status Register -------- */
#define PIO_IFSCSR_P0 (0x1u << 0) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P1 (0x1u << 1) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P2 (0x1u << 2) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P3 (0x1u << 3) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P4 (0x1u << 4) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P5 (0x1u << 5) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P6 (0x1u << 6) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P7 (0x1u << 7) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P8 (0x1u << 8) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P9 (0x1u << 9) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P10 (0x1u << 10) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P11 (0x1u << 11) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P12 (0x1u << 12) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P13 (0x1u << 13) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P14 (0x1u << 14) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P15 (0x1u << 15) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P16 (0x1u << 16) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P17 (0x1u << 17) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P18 (0x1u << 18) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P19 (0x1u << 19) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P20 (0x1u << 20) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P21 (0x1u << 21) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P22 (0x1u << 22) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P23 (0x1u << 23) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P24 (0x1u << 24) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P25 (0x1u << 25) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P26 (0x1u << 26) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P27 (0x1u << 27) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P28 (0x1u << 28) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P29 (0x1u << 29) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P30 (0x1u << 30) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
#define PIO_IFSCSR_P31 (0x1u << 31) /**< \brief (PIO_IFSCSR) Glitch or Debouncing Filter Selection Status */
/* -------- PIO_SCDR : (PIO Offset: 0x008C) Slow Clock Divider Debouncing Register -------- */
#define PIO_SCDR_DIV0 (0x1u << 0) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
#define PIO_SCDR_DIV1 (0x1u << 1) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
#define PIO_SCDR_DIV2 (0x1u << 2) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
#define PIO_SCDR_DIV3 (0x1u << 3) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
#define PIO_SCDR_DIV4 (0x1u << 4) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
#define PIO_SCDR_DIV5 (0x1u << 5) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
#define PIO_SCDR_DIV6 (0x1u << 6) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
#define PIO_SCDR_DIV7 (0x1u << 7) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
#define PIO_SCDR_DIV8 (0x1u << 8) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
#define PIO_SCDR_DIV9 (0x1u << 9) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
#define PIO_SCDR_DIV10 (0x1u << 10) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
#define PIO_SCDR_DIV11 (0x1u << 11) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
#define PIO_SCDR_DIV12 (0x1u << 12) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
#define PIO_SCDR_DIV13 (0x1u << 13) /**< \brief (PIO_SCDR) Slow Clock Divider Selection for Debouncing */
/* -------- PIO_PPDDR : (PIO Offset: 0x0090) Pad Pull-down Disable Register -------- */
#define PIO_PPDDR_P0 (0x1u << 0) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P1 (0x1u << 1) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P2 (0x1u << 2) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P3 (0x1u << 3) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P4 (0x1u << 4) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P5 (0x1u << 5) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P6 (0x1u << 6) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P7 (0x1u << 7) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P8 (0x1u << 8) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P9 (0x1u << 9) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P10 (0x1u << 10) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P11 (0x1u << 11) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P12 (0x1u << 12) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P13 (0x1u << 13) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P14 (0x1u << 14) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P15 (0x1u << 15) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P16 (0x1u << 16) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P17 (0x1u << 17) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P18 (0x1u << 18) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P19 (0x1u << 19) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P20 (0x1u << 20) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P21 (0x1u << 21) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P22 (0x1u << 22) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P23 (0x1u << 23) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P24 (0x1u << 24) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P25 (0x1u << 25) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P26 (0x1u << 26) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P27 (0x1u << 27) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P28 (0x1u << 28) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P29 (0x1u << 29) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P30 (0x1u << 30) /**< \brief (PIO_PPDDR) Pull Down Disable. */
#define PIO_PPDDR_P31 (0x1u << 31) /**< \brief (PIO_PPDDR) Pull Down Disable. */
/* -------- PIO_PPDER : (PIO Offset: 0x0094) Pad Pull-down Enable Register -------- */
#define PIO_PPDER_P0 (0x1u << 0) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P1 (0x1u << 1) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P2 (0x1u << 2) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P3 (0x1u << 3) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P4 (0x1u << 4) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P5 (0x1u << 5) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P6 (0x1u << 6) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P7 (0x1u << 7) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P8 (0x1u << 8) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P9 (0x1u << 9) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P10 (0x1u << 10) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P11 (0x1u << 11) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P12 (0x1u << 12) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P13 (0x1u << 13) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P14 (0x1u << 14) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P15 (0x1u << 15) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P16 (0x1u << 16) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P17 (0x1u << 17) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P18 (0x1u << 18) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P19 (0x1u << 19) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P20 (0x1u << 20) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P21 (0x1u << 21) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P22 (0x1u << 22) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P23 (0x1u << 23) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P24 (0x1u << 24) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P25 (0x1u << 25) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P26 (0x1u << 26) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P27 (0x1u << 27) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P28 (0x1u << 28) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P29 (0x1u << 29) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P30 (0x1u << 30) /**< \brief (PIO_PPDER) Pull Down Enable. */
#define PIO_PPDER_P31 (0x1u << 31) /**< \brief (PIO_PPDER) Pull Down Enable. */
/* -------- PIO_PPDSR : (PIO Offset: 0x0098) Pad Pull-down Status Register -------- */
#define PIO_PPDSR_P0 (0x1u << 0) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P1 (0x1u << 1) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P2 (0x1u << 2) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P3 (0x1u << 3) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P4 (0x1u << 4) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P5 (0x1u << 5) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P6 (0x1u << 6) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P7 (0x1u << 7) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P8 (0x1u << 8) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P9 (0x1u << 9) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P10 (0x1u << 10) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P11 (0x1u << 11) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P12 (0x1u << 12) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P13 (0x1u << 13) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P14 (0x1u << 14) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P15 (0x1u << 15) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P16 (0x1u << 16) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P17 (0x1u << 17) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P18 (0x1u << 18) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P19 (0x1u << 19) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P20 (0x1u << 20) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P21 (0x1u << 21) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P22 (0x1u << 22) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P23 (0x1u << 23) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P24 (0x1u << 24) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P25 (0x1u << 25) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P26 (0x1u << 26) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P27 (0x1u << 27) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P28 (0x1u << 28) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P29 (0x1u << 29) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P30 (0x1u << 30) /**< \brief (PIO_PPDSR) Pull Down Status. */
#define PIO_PPDSR_P31 (0x1u << 31) /**< \brief (PIO_PPDSR) Pull Down Status. */
/* -------- PIO_OWER : (PIO Offset: 0x00A0) Output Write Enable -------- */
#define PIO_OWER_P0 (0x1u << 0) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P1 (0x1u << 1) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P2 (0x1u << 2) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P3 (0x1u << 3) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P4 (0x1u << 4) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P5 (0x1u << 5) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P6 (0x1u << 6) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P7 (0x1u << 7) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P8 (0x1u << 8) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P9 (0x1u << 9) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P10 (0x1u << 10) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P11 (0x1u << 11) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P12 (0x1u << 12) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P13 (0x1u << 13) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P14 (0x1u << 14) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P15 (0x1u << 15) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P16 (0x1u << 16) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P17 (0x1u << 17) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P18 (0x1u << 18) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P19 (0x1u << 19) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P20 (0x1u << 20) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P21 (0x1u << 21) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P22 (0x1u << 22) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P23 (0x1u << 23) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P24 (0x1u << 24) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P25 (0x1u << 25) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P26 (0x1u << 26) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P27 (0x1u << 27) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P28 (0x1u << 28) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P29 (0x1u << 29) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P30 (0x1u << 30) /**< \brief (PIO_OWER) Output Write Enable. */
#define PIO_OWER_P31 (0x1u << 31) /**< \brief (PIO_OWER) Output Write Enable. */
/* -------- PIO_OWDR : (PIO Offset: 0x00A4) Output Write Disable -------- */
#define PIO_OWDR_P0 (0x1u << 0) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P1 (0x1u << 1) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P2 (0x1u << 2) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P3 (0x1u << 3) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P4 (0x1u << 4) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P5 (0x1u << 5) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P6 (0x1u << 6) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P7 (0x1u << 7) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P8 (0x1u << 8) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P9 (0x1u << 9) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P10 (0x1u << 10) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P11 (0x1u << 11) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P12 (0x1u << 12) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P13 (0x1u << 13) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P14 (0x1u << 14) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P15 (0x1u << 15) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P16 (0x1u << 16) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P17 (0x1u << 17) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P18 (0x1u << 18) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P19 (0x1u << 19) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P20 (0x1u << 20) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P21 (0x1u << 21) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P22 (0x1u << 22) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P23 (0x1u << 23) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P24 (0x1u << 24) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P25 (0x1u << 25) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P26 (0x1u << 26) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P27 (0x1u << 27) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P28 (0x1u << 28) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P29 (0x1u << 29) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P30 (0x1u << 30) /**< \brief (PIO_OWDR) Output Write Disable. */
#define PIO_OWDR_P31 (0x1u << 31) /**< \brief (PIO_OWDR) Output Write Disable. */
/* -------- PIO_OWSR : (PIO Offset: 0x00A8) Output Write Status Register -------- */
#define PIO_OWSR_P0 (0x1u << 0) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P1 (0x1u << 1) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P2 (0x1u << 2) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P3 (0x1u << 3) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P4 (0x1u << 4) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P5 (0x1u << 5) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P6 (0x1u << 6) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P7 (0x1u << 7) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P8 (0x1u << 8) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P9 (0x1u << 9) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P10 (0x1u << 10) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P11 (0x1u << 11) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P12 (0x1u << 12) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P13 (0x1u << 13) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P14 (0x1u << 14) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P15 (0x1u << 15) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P16 (0x1u << 16) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P17 (0x1u << 17) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P18 (0x1u << 18) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P19 (0x1u << 19) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P20 (0x1u << 20) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P21 (0x1u << 21) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P22 (0x1u << 22) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P23 (0x1u << 23) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P24 (0x1u << 24) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P25 (0x1u << 25) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P26 (0x1u << 26) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P27 (0x1u << 27) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P28 (0x1u << 28) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P29 (0x1u << 29) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P30 (0x1u << 30) /**< \brief (PIO_OWSR) Output Write Status. */
#define PIO_OWSR_P31 (0x1u << 31) /**< \brief (PIO_OWSR) Output Write Status. */
/* -------- PIO_AIMER : (PIO Offset: 0x00B0) Additional Interrupt Modes Enable Register -------- */
#define PIO_AIMER_P0 (0x1u << 0) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P1 (0x1u << 1) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P2 (0x1u << 2) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P3 (0x1u << 3) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P4 (0x1u << 4) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P5 (0x1u << 5) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P6 (0x1u << 6) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P7 (0x1u << 7) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P8 (0x1u << 8) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P9 (0x1u << 9) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P10 (0x1u << 10) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P11 (0x1u << 11) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P12 (0x1u << 12) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P13 (0x1u << 13) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P14 (0x1u << 14) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P15 (0x1u << 15) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P16 (0x1u << 16) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P17 (0x1u << 17) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P18 (0x1u << 18) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P19 (0x1u << 19) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P20 (0x1u << 20) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P21 (0x1u << 21) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P22 (0x1u << 22) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P23 (0x1u << 23) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P24 (0x1u << 24) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P25 (0x1u << 25) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P26 (0x1u << 26) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P27 (0x1u << 27) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P28 (0x1u << 28) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P29 (0x1u << 29) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P30 (0x1u << 30) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
#define PIO_AIMER_P31 (0x1u << 31) /**< \brief (PIO_AIMER) Additional Interrupt Modes Enable. */
/* -------- PIO_AIMDR : (PIO Offset: 0x00B4) Additional Interrupt Modes Disables Register -------- */
#define PIO_AIMDR_P0 (0x1u << 0) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P1 (0x1u << 1) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P2 (0x1u << 2) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P3 (0x1u << 3) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P4 (0x1u << 4) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P5 (0x1u << 5) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P6 (0x1u << 6) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P7 (0x1u << 7) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P8 (0x1u << 8) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P9 (0x1u << 9) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P10 (0x1u << 10) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P11 (0x1u << 11) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P12 (0x1u << 12) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P13 (0x1u << 13) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P14 (0x1u << 14) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P15 (0x1u << 15) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P16 (0x1u << 16) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P17 (0x1u << 17) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P18 (0x1u << 18) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P19 (0x1u << 19) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P20 (0x1u << 20) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P21 (0x1u << 21) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P22 (0x1u << 22) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P23 (0x1u << 23) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P24 (0x1u << 24) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P25 (0x1u << 25) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P26 (0x1u << 26) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P27 (0x1u << 27) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P28 (0x1u << 28) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P29 (0x1u << 29) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P30 (0x1u << 30) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
#define PIO_AIMDR_P31 (0x1u << 31) /**< \brief (PIO_AIMDR) Additional Interrupt Modes Disable. */
/* -------- PIO_AIMMR : (PIO Offset: 0x00B8) Additional Interrupt Modes Mask Register -------- */
#define PIO_AIMMR_P0 (0x1u << 0) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P1 (0x1u << 1) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P2 (0x1u << 2) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P3 (0x1u << 3) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P4 (0x1u << 4) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P5 (0x1u << 5) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P6 (0x1u << 6) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P7 (0x1u << 7) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P8 (0x1u << 8) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P9 (0x1u << 9) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P10 (0x1u << 10) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P11 (0x1u << 11) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P12 (0x1u << 12) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P13 (0x1u << 13) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P14 (0x1u << 14) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P15 (0x1u << 15) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P16 (0x1u << 16) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P17 (0x1u << 17) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P18 (0x1u << 18) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P19 (0x1u << 19) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P20 (0x1u << 20) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P21 (0x1u << 21) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P22 (0x1u << 22) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P23 (0x1u << 23) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P24 (0x1u << 24) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P25 (0x1u << 25) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P26 (0x1u << 26) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P27 (0x1u << 27) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P28 (0x1u << 28) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P29 (0x1u << 29) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P30 (0x1u << 30) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
#define PIO_AIMMR_P31 (0x1u << 31) /**< \brief (PIO_AIMMR) Peripheral CD Status. */
/* -------- PIO_ESR : (PIO Offset: 0x00C0) Edge Select Register -------- */
#define PIO_ESR_P0 (0x1u << 0) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P1 (0x1u << 1) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P2 (0x1u << 2) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P3 (0x1u << 3) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P4 (0x1u << 4) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P5 (0x1u << 5) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P6 (0x1u << 6) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P7 (0x1u << 7) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P8 (0x1u << 8) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P9 (0x1u << 9) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P10 (0x1u << 10) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P11 (0x1u << 11) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P12 (0x1u << 12) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P13 (0x1u << 13) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P14 (0x1u << 14) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P15 (0x1u << 15) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P16 (0x1u << 16) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P17 (0x1u << 17) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P18 (0x1u << 18) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P19 (0x1u << 19) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P20 (0x1u << 20) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P21 (0x1u << 21) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P22 (0x1u << 22) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P23 (0x1u << 23) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P24 (0x1u << 24) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P25 (0x1u << 25) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P26 (0x1u << 26) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P27 (0x1u << 27) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P28 (0x1u << 28) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P29 (0x1u << 29) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P30 (0x1u << 30) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
#define PIO_ESR_P31 (0x1u << 31) /**< \brief (PIO_ESR) Edge Interrupt Selection. */
/* -------- PIO_LSR : (PIO Offset: 0x00C4) Level Select Register -------- */
#define PIO_LSR_P0 (0x1u << 0) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P1 (0x1u << 1) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P2 (0x1u << 2) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P3 (0x1u << 3) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P4 (0x1u << 4) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P5 (0x1u << 5) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P6 (0x1u << 6) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P7 (0x1u << 7) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P8 (0x1u << 8) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P9 (0x1u << 9) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P10 (0x1u << 10) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P11 (0x1u << 11) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P12 (0x1u << 12) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P13 (0x1u << 13) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P14 (0x1u << 14) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P15 (0x1u << 15) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P16 (0x1u << 16) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P17 (0x1u << 17) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P18 (0x1u << 18) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P19 (0x1u << 19) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P20 (0x1u << 20) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P21 (0x1u << 21) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P22 (0x1u << 22) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P23 (0x1u << 23) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P24 (0x1u << 24) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P25 (0x1u << 25) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P26 (0x1u << 26) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P27 (0x1u << 27) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P28 (0x1u << 28) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P29 (0x1u << 29) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P30 (0x1u << 30) /**< \brief (PIO_LSR) Level Interrupt Selection. */
#define PIO_LSR_P31 (0x1u << 31) /**< \brief (PIO_LSR) Level Interrupt Selection. */
/* -------- PIO_ELSR : (PIO Offset: 0x00C8) Edge/Level Status Register -------- */
#define PIO_ELSR_P0 (0x1u << 0) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P1 (0x1u << 1) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P2 (0x1u << 2) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P3 (0x1u << 3) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P4 (0x1u << 4) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P5 (0x1u << 5) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P6 (0x1u << 6) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P7 (0x1u << 7) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P8 (0x1u << 8) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P9 (0x1u << 9) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P10 (0x1u << 10) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P11 (0x1u << 11) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P12 (0x1u << 12) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P13 (0x1u << 13) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P14 (0x1u << 14) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P15 (0x1u << 15) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P16 (0x1u << 16) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P17 (0x1u << 17) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P18 (0x1u << 18) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P19 (0x1u << 19) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P20 (0x1u << 20) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P21 (0x1u << 21) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P22 (0x1u << 22) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P23 (0x1u << 23) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P24 (0x1u << 24) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P25 (0x1u << 25) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P26 (0x1u << 26) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P27 (0x1u << 27) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P28 (0x1u << 28) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P29 (0x1u << 29) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P30 (0x1u << 30) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
#define PIO_ELSR_P31 (0x1u << 31) /**< \brief (PIO_ELSR) Edge/Level Interrupt source selection. */
/* -------- PIO_FELLSR : (PIO Offset: 0x00D0) Falling Edge/Low Level Select Register -------- */
#define PIO_FELLSR_P0 (0x1u << 0) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P1 (0x1u << 1) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P2 (0x1u << 2) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P3 (0x1u << 3) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P4 (0x1u << 4) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P5 (0x1u << 5) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P6 (0x1u << 6) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P7 (0x1u << 7) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P8 (0x1u << 8) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P9 (0x1u << 9) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P10 (0x1u << 10) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P11 (0x1u << 11) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P12 (0x1u << 12) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P13 (0x1u << 13) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P14 (0x1u << 14) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P15 (0x1u << 15) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P16 (0x1u << 16) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P17 (0x1u << 17) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P18 (0x1u << 18) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P19 (0x1u << 19) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P20 (0x1u << 20) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P21 (0x1u << 21) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P22 (0x1u << 22) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P23 (0x1u << 23) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P24 (0x1u << 24) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P25 (0x1u << 25) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P26 (0x1u << 26) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P27 (0x1u << 27) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P28 (0x1u << 28) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P29 (0x1u << 29) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P30 (0x1u << 30) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
#define PIO_FELLSR_P31 (0x1u << 31) /**< \brief (PIO_FELLSR) Falling Edge/Low Level Interrupt Selection. */
/* -------- PIO_REHLSR : (PIO Offset: 0x00D4) Rising Edge/ High Level Select Register -------- */
#define PIO_REHLSR_P0 (0x1u << 0) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P1 (0x1u << 1) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P2 (0x1u << 2) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P3 (0x1u << 3) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P4 (0x1u << 4) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P5 (0x1u << 5) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P6 (0x1u << 6) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P7 (0x1u << 7) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P8 (0x1u << 8) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P9 (0x1u << 9) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P10 (0x1u << 10) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P11 (0x1u << 11) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P12 (0x1u << 12) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P13 (0x1u << 13) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P14 (0x1u << 14) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P15 (0x1u << 15) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P16 (0x1u << 16) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P17 (0x1u << 17) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P18 (0x1u << 18) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P19 (0x1u << 19) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P20 (0x1u << 20) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P21 (0x1u << 21) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P22 (0x1u << 22) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P23 (0x1u << 23) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P24 (0x1u << 24) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P25 (0x1u << 25) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P26 (0x1u << 26) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P27 (0x1u << 27) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P28 (0x1u << 28) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P29 (0x1u << 29) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P30 (0x1u << 30) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
#define PIO_REHLSR_P31 (0x1u << 31) /**< \brief (PIO_REHLSR) Rising Edge /High Level Interrupt Selection. */
/* -------- PIO_FRLHSR : (PIO Offset: 0x00D8) Fall/Rise - Low/High Status Register -------- */
#define PIO_FRLHSR_P0 (0x1u << 0) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P1 (0x1u << 1) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P2 (0x1u << 2) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P3 (0x1u << 3) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P4 (0x1u << 4) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P5 (0x1u << 5) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P6 (0x1u << 6) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P7 (0x1u << 7) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P8 (0x1u << 8) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P9 (0x1u << 9) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P10 (0x1u << 10) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P11 (0x1u << 11) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P12 (0x1u << 12) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P13 (0x1u << 13) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P14 (0x1u << 14) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P15 (0x1u << 15) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P16 (0x1u << 16) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P17 (0x1u << 17) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P18 (0x1u << 18) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P19 (0x1u << 19) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P20 (0x1u << 20) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P21 (0x1u << 21) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P22 (0x1u << 22) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P23 (0x1u << 23) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P24 (0x1u << 24) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P25 (0x1u << 25) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P26 (0x1u << 26) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P27 (0x1u << 27) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P28 (0x1u << 28) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P29 (0x1u << 29) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P30 (0x1u << 30) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
#define PIO_FRLHSR_P31 (0x1u << 31) /**< \brief (PIO_FRLHSR) Edge /Level Interrupt Source Selection. */
/* -------- PIO_LOCKSR : (PIO Offset: 0x00E0) Lock Status -------- */
#define PIO_LOCKSR_P0 (0x1u << 0) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P1 (0x1u << 1) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P2 (0x1u << 2) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P3 (0x1u << 3) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P4 (0x1u << 4) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P5 (0x1u << 5) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P6 (0x1u << 6) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P7 (0x1u << 7) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P8 (0x1u << 8) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P9 (0x1u << 9) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P10 (0x1u << 10) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P11 (0x1u << 11) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P12 (0x1u << 12) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P13 (0x1u << 13) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P14 (0x1u << 14) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P15 (0x1u << 15) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P16 (0x1u << 16) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P17 (0x1u << 17) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P18 (0x1u << 18) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P19 (0x1u << 19) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P20 (0x1u << 20) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P21 (0x1u << 21) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P22 (0x1u << 22) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P23 (0x1u << 23) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P24 (0x1u << 24) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P25 (0x1u << 25) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P26 (0x1u << 26) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P27 (0x1u << 27) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P28 (0x1u << 28) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P29 (0x1u << 29) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P30 (0x1u << 30) /**< \brief (PIO_LOCKSR) Lock Status. */
#define PIO_LOCKSR_P31 (0x1u << 31) /**< \brief (PIO_LOCKSR) Lock Status. */
/* -------- PIO_WPMR : (PIO Offset: 0x00E4) Write Protect Mode Register -------- */
#define PIO_WPMR_WPEN (0x1u << 0) /**< \brief (PIO_WPMR) Write Protect Enable */
#define PIO_WPMR_WPKEY_Pos 8
#define PIO_WPMR_WPKEY_Msk (0xffffffu << PIO_WPMR_WPKEY_Pos) /**< \brief (PIO_WPMR) Write Protect KEY */
#define PIO_WPMR_WPKEY(value) ((PIO_WPMR_WPKEY_Msk & ((value) << PIO_WPMR_WPKEY_Pos)))
/* -------- PIO_WPSR : (PIO Offset: 0x00E8) Write Protect Status Register -------- */
#define PIO_WPSR_WPVS (0x1u << 0) /**< \brief (PIO_WPSR) Write Protect Violation Status */
#define PIO_WPSR_WPVSRC_Pos 8
#define PIO_WPSR_WPVSRC_Msk (0xffffu << PIO_WPSR_WPVSRC_Pos) /**< \brief (PIO_WPSR) Write Protect Violation Source */

/*@}*/

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Supply Controller */
/* ============================================================================= */
/** \addtogroup SAM3S_SUPC Supply Controller */
/*@{*/


#ifndef __ASSEMBLY__
/** \brief Supc hardware registers */
typedef struct {
  WoReg SUPC_CR;   /**< \brief (Supc Offset: 0x00) Supply Controller Control Register */
  RwReg SUPC_SMMR; /**< \brief (Supc Offset: 0x04) Supply Controller Supply Monitor Mode Register */
  RwReg SUPC_MR;   /**< \brief (Supc Offset: 0x08) Supply Controller Mode Register */
  RwReg SUPC_WUMR; /**< \brief (Supc Offset: 0x0C) Supply Controller Wake Up Mode Register */
  RwReg SUPC_WUIR; /**< \brief (Supc Offset: 0x10) Supply Controller Wake Up Inputs Register */
  RoReg SUPC_SR;   /**< \brief (Supc Offset: 0x14) Supply Controller Status Register */
} Supc;
#endif /* __ASSEMBLY__ */
/* -------- SUPC_CR : (SUPC Offset: 0x00) Supply Controller Control Register -------- */
#define SUPC_CR_VROFF (0x1u << 2) /**< \brief (SUPC_CR) Voltage Regulator Off */
#define   SUPC_CR_VROFF_NO_EFFECT (0x0u << 2) /**< \brief (SUPC_CR) no effect. */
#define   SUPC_CR_VROFF_STOP_VREG (0x1u << 2) /**< \brief (SUPC_CR) if KEY is correct, asserts vddcore_nreset and stops the voltage regulator. */
#define SUPC_CR_XTALSEL (0x1u << 3) /**< \brief (SUPC_CR) Crystal Oscillator Select */
#define   SUPC_CR_XTALSEL_NO_EFFECT (0x0u << 3) /**< \brief (SUPC_CR) no effect. */
#define   SUPC_CR_XTALSEL_CRYSTAL_SEL (0x1u << 3) /**< \brief (SUPC_CR) if KEY is correct, switches the slow clock on the crystal oscillator output. */
#define SUPC_CR_KEY_Pos 24
#define SUPC_CR_KEY_Msk (0xffu << SUPC_CR_KEY_Pos) /**< \brief (SUPC_CR) Password */
#define SUPC_CR_KEY(value) ((uint32_t)(SUPC_CR_KEY_Msk & ((value) << SUPC_CR_KEY_Pos)))
/* -------- SUPC_SMMR : (SUPC Offset: 0x04) Supply Controller Supply Monitor Mode Register -------- */
#define SUPC_SMMR_SMTH_Pos 0
#define SUPC_SMMR_SMTH_Msk (0xfu << SUPC_SMMR_SMTH_Pos) /**< \brief (SUPC_SMMR) Supply Monitor Threshold */
#define   SUPC_SMMR_SMTH_1_9V (0x0u << 0) /**< \brief (SUPC_SMMR) 1.9 V */
#define   SUPC_SMMR_SMTH_2_0V (0x1u << 0) /**< \brief (SUPC_SMMR) 2.0 V */
#define   SUPC_SMMR_SMTH_2_1V (0x2u << 0) /**< \brief (SUPC_SMMR) 2.1 V */
#define   SUPC_SMMR_SMTH_2_2V (0x3u << 0) /**< \brief (SUPC_SMMR) 2.2 V */
#define   SUPC_SMMR_SMTH_2_3V (0x4u << 0) /**< \brief (SUPC_SMMR) 2.3 V */
#define   SUPC_SMMR_SMTH_2_4V (0x5u << 0) /**< \brief (SUPC_SMMR) 2.4 V */
#define   SUPC_SMMR_SMTH_2_5V (0x6u << 0) /**< \brief (SUPC_SMMR) 2.5 V */
#define   SUPC_SMMR_SMTH_2_6V (0x7u << 0) /**< \brief (SUPC_SMMR) 2.6 V */
#define   SUPC_SMMR_SMTH_2_7V (0x8u << 0) /**< \brief (SUPC_SMMR) 2.7 V */
#define   SUPC_SMMR_SMTH_2_8V (0x9u << 0) /**< \brief (SUPC_SMMR) 2.8 V */
#define   SUPC_SMMR_SMTH_2_9V (0xAu << 0) /**< \brief (SUPC_SMMR) 2.9 V */
#define   SUPC_SMMR_SMTH_3_0V (0xBu << 0) /**< \brief (SUPC_SMMR) 3.0 V */
#define   SUPC_SMMR_SMTH_3_1V (0xCu << 0) /**< \brief (SUPC_SMMR) 3.1 V */
#define   SUPC_SMMR_SMTH_3_2V (0xDu << 0) /**< \brief (SUPC_SMMR) 3.2 V */
#define   SUPC_SMMR_SMTH_3_3V (0xEu << 0) /**< \brief (SUPC_SMMR) 3.3 V */
#define   SUPC_SMMR_SMTH_3_4V (0xFu << 0) /**< \brief (SUPC_SMMR) 3.4 V */
#define SUPC_SMMR_SMSMPL_Pos 8
#define SUPC_SMMR_SMSMPL_Msk (0x7u << SUPC_SMMR_SMSMPL_Pos) /**< \brief (SUPC_SMMR) Supply Monitor Sampling Period */
#define   SUPC_SMMR_SMSMPL_SMD (0x0u << 8) /**< \brief (SUPC_SMMR) Supply Monitor disabled */
#define   SUPC_SMMR_SMSMPL_CSM (0x1u << 8) /**< \brief (SUPC_SMMR) Continuous Supply Monitor */
#define   SUPC_SMMR_SMSMPL_32SLCK (0x2u << 8) /**< \brief (SUPC_SMMR) Supply Monitor enabled one SLCK period every 32 SLCK periods */
#define   SUPC_SMMR_SMSMPL_256SLCK (0x3u << 8) /**< \brief (SUPC_SMMR) Supply Monitor enabled one SLCK period every 256 SLCK periods */
#define   SUPC_SMMR_SMSMPL_2048SLCK (0x4u << 8) /**< \brief (SUPC_SMMR) Supply Monitor enabled one SLCK period every 2,048 SLCK periods */
#define SUPC_SMMR_SMRSTEN (0x1u << 12) /**< \brief (SUPC_SMMR) Supply Monitor Reset Enable */
#define   SUPC_SMMR_SMRSTEN_NOT_ENABLE (0x0u << 12) /**< \brief (SUPC_SMMR) the core reset signal "vddcore_nreset" is not affected when a supply monitor detection occurs. */
#define   SUPC_SMMR_SMRSTEN_ENABLE (0x1u << 12) /**< \brief (SUPC_SMMR) the core reset signal, vddcore_nreset is asserted when a supply monitor detection occurs. */
#define SUPC_SMMR_SMIEN (0x1u << 13) /**< \brief (SUPC_SMMR) Supply Monitor Interrupt Enable */
#define   SUPC_SMMR_SMIEN_NOT_ENABLE (0x0u << 13) /**< \brief (SUPC_SMMR) the SUPC interrupt signal is not affected when a supply monitor detection occurs. */
#define   SUPC_SMMR_SMIEN_ENABLE (0x1u << 13) /**< \brief (SUPC_SMMR) the SUPC interrupt signal is asserted when a supply monitor detection occurs. */
/* -------- SUPC_MR : (SUPC Offset: 0x08) Supply Controller Mode Register -------- */
#define SUPC_MR_BODRSTEN (0x1u << 12) /**< \brief (SUPC_MR) Brownout Detector Reset Enable */
#define   SUPC_MR_BODRSTEN_NOT_ENABLE (0x0u << 12) /**< \brief (SUPC_MR) the core reset signal "vddcore_nreset" is not affected when a brownout detection occurs. */
#define   SUPC_MR_BODRSTEN_ENABLE (0x1u << 12) /**< \brief (SUPC_MR) the core reset signal, vddcore_nreset is asserted when a brownout detection occurs. */
#define SUPC_MR_BODDIS (0x1u << 13) /**< \brief (SUPC_MR) Brownout Detector Disable */
#define   SUPC_MR_BODDIS_ENABLE (0x0u << 13) /**< \brief (SUPC_MR) the core brownout detector is enabled. */
#define   SUPC_MR_BODDIS_DISABLE (0x1u << 13) /**< \brief (SUPC_MR) the core brownout detector is disabled. */
#define SUPC_MR_ONREG (0x1u << 14) /**< \brief (SUPC_MR) Voltage Regulator enable */
#define   SUPC_MR_ONREG_ONREG_UNUSED (0x0u << 14) /**< \brief (SUPC_MR) Voltage Regulator is not used */
#define   SUPC_MR_ONREG_ONREG_USED (0x1u << 14) /**< \brief (SUPC_MR) Voltage Regulator is used */
#define SUPC_MR_OSCBYPASS (0x1u << 20) /**< \brief (SUPC_MR) Oscillator Bypass */
#define   SUPC_MR_OSCBYPASS_NO_EFFECT (0x0u << 20) /**< \brief (SUPC_MR) no effect. Clock selection depends on XTALSEL value. */
#define   SUPC_MR_OSCBYPASS_BYPASS (0x1u << 20) /**< \brief (SUPC_MR) the 32-KHz XTAL oscillator is selected and is put in bypass mode. */
#define SUPC_MR_KEY_Pos 24
#define SUPC_MR_KEY_Msk (0xffu << SUPC_MR_KEY_Pos) /**< \brief (SUPC_MR) Password Key */
#define SUPC_MR_KEY(value) ((SUPC_MR_KEY_Msk & ((value) << SUPC_MR_KEY_Pos)))
/* -------- SUPC_WUMR : (SUPC Offset: 0x0C) Supply Controller Wake Up Mode Register -------- */
#define SUPC_WUMR_SMEN (0x1u << 1) /**< \brief (SUPC_WUMR) Supply Monitor Wake Up Enable */
#define   SUPC_WUMR_SMEN_NOT_ENABLE (0x0u << 1) /**< \brief (SUPC_WUMR) the supply monitor detection has no wake up effect. */
#define   SUPC_WUMR_SMEN_ENABLE (0x1u << 1) /**< \brief (SUPC_WUMR) the supply monitor detection forces the wake up of the core power supply. */
#define SUPC_WUMR_RTTEN (0x1u << 2) /**< \brief (SUPC_WUMR) Real Time Timer Wake Up Enable */
#define   SUPC_WUMR_RTTEN_NOT_ENABLE (0x0u << 2) /**< \brief (SUPC_WUMR) the RTT alarm signal has no wake up effect. */
#define   SUPC_WUMR_RTTEN_ENABLE (0x1u << 2) /**< \brief (SUPC_WUMR) the RTT alarm signal forces the wake up of the core power supply. */
#define SUPC_WUMR_RTCEN (0x1u << 3) /**< \brief (SUPC_WUMR) Real Time Clock Wake Up Enable */
#define   SUPC_WUMR_RTCEN_NOT_ENABLE (0x0u << 3) /**< \brief (SUPC_WUMR) the RTC alarm signal has no wake up effect. */
#define   SUPC_WUMR_RTCEN_ENABLE (0x1u << 3) /**< \brief (SUPC_WUMR) the RTC alarm signal forces the wake up of the core power supply. */
#define SUPC_WUMR_WKUPDBC_Pos 12
#define SUPC_WUMR_WKUPDBC_Msk (0x7u << SUPC_WUMR_WKUPDBC_Pos) /**< \brief (SUPC_WUMR) Wake Up Inputs Debouncer Period */
#define   SUPC_WUMR_WKUPDBC_IMMEDIATE (0x0u << 12) /**< \brief (SUPC_WUMR) Immediate, no debouncing, detected active at least on one Slow Clock edge. */
#define   SUPC_WUMR_WKUPDBC_3_SCLK (0x1u << 12) /**< \brief (SUPC_WUMR) WKUPx shall be in its active state for at least 3 SLCK periods */
#define   SUPC_WUMR_WKUPDBC_32_SCLK (0x2u << 12) /**< \brief (SUPC_WUMR) WKUPx shall be in its active state for at least 32 SLCK periods */
#define   SUPC_WUMR_WKUPDBC_512_SCLK (0x3u << 12) /**< \brief (SUPC_WUMR) WKUPx shall be in its active state for at least 512 SLCK periods */
#define   SUPC_WUMR_WKUPDBC_4096_SCLK (0x4u << 12) /**< \brief (SUPC_WUMR) WKUPx shall be in its active state for at least 4,096 SLCK periods */
#define   SUPC_WUMR_WKUPDBC_32768_SCLK (0x5u << 12) /**< \brief (SUPC_WUMR) WKUPx shall be in its active state for at least 32,768 SLCK periods */
/* -------- SUPC_WUIR : (SUPC Offset: 0x10) Supply Controller Wake Up Inputs Register -------- */
#define SUPC_WUIR_WKUPEN0 (0x1u << 0) /**< \brief (SUPC_WUIR) Wake Up Input Enable 0 */
#define   SUPC_WUIR_WKUPEN0_NOT_ENABLE (0x0u << 0) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN0_ENABLE (0x1u << 0) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN1 (0x1u << 1) /**< \brief (SUPC_WUIR) Wake Up Input Enable 1 */
#define   SUPC_WUIR_WKUPEN1_NOT_ENABLE (0x0u << 1) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN1_ENABLE (0x1u << 1) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN2 (0x1u << 2) /**< \brief (SUPC_WUIR) Wake Up Input Enable 2 */
#define   SUPC_WUIR_WKUPEN2_NOT_ENABLE (0x0u << 2) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN2_ENABLE (0x1u << 2) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN3 (0x1u << 3) /**< \brief (SUPC_WUIR) Wake Up Input Enable 3 */
#define   SUPC_WUIR_WKUPEN3_NOT_ENABLE (0x0u << 3) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN3_ENABLE (0x1u << 3) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN4 (0x1u << 4) /**< \brief (SUPC_WUIR) Wake Up Input Enable 4 */
#define   SUPC_WUIR_WKUPEN4_NOT_ENABLE (0x0u << 4) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN4_ENABLE (0x1u << 4) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN5 (0x1u << 5) /**< \brief (SUPC_WUIR) Wake Up Input Enable 5 */
#define   SUPC_WUIR_WKUPEN5_NOT_ENABLE (0x0u << 5) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN5_ENABLE (0x1u << 5) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN6 (0x1u << 6) /**< \brief (SUPC_WUIR) Wake Up Input Enable 6 */
#define   SUPC_WUIR_WKUPEN6_NOT_ENABLE (0x0u << 6) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN6_ENABLE (0x1u << 6) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN7 (0x1u << 7) /**< \brief (SUPC_WUIR) Wake Up Input Enable 7 */
#define   SUPC_WUIR_WKUPEN7_NOT_ENABLE (0x0u << 7) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN7_ENABLE (0x1u << 7) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN8 (0x1u << 8) /**< \brief (SUPC_WUIR) Wake Up Input Enable 8 */
#define   SUPC_WUIR_WKUPEN8_NOT_ENABLE (0x0u << 8) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN8_ENABLE (0x1u << 8) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN9 (0x1u << 9) /**< \brief (SUPC_WUIR) Wake Up Input Enable 9 */
#define   SUPC_WUIR_WKUPEN9_NOT_ENABLE (0x0u << 9) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN9_ENABLE (0x1u << 9) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN10 (0x1u << 10) /**< \brief (SUPC_WUIR) Wake Up Input Enable 10 */
#define   SUPC_WUIR_WKUPEN10_NOT_ENABLE (0x0u << 10) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN10_ENABLE (0x1u << 10) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN11 (0x1u << 11) /**< \brief (SUPC_WUIR) Wake Up Input Enable 11 */
#define   SUPC_WUIR_WKUPEN11_NOT_ENABLE (0x0u << 11) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN11_ENABLE (0x1u << 11) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN12 (0x1u << 12) /**< \brief (SUPC_WUIR) Wake Up Input Enable 12 */
#define   SUPC_WUIR_WKUPEN12_NOT_ENABLE (0x0u << 12) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN12_ENABLE (0x1u << 12) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN13 (0x1u << 13) /**< \brief (SUPC_WUIR) Wake Up Input Enable 13 */
#define   SUPC_WUIR_WKUPEN13_NOT_ENABLE (0x0u << 13) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN13_ENABLE (0x1u << 13) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN14 (0x1u << 14) /**< \brief (SUPC_WUIR) Wake Up Input Enable 14 */
#define   SUPC_WUIR_WKUPEN14_NOT_ENABLE (0x0u << 14) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN14_ENABLE (0x1u << 14) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPEN15 (0x1u << 15) /**< \brief (SUPC_WUIR) Wake Up Input Enable 15 */
#define   SUPC_WUIR_WKUPEN15_NOT_ENABLE (0x0u << 15) /**< \brief (SUPC_WUIR) the corresponding wake-up input has no wake up effect. */
#define   SUPC_WUIR_WKUPEN15_ENABLE (0x1u << 15) /**< \brief (SUPC_WUIR) the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT0 (0x1u << 16) /**< \brief (SUPC_WUIR) Wake Up Input Transition 0 */
#define   SUPC_WUIR_WKUPT0_HIGH_TO_LOW (0x0u << 16) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT0_LOW_TO_HIGH (0x1u << 16) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT1 (0x1u << 17) /**< \brief (SUPC_WUIR) Wake Up Input Transition 1 */
#define   SUPC_WUIR_WKUPT1_HIGH_TO_LOW (0x0u << 17) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT1_LOW_TO_HIGH (0x1u << 17) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT2 (0x1u << 18) /**< \brief (SUPC_WUIR) Wake Up Input Transition 2 */
#define   SUPC_WUIR_WKUPT2_HIGH_TO_LOW (0x0u << 18) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT2_LOW_TO_HIGH (0x1u << 18) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT3 (0x1u << 19) /**< \brief (SUPC_WUIR) Wake Up Input Transition 3 */
#define   SUPC_WUIR_WKUPT3_HIGH_TO_LOW (0x0u << 19) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT3_LOW_TO_HIGH (0x1u << 19) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT4 (0x1u << 20) /**< \brief (SUPC_WUIR) Wake Up Input Transition 4 */
#define   SUPC_WUIR_WKUPT4_HIGH_TO_LOW (0x0u << 20) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT4_LOW_TO_HIGH (0x1u << 20) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT5 (0x1u << 21) /**< \brief (SUPC_WUIR) Wake Up Input Transition 5 */
#define   SUPC_WUIR_WKUPT5_HIGH_TO_LOW (0x0u << 21) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT5_LOW_TO_HIGH (0x1u << 21) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT6 (0x1u << 22) /**< \brief (SUPC_WUIR) Wake Up Input Transition 6 */
#define   SUPC_WUIR_WKUPT6_HIGH_TO_LOW (0x0u << 22) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT6_LOW_TO_HIGH (0x1u << 22) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT7 (0x1u << 23) /**< \brief (SUPC_WUIR) Wake Up Input Transition 7 */
#define   SUPC_WUIR_WKUPT7_HIGH_TO_LOW (0x0u << 23) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT7_LOW_TO_HIGH (0x1u << 23) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT8 (0x1u << 24) /**< \brief (SUPC_WUIR) Wake Up Input Transition 8 */
#define   SUPC_WUIR_WKUPT8_HIGH_TO_LOW (0x0u << 24) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT8_LOW_TO_HIGH (0x1u << 24) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT9 (0x1u << 25) /**< \brief (SUPC_WUIR) Wake Up Input Transition 9 */
#define   SUPC_WUIR_WKUPT9_HIGH_TO_LOW (0x0u << 25) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT9_LOW_TO_HIGH (0x1u << 25) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT10 (0x1u << 26) /**< \brief (SUPC_WUIR) Wake Up Input Transition 10 */
#define   SUPC_WUIR_WKUPT10_HIGH_TO_LOW (0x0u << 26) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT10_LOW_TO_HIGH (0x1u << 26) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT11 (0x1u << 27) /**< \brief (SUPC_WUIR) Wake Up Input Transition 11 */
#define   SUPC_WUIR_WKUPT11_HIGH_TO_LOW (0x0u << 27) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT11_LOW_TO_HIGH (0x1u << 27) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT12 (0x1u << 28) /**< \brief (SUPC_WUIR) Wake Up Input Transition 12 */
#define   SUPC_WUIR_WKUPT12_HIGH_TO_LOW (0x0u << 28) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT12_LOW_TO_HIGH (0x1u << 28) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT13 (0x1u << 29) /**< \brief (SUPC_WUIR) Wake Up Input Transition 13 */
#define   SUPC_WUIR_WKUPT13_HIGH_TO_LOW (0x0u << 29) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT13_LOW_TO_HIGH (0x1u << 29) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT14 (0x1u << 30) /**< \brief (SUPC_WUIR) Wake Up Input Transition 14 */
#define   SUPC_WUIR_WKUPT14_HIGH_TO_LOW (0x0u << 30) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT14_LOW_TO_HIGH (0x1u << 30) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define SUPC_WUIR_WKUPT15 (0x1u << 31) /**< \brief (SUPC_WUIR) Wake Up Input Transition 15 */
#define   SUPC_WUIR_WKUPT15_HIGH_TO_LOW (0x0u << 31) /**< \brief (SUPC_WUIR) a high to low level transition on the corresponding wake-up input forces the wake up of the core power supply. */
#define   SUPC_WUIR_WKUPT15_LOW_TO_HIGH (0x1u << 31) /**< \brief (SUPC_WUIR) a low to high level transition on the corresponding wake-up input forces the wake up of the core power supply. */
/* -------- SUPC_SR : (SUPC Offset: 0x14) Supply Controller Status Register -------- */
#define SUPC_SR_WKUPS (0x1u << 1) /**< \brief (SUPC_SR) WKUP Wake Up Status */
#define   SUPC_SR_WKUPS_NO (0x0u << 1) /**< \brief (SUPC_SR) no wake up due to the assertion of the WKUP pins has occurred since the last read of SUPC_SR. */
#define   SUPC_SR_WKUPS_PRESENT (0x1u << 1) /**< \brief (SUPC_SR) at least one wake up due to the assertion of the WKUP pins has occurred since the last read of SUPC_SR. */
#define SUPC_SR_SMWS (0x1u << 2) /**< \brief (SUPC_SR) Supply Monitor Detection Wake Up Status */
#define   SUPC_SR_SMWS_NO (0x0u << 2) /**< \brief (SUPC_SR) no wake up due to a supply monitor detection has occurred since the last read of SUPC_SR. */
#define   SUPC_SR_SMWS_PRESENT (0x1u << 2) /**< \brief (SUPC_SR) at least one wake up due to a supply monitor detection has occurred since the last read of SUPC_SR. */
#define SUPC_SR_BODRSTS (0x1u << 3) /**< \brief (SUPC_SR) Brownout Detector Reset Status */
#define   SUPC_SR_BODRSTS_NO (0x0u << 3) /**< \brief (SUPC_SR) no core brownout rising edge event has been detected since the last read of the SUPC_SR. */
#define   SUPC_SR_BODRSTS_PRESENT (0x1u << 3) /**< \brief (SUPC_SR) at least one brownout output rising edge event has been detected since the last read of the SUPC_SR. */
#define SUPC_SR_SMRSTS (0x1u << 4) /**< \brief (SUPC_SR) Supply Monitor Reset Status */
#define   SUPC_SR_SMRSTS_NO (0x0u << 4) /**< \brief (SUPC_SR) no supply monitor detection has generated a core reset since the last read of the SUPC_SR. */
#define   SUPC_SR_SMRSTS_PRESENT (0x1u << 4) /**< \brief (SUPC_SR) at least one supply monitor detection has generated a core reset since the last read of the SUPC_SR. */
#define SUPC_SR_SMS (0x1u << 5) /**< \brief (SUPC_SR) Supply Monitor Status */
#define   SUPC_SR_SMS_NO (0x0u << 5) /**< \brief (SUPC_SR) no supply monitor detection since the last read of SUPC_SR. */
#define   SUPC_SR_SMS_PRESENT (0x1u << 5) /**< \brief (SUPC_SR) at least one supply monitor detection since the last read of SUPC_SR. */
#define SUPC_SR_SMOS (0x1u << 6) /**< \brief (SUPC_SR) Supply Monitor Output Status */
#define   SUPC_SR_SMOS_HIGH (0x0u << 6) /**< \brief (SUPC_SR) the supply monitor detected VDDIO higher than its threshold at its last measurement. */
#define   SUPC_SR_SMOS_LOW (0x1u << 6) /**< \brief (SUPC_SR) the supply monitor detected VDDIO lower than its threshold at its last measurement. */
#define SUPC_SR_OSCSEL (0x1u << 7) /**< \brief (SUPC_SR) 32-kHz Oscillator Selection Status */
#define   SUPC_SR_OSCSEL_RC (0x0u << 7) /**< \brief (SUPC_SR) the slow clock, SLCK is generated by the embedded 32-kHz RC oscillator. */
#define   SUPC_SR_OSCSEL_CRYST (0x1u << 7) /**< \brief (SUPC_SR) the slow clock, SLCK is generated by the 32-kHz crystal oscillator. */
#define SUPC_SR_WKUPIS0 (0x1u << 16) /**< \brief (SUPC_SR) WKUP Input Status 0 */
#define   SUPC_SR_WKUPIS0_DIS (0x0u << 16) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS0_EN (0x1u << 16) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS1 (0x1u << 17) /**< \brief (SUPC_SR) WKUP Input Status 1 */
#define   SUPC_SR_WKUPIS1_DIS (0x0u << 17) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS1_EN (0x1u << 17) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS2 (0x1u << 18) /**< \brief (SUPC_SR) WKUP Input Status 2 */
#define   SUPC_SR_WKUPIS2_DIS (0x0u << 18) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS2_EN (0x1u << 18) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS3 (0x1u << 19) /**< \brief (SUPC_SR) WKUP Input Status 3 */
#define   SUPC_SR_WKUPIS3_DIS (0x0u << 19) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS3_EN (0x1u << 19) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS4 (0x1u << 20) /**< \brief (SUPC_SR) WKUP Input Status 4 */
#define   SUPC_SR_WKUPIS4_DIS (0x0u << 20) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS4_EN (0x1u << 20) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS5 (0x1u << 21) /**< \brief (SUPC_SR) WKUP Input Status 5 */
#define   SUPC_SR_WKUPIS5_DIS (0x0u << 21) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS5_EN (0x1u << 21) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS6 (0x1u << 22) /**< \brief (SUPC_SR) WKUP Input Status 6 */
#define   SUPC_SR_WKUPIS6_DIS (0x0u << 22) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS6_EN (0x1u << 22) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS7 (0x1u << 23) /**< \brief (SUPC_SR) WKUP Input Status 7 */
#define   SUPC_SR_WKUPIS7_DIS (0x0u << 23) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS7_EN (0x1u << 23) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS8 (0x1u << 24) /**< \brief (SUPC_SR) WKUP Input Status 8 */
#define   SUPC_SR_WKUPIS8_DIS (0x0u << 24) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS8_EN (0x1u << 24) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS9 (0x1u << 25) /**< \brief (SUPC_SR) WKUP Input Status 9 */
#define   SUPC_SR_WKUPIS9_DIS (0x0u << 25) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS9_EN (0x1u << 25) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS10 (0x1u << 26) /**< \brief (SUPC_SR) WKUP Input Status 10 */
#define   SUPC_SR_WKUPIS10_DIS (0x0u << 26) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS10_EN (0x1u << 26) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS11 (0x1u << 27) /**< \brief (SUPC_SR) WKUP Input Status 11 */
#define   SUPC_SR_WKUPIS11_DIS (0x0u << 27) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS11_EN (0x1u << 27) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS12 (0x1u << 28) /**< \brief (SUPC_SR) WKUP Input Status 12 */
#define   SUPC_SR_WKUPIS12_DIS (0x0u << 28) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS12_EN (0x1u << 28) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS13 (0x1u << 29) /**< \brief (SUPC_SR) WKUP Input Status 13 */
#define   SUPC_SR_WKUPIS13_DIS (0x0u << 29) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS13_EN (0x1u << 29) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS14 (0x1u << 30) /**< \brief (SUPC_SR) WKUP Input Status 14 */
#define   SUPC_SR_WKUPIS14_DIS (0x0u << 30) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS14_EN (0x1u << 30) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */
#define SUPC_SR_WKUPIS15 (0x1u << 31) /**< \brief (SUPC_SR) WKUP Input Status 15 */
#define   SUPC_SR_WKUPIS15_DIS (0x0u << 31) /**< \brief (SUPC_SR) the corresponding wake-up input is disabled, or was inactive at the time the debouncer triggered a wake up event. */
#define   SUPC_SR_WKUPIS15_EN (0x1u << 31) /**< \brief (SUPC_SR) the corresponding wake-up input was active at the time the debouncer triggered a wake up event. */

/*@}*/


/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Power Management Controller */
/* ============================================================================= */
/** \addtogroup SAM3U_PMC Power Management Controller */
/*@{*/

#ifndef __ASSEMBLY__
/** \brief Pmc hardware registers */
typedef struct {
  AT91_REG  PMC_SCER;      // System Clock Enable Register
  AT91_REG  PMC_SCDR;      // System Clock Disable Register
  AT91_REG  PMC_SCSR;      // System Clock Status Register
  AT91_REG  Reserved0[1];  // 
  AT91_REG  PMC_PCER;      // Peripheral Clock Enable Register
  AT91_REG  PMC_PCDR;      // Peripheral Clock Disable Register
  AT91_REG  PMC_PCSR;      // Peripheral Clock Status Register
  AT91_REG  PMC_UCKR;      // UTMI Clock Configuration Register
  AT91_REG  PMC_MOR;       // Main Oscillator Register
  AT91_REG  PMC_MCFR;      // Main Clock  Frequency Register
  AT91_REG  PMC_PLLAR;     // PLL Register
  AT91_REG  Reserved1[1];  // 
  AT91_REG  PMC_MCKR;      // Master Clock Register
  AT91_REG  Reserved2[3];  // 
  AT91_REG  PMC_PCKR[8];   // Programmable Clock Register
  AT91_REG  PMC_IER;       // Interrupt Enable Register
  AT91_REG  PMC_IDR;       // Interrupt Disable Register
  AT91_REG  PMC_SR;        // Status Register
  AT91_REG  PMC_IMR;       // Interrupt Mask Register
  AT91_REG  PMC_FSMR;      // Fast Startup Mode Register
  AT91_REG  PMC_FSPR;      // Fast Startup Polarity Register
  AT91_REG  PMC_FOCR;      // Fault Output Clear Register
  AT91_REG  Reserved3[28]; // 
  AT91_REG  PMC_ADDRSIZE;  // PMC ADDRSIZE REGISTER 
  AT91_REG  PMC_IPNAME1;   // PMC IPNAME1 REGISTER 
  AT91_REG  PMC_IPNAME2;   // PMC IPNAME2 REGISTER 
  AT91_REG  PMC_FEATURES;  // PMC FEATURES REGISTER 
  AT91_REG  PMC_VER;       // APMC VERSION REGISTER
} Pmc;
#endif /* __ASSEMBLY__ */
/* -------- PMC_SCER : (PMC Offset: 0x0000) System Clock Enable Register -------- */
#define PMC_SCER_UDP (0x1u << 7) /**< \brief (PMC_SCER) USB Device Port Clock Enable */
#define PMC_SCER_PCK0 (0x1u << 8) /**< \brief (PMC_SCER) Programmable Clock 0 Output Enable */
#define PMC_SCER_PCK1 (0x1u << 9) /**< \brief (PMC_SCER) Programmable Clock 1 Output Enable */
#define PMC_SCER_PCK2 (0x1u << 10) /**< \brief (PMC_SCER) Programmable Clock 2 Output Enable */
/* -------- PMC_SCDR : (PMC Offset: 0x0004) System Clock Disable Register -------- */
#define PMC_SCDR_UDP (0x1u << 7) /**< \brief (PMC_SCDR) USB Device Port Clock Disable */
#define PMC_SCDR_PCK0 (0x1u << 8) /**< \brief (PMC_SCDR) Programmable Clock 0 Output Disable */
#define PMC_SCDR_PCK1 (0x1u << 9) /**< \brief (PMC_SCDR) Programmable Clock 1 Output Disable */
#define PMC_SCDR_PCK2 (0x1u << 10) /**< \brief (PMC_SCDR) Programmable Clock 2 Output Disable */
/* -------- PMC_SCSR : (PMC Offset: 0x0008) System Clock Status Register -------- */
#define PMC_SCSR_UDP (0x1u << 7) /**< \brief (PMC_SCSR) USB Device Port Clock Status */
#define PMC_SCSR_PCK0 (0x1u << 8) /**< \brief (PMC_SCSR) Programmable Clock 0 Output Status */
#define PMC_SCSR_PCK1 (0x1u << 9) /**< \brief (PMC_SCSR) Programmable Clock 1 Output Status */
#define PMC_SCSR_PCK2 (0x1u << 10) /**< \brief (PMC_SCSR) Programmable Clock 2 Output Status */
/* -------- PMC_PCER : (PMC Offset: 0x0010) Peripheral Clock Enable Register -------- */
#define PMC_PCER_PID2 (0x1u << 2) /**< \brief (PMC_PCER) Peripheral Clock 2 Enable */
#define PMC_PCER_PID3 (0x1u << 3) /**< \brief (PMC_PCER) Peripheral Clock 3 Enable */
#define PMC_PCER_PID4 (0x1u << 4) /**< \brief (PMC_PCER) Peripheral Clock 4 Enable */
#define PMC_PCER_PID5 (0x1u << 5) /**< \brief (PMC_PCER) Peripheral Clock 5 Enable */
#define PMC_PCER_PID6 (0x1u << 6) /**< \brief (PMC_PCER) Peripheral Clock 6 Enable */
#define PMC_PCER_PID7 (0x1u << 7) /**< \brief (PMC_PCER) Peripheral Clock 7 Enable */
#define PMC_PCER_PID8 (0x1u << 8) /**< \brief (PMC_PCER) Peripheral Clock 8 Enable */
#define PMC_PCER_PID9 (0x1u << 9) /**< \brief (PMC_PCER) Peripheral Clock 9 Enable */
#define PMC_PCER_PID10 (0x1u << 10) /**< \brief (PMC_PCER) Peripheral Clock 10 Enable */
#define PMC_PCER_PID11 (0x1u << 11) /**< \brief (PMC_PCER) Peripheral Clock 11 Enable */
#define PMC_PCER_PID12 (0x1u << 12) /**< \brief (PMC_PCER) Peripheral Clock 12 Enable */
#define PMC_PCER_PID13 (0x1u << 13) /**< \brief (PMC_PCER) Peripheral Clock 13 Enable */
#define PMC_PCER_PID14 (0x1u << 14) /**< \brief (PMC_PCER) Peripheral Clock 14 Enable */
#define PMC_PCER_PID15 (0x1u << 15) /**< \brief (PMC_PCER) Peripheral Clock 15 Enable */
#define PMC_PCER_PID16 (0x1u << 16) /**< \brief (PMC_PCER) Peripheral Clock 16 Enable */
#define PMC_PCER_PID17 (0x1u << 17) /**< \brief (PMC_PCER) Peripheral Clock 17 Enable */
#define PMC_PCER_PID18 (0x1u << 18) /**< \brief (PMC_PCER) Peripheral Clock 18 Enable */
#define PMC_PCER_PID19 (0x1u << 19) /**< \brief (PMC_PCER) Peripheral Clock 19 Enable */
#define PMC_PCER_PID20 (0x1u << 20) /**< \brief (PMC_PCER) Peripheral Clock 20 Enable */
#define PMC_PCER_PID21 (0x1u << 21) /**< \brief (PMC_PCER) Peripheral Clock 21 Enable */
#define PMC_PCER_PID22 (0x1u << 22) /**< \brief (PMC_PCER) Peripheral Clock 22 Enable */
#define PMC_PCER_PID23 (0x1u << 23) /**< \brief (PMC_PCER) Peripheral Clock 23 Enable */
#define PMC_PCER_PID24 (0x1u << 24) /**< \brief (PMC_PCER) Peripheral Clock 24 Enable */
#define PMC_PCER_PID25 (0x1u << 25) /**< \brief (PMC_PCER) Peripheral Clock 25 Enable */
#define PMC_PCER_PID26 (0x1u << 26) /**< \brief (PMC_PCER) Peripheral Clock 26 Enable */
#define PMC_PCER_PID27 (0x1u << 27) /**< \brief (PMC_PCER) Peripheral Clock 27 Enable */
#define PMC_PCER_PID28 (0x1u << 28) /**< \brief (PMC_PCER) Peripheral Clock 28 Enable */
#define PMC_PCER_PID29 (0x1u << 29) /**< \brief (PMC_PCER) Peripheral Clock 29 Enable */
#define PMC_PCER_PID30 (0x1u << 30) /**< \brief (PMC_PCER) Peripheral Clock 30 Enable */
#define PMC_PCER_PID31 (0x1u << 31) /**< \brief (PMC_PCER) Peripheral Clock 31 Enable */
/* -------- PMC_PCDR : (PMC Offset: 0x0014) Peripheral Clock Disable Register -------- */
#define PMC_PCDR_PID2 (0x1u << 2) /**< \brief (PMC_PCDR) Peripheral Clock 2 Disable */
#define PMC_PCDR_PID3 (0x1u << 3) /**< \brief (PMC_PCDR) Peripheral Clock 3 Disable */
#define PMC_PCDR_PID4 (0x1u << 4) /**< \brief (PMC_PCDR) Peripheral Clock 4 Disable */
#define PMC_PCDR_PID5 (0x1u << 5) /**< \brief (PMC_PCDR) Peripheral Clock 5 Disable */
#define PMC_PCDR_PID6 (0x1u << 6) /**< \brief (PMC_PCDR) Peripheral Clock 6 Disable */
#define PMC_PCDR_PID7 (0x1u << 7) /**< \brief (PMC_PCDR) Peripheral Clock 7 Disable */
#define PMC_PCDR_PID8 (0x1u << 8) /**< \brief (PMC_PCDR) Peripheral Clock 8 Disable */
#define PMC_PCDR_PID9 (0x1u << 9) /**< \brief (PMC_PCDR) Peripheral Clock 9 Disable */
#define PMC_PCDR_PID10 (0x1u << 10) /**< \brief (PMC_PCDR) Peripheral Clock 10 Disable */
#define PMC_PCDR_PID11 (0x1u << 11) /**< \brief (PMC_PCDR) Peripheral Clock 11 Disable */
#define PMC_PCDR_PID12 (0x1u << 12) /**< \brief (PMC_PCDR) Peripheral Clock 12 Disable */
#define PMC_PCDR_PID13 (0x1u << 13) /**< \brief (PMC_PCDR) Peripheral Clock 13 Disable */
#define PMC_PCDR_PID14 (0x1u << 14) /**< \brief (PMC_PCDR) Peripheral Clock 14 Disable */
#define PMC_PCDR_PID15 (0x1u << 15) /**< \brief (PMC_PCDR) Peripheral Clock 15 Disable */
#define PMC_PCDR_PID16 (0x1u << 16) /**< \brief (PMC_PCDR) Peripheral Clock 16 Disable */
#define PMC_PCDR_PID17 (0x1u << 17) /**< \brief (PMC_PCDR) Peripheral Clock 17 Disable */
#define PMC_PCDR_PID18 (0x1u << 18) /**< \brief (PMC_PCDR) Peripheral Clock 18 Disable */
#define PMC_PCDR_PID19 (0x1u << 19) /**< \brief (PMC_PCDR) Peripheral Clock 19 Disable */
#define PMC_PCDR_PID20 (0x1u << 20) /**< \brief (PMC_PCDR) Peripheral Clock 20 Disable */
#define PMC_PCDR_PID21 (0x1u << 21) /**< \brief (PMC_PCDR) Peripheral Clock 21 Disable */
#define PMC_PCDR_PID22 (0x1u << 22) /**< \brief (PMC_PCDR) Peripheral Clock 22 Disable */
#define PMC_PCDR_PID23 (0x1u << 23) /**< \brief (PMC_PCDR) Peripheral Clock 23 Disable */
#define PMC_PCDR_PID24 (0x1u << 24) /**< \brief (PMC_PCDR) Peripheral Clock 24 Disable */
#define PMC_PCDR_PID25 (0x1u << 25) /**< \brief (PMC_PCDR) Peripheral Clock 25 Disable */
#define PMC_PCDR_PID26 (0x1u << 26) /**< \brief (PMC_PCDR) Peripheral Clock 26 Disable */
#define PMC_PCDR_PID27 (0x1u << 27) /**< \brief (PMC_PCDR) Peripheral Clock 27 Disable */
#define PMC_PCDR_PID28 (0x1u << 28) /**< \brief (PMC_PCDR) Peripheral Clock 28 Disable */
#define PMC_PCDR_PID29 (0x1u << 29) /**< \brief (PMC_PCDR) Peripheral Clock 29 Disable */
#define PMC_PCDR_PID30 (0x1u << 30) /**< \brief (PMC_PCDR) Peripheral Clock 30 Disable */
#define PMC_PCDR_PID31 (0x1u << 31) /**< \brief (PMC_PCDR) Peripheral Clock 31 Disable */
/* -------- PMC_PCSR : (PMC Offset: 0x0018) Peripheral Clock Status Register -------- */
#define PMC_PCSR_PID2 (0x1u << 2) /**< \brief (PMC_PCSR) Peripheral Clock 2 Status */
#define PMC_PCSR_PID3 (0x1u << 3) /**< \brief (PMC_PCSR) Peripheral Clock 3 Status */
#define PMC_PCSR_PID4 (0x1u << 4) /**< \brief (PMC_PCSR) Peripheral Clock 4 Status */
#define PMC_PCSR_PID5 (0x1u << 5) /**< \brief (PMC_PCSR) Peripheral Clock 5 Status */
#define PMC_PCSR_PID6 (0x1u << 6) /**< \brief (PMC_PCSR) Peripheral Clock 6 Status */
#define PMC_PCSR_PID7 (0x1u << 7) /**< \brief (PMC_PCSR) Peripheral Clock 7 Status */
#define PMC_PCSR_PID8 (0x1u << 8) /**< \brief (PMC_PCSR) Peripheral Clock 8 Status */
#define PMC_PCSR_PID9 (0x1u << 9) /**< \brief (PMC_PCSR) Peripheral Clock 9 Status */
#define PMC_PCSR_PID10 (0x1u << 10) /**< \brief (PMC_PCSR) Peripheral Clock 10 Status */
#define PMC_PCSR_PID11 (0x1u << 11) /**< \brief (PMC_PCSR) Peripheral Clock 11 Status */
#define PMC_PCSR_PID12 (0x1u << 12) /**< \brief (PMC_PCSR) Peripheral Clock 12 Status */
#define PMC_PCSR_PID13 (0x1u << 13) /**< \brief (PMC_PCSR) Peripheral Clock 13 Status */
#define PMC_PCSR_PID14 (0x1u << 14) /**< \brief (PMC_PCSR) Peripheral Clock 14 Status */
#define PMC_PCSR_PID15 (0x1u << 15) /**< \brief (PMC_PCSR) Peripheral Clock 15 Status */
#define PMC_PCSR_PID16 (0x1u << 16) /**< \brief (PMC_PCSR) Peripheral Clock 16 Status */
#define PMC_PCSR_PID17 (0x1u << 17) /**< \brief (PMC_PCSR) Peripheral Clock 17 Status */
#define PMC_PCSR_PID18 (0x1u << 18) /**< \brief (PMC_PCSR) Peripheral Clock 18 Status */
#define PMC_PCSR_PID19 (0x1u << 19) /**< \brief (PMC_PCSR) Peripheral Clock 19 Status */
#define PMC_PCSR_PID20 (0x1u << 20) /**< \brief (PMC_PCSR) Peripheral Clock 20 Status */
#define PMC_PCSR_PID21 (0x1u << 21) /**< \brief (PMC_PCSR) Peripheral Clock 21 Status */
#define PMC_PCSR_PID22 (0x1u << 22) /**< \brief (PMC_PCSR) Peripheral Clock 22 Status */
#define PMC_PCSR_PID23 (0x1u << 23) /**< \brief (PMC_PCSR) Peripheral Clock 23 Status */
#define PMC_PCSR_PID24 (0x1u << 24) /**< \brief (PMC_PCSR) Peripheral Clock 24 Status */
#define PMC_PCSR_PID25 (0x1u << 25) /**< \brief (PMC_PCSR) Peripheral Clock 25 Status */
#define PMC_PCSR_PID26 (0x1u << 26) /**< \brief (PMC_PCSR) Peripheral Clock 26 Status */
#define PMC_PCSR_PID27 (0x1u << 27) /**< \brief (PMC_PCSR) Peripheral Clock 27 Status */
#define PMC_PCSR_PID28 (0x1u << 28) /**< \brief (PMC_PCSR) Peripheral Clock 28 Status */
#define PMC_PCSR_PID29 (0x1u << 29) /**< \brief (PMC_PCSR) Peripheral Clock 29 Status */
#define PMC_PCSR_PID30 (0x1u << 30) /**< \brief (PMC_PCSR) Peripheral Clock 30 Status */
#define PMC_PCSR_PID31 (0x1u << 31) /**< \brief (PMC_PCSR) Peripheral Clock 31 Status */
/* -------- CKGR_MOR : (PMC Offset: 0x0020) Main Oscillator Register -------- */
#define CKGR_MOR_MOSCXTEN (0x1u << 0) /**< \brief (CKGR_MOR) Main Crystal Oscillator Enable */
#define CKGR_MOR_MOSCXTBY (0x1u << 1) /**< \brief (CKGR_MOR) Main Crystal Oscillator Bypass */
#define CKGR_MOR_WAITMODE (0x1u << 2) /**< \brief (CKGR_MOR) Wait Mode Command */
#define CKGR_MOR_MOSCRCEN (0x1u << 3) /**< \brief (CKGR_MOR) Main On-Chip RC Oscillator Enable */
#define CKGR_MOR_MOSCRCF_Pos 4
#define CKGR_MOR_MOSCRCF_Msk (0x7u << CKGR_MOR_MOSCRCF_Pos) /**< \brief (CKGR_MOR) Main On-Chip RC Oscillator Frequency Selection */
#define   CKGR_MOR_MOSCRCF_4MHZ (0x0u << 4) /**< \brief (CKGR_MOR) Fast RC Oscillator Frequency is at 4 MHz */
#define   CKGR_MOR_MOSCRCF_8MHZ (0x1u << 4) /**< \brief (CKGR_MOR) Fast RC Oscillator Frequency is at 8 MHz */
#define   CKGR_MOR_MOSCRCF_12MHZ (0x2u << 4) /**< \brief (CKGR_MOR) Fast RC Oscillator Frequency is at 12 MHz */
#define CKGR_MOR_MOSCXTST_Pos 8
#define CKGR_MOR_MOSCXTST_Msk (0xffu << CKGR_MOR_MOSCXTST_Pos) /**< \brief (CKGR_MOR) Main Crystal Oscillator Start-up Time */
#define CKGR_MOR_MOSCXTST(value) ((CKGR_MOR_MOSCXTST_Msk & ((value) << CKGR_MOR_MOSCXTST_Pos)))
#define CKGR_MOR_KEY_Pos 16
#define CKGR_MOR_KEY_Msk (0xffu << CKGR_MOR_KEY_Pos) /**< \brief (CKGR_MOR) Password */
#define CKGR_MOR_KEY(value) ((CKGR_MOR_KEY_Msk & ((value) << CKGR_MOR_KEY_Pos)))
#define CKGR_MOR_MOSCSEL (0x1u << 24) /**< \brief (CKGR_MOR) Main Oscillator Selection */
#define CKGR_MOR_CFDEN (0x1u << 25) /**< \brief (CKGR_MOR) Clock Failure Detector Enable */
/* -------- CKGR_MCFR : (PMC Offset: 0x0024) Main Clock Frequency Register -------- */
#define CKGR_MCFR_MAINF_Pos 0
#define CKGR_MCFR_MAINF_Msk (0xffffu << CKGR_MCFR_MAINF_Pos) /**< \brief (CKGR_MCFR) Main Clock Frequency */
#define CKGR_MCFR_MAINFRDY (0x1u << 16) /**< \brief (CKGR_MCFR) Main Clock Ready */
/* -------- CKGR_PLLAR : (PMC Offset: 0x0028) PLLA Register -------- */
#define CKGR_PLLAR_DIVA_Pos 0
#define CKGR_PLLAR_DIVA_Msk (0xffu << CKGR_PLLAR_DIVA_Pos) /**< \brief (CKGR_PLLAR) Divider */
#define CKGR_PLLAR_DIVA(value) ((CKGR_PLLAR_DIVA_Msk & ((value) << CKGR_PLLAR_DIVA_Pos)))
#define CKGR_PLLAR_PLLACOUNT_Pos 8
#define CKGR_PLLAR_PLLACOUNT_Msk (0x3fu << CKGR_PLLAR_PLLACOUNT_Pos) /**< \brief (CKGR_PLLAR) PLLA Counter */
#define CKGR_PLLAR_PLLACOUNT(value) ((CKGR_PLLAR_PLLACOUNT_Msk & ((value) << CKGR_PLLAR_PLLACOUNT_Pos)))
#define CKGR_PLLAR_MULA_Pos 16
#define CKGR_PLLAR_MULA_Msk (0x7ffu << CKGR_PLLAR_MULA_Pos) /**< \brief (CKGR_PLLAR) PLLA Multiplier */
#define CKGR_PLLAR_MULA(value) ((CKGR_PLLAR_MULA_Msk & ((value) << CKGR_PLLAR_MULA_Pos)))
#define CKGR_PLLAR_STUCKTO1 (0x1u << 29) /**< \brief (CKGR_PLLAR)  */
/* -------- PMC_MCKR : (PMC Offset: 0x0030) Master Clock Register -------- */
#define PMC_MCKR_CSS_Pos 0
#define PMC_MCKR_CSS_Msk (0x3u << PMC_MCKR_CSS_Pos) /**< \brief (PMC_MCKR) Master Clock Source Selection */
#define   PMC_MCKR_CSS_SLOW_CLK (0x0u << 0) /**< \brief (PMC_MCKR) Slow Clock is selected */
#define   PMC_MCKR_CSS_MAIN_CLK (0x1u << 0) /**< \brief (PMC_MCKR) Main Clock is selected */
#define   PMC_MCKR_CSS_PLLA_CLK (0x2u << 0) /**< \brief (PMC_MCKR) PLLA Clock is selected */
#define   PMC_MCKR_CSS_PLLB_CLK (0x3u << 0) /**< \brief (PMC_MCKR) PLLB Clock is selected */
#define PMC_MCKR_PRES_Pos 4
#define PMC_MCKR_PRES_Msk (0x7u << PMC_MCKR_PRES_Pos) /**< \brief (PMC_MCKR) Processor Clock Prescaler */
#define   PMC_MCKR_PRES_CLK (0x0u << 4) /**< \brief (PMC_MCKR) Selected clock */
#define   PMC_MCKR_PRES_CLK_2 (0x1u << 4) /**< \brief (PMC_MCKR) Selected clock divided by 2 */
#define   PMC_MCKR_PRES_CLK_4 (0x2u << 4) /**< \brief (PMC_MCKR) Selected clock divided by 4 */
#define   PMC_MCKR_PRES_CLK_8 (0x3u << 4) /**< \brief (PMC_MCKR) Selected clock divided by 8 */
#define   PMC_MCKR_PRES_CLK_16 (0x4u << 4) /**< \brief (PMC_MCKR) Selected clock divided by 16 */
#define   PMC_MCKR_PRES_CLK_32 (0x5u << 4) /**< \brief (PMC_MCKR) Selected clock divided by 32 */
#define   PMC_MCKR_PRES_CLK_64 (0x6u << 4) /**< \brief (PMC_MCKR) Selected clock divided by 64 */
#define   PMC_MCKR_PRES_CLK_3 (0x7u << 4) /**< \brief (PMC_MCKR) Selected clock divided by 3 */
#define PMC_MCKR_PLLADIV2 (0x1u << 12) /**< \brief (PMC_MCKR) PLLA Divisor by 2 */
#define PMC_MCKR_PLLBDIV2 (0x1u << 13) /**< \brief (PMC_MCKR) PLLB Divisor by 2 */
/* -------- PMC_IER : (PMC Offset: 0x0060) Interrupt Enable Register -------- */
#define PMC_IER_MOSCXTS (0x1u << 0) /**< \brief (PMC_IER) Main Crystal Oscillator Status Interrupt Enable */
#define PMC_IER_LOCKA (0x1u << 1) /**< \brief (PMC_IER) PLLA Lock Interrupt Enable */
#define PMC_IER_LOCKB (0x1u << 2) /**< \brief (PMC_IER) PLLB Lock Interrupt Enable */
#define PMC_IER_MCKRDY (0x1u << 3) /**< \brief (PMC_IER) Master Clock Ready Interrupt Enable */
#define PMC_IER_PCKRDY0 (0x1u << 8) /**< \brief (PMC_IER) Programmable Clock Ready 0 Interrupt Enable */
#define PMC_IER_PCKRDY1 (0x1u << 9) /**< \brief (PMC_IER) Programmable Clock Ready 1 Interrupt Enable */
#define PMC_IER_PCKRDY2 (0x1u << 10) /**< \brief (PMC_IER) Programmable Clock Ready 2 Interrupt Enable */
#define PMC_IER_MOSCSELS (0x1u << 16) /**< \brief (PMC_IER) Main Oscillator Selection Status Interrupt Enable */
#define PMC_IER_MOSCRCS (0x1u << 17) /**< \brief (PMC_IER) Main On-Chip RC Status Interrupt Enable */
#define PMC_IER_CFDEV (0x1u << 18) /**< \brief (PMC_IER) Clock Failure Detector Event Interrupt Enable */
/* -------- PMC_IDR : (PMC Offset: 0x0064) Interrupt Disable Register -------- */
#define PMC_IDR_MOSCXTS (0x1u << 0) /**< \brief (PMC_IDR) Main Crystal Oscillator Status Interrupt Disable */
#define PMC_IDR_LOCKA (0x1u << 1) /**< \brief (PMC_IDR) PLLA Lock Interrupt Disable */
#define PMC_IDR_LOCKB (0x1u << 2) /**< \brief (PMC_IDR) PLLB Lock Interrupt Disable */
#define PMC_IDR_MCKRDY (0x1u << 3) /**< \brief (PMC_IDR) Master Clock Ready Interrupt Disable */
#define PMC_IDR_PCKRDY0 (0x1u << 8) /**< \brief (PMC_IDR) Programmable Clock Ready 0 Interrupt Disable */
#define PMC_IDR_PCKRDY1 (0x1u << 9) /**< \brief (PMC_IDR) Programmable Clock Ready 1 Interrupt Disable */
#define PMC_IDR_PCKRDY2 (0x1u << 10) /**< \brief (PMC_IDR) Programmable Clock Ready 2 Interrupt Disable */
#define PMC_IDR_MOSCSELS (0x1u << 16) /**< \brief (PMC_IDR) Main Oscillator Selection Status Interrupt Disable */
#define PMC_IDR_MOSCRCS (0x1u << 17) /**< \brief (PMC_IDR) Main On-Chip RC Status Interrupt Disable */
#define PMC_IDR_CFDEV (0x1u << 18) /**< \brief (PMC_IDR) Clock Failure Detector Event Interrupt Disable */
/* -------- PMC_SR : (PMC Offset: 0x0068) Status Register -------- */
#define PMC_SR_MOSCXTS (0x1u << 0) /**< \brief (PMC_SR) Main XTAL Oscillator Status */
#define PMC_SR_LOCKA (0x1u << 1) /**< \brief (PMC_SR) PLLA Lock Status */
#define PMC_SR_LOCKB (0x1u << 2) /**< \brief (PMC_SR) PLLB Lock Status */
#define PMC_SR_MCKRDY (0x1u << 3) /**< \brief (PMC_SR) Master Clock Status */
#define PMC_SR_OSCSELS (0x1u << 7) /**< \brief (PMC_SR) Slow Clock Oscillator Selection */
#define PMC_SR_PCKRDY0 (0x1u << 8) /**< \brief (PMC_SR) Programmable Clock Ready Status */
#define PMC_SR_PCKRDY1 (0x1u << 9) /**< \brief (PMC_SR) Programmable Clock Ready Status */
#define PMC_SR_PCKRDY2 (0x1u << 10) /**< \brief (PMC_SR) Programmable Clock Ready Status */
#define PMC_SR_MOSCSELS (0x1u << 16) /**< \brief (PMC_SR) Main Oscillator Selection Status */
#define PMC_SR_MOSCRCS (0x1u << 17) /**< \brief (PMC_SR) Main On-Chip RC Oscillator Status */
#define PMC_SR_CFDEV (0x1u << 18) /**< \brief (PMC_SR) Clock Failure Detector Event */
#define PMC_SR_CFDS (0x1u << 19) /**< \brief (PMC_SR) Clock Failure Detector Status */
#define PMC_SR_FOS (0x1u << 20) /**< \brief (PMC_SR) Clock Failure Detector Fault Output Status */
/* -------- PMC_IMR : (PMC Offset: 0x006C) Interrupt Mask Register -------- */
#define PMC_IMR_MOSCXTS (0x1u << 0) /**< \brief (PMC_IMR) Main Crystal Oscillator Status Interrupt Mask */
#define PMC_IMR_LOCKA (0x1u << 1) /**< \brief (PMC_IMR) PLLA Lock Interrupt Mask */
#define PMC_IMR_LOCKB (0x1u << 2) /**< \brief (PMC_IMR) PLLB Lock Interrupt Mask */
#define PMC_IMR_MCKRDY (0x1u << 3) /**< \brief (PMC_IMR) Master Clock Ready Interrupt Mask */
#define PMC_IMR_PCKRDY0 (0x1u << 8) /**< \brief (PMC_IMR) Programmable Clock Ready 0 Interrupt Mask */
#define PMC_IMR_PCKRDY1 (0x1u << 9) /**< \brief (PMC_IMR) Programmable Clock Ready 1 Interrupt Mask */
#define PMC_IMR_PCKRDY2 (0x1u << 10) /**< \brief (PMC_IMR) Programmable Clock Ready 2 Interrupt Mask */
#define PMC_IMR_MOSCSELS (0x1u << 16) /**< \brief (PMC_IMR) Main Oscillator Selection Status Interrupt Mask */
#define PMC_IMR_MOSCRCS (0x1u << 17) /**< \brief (PMC_IMR) Main On-Chip RC Status Interrupt Mask */
#define PMC_IMR_CFDEV (0x1u << 18) /**< \brief (PMC_IMR) Clock Failure Detector Event Interrupt Mask */
/* -------- PMC_FSMR : (PMC Offset: 0x0070) Fast Startup Mode Register -------- */
#define PMC_FSMR_FSTT0 (0x1u << 0) /**< \brief (PMC_FSMR) Fast Startup Input Enable 0 */
#define PMC_FSMR_FSTT1 (0x1u << 1) /**< \brief (PMC_FSMR) Fast Startup Input Enable 1 */
#define PMC_FSMR_FSTT2 (0x1u << 2) /**< \brief (PMC_FSMR) Fast Startup Input Enable 2 */
#define PMC_FSMR_FSTT3 (0x1u << 3) /**< \brief (PMC_FSMR) Fast Startup Input Enable 3 */
#define PMC_FSMR_FSTT4 (0x1u << 4) /**< \brief (PMC_FSMR) Fast Startup Input Enable 4 */
#define PMC_FSMR_FSTT5 (0x1u << 5) /**< \brief (PMC_FSMR) Fast Startup Input Enable 5 */
#define PMC_FSMR_FSTT6 (0x1u << 6) /**< \brief (PMC_FSMR) Fast Startup Input Enable 6 */
#define PMC_FSMR_FSTT7 (0x1u << 7) /**< \brief (PMC_FSMR) Fast Startup Input Enable 7 */
#define PMC_FSMR_FSTT8 (0x1u << 8) /**< \brief (PMC_FSMR) Fast Startup Input Enable 8 */
#define PMC_FSMR_FSTT9 (0x1u << 9) /**< \brief (PMC_FSMR) Fast Startup Input Enable 9 */
#define PMC_FSMR_FSTT10 (0x1u << 10) /**< \brief (PMC_FSMR) Fast Startup Input Enable 10 */
#define PMC_FSMR_FSTT11 (0x1u << 11) /**< \brief (PMC_FSMR) Fast Startup Input Enable 11 */
#define PMC_FSMR_FSTT12 (0x1u << 12) /**< \brief (PMC_FSMR) Fast Startup Input Enable 12 */
#define PMC_FSMR_FSTT13 (0x1u << 13) /**< \brief (PMC_FSMR) Fast Startup Input Enable 13 */
#define PMC_FSMR_FSTT14 (0x1u << 14) /**< \brief (PMC_FSMR) Fast Startup Input Enable 14 */
#define PMC_FSMR_FSTT15 (0x1u << 15) /**< \brief (PMC_FSMR) Fast Startup Input Enable 15 */
#define PMC_FSMR_RTTAL (0x1u << 16) /**< \brief (PMC_FSMR) RTT Alarm Enable */
#define PMC_FSMR_RTCAL (0x1u << 17) /**< \brief (PMC_FSMR) RTC Alarm Enable */
#define PMC_FSMR_USBAL (0x1u << 18) /**< \brief (PMC_FSMR) USB Alarm Enable */
#define PMC_FSMR_LPM (0x1u << 20) /**< \brief (PMC_FSMR) Low Power Mode */
/* -------- PMC_FSPR : (PMC Offset: 0x0074) Fast Startup Polarity Register -------- */
#define PMC_FSPR_FSTP0 (0x1u << 0) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP1 (0x1u << 1) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP2 (0x1u << 2) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP3 (0x1u << 3) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP4 (0x1u << 4) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP5 (0x1u << 5) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP6 (0x1u << 6) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP7 (0x1u << 7) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP8 (0x1u << 8) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP9 (0x1u << 9) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP10 (0x1u << 10) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP11 (0x1u << 11) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP12 (0x1u << 12) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP13 (0x1u << 13) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP14 (0x1u << 14) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
#define PMC_FSPR_FSTP15 (0x1u << 15) /**< \brief (PMC_FSPR) Fast Startup Input Polarityx */
/* -------- PMC_FOCR : (PMC Offset: 0x0078) Fault Output Clear Register -------- */
#define PMC_FOCR_FOCLR (0x1u << 0) /**< \brief (PMC_FOCR) Fault Output Clear */
/* -------- PMC_WPMR : (PMC Offset: 0x00E4) Write Protect Mode Register -------- */
#define PMC_WPMR_WPEN (0x1u << 0) /**< \brief (PMC_WPMR) Write Protect Enable */
#define PMC_WPMR_WPKEY_Pos 8
#define PMC_WPMR_WPKEY_Msk (0xffffffu << PMC_WPMR_WPKEY_Pos) /**< \brief (PMC_WPMR) Write Protect KEY */
#define PMC_WPMR_WPKEY(value) ((PMC_WPMR_WPKEY_Msk & ((value) << PMC_WPMR_WPKEY_Pos)))
/* -------- PMC_WPSR : (PMC Offset: 0x00E8) Write Protect Status Register -------- */
#define PMC_WPSR_WPVS (0x1u << 0) /**< \brief (PMC_WPSR) Write Protect Violation Status */
#define PMC_WPSR_WPVSRC_Pos 8
#define PMC_WPSR_WPVSRC_Msk (0xffffu << PMC_WPSR_WPVSRC_Pos) /**< \brief (PMC_WPSR) Write Protect Violation Source */

/*@}*/


/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Reset Controller */
/* ============================================================================= */
/** \addtogroup SAM3U_RSTC Reset Controller */
/*@{*/

#ifndef __ASSEMBLY__
/** \brief Rstc hardware registers */
typedef struct {
  WoReg RSTC_CR; /**< \brief (Rstc Offset: 0x00) Control Register */
  RoReg RSTC_SR; /**< \brief (Rstc Offset: 0x04) Status Register */
  RwReg RSTC_MR; /**< \brief (Rstc Offset: 0x08) Mode Register */
} Rstc;
#endif /* __ASSEMBLY__ */
/* -------- RSTC_CR : (RSTC Offset: 0x00) Control Register -------- */
#define RSTC_CR_PROCRST (0x1u << 0) /**< \brief (RSTC_CR) Processor Reset */
#define RSTC_CR_PERRST (0x1u << 2) /**< \brief (RSTC_CR) Peripheral Reset */
#define RSTC_CR_EXTRST (0x1u << 3) /**< \brief (RSTC_CR) External Reset */
#define RSTC_CR_KEY_Pos 24
#define RSTC_CR_KEY_Msk (0xffu << RSTC_CR_KEY_Pos) /**< \brief (RSTC_CR) Password */
#define RSTC_CR_KEY(value) ((RSTC_CR_KEY_Msk & ((value) << RSTC_CR_KEY_Pos)))
/* -------- RSTC_SR : (RSTC Offset: 0x04) Status Register -------- */
#define RSTC_SR_URSTS (0x1u << 0) /**< \brief (RSTC_SR) User Reset Status */
#define RSTC_SR_RSTTYP_Pos 8
#define RSTC_SR_RSTTYP_Msk (0x7u << RSTC_SR_RSTTYP_Pos) /**< \brief (RSTC_SR) Reset Type */
#define RSTC_SR_NRSTL (0x1u << 16) /**< \brief (RSTC_SR) NRST Pin Level */
#define RSTC_SR_SRCMP (0x1u << 17) /**< \brief (RSTC_SR) Software Reset Command in Progress */
/* -------- RSTC_MR : (RSTC Offset: 0x08) Mode Register -------- */
#define RSTC_MR_URSTEN (0x1u << 0) /**< \brief (RSTC_MR) User Reset Enable */
#define RSTC_MR_URSTIEN (0x1u << 4) /**< \brief (RSTC_MR) User Reset Interrupt Enable */
#define RSTC_MR_ERSTL_Pos 8
#define RSTC_MR_ERSTL_Msk (0xfu << RSTC_MR_ERSTL_Pos) /**< \brief (RSTC_MR) External Reset Length */
#define RSTC_MR_ERSTL(value) ((RSTC_MR_ERSTL_Msk & ((value) << RSTC_MR_ERSTL_Pos)))
#define RSTC_MR_KEY_Pos 24
#define RSTC_MR_KEY_Msk (0xffu << RSTC_MR_KEY_Pos) /**< \brief (RSTC_MR) Password */
#define RSTC_MR_KEY(value) ((RSTC_MR_KEY_Msk & ((value) << RSTC_MR_KEY_Pos)))

/*@}*/


/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Universal Asynchronous Receiver Transmitter */
/* ============================================================================= */
/** \addtogroup SAM3U_UART Universal Asynchronous Receiver Transmitter */
/*@{*/

#ifndef __ASSEMBLY__
/** \brief Uart hardware registers */
typedef struct {
  WoReg UART_CR;       /**< \brief (Uart Offset: 0x0000) Control Register */
  RwReg UART_MR;       /**< \brief (Uart Offset: 0x0004) Mode Register */
  WoReg UART_IER;      /**< \brief (Uart Offset: 0x0008) Interrupt Enable Register */
  WoReg UART_IDR;      /**< \brief (Uart Offset: 0x000C) Interrupt Disable Register */
  RoReg UART_IMR;      /**< \brief (Uart Offset: 0x0010) Interrupt Mask Register */
  RoReg UART_SR;       /**< \brief (Uart Offset: 0x0014) Status Register */
  RoReg UART_RHR;      /**< \brief (Uart Offset: 0x0018) Receive Holding Register */
  WoReg UART_THR;      /**< \brief (Uart Offset: 0x001C) Transmit Holding Register */
  RwReg UART_BRGR;     /**< \brief (Uart Offset: 0x0020) Baud Rate Generator Register */
} Uart;
#endif /* __ASSEMBLY__ */
/* -------- UART_CR : (UART Offset: 0x0000) Control Register -------- */
#define UART_CR_RSTRX (0x1u << 2) /**< \brief (UART_CR) Reset Receiver */
#define UART_CR_RSTTX (0x1u << 3) /**< \brief (UART_CR) Reset Transmitter */
#define UART_CR_RXEN (0x1u << 4) /**< \brief (UART_CR) Receiver Enable */
#define UART_CR_RXDIS (0x1u << 5) /**< \brief (UART_CR) Receiver Disable */
#define UART_CR_TXEN (0x1u << 6) /**< \brief (UART_CR) Transmitter Enable */
#define UART_CR_TXDIS (0x1u << 7) /**< \brief (UART_CR) Transmitter Disable */
#define UART_CR_RSTSTA (0x1u << 8) /**< \brief (UART_CR) Reset Status Bits */
/* -------- UART_MR : (UART Offset: 0x0004) Mode Register -------- */
#define UART_MR_PAR_Pos 9
#define UART_MR_PAR_Msk (0x7u << UART_MR_PAR_Pos) /**< \brief (UART_MR) Parity Type */
#define   UART_MR_PAR_EVEN (0x0u << 9) /**< \brief (UART_MR) Even parity */
#define   UART_MR_PAR_ODD (0x1u << 9) /**< \brief (UART_MR) Odd parity */
#define   UART_MR_PAR_SPACE (0x2u << 9) /**< \brief (UART_MR) Space: parity forced to 0 */
#define   UART_MR_PAR_MARK (0x3u << 9) /**< \brief (UART_MR) Mark: parity forced to 1 */
#define   UART_MR_PAR_NO (0x4u << 9) /**< \brief (UART_MR) No parity */
#define UART_MR_CHMODE_Pos 14
#define UART_MR_CHMODE_Msk (0x3u << UART_MR_CHMODE_Pos) /**< \brief (UART_MR) Channel Mode */
#define   UART_MR_CHMODE_NORMAL (0x0u << 14) /**< \brief (UART_MR) Normal Mode */
#define   UART_MR_CHMODE_AUTOMATIC (0x1u << 14) /**< \brief (UART_MR) Automatic Echo */
#define   UART_MR_CHMODE_LOCAL_LOOPBACK (0x2u << 14) /**< \brief (UART_MR) Local Loopback */
#define   UART_MR_CHMODE_REMOTE_LOOPBACK (0x3u << 14) /**< \brief (UART_MR) Remote Loopback */
/* -------- UART_IER : (UART Offset: 0x0008) Interrupt Enable Register -------- */
#define UART_IER_RXRDY (0x1u << 0) /**< \brief (UART_IER) Enable RXRDY Interrupt */
#define UART_IER_TXRDY (0x1u << 1) /**< \brief (UART_IER) Enable TXRDY Interrupt */
#define UART_IER_ENDRX (0x1u << 3) /**< \brief (UART_IER) Enable End of Receive Transfer Interrupt */
#define UART_IER_ENDTX (0x1u << 4) /**< \brief (UART_IER) Enable End of Transmit Interrupt */
#define UART_IER_OVRE (0x1u << 5) /**< \brief (UART_IER) Enable Overrun Error Interrupt */
#define UART_IER_FRAME (0x1u << 6) /**< \brief (UART_IER) Enable Framing Error Interrupt */
#define UART_IER_PARE (0x1u << 7) /**< \brief (UART_IER) Enable Parity Error Interrupt */
#define UART_IER_TXEMPTY (0x1u << 9) /**< \brief (UART_IER) Enable TXEMPTY Interrupt */
#define UART_IER_TXBUFE (0x1u << 11) /**< \brief (UART_IER) Enable Buffer Empty Interrupt */
#define UART_IER_RXBUFF (0x1u << 12) /**< \brief (UART_IER) Enable Buffer Full Interrupt */
/* -------- UART_IDR : (UART Offset: 0x000C) Interrupt Disable Register -------- */
#define UART_IDR_RXRDY (0x1u << 0) /**< \brief (UART_IDR) Disable RXRDY Interrupt */
#define UART_IDR_TXRDY (0x1u << 1) /**< \brief (UART_IDR) Disable TXRDY Interrupt */
#define UART_IDR_ENDRX (0x1u << 3) /**< \brief (UART_IDR) Disable End of Receive Transfer Interrupt */
#define UART_IDR_ENDTX (0x1u << 4) /**< \brief (UART_IDR) Disable End of Transmit Interrupt */
#define UART_IDR_OVRE (0x1u << 5) /**< \brief (UART_IDR) Disable Overrun Error Interrupt */
#define UART_IDR_FRAME (0x1u << 6) /**< \brief (UART_IDR) Disable Framing Error Interrupt */
#define UART_IDR_PARE (0x1u << 7) /**< \brief (UART_IDR) Disable Parity Error Interrupt */
#define UART_IDR_TXEMPTY (0x1u << 9) /**< \brief (UART_IDR) Disable TXEMPTY Interrupt */
#define UART_IDR_TXBUFE (0x1u << 11) /**< \brief (UART_IDR) Disable Buffer Empty Interrupt */
#define UART_IDR_RXBUFF (0x1u << 12) /**< \brief (UART_IDR) Disable Buffer Full Interrupt */
/* -------- UART_IMR : (UART Offset: 0x0010) Interrupt Mask Register -------- */
#define UART_IMR_RXRDY (0x1u << 0) /**< \brief (UART_IMR) Mask RXRDY Interrupt */
#define UART_IMR_TXRDY (0x1u << 1) /**< \brief (UART_IMR) Disable TXRDY Interrupt */
#define UART_IMR_ENDRX (0x1u << 3) /**< \brief (UART_IMR) Mask End of Receive Transfer Interrupt */
#define UART_IMR_ENDTX (0x1u << 4) /**< \brief (UART_IMR) Mask End of Transmit Interrupt */
#define UART_IMR_OVRE (0x1u << 5) /**< \brief (UART_IMR) Mask Overrun Error Interrupt */
#define UART_IMR_FRAME (0x1u << 6) /**< \brief (UART_IMR) Mask Framing Error Interrupt */
#define UART_IMR_PARE (0x1u << 7) /**< \brief (UART_IMR) Mask Parity Error Interrupt */
#define UART_IMR_TXEMPTY (0x1u << 9) /**< \brief (UART_IMR) Mask TXEMPTY Interrupt */
#define UART_IMR_TXBUFE (0x1u << 11) /**< \brief (UART_IMR) Mask TXBUFE Interrupt */
#define UART_IMR_RXBUFF (0x1u << 12) /**< \brief (UART_IMR) Mask RXBUFF Interrupt */
/* -------- UART_SR : (UART Offset: 0x0014) Status Register -------- */
#define UART_SR_RXRDY (0x1u << 0) /**< \brief (UART_SR) Receiver Ready */
#define UART_SR_TXRDY (0x1u << 1) /**< \brief (UART_SR) Transmitter Ready */
#define UART_SR_ENDRX (0x1u << 3) /**< \brief (UART_SR) End of Receiver Transfer */
#define UART_SR_ENDTX (0x1u << 4) /**< \brief (UART_SR) End of Transmitter Transfer */
#define UART_SR_OVRE (0x1u << 5) /**< \brief (UART_SR) Overrun Error */
#define UART_SR_FRAME (0x1u << 6) /**< \brief (UART_SR) Framing Error */
#define UART_SR_PARE (0x1u << 7) /**< \brief (UART_SR) Parity Error */
#define UART_SR_TXEMPTY (0x1u << 9) /**< \brief (UART_SR) Transmitter Empty */
#define UART_SR_TXBUFE (0x1u << 11) /**< \brief (UART_SR) Transmission Buffer Empty */
#define UART_SR_RXBUFF (0x1u << 12) /**< \brief (UART_SR) Receive Buffer Full */
/* -------- UART_RHR : (UART Offset: 0x0018) Receive Holding Register -------- */
#define UART_RHR_RXCHR_Pos 0
#define UART_RHR_RXCHR_Msk (0xffu << UART_RHR_RXCHR_Pos) /**< \brief (UART_RHR) Received Character */
/* -------- UART_THR : (UART Offset: 0x001C) Transmit Holding Register -------- */
#define UART_THR_TXCHR_Pos 0
#define UART_THR_TXCHR_Msk (0xffu << UART_THR_TXCHR_Pos) /**< \brief (UART_THR) Character to be Transmitted */
#define UART_THR_TXCHR(value) ((UART_THR_TXCHR_Msk & ((value) << UART_THR_TXCHR_Pos)))
/* -------- UART_BRGR : (UART Offset: 0x0020) Baud Rate Generator Register -------- */
#define UART_BRGR_CD_Pos 0
#define UART_BRGR_CD_Msk (0xffffu << UART_BRGR_CD_Pos) /**< \brief (UART_BRGR) Clock Divisor */
#define UART_BRGR_CD(value) ((UART_BRGR_CD_Msk & ((value) << UART_BRGR_CD_Pos)))

/*@}*/


/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR Watchdog Timer */
/* ============================================================================= */
/** \addtogroup SAM3U_WDT Watchdog Timer */
/*@{*/

#ifndef __ASSEMBLY__
/** \brief Wdt hardware registers */
typedef struct {
  WoReg WDT_CR; /**< \brief (Wdt Offset: 0x00) Control Register */
  RwReg WDT_MR; /**< \brief (Wdt Offset: 0x04) Mode Register */
  RoReg WDT_SR; /**< \brief (Wdt Offset: 0x08) Status Register */
} Wdt;
#endif /* __ASSEMBLY__ */
/* -------- WDT_CR : (WDT Offset: 0x00) Control Register -------- */
#define WDT_CR_WDRSTT (0x1u << 0) /**< \brief (WDT_CR) Watchdog Restart */
#define WDT_CR_KEY_Pos 24
#define WDT_CR_KEY_Msk (0xffu << WDT_CR_KEY_Pos) /**< \brief (WDT_CR) Password */
#define WDT_CR_KEY(value) ((WDT_CR_KEY_Msk & ((value) << WDT_CR_KEY_Pos)))
/* -------- WDT_MR : (WDT Offset: 0x04) Mode Register -------- */
#define WDT_MR_WDV_Pos 0
#define WDT_MR_WDV_Msk (0xfffu << WDT_MR_WDV_Pos) /**< \brief (WDT_MR) Watchdog Counter Value */
#define WDT_MR_WDV(value) ((WDT_MR_WDV_Msk & ((value) << WDT_MR_WDV_Pos)))
#define WDT_MR_WDFIEN (0x1u << 12) /**< \brief (WDT_MR) Watchdog Fault Interrupt Enable */
#define WDT_MR_WDRSTEN (0x1u << 13) /**< \brief (WDT_MR) Watchdog Reset Enable */
#define WDT_MR_WDRPROC (0x1u << 14) /**< \brief (WDT_MR) Watchdog Reset Processor */
#define WDT_MR_WDDIS (0x1u << 15) /**< \brief (WDT_MR) Watchdog Disable */
#define WDT_MR_WDD_Pos 16
#define WDT_MR_WDD_Msk (0xfffu << WDT_MR_WDD_Pos) /**< \brief (WDT_MR) Watchdog Delta Value */
#define WDT_MR_WDD(value) ((WDT_MR_WDD_Msk & ((value) << WDT_MR_WDD_Pos)))
#define WDT_MR_WDDBGHLT (0x1u << 28) /**< \brief (WDT_MR) Watchdog Debug Halt */
#define WDT_MR_WDIDLEHLT (0x1u << 29) /**< \brief (WDT_MR) Watchdog Idle Halt */
/* -------- WDT_SR : (WDT Offset: 0x08) Status Register -------- */
#define WDT_SR_WDUNF (0x1u << 0) /**< \brief (WDT_SR) Watchdog Underflow */
#define WDT_SR_WDERR (0x1u << 1) /**< \brief (WDT_SR) Watchdog Error */

/*@}*/

/* ============================================================================= */
/**  SOFTWARE API DEFINITION FOR AHB Bus Matrix */
/* ============================================================================= */
/** \addtogroup SAM3S_MATRIX AHB Bus Matrix */
/*@{*/

#ifndef __ASSEMBLY__
/** \brief Matrix hardware registers */
typedef struct {
   AT91_REG  HMATRIX2_MCFG0;  //  Master Configuration Register 0 : ARM I and D
   AT91_REG  HMATRIX2_MCFG1;  //  Master Configuration Register 1 : ARM S
   AT91_REG  HMATRIX2_MCFG2;  //  Master Configuration Register 2
   AT91_REG  HMATRIX2_MCFG3;  //  Master Configuration Register 3
   AT91_REG  HMATRIX2_MCFG4;  //  Master Configuration Register 4
   AT91_REG  HMATRIX2_MCFG5;  //  Master Configuration Register 5
   AT91_REG  HMATRIX2_MCFG6;  //  Master Configuration Register 6
   AT91_REG  HMATRIX2_MCFG7;  //  Master Configuration Register 7
   AT91_REG  Reserved0[8];    // 
   AT91_REG  HMATRIX2_SCFG0;  //  Slave Configuration Register 0
   AT91_REG  HMATRIX2_SCFG1;  //  Slave Configuration Register 1
   AT91_REG  HMATRIX2_SCFG2;  //  Slave Configuration Register 2
   AT91_REG  HMATRIX2_SCFG3;  //  Slave Configuration Register 3
   AT91_REG  HMATRIX2_SCFG4;  //  Slave Configuration Register 4
   AT91_REG  HMATRIX2_SCFG5;  //  Slave Configuration Register 5
   AT91_REG  HMATRIX2_SCFG6;  //  Slave Configuration Register 6
   AT91_REG  HMATRIX2_SCFG7;  //  Slave Configuration Register 5
   AT91_REG  HMATRIX2_SCFG8;  //  Slave Configuration Register 8
   AT91_REG  HMATRIX2_SCFG9;  //  Slave Configuration Register 9
   AT91_REG  Reserved1[42];   // 
   AT91_REG  HMATRIX2_SFR0 ;  //  Special Function Register 0
   AT91_REG  HMATRIX2_SFR1 ;  //  Special Function Register 1
   AT91_REG  HMATRIX2_SFR2 ;  //  Special Function Register 2
   AT91_REG  HMATRIX2_SFR3 ;  //  Special Function Register 3
   AT91_REG  HMATRIX2_SFR4 ;  //  Special Function Register 4
   AT91_REG  HMATRIX2_SFR5 ;  //  Special Function Register 5
   AT91_REG  HMATRIX2_SFR6 ;  //  Special Function Register 6
   AT91_REG  HMATRIX2_SFR7 ;  //  Special Function Register 7
   AT91_REG  HMATRIX2_SFR8 ;  //  Special Function Register 8
   AT91_REG  HMATRIX2_SFR9 ;  //  Special Function Register 9
   AT91_REG  HMATRIX2_SFR10;  //  Special Function Register 10
   AT91_REG  HMATRIX2_SFR11;  //  Special Function Register 11
   AT91_REG  HMATRIX2_SFR12;  //  Special Function Register 12
   AT91_REG  HMATRIX2_SFR13;  //  Special Function Register 13
   AT91_REG  HMATRIX2_SFR14;  //  Special Function Register 14
   AT91_REG  HMATRIX2_SFR15;  //  Special Function Register 15
   AT91_REG  Reserved2[39];   // 
   AT91_REG  HMATRIX2_ADDRSIZE;  // HMATRIX2 ADDRSIZE REGISTER 
   AT91_REG  HMATRIX2_IPNAME1;   // HMATRIX2 IPNAME1 REGISTER 
   AT91_REG  HMATRIX2_IPNAME2;   // HMATRIX2 IPNAME2 REGISTER 
   AT91_REG  HMATRIX2_FEATURES;  // HMATRIX2 FEATURES REGISTER 
   AT91_REG  HMATRIX2_VER;    // HMATRIX2 VERSION REGISTER 
} Matrix;
#endif /* __ASSEMBLY__ */

/* -------- MATRIX_SCFG : Slave Configuration Register -------- */
#define AT91C_MATRIX_DEFMSTR_TYPE_LAST_DEFMSTR     (0x1 << 16) // (HMATRIX2) Last Default Master. At the end of current slave access, if no other master request is pending, the slave stay connected with the last master having accessed it. This results in not having the one cycle latency when the last master re-trying access on the slave.
#define AT91C_MATRIX_DEFMSTR_TYPE_FIXED_DEFMSTR    (0x2 << 16) // (HMATRIX2) Fixed Default Master. At the end of current slave access, if no other master request is pending, the slave connects with fixed which number is in FIXED_DEFMSTR field. This results in not having the one cycle latency when the fixed master re-trying access on the slave.
#define AT91C_MATRIX_DEFMSTR_TYPE                  (0x3 << 16) // (HMATRIX2) Default Master Type
#define AT91C_MATRIX_FIXED_DEFMSTR_SCFG_ARMS       (0x1 << 18) // (HMATRIX2) ARMS is Default Master
#define AT91C_MATRIX_FIXED_DEFMSTR_SCFG_ARMC       (0x0 << 18) // (HMATRIX2) ARMC is Default Master

/*@}*/

/*@}*/


/* ************************************************************************** */
/*   PERIPHERAL ID DEFINITIONS FOR SAM3U */
/* ************************************************************************** */
/** \addtogroup SAM3U_id Peripheral Ids Definitions */
/*@{*/

#define ID_SUPC   ( 0) /**< \brief Supply Controller (SUPC) */
#define ID_RSTC   ( 1) /**< \brief Reset Controller (RSTC) */
#define ID_RTC    ( 2) /**< \brief Real Time Clock (RTC) */
#define ID_RTT    ( 3) /**< \brief Real Time Timer (RTT) */
#define ID_WDT    ( 4) /**< \brief Watchdog Timer (WDT) */
#define ID_PMC    ( 5) /**< \brief Power Management Controller (PMC) */
#define ID_EFC0   ( 6) /**< \brief Enhanced Embedded Flash Controller 0 (EFC0) */
#define ID_EFC1   ( 7) /**< \brief Enhanced Embedded Flash Controller 1 (EFC1) */
#define ID_UART   ( 8) /**< \brief UART (UART) */
#define ID_SMC    ( 9) /**< \brief Static Memory Controller (SMC) */
#define ID_PIOA   (10) /**< \brief Parallel I/O Controller A (PIOA) */
#define ID_PIOB   (11) /**< \brief Parallel I/O Controller B (PIOB) */
#define ID_PIOC   (12) /**< \brief Parallel I/O Controller C (PIOC) */
#define ID_USART0 (13) /**< \brief USART 0 (USART0) */
#define ID_USART1 (14) /**< \brief USART 2 (USART1) */
#define ID_USART2 (15) /**< \brief USART 3 (USART2) */
#define ID_USART3 (16) /**< \brief USART 4 (USART3) */
#define ID_HSMCI  (17) /**< \brief Multimedia Card Interface (HSMCI) */
#define ID_TWI0   (18) /**< \brief Two Wire Interface 0 (TWI0) */
#define ID_TWI1   (19) /**< \brief Two Wire Interface 1 (TWI1) */
#define ID_SPI    (20) /**< \brief Serial Peripheral Interface (SPI) */
#define ID_SSC    (21) /**< \brief Synchronous Serial Controler (SSC) */
#define ID_TC0    (22) /**< \brief Timer/Counter 0 (TC0) */
#define ID_TC1    (23) /**< \brief Timer/Counter 1 (TC1) */
#define ID_TC2    (24) /**< \brief Timer/Counter 2 (TC2) */
#define ID_PWM    (25) /**< \brief Pulse Width Modulation (PWM) */
#define ID_ADC12B (26) /**< \brief Analog To Digital Converter (ADC12B) */
#define ID_ADC    (27) /**< \brief Analog To Digital Converter (ADC) */

#define ID_DMAC   (28) /**< \brief DMA Controller (DMAC) */
#define ID_UDPHS  (29) /**< \brief USB Device High Speed (UDPHS) */
/*@}*/

/* ************************************************************************** */
/*   BASE ADDRESS DEFINITIONS FOR SAM3U */
/* ************************************************************************** */
/** \addtogroup SAM3U_base Peripheral Base Address Definitions */
/*@{*/

#define MATRIX     CAST(Matrix    , 0x400E0200U) /**< \brief (MATRIX    ) Base Address */
#define PMC        CAST(Pmc       , 0x400E0400U) /**< \brief (PMC       ) Base Address */
#define UART0      CAST(Uart      , 0x400E0600U) /**< \brief (UART0     ) Base Address */
#define EFC0       CAST(Efc       , 0x400E0800U) /**< \brief (EFC0       ) Base Address */
#define EFC1       CAST(Efc       , 0x400E0A00U) /**< \brief (EFC1       ) Base Address */
#define PIOA       CAST(Pio       , 0x400E0C00U) /**< \brief (PIOA      ) Base Address */
#define PIOB       CAST(Pio       , 0x400E0E00U) /**< \brief (PIOB      ) Base Address */
#define PIOC       CAST(Pio       , 0x400E1000U) /**< \brief (PIOC      ) Base Address */
#define RSTC       CAST(Rstc      , 0x400E1200U) /**< \brief (RSTC      ) Base Address */
#define SUPC       CAST(Supc      , 0x400E1210U) /**< \brief (SUPC      ) Base Address */
#define WDT        CAST(Wdt       , 0x400E1250U) /**< \brief (WDT       ) Base Address */

/*@}*/

/* ************************************************************************** */
/*   PIO DEFINITIONS FOR SAM3U */
/* ************************************************************************** */
/** \addtogroup SAM3U_pio Peripheral Pio Definitions */
/*@{*/

#define PIO_PA0                (1u << 0) /**< \brief Pin Controlled by PA0 */
#define PIO_PA1                (1u << 1) /**< \brief Pin Controlled by PA1 */
#define PIO_PA2                (1u << 2) /**< \brief Pin Controlled by PA2 */
#define PIO_PA3                (1u << 3) /**< \brief Pin Controlled by PA3 */
#define PIO_PA4                (1u << 4) /**< \brief Pin Controlled by PA4 */
#define PIO_PA5                (1u << 5) /**< \brief Pin Controlled by PA5 */
#define PIO_PA6                (1u << 6) /**< \brief Pin Controlled by PA6 */
#define PIO_PA7                (1u << 7) /**< \brief Pin Controlled by PA7 */
#define PIO_PA8                (1u << 8) /**< \brief Pin Controlled by PA8 */
#define PIO_PA9                (1u << 9) /**< \brief Pin Controlled by PA9 */
#define PIO_PA10               (1u << 10) /**< \brief Pin Controlled by PA10 */
#define PIO_PA11               (1u << 11) /**< \brief Pin Controlled by PA11 */
#define PIO_PA12               (1u << 12) /**< \brief Pin Controlled by PA12 */
#define PIO_PA13               (1u << 13) /**< \brief Pin Controlled by PA13 */
#define PIO_PA14               (1u << 14) /**< \brief Pin Controlled by PA14 */
#define PIO_PA15               (1u << 15) /**< \brief Pin Controlled by PA15 */
#define PIO_PA16               (1u << 16) /**< \brief Pin Controlled by PA16 */
#define PIO_PA17               (1u << 17) /**< \brief Pin Controlled by PA17 */
#define PIO_PA18               (1u << 18) /**< \brief Pin Controlled by PA18 */
#define PIO_PA19               (1u << 19) /**< \brief Pin Controlled by PA19 */
#define PIO_PA20               (1u << 20) /**< \brief Pin Controlled by PA20 */
#define PIO_PA21               (1u << 21) /**< \brief Pin Controlled by PA21 */
#define PIO_PA22               (1u << 22) /**< \brief Pin Controlled by PA22 */
#define PIO_PA23               (1u << 23) /**< \brief Pin Controlled by PA23 */
#define PIO_PA24               (1u << 24) /**< \brief Pin Controlled by PA24 */
#define PIO_PA25               (1u << 25) /**< \brief Pin Controlled by PA25 */
#define PIO_PA26               (1u << 26) /**< \brief Pin Controlled by PA26 */
#define PIO_PA27               (1u << 27) /**< \brief Pin Controlled by PA27 */
#define PIO_PA28               (1u << 28) /**< \brief Pin Controlled by PA28 */
#define PIO_PA29               (1u << 29) /**< \brief Pin Controlled by PA29 */
#define PIO_PA30               (1u << 30) /**< \brief Pin Controlled by PA30 */
#define PIO_PA31               (1u << 31) /**< \brief Pin Controlled by PA31 */
#define PIO_PB0                (1u << 0) /**< \brief Pin Controlled by PB0 */
#define PIO_PB1                (1u << 1) /**< \brief Pin Controlled by PB1 */
#define PIO_PB2                (1u << 2) /**< \brief Pin Controlled by PB2 */
#define PIO_PB3                (1u << 3) /**< \brief Pin Controlled by PB3 */
#define PIO_PB4                (1u << 4) /**< \brief Pin Controlled by PB4 */
#define PIO_PB5                (1u << 5) /**< \brief Pin Controlled by PB5 */
#define PIO_PB6                (1u << 6) /**< \brief Pin Controlled by PB6 */
#define PIO_PB7                (1u << 7) /**< \brief Pin Controlled by PB7 */
#define PIO_PB8                (1u << 8) /**< \brief Pin Controlled by PB8 */
#define PIO_PB9                (1u << 9) /**< \brief Pin Controlled by PB9 */
#define PIO_PB10               (1u << 10) /**< \brief Pin Controlled by PB10 */
#define PIO_PB11               (1u << 11) /**< \brief Pin Controlled by PB11 */
#define PIO_PB12               (1u << 12) /**< \brief Pin Controlled by PB12 */
#define PIO_PB13               (1u << 13) /**< \brief Pin Controlled by PB13 */
#define PIO_PB14               (1u << 14) /**< \brief Pin Controlled by PB14 */
#define PIO_PC0                (1u << 0) /**< \brief Pin Controlled by PC0 */
#define PIO_PC1                (1u << 1) /**< \brief Pin Controlled by PC1 */
#define PIO_PC2                (1u << 2) /**< \brief Pin Controlled by PC2 */
#define PIO_PC3                (1u << 3) /**< \brief Pin Controlled by PC3 */
#define PIO_PC4                (1u << 4) /**< \brief Pin Controlled by PC4 */
#define PIO_PC5                (1u << 5) /**< \brief Pin Controlled by PC5 */
#define PIO_PC6                (1u << 6) /**< \brief Pin Controlled by PC6 */
#define PIO_PC7                (1u << 7) /**< \brief Pin Controlled by PC7 */
#define PIO_PC8                (1u << 8) /**< \brief Pin Controlled by PC8 */
#define PIO_PC9                (1u << 9) /**< \brief Pin Controlled by PC9 */
#define PIO_PC10               (1u << 10) /**< \brief Pin Controlled by PC10 */
#define PIO_PC11               (1u << 11) /**< \brief Pin Controlled by PC11 */
#define PIO_PC12               (1u << 12) /**< \brief Pin Controlled by PC12 */
#define PIO_PC13               (1u << 13) /**< \brief Pin Controlled by PC13 */
#define PIO_PC14               (1u << 14) /**< \brief Pin Controlled by PC14 */
#define PIO_PC15               (1u << 15) /**< \brief Pin Controlled by PC15 */
#define PIO_PC16               (1u << 16) /**< \brief Pin Controlled by PC16 */
#define PIO_PC17               (1u << 17) /**< \brief Pin Controlled by PC17 */
#define PIO_PC18               (1u << 18) /**< \brief Pin Controlled by PC18 */
#define PIO_PC19               (1u << 19) /**< \brief Pin Controlled by PC19 */
#define PIO_PC20               (1u << 20) /**< \brief Pin Controlled by PC20 */
#define PIO_PC21               (1u << 21) /**< \brief Pin Controlled by PC21 */
#define PIO_PC22               (1u << 22) /**< \brief Pin Controlled by PC22 */
#define PIO_PC23               (1u << 23) /**< \brief Pin Controlled by PC23 */
#define PIO_PC24               (1u << 24) /**< \brief Pin Controlled by PC24 */
#define PIO_PC25               (1u << 25) /**< \brief Pin Controlled by PC25 */
#define PIO_PC26               (1u << 26) /**< \brief Pin Controlled by PC26 */
#define PIO_PC27               (1u << 27) /**< \brief Pin Controlled by PC27 */
#define PIO_PC28               (1u << 28) /**< \brief Pin Controlled by PC28 */
#define PIO_PC29               (1u << 29) /**< \brief Pin Controlled by PC29 */
#define PIO_PC30               (1u << 30) /**< \brief Pin Controlled by PC30 */
#define PIO_PC31               (1u << 31) /**< \brief Pin Controlled by PC31 */

#ifdef __cplusplus
}
#endif

/*@}*/

#endif /* SAM3S_H */
