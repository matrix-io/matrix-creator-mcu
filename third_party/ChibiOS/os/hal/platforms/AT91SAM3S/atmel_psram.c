#include "board.h"
#include "pmc.h"

extern void BOARD_ConfigurePSRAM( Smc* pSmc )
{
    uint32_t dwTmp ;

    /* Enable peripheral clock */
    PMC_EnablePeripheral( ID_SMC ) ;

    /* Configure SMC, NCS3 is assigned to a external PSRAM */
    /**
     * PSRAM IS66WV51216BLL
     * 55 ns Access time
     * tdoe = 25 ns max
     * SMC1 (timing SAM3S read mode SMC) = 21 ns of setup
     * 21 + 55 = 76 ns => at least 5 cycles at 64 MHz
     * Write pulse width minimum = 45 ns (PSRAM)
     */
    pSmc->SMC_CS_NUMBER[3].SMC_SETUP = SMC_SETUP_NWE_SETUP( 3 )
                                     | SMC_SETUP_NCS_WR_SETUP( 4 )
                                     | SMC_SETUP_NRD_SETUP( 3 )
                                     | SMC_SETUP_NCS_RD_SETUP( 5 ) ;

    pSmc->SMC_CS_NUMBER[3].SMC_PULSE = SMC_PULSE_NWE_PULSE( 3 )
                                     | SMC_PULSE_NCS_WR_PULSE( 4 )
                                     | SMC_PULSE_NRD_PULSE( 3 )
                                     | SMC_PULSE_NCS_RD_PULSE( 5 ) ;

    /* NWE_CYCLE:     The total duration of the write cycle.
                      NWE_CYCLE = NWE_SETUP + NWE_PULSE + NWE_HOLD
                      = NCS_WR_SETUP + NCS_WR_PULSE + NCS_WR_HOLD
                      (tWC) Write Cycle Time min. 70ns
       NRD_CYCLE:     The total duration of the read cycle.
                      NRD_CYCLE = NRD_SETUP + NRD_PULSE + NRD_HOLD
                      = NCS_RD_SETUP + NCS_RD_PULSE + NCS_RD_HOLD
                      (tRC) Read Cycle Time min. 70ns. */
    pSmc->SMC_CS_NUMBER[3].SMC_CYCLE = SMC_CYCLE_NWE_CYCLE( 4 )
                                     | SMC_CYCLE_NRD_CYCLE( 5 ) ;

    dwTmp = SMC->SMC_CS_NUMBER[3].SMC_MODE & (uint32_t)(~(SMC_MODE_DBW_Msk)) ;
    pSmc->SMC_CS_NUMBER[3].SMC_MODE  = dwTmp
                                     | SMC_MODE_READ_MODE
                                     | SMC_MODE_WRITE_MODE
                                     | SMC_MODE_DBW_8_BIT ;
}
