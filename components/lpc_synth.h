/*******************************************************************************
* Written by Jaroslav Groman, for PSoC Analog Coprocessor
* https://www.hackster.io/jardag/touch-controlled-talking-clock-for-psoc-analog-coprocessor-98c5a0
*
* Modified by Maanika Kenneth Koththioda, for use on PSoC5LP
* Last Modified on 25/10/2020
*
* File:     lpc_synth.h
* Version:  1.0.0
*
* Brief: Project independent LPC audio decoder implementation.
*
* Target device:
*    CY8C5888LTI - LP097
*
* Code Tested With:
*    - Silicon: PSoC 5LP
*    - IDE: PSoC Creator 4.3
*    - Compiler: GCC 5.4
*
* Components:
*    - cy_clock       [Clock_Synth]
*    - TCPWM_P4       [Timer_Synth]
*    - cy_isr         [isr_Synth]
*    - UAB_VDAC       [VDAC_Synth]
*    - cy_pins        [Pin_Synth_Out]
*
* Source:
*    LPC synth code adapted from https://github.com/going-digital/Talkie
*
*******************************************************************************/
#ifndef LPC_SYNTH_H
	#define LPC_SYNTH_H
    
/*******************************************************************************
*   Included Headers
*******************************************************************************/
    #include <project.h>

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/
    /* Synthesizer timer clock frequency in Hz */
    #define CLOCK_SYNTH  1000000
        
    /* @brief LPC audio sample rate in Hz */
    #define SAMPLE_RATE     8000

    /* @brief LPC chirp sample size in bytes */
    #define CHIRP_SIZE        41

/*******************************************************************************
*   Function Declarations
*******************************************************************************/

    // Brief: Initialize synthesizer hardware components.
    // Return: none
    void synthInitialize(void);

    // Brief: Generate speech audio based on LPC encoded bitstream.
    // Param: p_lpc_data Pointer to LPC encoded bitstream data.
    // Return: none
    void synth_say(const uint8_t* p_lpc_data);

    // Brief: Synth ISR prototype.
    // Param: synth_isr ISR name.
    // Return: none
    CY_ISR_PROTO(synth_isr);
    
#endif

/* [] END OF FILE */
