/*******************************************************************************
* Written by Maanika Kenneth Koththioda, for PSoC5LP
* Last Modified on 4/1/2021
*
* File: sound.h
* Version: 1.0.0
*
* Brief: Sound Generator - Sine waves with arbitary phase
*        frequency, and amplitude.
*
* Target device:
*    CY8C5888LTI - LP097
*
* Code Tested With:
*    - Silicon: PSoC 5LP
*    - IDE: PSoC Creator 4.3
*    - Compiler: GCC 5.4
*
* Notes:
*  ALWAYS remeber to CALL dmaConfiguration() 
*  AND sineWaveInitialize() IN MAIN Program.
*  MAX STABLE FREQ IS 19KHZ
*  MIN STABLE FREQ 100Hz
*  ______________________________________
* |          |           |       |       |
* | Pin Name | Pin       | Value | Ear   |
* |----------|-----------|-------|-------|
* | Vout1    | Pin3.5    | 1     | LEFT  |
* | Vout2    | Pin3.7    | 2     | RIGHT |
* |__________|___________|_______|_______|
*
* Sources:
*   Freuency control is obtained using the Digital Direct Sysnthesis component
*   from:
*https://community.cypress.com/t5/Code-Examples/DDS24-24-bit-DDS-arbitrary-frequency-generator-component/td-p/46572?start=0&tstart=0
*
*******************************************************************************/
#ifndef SOUND_H
#define SOUND_H
    
/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include "project.h"
    
/*******************************************************************************
*   Function Declarations
*******************************************************************************/

// Brief: Starts sound peripherals.
// Param:  none.
// Return: none 
void startSoundComponents(void);

// Brief: Sets up DMA configs.
// Param:  none.
// Return: none 
void dmaConfiguration(void);

// Brief: initializes two seperate sine tables for each channel (left/right ear).
// Param:  frequency
// Return: none 
void sineWaveInitialize(int freq);

// Brief: sets phase and amplitude of the sinewave.
// Param:  offset cycles, attenutaion, sinewave number (which channel).
// Return:  1 or 0. 
int updateSineWave(uint8 phase, double att,int waveNum);

#endif

/* [] END OF FILE */
