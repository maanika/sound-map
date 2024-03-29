/*******************************************************************************
* Written by Maanika Kenneth Koththioda, for PSoC5LP
* Last Modified on 4/1/2021
*
* File: sound.c
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
*******************************************************************************
*   Included Headers
*******************************************************************************/
#include "project.h"
#include "stdio.h"
#include "sound.h"

/*******************************************************************************
*   Constant definitions
*******************************************************************************/

/* DMA Configs */
#define DMA_BYTES_PER_BURST 1
#define DMA_REQUEST_PER_BURST 1

/* Sine Table Length */
#define TABLE_LENGTH   720

/* Sine look up table with 180 points stored in Flash */
CYCODE const uint8 sineTable[1440] = {
0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x8a,
0x8b,0x8c,0x8d,0x8e,0x8f,0x90,0x91,0x92,0x93,0x95,
0x96,0x97,0x98,0x99,0x9a,0x9b,0x9c,0x9d,0x9e,0x9f,
0xa0,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,
0xab,0xac,0xad,0xae,0xaf,0xb0,0xb1,0xb2,0xb3,0xb4,
0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xbb,0xbc,0xbd,0xbe,
0xbf,0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,
0xc9,0xca,0xca,0xcb,0xcc,0xcd,0xce,0xcf,0xd0,0xd1,
0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd6,0xd7,0xd8,0xd9,
0xda,0xda,0xdb,0xdc,0xdd,0xde,0xde,0xdf,0xe0,0xe0,
0xe1,0xe2,0xe3,0xe3,0xe4,0xe5,0xe5,0xe6,0xe7,0xe7,
0xe8,0xe9,0xe9,0xea,0xea,0xeb,0xec,0xec,0xed,0xed,
0xee,0xee,0xef,0xf0,0xf0,0xf1,0xf1,0xf2,0xf2,0xf3,
0xf3,0xf4,0xf4,0xf4,0xf5,0xf5,0xf6,0xf6,0xf7,0xf7,
0xf7,0xf8,0xf8,0xf8,0xf9,0xf9,0xf9,0xfa,0xfa,0xfa,
0xfb,0xfb,0xfb,0xfb,0xfc,0xfc,0xfc,0xfc,0xfd,0xfd,
0xfd,0xfd,0xfd,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfd,0xfd,
0xfd,0xfd,0xfd,0xfc,0xfc,0xfc,0xfc,0xfb,0xfb,0xfb,
0xfb,0xfa,0xfa,0xfa,0xf9,0xf9,0xf9,0xf8,0xf8,0xf8,
0xf7,0xf7,0xf7,0xf6,0xf6,0xf5,0xf5,0xf4,0xf4,0xf4,
0xf3,0xf3,0xf2,0xf2,0xf1,0xf1,0xf0,0xf0,0xef,0xee,
0xee,0xed,0xed,0xec,0xec,0xeb,0xea,0xea,0xe9,0xe9,
0xe8,0xe7,0xe7,0xe6,0xe5,0xe5,0xe4,0xe3,0xe3,0xe2,
0xe1,0xe0,0xe0,0xdf,0xde,0xde,0xdd,0xdc,0xdb,0xda,
0xda,0xd9,0xd8,0xd7,0xd6,0xd6,0xd5,0xd4,0xd3,0xd2,
0xd1,0xd1,0xd0,0xcf,0xce,0xcd,0xcc,0xcb,0xca,0xca,
0xc9,0xc8,0xc7,0xc6,0xc5,0xc4,0xc3,0xc2,0xc1,0xc0,
0xbf,0xbe,0xbd,0xbc,0xbb,0xba,0xb9,0xb8,0xb7,0xb6,
0xb5,0xb4,0xb3,0xb2,0xb1,0xb0,0xaf,0xae,0xad,0xac,
0xab,0xaa,0xa9,0xa8,0xa7,0xa6,0xa5,0xa4,0xa3,0xa2,
0xa0,0x9f,0x9e,0x9d,0x9c,0x9b,0x9a,0x99,0x98,0x97,
0x96,0x95,0x93,0x92,0x91,0x90,0x8f,0x8e,0x8d,0x8c,
0x8b,0x8a,0x88,0x87,0x86,0x85,0x84,0x83,0x82,0x81,
0x80,0x7e,0x7d,0x7c,0x7b,0x7a,0x79,0x78,0x77,0x75,
0x74,0x73,0x72,0x71,0x70,0x6f,0x6e,0x6d,0x6c,0x6a,
0x69,0x68,0x67,0x66,0x65,0x64,0x63,0x62,0x61,0x60,
0x5f,0x5d,0x5c,0x5b,0x5a,0x59,0x58,0x57,0x56,0x55,
0x54,0x53,0x52,0x51,0x50,0x4f,0x4e,0x4d,0x4c,0x4b,
0x4a,0x49,0x48,0x47,0x46,0x45,0x44,0x43,0x42,0x41,
0x40,0x3f,0x3e,0x3d,0x3c,0x3b,0x3a,0x39,0x38,0x37,
0x36,0x35,0x35,0x34,0x33,0x32,0x31,0x30,0x2f,0x2e,
0x2e,0x2d,0x2c,0x2b,0x2a,0x29,0x29,0x28,0x27,0x26,
0x25,0x25,0x24,0x23,0x22,0x21,0x21,0x20,0x1f,0x1f,
0x1e,0x1d,0x1c,0x1c,0x1b,0x1a,0x1a,0x19,0x18,0x18,
0x17,0x16,0x16,0x15,0x15,0x14,0x13,0x13,0x12,0x12,
0x11,0x11,0x10,0xf,0xf,0xe,0xe,0xd,0xd,0xc,
0xc,0xb,0xb,0xb,0xa,0xa,0x9,0x9,0x8,0x8,
0x8,0x7,0x7,0x7,0x6,0x6,0x6,0x5,0x5,0x5,
0x4,0x4,0x4,0x4,0x3,0x3,0x3,0x3,0x2,0x2,
0x2,0x2,0x2,0x1,0x1,0x1,0x1,0x1,0x1,0x1,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x2,0x2,
0x2,0x2,0x2,0x3,0x3,0x3,0x3,0x4,0x4,0x4,
0x4,0x5,0x5,0x5,0x6,0x6,0x6,0x7,0x7,0x7,
0x8,0x8,0x8,0x9,0x9,0xa,0xa,0xb,0xb,0xb,
0xc,0xc,0xd,0xd,0xe,0xe,0xf,0xf,0x10,0x11,
0x11,0x12,0x12,0x13,0x13,0x14,0x15,0x15,0x16,0x16,
0x17,0x18,0x18,0x19,0x1a,0x1a,0x1b,0x1c,0x1c,0x1d,
0x1e,0x1f,0x1f,0x20,0x21,0x21,0x22,0x23,0x24,0x25,
0x25,0x26,0x27,0x28,0x29,0x29,0x2a,0x2b,0x2c,0x2d,
0x2e,0x2e,0x2f,0x30,0x31,0x32,0x33,0x34,0x35,0x35,
0x36,0x37,0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,
0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
0x4a,0x4b,0x4c,0x4d,0x4e,0x4f,0x50,0x51,0x52,0x53,
0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x5b,0x5c,0x5d,
0x5f,0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,
0x69,0x6a,0x6c,0x6d,0x6e,0x6f,0x70,0x71,0x72,0x73,
0x74,0x75,0x77,0x78,0x79,0x7a,0x7b,0x7c,0x7d,0x7e,
0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x8a,
0x8b,0x8c,0x8d,0x8e,0x8f,0x90,0x91,0x92,0x93,0x95,
0x96,0x97,0x98,0x99,0x9a,0x9b,0x9c,0x9d,0x9e,0x9f,
0xa0,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,
0xab,0xac,0xad,0xae,0xaf,0xb0,0xb1,0xb2,0xb3,0xb4,
0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xbb,0xbc,0xbd,0xbe,
0xbf,0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,
0xc9,0xca,0xca,0xcb,0xcc,0xcd,0xce,0xcf,0xd0,0xd1,
0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd6,0xd7,0xd8,0xd9,
0xda,0xda,0xdb,0xdc,0xdd,0xde,0xde,0xdf,0xe0,0xe0,
0xe1,0xe2,0xe3,0xe3,0xe4,0xe5,0xe5,0xe6,0xe7,0xe7,
0xe8,0xe9,0xe9,0xea,0xea,0xeb,0xec,0xec,0xed,0xed,
0xee,0xee,0xef,0xf0,0xf0,0xf1,0xf1,0xf2,0xf2,0xf3,
0xf3,0xf4,0xf4,0xf4,0xf5,0xf5,0xf6,0xf6,0xf7,0xf7,
0xf7,0xf8,0xf8,0xf8,0xf9,0xf9,0xf9,0xfa,0xfa,0xfa,
0xfb,0xfb,0xfb,0xfb,0xfc,0xfc,0xfc,0xfc,0xfd,0xfd,
0xfd,0xfd,0xfd,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
0xff,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfd,0xfd,
0xfd,0xfd,0xfd,0xfc,0xfc,0xfc,0xfc,0xfb,0xfb,0xfb,
0xfb,0xfa,0xfa,0xfa,0xf9,0xf9,0xf9,0xf8,0xf8,0xf8,
0xf7,0xf7,0xf7,0xf6,0xf6,0xf5,0xf5,0xf4,0xf4,0xf4,
0xf3,0xf3,0xf2,0xf2,0xf1,0xf1,0xf0,0xf0,0xef,0xee,
0xee,0xed,0xed,0xec,0xec,0xeb,0xea,0xea,0xe9,0xe9,
0xe8,0xe7,0xe7,0xe6,0xe5,0xe5,0xe4,0xe3,0xe3,0xe2,
0xe1,0xe0,0xe0,0xdf,0xde,0xde,0xdd,0xdc,0xdb,0xda,
0xda,0xd9,0xd8,0xd7,0xd6,0xd6,0xd5,0xd4,0xd3,0xd2,
0xd1,0xd1,0xd0,0xcf,0xce,0xcd,0xcc,0xcb,0xca,0xca,
0xc9,0xc8,0xc7,0xc6,0xc5,0xc4,0xc3,0xc2,0xc1,0xc0,
0xbf,0xbe,0xbd,0xbc,0xbb,0xba,0xb9,0xb8,0xb7,0xb6,
0xb5,0xb4,0xb3,0xb2,0xb1,0xb0,0xaf,0xae,0xad,0xac,
0xab,0xaa,0xa9,0xa8,0xa7,0xa6,0xa5,0xa4,0xa3,0xa2,
0xa0,0x9f,0x9e,0x9d,0x9c,0x9b,0x9a,0x99,0x98,0x97,
0x96,0x95,0x93,0x92,0x91,0x90,0x8f,0x8e,0x8d,0x8c,
0x8b,0x8a,0x88,0x87,0x86,0x85,0x84,0x83,0x82,0x81,
0x80,0x7e,0x7d,0x7c,0x7b,0x7a,0x79,0x78,0x77,0x75,
0x74,0x73,0x72,0x71,0x70,0x6f,0x6e,0x6d,0x6c,0x6a,
0x69,0x68,0x67,0x66,0x65,0x64,0x63,0x62,0x61,0x60,
0x5f,0x5d,0x5c,0x5b,0x5a,0x59,0x58,0x57,0x56,0x55,
0x54,0x53,0x52,0x51,0x50,0x4f,0x4e,0x4d,0x4c,0x4b,
0x4a,0x49,0x48,0x47,0x46,0x45,0x44,0x43,0x42,0x41,
0x40,0x3f,0x3e,0x3d,0x3c,0x3b,0x3a,0x39,0x38,0x37,
0x36,0x35,0x35,0x34,0x33,0x32,0x31,0x30,0x2f,0x2e,
0x2e,0x2d,0x2c,0x2b,0x2a,0x29,0x29,0x28,0x27,0x26,
0x25,0x25,0x24,0x23,0x22,0x21,0x21,0x20,0x1f,0x1f,
0x1e,0x1d,0x1c,0x1c,0x1b,0x1a,0x1a,0x19,0x18,0x18,
0x17,0x16,0x16,0x15,0x15,0x14,0x13,0x13,0x12,0x12,
0x11,0x11,0x10,0xf,0xf,0xe,0xe,0xd,0xd,0xc,
0xc,0xb,0xb,0xb,0xa,0xa,0x9,0x9,0x8,0x8,
0x8,0x7,0x7,0x7,0x6,0x6,0x6,0x5,0x5,0x5,
0x4,0x4,0x4,0x4,0x3,0x3,0x3,0x3,0x2,0x2,
0x2,0x2,0x2,0x1,0x1,0x1,0x1,0x1,0x1,0x1,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
0x0,0x1,0x1,0x1,0x1,0x1,0x1,0x1,0x2,0x2,
0x2,0x2,0x2,0x3,0x3,0x3,0x3,0x4,0x4,0x4,
0x4,0x5,0x5,0x5,0x6,0x6,0x6,0x7,0x7,0x7,
0x8,0x8,0x8,0x9,0x9,0xa,0xa,0xb,0xb,0xb,
0xc,0xc,0xd,0xd,0xe,0xe,0xf,0xf,0x10,0x11,
0x11,0x12,0x12,0x13,0x13,0x14,0x15,0x15,0x16,0x16,
0x17,0x18,0x18,0x19,0x1a,0x1a,0x1b,0x1c,0x1c,0x1d,
0x1e,0x1f,0x1f,0x20,0x21,0x21,0x22,0x23,0x24,0x25,
0x25,0x26,0x27,0x28,0x29,0x29,0x2a,0x2b,0x2c,0x2d,
0x2e,0x2e,0x2f,0x30,0x31,0x32,0x33,0x34,0x35,0x35,
0x36,0x37,0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,
0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,
0x4a,0x4b,0x4c,0x4d,0x4e,0x4f,0x50,0x51,0x52,0x53,
0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x5b,0x5c,0x5d,
0x5f,0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,
0x69,0x6a,0x6c,0x6d,0x6e,0x6f,0x70,0x71,0x72,0x73,
0x74,0x75,0x77,0x78,0x79,0x7a,0x7b,0x7c,0x7d,0x7e
};

/* sineTable_1 for Vout1 */
CYCODE uint8 sineTable_1[TABLE_LENGTH] = { };

/* sineTable_2 for Vout2 */
CYCODE uint8 sineTable_2[TABLE_LENGTH] = { };	

/* Variable declarations for DMA */
/* The DMA Channel */
uint8 DMA_1_Chan;
uint8 DMA_2_Chan;

/* The DMA Task Description */
uint8 DMA_1_TD[1];
uint8 DMA_2_TD[1];

/*******************************************************************************
* Function Name: startSoundComponents
********************************************************************************
* Summary:
*    starts required peripherals.
*******************************************************************************/
void startSoundComponents(void)
{
    /* Start Components */
    VDAC8_1_Start();
    VDAC8_2_Start();
    Opamp_1_Start();
    Opamp_2_Start();
    DDS24_1_Start();
}

/*******************************************************************************
* Function Name: dmaConfiguration
********************************************************************************
* Summary:
*    initializes DMA configurations and setup.
*******************************************************************************/
void dmaConfiguration(void)
{
    /* DMA_1 configuration */
    #define DMA_1_SRC_BASE (&sineTable_1[0])
    #define DMA_1_DST_BASE (CYDEV_PERIPH_BASE)
    
    /* Initialize the DMA_1 channel */
    DMA_1_Chan = DMA_1_DmaInitialize(DMA_BYTES_PER_BURST, DMA_REQUEST_PER_BURST, HI16(DMA_1_SRC_BASE), HI16(DMA_1_DST_BASE));
    
    /* Allocate and Configure TD */
    DMA_1_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(DMA_1_TD[0], TABLE_LENGTH, DMA_1_TD[0], TD_INC_SRC_ADR);
    CyDmaTdSetAddress(DMA_1_TD[0], LO16((uint32)sineTable_1), LO16((uint32)VDAC8_1_Data_PTR));
    
    /*Map the TD to the DMA Channel */
    CyDmaChSetInitialTd(DMA_1_Chan, DMA_1_TD[0]);
    
    /* Enable DMA_1 channel */
    CyDmaChEnable(DMA_1_Chan, 1);
    
    /* DMA_2 configuration */
    #define DMA_2_SRC_BASE (&sineTable_2[0])
    #define DMA_2_DST_BASE (CYDEV_PERIPH_BASE)
    
    /* Initialize the DMA_2 channel */
    DMA_2_Chan = DMA_2_DmaInitialize(DMA_BYTES_PER_BURST, DMA_REQUEST_PER_BURST, HI16(DMA_2_SRC_BASE), HI16(DMA_2_DST_BASE));
    
    /* Allocate and Configure TD */
    DMA_2_TD[0] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(DMA_2_TD[0], TABLE_LENGTH, DMA_2_TD[0], TD_INC_SRC_ADR);
    CyDmaTdSetAddress(DMA_2_TD[0], LO16((uint32)sineTable_2), LO16((uint32)VDAC8_2_Data_PTR));
    
    /*Map the TD to the DMA Channel */
    CyDmaChSetInitialTd(DMA_2_Chan, DMA_2_TD[0]);
    
    /* Enable DMA_1 channel */
    CyDmaChEnable(DMA_2_Chan, 1);
}

/*******************************************************************************
* Function Name: sineWaveInitialize
********************************************************************************
* Summary:
*    initializes two seperate sine tables for each channel (left/right ear)
*******************************************************************************/
void sineWaveInitialize(int freq)
{
    /* Set DDS24_1 parameters */
    DDS24_1_SetFrequency(freq*TABLE_LENGTH);
    
    for (int i = 0; i < 90; i++)  // populate sineWave
    {
        sineTable_1[i] = sineTable[i]  ;    // sine   
        sineTable_2[i] = sineTable[i]  ;    // sine+phase
    }
}

/*******************************************************************************
* Function Name: updateSineWave
********************************************************************************
* Summary:
*    phase of the sine wave is controlled by the input offset cycles.
*    the amplitude is controlled by multiplying the sine table with input.
*    function performs for each waveform (left/right) using input 'waveNum'. 
*******************************************************************************/
int updateSineWave(uint8 phase, double att,int waveNum)
{
    if (waveNum == 1) 
    {
        for (int i = 0; i < TABLE_LENGTH; i++) // populate sineWave
        {
            sineTable_1[i] = sineTable[i+phase] * att ; // sine  + phase
        }   
        return 1;
    }
    if (waveNum == 2) 
    {
        for (int i = 0; i < TABLE_LENGTH; i++)  // populate sineWave
        {
            sineTable_2[i] = sineTable[i+phase] *att ; // sine  + phase
        } 
        return 1;
    }
    else return 0;
}

/* [] END OF FILE */
