#include "project.h"
#include "stdio.h"
#include "sound.h"


#define DMA_BYTES_PER_BURST 1
#define DMA_REQUEST_PER_BURST 1
#define TABLE_LENGTH 90

/* Sine look up table with 180 points stored in Flash */
CYCODE const uint8 sineTable[180]= {
    127,136,145,154, 162,171,179,187, 195,202,209,216, 222,228,233,238,
    242,246,249,251, 253,254,255,255, 254,253,251,249, 246,242,238,233,
    228,222,216,209, 202,195,187,179, 171,162,154,145, 136,127,118,109,
    100, 92, 83, 75,  67, 59, 52, 45,  38, 32, 26, 21,  16, 12,  8,  5,
      3,  1,  0,  0,   0,  0,  1,  3,   5,  8, 12, 16,  21, 26, 32, 38,
     45, 52, 59, 67,  75, 83, 92,100, 109,118,127,136, 145,154,162,171,
    179,187,195,202, 209,216,222,228, 233,238,242,246, 249,251,253,254,
    255,255,254,253, 251,249,246,242, 238,233,228,222, 216,209,202,195,
    187,179,171,162, 154,145,136,127, 118,109,100, 92,  83, 75, 67, 59,
     52, 45, 38, 32,  26, 21, 16, 12,   8,  5,  3,  1,   0,  0,  0,  0,
      1,  3,  5,  8,  12, 16, 21, 26,  32, 38, 45, 52,  59, 67, 75, 83,
     92,100,109,118
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

void startSoundComponents(void)
{
    /* Start Components */
    VDAC8_1_Start();
    VDAC8_2_Start();
    Opamp_1_Start();
    Opamp_2_Start();
    DDS24_1_Start();
}

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

void sineWaveInitialize(int freq)
{
    /* Set DDS24_1 parameters */
    DDS24_1_SetFrequency(freq*TABLE_LENGTH);
    
    for (int i = 0; i < 90; i++)  // populate sineWave
    {
        sineTable_1[i] = sineTable[i]  ;          // sine   
        sineTable_2[i] = sineTable[i]  ;    // sine+phase
    }
}

int updatePhase(uint8 phase, int waveNum)
{
    if (waveNum == 1) 
    {
        for (int i = 0; i < 90; i++)  // populate sineWave
        {
            sineTable_1[i] = sineTable[i+phase]  ;          // sine  + phase
        }   
        return 1;
    }
    if (waveNum == 2) 
    {
        for (int i = 0; i < 90; i++)  // populate sineWave
        {
            sineTable_2[i] = sineTable[i+phase]  ;          // sine  + phase
        } 
        return 1;
    }
    else return 0;
}

int updateAmplitude(double att, int waveNum)
{
    if (waveNum == 1) 
    {
        for (int i = 0; i < 90; i++)  // populate sineWave
        {
            sineTable_1[i] = sineTable[i] * att;          // sine  + phase
        }   
        return 1;
    }
    if (waveNum == 2) 
    {
        for (int i = 0; i < 90; i++)  // populate sineWave
        {
            sineTable_2[i] = sineTable[i] * att;          // sine
        } 
        return 1;
    }
    else return 0;
}

