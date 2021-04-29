// include guard (see e.g. https://en.wikipedia.org/wiki/Include_guard)
#ifndef SOUND_H
#define SOUND_H
    
#include "project.h"
    
    
/* Function Prototypes */
void startSoundComponents(void);
void dmaConfiguration(void);
void sineWaveInitialize(int freq);
int updateSineWave(uint8 phase, double att,int waveNum);

/* ALWAYS CALL dmaConfiguration() AND sineWaveInitialize() IN MAIN*/

#endif

