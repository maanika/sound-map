/*******************************************************************************
* Written by Maanika Kenneth Koththioda, for PSoC5LP
* Last Modified on 3/1/2021
*
* File: custom_synth.h
* Version: 1.0.0
*
* Brief: Project dependent LPC audio constants and functions.
*
* Target device:
*    CY8C5888LTI - LP097
*
* Code Tested With:
*    - Silicon: PSoC 5LP
*    - IDE: PSoC Creator 4.3
*    - Compiler: GCC 5.4
*
* Creating LPC encoded sound samples:
* Download python_wizard from https://github.com/ptwz/python_wizard.
* This program is used to convert audio streams with a .WAV extension into LPC 
* bit streams.
*
* Using python_wizard:
* 1. Download python_wizard and install prerequisites.
* 2. Ensure python_wizard and python_wizard_gui file has a .py extension to be 
* identifed by python editors.
* 3. Place EXAMPLE.wav file in the same directory as python_wizard.py.
* 4. Open the command line, cd to the project path and enter the command 
* python python_wizard.py -f arduino EXAMPLE.wav > EXAMPLE.h
* 5. Open EXAMPLE.h to obtain reuquired LPC code.
*
* Notes:
*   Optimized LPC codes stored in the ROM of Speech Synthesisers can be found here
* https://github.com/going-digital/Talkie/tree/master/Talkie/examples
*
*******************************************************************************/
#ifndef CUSTOM_SYNTH_H
	#define CUSTOM_SYNTH_H
    
/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include "project.h"
    
/*******************************************************************************
*   Function Declarations
*******************************************************************************/

// Brief: Vocalize given number parameter.
// Param:  number Number to vocalize.
// Return: none 
void sayHargraveLibrary();

// Brief: Vocalize given number parameter.
// Param:  number Number to vocalize.
// Return: none 
void sayCampbellHall();

// Brief: Vocalize given number parameter.
// Param:  number Number to vocalize.
// Return: none 
void sayCampusCentre();

// Brief: Vocalize given number parameter.
// Param:  number Number to vocalize.
// Return: none 
void sayArrived();

// Brief: Vocalize given number parameter.
// Param:  number Number to vocalize.
// Return: none 
void sayFix();

// Brief: Vocalize given number parameter.
// Param:  number Number to vocalize.
// Return: none 
void sayBatteryPercent(int number);

// Brief: Vocalize weolcome message.
// Param: none.
// Return: none 
void sayWelocome();

// Brief: Vocalize a pause.
// Param: none.
// Return: none 
void sayPause();

#endif

/* [] END OF FILE */
