/* ============================================================================
 * File Name: `$INSTANCE_NAME`.h
 * Version `$CY_MAJOR_VERSION`.`$CY_MINOR_VERSION`
 *
 * Description:
 *   DDS24: 24-bit DDS frequency generator component (v 0.0)
 *   Produces two syncronous fractional frequency outputs (out1 and out2)
 *   Outputs out1 and out2 separated by variable 8-bit phase shift [0..255]. 
 *
 * Credits:
 *   based on original IQ_DDS code by <PSoC73>:
 *   http://www.cypress.com/?app=forum&id=2492&rID=88149
 * 
 *   Special thanks to <pavloven>, <vdvorak>, <kabron>
 *
 * Note:
 *   frequency resolution = clock_frequency / 2^24
 *   frequency maximum    = clock_frequency / 2
 *   frequency minimum    = clock_frequency / 2^24
 *
 * ============================================================================
 * PROVIDED AS-IS, NO WARRANTY OF ANY KIND, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * FREE TO SHARE, USE AND MODIFY UNDER TERMS: CREATIVE COMMONS - SHARE ALIKE
 * ============================================================================
*/


#ifndef `$INSTANCE_NAME`_H
#define `$INSTANCE_NAME`_H

#include <project.h>




#define true  1 // but not True
#define false 0 // but not False
    
    
/***************************************
*   Conditional Compilation Parameters
***************************************/

#define `$INSTANCE_NAME`_CLOCK_FREQ (double) `=$ClockFreq                 `   // input clock frequency
#define `$INSTANCE_NAME`_HARDWARE_LOAD       `=$HardwareLoad              `   // load frequency and phase on hardware digital signal 
#define `$INSTANCE_NAME`_OUT2_ENABLED        `=$out2_enable               `   // show out2 terminal
#define `$INSTANCE_NAME`_CR_ENABLE           `=($EnableMode == CR_only || $EnableMode == HW_and_CR)`  // enable mode = SW or SW_&_HW
#define `$INSTANCE_NAME`_CONTROL_FREQ_API    `=($ControlFreq  == Ctrl_API)`   // use API to set DDS frequency
#define `$INSTANCE_NAME`_CONTROL_PHASE_API   `=($ControlPhase == Ctrl_API)`   // use API to set out2 phase
 
    
/***************************************
*           API Constants
***************************************/   

#define `$INSTANCE_NAME`_PresetFreq          `=$Frequency `  // preset startup frequency
#define `$INSTANCE_NAME`_PresetPhase         `=$out2_phase`  // preset startup phase


    
/***************************************
*        Function Prototypes
***************************************/

void   `$INSTANCE_NAME`_Init();                      // initialize DDS
void   `$INSTANCE_NAME`_Start();                     // Init and Enable DDS
void   `$INSTANCE_NAME`_Stop();                      // Stop (disable) DDS 
void   `$INSTANCE_NAME`_Enable();                    // Enable DDS (must init first) // todo: add _Enabled and _Initialized property
uint8  `$INSTANCE_NAME`_SetFrequency( double Freq ); // Set desired output frequency
void   `$INSTANCE_NAME`_SetPhase( uint8 Phase );     // set out2 phase
double `$INSTANCE_NAME`_GetOutFreq();                // read actual output frequency
double `$INSTANCE_NAME`_GetFrequency();              // return current set frequency 
uint8  `$INSTANCE_NAME`_GetPhase();                  // read phase 


#endif


/* [] END OF FILE */
