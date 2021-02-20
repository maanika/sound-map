/* ============================================================================
 * File Name: `$INSTANCE_NAME`.c
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

#include <`$INSTANCE_NAME`.h>

  

#define DDS_RESOLUTION (double) 16777216.0 // 16777216 = 2^24
#define TUNE_WORD_MAX  (uint32)  8388607u  //  8388607 = 2^23 - 1


double Tdiv;		    // time period per bit increment
uint32 tune_word;       // DDS tune word
double SetFreq;         // DDS set frequency 
uint8  PhaseShift;      // out2 phase shift 


//==============================================================================
// Initialize component
// Set input clock frequency and default values for output frequency and phase 
//==============================================================================

void `$INSTANCE_NAME`_Init()
{   
    // make record of the input clock frequency into period   
    Tdiv = (double) (DDS_RESOLUTION / `$INSTANCE_NAME`_CLOCK_FREQ);	
    
    `$INSTANCE_NAME`_SetFrequency( `$INSTANCE_NAME`_PresetFreq );
    `$INSTANCE_NAME`_SetPhase( `$INSTANCE_NAME`_PresetPhase );   
}

//==============================================================================
// Start component
// Initialize and enable DDS component
//==============================================================================

void `$INSTANCE_NAME`_Start()
{   
    `$INSTANCE_NAME`_Init();
    `$INSTANCE_NAME`_Enable();               
}

//==============================================================================
// Enable component
// Have effect only in Software_only and Software_and_Hardware modes 
//==============================================================================

void `$INSTANCE_NAME`_Enable()
{   
    #if `$INSTANCE_NAME`_CR_ENABLE
        CY_SET_REG8( `$INSTANCE_NAME`_DDSCore_sCTRLReg_ctrlreg__CONTROL_REG, 1 ); //software enable
    #endif                    
}

//==============================================================================
// Stop component
// Have effect only in Software_only and Software_and_Hardware modes 
//==============================================================================

void `$INSTANCE_NAME`_Stop()
{   
    #if `$INSTANCE_NAME`_CR_ENABLE
        CY_SET_REG8( `$INSTANCE_NAME`_DDSCore_sCTRLReg_ctrlreg__CONTROL_REG, 0 ); //DDS software stop
    #endif                    
}

//==============================================================================
// Set output frequency
//==============================================================================

uint8 `$INSTANCE_NAME`_SetFrequency( double freq )
{ 
    uint8 result = 0; // success = 1, fail = 0 
    
    
    #if `$INSTANCE_NAME`_CONTROL_FREQ_API
        
        uint32 tmp = (uint32) (freq * Tdiv + 0.5);           // calculate tune word
        if ( (tmp < 1) || (tmp > TUNE_WORD_MAX) )  return 0; // fail -> exit if outside of the valid raange 
          
        tune_word = tmp;
        
        CY_SET_REG8( `$INSTANCE_NAME`_DDSCore_sControl_ControlPhaseA__CONTROL_REG,  tune_word     &0xFF ); //
        CY_SET_REG8( `$INSTANCE_NAME`_DDSCore_sControl_ControlPhaseB__CONTROL_REG, (tune_word>> 8)&0xFF ); //
        CY_SET_REG8( `$INSTANCE_NAME`_DDSCore_sControl_ControlPhaseC__CONTROL_REG, (tune_word>>16)&0xFF ); // 
        
        SetFreq = freq; // backup value
        result = 1;     // success
        
    #endif
    
    return result;
}

//==============================================================================
// Set out1-out2 phase shift
// (any integer value will be truncated to uint8)
//==============================================================================

void `$INSTANCE_NAME`_SetPhase( uint8 Phase ) //todo: rename SetPhase?
{ 	   
    #if (`$INSTANCE_NAME`_OUT2_ENABLED && `$INSTANCE_NAME`_CONTROL_PHASE_API)
        CY_SET_REG8( `$INSTANCE_NAME`_DDSCore_sControl_ControlPhaseD__CONTROL_REG,  Phase & 0xFF ); // write phase shift
        PhaseShift = Phase; //backup 
    #endif    
}

//==============================================================================
//     Helper functions
//==============================================================================

double `$INSTANCE_NAME`_GetOutFreq() //return actual output frequency 
{	  
    double OutputFreq  = (((double) tune_word) / DDS_RESOLUTION ) * `$INSTANCE_NAME`_CLOCK_FREQ;   
    return OutputFreq;
}

double `$INSTANCE_NAME`_GetFrequency() // return current set frequency 
{	  
    return SetFreq;
}

uint8 `$INSTANCE_NAME`_GetPhase() //return current phase shift
{	
    return PhaseShift;
}

//==============================================================================
/* [] END OF FILE */
