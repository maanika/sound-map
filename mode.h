/*******************************************************************************
* Written by Maanika Kenneth Koththigoda, for PSoC5LP
* Last Modified on 21/02/2021
*
* File:     mode.h
* Version:  1.0.0
*
* Brief: Activate/Deactivate different running modes of the program.
*
* Target device:
*    CY8C5888LTI - LP097
*
* Code Tested With:
*    - Silicon: PSoC 5LP
*    - IDE: PSoC Creator 4.3
*    - Compiler: GCC 5.4
*
*******************************************************************************/
#ifndef MODE_H
	#define MODE_H
/*******************************************************************************
*   Included Headers
*******************************************************************************/
    #include <project.h>

/*******************************************************************************
*   Macros and #define Constants
*******************************************************************************/
    // Run Debgging mode (allow print statements).
    // This should be truned off when device is under actual use, since print
    // statements slows the system down.(only set to 1 when debugging/testing).
    #define DEBUG_PRINT_MODE 1
    
    // Run Obstacle Detection System.
    #define OBJ_DETECT_MODE  1

#endif

/* [] END OF FILE */
