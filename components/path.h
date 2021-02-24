/*******************************************************************************
* Written by Maanika Kenneth Koththioda, for PSoC5LP
* Last Modified on 24/02/2021
*
* File: path.h
* Version: 1.0.0
*
* Brief: Initializes path variables for selected destination.
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
#ifndef TEMPLATE_MODULE_H
	#define TEMPLATE_MODULE_H
/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include "project.h"
#include "FreeRTOS.h"
    
/* Path planning variables */
struct Path{
    const double checkpointLat[15];
    const double checkpointLon[15];
    int checkpointCurrent;              // {0,1,2,3,4,5,6,7,8,9,10,11,12,13}
    int checkpointDest;                 // 0,3, 8
    int checkpointOperation;            // 0 - addition, 1- substraction.
    BaseType_t checkpointDestSelected;  // True if user has selected a valid destination
    char checkpointDestName;
    BaseType_t atDestination;           // True if user is within the proximity of the selected destination
};

#endif
/* [] END OF FILE */
