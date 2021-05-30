/*******************************************************************************
* Written by Maanika Kenneth Koththioda, for PSoC5LP
* Last Modified on 24/02/2021
*
* File: path.c
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
********************************************************************************
*   Included Headers
*******************************************************************************/
#include "project.h"
#include "stdio.h"
#include "path.h"
#include "gps.h"
#include "mode.h"

/*******************************************************************************
*   Variables
*******************************************************************************/
/* Checkpoint names - for debugging/testing */
#if DEBUG_PRINT_MODE == 1
    extern int checkpointName[15];
    char tempStr[100];
#endif

/*******************************************************************************
* Function Name:
********************************************************************************
* Summary:
*   Checks distance from current location to starting locations.
*   Closest distance will be set as the starting location.
*   Set operation (+/-) using starting location and destination.
*******************************************************************************/
void pathStart ( struct Path *path, double latitudeInDec, double longitudeInDec )
{
    double diffDistanceToStart[4];/* Starting points H0,H3,H8 */
    int startArrayLocation;
   
    /* Distance from current location to each starting point */
    diffDistanceToStart[0] = distance( latitudeInDec, longitudeInDec, path->checkpointLat[0], path->checkpointLon[0] );
    diffDistanceToStart[1] = distance( latitudeInDec, longitudeInDec, path->checkpointLat[3], path->checkpointLon[3] );
    diffDistanceToStart[2] = distance( latitudeInDec, longitudeInDec, path->checkpointLat[8], path->checkpointLon[8] );
    
    if( (diffDistanceToStart[0] < diffDistanceToStart[1]) && (diffDistanceToStart[0] < diffDistanceToStart[2]) )
    {
       startArrayLocation = 0;
    }
    else if( diffDistanceToStart[1] < diffDistanceToStart[2] )
    {
       startArrayLocation = 1;
    }
    else
    {
       startArrayLocation = 2;
    }

    /* Determine starting location, chechpoint opertion and destination (for now)*/
    switch ( startArrayLocation )
    {
        case 0: /* H0 is the starting point*/
            path->checkpointCurrent = 0;
            switch (path->checkpointDestName )
            {
                case 'C':
                    path->checkpointDest = 0;
                    path->atDestination = pdTRUE; /* at destnation */
                    break;
                case 'H':
                    path->checkpointDest = 3;
                    path->checkpointOperation = 0; /* addition */
                    break;
                case 'L':
                    path->checkpointDest = 8;
                    path->checkpointOperation = 1; /* substraction */
                    break;
                default:
                    /* error */
                    break;
            }
            #if DEBUG_PRINT_MODE == 1
                sprintf( tempStr, "Starting Point: H%d \nDestination is: H%d\n", 
                    checkpointName[path->checkpointCurrent], checkpointName[path->checkpointDest] );
                UART_PutString( tempStr );
            #endif
            break;
        case 1: /* H3 is the starting point */
            path->checkpointCurrent = 3;
            switch (path->checkpointDestName )
            {
                case 'C':
                    path->checkpointDest = 0;
                    path->checkpointOperation = 1; /* substraction */
                    break;
                case 'H':
                    path->checkpointDest = 3;
                    path->atDestination = pdTRUE; /* at destnation */
                    break;
                case 'L':
                    path->checkpointDest = 8;
                    path->checkpointOperation = 0; /* addition */
                    break;
                default:
                    /* error */
                    break;
            }
            #if DEBUG_PRINT_MODE == 1
                sprintf( tempStr, "Starting Point: H%d \nDestination is: H%d\n", 
                    checkpointName[path->checkpointCurrent], checkpointName[path->checkpointDest] );
                UART_PutString( tempStr );
            #endif
            break;
        case 2: /* H8 is the starting point */
            path->checkpointCurrent = 8;
            switch ( path->checkpointDestName )
            {
                case 'C':
                    path->checkpointDest = 0;
                    path->checkpointOperation = 0; /* addition */
                    break;
                case 'H':
                    path->checkpointDest = 3;
                    path->checkpointOperation = 1; /* substraction */
                    break;
                case 'L':
                    path->checkpointDest = 8;
                    path->atDestination = pdTRUE; /* at destnation */
                    break;
                default:
                    /* error */
                    break;
            }
            #if DEBUG_PRINT_MODE == 1
                sprintf( tempStr, "Starting Point: H%d \nDestination is: H%d\n", 
                    checkpointName[path->checkpointCurrent], checkpointName[path->checkpointDest] );
                UART_PutString( tempStr );
            #endif
            break;
        default:
            #if DEBUG_PRINT_MODE == 1
                sprintf( tempStr, "ERROR in destination name\n");
                UART_PutString( tempStr );
            #endif
            break;
    }
}
/* [] END OF FILE */
