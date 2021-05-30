/*******************************************************************************
* Written by Maanika Kenneth Koththioda, for PSoC5LP
* Last Modified on 30/05/2021
*
* File: gps.h
* Version: 1.0.0
*
* Brief: GPS Functions.
*
* Target device:
*    CY8C5888LTI - LP097
*
* Code Tested With:
*    - Silicon: PSoC 5LP
*    - IDE: PSoC Creator 4.3
*    - Compiler: GCC 5.4
*
* Notes: Equations were obtained from
*        https://www.movable-type.co.uk/scripts/latlong.html
*******************************************************************************/
#ifndef GPS_H
#define GPS_H
    
/*******************************************************************************
*   Included Headers
*******************************************************************************/
#include "project.h"

/*******************************************************************************
*   Function Declarations
*******************************************************************************/
    
// Brief: obtains distance between two corrdinates
// Param:  two coordinates in degrees.
// Return: distnace in meters.
long double distance(long double lat1, long double long1, long double lat2, long double long2) ;

// Brief: obtains bearing between two corrdinates
// Param:  two coordinates in degrees.
// Return: angle in degrees.
double GPSbearing(double lat,double lon,double lat2,double lon2);

// Brief: convert minutes to degrees.
// Param:  minuites.
// Return: degrees.
long double min2dec(double inmin);
    
#endif

/* [] END OF FILE */
