/*******************************************************************************
* Written by Maanika Kenneth Koththioda, for PSoC5LP
* Last Modified on 30/05/2021
*
* File: gps.c
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
*******************************************************************************
*   Included Headers
*******************************************************************************/
#include "project.h"
#include "math.h"

/*******************************************************************************
* Function Name: toRadians
********************************************************************************
* Summary:
*   This function converts decimal degrees to radians
*******************************************************************************/
static long double toRadians(double deg) {
  return (deg * M_PI / 180);
}
/*******************************************************************************
* Function Name: toDegrees
********************************************************************************
* Summary:
*   This function converts decimal radians to degrees
*******************************************************************************/
static long double toDegrees(double rad) {
  return (rad * 180 / M_PI);
}

/*******************************************************************************
* Function Name: min2dec
********************************************************************************
* Summary:
*    function to convert minutes to degrees
*******************************************************************************/
long double min2dec(double inmin){
    int degrees;
    double minutes;
    degrees = (inmin / 100.0);
    minutes = 100 * (inmin / 100.0 - 1.0*degrees);
    return 1.0*degrees + minutes/60.0;
}
  
/*******************************************************************************
* Function Name: distance
********************************************************************************
* Summary:
*    This functions obtains distance between two corrdinates
*******************************************************************************/
long double distance(long double lat1, long double long1, long double lat2, long double long2) 
{ 
    // Convert the latitudes  
    // and longitudes 
    // from degree to radians. 
    lat1 = toRadians(lat1); 
    long1 = toRadians(long1); 
    lat2 = toRadians(lat2); 
    long2 = toRadians(long2); 
      
    // Haversine Formula 
    long double dlong = long2 - long1; 
    long double dlat = lat2 - lat1;
    long double answer = pow(sin(dlat / 2), 2) +  
                          cos(lat1) * cos(lat2) *  
                          pow(sin(dlong / 2), 2); 
                        
    answer = 2 * asin(sqrt(answer));
  
    // Radius of Earth in  
    // Kilometers, R = 6371 
    // Use R = 3956 for miles 
    long double R = 6371; 

    // Calculate the result 
    answer = answer * R *1000; 

    return answer;
} 

/*******************************************************************************
* Function Name: GPSbearing
********************************************************************************
* Summary:
*    This function obtains the angle between two coordinates
*******************************************************************************/
double GPSbearing(double lat,double lon,double lat2,double lon2){

    double teta1 = toRadians(lat);
    double teta2 = toRadians(lat2);
    double delta1 = toRadians(lat2-lat);
    double delta2 = toRadians(lon2-lon);

    double y = sin(delta2) * cos(teta2);
    double x = cos(teta1)*sin(teta2) - sin(teta1)*cos(teta2)*cos(delta2);
    double bearing = atan2(y,x);
    bearing = toDegrees(bearing);// radians to degrees
    bearing = ( ((int)bearing + 360) % 360 ); 

    return bearing;
}

/* [] END OF FILE */
