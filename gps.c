#include "project.h"
#include "math.h"


// This function converts decimal degrees to radians
static long double toRadians(double deg) {
  return (deg * M_PI / 180);
}
// This function converts decimal radians to degrees
static long double toDegrees(double rad) {
  return (rad * 180 / M_PI);
}
  
  
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
  
    long double ans = pow(sin(dlat / 2), 2) +  
                          cos(lat1) * cos(lat2) *  
                          pow(sin(dlong / 2), 2); 
  
    ans = 2 * asin(sqrt(ans)); 
  
    // Radius of Earth in  
    // Kilometers, R = 6371 
    // Use R = 3956 for miles 
    long double R = 6371; 
      
    // Calculate the result 
    ans = ans * R *1000; 
  
    return ans; 
} 

double GPSbearing(double lat,double lon,double lat2,double lon2){

    double teta1 = toRadians(lat);
    double teta2 = toRadians(lat2);
    double delta1 = toRadians(lat2-lat);
    double delta2 = toRadians(lon2-lon);

    //==================Heading Formula Calculation================//

    double y = sin(delta2) * cos(teta2);
    double x = cos(teta1)*sin(teta2) - sin(teta1)*cos(teta2)*cos(delta2);
    double brng = atan2(y,x);
    brng = toDegrees(brng);// radians to degrees
    brng = ( ((int)brng + 360) % 360 ); 

    return brng;

  }


//macro to convert minutes to degrees
long double min2dec(double inmin){
    int degrees;
    double minutes;
    degrees = (inmin / 100.0);
    minutes = 100 * (inmin / 100.0 - 1.0*degrees);
    return 1.0*degrees + minutes/60.0;
}
