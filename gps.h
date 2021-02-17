// include guard (see e.g. https://en.wikipedia.org/wiki/Include_guard)
#ifndef GPS_H
#define GPS_H
    
#include "project.h"
    
    
long double distance(long double lat1, long double long1, long double lat2, long double long2) ;
double GPSbearing(double lat,double lon,double lat2,double lon2);
long double min2dec(double inmin);
    
#endif