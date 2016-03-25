#ifndef __GPS_H__
#define __GPS_H__

/** Getting X,Y from point a few miles from lat0 and lon0 */
#define K_NS 111120.00
//#define lat0 42.320137
#define lat0 42.67813235
#define K_EW (K_NS)*cos(DEG2RAD(lat0))

#ifndef GPSX
#define GPSX(a, b) (K_EW) * ((a) - (b))
#endif

#ifndef GPSY
#define GPSY(a, b)  (K_NS) * ((a)  - (b))
#endif

typedef struct {
    double latitude;
    double longitude;
    double speed;
    double course;
    double time;
    double confidence;
} GPSVAR;
extern GPSVAR gpsvar;
extern GPSVAR predicted, filtered;

typedef struct {
    double lat;
    double lon;
} GPSPNT;

extern double startLongitude, startLatitude;

void initGPS();
void initParser();
void parseChar(char c);
int gpsUpdated();
void readGPS();
void debugGPS();
#endif

