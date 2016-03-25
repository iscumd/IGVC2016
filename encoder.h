#ifndef __ENCODER_H__
#define __ENCODER_H__

//#define DEBUGODOMETRY

typedef struct{
    double easting;
    double northing;
    double heading;
} ODOMETRY;

extern ODOMETRY encoderVar;

extern double cpmLeft, cpmRight;
extern double robotWidth;

void initEncoder();
void initOdometry();
void debugEncoder();
void debugOdometry();
void odometry();
#endif //__ENCODER_H__
