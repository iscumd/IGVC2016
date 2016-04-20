#ifndef __BUFFER_H
#define __BUFFER_H
// The next two structures are in bufferCombined.h


typedef struct{
    double x;
    double y;
} Point;

typedef struct{
    double left;// left turn angle
    double right;//right turn angle
} Angle_Cab;


double bufferTargDist;// the halo distance, note if you are closer to GPS location then ignore the obstacle.
Angle_Cab buffer_angles;
void getBufferAngles(double target_angle);
void debugBufferAngles();

extern double combinedTargDist;
Angle_Cab combinedBufferAngles;
void getCombinedBufferAngles(double target_angle, int useVision);// find best angle
void debugCombinedBufferAngles();//debug
#endif
