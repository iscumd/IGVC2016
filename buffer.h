#ifndef __BUFFER_H
#define __BUFFER_H
// The next two structures are in bufferCombined.h


typedef struct{
    double x;
    double y;
} Point;

typedef struct{
    double left;
    double right;
} Angle_Cab;


double bufferTargDist;
Angle_Cab buffer_angles;
void getBufferAngles(double target_angle);
void debugBufferAngles();

extern double combinedTargDist;
Angle_Cab combinedBufferAngles;
void getCombinedBufferAngles(double target_angle, int useVision);
void debugCombinedBufferAngles();
#endif
