#ifndef __BUFFER_COMBINED_H__
#define __BUFFER_COMBINED_H__
// ALL OF THESE ARE IN BUFER.H
#if 0
typedef struct{
    double x;
    double y;
} Point;

typedef struct{
    double left;
    double right;
} Angle_Cab;

extern double combinedTargDist;
Angle_Cab combinedBufferAngles;
void getCombinedBufferAngles(double target_angle, int useVision);
void debugCombinedBufferAngles();
#else
#include "buffer.h"
#endif
#endif
