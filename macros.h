#ifndef __MACROS_H__
#define __MACROS_H__

#ifndef __XY__
#define __XY__
typedef struct{
    double x;
    double y;
} XY;
#endif //__XY__
#ifndef __ROBOT__
#define __ROBOT__
typedef struct{
    double x;
    double y;
    double theta;
} ROBOT;
#endif //__XY__


//typedef enum { false = 0, true } bool;

double adjust_angle(double angle, double circle);

#ifndef ADJUST_DEGREES
#define ADJUST_DEGREES(angle) adjust_angle(angle, 360.0)
#endif

#ifndef ADJUST_RADIANS
#define ADJUST_RADIANS(angle) adjust_angle(angle, M_PI*2)
#endif

#ifndef M_PI
#define M_PI 3.1415926537       // Miciosoft does not have this!
#endif

#ifndef TWO_PI
#define TWO_PI (2*M_PI)
#endif

#ifndef MAX
#define MAX(a,b) ((a)>(b)?(a):(b))
#endif

#ifndef MIN
#define MIN(a,b) ((a)<(b)? (a): (b))
#endif

#ifndef MAG
#define MAG(x) ((x)>(0)? x : x*-1)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*180.0/M_PI)
#endif

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*M_PI/180.0)
#endif

#ifndef POLAR2X
#define POLAR2X(angle,mag) mag*sin(DEG2RAD((135-angle*0.25)))
#endif

#ifndef POLAR2Y
#define POLAR2Y(angle,mag) mag*cos(DEG2RAD((135-angle*0.25)))
#endif

#ifndef BIT
#define BIT(k) ((1)<<(k))
#endif

#ifndef DIM
#define DIM(a) (sizeof(a)/sizeof(a[0]))
#endif

#ifndef D
#define D(x,y) (sqrt((x)*(x)+(y)*(y)))
#endif

#ifndef SIGN
#define SIGN(x) ((x)>0? 1: -1)
#endif

#ifndef INT2DBL
#define INT2DBL(x) (((x)-32767.0)/32767.0)
#endif

#ifndef M2MM
#define M2MM 1000
#endif

#ifndef MM2M
#define MM2M 0.001
#endif

#endif //__MACROS_H__
