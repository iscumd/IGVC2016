#ifndef __MACROS_C__
#define __MACROS_C__
#include <math.h>

#include "macros.h"

double adjust_angle(double angle, double circle)
{                               //circle = 2pi for radians, 360 for degrees
    // Subtract multiples of circle
    angle -= floor(angle / circle) * circle;
    angle -= floor(2 * angle / circle) * circle;

    return angle;
}

#endif //__MACROS_C__
