#ifndef __CONTROL_C__
#define __CONTROL_C__

#include <math.h>
#include <time.h>
#include <stdlib.h>

#include "serial.h"
#include "macros.h"
#include "control.h"

double lastTime, thisTime;
double maxIntErr = 0.5;



/*

deltaN = (targetLat-robotLat)*Rearth;
deltaE = (targetLon-robotLon)*Rearth*cos(lat);



*/


void initGuide(){
    lastTime = clock()/((double) CLOCKS_PER_SEC);
    cvar.pErr=cvar.iErr=cvar.dErr=0;
}

int guide(XY robot, XY currentTarget, XY lastTarget, XY nextTarget, double heading,  double rWidth, int dir)
{
    double dx, dy, s, c, nx, ny, dt, temp;
    double dlastx, dlasty, lastDist;
    double desiredAngle;

    int reachedTarget;

    if (dir < 0) {
        heading = heading - M_PI * SIGN(heading);
    }

    dx = currentTarget.x - robot.x;
    dy = currentTarget.y - robot.y;

    dlastx = lastTarget.x - robot.x;
    dlasty = lastTarget.y - robot.y;
    lastDist = sqrt(dlastx*dlastx + dlasty*dlasty);

    c = cos(heading);
    s = sin(heading);

    cvar.targdist = sqrt(dx*dx + dy*dy);
    cvar.right = dx*c - dy*s;
    cvar.front = dy*c + dx*s;
    desiredAngle = ADJUST_RADIANS(atan2(dx,dy));

//    DEBUG(heading); DEBUG(desiredAngle); DEBUGN(nextAngle);

/*PID Calculations cvar3 Heading*/

    nx = -(currentTarget.y-lastTarget.y); // -Delta Y
    ny =  (currentTarget.x-lastTarget.x); //  Delta X
    temp = hypot(nx, ny)+1e-3; // To avoid division by zero
    nx /= temp;
    ny /= temp;

    thisTime = clock() /((double) CLOCKS_PER_SEC);
    dt = thisTime-lastTime;

    /*Current Target Heading PID Calculations*/
    cvar.lastpErr = cvar.pErr;
    cvar.pErr = ADJUST_RADIANS(heading - desiredAngle);
    cvar.iErr = cvar.iErr + cvar.pErr*dt;
    cvar.iErr = SIGN(cvar.iErr)*MIN(fabs(cvar.iErr), maxIntErr);
    if(dt!=0) cvar.dErr = (cvar.pErr-cvar.lastpErr)/dt;
    cvar.turn = -(cvar.kP*cvar.pErr + cvar.kI*cvar.iErr + cvar.kD*cvar.dErr);

    lastTime = thisTime;

    inLastTarget = (lastDist<leaveTargetThresh);
    approachingTarget = (cvar.targdist < approachingThresh);
    reachedTarget = (cvar.targdist < destinationThresh);

    return reachedTarget;
}

void debugGuide(){
        fprintf(stderr, "pE3: %.4f\t iE3: %.4f\t dE3: %.4f\t t3: %.4f\n", cvar.pErr, cvar.iErr, cvar.dErr, cvar.turn);
}

#endif //__CONTROL_C__
