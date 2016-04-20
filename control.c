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
    lastTime = clock()/((double) CLOCKS_PER_SEC);//storing what time PID control is started.
    cvar.pErr=cvar.iErr=cvar.dErr=0;//initializing all errors back to zero
}

int guide(XY robot, XY currentTarget, XY lastTarget, XY nextTarget,
         double heading,  double rWidth, int dir)//returns 1 if target reached, otherwise returns 0
{
    double dx, dy, s, c, nx, ny, dt, temp;
    double dlastx, dlasty, lastDist;
    double desiredAngle;

    int reachedTarget;

    if (dir < 0) {//change directions if your is facing the wrong direction
        heading = heading - M_PI * SIGN(heading);
    }

    dx = currentTarget.x - robot.x;// dx is distance between current target and x robot
    dy = currentTarget.y - robot.y;//dy is distance between current target and robot y position

    dlastx = lastTarget.x - robot.x;// dlastx is distance between current robot location and last target y locations
    dlasty = lastTarget.y - robot.y;// dlasty is distance between current robot location and last target y locations
    lastDist = sqrt(dlastx*dlastx + dlasty*dlasty);//solving for absolute distance from last target

    c = cos(heading);
    s = sin(heading);

    cvar.targdist = sqrt(dx*dx + dy*dy);// current absolute distance from target
    cvar.right = dx*c - dy*s; // How far target is to the right of robot
    cvar.front = dy*c + dx*s;//How far target is in front of the robot
    desiredAngle = ADJUST_RADIANS(atan2(dx,dy));//calculation of desired heading from current heading

//    DEBUG(heading); DEBUG(desiredAngle); DEBUGN(nextAngle);

/*PID Calculations cvar3 Heading*/

    nx = -(currentTarget.y-lastTarget.y); // -Delta Y
    ny =  (currentTarget.x-lastTarget.x); //  Delta X
    temp = hypot(nx, ny)+1e-3; // To avoid division by zero
    nx /= temp; //equivalent to nx = nx / temp;
    ny /= temp;//equivalent to ny = ny / temp;


    thisTime = clock() /((double) CLOCKS_PER_SEC);// storing the current time
    dt = thisTime-lastTime;// finding difference between last iteration of guide, and this iteration of guide.

    /*Current Target Heading PID Calculations*/
    cvar.lastpErr = cvar.pErr;//storing current pErr value to be previous value.
    cvar.pErr = ADJUST_RADIANS(heading - desiredAngle);//pError=e(t)
    cvar.iErr = cvar.iErr + cvar.pErr*dt;//iErr=summing all previous iErr's and current*time change
    cvar.iErr = SIGN(cvar.iErr)*MIN(fabs(cvar.iErr), maxIntErr);// limiting the maximum value of iErr, using MaxIntErr (which is .5)
    if(dt!=0) cvar.dErr = (cvar.pErr-cvar.lastpErr)/dt;// if this is not the first time the PID calculation is occuring, then find dErr
    cvar.turn = -(cvar.kP*cvar.pErr + cvar.kI*cvar.iErr + cvar.kD*cvar.dErr);//solving for

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
