#ifndef __ENCODER_C__
#define __ENCODER_C__

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <time.h>

#include "macros.h"
#include "encoder.h"
#include "socket.h"

int encoderSocket;
char encoderBuffer[10000];
static int oLastLeft = 0, oLastRight = 0;//static only initialized once


void getCounts(){
    getMsg(encoderSocket,encoderBuffer);
}

void resetCount(){
    sendMsg(encoderSocket, "c");
}

void initEncoder()
{
    encoderSocket = initSocket("192.168.0.202", 5000);
    resetCount();
}

void initOdometry(){
    oLastLeft = oLastRight = 0;
    encoderVar.northing=0.0;
    encoderVar.easting=0.0;
    encoderVar.heading=0.0;
    resetCount();
}

void debugEncoder(){
    printf("%s\n", encoderBuffer);
}

void debugOdometry(){
    printf("E: %.4f N: %.4f H: %.4f\n",encoderVar.easting, encoderVar.northing, encoderVar.heading);
}

void odometry()
{
    double dl = 0.0, dr = 0.0, ds = 0.0;
    double dheading = 0.0, meanheading = 0.0;
    int oThisLeft = 0, oThisRight = 0;

    getCounts();
    sscanf(encoderBuffer,"%d %d", &oThisRight,&oThisLeft);
    oThisLeft*= -1;
    oThisRight*= -1;
    dl = (oThisLeft - oLastLeft) / cpmLeft;
    dr = (oThisRight - oLastRight) / cpmRight;
    dheading = (dl - dr) / robotWidth;
    ds = (dl + dr) / 2;         // Distance travelled by the robot
    meanheading = (encoderVar.heading + dheading / 2);

    encoderVar.northing += ds * cos(meanheading);
    encoderVar.easting += ds * sin(meanheading);
    encoderVar.heading += dheading;
    encoderVar.heading = ADJUST_RADIANS(encoderVar.heading);

    oLastLeft=oThisLeft;
    oLastRight=oThisRight;
}
#endif //__ENCODER_C__
