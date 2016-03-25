#ifndef __BUFFER_VISION_C
#define __BUFFER_VISION_C


#include "macros.h"
#include "buffer.h"
#include "bufferVision.h"
#include "vision_nav.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef SAMPLING_DISTANCE
#   define SAMPLING_DISTANCE 50.0
#endif
#define ANGLE_SAMPLES 18
#define SAMPLING_ANGLE M_PI/ANGLE_SAMPLES
#define MAXDATAPOINTS 1081
Point visionBufPoints[MAXDATAPOINTS];

double visionTargDist;
double visionBufferWidth = 450;
double visionBufferLength = 2500.0;
double visionHalfRobotWidth = 350.0;
//double visionHalfRobotWidth = 500.0;
int visionNumPoints = 0;


bool visionVectorScan(Point source,Point destination){
    double target_angle = ADJUST_RADIANS(atan2(destination.x - source.x,destination.y - source.y));
    double target_dist = D(destination.x - source.x,destination.y - source.y);
    double dist=0.0;
    int i=0,j=0;
    double tempDist = 0.0;
    Point sample_point;

    do {
        sample_point.x = (i*SAMPLING_DISTANCE) * sin(target_angle);
        sample_point.y = (i*SAMPLING_DISTANCE) * cos(target_angle);
        for(j=0;j<visionNumPoints;j++){
            if(visionBufPoints[j].y < 0) continue;

            tempDist = D(visionBufPoints[j].x,visionBufPoints[j].y)*MM2M;
            if(tempDist>visionTargDist) continue;

            dist = D(sample_point.x - visionBufPoints[j].x,sample_point.y - visionBufPoints[j].y);
//            if(dist < visionBufferWidth+visionHalfRobotWidth){
            if(dist < visionBufferWidth){
                return false;
            }
        }
        i++;
    } while (SAMPLING_DISTANCE*i < target_dist);
    return true;
}

bool visionCheckAngle(double target_angle/*, double distance*/){
    Point leftWheel, rightWheel;
    Point target;

    leftWheel.y = rightWheel.y = 0;
    leftWheel.x = -visionHalfRobotWidth;
    rightWheel.x = visionHalfRobotWidth;
    target.x = visionBufferLength/*distance*/ * sin(target_angle);
    target.y = visionBufferLength/*distance*/ * cos(target_angle);

    return (visionVectorScan(leftWheel,target)&&visionVectorScan(rightWheel,target));
}

double visionRightWheelScan(Point target){
    Point source;
    double target_angle = ADJUST_RADIANS(atan2(target.x,target.y));
    double sample_phi;
    int i = 0;
    source.x = visionHalfRobotWidth;
    source.y = 0;

    do {
        Point sample_point;
        sample_point.x = visionBufferLength * sin(target_angle-SAMPLING_ANGLE*i);
        sample_point.y = visionBufferLength * cos(target_angle-SAMPLING_ANGLE*i);
        sample_phi = ADJUST_RADIANS(atan2(sample_point.x, sample_point.y));
        if(visionVectorScan(source,sample_point)){
            if(visionCheckAngle(sample_phi)){
                if(i==0) return 0;
                return sample_phi;
            }
        }
        i++;
    } while (i < ANGLE_SAMPLES);

//    return DOOM;
    return DEG2RAD(-135.0);
}

double visionLeftWheelScan(Point target){
    Point source;
    double target_angle = ADJUST_RADIANS(atan2(target.x,target.y));
    double sample_phi;
    int i = 0;

    source.x = -visionHalfRobotWidth;
    source.y = 0.0;

    do {
        Point sample_point;
        sample_point.x = visionBufferLength * sin(target_angle+SAMPLING_ANGLE*i);
        sample_point.y = visionBufferLength * cos(target_angle+SAMPLING_ANGLE*i);
        sample_phi = ADJUST_RADIANS(atan2(sample_point.x, sample_point.y));
        if(visionVectorScan(source,sample_point)){
            if(visionCheckAngle(sample_phi)){
                if(i==0) return 0;
                return sample_phi;
            }
        }
        i++;
    } while (i < ANGLE_SAMPLES);
//    return DOOM;
    return DEG2RAD(135.0);
}

#define PDISTMIN 800
double visionPAngle;
void visionUpdatePoints(void){
    int i;
    int tempdata;
    double pdist = 0.0, tempAngle=0.0;
    Point p1,p2;
    Point b1,b2;
    b1.x = -visionBufferLength;
    b1.y = 0;
    b2.x = visionBufferLength;
    b2.y = 0;
    visionPAngle = DEG2RAD(180);
    visionNumPoints = 0;
    for(i=0;i<1081;i++){
         tempdata = vradar[i];
        if(tempdata < (short) visionBufferLength && tempdata !=0){
            double ty = (double) POLAR2Y(i, tempdata);
            if(ty>0){
                visionBufPoints[visionNumPoints].x = (double) POLAR2X(i, tempdata);
                visionBufPoints[visionNumPoints].y = (double) POLAR2Y(i, tempdata);
                visionNumPoints++;
            }
        }
    }
    p1 = b1;
    for(i=0;i<visionNumPoints;i++){
        p2 = visionBufPoints[i];
        pdist = D((b1.x-p2.x),(b1.y-p2.y));
        if(pdist>PDISTMIN){
            tempAngle=atan2(((b1.x+p2.x)/2),((b1.y+p2.y)/2));
            if(fabs(tempAngle)<=fabs(visionPAngle)){
                    visionPAngle = tempAngle;
                    b1 = p1;
                    b2 = p2;
            }
        }
        p1=p2;
    }
    if(visionNumPoints<2) visionPAngle=0.0;
//    printf("pAng: %f NP: %d\n",visionPAngle, visionNumPoints);
//    printf("b1x: %f b1y: %f\n",b1.x*MM2M,b1.y*MM2M);
//    printf("b2x: %f b2y: %f\n",b2.x*MM2M,b2.y*MM2M);
}
void getVisionBufferAngles(double target_angle){
    Point target;
    double left_angle, right_angle;

    visionUpdatePoints();

    target.x = visionBufferLength * sin(target_angle);
    target.y = visionBufferLength * cos(target_angle);

    left_angle = visionLeftWheelScan(target);
    right_angle = visionRightWheelScan(target);

    visionBufferAngles.left = left_angle;
    visionBufferAngles.right = right_angle;
}

void debugVisionBufferAngles(){
    fprintf(stderr, "VLeft=%f\tVRight=%f\t\n",visionBufferAngles.left,visionBufferAngles.right);
}
#endif
