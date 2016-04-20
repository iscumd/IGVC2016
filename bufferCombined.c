#ifndef __BUFFER_COMBINED_C__
#define __BUFFER_COMBINED_C__


#include "macros.h"
#include "bufferCombined.h"
#include "vision_nav.h"
#include "sick.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef SAMPLING_DISTANCE
#   define SAMPLING_DISTANCE 100.0
#endif

#define ANGLE_SAMPLES 18
#define SAMPLING_ANGLE M_PI/ANGLE_SAMPLES
#define MAXDATAPOINTS 1081
Point combinedBufPoints[MAXDATAPOINTS];

double combinedTargDist;
//double combinedBufferWidth = 600;
//double combinedBufferWidth = 500;
double combinedBufferWidth = 425;
//double combinedBufferWidth = 350;
double combinedBufferLength = 2000.0;//this is distance for how far the robot is looking away from it in millimeters
//double combinedBufferLength = 2500.0;
double combinedHalfRobotWidth = 350.0;
//double combinedHalfRobotWidth = 500.0;
int combinedNumPoints = 0;
int badCount = 0;
#define BADCOUNTTHRESH 1
bool combinedVectorScan(Point source,Point destination){//returning true if something is infront of it
    double target_angle = ADJUST_RADIANS(atan2(destination.x - source.x,destination.y - source.y));
    double target_dist = D(destination.x - source.x,destination.y - source.y);
    double dist=0.0;
    int i=0,j=0;
    double tempDist = 0.0;
    Point sample_point;

    do {
        sample_point.x = (i*SAMPLING_DISTANCE) * sin(target_angle);
        sample_point.y = (i*SAMPLING_DISTANCE) * cos(target_angle);
        for(j=0;j<combinedNumPoints;j++){
            if(combinedBufPoints[j].y < 0) continue;

            tempDist = D(combinedBufPoints[j].x,combinedBufPoints[j].y)*MM2M;
            if(tempDist>combinedTargDist) continue;

            dist = D(sample_point.x - combinedBufPoints[j].x,sample_point.y - combinedBufPoints[j].y);
//            if(dist < combinedBufferWidth+combinedHalfRobotWidth){
            if(dist < combinedBufferWidth){
                badCount++;
                if(badCount>BADCOUNTTHRESH) return false;
            }
        }
        i++;
    } while (SAMPLING_DISTANCE*i < target_dist);
    return true;
}

bool combinedCheckAngle(double target_angle/*, double distance*/){//verifying both wheels can make the turn
    Point leftWheel, rightWheel;
    Point target;

    leftWheel.y = rightWheel.y = 0;
    leftWheel.x = -combinedHalfRobotWidth;
    rightWheel.x = combinedHalfRobotWidth;
    target.x = combinedBufferLength/*distance*/ * sin(target_angle);
    target.y = combinedBufferLength/*distance*/ * cos(target_angle);

    return (combinedVectorScan(leftWheel,target)&&combinedVectorScan(rightWheel,target));
}

double combinedRightWheelScan(Point target){
    Point source;
    double target_angle = ADJUST_RADIANS(atan2(target.x,target.y));
    double sample_phi;
    int i = 0;
    source.x = combinedHalfRobotWidth;
    source.y = 0;

    do {
        Point sample_point;
        sample_point.x = combinedBufferLength * sin(target_angle-SAMPLING_ANGLE*i);
        sample_point.y = combinedBufferLength * cos(target_angle-SAMPLING_ANGLE*i);
        sample_phi = ADJUST_RADIANS(atan2(sample_point.x, sample_point.y));
        if(combinedVectorScan(source,sample_point)){
            if(combinedCheckAngle(sample_phi)){
                if(i==0) return 0;
                return sample_phi;
            }
        }
        i++;
    } while (i < ANGLE_SAMPLES);

//    return DOOM;
    return DEG2RAD(-135.0);
}

double combinedLeftWheelScan(Point target){
    Point source;
    double target_angle = ADJUST_RADIANS(atan2(target.x,target.y));
    double sample_phi;
    int i = 0;

    source.x = -combinedHalfRobotWidth;
    source.y = 0.0;

    do {
        Point sample_point;
        sample_point.x = combinedBufferLength * sin(target_angle+SAMPLING_ANGLE*i);
        sample_point.y = combinedBufferLength * cos(target_angle+SAMPLING_ANGLE*i);
        sample_phi = ADJUST_RADIANS(atan2(sample_point.x, sample_point.y));
        if(combinedVectorScan(source,sample_point)){
            if(combinedCheckAngle(sample_phi)){
                if(i==0) return 0;
                return sample_phi;
            }
        }
        i++;
    } while (i < ANGLE_SAMPLES);
//    return DOOM;
    return DEG2RAD(135.0);
}

void combinedUpdatePoints(int useVision){
    int i;
    int tempdata;
    combinedNumPoints = 0;
    for(i=0;i<1081;i++){
        if(useVision){
            tempdata = vradar[i];
            if(tempdata < (short) combinedBufferLength){
                double ty = (double) POLAR2Y(i, tempdata);
                if(ty>0){
                    combinedBufPoints[combinedNumPoints].x = (double) POLAR2X(i, tempdata);
                    combinedBufPoints[combinedNumPoints].y = (double) POLAR2Y(i, tempdata);
                    combinedNumPoints++;
                }
            }
        }
        tempdata = LMSdata[i];
        if(tempdata < (short) combinedBufferLength){// if within halo, then convert location to xy
            double ty = (double) POLAR2Y(i, tempdata);
            if(ty>0){
                combinedBufPoints[combinedNumPoints].x = (double) POLAR2X(i, tempdata);
                combinedBufPoints[combinedNumPoints].y = (double) POLAR2Y(i, tempdata);
                combinedNumPoints++;
            }
        }
    }
}
void getCombinedBufferAngles(double target_angle, int useVision){
    Point target;
    double left_angle, right_angle;
    badCount = 0;
    combinedUpdatePoints(useVision);

    target.x = combinedBufferLength * sin(target_angle);
    target.y = combinedBufferLength * cos(target_angle);

    left_angle = combinedLeftWheelScan(target);
    right_angle = combinedRightWheelScan(target);

    combinedBufferAngles.left = left_angle;
    combinedBufferAngles.right = right_angle;
}

void debugCombinedBufferAngles(){
    fprintf(stderr, "CLeft=%f\tCRight=%f\t\n",combinedBufferAngles.left,combinedBufferAngles.right);
}
#endif
