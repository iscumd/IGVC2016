#ifndef __BUFFER_C
#define __BUFFER_C


#include "macros.h"
#include "buffer.h"
#include "sick.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef SAMPLING_DISTANCE
#define SAMPLING_DISTANCE 50.0
#endif


#define ANGLE_SAMPLES 18
#define SAMPLING_ANGLE M_PI/ANGLE_SAMPLES
#define MAXDATAPOINTS 1081
Point bufpoints[MAXDATAPOINTS];
extern short LMSdata[];

double buffer_width = 550;
double buffer_length = 2000.0;
double half_robot_width = 350.0;
//double half_robot_width = 500.0;
int num_points = 0;


bool vector_scan(Point source,Point destination){
    double target_angle = ADJUST_RADIANS(atan2(destination.x - source.x,destination.y - source.y));
    double target_dist = D(destination.x - source.x,destination.y - source.y);
    double dist=0.0;
    int i=0,j=0;
    double tempDist = 0.0;
    Point sample_point;

    do {
        sample_point.x = (i*SAMPLING_DISTANCE) * sin(target_angle);
        sample_point.y = (i*SAMPLING_DISTANCE) * cos(target_angle);
        for(j=0;j<num_points;j++){
            if(bufpoints[j].y < 0) continue;

            tempDist = D(bufpoints[j].x,bufpoints[j].y)*MM2M;
            if(tempDist>bufferTargDist) continue;

            dist = D(sample_point.x - bufpoints[j].x,sample_point.y - bufpoints[j].y);
//            if(dist < buffer_width+half_robot_width){
            if(dist < buffer_width){
                return false;
            }
        }
        i++;
    } while (SAMPLING_DISTANCE*i < target_dist);
    return true;
}
#define DOOM M_PI - 0.1

bool checkAngle(double target_angle/*, double distance*/){
    Point leftWheel, rightWheel;
    Point target;

    leftWheel.y = rightWheel.y = 0;
    leftWheel.x = -half_robot_width;
    rightWheel.x = half_robot_width;
    target.x = buffer_length/*distance*/ * sin(target_angle);
    target.y = buffer_length/*distance*/ * cos(target_angle);

    return (vector_scan(leftWheel,target)&&vector_scan(rightWheel,target));
}

double rightWheelScan(Point target){
    Point source;
    double target_angle = ADJUST_RADIANS(atan2(target.x,target.y));
    double sample_phi;
    int i = 0;
    source.x = half_robot_width;
    source.y = 0;

    do {
        Point sample_point;
        sample_point.x = buffer_length * sin(target_angle-SAMPLING_ANGLE*i);
        sample_point.y = buffer_length * cos(target_angle-SAMPLING_ANGLE*i);
        sample_phi = ADJUST_RADIANS(atan2(sample_point.x, sample_point.y));
        if(vector_scan(source,sample_point)){
            if(checkAngle(sample_phi)){
                if(i==0) return 0;
                return sample_phi;
            }
        }
        i++;
    } while (i < ANGLE_SAMPLES);

//    return DOOM;
    return DEG2RAD(-135.0);
}

double leftWheelScan(Point target){
    Point source;
    double target_angle = ADJUST_RADIANS(atan2(target.x,target.y));
    double sample_phi;
    int i = 0;

    source.x = -half_robot_width;
    source.y = 0.0;

    do {
        Point sample_point;
        sample_point.x = buffer_length * sin(target_angle+SAMPLING_ANGLE*i);
        sample_point.y = buffer_length * cos(target_angle+SAMPLING_ANGLE*i);
        sample_phi = ADJUST_RADIANS(atan2(sample_point.x, sample_point.y));
        if(vector_scan(source,sample_point)){
            if(checkAngle(sample_phi)){
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
double pAngle;
void update_points(void){
    int i;
    int tempdata;
    double pdist = 0.0, tempAngle=0.0;
    Point p1,p2;
    Point b1,b2;
    b1.x = -buffer_length;
    b1.y = 0;
    b2.x = buffer_length;
    b2.y = 0;
    pAngle = DEG2RAD(180);
    num_points = 0;
    for(i=0;i<SICK_NUMPTS;i++){
         tempdata = LMSdata[i];
        if(tempdata < (short) buffer_length){
            double ty = (double) POLAR2Y(i, tempdata);
            if(ty>0){
                bufpoints[num_points].x = (double) POLAR2X(i, tempdata);
                bufpoints[num_points].y = (double) POLAR2Y(i, tempdata);
                num_points++;
            }
        }
    }
    p1 = b1;
    for(i=0;i<num_points;i++){
        p2 = bufpoints[i];
        pdist = D((b1.x-p2.x),(b1.y-p2.y));
        if(pdist>PDISTMIN){
            tempAngle=atan2(((b1.x+p2.x)/2),((b1.y+p2.y)/2));
            if(fabs(tempAngle)<=fabs(pAngle)){
                    pAngle = tempAngle;
                    b1 = p1;
                    b2 = p2;
            }
        }
        p1=p2;
    }
    if(num_points<2) pAngle=0.0;
//    printf("pAng: %f NP: %d\n",pAngle, num_points);
//    printf("b1x: %f b1y: %f\n",b1.x*MM2M,b1.y*MM2M);
//    printf("b2x: %f b2y: %f\n",b2.x*MM2M,b2.y*MM2M);
}
void getBufferAngles(double target_angle){
    Point target;
    double left_angle, right_angle;

    update_points();

    target.x = buffer_length * sin(target_angle);
    target.y = buffer_length * cos(target_angle);

    left_angle = leftWheelScan(target);
    right_angle = rightWheelScan(target);

    buffer_angles.left = left_angle;
    buffer_angles.right = right_angle;
}

void debugBufferAngles(){
    fprintf(stderr, "Left=%f\tRight=%f\t\n",buffer_angles.left,buffer_angles.right);
}
#endif
