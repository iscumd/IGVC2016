#ifndef VISUALIZER_C
#define VISUALIZER_C
#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <conio.h>

#include "macros.h"
#include "visualizer.h"
#include "sick.h"
#include "location.h"
#include "objects.h"
#include "landmarks.h"
#include "vision_nav.h"

IplImage *path_planner, *pathMap;

CvFont font_big, font_small, font_subscript;
float snow_w = 1.5, snow_h = 10.0;//add to config
float max_fw = 18.0, max_fl = 18.0;//add to config
float ppmx, ppmy;
int center_x, base_y;
float map_ppmx, map_ppmy;
int map_center_x, map_base_y;

int PROWS, PCOLS;

void pp_waypoints(double rx, double ry, double rt){
    CvScalar color = ORANGE;
    int z = 0;


    cvLine(path_planner, cvPoint(center_x, base_y),
           cvPoint(center_x + ppmx*targetListXY[z].x, base_y - ppmy*targetListXY[z].y), color, robotWidth*ppmx, LINE_TYPE, SHIFT);

//Plot the targetListXYs Path line
    for( z = 0; z< maxTargets-1; z++){
        if(z%2) color = ORANGE;
        else color = WHITE;

        cvLine(path_planner,  cvPoint(center_x + ppmx*targetListXY[z].x, base_y - ppmy*targetListXY[z].y),
                cvPoint(center_x + ppmx*targetListXY[z+1].x, base_y - ppmy*targetListXY[z+1].y), color, robotWidth*ppmx, LINE_TYPE, SHIFT);
    }
//Plot targetList Point Circle
    cvCircle(path_planner, cvPoint(center_x, base_y), 0.20*ppmx, RED, CV_FILLED, LINE_TYPE, SHIFT);
    for(z = 0; z< maxTargets; z++){
        color = RED;
        cvCircle(path_planner, cvPoint(center_x + ppmx*targetListXY[z].x, base_y - ppmy*targetListXY[z].y), 0.20*ppmx, color, CV_FILLED, LINE_TYPE, SHIFT);
    }
//Plot Current Robot Point Circle
    cvCircle(path_planner, cvPoint(center_x + rx*ppmx, base_y - ry*ppmy), 0.20*ppmx, YELLOW, CV_FILLED, LINE_TYPE, SHIFT);
}

void pp_landmarks(){
    //Plot the Landmarks
        int z = 0;
        CvScalar color;
        for (z = 0; z < num_landmarks; z++) {
            if (KLM[z].rejected)
                color = RED;
            else if(KLM[z].found){
                color = GREEN;
            }
            else
                color = YELLOW;

            float relx = ppmx * KLM[z].x * MM2M;
            float rely = ppmy * KLM[z].y * MM2M;
            cvCircle(path_planner, cvPoint(center_x + relx, base_y - rely), landmark_tolerance*MM2M*ppmy, color, 2, LINE_TYPE, SHIFT);
        }

}

void pp_objects(){
    //Plot the Landmarks
        int z = 0;
        CvScalar color;
        for (z = 0; z < num_objects; z++) {
            color = BLUE;

            float relx = ppmx * Objects[z].x * MM2M;
            float rely = ppmy * Objects[z].y * MM2M;
            cvCircle(path_planner, cvPoint(center_x + relx, base_y - rely), landmark_tolerance*MM2M*ppmy, color, 2, LINE_TYPE, SHIFT);
        }

}


void pp_robot(double rx, double ry, double rt){
    //Paint the robot
        int point_x = center_x + (ppmx * rx);
        int point_y = base_y - (ppmy * ry);

/*Plot Robot Frame*/
        float ang1 = atan2(robotWidth/2, 0.30);
        float dist1 = D(robotWidth/2, 0.30);
        float ang2 = atan2(-robotWidth/2, 0.30);
        float dist2 = D(-robotWidth/2, 0.30);
        float ang3 = atan2(-robotWidth/2, -0.45);
        float dist3 = D(-robotWidth/2, -0.45);
        float ang4 = atan2(robotWidth/2, -0.45);
        float dist4 = D(robotWidth/2, -0.45);
        int robotnpts[] = {4};
        CvPoint robot[4] =   {point_x + ppmx*(dist1*sin(rt+ang1)), point_y - ppmy*(dist1*cos(rt+ang1)),
                                    point_x + ppmx*(dist2*sin(rt+ang2)), point_y - ppmy*(dist2*cos(rt+ang2)),
                                        point_x + ppmx*(dist3*sin(rt+ang3)), point_y - ppmy*(dist3*cos(rt+ang3)),
                                            point_x + ppmx*(dist4*sin(rt+ang4)), point_y - ppmy*(dist4*cos(rt+ang4))
                                };
        CvPoint *rarr[1] = {robot};
        cvFillPoly(path_planner, rarr, robotnpts, 1, BLUE, LINE_TYPE, SHIFT);
        cvPolyLine(path_planner, rarr, robotnpts, 1, true,YELLOW, 1.75, LINE_TYPE, SHIFT);
/*Plot Robot Trap Frame*/
        ang1 = atan2(0.2041,-0.45);
        dist1 = D(0.2041,-0.45);
        ang2 = atan2(0.1020,-0.65);
        dist2 = D(0.1020,-0.65);
        ang3 = atan2(-0.1020,-0.65);
        dist3 = D(-0.1020,-0.65);
        ang4 = atan2(-0.2041,-0.45);
        dist4 = D(-0.2041,-0.45);
        int trapnpts[] = {4};
        CvPoint trapezoid[4] =   {point_x + ppmx*(dist1*sin(rt+ang1)), point_y - ppmy*(dist1*cos(rt+ang1)),
                                    point_x + ppmx*(dist2*sin(rt+ang2)), point_y - ppmy*(dist2*cos(rt+ang2)),
                                        point_x + ppmx*(dist3*sin(rt+ang3)), point_y - ppmy*(dist3*cos(rt+ang3)),
                                            point_x + ppmx*(dist4*sin(rt+ang4)), point_y - ppmy*(dist4*cos(rt+ang4))
                                };
        CvPoint *tarr[1] = {trapezoid};
        cvFillPoly(path_planner, tarr, trapnpts, 1, BLUE, LINE_TYPE, SHIFT);
        cvPolyLine(path_planner, tarr, trapnpts, 1, true,YELLOW, 1.75, LINE_TYPE, SHIFT);
/*Plot Front Right Tire*/
        ang1 = atan2(0.45, -0.10);
        dist1 = D(0.45, -0.10);
        ang2 = atan2(0.27, -0.10);
        dist2 = D(0.27, -0.10);
        ang3 = atan2(0.27, 0.15);
        dist3 = D(0.27, 0.15);
        ang4 = atan2(0.45, 0.15);
        dist4 = D(0.45, 0.15);
        int frtirenpts[] = {4};
        CvPoint frtire[4] =   {point_x + ppmx*(dist1*sin(rt+ang1)), point_y - ppmy*(dist1*cos(rt+ang1)),
                                    point_x + ppmx*(dist2*sin(rt+ang2)), point_y - ppmy*(dist2*cos(rt+ang2)),
                                        point_x + ppmx*(dist3*sin(rt+ang3)), point_y - ppmy*(dist3*cos(rt+ang3)),
                                            point_x + ppmx*(dist4*sin(rt+ang4)), point_y - ppmy*(dist4*cos(rt+ang4))
                                };
        CvPoint *frarr[1] = {frtire};
        cvFillPoly(path_planner, frarr, frtirenpts, 1, BLACK, LINE_TYPE, SHIFT);
        cvPolyLine(path_planner, frarr, frtirenpts, 1, true,YELLOW, 1.25, LINE_TYPE, SHIFT);

/*Plot Front Left Tire*/
        ang1 = atan2(-0.45, -0.10);
        dist1 = D(-0.45, -0.10);
        ang2 = atan2(-0.27, -0.10);
        dist2 = D(-0.27, -0.10);
        ang3 = atan2(-0.27, 0.15);
        dist3 = D(-0.27, 0.15);
        ang4 = atan2(-0.45, 0.15);
        dist4 = D(-0.45, 0.15);
        int fltirenpts[] = {4};
        CvPoint fltire[4] =   {point_x + ppmx*(dist1*sin(rt+ang1)), point_y - ppmy*(dist1*cos(rt+ang1)),
                                    point_x + ppmx*(dist2*sin(rt+ang2)), point_y - ppmy*(dist2*cos(rt+ang2)),
                                        point_x + ppmx*(dist3*sin(rt+ang3)), point_y - ppmy*(dist3*cos(rt+ang3)),
                                            point_x + ppmx*(dist4*sin(rt+ang4)), point_y - ppmy*(dist4*cos(rt+ang4))
                                };
        CvPoint *flarr[1] = {fltire};
        cvFillPoly(path_planner, flarr, fltirenpts, 1, BLACK, LINE_TYPE, SHIFT);
        cvPolyLine(path_planner, flarr, fltirenpts, 1, true,YELLOW, 1.25, LINE_TYPE, SHIFT);

/*/*Plot Back Tire*/
        ang1 = atan2(0.09, -0.80);
        dist1 = D(0.09, -0.80);
        ang2 = atan2(-0.09, -0.80);
        dist2 = D(-0.09, -0.80);
        ang3 = atan2(-0.09, -0.55);
        dist3 = D(-0.09, -0.55);
        ang4 = atan2(0.09, -0.55);
        dist4 = D(0.09, -0.55);
        int brtirenpts[] = {4};
        CvPoint brtire[4] =   {point_x + ppmx*(dist1*sin(rt+ang1)), point_y - ppmy*(dist1*cos(rt+ang1)),
                                    point_x + ppmx*(dist2*sin(rt+ang2)), point_y - ppmy*(dist2*cos(rt+ang2)),
                                        point_x + ppmx*(dist3*sin(rt+ang3)), point_y - ppmy*(dist3*cos(rt+ang3)),
                                            point_x + ppmx*(dist4*sin(rt+ang4)), point_y - ppmy*(dist4*cos(rt+ang4))
                                };
        CvPoint *brarr[1] = {brtire};
        cvFillPoly(path_planner, brarr, brtirenpts, 1, BLACK, LINE_TYPE, SHIFT);
        cvPolyLine(path_planner, brarr, brtirenpts, 1, true,YELLOW, 1.25, LINE_TYPE, SHIFT);
}

void pp_SICK(double rx, double ry, double rt)
{
    int j = 0;
    float angle;

    for (j = 0; j < SICK_NUMPTS; j++){
        int tempdata = LMSdata[j];
        angle = 135 - j * RESOLUTION;
        if (tempdata <= 15000){//Max_Radius
            cvCircle(path_planner, cvPoint( center_x + rx *ppmx + (tempdata * MM2M * ppmx) * sin(DEG2RAD(angle)+rt), base_y - ry * ppmy - (tempdata * MM2M * ppmy) * cos(DEG2RAD(angle)+rt)), 2, YELLOW, CV_FILLED, LINE_TYPE, SHIFT);     //circle for robot
       }
    }
}
void pp_VSICK(double rx, double ry, double rt)
{
    int j = 0;
    float angle;

    for (j = 0; j < SICK_NUMPTS; j++){
        int tempdata = vradar[j];
        angle = 135 - j * RESOLUTION;
        if (tempdata <= 15000){//Max_Radius
            cvCircle(path_planner, cvPoint( center_x + rx *ppmx + (tempdata * MM2M * ppmx) * sin(DEG2RAD(angle)+rt), base_y - ry * ppmy - (tempdata * MM2M * ppmy) * cos(DEG2RAD(angle)+rt)), 2, GREEN, CV_FILLED, LINE_TYPE, SHIFT);     //circle for robot
       }
    }
}

void paintPathPlanner(double rx, double ry, double rt)
{

    if (path_planner == NULL) {
        path_planner = cvCreateImage(cvSize(PCOLS, PROWS), 8, 3);
        cvZero(path_planner);
    }

    if(path_planner != NULL){
//        cvNamedWindow("Path Planner", CV_WINDOW_AUTOSIZE);
        cvNamedWindow("Path Planner", CV_WINDOW_NORMAL);

        float t = 0.0;

        int starting_x = 0;
        int starting_y = 0;
        int y_indent = 40, x_indent = 40;//add to config
        int max_x = PCOLS-2*x_indent;
        int max_y = PROWS-2*y_indent;
        int x_range = max_x - starting_x;
        int y_range = max_y - starting_y;


        ppmx = (float) (x_range - 2 * x_indent) / (fieldWidth + t);
        ppmy = (float) (y_range - y_indent) / (fieldLength);
        while (fieldLength * ppmx > (float) (max_y - y_indent)) {
            t += 0.5;
            ppmx = (float) x_range / (fieldWidth + t);
        }

        int NWcorner_x = starting_x + x_indent;
        int NWcorner_y = starting_y + y_indent;
        int SEcorner_x = (int) NWcorner_x + fieldWidth * ppmx;
        int SEcorner_y = (int) NWcorner_y + fieldLength * ppmy;

    //Draw field
        cvRectangle(path_planner, cvPoint(NWcorner_x, NWcorner_y),
                    cvPoint(SEcorner_x, SEcorner_y), GRAY,
                    CV_FILLED, LINE_TYPE, SHIFT);



        center_x = NWcorner_x + (ppmx * (fieldWidth / 2));
        base_y = (NWcorner_y+SEcorner_y)/2;

    //Paint Grid Lines
        int spacing, counter, point_x, point_y;

        for(spacing = 1, counter = 0; spacing*counter < fieldLength; counter++){
            point_y = max_y - (ppmy * counter * spacing);
            cvLine(path_planner, cvPoint(SEcorner_x, point_y), cvPoint(NWcorner_x, point_y), YELLOW, 1, LINE_TYPE, SHIFT);   //Horozontal-axis
        }
        for(spacing = 1, counter = 0; spacing*counter < fieldWidth; counter++){
            point_x = center_x - fieldWidth/2*ppmx+ (ppmx*spacing*counter);
            cvLine(path_planner, cvPoint(point_x, SEcorner_y), cvPoint(point_x, NWcorner_y), YELLOW, 1, LINE_TYPE, SHIFT);        //Vertical-axis
        }
    //Draw the origin
        cvCircle(path_planner, cvPoint(center_x, base_y), 3, WHITE, CV_FILLED, LINE_TYPE, SHIFT);
        cvLine(path_planner, cvPoint(SEcorner_x, base_y), cvPoint(NWcorner_x, base_y), RED, 2, LINE_TYPE, SHIFT);   //Horozontal-axis
        cvLine(path_planner, cvPoint(center_x, max_y), cvPoint(center_x, starting_y + y_indent), RED, 2, LINE_TYPE, SHIFT);        //Vertical-axis\

//        pp_waypoints(rx, ry, rt);
//        pp_objects();
//        pp_landmarks(rx, ry, rt);
//        pp_robot(rx, ry, rt);
//        pp_SICK(rx, ry, rt);
        pp_VSICK(rx, ry, rt);
        char buffer[100];
        sprintf(buffer, "Target X = %.2f Target Y = %.2f", targetX, targetY);
        cvPutText(path_planner, buffer, cvPoint(NWcorner_x, SEcorner_y+60), &font_small, WHITE);

        sprintf(buffer, "X = %.2f  Y = %.2f  Theata = %.2f", rx, ry, rt);
        cvPutText(path_planner, buffer, cvPoint(NWcorner_x, SEcorner_y+90), &font_small, WHITE);
    }
}




void showPlot()
{
    cvShowImage("Path Planner", path_planner);
    cvWaitKey(1);
    cvReleaseImage(&path_planner);
}


void mapRobot(double rx, double ry, double rt){
    //Paint the robot
        int point_x = map_center_x + (map_ppmx * rx);
        int point_y = map_base_y - (map_ppmy * ry);

/*Plot Robot Frame*/
        float ang1 = atan2(robotWidth/2, 0.30);
        float dist1 = D(robotWidth/2, 0.30);
        float ang2 = atan2(-robotWidth/2, 0.30);
        float dist2 = D(-robotWidth/2, 0.30);
        float ang3 = atan2(-robotWidth/2, -0.45);
        float dist3 = D(-robotWidth/2, -0.45);
        float ang4 = atan2(robotWidth/2, -0.45);
        float dist4 = D(robotWidth/2, -0.45);
        int robotnpts[] = {4};
        CvPoint robot[4] =   {point_x + map_ppmx*(dist1*sin(rt+ang1)), point_y - map_ppmy*(dist1*cos(rt+ang1)),
                                    point_x + map_ppmx*(dist2*sin(rt+ang2)), point_y - map_ppmy*(dist2*cos(rt+ang2)),
                                        point_x + map_ppmx*(dist3*sin(rt+ang3)), point_y - map_ppmy*(dist3*cos(rt+ang3)),
                                            point_x + map_ppmx*(dist4*sin(rt+ang4)), point_y - map_ppmy*(dist4*cos(rt+ang4))
                                };
        CvPoint *rarr[1] = {robot};
        cvFillPoly(pathMap, rarr, robotnpts, 1, BLUE, LINE_TYPE, SHIFT);
        cvPolyLine(pathMap, rarr, robotnpts, 1, true,YELLOW, 1.75, LINE_TYPE, SHIFT);
/*Plot Robot Trap Frame*/
        ang1 = atan2(0.2041,-0.45);
        dist1 = D(0.2041,-0.45);
        ang2 = atan2(0.1020,-0.65);
        dist2 = D(0.1020,-0.65);
        ang3 = atan2(-0.1020,-0.65);
        dist3 = D(-0.1020,-0.65);
        ang4 = atan2(-0.2041,-0.45);
        dist4 = D(-0.2041,-0.45);
        int trapnpts[] = {4};
        CvPoint trapezoid[4] =   {point_x + map_ppmx*(dist1*sin(rt+ang1)), point_y - map_ppmy*(dist1*cos(rt+ang1)),
                                    point_x + map_ppmx*(dist2*sin(rt+ang2)), point_y - map_ppmy*(dist2*cos(rt+ang2)),
                                        point_x + map_ppmx*(dist3*sin(rt+ang3)), point_y - map_ppmy*(dist3*cos(rt+ang3)),
                                            point_x + map_ppmx*(dist4*sin(rt+ang4)), point_y - map_ppmy*(dist4*cos(rt+ang4))
                                };
        CvPoint *tarr[1] = {trapezoid};
        cvFillPoly(pathMap, tarr, trapnpts, 1, BLUE, LINE_TYPE, SHIFT);
        cvPolyLine(pathMap, tarr, trapnpts, 1, true,YELLOW, 1.75, LINE_TYPE, SHIFT);
/*Plot Front Right Tire*/
        ang1 = atan2(0.45, -0.10);
        dist1 = D(0.45, -0.10);
        ang2 = atan2(0.27, -0.10);
        dist2 = D(0.27, -0.10);
        ang3 = atan2(0.27, 0.15);
        dist3 = D(0.27, 0.15);
        ang4 = atan2(0.45, 0.15);
        dist4 = D(0.45, 0.15);
        int frtirenpts[] = {4};
        CvPoint frtire[4] =   {point_x + map_ppmx*(dist1*sin(rt+ang1)), point_y - map_ppmy*(dist1*cos(rt+ang1)),
                                    point_x + map_ppmx*(dist2*sin(rt+ang2)), point_y - map_ppmy*(dist2*cos(rt+ang2)),
                                        point_x + map_ppmx*(dist3*sin(rt+ang3)), point_y - map_ppmy*(dist3*cos(rt+ang3)),
                                            point_x + map_ppmx*(dist4*sin(rt+ang4)), point_y - map_ppmy*(dist4*cos(rt+ang4))
                                };
        CvPoint *frarr[1] = {frtire};
        cvFillPoly(pathMap, frarr, frtirenpts, 1, BLACK, LINE_TYPE, SHIFT);
        cvPolyLine(pathMap, frarr, frtirenpts, 1, true,YELLOW, 1.25, LINE_TYPE, SHIFT);

/*Plot Front Left Tire*/
        ang1 = atan2(-0.45, -0.10);
        dist1 = D(-0.45, -0.10);
        ang2 = atan2(-0.27, -0.10);
        dist2 = D(-0.27, -0.10);
        ang3 = atan2(-0.27, 0.15);
        dist3 = D(-0.27, 0.15);
        ang4 = atan2(-0.45, 0.15);
        dist4 = D(-0.45, 0.15);
        int fltirenpts[] = {4};
        CvPoint fltire[4] =   {point_x + map_ppmx*(dist1*sin(rt+ang1)), point_y - map_ppmy*(dist1*cos(rt+ang1)),
                                    point_x + map_ppmx*(dist2*sin(rt+ang2)), point_y - map_ppmy*(dist2*cos(rt+ang2)),
                                        point_x + map_ppmx*(dist3*sin(rt+ang3)), point_y - map_ppmy*(dist3*cos(rt+ang3)),
                                            point_x + map_ppmx*(dist4*sin(rt+ang4)), point_y - map_ppmy*(dist4*cos(rt+ang4))
                                };
        CvPoint *flarr[1] = {fltire};
        cvFillPoly(pathMap, flarr, fltirenpts, 1, BLACK, LINE_TYPE, SHIFT);
        cvPolyLine(pathMap, flarr, fltirenpts, 1, true,YELLOW, 1.25, LINE_TYPE, SHIFT);

/*/*Plot Back Tire*/
        ang1 = atan2(0.09, -0.80);
        dist1 = D(0.09, -0.80);
        ang2 = atan2(-0.09, -0.80);
        dist2 = D(-0.09, -0.80);
        ang3 = atan2(-0.09, -0.55);
        dist3 = D(-0.09, -0.55);
        ang4 = atan2(0.09, -0.55);
        dist4 = D(0.09, -0.55);
        int brtirenpts[] = {4};
        CvPoint brtire[4] =   {point_x + map_ppmx*(dist1*sin(rt+ang1)), point_y - map_ppmy*(dist1*cos(rt+ang1)),
                                    point_x + map_ppmx*(dist2*sin(rt+ang2)), point_y - map_ppmy*(dist2*cos(rt+ang2)),
                                        point_x + map_ppmx*(dist3*sin(rt+ang3)), point_y - map_ppmy*(dist3*cos(rt+ang3)),
                                            point_x + map_ppmx*(dist4*sin(rt+ang4)), point_y - map_ppmy*(dist4*cos(rt+ang4))
                                };
        CvPoint *brarr[1] = {brtire};
        cvFillPoly(pathMap, brarr, brtirenpts, 1, BLACK, LINE_TYPE, SHIFT);
        cvPolyLine(pathMap, brarr, brtirenpts, 1, true,YELLOW, 1.25, LINE_TYPE, SHIFT);
}
void mapSICK(double rx, double ry, double rt)
{
    int j = 0;
    float angle;

    for (j = 0; j < SICK_NUMPTS; j++){
        int tempdata = LMSdata[j];
        angle = 135 - j * RESOLUTION;
        if (tempdata <= 4000&&fabs(angle<90)){//Max_Radius
//        if (tempdata <= 4000){//Max_Radius
            cvCircle(pathMap, cvPoint( map_center_x + rx *map_ppmx + (tempdata * MM2M * map_ppmx) * sin(DEG2RAD(angle)+rt), map_base_y - ry * map_ppmy - (tempdata * MM2M * map_ppmy) * cos(DEG2RAD(angle)+rt)), 1, WHITE, CV_FILLED, LINE_TYPE, SHIFT);     //circle for robot
       }
    }
}
void clearMapSection(double rx, double ry, double rt){
    int j = 0;

    cvCircle(pathMap, cvPoint( map_center_x + rx *map_ppmx, map_base_y - ry * map_ppmy), 4, BLUE, CV_FILLED, LINE_TYPE, SHIFT);     //circle for robot

    for(j=0;j<4001;j++){
        cvEllipse(pathMap,cvPoint(map_center_x + rx *map_ppmx, map_base_y-ry*map_ppmy),cvSize(j*MM2M*map_ppmx,j*MM2M*map_ppmy),RAD2DEG(rt),0,180,BLACK,2,8 ,0);
    }
}
void mapVSICK(double rx, double ry, double rt)
{
    int j = 0;
    float angle;
    for (j = 0; j < SICK_NUMPTS; j++){
        int tempdata = vradar[j];
        angle = 135 - j * RESOLUTION;
        if (tempdata <= 4000){//Max_Radius
            cvCircle(pathMap, cvPoint( map_center_x + rx *map_ppmx + (tempdata * MM2M * map_ppmx) * sin(DEG2RAD(angle)+rt), map_base_y - ry * map_ppmy - (tempdata * MM2M * map_ppmy) * cos(DEG2RAD(angle)+rt)), 1, WHITE, CV_FILLED, LINE_TYPE, SHIFT);     //circle for robot
       }
    }
}
void initMap(double rx, double ry, double rt){

    PCOLS = 800; PROWS = 800;
    map_ppmx = (float) (PCOLS) / (fieldWidth);
    map_ppmy = (float) (PROWS) / (fieldLength);
    map_base_y = PROWS/2 -1;
    map_center_x = PCOLS/2-1;
    if (pathMap == NULL) {
//        pathMap = cvCreateImage(cvSize(PCOLS, PROWS), 8, 1);
        pathMap = cvCreateImage(cvSize(PCOLS, PROWS), 8, 3);
        cvNamedWindow("Path Map", CV_WINDOW_AUTOSIZE);
        cvZero(pathMap);
    }else{
        cvZero(pathMap);
    }
}
void showMap()
{
    cvShowImage("Path Map", pathMap);
    cvWaitKey(1);
//    cvReleaseImage(&path_planner);
}
void initVisualizer()
{
    PCOLS = 800; PROWS = 800;
    cvInitFont(&font_subscript, CV_FONT_HERSHEY_COMPLEX_SMALL, 0.7, 0.7, 0, 1,CV_AA);
    cvInitFont(&font_small, CV_FONT_HERSHEY_SIMPLEX, 0.55, 0.55, 0, 1,CV_AA);
    cvInitFont(&font_big, CV_FONT_HERSHEY_SIMPLEX, 0.8, 0.8, 0, 2, CV_AA);
}
#endif
