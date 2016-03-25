#include <stdio.h>
#include <conio.h>
#include <stdlib.h>
#include <math.h>


#define EVERGREENLAT 42.320302 //Evergreen Corner Latitude
#define EVERGREENLON -83.232685 //Evergreen Corner Longitude
#define LANELAT 42.320137;//IAVS Tree Latitude
#define LANELON -83.232622//IAVS Tree Longitude
#define ELBLAT 42.320197;//ELB Mid Trees Latitude
#define ELBLON -83.232787;//ELB Mid Trees Longitude
#define CENTERLAT 42.320229;//Center Island Latitude
#define CENTERLON -83.232730;//Center Island Longitude
#define GUESSLAT 42.320176;//Center Island Latitude
#define GUESSLON -83.232620;//Center Island Longitude
#define GUESS2LAT 42.320121;//Center Island Latitude
#define GUESS2LON -83.232624;//Center Island Longitude
//42.320149 -83.232622

#include "macros.h"
#include "serial.h"
#include "socket.h"
#include "joystick.h"
#include "control.h"
#include "roboteq.h"
#include "nmeap.h"
#include "gps.h"
#include "buffer.h"
#include "bufferVision.h"
#include "bufferCombined.h"
#include "sick.h"
#include "landmarks.h"
#include "objects.h"
#include "location.h"
#include "visualizer.h"
#include "vision_nav.h"
#include "os5000.h"

/*SENSOR OPTIONS*/
#define USE_VISION
#define USE_LIDAR
#define USE_GPS
#define USE_GPS2
#define USE_COMPASS

/*DEBUG OPTIONS*/
//#define DEBUG_TARGET
//#define DEBUG_GUIDE
#define DEBUG_GPS
#define DEBUG_GPS2
//#define DEBUG_LIDAR
#define DEBUG_VISUALIZER
//#define DEBUG_BUFFER
//#define DEBUG_COMPASS

/*joystick.h Definitions*/
struct JoystickData joy0;

/*roboteq.h Definitions*/
//int roboteqPort = 12;
int roboteqPort = 7;

/*os5000.h Definitions*/
struct _OSDATA_ os5000data;
int os5000port = 10;
int os5000baud = 115200;

/*control.h Definitions*/
CVAR cvar,cvar2,cvar3,cvar4;
double destinationThresh = 0.75;
double approachingThresh = 1.75;
double maxTurn = 1.0;
int approachingTarget, reachedSubTarget;
int useSubTarget=0;

/*gps.h Definitions*/
GPSVAR gpsvar,gpsvar2;
GPSPNT targetsGPS[4];
double startLatitude = LANELAT;//Center Island Latitude
double startLongitude = LANELON;//Center Island Longitude

/*objects.h Definitions*/
int SEPARATION_THRESH=750;
int NONSEPARATION_THRESH=350;
int HIGH_THRESH=100;
int LOW_THRESH=3;
int MAX_OBJECTS=50;



/*landmarks.h, visualizer.h*/
double robotX = 0.0, robotY = 0.0, robotTheta = 0.0;
double robotX2 = 0.0, robotY2 = 0.0, robotTheta2 = 0.0;
double targetX, targetY;
double landmark_tolerance = 500.0;
double fieldWidth = 30.0, fieldLength = 30.0;

/*main Definitions*/
void debugTarget();//Debug Target function
int imuCalibrated = 0;
double robotWidth = 0.7112;
int maxSpeed = 50;
int currentTargetIndex = 0, nextTargetIndex = 0, maxTargets = 0, allTargetsReached = 0;
int autoOn = 0;
int mode = 0;
XY targetListXY[100];
XY currentXY, targetXY, previousXY, nextXY;
XY subXY;
XY currentXY2;

int main(){
    int i = 0;
    int mapCount = 0;
    targetsGPS[maxTargets].lat = LANELAT;
    targetsGPS[maxTargets].lon = LANELON;
    maxTargets++;
    targetsGPS[maxTargets].lat = GUESSLAT;
    targetsGPS[maxTargets].lon = GUESSLON;
    maxTargets++;
#if 1
    targetsGPS[maxTargets].lat = ELBLAT;
    targetsGPS[maxTargets].lon = ELBLON;
    maxTargets++;
    targetsGPS[maxTargets].lat = CENTERLAT;
    targetsGPS[maxTargets].lon = CENTERLON;
    maxTargets++;
#else
    targetsGPS[maxTargets].lat = CENTERLAT;
    targetsGPS[maxTargets].lon = CENTERLON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ELBLAT;
    targetsGPS[maxTargets].lon = ELBLON;
    maxTargets++;
#endif
    targetsGPS[maxTargets].lat = GUESSLAT;
    targetsGPS[maxTargets].lon = GUESSLON;
    maxTargets++;
    targetsGPS[maxTargets].lat = LANELAT;
    targetsGPS[maxTargets].lon = LANELON;
    maxTargets++;
    targetsGPS[maxTargets].lat = GUESS2LAT;
    targetsGPS[maxTargets].lon = GUESS2LON;
    maxTargets++;


    for(i=0;i<maxTargets;i++){
        targetListXY[i].x = GPSX(targetsGPS[i].lon, startLongitude);
        targetListXY[i].y = GPSY(targetsGPS[i].lat, startLatitude);
    }
    currentXY.x = GPSX(gpsvar.longitude,startLongitude);
    currentXY.y = GPSY(gpsvar.latitude,startLatitude);

    targetXY = targetListXY[currentTargetIndex];
    subXY = targetXY;
    nextTargetIndex = (currentTargetIndex + 1)%maxTargets;
    nextXY = targetListXY[nextTargetIndex];
    previousXY.x = GPSX(startLongitude, startLongitude);
    previousXY.y = GPSY(startLatitude, startLatitude);

    initRoboteq();  /* Initialize roboteq */
    initGuide();
#ifdef USE_VISION
    initVision();
#endif //USE_VISION
#ifdef USE_GPS
    initGPS();
    initParser();
#endif //USE_GPS
#ifdef USE_GPS2
    initGPS2();
    initParser2();
#endif //USE_GPS2
#ifdef USE_LIDAR
    initObjects();
    initSICK();
#endif //USE_LIDAR
#ifdef USE_COMPASS
    initOS5000();
#endif //USE_COMPASS
#ifdef DEBUG_VISUALIZER
    initVisualizer();
    initMap(0,0,0);
#endif //DEBUG_VISUALIZER
    while(1){
        double dir = 1.0;
        double speed = 0.0, turn = 0.0;
        static double turnBoost = 0.750;//Multiplier for turn. Adjust to smooth jerky motions. Usually < 1.0
        static int lSpeed = 0, rSpeed = 0;//Wheel Speed Variables
        int useVisionOnly = 0, useVisLidar=0;
        if (joystick() != 0) {
            if (joy0.buttons & LB_BTN) {
                speed = -joy0.axis[1]; //Up is negative on joystick negate so positive when going forward
                turn = joy0.axis[0];

                lSpeed = (int)((speed + turnBoost*turn)*maxSpeed);
                rSpeed = (int)((speed - turnBoost*turn)*maxSpeed);
                }else{
                     rSpeed=lSpeed=0;
            }
            if(joy0.buttons & A_BTN){
//                lSpeed=rSpeed=maxSpeed;
                useVisionOnly =1;
            }else{
                useVisionOnly =0;
            }
            if(joy0.buttons & B_BTN){
                saveImage =1;
            }else{
                saveImage =0;
            }
            if(joy0.buttons & X_BTN){
                useVisLidar =1;
            }else{
                useVisLidar =0;
            }
            if(joy0.buttons & BACK_BTN){
                subXY = targetXY;
                allTargetsReached = 0;
                currentTargetIndex = 0;
                targetXY = targetListXY[0];
                mode = 0;
            }
            if(joy0.buttons & START_BTN){
                allTargetsReached = 0;
                currentTargetIndex = 2;
                targetXY = targetListXY[currentTargetIndex];
                subXY = targetXY;
                mode = 3;
                autoOn = 1;
            }
            if(joy0.buttons & RB_BTN){
                autoOn = 1;
                mode=1;
            }
            if(joy0.buttons & Y_BTN){
                autoOn = 0;
                mode =0;
            }
        } else{
            printf("No Joystick Found!\n");
            rSpeed=lSpeed=0;
        }



#ifdef USE_GPS
        readGPS();
        currentXY.x = GPSX(gpsvar.longitude,startLongitude);
        currentXY.y = GPSY(gpsvar.latitude,startLatitude);
        robotTheta = ADJUST_RADIANS(DEG2RAD(gpsvar.course));
#else
        currentXY.x = 0.0;
        currentXY.y = 0.0;
        robotTheta = 0.0;
#endif //USE_GPS
#ifdef USE_GPS2
        readGPS2();
        currentXY2.x = GPSX(gpsvar2.longitude,startLongitude);
        currentXY2.y = GPSY(gpsvar2.latitude,startLatitude);
        robotTheta2 = ADJUST_RADIANS(DEG2RAD(gpsvar2.course));
#endif //USE_GPS2

#ifdef USE_COMPASS
        readOs5000();
#endif //USE_COMPASS
        if(currentTargetIndex <= 1 || currentTargetIndex >= maxTargets-2){
            approachingThresh=3.0;
            destinationThresh=3.0;
        }else{
            destinationThresh=1.0;
            approachingThresh=1.75;
        }
#define JUST_BECAUSE
#ifdef JUST_BECAUSE
        currentXY.x = (0.75*currentXY.x+0.25*currentXY2.x);
        currentXY.y = (0.75*currentXY.y+0.25*currentXY2.y);
        robotTheta = ADJUST_RADIANS(0.75*robotTheta+0.25*ADJUST_RADIANS(DEG2RAD(osdata.C)));
#endif

       if(guide(currentXY, targetXY, previousXY, nextXY, robotTheta, robotWidth, 1, subXY)&& !allTargetsReached){//Reached Target
            printf("REACHED TARGET\n");
            initGuide();
            previousXY = targetXY;
            currentTargetIndex = (currentTargetIndex + 1);
            if(currentTargetIndex == maxTargets){
                 allTargetsReached = 1;
            }else{
//                currentTargetIndex = (currentTargetIndex + 1)% maxTargets;
                nextTargetIndex = (currentTargetIndex + 1)% maxTargets;
                targetXY = targetListXY[currentTargetIndex];
                subXY = targetXY;
                nextXY = targetListXY[nextTargetIndex];
            }
        }
        if(reachedSubTarget){
            subXY = targetXY;
        }
        if(autoOn&&(currentTargetIndex == 0&&!approachingTarget)||allTargetsReached){
            mode =1;
            distanceMultiplier = 50;
        } else if(autoOn&&approachingTarget&&(currentTargetIndex<=1||currentTargetIndex>=maxTargets-2)){
            mode =2;
            distanceMultiplier = 50;
        } else if(autoOn&&currentTargetIndex!=0){
            mode =3;
            distanceMultiplier = 25;
        }

        /*Current Target Heading PID Control Adjustment*/
        cvar3.lookAhead = 0.00;
//        cvar3.kP = 0.25; cvar3.kI = 0.000; cvar3.kD = 0.15;
        cvar3.kP = 0.20; cvar3.kI = 0.000; cvar3.kD = 0.15;

        /*Next Target Heading PID Control Adjustment*/
        cvar4.lookAhead = 0.00;
        cvar4.kP = 0.25; cvar4.kI = 0.000; cvar4.kD = 0.005;

        if(!useSubTarget||1){//NO SUB TARGET
            turn = cvar3.turn;
        }else{
            turn = cvar4.turn;
        }


        int bestVisGpsMask = 99;
        int h = 0;
        double minVisGpsTurn = 9999;
        for(h=0;h<11;h++){
            if(fabs((cvar3.turn-turn_angle[h]))<minVisGpsTurn){
                minVisGpsTurn=fabs((cvar3.turn-turn_angle[h]));
                bestVisGpsMask = h;
            }
        }
        bestGpsMask = bestVisGpsMask;
//        printf("bvg: %d \n", bestVisGpsMask);
//        printf("vgt: %f cv3: %f\n", minVisGpsTurn,cvar3.turn);

#ifdef USE_VISION
        visionTargDist = cvar.targdist;
        double visTurnBoost = 0.50;
        if(imageProc() == -1) break;
        if(mode==1||mode==2||useVisionOnly||useVisLidar){
            turn = turn_angle[bestmask];
            turn *= visTurnBoost;
        }else if(mode==3 && fabs(turn_angle[bestmask])>0.70){
            turn = turn_angle[bestmask];
            turn *= visTurnBoost;
        }
        if(worstmask == 21 && worstmaskval > 60 && 0) dir = -1.0;//NO REVERSE
        else dir = 1.0;
        getVisionBufferAngles(0);
//        debugVisionBufferAngles();
        if(visionBufferAngles.left != 0 || visionBufferAngles.right !=0){
            if(mode==1||mode==2||useVisLidar||(mode==3 && fabs(turn_angle[bestmask])>0.70)){
                if(turn<0){
                    printf("RIGHT! %f\n",turn);
                    turn = visionBufferAngles.right;
                }else {
                    printf("LEFT! %f\n",turn);
                    turn = visionBufferAngles.left;
                }
            }
        }
#endif //USE_VISION
#ifdef USE_LIDAR
        bufferTargDist = cvar.targdist;
        updateSick();
//        findObjects();
        getBufferAngles(0);
        if(buffer_angles.left != 0 || buffer_angles.right !=0){

            if(mode == 1 || mode==2 || useVisionOnly){
                if(fabs(buffer_angles.right)==fabs(buffer_angles.left)){
                    if(turn<0.01) dir = -1.0;
                    turn = turn_angle[bestmask];
                } else if(fabs(buffer_angles.right-turn)<fabs(buffer_angles.left-turn)){
//                } else if(turn<=0){
                    turn = buffer_angles.right;
                }else {
                    turn = buffer_angles.left;
                }
            }else if(mode==3){
                if(fabs(buffer_angles.right)==fabs(buffer_angles.left)){
                    if(turn<0.01) dir = -1.0;
                    turn = turn_angle[bestmask];
                } else if(fabs(buffer_angles.right-turn)<fabs(buffer_angles.left-turn)){
//                } else if(turn<=0){
                    turn = buffer_angles.right;
                }else {
                    turn = buffer_angles.left;
                }

            }
            double tempTurn = ADJUST_RADIANS(turn+robotTheta);
            subXY.x = currentXY.x + 3.0*sin(tempTurn);
            subXY.y = currentXY.y + 3.0*cos(tempTurn);
            //turn = cvar4.turn;
        }else{
            //turn = 0.0;//if no gps
        }
#endif //USE_LIDAR

#ifdef USE_COMBINED
        combinedTargDist = cvar.targdist;
        getcombinedBufferAngles(0)
        if(combinedBufferAngles.left != 0 || combinedBufferAngles.right !=0){
            if(mode == 1 || mode==2 || mode==3){
               if(fabs(combinedBufferAngles.right-turn)<fabs(combinedBufferAngles.left-turn)){
//                } else if(turn<=0){
                    turn = combinedBufferAngles.right;
                }else {
                    turn = combinedBufferAngles.left;
                }
                if(fabs(combinedBufferAngles.right)==fabs(combinedBufferAngles.left)){
                    dir = -1.0;
                }
            }
        }
#endif //USE_COMBINED


        //turn *= dir;
        turn = SIGN(turn) * MIN(fabs(turn), 1.0);
//        speed = 1.0/(1.0+3.0*fabs(turn))*dir;
        speed = 1.0/(1.0+1.0*fabs(turn))*dir;
        speed = SIGN(speed) * MIN(fabs(speed), 1.0);
        if(dir<0){
            maxSpeed = 30;
        }else if(mode<=2||(mode==3 && fabs(turn_angle[bestmask])>0.25)){
            maxSpeed = 60 - 35*fabs(turn);
        }else{
            maxSpeed = 90 - 65*fabs(turn);
        }

        if((useVisionOnly||useVisLidar)&&!autoOn){
            maxSpeed = 60 - 40*fabs(turn);
        }else if(!autoOn){
            maxSpeed = 60;
        }
//        speed = SIGN(speed) * MIN(fabs(speed), 1.0*cvar.targdist);
        if(autoOn||useVisionOnly||useVisLidar){
            lSpeed = (speed + turnBoost*turn) * maxSpeed;
            rSpeed = (speed - turnBoost*turn) * maxSpeed;
        }
        printf("s:%.4f t: %.4f m: %d idx: %d\n", speed, turn, mode, currentTargetIndex+1);
#ifdef DEBUG_TARGET
        debugTarget();
#endif //DEBUG_TARGET
#ifdef DEBUG_GUIDE
        debugGuide();
#endif //DEBUG_GUIDE
#ifdef DEBUG_GPS
        debugGPS();
#endif //DEBUG_GPS
#ifdef DEBUG_GPS2
        debugGPS2();
#endif //DEBUG_GPS2
#ifdef DEBUG_LIDAR
        debugSICK();
#endif //DEBUG_LIDAR
#ifdef DEBUG_BUFFER
        debugBufferAngles();
#endif //DEBUG_BUFFER
#ifdef DEBUG_COMPASS
        DEBUGN(osdata.C);
#endif //DEBUG_COMPASS
#ifdef DEBUG_VISUALIZER
        robotX = currentXY.x;
        robotY = currentXY.y;
        robotTheta = robotTheta;//redundant I know....
        targetX = targetXY.x;
        targetY = targetXY.y;
//        should probably pass the above to the function...
//        paintPathPlanner(robotX,robotY,robotTheta);
//        if(getch()=='n') paintPathPlanner(2,2,0);
//        else paintPathPlanner(0,0,0);
//        showPlot();
#define MAPDELAY 10
       if(mapCount==0&&autoOn){
            double tempTheta = ADJUST_RADIANS(DEG2RAD(osdata.C));
//            mapRobot(currentXY.x,currentXY.y,tempTheta);
            mapVSICK(currentXY.x,currentXY.y,tempTheta);
#ifdef USE_LIDAR
            mapSICK(currentXY.x,currentXY.y,tempTheta);
#endif
            showMap();
            printf("MAPPING\n");
       }
            mapCount= (mapCount+1)%MAPDELAY;
#endif //VISUALIZER

        sendSpeed(lSpeed,rSpeed);
        Sleep(5);
    }

    return 0;
}

void debugTarget(){
        fprintf(stderr, "CX: %.4f CY: %.4f \n", currentXY.x, currentXY.y);
        fprintf(stderr, "TX%d: %.4f TY%d: %.4f \n", currentTargetIndex, targetXY.x, currentTargetIndex, targetXY.y);
        fprintf(stderr, "SX: %.4f SY: %.4f \n", subXY.x, subXY.y);
}
