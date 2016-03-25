#include <stdio.h>
#include <conio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#include "macros.h"
#include "serial.h"
#include "socket.h"
#include "joystick.h"
#include "control.h"
#include "roboteq.h"
#include "nmeap.h"
#include "gps.h"
#include "bufferCombined.h"
#include "sick.h"
#include "landmarks.h"
#include "objects.h"
#include "location.h"
#include "visualizer.h"
#include "vision_nav.h"

//#define FLAG_TESTING
//#define USE_NORTH

#define QUALIFYP1LAT  42.678162883
#define QUALIFYP1LON -83.195474886
#define QUALIFYP2LAT  42.677943958
#define QUALIFYP2LON -83.195503131

#define PRACTICE1LAT  42.67821411
#define PRACTICE1LON -83.19513868
#define PRACTICE2LAT  42.67810953
#define PRACTICE2LON -83.19505257
#define PRACTICE3LAT  42.67817488
#define PRACTICE3LON -83.19474585
#define PRACTICE4LAT  42.67823154
#define PRACTICE4LON -83.19473959

#define BASIC1LAT  42.67908277
#define BASIC1LON -83.19505109
#define BASIC2LAT  42.67888636
#define BASIC2LON -83.19504497

#define LATGIRTH  29428755.56476586
#define LONGIRTH  40030230.14070887

#define BASIC3LAT  ((BASIC1LAT+BASIC2LAT)/2)
#define BASIC3LON  ((BASIC1LON+BASIC2LON)/2 - 8/LATGIRTH*360)
#define BASIC4LAT  (BASIC1LAT + 2/LONGIRTH*360)
#define BASIC4LON  BASIC1LON
#define BASIC5LAT  (BASIC2LAT - 2/LONGIRTH*360)
#define BASIC5LON  BASIC2LON

#define ADVANCED1LAT  42.67931643
#define ADVANCED1LON -83.19539695
#define ADVANCED2LAT  42.67938650
#define ADVANCED2LON -83.19524132
#define ADVANCED3LAT  42.67932379
#define ADVANCED3LON -83.19512757
#define ADVANCED4LAT  42.67924434
#define ADVANCED4LON -83.19494908
#define ADVANCED5LAT  42.67962112
#define ADVANCED5LON -83.19495897
#define ADVANCED6LAT  42.67953808
#define ADVANCED6LON -83.19513459
#define ADVANCED7LAT  42.67947513
#define ADVANCED7LON -83.19524827
#define ADVANCED8LAT  42.67954487
#define ADVANCED8LON -83.19540665

#define ADVANCED9LAT  (ADVANCED4LAT - 2/LONGIRTH*360)
#define ADVANCED9LON  (ADVANCED4LON - 1/LATGIRTH*360)
#define ADVANCED10LAT  (ADVANCED5LAT + 2/LONGIRTH*360)
#define ADVANCED10LON  (ADVANCED5LON - 1/LATGIRTH*360)
#define ADVANCED11LAT  ((ADVANCED6LAT + ADVANCED7LAT)/2)
#define ADVANCED11LON  (ADVANCED5LON - 5/LATGIRTH*360)
#define ADVANCED12LAT  ((ADVANCED2LAT + ADVANCED3LAT)/2)
#define ADVANCED12LON  (ADVANCED4LON - 5/LATGIRTH*360)

#define TESTLAT 42.678482
#define TESTLON -83.194895

#define TARGET_LIMIT 120



#define LANEQUALIFICATION 0
#define FIELDQUALIFICATION 1

/*SENSOR OPTIONS*/
#define USE_VISION
//#define USE_LIDAR
//#define USE_GPS
//#define USE_COMBINED_BUFFER
//#define USE_MAP

/*CONTROL OPTIONS*/
//#define REVFRAMES 25
#define REVFRAMES 2,0
#define SWAPTIME 20.0 //in seconds
//#define AUTO_SWAP

#define OPEN_FIELD_INDEX 0
#define END_LANE_INDEX 1

#define MAPDELAY 10
#define CLEARMAPDELAY 5
#define DUMPGPSDELAY 20
/*DEBUG OPTIONS*/
//#define DUMP_GPS
//#define DEBUG_TARGET
//#define DEBUG_GUIDE
//#define DEBUG_GPS
//#define DEBUG_LIDAR
//#define DEBUG_VISUALIZER
#define DEBUG_BUFFER
#define DEBUG_MAIN

/*joystick.h Definitions*/
struct JoystickData joy0;

/*roboteq.h Definitions*/
//int roboteqPort = 12;
int roboteqPort = 7;

/*control.h Definitions*/
CVAR cvar;
double destinationThresh = 0.75;
double approachingThresh = 1.75;
double leaveTargetThresh = 1.75;
double maxTurn = 1.0;
int approachingTarget;
int inLastTarget;

/*gps.h Definitions*/
GPSVAR gpsvar;
GPSPNT targetsGPS[4];
double startLatitude = ADVANCED1LAT;//Center Island Latitude
double startLongitude = ADVANCED1LON;//Center Island Longitude

/*objects.h Definitions*/
int SEPARATION_THRESH=750;
int NONSEPARATION_THRESH=350;
int HIGH_THRESH=100;
int LOW_THRESH=3;
int MAX_OBJECTS=50;



/*landmarks.h, visualizer.h*/
double robotX = 0.0, robotY = 0.0, robotTheta = 0.0;
double targetX, targetY;
double landmark_tolerance = 500.0;
double fieldWidth = 60.0, fieldLength = 60.0;

/*main Definitions*/
void debugTarget();//Debug Target function
double robotWidth = 0.7112;
int maxSpeed = 50;
int currentTargetIndex = 0, nextTargetIndex = 0, maxTargets = 0, allTargetsReached = 0;
int autoOn = 0, maxTargetIndex = 0;
int mode = 0;
XY targetListXY[100];
XY currentXY, targetXY, previousXY, nextXY;
XY swapXY;
/*FLAG STUFF*/
#define FLAG_DIST_THRESH 12.5
#define FLAG_X_ADJUST 4
XY flagXY;
int flagPointSet=0;
double flagPointDistance = 0.0;

/*MODE2 TIMER*/
#define MODE2DELAY 45.0
double startAutoTime=0.0, currentAutoTime=0.0, totalAutoTime=0.0;
int mode1TimeUp=0;

double startTime=0.0, currentTime=0.0, totalTime = 0.0;
int lastButtons, targetIndexMem = 0;

void swap(){
    if(currentTargetIndex>OPEN_FIELD_INDEX&&nextTargetIndex<maxTargetIndex-END_LANE_INDEX){
        targetXY=targetListXY[nextTargetIndex];
        swapXY=targetListXY[currentTargetIndex];
        nextXY=targetListXY[nextTargetIndex]=swapXY;
        targetListXY[currentTargetIndex]=targetXY;

    } else if(currentTargetIndex>OPEN_FIELD_INDEX&&nextTargetIndex<maxTargetIndex){
        previousXY = targetXY;
        currentTargetIndex = (currentTargetIndex + 1);
        nextTargetIndex = (currentTargetIndex + 1)% maxTargets;
        targetXY = targetListXY[currentTargetIndex];
        nextXY = targetListXY[nextTargetIndex];
    }
    initGuide();
}


int main(){
    int i = 0;
    int mapCount = 0, clearMapCount = 0, dumpCount=0;
    int revFrameCount = 0;

#ifdef USE_NORTH
    targetsGPS[maxTargets].lat = ADVANCED5LAT;
    targetsGPS[maxTargets].lon = ADVANCED5LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED6LAT;
    targetsGPS[maxTargets].lon = ADVANCED6LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED7LAT;
    targetsGPS[maxTargets].lon = ADVANCED7LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED8LAT;
    targetsGPS[maxTargets].lon = ADVANCED8LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED2LAT;
    targetsGPS[maxTargets].lon = ADVANCED2LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED1LAT;
    targetsGPS[maxTargets].lon = ADVANCED1LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED3LAT;
    targetsGPS[maxTargets].lon = ADVANCED3LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED12LAT;
    targetsGPS[maxTargets].lon = ADVANCED12LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED4LAT;
    targetsGPS[maxTargets].lon = ADVANCED4LON;
    maxTargets++;
#else
    targetsGPS[maxTargets].lat = ADVANCED4LAT;
    targetsGPS[maxTargets].lon = ADVANCED4LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED1LAT;
    targetsGPS[maxTargets].lon = ADVANCED1LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED2LAT;
    targetsGPS[maxTargets].lon = ADVANCED2LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED3LAT;
    targetsGPS[maxTargets].lon = ADVANCED3LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED11LAT;
    targetsGPS[maxTargets].lon = ADVANCED11LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED8LAT;
    targetsGPS[maxTargets].lon = ADVANCED8LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED7LAT;
    targetsGPS[maxTargets].lon = ADVANCED7LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED6LAT;
    targetsGPS[maxTargets].lon = ADVANCED6LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED11LAT;
    targetsGPS[maxTargets].lon = ADVANCED11LON;
    maxTargets++;
    targetsGPS[maxTargets].lat = ADVANCED5LAT;
    targetsGPS[maxTargets].lon = ADVANCED5LON;
    maxTargets++;
#endif

    maxTargetIndex=maxTargets-1;

    for(i=0;i<maxTargets;i++){
        targetListXY[i].x = GPSX(targetsGPS[i].lon, startLongitude);
        targetListXY[i].y = GPSY(targetsGPS[i].lat, startLatitude);
    }
    currentXY.x = GPSX(gpsvar.longitude,startLongitude);
    currentXY.y = GPSY(gpsvar.latitude,startLatitude);

    targetXY = targetListXY[currentTargetIndex];
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
#ifdef USE_LIDAR
    initObjects();
    initSICK();
#endif //USE_LIDAR
#ifdef DEBUG_VISUALIZER
    initVisualizer();
#endif //DEBUG_VISUALIZER
#ifdef USE_MAP
    initMap(0,0,0);
#endif //USE_MAP
#ifdef DUMP_GPS
    FILE *fp;
    fp = fopen("gpsdump.txt", "w");
#endif // DUMP_GPS
    while(1){
        double dir = 1.0;
        double speed = 0.0, turn = 0.0;
        static double turnBoost = 0.750;//Multiplier for turn. Adjust to smooth jerky motions. Usually < 1.0
        static int lSpeed = 0, rSpeed = 0;//Wheel Speed Variables
        if (joystick() != 0) {
            if (joy0.buttons & LB_BTN) {
                speed = -joy0.axis[1]; //Up is negative on joystick negate so positive when going forward
                turn = joy0.axis[0];

                lSpeed = (int)((speed + turnBoost*turn)*maxSpeed);
                rSpeed = (int)((speed - turnBoost*turn)*maxSpeed);
                }else{
                     rSpeed=lSpeed=0;
            }
            if(((joy0.buttons & B_BTN)||autoOn)&& (saveImage==0)){
                saveImage =DEBOUNCE_FOR_SAVE_IMAGE;
            }else{
                if (saveImage) saveImage--;
            }
            if(joy0.buttons & RB_BTN){
                autoOn = 1;
                mode=1;
            }
            if(joy0.buttons & Y_BTN){
                autoOn = 0;
                mode =0;
            }
            lastButtons = joy0.buttons;
        } else{
//            printf("No Joystick Found!\n");
            rSpeed=lSpeed=0;
        }
//
//        printf("3: %f %f\n",BASIC3LAT,BASIC3LON);
//        printf("4: %f %f\n",BASIC4LAT,BASIC4LON);
//        printf("5: %f %f\n",BASIC5LAT,BASIC5LON);
//        getchar();
#ifdef AUTO_SWAP
        if((currentTargetIndex>1&&targetIndexMem!=currentTargetIndex)||!autoOn||!mode==3){
            startTime=currentTime=(float)(clock()/CLOCKS_PER_SEC);
            targetIndexMem = currentTargetIndex;
        }else{
            currentTime=(float)(clock()/CLOCKS_PER_SEC);
        }
        totalTime = currentTime-startTime;
        if(totalTime>=SWAPTIME&&autoOn){
            swap();
            targetIndexMem = 0;
        }
#endif //AUTO_SWAP

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

        if(autoOn&&!flagPointSet){
            flagXY.x=currentXY.x+FLAG_X_ADJUST;
            flagXY.y=currentXY.y;
            flagPointSet=1;
            startAutoTime=currentAutoTime=(float)(clock()/CLOCKS_PER_SEC);
        }
        if(autoOn){
            currentAutoTime=(float)(clock()/CLOCKS_PER_SEC);
            totalAutoTime = currentAutoTime-startAutoTime;
            if(totalAutoTime>=MODE2DELAY){
                mode1TimeUp=1;
            }
            printf("TIMEING\n");
        }

//        if(currentTargetIndex <= OPEN_FIELD_INDEX || currentTargetIndex >= maxTargetIndex){
        if(currentTargetIndex <= OPEN_FIELD_INDEX){
            approachingThresh=4.0;
            destinationThresh=3.0;
        }else{
//            destinationThresh=1.0;
            destinationThresh=0.75;
            approachingThresh=2.5;
        }


       if(guide(currentXY, targetXY, previousXY, nextXY, robotTheta, robotWidth, 1)&& !allTargetsReached){//Reached Target
            printf("REACHED TARGET\n");
            initGuide();
            previousXY = targetXY;
            if(currentTargetIndex == maxTargetIndex){
                 allTargetsReached = 1;
            }else{
                currentTargetIndex = (currentTargetIndex + 1);
                nextTargetIndex = (currentTargetIndex + 1)% maxTargets;
                targetXY = targetListXY[currentTargetIndex];
                nextXY = targetListXY[nextTargetIndex];
            }
        }
        if((autoOn&&(currentTargetIndex == 0&&!approachingTarget&&!mode1TimeUp))||allTargetsReached){
            mode =1;
            distanceMultiplier = 50;
        } else if((autoOn&&currentTargetIndex == 0&&mode1TimeUp)||(autoOn&&approachingTarget&&(currentTargetIndex<=OPEN_FIELD_INDEX||currentTargetIndex>=maxTargetIndex-END_LANE_INDEX))){
            mode =2;
            distanceMultiplier = 50;
        } else if((autoOn&&currentTargetIndex!=0)){
            mode =3;
            distanceMultiplier = 12;
        }
        flagPointDistance = D((currentXY.x-flagXY.x),(currentXY.y-flagXY.y));
        if(allTargetsReached&&flagPointDistance<FLAG_DIST_THRESH){
            mode =4;
        }
#ifdef FLAG_TESTING
        /*FLAG TESTING*/
        mode=4;
#endif //FLAG_TESTING

        /*Current Target Heading PID Control Adjustment*/
        cvar.lookAhead = 0.00;
        cvar.kP = 0.20; cvar.kI = 0.000; cvar.kD = 0.15;

        turn = cvar.turn;


        int bestVisGpsMask = 99;
        int h = 0;
        double minVisGpsTurn = 9999;
        for(h=0;h<11;h++){
            if(fabs((cvar.turn-turn_angle[h]))<minVisGpsTurn){
                minVisGpsTurn=fabs((cvar.turn-turn_angle[h]));
                bestVisGpsMask = h;
            }
        }
        bestGpsMask = bestVisGpsMask;
//        printf("bvg: %d \n", bestVisGpsMask);
//        printf("vgt: %f cv3: %f\n", minVisGpsTurn,cvar3.turn);

#ifdef USE_VISION
//        double visTurnBoost = 0.50;
        double visTurnBoost = 1.0;
        if(imageProc(mode) == -1) break;
        if(mode==1||mode==2){
            turn = turn_angle[bestmask];
            turn *= visTurnBoost;
        }else if(mode==3 && fabs(turn_angle[bestmask])>0.70){
            turn = turn_angle[bestmask];
            turn *= visTurnBoost;
        }
#endif //USE_VISION
#ifdef USE_LIDAR
        updateSick();
//        findObjects();
#endif //USE_LIDAR

#ifdef USE_COMBINED_BUFFER
#define WORSTTHRESH 10
#define BESTTHRESH 3
        if(mode==4){
#ifdef USE_NORTH
            turn = (0.5*turn_angle[bestBlueMask]+0.5*turn_angle[bestRedMask]);
#else
            turn = (0.65*turn_angle[bestBlueMask]+0.35*turn_angle[bestRedMask]);
#endif
            turn *= 0.75;
        }
        combinedTargDist = cvar.targdist;
        if(((approachingTarget||inLastTarget)&&currentTargetIndex>OPEN_FIELD_INDEX
            &&currentTargetIndex<maxTargetIndex-END_LANE_INDEX)||(MAG(howbad[worstmask]-howbad[bestmask]))<BESTTHRESH||mode==4){
            getCombinedBufferAngles(0,0);//Don't Use Vision Radar Data
        }else{
            getCombinedBufferAngles(0,1);//Use Vision Radar Data
        }
        if(combinedBufferAngles.left != 0 || combinedBufferAngles.right !=0){
            if(mode == 1 || mode==2 || mode==3 || mode==4){
//            if(mode == 1 || mode==2 || mode==3){
//            if(mode==2 || mode==3){
//            if(mode==3){
                if(fabs(combinedBufferAngles.right)==fabs(combinedBufferAngles.left)){
                    double revTurn;
                    double revDistLeft, revDistRight;
                    int revIdx;
                    if(fabs(turn)<0.10) dir = -1.0;
                    if(fabs(combinedBufferAngles.left)>1.25) dir = -1.0;
                    if(dir<0){
                        revIdx = 540-RAD2DEG(combinedBufferAngles.left)*4;
                        revIdx = MIN(revIdx,1080);
                        revIdx = MAX(revIdx,0);
                        revDistLeft = LMSdata[revIdx];

                        revIdx = 540-RAD2DEG(combinedBufferAngles.right)*4;
                        revIdx = MIN(revIdx,1080);
                        revIdx = MAX(revIdx,0);
                        revDistRight = LMSdata[revIdx];
                        if(revDistLeft>=revDistRight){
                            revTurn = combinedBufferAngles.left;
                        }else {
                            revTurn = combinedBufferAngles.right;
                        }
                        turn = revTurn;
                    }else{
                        turn = turn_angle[bestmask];
                    }
                } else if(fabs(combinedBufferAngles.right-turn)<fabs(combinedBufferAngles.left-turn)){
//                } else if(turn<=0){
                    turn = combinedBufferAngles.right;
                }else {
                    turn = combinedBufferAngles.left;
                }
            }
        }
#endif //USE_COMBINED_BUFFER
        if(dir<0||revFrameCount!=0){
            dir = -1.0;
            revFrameCount = (revFrameCount+1)%REVFRAMES;
        }
        //        turn *= dir;
        turn = SIGN(turn) * MIN(fabs(turn), 1.0);
        speed = 1.0/(1.0+1.0*fabs(turn))*dir;
        speed = SIGN(speed) * MIN(fabs(speed), 1.0);
        if(!autoOn){
            maxSpeed = 60;
            targetIndexMem = 0;
        }else if(dir<0){
            maxSpeed = 30;
        }else if(mode<=2||(mode==3 && fabs(turn_angle[bestmask])>0.25)){
            maxSpeed = 60 - 25*fabs(turn);
//            maxSpeed = 70 - 35*fabs(turn);
//            maxSpeed = 90 - 50*fabs(turn);
//            maxSpeed = 100 - 65*fabs(turn);
        }else if(mode==4){
            maxSpeed = 45-20*fabs(turn);
        }else{
            maxSpeed = 85 - 50*fabs(turn);
//            maxSpeed = 100 - 65*fabs(turn);
//            maxSpeed = 110 - 70*fabs(turn);
//            maxSpeed = 120 - 85*fabs(turn);
        }
        if(autoOn){
            lSpeed = (speed + turnBoost*turn) * maxSpeed;
            rSpeed = (speed - turnBoost*turn) * maxSpeed;
        }
#ifdef DEBUG_MAIN
        printf("s:%.4f t: %.4f m: %d vt:%f dir:%f tmr: %f\n", speed, turn, mode, turn_angle[bestmask], flagPointDistance, totalAutoTime);
#endif //DEBUG_MAIN
#ifdef DUMP_GPS
    if(dumpCount==0){
        if (fp != NULL) {
                fprintf(fp, "%f %f %f %f %f\n",gpsvar.latitude,gpsvar.longitude, gpsvar.course, gpsvar.speed, gpsvar.time);
            }
    }
        dumpCount = dumpCount+1%DUMPGPSDELAY;

#endif //DUMP_GPS
#ifdef DEBUG_TARGET
        debugTarget();
#endif //DEBUG_TARGET
#ifdef DEBUG_GUIDE
        debugGuide();
#endif //DEBUG_GUIDE
#ifdef DEBUG_GPS
        debugGPS();
#endif //DEBUG_GPS
#ifdef DEBUG_LIDAR
        debugSICK();
#endif //DEBUG_LIDAR
#ifdef DEBUG_BUFFER
        debugCombinedBufferAngles();
#endif //DEBUG_BUFFE
#ifdef DEBUG_VISUALIZER
        robotX = currentXY.x;
        robotY = currentXY.y;
        robotTheta = robotTheta;//redundant I know....
        targetX = targetXY.x;
        targetY = targetXY.y;
//        should probably pass the above to the function...
        paintPathPlanner(robotX,robotY,robotTheta);
        showPlot();
#endif //VISUALIZER

#ifdef USE_MAP
       if(mapCount==0){
//            mapRobot(currentXY.x,currentXY.y,robotTheta);
            if(clearMapCount==0) clearMapSection(currentXY.x,currentXY.y,robotTheta);
            else clearMapCount = (clearMapCount+1)%CLEARMAPDELAY;
            mapVSICK(currentXY.x,currentXY.y,robotTheta);
//            mapVSICK(0,0,0);
#ifdef USE_LIDAR
            mapSICK(currentXY.x,currentXY.y,robotTheta);
#endif
            showMap();
//            printf("MAPPING\n");
       }
            mapCount= (mapCount+1)%MAPDELAY;

#endif //USE_MAP
        sendSpeed(lSpeed,rSpeed);
        Sleep(5);
    }
#ifdef DUMP_GPS
    fclose(fp);
#endif
    return 0;
}

void debugTarget(){
        fprintf(stderr, "CX: %.4f CY: %.4f Dist:%.4f\n", currentXY.x, currentXY.y, D((targetXY.x-currentXY.x),(targetXY.y-currentXY.y)));
        fprintf(stderr, "TX%d: %.4f TY%d: %.4f \n", currentTargetIndex, targetXY.x, currentTargetIndex, targetXY.y);
        fprintf(stderr, "NX%d: %.4f NY%d: %.4f \n", nextTargetIndex, nextXY.x, nextTargetIndex, nextXY.y);
}
