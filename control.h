#ifndef __CONTROL_H__
#define __CONTROL_H__

#define DEBUGCVAR

#ifndef _CVAR_
#define _CVAR_
typedef struct{
    double targdist;
    double targbearing;
    double front;
    double right;
    double speed;               // between -1 to 1
    double turn;                // between -1 to 1
    double pErr;
    double lastpErr;
    double kP;
    double dErr;
    double kD;
    double iErr;
    double kI;
    double lookAhead;
}CVAR;
#endif //_CVAR_

extern CVAR cvar;

extern double destinationThresh, approachingThresh, leaveTargetThresh;
extern double maxTurn;

extern int approachingTarget, inLastTarget;
void initGuide();
int guide(XY robot, XY currentTarget, XY lastTarget, XY nextTarget, double heading,  double rWidth, int dir);
void debugGuide();

#endif// __CONTROL_H__
