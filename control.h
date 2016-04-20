#ifndef __CONTROL_H__
#define __CONTROL_H__

#define DEBUGCVAR

#ifndef _CVAR_
#define _CVAR_
typedef struct{
    double targdist;//distance to current target
    double targbearing;
    double front;//distance to the target on the right
    double right;//How far target is to the right of robot
    double speed;               // between -1 to 1
    double turn;                // between -1 to 1
    /* USED FOR PID CONTROL OF HEADING , look it up on wiki for more details*/
    // where e(t) is current difference between desired heading angle, and current heading
    //overall PID control equation is u(t)=pErr+dErr+iErr
    double kP;//constant gain for proportion term
    double pErr;//proportional error, AKA Kp*e(t),
    double lastpErr;//last proportional error, AKA Last pErr Value

    double kD;//Derivative gain
    double dErr;//derivative error, d/dt(e(t)). IE how quickly heading is changing

    double kI;//Integral Gain
    double iErr;//integration error, this is the sum of e(t) over time
    //=0 at the start, want it to reduce to 0.  this is heading control

    double lookAhead;//?
}CVAR;
#endif //_CVAR_

extern CVAR cvar;

extern double destinationThresh, approachingThresh, leaveTargetThresh;
extern double maxTurn;

extern int approachingTarget, inLastTarget;
void initGuide();
int guide(XY robot, XY currentTarget, XY lastTarget, XY nextTarget, double heading,  double rWidth, int dir);//updates all control variables
void debugGuide();

#endif// __CONTROL_H__
