#ifndef LOCATION_C
#define LOCATION_C

#include <math.h>
#include <stdio.h>

#include "landmarks.h"
#include "macros.h"
#include "location.h"
#define JMAX 5                //Set to maximum number of iterations.

double mu = 0.1, px=0.0, pt=0.0, py=0.0, maxx=0.25, minx=0.0001, maxy=0.25, miny=0.0001, maxt=DEG2RAD(5), mint=DEG2RAD(0.1);
void locate(int num_lm, double tolerance, double *rx, double *ry, double *rt)
{
    int i,j,k, updateh;
    double H[3][3], H_lm[3][3], H_inv[3][3], x_err, y_err, t_err, ex, ey, dx, dy, dt, det;
    double tx=*rx, ty=*ry, tt=*rt;
    double errsqrd=0.0,derr=0.0,lasterrsqrd=0.0,lambda=10.0;
/*Set Hessian To Zero*/
    for(i=0;i<3;i++){
        for(j=0;j<3;j++){
            H[i][j]=H_lm[i][j]=H_inv[i][j]=0;
        }
    }

/*Initalize Diag*/
    for (j=0; j < JMAX; j++) {
        errsqrd=0;
        derr=0;
        if(j==0){
            updateh=1;
            for(i=0;i<num_lm;i++){
                ex = CLM[i].x-tx-CLM[i].dist*sin(tt+CLM[i].phi);
                ey = CLM[i].y-ty-CLM[i].dist*cos(tt+CLM[i].phi);
                lasterrsqrd=pow(ex,2)+pow(ey, 2);
            }
        }
        if(updateh){
            H[0][0]=H[1][1]=H[2][2]= mu*num_lm;//
            x_err=mu*num_lm*(px-tx);
            y_err=mu*num_lm*(py-ty);
            t_err=mu*num_lm*(pt-tt);
            for(i=0;i<num_lm;i++) {
                /*First Derivative*/
                ex = CLM[i].x-tx-CLM[i].dist*sin(tt+CLM[i].phi);
                ey = CLM[i].y-ty-CLM[i].dist*cos(tt+CLM[i].phi);
                derr+=ex+ey;
                errsqrd+=pow(ex,2)+pow(ey, 2);
                x_err += ex;
                y_err += ey;
                t_err += ex*(CLM[i].dist*cos(tt+CLM[i].phi))-ey*(CLM[i].dist*sin(tt+CLM[i].phi));

                H[0][0] += 1;
                H[0][1] += 0;
                H[0][2] +=(CLM[i].dist*cos(tt + CLM[i].phi));
                H[1][0] += 0;
                H[1][1] += 1;
                H[1][2] += -(CLM[i].dist * sin(tt + CLM[i].phi));
                H[2][0] += (CLM[i].dist * cos(tt + CLM[i].phi));
                H[2][1] += -(CLM[i].dist * sin(tt + CLM[i].phi));
                H[2][2] += pow(CLM[i].dist,2);

            }
        }
        for(i=0;i<3;i++){
            for(k=0;k<3;k++){
                H_lm[i][k]=H[i][k];
                if(i==k)H_lm[i][k]+=lambda*1.0;
            }
        }

        det = H_lm[0][0]*(H_lm[2][2]*H_lm[1][1]-H_lm[2][1]*H_lm[1][2]) -
                H_lm[1][0]*(H_lm[2][2]*H_lm[0][1]-H_lm[2][1]*H_lm[0][2]) +
                H_lm[2][0]*(H_lm[1][2]*H_lm[0][1]-H_lm[1][1]*H_lm[0][2]);

        H_inv[0][0] = (H_lm[2][2]*H_lm[1][1]-H_lm[2][1]*H_lm[1][2])/ det;
        H_inv[0][1] = -(H_lm[2][2]*H_lm[0][1]-H_lm[2][1]*H_lm[0][2])/ det;
        H_inv[0][2] = (H_lm[1][2]*H_lm[0][1]-H_lm[1][1]*H_lm[0][2]) /det;
        H_inv[1][0] = -(H_lm[2][2]*H_lm[1][0]-H_lm[2][0]*H_lm[1][2])/ det;
        H_inv[1][1] = (H_lm[2][2]*H_lm[0][0]-H_lm[2][0]*H_lm[0][2])/ det;
        H_inv[1][2] = -(H_lm[1][2]*H_lm[0][0]-H_lm[1][0]*H_lm[0][2])/ det;
        H_inv[2][0] = (H_lm[2][1]*H_lm[1][0]-H_lm[2][0]*H_lm[1][1])/ det;
        H_inv[2][1] = -(H_lm[2][1]*H_lm[0][0]-H_lm[2][0]*H_lm[0][1])/ det;
        H_inv[2][2] = (H_lm[1][1]*H_lm[0][0]-H_lm[1][0]*H_lm[0][1])/ det;


  /* Update Here*/
  /* Hessian Inverse times Gradiant*/
        dx = (H_inv[0][0]*x_err)+(H_inv[0][1]*y_err)+(H_inv[0][2]*t_err);
        dy = (H_inv[1][0]*x_err)+(H_inv[1][1]*y_err)+(H_inv[1][2]*t_err);
        dt = (H_inv[2][0]*x_err)+(H_inv[2][1]*y_err)+(H_inv[2][2]*t_err);


        if(fabs(derr)<tolerance){
            j=JMAX;
        }
        if(errsqrd<lasterrsqrd){
            updateh=1;
            lambda/=10;
            lasterrsqrd=errsqrd;
            tx += dx;
            ty += dy;
            tt += dt;
        }else{
            updateh=0;
            lambda*=10;
        }
    }
        if(fabs(tx-px)<maxx&&fabs(ty-py)<maxy&&fabs(tt-pt)<maxt){
            if(fabs(tx-px)>minx) *rx = px = tx;
            if(fabs(ty-py)>miny) *ry= py = ty;
            if(fabs(tt-pt)>mint) *rt = pt = ADJUST_RADIANS(tt);
        }

}

#endif
