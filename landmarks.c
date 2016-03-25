#ifndef LANDMARKS_C
#define LANDMARKS_C

#include <stdio.h>
#include <math.h>

#include "landmarks.h"
#include "objects.h"
#include "macros.h"
#include "sick.h"

extern double fieldLength, fieldWidth;

void known_landmarks_init(char *fname){
    float tempx, tempy;
    int u = 0;
    FILE *fp;
    if ((fp = fopen(fname, "r")) == NULL) {
        fprintf(stderr, "Could not open landmark file\n");
    }
    num_landmarks = 0;
    while (!feof(fp)) {
        fscanf(fp, "%f %f", &tempx, &tempy);
        KLM[u].x = tempx*M2MM;
        KLM[u].y = tempy*M2MM;
        KLM[u].found = false;
        KLM[u].rejected = false;
        KLM[u].phi = atan2(KLM[u].x, KLM[u].y);
        KLM[u].dist=D(KLM[u].x, KLM[u].y);
        u++;
    }
    num_landmarks = u;
    fclose(fp);
}

int objBound(float x, float y)
{
    if (fabs(x) < (fieldWidth * M2MM / 2)) {
        if (y < fieldLength * M2MM) {
            if (y > -3.0 * M2MM) {
                return 1;
            }
        }
    }
    return 0;
}

void obj2landmarks()
{
    num_landmarks = 0;
    int i;
    double temp = 99999999;
    for (i = 0; i < num_objects; i++) {
        if (objBound(Objects[i].x, Objects[i].y)) {
            KLM[num_landmarks].x =
                robotX * M2MM + Objects[i].x;//Objects[i].dist * sin(Objects[i].phi + robotTheta);
            KLM[num_landmarks].y =
                robotY * M2MM + Objects[i].y;//Objects[i].dist * cos(Objects[i].phi + robotTheta);
            KLM[num_landmarks].found = false;
            KLM[num_landmarks].rejected = false;
            KLM[num_landmarks].phi =
                atan2(KLM[num_landmarks].x,
                      KLM[num_landmarks].y);
            KLM[num_landmarks].dist=D(KLM[num_landmarks].x, KLM[num_landmarks].y);
            temp = MIN(temp, fabs(KLM[num_landmarks].x) - 0);
            num_landmarks++;
        }
    }
}


bool scan_landmark(int i)
{
    int k;
    int found_points = 0;
    double lmdist = 0, lmtheta = 0;
    double px,py,dx,dy,separation;
    short d;

    if(KLM[i].rejected){
        return false;
    }

    for (k = 0; k < SICK_NUMPTS; k++) {
        d = LMSdata[k];
        px = robotX*M2MM+d*sin(DEG2RAD(135-k*RESOLUTION)+robotTheta);
        py = robotY*M2MM+d*cos(DEG2RAD(135-k*RESOLUTION)+robotTheta);
        dx = KLM[i].x - px;
        dy = KLM[i].y - py;
        separation = D(dx, dy);
        if (separation < landmark_tolerance) {
            lmdist += d;
            lmtheta += 135-k*RESOLUTION;
            found_points++;
        }
    }

    if (found_points > LM_POINTS_THRESH) {
        lmdist = lmdist / (double) found_points;
        lmtheta = lmtheta / (double) found_points;

        if ((lmdist < MAX_LANDMARKS_DISTANCE) && landmarks_seen < MAX_LANDMARKS_LOADED) {
            KLM[i].found = true;
            CLM[landmarks_seen].x = KLM[i].x * MM2M;
            CLM[landmarks_seen].y = KLM[i].y * MM2M;
            CLM[landmarks_seen].dist = 0.4*lmdist*MM2M;
            CLM[landmarks_seen].dist += 0.6*D((robotX-CLM[landmarks_seen].x), (robotY-CLM[landmarks_seen].y));
            CLM[landmarks_seen].phi = DEG2RAD(lmtheta);
            landmarks_seen++;
            return true;
        }
    }
    KLM[i].found = false;
    return false;
}

void scan_landmarks()
{
    int i;
    landmarks_seen = 0;
    for (i = 0; i < num_landmarks; i++){
            scan_landmark(i);
    }
}
#endif
