#ifndef OBJECTS_C
#define OBJECTS_C

#include <math.h>
#include <stdlib.h>

#include "objects.h"
#include "macros.h"
#include "sick.h"
#include "landmarks.h"

int DIFF(int x, int y)
{
    int c = x - y;
    return MAG(c);
}

struct State_Cab {
    int state;
    int linked;
    int sum;
    int base;
} *cab;

void clearState()
{
    free(cab);
    cab = (struct State_Cab *) malloc(sizeof(struct State_Cab));

    cab->state = ZERO;
    cab->sum = 0;
    cab->linked = 0;
    cab->base = 0;
}

void linkPoint(int i, short dist)
{
    cab->linked++;
    cab->sum += dist;
    cab->base = dist;
}

void updateState(int NEXT)
{
    cab->state = NEXT;
}

void addObject(int z)
{
    double mag = (double) cab->sum / (double) cab->linked;
    int index = z - cab->linked / 2;    // -1 to account for point not found on index

    if (mag > MAX_RADIUS || cab->linked > HIGH_THRESH
        || cab->linked < LOW_THRESH) {
        clearState();
    } else {
//
        Objects[num_objects].x = POLAR2X(index, mag);
        Objects[num_objects].y = POLAR2Y(index, mag);
        Objects[num_objects].dist =
            D(Objects[num_objects].x, Objects[num_objects].y);
        Objects[num_objects].phi =
            ADJUST_RADIANS(atan2
                           (Objects[num_objects].x,
                            Objects[num_objects].y));
        Objects[num_objects].theta =
            ADJUST_RADIANS(Objects[num_objects].phi);

        num_objects++;
    }
}

void clearObjects()
{
    int i;
    for (i = 0; i < MAX_OBJECTS; i++) {
        Objects[i].x = 0;
        Objects[i].y = 0;
        Objects[i].phi = 0;
        Objects[i].dist = 0;
    }
    num_objects = 0;
}


void check_previous(){
    int i;
    for (i = 0; i < MAX_OBJECTS; i++) {

    }
}

void removeObject(int i){
    for(;i<num_objects;i++){
        Objects[i] = Objects[i+1];
    }
    Objects[i].x = 0;
    Objects[i].y = 0;
    Objects[i].phi = 0;
    Objects[i].dist = 0;
    num_objects--;
}

void combine2Objects(int i, int j){
//    printf("COMBINING\n");
    Objects[i].x = (Objects[i].x+Objects[j].x)/2;
    Objects[i].y = (Objects[i].y+Objects[j].y)/2;
    removeObject(j);
}

void combineObjects(){
    int i;
    double dx, dy;

    for (i = 0; i < num_objects-1;) {
        dx = Objects[i].x-Objects[i+1].x;
        dy = Objects[i].y-Objects[i+1].y;
//        printf("OBJECTS D: %f\n", D(dx,dy));
        if(D(dx,dy)<SEPARATION_THRESH){
            combine2Objects(i,i+1);
            i=0;
        }else{
            i++;
        }
    }
}

void findObjects()
{
    int i;
    //BEGIN STATE MACHINE
    //check_previous();
    clearObjects();
    clearState();
    for (i = 0; i < SICK_NUMPTS - 1; i++) {
        //printf("Mag: %i State: %i i: %i\n",array[i],cab->state,i);
        short d1 = LMSdata[i];
        short d2 = LMSdata[i+1];

        if (cab->state == ZERO) {
            if (d1 <= MAX_RADIUS) {
                if (MAG(d1 - d2) <
                    NONSEPARATION_THRESH) {
                    linkPoint(i, d2);
                    updateState(ONE);
                } else {
                    clearState();
                }
            }
        } else if (cab->state == ONE) {
            if (DIFF(cab->base, d2) <
                NONSEPARATION_THRESH) {
                linkPoint(i, d1);
                updateState(TWO);
            } else {
                updateState(TWOA);
            }
        } else if (cab->state == TWO) {
            if (DIFF(cab->base, d2) <
                NONSEPARATION_THRESH) {
                linkPoint(i, d1);
                updateState(THREE);
            } else {
                updateState(TWOA);
            }
        }else if (cab->state == TWOA) {
            if (DIFF(cab->base, d2) <
                NONSEPARATION_THRESH) {
                linkPoint(i, d1);
                updateState(TWO);
            } else if (cab->linked < LOW_THRESH) {
                clearState();
            } else {
                addObject(i);
                clearState();
            }
        } else if (cab->state == THREE) {
            if (DIFF(cab->base, d2) <
                NONSEPARATION_THRESH) {
                linkPoint(i, d1);
                updateState(THREE);
            } else {
                updateState(TWOA);
            }
        }
    }
    combineObjects();
}

void initObjects()
{
    free(Objects);
    free(Prev_Objects);
    free(Vis_Objects);
    Prev_Objects = (struct Object *) malloc(MAX_OBJECTS * sizeof(struct Object));
    Vis_Objects = (struct Object *) malloc(MAX_OBJECTS * sizeof(struct Object));
    Objects = (struct Object *) malloc(MAX_OBJECTS * sizeof(struct Object));
    num_objects = 0;
}
#endif
