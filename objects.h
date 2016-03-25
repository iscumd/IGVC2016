#ifndef OBJECTS_H
#define OBJECTS_H

#define ZERO  0
#define ONE   1
#define TWO   2
#define TWOA  3
#define TWOB  4
#define TWOC  5
#define TWOD  6
#define TWOE  7
#define TWOF  8
#define THREE 9

#define OBJECT_COLOR YELLOW
#define OBJECT_COLOR_2 BLACK

#define LABEL_COLOR YELLOW

struct Object {
    double x;
    double y;
    double dist;
    double phi;
    double theta;
    double lmx;
    double lmy;
    int num;
};


struct Object *Objects, *Prev_Objects, *Vis_Objects;
void initObjects();
void findObjects();

int num_objects;
extern int SEPARATION_THRESH, NONSEPARATION_THRESH;
extern int HIGH_THRESH, LOW_THRESH;
extern int MAX_OBJECTS;

#endif
