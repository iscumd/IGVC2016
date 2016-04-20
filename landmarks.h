#ifndef LANDMARKS_H
#define LANDMARKS_H//pretty much this whole thing, is it some sort of attempt to map?
struct Landmark {
    double x;
    double y;
    double dist;
    double phi;
    int found;
    int rejected;
} CLM[100], KLM[100];


extern double robotX, robotY, robotTheta;
extern double landmark_tolerance;
float MAX_LANDMARKS_DISTANCE;
int num_landmarks, landmarks_seen;
int LM_POINTS_THRESH, LM_SCAN_POINTS, MAX_LANDMARKS_LOADED;
void scan_landmarks();
void known_landmarks_init();
void obj2landmarks();
#endif
