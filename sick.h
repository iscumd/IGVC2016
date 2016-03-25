#ifndef SICK_H
#define SICK_H


#define MAX_RADIUS 15000
#define RESOLUTION 0.25
#define SICK_NUMPTS 1081

int SickSock;
char *SickIP;//"192.169.0.1"
short LMSdata[1081];
short *LMS360data;
unsigned short SickPort;//2111
void updateSick();
void initSICK();
void debugSick();
#endif
