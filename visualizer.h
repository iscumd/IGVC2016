#ifndef VISUALIZER_H
#define VISUALIZER_H

#define YELLOW CV_RGB(255,255,0)
#define BLACK CV_RGB(0, 0, 0)
#define RED CV_RGB(255, 0, 0)
#define ORANGE CV_RGB(255, 127, 39)
#define GREEN CV_RGB(0, 128, 0)
#define BLUE CV_RGB(0, 0, 255)
#define WHITE CV_RGB(255, 255, 255)
#define SNOW_WHITE CV_RGB(223, 223, 223)
#define PURPLE CV_RGB(128, 0, 255)
#define GRAY CV_RGB(160, 160, 160)

// ROWLOC: gets the pointer to a row
#define ROWLOC(im,row) ((uchar*)((im)->imageData+(im)->widthStep*(row)))

// PIXLOC: gets the pointe to a row, col
#define PIXLOC(im,row,col) (ROWLOC(im,row)+col*(im->nChannels))


#define LINE_TYPE 8
#define THICKNESS 1
#define SHIFT 0

#ifndef M_PI
#define M_PI 3.1415926537       // Miciosoft does not have this!
#endif


extern XY targetListXY[100];
extern int maxTargets;
extern double fieldLength, fieldWidth;
extern double robotX, robotY, robotTheta, robotWidth;
extern double targetX, targetY;

void paintPathPlanner(double rx, double ry, double rt);
void showPlot();
void initVisualizer();
void showMap();
void mapSICK(double rx, double ry, double rt);
void mapVSICK(double rx, double ry, double rt);
void mapRobot(double rx, double ry, double rt);
void clearMapSection(double rx, double ry, double rt);
void initMap(double rx, double ry, double rt);
#endif
