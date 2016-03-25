#ifndef VISION_NAV_H
#define VISION_NAV_H
// This has the info for interfacing with other parts of the code
extern  double turn_vals[];
extern  double turn_angle[];
extern int howbad[];
extern unsigned short vradar[];
extern int distanceMultiplier;
extern int bestGpsMask;
extern int mode;
int saveImage;
int bestmask, worstmask;
int bestRedMask, worstRedMask;
int bestBlueMask, worstBlueMask;
int worstmaskval;
int imageProc(int findFlags); // Returns -1 if user presses Esc or else returns 0
int initVision();
#define DEBOUNCE_FOR_SAVE_IMAGE 10
#endif
