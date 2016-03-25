#ifndef __VISION_NAV_C
#define __VISION_NAV_C
#include "macros.h"
#include "vision_nav.h"

#define FLAG_NORTH
#define CAPTURE_A 1
#define SKIP 1
#define BESTGPSTHRESH 10

//#define TESTING
#define TESTIMAGE "400.jpeg"

//"1.jpeg"
//"2.jpeg"
//"3.jpeg"
//"4.jpeg"
//"5.jpeg"
//"6.jpeg"
//"test2.jpeg"
//"testimg.jpeg"
//"find_lines.jpeg"
//"last_turn.jpeg"
//"image-B-16500.jpeg"
//"image-B-13000.jpeg"

#define SCALE 2

#define NROWS (480/SCALE)
#define NCOLS (640/SCALE)

#define WORLDMAP_NROWS (480/SCALE)
#define WORLDMAP_NCOLS (640/SCALE)

#include <stdio.h>
#include <stdlib.h>
#include "cv.h"
#include "highgui.h"
double R,F;

 IplImage *debugging;
 IplImage *world_map, *edgemap;
 IplImage *composite_out, *composite_inter;
 IplImage *flag_out_red, *flag_out_blue, *flag_inter;

float field_of_view=2; // See just + - field of view

typedef struct camera{
    double rinf;
    double cl;
    double kcbar;
    double kF;
    double kr;

    double rotation;
    double cs, sn;
}CAMERA_PARAM;
typedef struct {
        CvCapture* capture;
         CAMERA_PARAM cam_para;
         IplImage *image,*gray_inter, *gray,*image_threshold,*image_threshold3;

}  CAMERA;
CAMERA A = {
        NULL,
        {
         -0.0535,
         0.54,
         0.38,
         0.7953,
         1.31,
         0
        }
};
CvFont font;
double means[3], cov[3][3], covinv[3][3];
int npts;

/*
BRIGHT DAYLIGHT
R=245
G=245
B=245
*/

#define MATCHVAL 200
#define HEIGHTDIV 1000
#define WIDTHDIV 0.00
#define STATFRAC 0.15
#define METHOD CV_HOUGH_PROBABILISTIC
const int alpha_max = 25;
int alpha=2;
const int mean_max_slider_max = 25;
int mean_max_slider=4;
const int field_max = 500;
int field_int = 200;
const int houghThreshMax = 100;
int houghThresh=10;
//int houghThresh=16;
const int houghLenMax = 100;
int houghLen=8;
//int houghLen=16;
const int houghGapMax = 100;
int houghGap = 30;
const int blueThreshMax = 255;
//int blueThresh=205;
//int blueThresh=190;
int blueThresh=215;
//int blueThresh=160;
const int redThreshMax = 255;
int redThresh=230;
const int greenThreshMax = 255;
int greenThresh = 200;
const int distanceMultiplierMax = 255;
int distanceMultiplier = 50;
const int maxThreshVal = 255;
//int ThreshValSlider = 50;
int ThreshValSlider = 30;


IplImage *paramSliders;

void sliders(){
    if (paramSliders == NULL) {
        paramSliders = cvCreateImage(cvSize(NCOLS, NROWS), 8, 0);
        cvZero(paramSliders);
    }
    if(paramSliders != NULL){
        cvNamedWindow("Parameters", CV_WINDOW_AUTOSIZE);

        cvCreateTrackbar( "RThresh", "Parameters", &redThresh, redThreshMax, NULL );
        cvCreateTrackbar( "GThresh", "Parameters", &greenThresh, greenThreshMax, NULL );
        cvCreateTrackbar( "BThresh", "Parameters", &blueThresh, blueThreshMax, NULL );

        cvCreateTrackbar( "HThresh", "Parameters", &houghThresh, houghThreshMax, NULL );
        cvCreateTrackbar( "HLength", "Parameters", &houghLen, houghLenMax, NULL );
        cvCreateTrackbar( "HGap", "Parameters", &houghGap, houghGapMax, NULL );

        cvCreateTrackbar( "DMulti", "Parameters", &distanceMultiplier, distanceMultiplierMax, NULL );
        cvCreateTrackbar( "Floor", "Parameters", &alpha, alpha_max, NULL );
        cvCreateTrackbar( "Mean", "Parameters", &mean_max_slider, mean_max_slider_max, NULL );
        cvCreateTrackbar( "Thresh", "Parameters", &ThreshValSlider, maxThreshVal, NULL );
        cvShowImage("Parameters",paramSliders);
        cvReleaseImage(&paramSliders);
    }
}
void hough()
{

    CvMemStorage *storage = NULL;
    CvSeq *lines;
    int i;


    houghThresh = MAX(houghThresh,1);
    houghLen = MAX(houghLen,1);
    houghGap = MAX(houghGap,1);
    if (storage == NULL) storage = cvCreateMemStorage(0);
//    lines = cvHoughLines2(composite_out, storage, METHOD, 1, M_PI/180.0, NROWS/ houghThresh, NROWS / houghLen, NROWS/houghGap);    // Check these numbers
    lines = cvHoughLines2(composite_out, storage, METHOD, 1, M_PI/18.0, NROWS/ houghThresh, NROWS / houghLen, NROWS/houghGap);    // Check these numbers
 //   SHOW(lines->total);

    cvZero(composite_out);
    for (i = 0; i < lines->total; i++) {
        CvPoint *line = (CvPoint *) cvGetSeqElem(lines, i);
        CvScalar color = CV_RGB(255, 255, 255);;

        cvLine(composite_out, line[0], line[1], color, 1, CV_AA, 0);
    }
    cvClearMemStorage(storage);
    cvReleaseMemStorage(&storage);
//    cvClearSeq(lines);
//    cvReleaseMemStorage(&lines->storage);
}

#define ROBOTWIDTH 0.7
//#define PATHWIDTH  1.0
#define PATHWIDTH  1.25
#define MAXDIST 3.0
#define MINDIST 0.315
//#define MINDIST 0.0
#define MAXWIDTH 4.0
#define F2ROW(F) ((MAXDIST-F)/(MAXDIST-MINDIST)*NROWS)
#define ROW2F(ROW) (MAXDIST - (MAXDIST-MINDIST)*(ROW-NROWS/HEIGHTDIV)/((double) NROWS))

//#define R2COL(R,to) ((R+MAXWIDTH/2.0-PATHWIDTH/2.0)/MAXWIDTH*to->width)
#define R2COL(R) ((R+MAXWIDTH/2.0)/MAXWIDTH*NCOLS)
#define COL2R(COL) (((COL*MAXWIDTH)/NCOLS)-(MAXWIDTH/2))


void cam2world(double row, double col, CAMERA_PARAM cam_para){
     double rbar, cbar;
     double Fc, Rc;
     rbar = row - cam_para.rinf;
     cbar = col - cam_para.cl;

     Fc = (cam_para.kr/rbar - 1)/cam_para.kF;
     Rc = cbar/rbar/cam_para.kcbar;


     cam_para.cs = cos(cam_para.rotation);
     cam_para.sn = sin(cam_para.rotation);

     F = Fc*cam_para.cs- Rc*cam_para.sn;
     R = Fc*cam_para.sn + Rc*cam_para.cs;
//     F = (cam_para.kr/rbar - 1)/cam_para.kF;
//     R = cbar/rbar/cam_para.kcbar;
//     R = cbar/rbar/cam_para.kcbar*cam_para.kr;
    // SHOW(row); SHOW(col); SHOW(F); SHOWN(R);
}

unsigned short vradar[1081];
unsigned short vr_count[1081];
#define ANG2INDEX(theta) (540-(theta)*4)
#define VIS_LIDAR_THRESH 64
//#define VRTHRESH 5
#define VRTHRESH 1
void vision2lidar(){
    double theta=RAD2DEG(atan2(R,F));
    unsigned short dist=(unsigned short)(hypot(R,F)*1000);
    int idx=ANG2INDEX(theta);
    if(idx>=0&&idx<=1080){
        vr_count[idx]+=1;
//#define USE_VRAVG
#ifdef USE_VRAVG
        if(vr_count[idx]==1) vradar[idx] = dist;
        else vradar[idx]+= (dist-vradar[idx])/vr_count[idx]+0.5;
#else
        if(vradar[idx]>dist) vradar[idx]=dist;
#endif // USE_VRAVG
    }
}
void cam2worldxform(IplImage * in, IplImage * to, CAMERA_PARAM cam_para, double scaling){
    int row,col,trow,tcol;
    int k = 0;
     for(k = 0; k<1081;k++){
#ifdef USE_VRAVG
        vradar[k]= 0;
#else
        vradar[k]= 15000;
#endif // USE_VRAVG
        vr_count[k] = 0;
     }
     for (row = 0; row < in->height; row+=SKIP){
        unsigned char *rowstart, *pixel;
        rowstart = in->imageData;
        rowstart += row * in->widthStep;
        for (col = 0; col < in->width; col+=SKIP){
             pixel = rowstart + col*in->nChannels;
             cam2world( row/((double)in->height),
                   col/((double)in->width),
                   cam_para);

        trow = F2ROW(F);
        tcol = R2COL(R);
        if(trow>=0 && trow<to->height && tcol>=0 && tcol<to->width){
            unsigned char *topix;
            topix = to->imageData;
            topix += trow*to->widthStep + tcol*to->nChannels;
            //SHOW(F); SHOW(trow); SHOW(R); SHOWN(tcol);
            *topix = *pixel;
            if(*pixel > VIS_LIDAR_THRESH)vision2lidar();
        }
     }
}
}
double COLOR(double r, double g, double b){
    double distance=0,meanMag=0,valMag=0,diffMag=0;
    int i, j;
    int nok = 0;
    double values[3];


    values[0]=b-means[0]; values[1]=g-means[1]; values[2]=r-means[2];
#define RMG
#ifdef RMG
    int sum = r+b-2*g;
    int rmg = MAG(r-g);
    if(rmg>10){
//         if(sum<0||sum>30) return 0;
         if(sum<0||sum>ThreshValSlider) return 0;
    }else if(sum<0||sum>1.4*ThreshValSlider) return 0;
#endif //RMG
#define COLORMATCHING
#ifdef COLORMATCHING
    if(b<blueThresh){
        nok++;
    }else nok--;
    if(g<greenThresh){
        nok++;
    }
    if(r<redThresh){
        nok++;
    }//else nok--;
//    if(r>245) nok++;
//    if(b>245) nok++;
//    if(g>245) nok++;
    if(nok>=2) return 0;
//    else return 255;
#endif


    for(i=0;i<3;i++) {
        distance += covinv[i][i]*values[i]*values[i];
        for (j=i+1; j<3; j++)
          distance+=values[i]*covinv[i][j]*values[j];
    }

//    alpha=MAX(alpha,1);
   distance -=alpha;
   distance *=distanceMultiplier;
   if(distance>255) distance=255;
    return distance;
}

#define SHEIGHTDIV 0.25
#define SWIDTHDIV 0.00
void collect_stats() {
      int row, col, i,j;
      int nrows, ncols;
      int numsamples;
      unsigned char *startline,*pixel, *debline, *debpixel;
      double values[3];
      double tf, tr;
      srand(0);
      nrows=NROWS;
      ncols=NCOLS;
      numsamples = STATFRAC*nrows*ncols;
      while(numsamples--) {
              row = rand()%nrows;
              col = rand()%ncols;
              tf = ROW2F(row);
              tr = COL2R(col);
//              if(row>NROWS*SHEIGHTDIV) continue;
//              if(col<NCOLS*SWIDTHDIV||col>NCOLS*(1-SWIDTHDIV)) continue;
              if(tf>MAXDIST||tf<MINDIST) continue;
              if(fabs(tr)>MAXWIDTH) continue;
              startline=A.image->imageData;
              startline+=row*A.image->widthStep;

             pixel = startline + col*A.image->nChannels;

             values[0]=pixel[0];
             values[1]=pixel[1];
             values[2]=pixel[2];
             npts++;
             for (i=0; i<3; i++) {
                 means[i] += values[i];
                 for(j=0; j<3;j++) {
                          cov[i][j]+=values[i]*values[j];
                 }
             }
      }
}

void init_stats() {
     int i,j;
     npts=0;
     for(i=0; i<3; i++) {
              means[i]=0.0;
              for (j=0; j<3; j++)
                  cov[i][j]=0;
     }
}
void invert_cov() {
     int i,j,k;
           for (i=0; i<3; i++) {
          means[i] /= npts;
          for (j=0; j<3; j++) cov[i][j] /=npts;
      }
      for (i=0; i<3; i++) {
         for (j=0; j<3; j++) cov[i][j] -= means[i]*means[j];
      }
      // Invert cov matrix
      //Initialize inverse = identity
      for (i=0; i<3; i++) {
          for (j=0; j<3; j++) covinv[i][j]=0;
          covinv[i][i]=1;
      }
      for (i=0; i<3; i++) {
          //pivot on diagonal
          double pivot;
          int k;
          pivot=cov[i][i];
          for (j=0; j<3; j++) {
              cov[i][j] /= pivot;
              covinv[i][j]/=pivot;
          }
          // Eliminate variable i from all other rows
          for (k=0; k<3;k++) if (k!=i) {
              pivot = cov[k][i];
              for (j=0; j<3; j++) {
                  cov[k][j] -= pivot*cov[i][j];
                  covinv[k][j] -= pivot*covinv[i][j];
              }
          }
      }

   for(i=0; i<3; i++)
            for (j=i+1; j<3; j++) covinv[i][j]*=2;
}

void find_diff(){
     int row, col, rednps;
     double distance, avgness=0, maxness=0, threshold;
     double m00,m01,m10;
     unsigned char *startline, *startline2,*pixel,*pixel_red;
     double r,g,b;

     field_of_view = field_int/100.0;
     cvZero(composite_inter);
     row = NROWS/HEIGHTDIV;
     rednps=0;
//     for(row=0;row<in->height;row++){
     for(;row<NROWS;row++){
        int start_col, end_col;
        double r;
     startline=A.image->imageData;
     startline+=row*A.image->widthStep;
     startline2=composite_inter->imageData;
     startline2+=row*composite_inter->widthStep;
     pixel = startline;
     pixel_red = startline2;

     r = row/((double) NROWS);
     start_col = 0;
     end_col = NCOLS;
     if (start_col<NCOLS*WIDTHDIV) start_col=NCOLS*WIDTHDIV;

     if (end_col> NCOLS*(1-WIDTHDIV)) end_col=NCOLS*(1-WIDTHDIV);

      pixel_red += start_col;
      pixel = startline+start_col*A.image->nChannels;

       for(col=start_col;col<end_col;col++){
          double randval;
          b= *pixel++;
          g= *pixel++;
          r= *pixel++;
         distance=COLOR(r,g,b);
              if (distance>0){
                       rednps++;
                       avgness +=distance;
                       maxness = maxness>distance?maxness:distance;
              } else distance=0;
                *pixel_red++ = distance;
     }
}
     avgness/=rednps;
     avgness /=2;
     threshold=avgness+mean_max_slider/100.0*(maxness-avgness);
//     cvMorphologyEx(composite_inter,composite_inter,NULL,NULL,CV_MOP_ERODE,1);
     cvThreshold(composite_inter, composite_out, threshold, 255, CV_THRESH_BINARY);
//     cvThreshold(composite_inter, composite_inter, ThreshValSlider, 255, CV_THRESH_TOZERO);
//     cvThreshold(composite_inter, composite_out, threshold, ThreshValSlider, CV_THRESH_TOZERO);
//     cvErode(composite_out,composite_out,NULL,1);
//     cvMorphologyEx(composite_out,composite_out,NULL,NULL,CV_MOP_CLOSE,1);
//     cvMorphologyEx(composite_out,composite_out,NULL,NULL,CV_MOP_DILATE,1);
}



#define NMASKS 11
IplImage *masks[NMASKS], *overlay[NMASKS]; // 2 more for two sides overlay used for debugging
IplImage *finalOverlay[NMASKS];
IplImage *flagmasks[NMASKS];
int howbad[NMASKS];
double turn_vals[NMASKS];
double turn_angle[NMASKS];
double importance[NMASKS];
double importanceBlue[NMASKS]={0.1, 0.1, 0.1, 0.1, 0.9, 1.5, 0.9, 0.8, 0.7, 0.6, 0.5};
double importanceRed[NMASKS]={0.5, 0.6, 0.7, 0.8, 0.9, 1.5, 0.8, 0.1, 0.1, 0.1, 0.1};
double masksize[NMASKS];
double flagmasksizeRed[NMASKS];
double flagmasksizeBlue[NMASKS];

#define MASKDIV 0.50
#define IMPORTANCEDIV 0.25
void initTurnVals(){
    int i = 0;
    double tempTurn;
    for(i=0;i<NMASKS;i++){
        tempTurn = pow((i - (NMASKS)/2),3)/pow((NMASKS*MASKDIV),3);
        turn_vals[i]=tempTurn;
//        importance[i]=2-IMPORTANCEDIV*fabs(tempTurn);
        importance[i]=0.75-IMPORTANCEDIV*fabs(tempTurn);
    }
}

void mkmasks() {
     int mskno;
     char name[80];
     initTurnVals();
     for (mskno=0; mskno<NMASKS;mskno++) {
         int row, col;
         double turn;
         masks[mskno]=cvCreateImage(cvSize(NCOLS, NROWS),
                              IPL_DEPTH_8U,1);
         overlay[mskno]=cvCreateImage(cvSize(NCOLS, NROWS),
                              IPL_DEPTH_8U,1);
         cvZero(masks[mskno]);
         masksize[mskno]=0;

         turn=turn_vals[mskno]; //SHOW(turn);
         turn_angle[mskno] = atan(0.75*turn*MAXDIST/ROBOTWIDTH);
        row = NROWS/(HEIGHTDIV);
        for (; row < NROWS; row++) {
//         for (row=0; row < to->height; row++) {
             double f,r,d ;
             double wt;
             int startcol, endcol;
             unsigned char *rowstart, *pixel;
             f = ROW2F(row); //SHOW(f);
             wt = (MAXDIST-f)/(MAXDIST);
             wt += 0.20;
             wt = MIN(1,wt);
             wt = MAX(wt,0.00);
             r = f*f*turn/ROBOTWIDTH-PATHWIDTH/2*wt;
//             r = f*f*turn/ROBOTWIDTH-PATHWIDTH/2;
//             if (f<0.5) continue;
             startcol=R2COL(r);
             r += PATHWIDTH*wt; // 1 meter wide path
//             r += PATHWIDTH; // 1 meter wide path
             endcol = R2COL(r);
             if (startcol <NCOLS*WIDTHDIV) startcol=NCOLS*WIDTHDIV;
             if (startcol >= NCOLS*(1-WIDTHDIV)) continue; //skip
             if (endcol < NCOLS*WIDTHDIV) continue;
             if (endcol >= NCOLS*(1-WIDTHDIV)) endcol = NCOLS*(1-WIDTHDIV);
             rowstart = masks[mskno]->imageData;
             rowstart += row*masks[mskno]->widthStep;
             wt = (1+MAXDIST-f)/(1+MAXDIST);
             for (col=startcol; col<endcol; col++) {
                 pixel = rowstart + col;
                 *pixel = 255*wt;
                 masksize[mskno] += *pixel;
             }
         }
         masksize[mskno] *= importance[mskno];
     }

     for (mskno=0; mskno<NMASKS;mskno++) {
         int row, col;
         double turn;
         flagmasks[mskno]=cvCreateImage(cvSize(NCOLS, NROWS),
                              IPL_DEPTH_8U,1);
         cvZero(flagmasks[mskno]);
         flagmasksizeBlue[mskno]=0;
         flagmasksizeRed[mskno]=0;

        turn=turn_vals[mskno]; //SHOW(turn);
        turn_angle[mskno] = atan(0.75*turn*MAXDIST/ROBOTWIDTH);
        row = NROWS/(HEIGHTDIV);
        for (; row < NROWS; row++) {
             double f,r,d ;
             double wt;
             int startcol, endcol;
             unsigned char *rowstart, *pixel;
             f = ROW2F(row); //SHOW(f);
             wt = (MAXDIST-f)/(MAXDIST);
             wt += 0.20;
             wt = MIN(1,wt);
             wt = MAX(wt,0.00);
//             r = f*f*turn/ROBOTWIDTH-PATHWIDTH/2*wt;
             r = f*f*turn/ROBOTWIDTH-PATHWIDTH/2;
//             if (f<0.5) continue;
             startcol=R2COL(r);
//             r += PATHWIDTH*wt; // 1 meter wide path
             r += PATHWIDTH; // 1 meter wide path
             endcol = R2COL(r);
             if (startcol <NCOLS*WIDTHDIV) startcol=NCOLS*WIDTHDIV;
             if (startcol >= NCOLS*(1-WIDTHDIV)) continue; //skip
             if (endcol < NCOLS*WIDTHDIV) continue;
             if (endcol >= NCOLS*(1-WIDTHDIV)) endcol = NCOLS*(1-WIDTHDIV);
             rowstart = flagmasks[mskno]->imageData;
             rowstart += row*flagmasks[mskno]->widthStep;
             wt = (1+MAXDIST-f)/(1+MAXDIST);
             for (col=startcol; col<endcol; col++) {
                 pixel = rowstart + col;
                 *pixel = 255*wt;
                 flagmasksizeRed[mskno] += *pixel;
                 flagmasksizeBlue[mskno] += *pixel;
             }
         }
         flagmasksizeBlue[mskno] *= importanceBlue[mskno];
         flagmasksizeRed[mskno] *= importanceRed[mskno];
     }
}


int initVision() { // returns -1 on failure 0 on success

    sliders();
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0, 1, CV_AA);

    A.image=NULL;
    A.capture=NULL;
#if !defined(TESTING)
    A.capture =cvCaptureFromCAM(CAPTURE_A);
    if (A.capture==NULL) return -1;
    if (A.capture){
       cvSetCaptureProperty(A.capture, CV_CAP_PROP_FRAME_WIDTH,NCOLS);
       cvSetCaptureProperty(A.capture, CV_CAP_PROP_FRAME_HEIGHT,NROWS);
       cvSetCaptureProperty(A.capture, CV_CAP_PROP_FPS,30);
    }
#endif

    {
      A.image = cvCreateImage(cvSize(NCOLS,NROWS), IPL_DEPTH_8U,3);
    }

    edgemap = cvCreateImage(cvSize(NCOLS, NROWS),IPL_DEPTH_8U,1);
    composite_out = cvCreateImage(cvSize(NCOLS, NROWS), IPL_DEPTH_8U,1);
    composite_inter = cvCreateImage(cvSize(NCOLS, NROWS), IPL_DEPTH_8U,1);
    world_map = cvCreateImage(cvSize(NCOLS, NROWS),IPL_DEPTH_8U,1);

/*FLAG IMAGES*/
    flag_out_blue = cvCreateImage(cvSize(NCOLS, NROWS), IPL_DEPTH_8U,1);
    flag_out_red = cvCreateImage(cvSize(NCOLS, NROWS), IPL_DEPTH_8U,1);
    flag_inter = cvCreateImage(cvSize(NCOLS, NROWS), IPL_DEPTH_8U,1);

    cvSet(composite_out, cvScalarAll(255.5) ,NULL);
    cvSet(composite_inter, cvScalarAll(255.5) ,NULL);
    cvSet(world_map, cvScalarAll(0) ,NULL);

    mkmasks(world_map);

#   if defined(TESTING)
     cvResize(cvLoadImage(TESTIMAGE, CV_LOAD_IMAGE_COLOR), A.image, CV_INTER_NN);
#   endif
    return 0;
}
void get_images() {
     if(A.capture) {
        cvResize(cvQueryFrame(A.capture), A.image, CV_INTER_NN);
     }
}

void extract_lanes() {
   if(A.image) {
       init_stats();
       collect_stats();
       invert_cov();
       find_diff();
       hough();
       cam2worldxform(composite_out, world_map, A.cam_para, 1.0);
//        cvMorphologyEx(world_map,world_map,NULL,NULL,CV_MOP_DILATE,1);
//        cvMorphologyEx(world_map,world_map,NULL,NULL,CV_MOP_CLOSE,1);

//        cvRectangle(composite_out, cvPoint(NCOLS*0.35, NROWS*0.90), cvPoint(NCOLS*0.65, NROWS), CV_RGB(0, 0, 0), CV_FILLED, 8, 0);
   }

}
int bestmask, worstmask, bestGpsMask;
int find_best() {
    double minbad, maxbad;
    int mskno;
    char buffer[1024];

    minbad=1e40;
    maxbad = -1;
   for (mskno=0; mskno<NMASKS; mskno++) {

//     howbad[mskno]=(double)(cvDotProduct(composite_out, masks[mskno])/masksize[mskno]);
     howbad[mskno]=((double)(cvDotProduct(world_map, masks[mskno])+0.1)/masksize[mskno]);
     if (howbad[mskno]<minbad||(howbad[mskno]==minbad&&importance[mskno]>importance[bestmask])) {
                   minbad = howbad[mskno];
                   bestmask=mskno;
//                   printf("bm: %d v: %f\n",bestmask,minbad);
//                   getchar();
     }
    if (howbad[mskno]>maxbad) {
                   maxbad = worstmaskval =howbad[mskno];
                   worstmask=mskno;
     }
    cvAddWeighted(composite_out, 1.0, masks[mskno], 1.0, 0.0, overlay[mskno]);
    cvAddWeighted(world_map, 1.0, masks[mskno], 1.0, 0.0, overlay[mskno]);
//    printf("hb: %d v: %d\n",mskno,howbad[mskno]);
   }
   if(howbad[bestGpsMask]<BESTGPSTHRESH&&(mode==2||mode==3)) bestmask = bestGpsMask;
//    printf("TV: %f\n",turn_angle[bestmask]);
}
#define FLAG_BOOST 2
void findBestBlueFlags() {
    double minbad, maxbad;
    int mskno, hb;//mask number, howbad
    int tempBest;

    minbad=1e40;
    maxbad = -1;
    for (mskno=5; mskno<NMASKS; mskno++) {
        hb=((double)(cvDotProduct(flag_out_blue, flagmasks[mskno])*FLAG_BOOST+0.1)/flagmasksizeBlue[mskno]);

        if (hb<minbad||(hb==minbad&&importanceBlue[mskno]>importanceBlue[bestBlueMask])) {
            minbad = hb;
            bestBlueMask=mskno;
            tempBest=bestBlueMask;
        }
        if (hb>maxbad) {
            maxbad = hb;
            worstBlueMask=mskno;
        }
//    printf("hbb: %d v: %d bm:%d \n",mskno,hb, tempBest);
    }
//    getchar();
//    if(worstBlueMask>=5&&tempBest!=5){
    if(worstBlueMask>=5){
        bestBlueMask = MIN(worstBlueMask+2,9);
        bestBlueMask = MAX(bestBlueMask,5);
    }
    if(bestBlueMask==worstBlueMask){
        bestBlueMask = MIN(tempBest,9);
        bestBlueMask = MAX(bestBlueMask,5);
    }
//    if(tempBest==5){
//        bestBlueMask=worstBlueMask;
//    }
//    printf("Best BLUE: %d v: %f\n",bestBlueMask,minbad);
//    printf("Worst BLUE: %d v: %f\n",worstBlueMask,maxbad);
}
void findBestRedFlags() {
    double minbad, maxbad;
    int mskno, hb;//mask number, howbad
    int tempBest;
    minbad=1e40;
    maxbad = -1;
//    cvAddWeighted(flag_out_blue, 1.0, flag_out_red, 1.0, 0.0, flag_out_red);
    for (mskno=0; mskno<6; mskno++) {
//    for (mskno=0; mskno<NMASKS; mskno++) {
        hb=((double)(cvDotProduct(flag_out_red, flagmasks[mskno])*FLAG_BOOST+0.1)/flagmasksizeRed[mskno]);

        if(hb<minbad||(hb==minbad&&importanceRed[mskno]>importanceRed[bestRedMask])) {
            minbad = hb;
            bestRedMask=mskno;
            tempBest = bestRedMask;
        }
        if(hb>maxbad) {
            maxbad = hb;
            worstRedMask=mskno;
        }
//    printf("hbr: %d v: %d bm:%d\n",mskno,hb, tempBest);
    }
//    getchar();
//    if(worstBlueMask<=5&&tempBest!=5){
    if(worstRedMask<=5&&maxbad!=0){
        bestRedMask = MAX(worstRedMask-2,1);
        bestRedMask = MIN(bestRedMask,5);
    }
    if(bestRedMask==worstRedMask){
        bestRedMask = MAX(tempBest,1);
        bestRedMask = MIN(bestRedMask,5);
    }
//    if(tempBest==5){
//        bestBlueMask=worstBlueMask;
//    }
    printf("Best RED: %d v: %f\n",bestRedMask,minbad);
    printf("Worst RED: %d v: %f\n",worstRedMask,maxbad);
}
#define FHEIGHTDIV 5
#define FWIDTHDIV 0.25
void flagDetection(){


     int row, col;
     unsigned char *startline, *startRed, *startBlue,*raw,*outputRed,*outputBlue;
     double r,g,b;
     row = NROWS/FHEIGHTDIV;
     for(;row<NROWS;row++){
        int start_col, end_col;
        double r;
     startline=A.image->imageData;
     startline+=row*A.image->widthStep;
     startRed=flag_out_red->imageData;
     startRed+=row*flag_out_red->widthStep;
     startBlue=flag_out_blue->imageData;
     startBlue+=row*flag_out_blue->widthStep;
     raw = startline;
     outputRed = startRed;
     outputBlue = startBlue;

     r = row/((double) NROWS);
     start_col = 0;
     end_col = NCOLS;
     if (start_col<NCOLS*FWIDTHDIV) start_col=NCOLS*FWIDTHDIV;
     if (end_col> NCOLS*(1-FWIDTHDIV)) end_col=NCOLS*(1-FWIDTHDIV);
        outputRed += start_col;
        outputBlue += start_col;
        raw = startline+start_col*A.image->nChannels;
        for(col=start_col;col<end_col;col++){
            double randval;
            b= *raw++;
            g= *raw++;
            r= *raw++;
            if((2*r-g-b>0&&b<170&&g<170&&r>200)||2*r-g-b>125){
                *outputRed = 255;
            }else{
                *outputRed = 0;
            }
            if((2*b-r-b>0&&r<170&&g<170&&b>200)||2*b-r-b>75){
                *outputBlue = 255;
            }else{
                *outputBlue = 0;
            }
#if 1
            /*GOING SOUTH*/
            if(col<NCOLS/4) *outputRed = 0;
            if(col>NCOLS-NCOLS/4) *outputBlue = 0;
#endif //FLAG_NORTH
            *outputBlue++;
            *outputRed++;
        }
    }
       cam2worldxform(flag_out_blue, world_map, A.cam_para, 1.0);
       cam2worldxform(flag_out_red, world_map, A.cam_para, 1.0);
//    cvThreshold(flag_inter, flag_out, 128, 255, CV_THRESH_BINARY);
}
int imageNumber = 400;
int imageProc(int findFlags) { // Returns -1 if user presses Esc or else returns 0
    double forward, turn;
    char buffer[1024];
    char cvkey = 0;
    int i =0;
    int p[3] = { CV_IMWRITE_JPEG_QUALITY, 100, 0 };
    char fname[80];
    static int imagecount;
    if(A.image == NULL) initVision();
    cvZero(world_map);
    cvZero(composite_inter);
    cvZero(composite_out);
    cvZero(edgemap);

    get_images();
    if(saveImage==DEBOUNCE_FOR_SAVE_IMAGE) {
            sprintf(fname, "igvcimg%04d.jpeg", imagecount++);
            cvSaveImage(fname, A.image, p);
    }
    extract_lanes();
    if(findFlags > 3){
        cvZero(flag_inter);
        cvZero(flag_out_blue);
        cvZero(flag_out_red);
        flagDetection();
        findBestBlueFlags();
        findBestRedFlags();
        cvAddWeighted(flag_out_blue, 1.0, flagmasks[bestBlueMask], 1.0, 0.0, flag_out_blue);
        cvAddWeighted(flag_out_red, 1.0,  flagmasks[bestRedMask], 1.0, 0.0, flag_out_red);
    }
    find_best();

    for(i=0;i<1080;i++){
    if(vr_count[i]<VRTHRESH){
            vradar[i] = 15000;
        }
    }

//    cvLine(A.image, cvPoint(NCOLS/2-2,0), cvPoint(NCOLS/2-2,NROWS-1), CV_RGB(255,0,0), 3, 8, 0);
    cvShowImage("Raw", A.image);

    cvShowImage("Worst mask", overlay[worstmask]);
    cvShowImage("Intermediate", composite_inter);
    cvShowImage("Output", composite_out);
//    cvShowImage("World Map", world_map);
    cvShowImage("Best mask", overlay[bestmask]);
//    cvShowImage("Edges", edgemap);
    cvShowImage("BLUE", flag_out_blue);
    cvShowImage("RED", flag_out_red);
    cvkey = cvWaitKey(1);
    if(cvkey==27) return -1;
    else if(cvkey == 'b' || cvkey == 'B'){
        char nextImage[20];
        sprintf(nextImage,"%d.jpeg",--imageNumber);
        cvResize(cvLoadImage(nextImage, CV_LOAD_IMAGE_COLOR), A.image, CV_INTER_NN);
    }
    else if(cvkey == 'n' || cvkey == 'N'){
        char nextImage[20];
        sprintf(nextImage,"%d.jpeg",++imageNumber);
        cvResize(cvLoadImage(nextImage, CV_LOAD_IMAGE_COLOR), A.image, CV_INTER_NN);
    }
    else return 0;
}
//#define TEST_VISION
#ifdef TEST_VISION
int main(){

    while(1){
        if(imageProc() == -1) break;
    }
}
#endif
#endif
