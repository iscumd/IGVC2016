#ifndef __IMG_TEST_C
#define __IMG_TEST_C

#include <stdio.h>
#include <highgui.h>
#include <cv.h>

#include <conio.h>


IplImage *img, *red, *green,*blue, *temp, *out, *thresh;
proc1img() {
    CvScalar mean, std;
    if (img==NULL) return;
    if (red == NULL) {
        red=cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1);
        green=cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1);
        blue=cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1);
        out=cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1);
        temp=cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1);
        thresh=cvCreateImage(cvGetSize(img),IPL_DEPTH_8U,1);
    }
    cvSplit(img,blue, green, red,NULL);
    cvAddWeighted(green, 0.3, blue, 15/22, 0.0, temp); // temp = 0.6*green+0.4 blue + 0.0;
    cvAddWeighted(temp,1, red,14/24,0.0,out);
    cvShowImage("Image",img);
    cvShowImage("Temp", temp);
    cvShowImage("Out",out);
    cvAvgSdv(out, &mean, &std, NULL);
    cvThreshold(out, thresh,mean.val[0]+std.val[0],255, CV_THRESH_BINARY);
    cvShowImage("Thresh", thresh);
    if(27==cvWaitKey(0)) exit(0);
}

main() {
    int i;
    char name[100];

    for (i=0;i<100;i++) {
        sprintf(name, "%d.jpeg", i);
        img=cvLoadImage(name, CV_LOAD_IMAGE_COLOR);
        proc1img();
    }


}
#endif
