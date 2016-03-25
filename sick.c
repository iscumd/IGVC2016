#ifndef SICK_C
#define SICK_C
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sick.h"
#include "socket.h"

char sickBuffer[10000];
char *header = "DIST1 3F800000 00000000 FFF92230 9C4 ";
char *start_cmd = "\x02sRN LMDscandata\x03";
char *stop_cmd = "\x02sRN LMDscandata\x03";

void update360(double rt){
    int i;
    for(i=0;i<SICK_NUMPTS;i++){
        LMS360data[i]=LMSdata[i];
    }
}

void initSICK(){
    SickIP = "192.168.0.100";
//    SickIP = "192.168.0.195";
    SickPort = 2111;
//    LMSdata = (short *) malloc(SICK_NUMPTS*sizeof(short));//dynamic don't care for now
    LMS360data = (short *) malloc(1441*sizeof(short));
    SickSock = initSocket(SickIP, SickPort);              //socket.c
}

void fillData(){
    short d = 0, mag = 0;
    int LMS_numpts = 0;
    int i = 0;

    char *lidar_msg=strstr(sickBuffer,header);//Look for header string. Move Pointer to begging of Header
//    char *lidar_msg=strstr(sockBuffer,header);//Look for header string. Move Pointer to begging of Header
    if(lidar_msg == NULL||*lidar_msg=='\0') return;//don't update if header wasn't found
    lidar_msg+=strlen(header);//move past length of header.

    while (*lidar_msg != ' '&&*lidar_msg!='\0') {
            if(*lidar_msg > '@') *lidar_msg -= '7';//'7' = 55 decimal
            else *lidar_msg -= '0';
            LMS_numpts=(LMS_numpts<<4) | *lidar_msg;
            lidar_msg++;
    }
    if(lidar_msg == NULL||*lidar_msg=='\0') return;//don't continue data not complete
    lidar_msg++;
    for (i = 0; i < LMS_numpts; i++) {
        d = mag = 0;
        while (*lidar_msg != ' '&&*lidar_msg!='\0') {
            if(*lidar_msg > '@') *lidar_msg -= '7';//'7' = 55 decimal
            else *lidar_msg -= '0';
            d=(d<<4) | *lidar_msg;
            lidar_msg++;
            if(lidar_msg == NULL||*lidar_msg=='\0') return;//don't continue data not complete
        }
        lidar_msg++;
        if(lidar_msg == NULL||*lidar_msg=='\0') return;//don't continue data not complete
        LMSdata[i]=d;
        if(d<0x20)  LMSdata[i]=MAX_RADIUS*2;;
    }
}

void updateSick(){
    sendMsg(SickSock, start_cmd);
    if(getMsg(SickSock,sickBuffer)){            //socket.c
        fillData();
    }else initSICK();
}

void debugSICK(){
    int i = 0;
    for(i = 0; i < SICK_NUMPTS; i++){
        printf("LMSdata[%d] = %d\n",i,LMSdata[i]);
    }
}
#endif
