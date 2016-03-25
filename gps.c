#ifndef __GPS_C__
#define __GPS_C__

#define NUMGPS 2

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "gps.h"
#include "serial.h"
#include "socket.h"
#include "macros.h"
#include "nmeap.h"

/** Values for nmea parser      */
static nmeap_context_t nmea;	   /* parser context */
static nmeap_rmc_t     rmc;		   /* this is where the data from RMC messages will show up */
static int             user_data; /* user can pass in anything. typically it will be a pointer to some user data */
static int             nmea_status; /* status variable for nmea parser */
static bool            newGPSvalue = false; /* flag for when new GPS data is available */

static int gpsSocket;
char gpsBuffer[10000];

void initGPS(){
    gpsSocket = initSocket("192.168.0.202", 5000);
}

/* called when a gprmc message is received and parsed */
static void gprmc_callout(nmeap_context_t *context,void *data,void *user_data)
{
    nmeap_rmc_t *rmc = (nmeap_rmc_t *)data;

    gpsvar.latitude = rmc->latitude;
    gpsvar.longitude = rmc->longitude;
    gpsvar.course = rmc->course;
    gpsvar.speed = rmc->speed;
    gpsvar.time = rmc->time;

    newGPSvalue = 1;
}/* called when a gprmc message is received and parsed */

//void GPS_Filter()
//{
//    int delta_T = time()
//    gpsvar.confidence = ;
//    filtered.confidence = gpsvar.confidence*exp(delta_T/)
//}


/*void initialize nmea parser */
void initParser()
{
    nmea_status = nmeap_init(&nmea,(void *)&user_data);  /* Initialize the nmea context */

	/* add standard GPRMC parser      */
    nmea_status = nmeap_addParser(&nmea,"GPRMC",nmeap_gprmc,gprmc_callout,&rmc);
}

void parseChar(char c)
{
    nmea_status = nmeap_parse(&nmea,c);
}

int gpsUpdated()
{
    if( newGPSvalue == true)
    {
        newGPSvalue = false;
        return true;
    }else
    {
        return false;
    }
}

void readGPS(){
        int gpsMsgIndex = 0;
        int gpsMsgLen = getMsg(gpsSocket,gpsBuffer);//Get Latest GPS Data

        while(!gpsUpdated()&&gpsMsgIndex<gpsMsgLen){
            parseChar(gpsBuffer[gpsMsgIndex]);
            gpsMsgIndex++;
        }

}

void debugGPS()
{
    fprintf(stderr,"GPRMC: %-6f, %-6f, %-6f, %-6f\n",gpsvar.latitude,gpsvar.longitude,gpsvar.course,gpsvar.speed);
}
#endif
