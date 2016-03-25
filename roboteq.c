#ifndef __ROBOTEQ_C__
#define __ROBOTEQ_C__
//#include "socket.h"
#include "serial.h"
#include "roboteq.h"

static HANDLE roboteqHandle = INVALID_HANDLE_VALUE;   /*  Handle to serial port */
//static int roboteqSocket;
void initRoboteq()
{
    roboteqHandle = initSerial(roboteqPort, 9600, 7, 2, 0, 0, 0, 0);
    //roboteqSocket = initSocket("192.168.0.204",5000);
    //sendMsg(roboteqSocket, "\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r");
}


#define LFWD ('b' ^ 3)
#define RFWD ('b')
#define LREV (LFWD ^ 32)
#define RREV (RFWD ^ 32)

#ifndef ABS
#define ABS(val) (val > 0 ? val: -val)
#endif


void sendSpeed(int leftSpeed, int rightSpeed)
{
    char lc, rc;
    char buffer[20];
    static int errorCount;

    //printf("lspeed = %d rspeed = %d\n",leftSpeed,rightSpeed);
    lc = leftSpeed > 0 ? LFWD : LREV;
    rc = rightSpeed > 0 ? RFWD : RREV;
    sprintf(buffer, "!%c%02X\r\n!%c%02X\r\n",
            lc, ABS(leftSpeed) & 0xFF, rc, ABS(rightSpeed) & 0xFF);
    if (roboteqHandle == INVALID_HANDLE_VALUE) {
        // Either the port was not opened or there was a failure
        // Try and open the robotEq again hope it works
        initRoboteq();
    }
    if (roboteqHandle != INVALID_HANDLE_VALUE) {
        SerialPuts(roboteqHandle, buffer);
        FlushRx(roboteqHandle);
        FlushTx(roboteqHandle);
    } else {
        if (++errorCount % 100 == 0)    // Print every 100 times this happens
            fprintf(stderr, "Invalid roboteqPort. Did you open the port?\n");
    }
/*
    if (!sendMsg(roboteqSocket, buffer)) {
        initRoboteq();
        if (++errorCount % 100 == 0)    // Print every 100 times this happens
            fprintf(stderr, "Invalid roboteqPort. Did you open the port?\n");
    }
*/
}
#endif //__ROBOTEQ_C__
