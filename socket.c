#ifndef SOCK_C
#define SOCK_C
#include "socket.h"

int getMsg(int socket, char *msgBuf){
    /*Try to read buffer*/
    int bytesread=recv(socket, sockBuffer, MAXBUFLEN, 0);
//    int bytesread=recv(socket, msgBuf, MAXBUFLEN, 0);
    if (bytesread < 0) {
        perror("recvfrom() failed");
        return 0;
    }
    sockBuffer[bytesread] = '\0';

    strcpy(msgBuf, sockBuffer);
    return bytesread;
}

int sendMsg(int socket, char *msg){
        /*Request latest data*/
    int sendbytes = strlen(msg);
    if (send(socket, msg, sendbytes, 0) != sendbytes){
       perror("send()  sent  a different  number  of bytes  than  expected");
        return 0;
    }
    return 1;
}

int initSocket(char *ServIP, unsigned short ServPort)
{
    struct sockaddr_in ServAddr;    /* Local address */
    int sock;
#if !defined(LINUX)
    {
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 0), &wsaData) != 0) {
            fprintf(stderr, "WSAStartup() failed");
            //exit(1);
        }
    }
#endif
    fprintf(stderr, "serving:  %s:%d\n", ServIP, ServPort);
    /* Create socket for sending/receiving datagrams */
    if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0) {
        perror("socket() failed");
        //exit(1);
    }
    /* Construct local address structure */
    memset(&ServAddr, 0, sizeof(ServAddr));
    ServAddr.sin_family = AF_INET;
    ServAddr.sin_addr.s_addr = inet_addr(ServIP);
    ServAddr.sin_port = htons(ServPort);

    /*  Establish  the  connection  to  the  echo  server  */
    if (connect(sock, (struct sockaddr *) &ServAddr, sizeof(ServAddr)) < 0){
        perror(" connect ()  failed");
    }else{
        return sock;
    }
    return 0;
}
#endif
