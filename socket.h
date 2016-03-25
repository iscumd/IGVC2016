#ifndef SOCKET_H
#define SOCKET_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#if !defined(LINUX)
#	include <winsock2.h>
// One of the following should work with dev C++
// For some reason none of them work!
// If it does not, you will see an error message that looks like:
//        [Linker error] undefined reference to `WSAStartup@8'
//#         pragma comment(lib, "wsock32.lib")

//
// You can always do it the old fashioned way with dev C++
//
// In version 4.0 Dev C++
// From the main menu select Project-> Project Options
// In the dialog box: Further object files or linker options enter
//     -lwsock32
// Note: it is minus letter-L not minus ONE
//
// In version 4.9.9.2
// The linker option is in the tab parameters
// Project->Project Options -> Parameters (tab) -> Linker pane

#else
#	include <arpa/inet.h>
#	include <sys/socket.h>
#	include <unistd.h>
#endif


#define MAXBUFLEN 10000

char sockBuffer[MAXBUFLEN];
int sendMsg(int socket, char *msg);
int getMsg(int socket, char *msgBuf);
int initSocket(char *ServIP, unsigned short ServPort);

#endif
