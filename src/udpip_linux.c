//TCPIP_LINUX  - July 10/2003
//also include one of the following;
//#include "tcpip_server.c"
//#include "tcpip_client.c"
//#include "tcpip_functions.c"

//functions
//
//tcpip_init();
//
//tcpip_close_socket(skt);


#ifndef _UDPIP_LINUX_C_
 #define _UDPIP_LINUX_C_

#include <stdlib.h>
#include <errno.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
 
//#include <stdio.h> 
#include <unistd.h>
#include <string.h>

//from WINSOCK2.H
typedef struct sockaddr_in SOCKADDR_IN;
typedef struct sockaddr    SOCKADDR;
typedef u_int              SOCKET;
#define SOCKET_ERROR    (-1)

//udpip_init();
//just a dummy in linux to sit in for mswindows needing a call first
int udpip_init(char debug)
{
return 0;
}



//udpip_close_socket(skt);
int udpip_close_socket(unsigned int skt)
{
int rv;
rv=close(skt);
return rv;
}

int udpip_recv(unsigned int skt,char *buffer, int len, int flags)
{
int rv=recv(skt,(char*)buffer,len,flags);
//if(rv==SOCKET_ERROR) rv=0;
/*
int recv (
  SOCKET s,
  char FAR* buf,
  int len,
  int flags
*/
return rv;
}


//udpip_close(1);  //arg=debug 1=printf, 0=silent
//just a dummy in linux to sit in for mswindows' WSACleanup()
void udpip_close(char debug)
{
}





#endif    //#ifndef _UDPIP_LINUX_C_


