// udpip_server_simple.c -Nov 19/2023 -simple 1-connection only UDPIP server

//--------------Simple UDP-IP server functions---------------
// if(udpip_open_simple_server_nonblocking(port_num,0))  {problem...  //last arg is debug  -returns 0 if ok, -1 if
// problem

// int num_rec_bytes=udp_simple_server_check_for_received_datagram(receive_buffer,
// RECEIVE_BUFFER_SIZE,remote_source_ip_address,&remote_source_portnum,1); if(num_rec_bytes>0)
//    {
//    printf("rec %d bytes from %s:%d
//    <%s>\n",num_rec_bytes,remote_source_ip_address,remote_source_portnum,receive_buffer);
//    memset(receive_buffer,0,RECEIVE_BUFFER_SIZE); //clear to all 0's so we don't get bits of previous packets
//    }

// if(udp_simple_server_send_datagram_to_open_socket((unsigned
// char*)message,strlen(message)+1,ip_address_string,port_num,1))  {problem...  //ret 0=ok, -1=problem last arg is debug

// udpip_close_simple_server();

#ifndef _UDPIP_SERVER_SIMPLE_C_
#define _UDPIP_SERVER_SIMPLE_C_

#include <cstdio>
#include <netinet/in.h>

#include "udpip_linux.c"

#ifdef _WIN32
#define WINDOWS_VERSION
#endif

unsigned int udpip_server_simple_socket;
struct sockaddr_in udpip_server_simple_sockaddr;

// if(udpip_open_simple_server_nonblocking(port_num,0))  {problem...  //last arg is debug
// returns 0 if ok, -1 if problem
char udpip_open_simple_server_nonblocking(int port_num, char debug) {
    struct sockaddr_in servaddr = {0};
    int rc;
#ifdef _WIN32
    unsigned long mode;
#endif

    // open socket
    // ReceivingSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    udpip_server_simple_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP); // for TCP was socket(AF_INET,SOCK_STREAM,0);
    if (udpip_server_simple_socket == -1) {
        if (debug)
            printf("ERROR: udpip_open_simple_server_nonblocking()  failed to create UDP socket\n");
        return -1;
    }

// Set socket to non-blocking mode
#ifdef _WIN32
    mode = 1;
    ioctlsocket(udpip_server_simple_socket, FIONBIO, &mode);
#else
    fcntl(udpip_server_simple_socket, F_SETFL, O_NONBLOCK);
#endif

    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons((unsigned short)port_num);
    servaddr.sin_addr.s_addr = INADDR_ANY;

    rc = bind(udpip_server_simple_socket, (const struct sockaddr *)&servaddr, sizeof(servaddr));
    if (rc == -1) {
        if (debug)
            printf("failed to bind UDP socket to port %d\n", port_num);
        // Close the socket
        udpip_close_socket(udpip_server_simple_socket); // assume error free
        /*
        #ifdef _WIN32
         closesocket(udpip_server_simple_socket);
        #else
         close(udpip_server_simple_socket);
        #endif
        */
        return -1;
    }
    if (debug)
        printf("Opened simple UDP server server (called 'bind') socket on port %d\n", port_num);

    return 0;
}

// if(udpip_close_connection(server_con)) {problem...
// udpip_close_simple_server();
char udpip_close_simple_server(void) {
    udpip_close_socket(udpip_server_simple_socket); // assume error free
    return 0;
}

// 1=num_bytes received, 0=no data
int udp_simple_server_check_for_received_datagram(unsigned char *rec_buffer, int max_rec_buffer_len,
                                                  char *client_ip_address, int *client_port_num, char debug) {
    struct sockaddr_in clientaddr;
    socklen_t client_len = sizeof(clientaddr); // Length of client address

    int num_bytes = recvfrom(udpip_server_simple_socket, (char *)rec_buffer, max_rec_buffer_len, 0,
                             (struct sockaddr *)&clientaddr, &client_len);
    // printf("Received %d bytes from recvfrom()\n",n);
    if (num_bytes >= 0) {
        // Get the sender's IP address
        strcpy(client_ip_address, inet_ntoa(clientaddr.sin_addr));
        *client_port_num = ntohs(clientaddr.sin_port);
        if (num_bytes + 1 < max_rec_buffer_len)
            rec_buffer[num_bytes] = '\0'; // Null-terminate the buffer
        // printf("Received <%s> from <%s:%d>\n", buffer,senderIP,clientPort);
    }
    if (num_bytes == -1) {
        char problem = 0;
#ifdef _WIN32
        int errCode = WSAGetLastError();
        if ((errCode != WSAEWOULDBLOCK) &&
            (errCode != WSAECONNRESET)) // ignore 10054=WSAECONNRESET=from previous error with sendto()
            problem = 1;
#else
        int errCode = errno;
        if ((errno != EAGAIN) && (errno != EWOULDBLOCK))
            problem = 1;
#endif
        if (problem) {
            if (debug)
                printf("recvfrom failed with error: %d\n", errCode);
        }
        num_bytes = 0;
    }
    return num_bytes;
}

// if(udp_simple_server_send_datagram_to_open_socket((unsigned
// char*)message,strlen(message)+1,ip_address_string,port_num,1)!=strlen(message)+1)  {problem... last arg is debug
int udp_simple_server_send_datagram_to_open_socket(unsigned char *packet, int packet_len, char *ip_address_string,
                                                   int port_num, char debug) {
    int len;
    struct sockaddr_in servaddr = {0};
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons((unsigned short)port_num);
    servaddr.sin_addr.s_addr = inet_addr(ip_address_string);

    len = sendto(udpip_server_simple_socket, (const char *)packet, packet_len, 0, (const struct sockaddr *)&servaddr,
                 sizeof(servaddr));
    if (len == -1) {
        if (debug)
            printf("udp_simple_server_send_datagram_to_open_socket() failed to send %d byte size packet\n", packet_len);
        return -1;
    }
    return len;
}

#endif // #ifndef _UDPIP_SERVER_SIMPLE_C_
