#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include "udpip_server_simple.c"

#define PORT_NUM 20100
#define RECEIVE_BUFFER_SIZE 2048

int main(int argc, char **argv) {
    unsigned char receive_buffer[RECEIVE_BUFFER_SIZE + 1];
    char remote_source_ip_address[128];
    int remote_source_portnum = -1;

    if (udpip_open_simple_server_nonblocking(PORT_NUM, 0)) // last arg is debug  -returns 0 if ok, -1 if problem
    {
        printf("ERROR attempting to open server socket on port %d\n", PORT_NUM);
        exit(1);
    }
    printf("Listening for UDP datagrams on port %d\n", PORT_NUM);

    ros::init(argc, argv, "joy_server");
    ros::NodeHandle nh;

    ros::Publisher joy_pub = nh.advertise<sensor_msgs::Joy>("joy_teleop/joy", 10);
    ros::Rate loop_rate(20);

    while (ros::ok()) {

        int num_rec_bytes = udp_simple_server_check_for_received_datagram(
            receive_buffer, RECEIVE_BUFFER_SIZE, remote_source_ip_address, &remote_source_portnum, 1);
        if (num_rec_bytes > 0) {
            receive_buffer[num_rec_bytes] = 0; // make sure C-string is terminated
            printf("rec %d bytes from %s:%d <%s>\n", num_rec_bytes, remote_source_ip_address, remote_source_portnum,
                   receive_buffer);

            sensor_msgs::Joy joy_msg;
            joy_msg.header.stamp = ros::Time::now();
            std::string receive_buffer_str(reinterpret_cast<char *>(receive_buffer));

            joy_msg.header.frame_id = receive_buffer_str;

            std::vector<float> axes = {0, 0, 0, 0, 0, 0, 0, 0};
            std::vector<int> buttons(13, 0);
            buttons[4] = 1;

            joy_msg.axes = axes;
            joy_msg.buttons = buttons;

            joy_pub.publish(joy_msg);

            memset(receive_buffer, 0,
                   RECEIVE_BUFFER_SIZE); // clear to all 0's so we don't confuse parts of previous packets

            loop_rate.sleep();
        }
    }
    udpip_close_simple_server();
    printf("Closed server socket on port %d\n", PORT_NUM);

    return 0;
}
