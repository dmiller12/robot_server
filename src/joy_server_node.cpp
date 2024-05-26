#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include "joy_server/udpip_server_simple.cpp"

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

    ros::Publisher joy_pub = nh.advertise<sensor_msgs::Joy>("joy", 10);
    ros::Rate loop_rate(20);

    double deadzone;

    nh.param<double>("deadzone", deadzone, 0.05);

    while (ros::ok()) {

        int num_rec_bytes = udp_simple_server_check_for_received_datagram(
            receive_buffer, RECEIVE_BUFFER_SIZE, remote_source_ip_address, &remote_source_portnum, 1);
        if (num_rec_bytes > 0) {
            receive_buffer[num_rec_bytes] = 0; // make sure C-string is terminated
            printf("rec %d bytes from %s:%d <%s>\n", num_rec_bytes, remote_source_ip_address, remote_source_portnum,
                   receive_buffer);

            std::string receive_buffer_str(reinterpret_cast<char *>(receive_buffer));
            sensor_msgs::Joy joy_msg;
            joy_msg.header.stamp = ros::Time::now();
            std::vector<float> axes = {0, 0, 0, 0, 0, 0, 0, 0};
            std::vector<int32_t> buttons(13, 0);

            const char *cstr = receive_buffer_str.c_str();

            std::sscanf(cstr, "oculus control left %1d X%f Y%f T%f G%f b%1d%1d%1d right %1d X%f Y%f T%f G%f b%1d%1d%1d",
                        &buttons[4], &axes[0], &axes[1], &axes[3], &axes[7], &buttons[1], &buttons[2], &buttons[0],
                        &buttons[5], &axes[2], &axes[5], &axes[4], &axes[6], &buttons[3], &buttons[6], &buttons[7]);
            joy_msg.header.frame_id = "/dev/occulus";
            axes[0] = -1.0 * axes[0];

            if (axes[7] > 0.8) {
                buttons[8] = 1;
            } else {
                buttons[8] = 0;
            }

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
