#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ros/duration.h>
#include <ros/ros.h>

#include "robot_server/udpip_server_simple.cpp"

#define DEFAULT_SERVER_PORT_NUM 20101
#define RECEIVE_BUFFER_SIZE 4096

class RobotStateServer {
  public:
    RobotStateServer();

  private:
    ros::NodeHandle nh;
    unsigned char receive_buffer[RECEIVE_BUFFER_SIZE + 1];
    char remote_source_ip_address[128];
    int remote_source_portnum = -1;
    ros::Subscriber follower_jointstate_sub;
    sensor_msgs::JointState follower_joint_state;
    ros::Time last_msg_time;
    float timeout_seconds = 0.5;

    void follower_cb(const sensor_msgs::JointState &_msg);
    Eigen::Matrix<double, 4, 4> computeCameraPose(std::vector<double> joint_positions);
};

RobotStateServer::RobotStateServer() {
    follower_jointstate_sub = nh.subscribe("/follower/joint_states", 1, &RobotStateServer::follower_cb, this);

    if (udpip_open_simple_server_nonblocking(DEFAULT_SERVER_PORT_NUM, 0)) {
        printf("ERROR attempting to open server socket on port %d\n", DEFAULT_SERVER_PORT_NUM);
        exit(1);
    }
    printf("Listening for UDP datagrams on port %d\n", DEFAULT_SERVER_PORT_NUM);

    Eigen::Matrix4d pose;
    std::vector<double> home_pos = {-0.001899214305, -1.860254709,     0.0009578542575, 3.117813375,
                                    0.09622961953,   -7.907117463e-05, -0.006883905746};

    // initialize follower_joint_state
    follower_joint_state.position.resize(7);
    follower_joint_state.position = home_pos;

    // initialize last_msg_time so the elaspsed time exceeds the timeout
    last_msg_time = ros::Time::now() - ros::Duration(timeout_seconds + 1);

    ros::Rate loop_rate(100);

    while (ros::ok()) {
        ros::spinOnce();

        int num_rec_bytes = udp_simple_server_check_for_received_datagram(
            receive_buffer, RECEIVE_BUFFER_SIZE, remote_source_ip_address, &remote_source_portnum, 1);
        if (num_rec_bytes > 0) {
            int num_bytes_to_send;
            // generate C-string of data to send back
            char husky_wam_data_str[2048];

            ros::Duration elasped_time = ros::Time::now() - last_msg_time;
            if (elasped_time.toSec() > timeout_seconds) {
                ROS_INFO("Joint position not received in last %0.2f seconds, using home position", timeout_seconds);
                pose = computeCameraPose(home_pos);
            } else {
                pose = computeCameraPose(follower_joint_state.position);
            }
            sprintf(husky_wam_data_str, "HUSKY WAM %2f %2f %2f %2f %2f %2f %2f %2f %2f %2f %2f %2f %2f %2f %2f %2f",
                    pose(0, 0), pose(0, 1), pose(0, 2), pose(0, 3), pose(1, 0), pose(1, 1), pose(1, 2), pose(1, 3),
                    pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3), pose(3, 0), pose(3, 1), pose(3, 2), pose(3, 3));
            num_bytes_to_send = strlen(husky_wam_data_str) + 1;
            printf("sending <%s> (%d bytes) back to %s:%d\n", husky_wam_data_str, num_bytes_to_send,
                   remote_source_ip_address, remote_source_portnum);
            if (udp_simple_server_send_datagram_to_open_socket((unsigned char *)husky_wam_data_str, num_bytes_to_send,
                                                               remote_source_ip_address, remote_source_portnum,
                                                               1) != num_bytes_to_send)
                printf("***WARNING  problem sending datagram to %s:%d\n", remote_source_ip_address,
                       remote_source_portnum);

            memset(receive_buffer, 0,
                   RECEIVE_BUFFER_SIZE); // clear to all 0's so we don't confuse parts of previous packets

            loop_rate.sleep();
        }
    }
    udpip_close_simple_server();
    printf("Closed server socket on port %d\n", DEFAULT_SERVER_PORT_NUM);
}

void RobotStateServer::follower_cb(const sensor_msgs::JointState &_msg) {
    follower_joint_state = _msg;
    last_msg_time = ros::Time::now();
}

Eigen::Matrix4d dhTransform(double a, double alpha, double d, double theta) {
    Eigen::Matrix4d T;
    T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta), sin(theta),
        cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta), 0, sin(alpha), cos(alpha), d, 0, 0, 0, 1;
    return T;
}

struct DHParameters {
    double a;
    double alpha;
    double d;
    double theta;
};

Eigen::Matrix<double, 4, 4> RobotStateServer::computeCameraPose(std::vector<double> joint_positions) {
    // DH parameters
    std::vector<DHParameters> dh_params = {{0, -M_PI / 2, 0, 0}, // ignore j1 angle, fixed relative to ladybug
                                           {0, M_PI / 2, 0, joint_positions[1]},
                                           {0.045, -M_PI / 2, 0.55, joint_positions[2]},
                                           {-0.045, M_PI / 2, 0, joint_positions[3]}};
    // frame 0 to ladybug fixed transform
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(1, 3) = -0.18; // y offset
    T(2, 3) = -0.33; // z offset

    for (const auto &params : dh_params) {
        T *= dhTransform(params.a, params.alpha, params.d, params.theta);
    }

    // Camera to frame 4 -> rotate about y -90 degrees
    Eigen::Matrix4d T_cam_to_4 = Eigen::Matrix4d::Identity();
    T_cam_to_4(0, 0) = cos(-M_PI / 2);
    T_cam_to_4(0, 2) = sin(-M_PI / 2);
    T_cam_to_4(2, 0) = -sin(-M_PI / 2);
    T_cam_to_4(2, 2) = cos(-M_PI / 2);
    T_cam_to_4(0, 3) = -0.065; // x offset
    T_cam_to_4(2, 3) = 0.275;  // z offset
    T *= T_cam_to_4;

    return T;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robotstate_server");
    RobotStateServer robotstate_server;
    return 0;
}
