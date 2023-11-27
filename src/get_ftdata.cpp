#include <ros/ros.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <math.h>
#include <mutex>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <stdio.h>
#include <geometry_msgs/WrenchStamped.h>
#include <stdlib.h>
#include <cstdlib>//注意。itoa函数要包含这个头文件

using namespace ur_rtde;

RTDEReceiveInterface rtde_receive("192.168.100.103");

geometry_msgs::WrenchStamped get_Parameter() {
    std::vector<double> urData;
    geometry_msgs::WrenchStamped urDataMsg;
    urData = rtde_receive.getActualTCPForce();
    urDataMsg.wrench.force.x = urData[0];
    urDataMsg.wrench.force.y = urData[1];
    urDataMsg.wrench.force.z = urData[2];
    urDataMsg.wrench.torque.x = urData[3];
    urDataMsg.wrench.torque.y = urData[4];
    urDataMsg.wrench.torque.z = urData[5];
    urDataMsg.header.stamp = ros::Time::now();
    return urDataMsg;
}

void publishMsg(ros::Publisher publisher, geometry_msgs::WrenchStamped msg)
{
    msg.header.stamp = ros::Time::now();
    publisher.publish(msg);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "get_ftdata");

    ros::NodeHandle n;
    ros::Publisher urDataPub = n.advertise<geometry_msgs::WrenchStamped>("ur_ftdata", 1);

    ros::Duration(3.0).sleep(); // 延迟第一条数据的发送，确保publisher在roscore注册完毕
    ros::Rate loop_rate(150);

    while(ros::ok()) {
        publishMsg(urDataPub, get_Parameter());
        // printf("urData.actualTCPForce = %2f, %2f, %2f, %2f, %2f, %2f\n",urData.actualTCPForce[0], urData.actualTCPForce[1], urData.actualTCPForce[2], urData.actualTCPForce[3], urData.actualTCPForce[4], urData.actualTCPForce[5]);
        // printf("urData.actualTCPPose = %2f, %2f, %2f, %2f, %2f, %2f\n",urData.actualTCPPose[0], urData.actualTCPPose[1], urData.actualTCPPose[2], urData.actualTCPPose[3], urData.actualTCPPose[4], urData.actualTCPPose[5]);
        loop_rate.sleep();
    }
    
    return 0;
}