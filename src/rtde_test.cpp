#include <ros/ros.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <math.h>
#include <mutex>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>//注意。itoa函数要包含这个头文件
#include "arm_control/RobotData.h"

using namespace ur_rtde;

RTDEControlInterface rtde_control("192.168.100.103");
RTDEReceiveInterface rtde_receive("192.168.100.103");

arm_control::RobotData get_Parameter() {
    arm_control::RobotData urData;
    urData.actualTCPPose = rtde_receive.getActualTCPPose();
    urData.targetTCPPose = rtde_receive.getTargetTCPPose();
    urData.actualTCPSpeed = rtde_receive.getActualTCPSpeed();
    urData.targetTCPSpeed = rtde_receive.getTargetTCPSpeed();
    urData.actualTCPForce = rtde_receive.getActualTCPForce();
    urData.actualToolAccel = rtde_receive.getActualToolAccelerometer();
    urData.digitalInputBits = rtde_receive.getActualDigitalInputBits();
    return urData;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "rtde_test");

    arm_control::RobotData urData;

    ros::NodeHandle n;
    // ros::Subscriber URMoveSub = n.subscribe("ur_control", 100, &UR_Control::urContralCallback, &urR);   //特别注意Subscribe的第四个参数用法,这里的消息由hrc_vf_main与hrc_vf_main发布
    ros::Duration(3.0).sleep(); // 延迟第一条数据的发送，确保publisher在roscore注册完毕
    ros::Rate loop_rate(0.5);

    while(ros::ok()) {
        urData = get_Parameter();
        printf("urData.actualTCPForce = %2f, %2f, %2f, %2f, %2f, %2f\n",urData.actualTCPForce[0], urData.actualTCPForce[1], urData.actualTCPForce[2], urData.actualTCPForce[3], urData.actualTCPForce[4], urData.actualTCPForce[5]);
        printf("urData.actualTCPPose = %2f, %2f, %2f, %2f, %2f, %2f\n",urData.actualTCPPose[0], urData.actualTCPPose[1], urData.actualTCPPose[2], urData.actualTCPPose[3], urData.actualTCPPose[4], urData.actualTCPPose[5]);
        loop_rate.sleep();
        sleep(1);
    }
    
    return 0;
}