#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include "arm_control/RobotData.h"
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>//注意。itoa函数要包含这个头文件

using namespace ur_rtde;
using namespace std::chrono;

class  UR_Data{
    private:
    RTDEReceiveInterface rtdeReceive;
    arm_control::RobotData urData;
    public:
    UR_Data(std::string IPAdress = "127.0.0.1"):rtdeReceive(IPAdress)
    {}


    arm_control::RobotData get_Parameter(void)
    {
        urData.actualTCPPose = rtdeReceive.getActualTCPPose();
        urData.targetTCPPose = rtdeReceive.getTargetTCPPose();
        urData.actualTCPSpeed = rtdeReceive.getActualTCPSpeed();
        urData.targetTCPSpeed = rtdeReceive.getTargetTCPSpeed();
        urData.actualTCPForce = rtdeReceive.getActualTCPForce();
        urData.actualToolAccel = rtdeReceive.getActualToolAccelerometer();
        urData.digitalInputBits = rtdeReceive.getActualDigitalInputBits();
        //    printf("urData.actualTCPSpeed = %2f\n",urData.actualTCPSpeed[0]);
        return urData;
    }
    void publishMsg(ros::Publisher publisher, arm_control::RobotData msg)
    {
        msg.header.stamp = ros::Time::now();
        publisher.publish(msg);
    }
};


int main(int argc, char **argv)
{
    UR_Data ur10e("192.168.100.103");
    ros::init(argc, argv, "ur_data");

    ros::NodeHandle n;

    ros::Publisher urDataPub = n.advertise<arm_control::RobotData>("ur_data", 1);

    ros::Rate loop_rate(128);

    while(ros::ok())
    {
        ur10e.publishMsg(urDataPub, ur10e.get_Parameter());
        loop_rate.sleep();
    }
    return 0;
}
