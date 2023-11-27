#ifndef RECORD_NODE_H
#define RECORD_NODE_H
#include <unistd.h>
#include "ros/ros.h"
#include <deque>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include "arm_control/RobotControl.h"
#include "arm_control/RobotData.h"
#include "arm_control/ProcessedFTData.h"


class Record_Robot_Data{
private:

public:
  double beginTime;
  long int updateFlag = 0;
  Eigen::Matrix<double,1,8> actualPose;
  Eigen::Matrix<double,1,8> targetPose;
  Eigen::Matrix<double,1,8> actualSpeed;
  Eigen::Matrix<double,1,8> targetSpeed;
  Eigen::Matrix<double,1,8> actualAccel;

  std::deque<Eigen::Matrix<double,1,8>> actualPoseDeque;
  std::deque<Eigen::Matrix<double,1,8>> targetPoseDeque;
  std::deque<Eigen::Matrix<double,1,8>> actualSpeedDeque;
  std::deque<Eigen::Matrix<double,1,8>> targetSpeedDeque;
  std::deque<Eigen::Matrix<double,1,8>> actualAccelDeque;

  void urDataCallback(const arm_control::RobotData &msg);
};

class Record_FT_Data{
public:

  long int updateFlag = 0;
  double beginTime;
  Eigen::Matrix<double,1,8> origen_ft;
  Eigen::Matrix<double,1,8> filtered_ft;
  Eigen::Matrix<double,1,8> origen_gtced_ft;
  Eigen::Matrix<double,1,8> filtered_gtced_ft;
  Eigen::Matrix<double,1,8> origen_world;
  Eigen::Matrix<double,1,8> filtered_world;
  Eigen::Matrix<double,1,8> origen_gtced_world;
  Eigen::Matrix<double,1,8> filtered_gtced_world;

  std::deque<Eigen::Matrix<double,1,8>> origen_ft_Deque;
  std::deque<Eigen::Matrix<double,1,8>> filtered_ft_Deque;
  std::deque<Eigen::Matrix<double,1,8>> origen_gtced_ft_Deque;
  std::deque<Eigen::Matrix<double,1,8>> filtered_gtced_ft_Deque;
  std::deque<Eigen::Matrix<double,1,8>> origen_world_Deque;
  std::deque<Eigen::Matrix<double,1,8>> filtered_world_Deque;
  std::deque<Eigen::Matrix<double,1,8>> origen_gtced_world_Deque;
  std::deque<Eigen::Matrix<double,1,8>> filtered_gtced_world_Deque;
  //力传感器 kw_ft_data topic的消息回调函数
  void ftDataCallback(const arm_control::ProcessedFTData &msg);
};

class Record_Ctrl_Data{
public:
  double beginTime;
  long int updateFlag = 0;
  Eigen::Matrix<double,1,8> poseData;
  std::deque<Eigen::Matrix<double,1,8>> poseDataDeque;
  Eigen::Matrix<double,1,8> speedData;
  std::deque<Eigen::Matrix<double,1,8>> speedDataDeque;
  void urContralCallback(const arm_control::RobotControl &msg);
};



class Record_Mode_Data{
private:

public:
  double beginTime;
  long int updateFlag = 0;
  int mode;
  std::deque<double> timeDeque;
  std::deque<int> modeDeque;
  void modeDataCallback(const std_msgs::Int32 &msg);
};
#endif // RECORD_NODE_H
