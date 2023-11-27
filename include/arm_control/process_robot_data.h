#ifndef PROCESS_ROBOT_DATA_H
#define PROCESS_ROBOT_DATA_H

#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <deque>
#include <math.h>
#include <mutex>
#include <map>

#include <Eigen/Dense>
#include <arm_control/RobotData.h>

# define INITPOSE {0.2, -0.5, 0.3, 0.0, 3.14, 0.0}  // 机械臂运动到的初始位姿，待调整！
#define T_WORLD_ROBOT { \
  1,0,0,0, \
  0,1,0,0, \
  0,0,1,0, \
  0,0,0,1}

#define T_FLANGE_FT { \
  1,0,0,0, \
  0,1,0,0, \
  0,0,1,0,  \
  0,0,0,1}

#define T_FT_TCP { \
  1,0,0,0, \
  0,1,0,0, \
  0,0,1,0, \
  0,0,0,1}

class Transfer_Matrix
{
public:
  Eigen::Matrix<double,3,3> R_world_robot;
  Eigen::Matrix<double,4,4> T_world_robot;
  Eigen::Matrix<double,3,3> R_robot_flange;
  Eigen::Matrix<double,4,4> T_robot_flange;
  Eigen::Matrix<double,3,3> R_flange_ft;
  Eigen::Matrix<double,4,4> T_flange_ft;
  Eigen::Matrix<double,3,3> R_ft_tcp;
  Eigen::Matrix<double,4,4> T_ft_tcp;
  Transfer_Matrix();
  void set_TR_robot_flange(Eigen::Matrix<double,6,1> actualFlangePose);
};

extern Transfer_Matrix transM;

typedef struct
{
  Eigen::Matrix<double,6,1> mean;
  Eigen::Matrix<double,6,1> stdev;
  std::deque<Eigen::Matrix<double,6,1>> Deque;
}Robot_Data_Analyze;

class Process_Robot_Data
{
private:
  std::mutex robot_mutex;
  const unsigned long robotDataBuffer = 50;

  uint64_t digitalInput;

  Eigen::Matrix<double,6,1> actualFlangePose;
  Eigen::Matrix<double,6,1> targetFlangePose;
  Eigen::Matrix<double,6,1> actualFlangeSpeed;
  Eigen::Matrix<double,6,1> targetFlangeSpeed;
  Eigen::Matrix<double,6,1> actualFlangeAccel;

  std::deque<Eigen::Matrix<double,6,1>> actualFlangePoseDeque;
  std::deque<Eigen::Matrix<double,6,1>> targetFlangePoseDeque;
  std::deque<Eigen::Matrix<double,6,1>> actualFlangeSpeedDeque;
  std::deque<Eigen::Matrix<double,6,1>> targetFlangeSpeedDeque;
  std::deque<Eigen::Matrix<double,6,1>> actualFlangeAccelDeque;

public:
  void robotDataCallback(const arm_control::RobotData &msg);
  uint64_t getDigitalInput(void);
  Eigen::Matrix<double,6,1> getRobotData(const char *dataType);
  std::deque<Eigen::Matrix<double,6,1>> getRobotDataDeque(const char *dataType);
  //检测机器人是否到达目标位置
  bool isReachTargetPose(Eigen::Matrix<double,6,1> targetPose, double error);
  Robot_Data_Analyze analyzeRobotDataQueue(const char *dataType);
};

typedef struct
{
  std::vector<double> _initPose = INITPOSE;
  Eigen::Matrix<double,6,1> pose = Eigen::Map<Eigen::MatrixXd>(&_initPose[0], 6, 1);
  double speed = 0;
  double accel = 0.5;
}Target_Pose;

class Robot_Move{
private:

public:
  void sendPoseMsg(ros::Publisher  urConParPub, Eigen::Matrix<double,6,1> pose);
  void sendSpeedLMsg(ros::Publisher  urConParPub, Eigen::Matrix<double,6,1> speedL);
  void sendteachMsg(ros::Publisher urConParPub,bool teachModeFlag);
  void sendMoveLMsg(ros::Publisher  urConParPub, Eigen::Matrix<double,6,1> pose);
};


//欧拉角转换为四元数
Eigen::Quaterniond euler_quaternion(Eigen::Vector3d eulerAngle);


//四元数转换为旋转矢量
Eigen::Matrix<double,3,1> quaternion_rotationVector(Eigen::Quaterniond quaternion);


//旋转矢量到四元数
Eigen::Quaterniond rotationVector_quaternion(Eigen::Matrix<double,3,1> rotationVector);


//四元数转换为旋转矩阵
Eigen::Matrix3d quaternion_rotationMatrix(Eigen::Quaterniond quaternion);

//旋转矩阵转换为欧拉角
//ZYX顺序，即先绕x轴roll,再绕y轴pitch,最后绕z轴yaw,0表示X轴,1表示Y轴,2表示Z轴
Eigen::Vector3d rotationMatrix_euler(Eigen::Matrix3d rotationMatrix);
Eigen::Vector3d  rotationMatrix_RPY(Eigen::Matrix3d rotationMatrix);

//RPY转换为旋转矩阵
Eigen::Matrix3d RPY_rotationMatrix(Eigen::Vector3d RPY);

//eulerXYZ转换为四元数
Eigen::Quaterniond eulerXYZ_quaternion(Eigen::Vector3d RPY);

//旋转矢量转换为RPY
Eigen::Vector3d rotationVector_RPY(Eigen::Matrix<double,3,1> rotationVector);

//用轴角xyz表示两个四元数的差角 q1 - q2
Eigen::Vector3d quaternionsDiff_eulerXYZ(Eigen::Quaterniond q1, Eigen::Quaterniond q2);
//用轴角xyz表示两个旋转矢量的差角 v1 - v2
Eigen::Vector3d rotationVectorDiff_eulerXYZ(Eigen::Vector3d v1, Eigen::Vector3d v2);

#endif // PROCESS_ROBOT_DATA_H
