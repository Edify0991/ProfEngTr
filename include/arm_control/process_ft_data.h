#ifndef PROCESS_FT_DATA_H
#define PROCESS_FT_DATA_H
#include <ros/ros.h>
#include <iostream>
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
#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <map>
#include <geometry_msgs/WrenchStamped.h>
#include "arm_control/RobotData.h"
#include "arm_control/ProcessedFTData.h"
#include "arm_control/process_robot_data.h"
#include "arm_control/generic_api.h"

using namespace ur_rtde;

extern Transfer_Matrix *transM_P;

# define FILTER_A {1.0, \
  -2.914871217488567811670918672461993992328643798828125,\
  2.83634174488836254823809213121421635150909423828125,\
  -0.92127787928746107670718856752500869333744049072265625}
# define FILTER_B {0.0000240810140417213602859192178584635257720947265625,\
  0.0000722430421251640808577576535753905773162841796875,\
  0.0000722430421251640808577576535753905773162841796875,\
  0.0000240810140417213602859192178584635257720947265625}

typedef struct
{
  double G = 0; // 
  double U = 0; // 安装倾角U
  double V = 0; // 安装倾角V
  Eigen::Matrix<double,3,1> F0 = {0,0,0}; // 力传感器零点力
  Eigen::Matrix<double,3,1> T0 = {0,0,0}; // 力传感器零点力矩
  Eigen::Matrix<double,3,1> P = {0,0,0};  // 重心在传感器坐标系下坐标
  Eigen::Matrix<double,3,1> A = {0,0,0};  // 
}Gravity_Compensation_Args;

typedef struct
{
  Eigen::Matrix<double,6,1> mean;
  Eigen::Matrix<double,6,1> stdev;
  std::deque<Eigen::Matrix<double,6,1>> Deque;
}FT_Data_Analyze;

class Process_FT_Data
{
private:
  std::mutex ft_mutex;
  std::deque<Eigen::Matrix<double,6,1>> origin_ft;  // 原始力传感器数据
  std::deque<Eigen::Matrix<double,6,1>> filtered_ft;  // 滤波后力传感器数据
  std::deque<Eigen::Matrix<double,6,1>> origin_gtced_ft;  // 原始力传感器数据经过重力补偿
  std::deque<Eigen::Matrix<double,6,1>> filtered_gtced_ft;  // 滤波后力传感器数据经过重力补偿
  std::deque<Eigen::Matrix<double,6,1>> origin_world; // 笛卡尔空间下原始力传感器数据
  std::deque<Eigen::Matrix<double,6,1>> filtered_world; // 笛卡尔空间下滤波后力传感器数据
  std::deque<Eigen::Matrix<double,6,1>> origin_gtced_world; // 笛卡尔空间下原始力传感器数据经过重力补偿
  std::deque<Eigen::Matrix<double,6,1>> filtered_gtced_world; // 笛卡尔空间下滤波后力传感器数据经过重力补偿

public:
  const unsigned long FTDataBuffer = 384;

  bool FTNeedCalibration = true;
  std::string subName;
  std::string pubName;
  ros::NodeHandle n;
  ros::Subscriber sub;
  ros::Publisher pub;
  arm_control::ProcessedFTData pubFTMsg;
  Gravity_Compensation_Args GCArgs;
  Eigen::Matrix<double,6,1> staticFrictionFT = (Eigen::MatrixXd(6,1) << 1.0,1.0,1.0,0.1,0.1,0.1).finished();
  arm_control::RobotData urData;

  Process_FT_Data(ros::NodeHandle& node_handle, const std::string sub_name, const std::string pub_name);

  void ftDataCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg);

  Eigen::Matrix<double,6,1> transFT_World(const Eigen::Matrix<double,6,1>& FTData, Transfer_Matrix* T);

  std::deque<Eigen::Matrix<double,6,1>> getFTDeque(const char *FTType);
  Eigen::Matrix<double,6,1> getFTData(const char *FTType);

  //设置重力补偿参数
  void setGCArgs(std::string toolName);

  //分析力传感器数据队列中的数据
  FT_Data_Analyze analyzeFTDataQueue(const char *FTType);

  //校准力传感器
  bool setFTDateZeroError(const double maxStdevSum = 0.02);

  //力传感器滤波
  Eigen::Matrix<double,6,1> filter(
      const std::deque<Eigen::Matrix<double,6,1>> &origenData,
      const std::deque<Eigen::Matrix<double,6,1>> &processedData,
      const std::vector<double> a = FILTER_A,
      const std::vector<double> b = FILTER_B);

  //工具,力传感器重力补偿
  //返回重力补偿值，用当前力传感器值减去重力补偿值即可得到重力补偿后的值
  Eigen::Matrix<double,6,1> gravityCompensation(
      const Gravity_Compensation_Args &GCArgs,
      const Eigen::Matrix<double,3,3> R_robot_ft);

  //虚拟静摩擦力 消除力动传感器零力附近波
  Eigen::Matrix<double,6,1> virtualStaticFrictionProcess(
      Eigen::Matrix<double,6,1> &origenData,
      Eigen::Matrix<double,6,1> frictionFT);
};

#endif // PROCESS_FT_DATA_H
