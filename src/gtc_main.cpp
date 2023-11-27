#include <iostream>
#include <fstream>
#include <termios.h>
#include <boost/thread/thread.hpp>
#include <unistd.h>
#include <thread>
#include <chrono>

#include <vector>
#include <deque>
#include <math.h>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>

#include "arm_control/gtc.h"
#include "arm_control/record_node.h"
#include "arm_control/process_robot_data.h"
#include "arm_control/process_ft_data.h"
#include "arm_control/RobotControl.h"
#include "arm_control/RobotData.h"
#include "generic_api.h"

using namespace std;
Eigen::Matrix<double,3,3> R_flange_ft;

static int keyValue;
bool teachMode = false;
bool needCollectData = true;
bool collectionOver = false;
bool needRecordData = true;
bool movingRecordData = false;
#define ESC 27
#define SPACE 32
#define ENTER 10


/***************************************机器人和传感器数据采集***************************************/

//一个位姿下采集一组力传感器数据和一组机器人位姿数据
void dataCollection(Process_FT_Data* KW_FT, Process_Robot_Data* URData, Metrical_Data &data)
{
const double maxSpeedStdevSum = 0.001;
const double maxFTStdevSum = 2.0;

Robot_Data_Analyze robotSpeed = URData->analyzeRobotDataQueue("actualFlangeSpeedDeque");
Robot_Data_Analyze robotPose = URData->analyzeRobotDataQueue("actualFlangePoseDeque");
FT_Data_Analyze FTData = KW_FT->analyzeFTDataQueue("origen_ft");

while(((robotSpeed.stdev.sum() > maxSpeedStdevSum) || (FTData.stdev.sum() > maxFTStdevSum))
  && ros::ok()){
  cout << "数据不合格" << endl;
  cout << "robotSpeed.stdev.sum() = " << robotSpeed.stdev.sum() << endl;
  cout << "FTData.stdev.sum() = " << FTData.stdev.sum() << endl;
  int i = 0;
  while(i < 200 && ros::ok())
  {
    i++;
    usleep(2000);
  }
  robotSpeed = URData->analyzeRobotDataQueue("actualFlangeSpeedDeque");
  robotPose = URData->analyzeRobotDataQueue("actualFlangePoseDeque");
  FTData = KW_FT->analyzeFTDataQueue("origen_ft");
}
data.pose.push_back(robotPose.mean);
data.FT.push_back(FTData.mean);
++data.poseNum;
}

//数据处理，处理后的格式要求按照重力补偿公式设定
void dataFormat(Metrical_Data &data)
{
data.R.resize(data.poseNum * 3,6);
data.F31.resize(data.poseNum * 3,1);
data.F36.resize(data.poseNum * 3,6);
data.T.resize(data.poseNum * 3,1);
Eigen::Matrix<double,3,1> tmpF;
for (unsigned long i=0, j=0; i < data.poseNum; i++, j+=3){
  data.R.block<3, 3>(j, 0) = (quaternion_rotationMatrix(rotationVector_quaternion(data.pose.at(i).block<3, 1>(3, 0))) * R_flange_ft).transpose();
  data.R.block<3, 3>(j, 3) = Eigen::MatrixXd::Identity(3,3);
  tmpF = data.FT.at(i).block<3, 1>(0, 0);
  data.F31.block<3, 1>(j, 0) = tmpF;
  data.F36.block<3, 3>(j, 0) << 0,   tmpF(2), -tmpF(1),
      -tmpF(2),       0,   tmpF(0),
      tmpF(1), -tmpF(0),       0;
  data.F36.block<3, 3>(j, 3) = Eigen::MatrixXd::Identity(3,3);
  data.T.block<3, 1>(j, 0) = data.FT.at(i).block<3, 1>(3, 0);
}
cout << "R " << data.R << endl;
cout << "F31 " << data.F31 << endl;
cout << "F36 " << data.F36 << endl;
cout << "T " << data.T << endl;
}


/*****************************************机器人和力传感器数据采集*************************************/

/*****************************************最小二乘重力补偿算法***************************************/

//矩阵最小二乘:{X}=(A^T * A)^{-1} * A^T * {Y}
Eigen::Matrix<double,6,1> matrixLeastSquares(const Eigen::Matrix<double,Eigen::Dynamic,6> &A, const Eigen::Matrix<double,Eigen::Dynamic,1> &Y)
{
return (A.transpose() * A).inverse() * A.transpose() * Y;
}

//计算工具和力传感器的重力补偿参数
void gravityCompensationArgs(const Eigen::Matrix<double,6,1> &a, const Eigen::Matrix<double,6,1> &b, Gravity_Compensation_Args &Args){

Args.A = a.block<3, 1>(0, 0);
Args.G = Args.A.norm();
Args.U = asin(-a(1) / Args.G);
Args.V = atan(-a(0) / a(2));
Args.F0 << a.block<3, 1>(3, 0);
Args.P << b.block<3, 1>(0, 0);
Eigen::Matrix<double,3,3> tmp;
tmp << 0,   b(2), -b(1),
    -b(2),    0,   b(0),
    b(1), -b(0),    0;
Args.T0 = b.block<3, 1>(3, 0) - tmp * Args.F0;

cout << "A: " << Args.A <<endl;
cout << "G: " << Args.G <<endl;
cout << "U: " << Args.U <<endl;
cout << "V: " << Args.V <<endl;
cout << "P: " << Args.P <<endl;
cout << "F0: " << Args.F0 <<endl;
cout << "T0: " << Args.T0 <<endl;
}

//返回重力补偿值，用当前力传感器值减去重力补偿值即可得到重力补偿后的值
Eigen::Matrix<double,6,1> gravityCompensation(const Eigen::Matrix<double,6,1> &pose, const Gravity_Compensation_Args &GCArgs)
{
Eigen::Matrix<double,6,1> FTc;
Eigen::Matrix<double,3,3> rotationMatrix;
Eigen::Matrix<double,3,1> gx, mgx;
FTc = Eigen::Matrix<double,6,1>::Zero();
rotationMatrix = (quaternion_rotationMatrix(rotationVector_quaternion(pose.block<3, 1>(3, 0))) * R_flange_ft).transpose();
gx = rotationMatrix * GCArgs.A;
mgx << gx(2) * GCArgs.P(1) - gx(1) * GCArgs.P(2),
    gx(0) * GCArgs.P(2) - gx(2) * GCArgs.P(0),
    gx(1) * GCArgs.P(0) - gx(0) * GCArgs.P(1);
FTc.block<3,1>(0,0) = GCArgs.F0 + gx;
FTc.block<3,1>(3,0) = GCArgs.T0 + mgx;
//  cout << "gx" << gx.transpose() << endl;
return FTc;
}
/*****************************************最小二乘重力补偿算法***************************************/


void staticDataRecord(std::string dir, Process_FT_Data* FTData, Process_Robot_Data* robotData, const Gravity_Compensation_Args &compensationArgs)
{
int dataNum = 0;
const double maxSpeedStdevSum = 0.001;
const double maxFTStdevSum = 2.0;

while(ros::ok())
{
  if(!teachMode && needRecordData){
    createFolders((dir + "/"+ to_string(++dataNum)).c_str());
    std::ofstream origrnFTTxt(dir + "/"+ to_string(dataNum)+ "/OrigenFT.txt");
    std::ofstream gtcedFTTxt(dir + "/"+ to_string(dataNum)+ "/gtcedFT.txt");
    std::ofstream robotPoseTxt(dir + "/"+ to_string(dataNum)+ "/robotPose.txt");
    origrnFTTxt.precision(9);
    gtcedFTTxt.precision(9);
    robotPoseTxt.precision(9);

    Robot_Data_Analyze robotSpeed = robotData->analyzeRobotDataQueue("actualFlangeSpeedDeque");
    Robot_Data_Analyze robotPose = robotData->analyzeRobotDataQueue("actualFlangePoseDeque");
    FT_Data_Analyze FTValue = FTData->analyzeFTDataQueue("origen_ft");

    while(((robotSpeed.stdev.sum() > maxSpeedStdevSum) || (FTValue.stdev.sum() > maxFTStdevSum))
      && ros::ok()){
      cout << "数据不合格" << endl;
      cout << "robotSpeed.stdev.sum() = " << robotSpeed.stdev.sum() << endl;
      cout << "FTValue.stdev.sum() = " << FTValue.stdev.sum() << endl;
      int i = 0;
      while(i < 200 && ros::ok())
      {
        i++;
        usleep(2000);
      }
      robotSpeed = robotData->analyzeRobotDataQueue("actualFlangeSpeedDeque");
      robotPose = robotData->analyzeRobotDataQueue("actualFlangePoseDeque");
      FTValue = FTData->analyzeFTDataQueue("origen_ft");
    }
    while(FTValue.Deque.size() != 0){
      origrnFTTxt << FTValue.Deque.front().transpose() << std::endl;
      gtcedFTTxt << (FTValue.Deque.front() - gravityCompensation(robotPose.mean, compensationArgs)).transpose() << std::endl;
      FTValue.Deque.pop_front();
    }
    while(robotPose.Deque.size() != 0){
      robotPoseTxt << robotPose.Deque.front().transpose() << std::endl;
      robotPose.Deque.pop_front();
    }
    origrnFTTxt.close();
    gtcedFTTxt.close();
    robotPoseTxt.close();


    Eigen::Matrix<double,6,1> originFTData;
    Eigen::Matrix<double,6,1> GTCFTData;
    Eigen::Matrix<double,6,1> poseData;
    poseData = robotData->getRobotData("actualFlangePose");
    cout << "pose" << poseData.transpose() << endl;
    originFTData = FTData->getFTData("origen_ft");
    cout << "没有重力补偿的力传感器数据为：" << originFTData.transpose() << endl;
    GTCFTData = originFTData - gravityCompensation(poseData, compensationArgs);
    cout << "重力补偿后的力传感器数据为：" << GTCFTData.transpose() << endl << endl;
    needRecordData = false;
  }else{
    usleep(10000);
  }
}
}
void movingDataRecord(Robot_Move* robotMove, const ros::Publisher  urConParPub, const std::string dir, Process_FT_Data* FTData, Process_Robot_Data* robotData, const Gravity_Compensation_Args &compensationArgs)
{
string filePath("/./home/cym/catkin_ws/src/HRC_Assembly/robot_pose.txt");
vector<vector<double>> txtData = readFile(' ',filePath);//读文件

//遍历txtData，map为eigen矩阵形式,输出数值化的数据
vector<Eigen::Matrix<double,6,1>> poseList;
for(auto intIter = txtData.begin(); intIter != txtData.end(); intIter++){
  vector<double> line = *intIter;
  if(line.size() < 6) continue;
  auto *p = &line[0];
  auto len = line.size();
  Eigen::Map<Eigen::MatrixXd> matrix(p,len,1);
  poseList.push_back(matrix.block<6, 1>(0, 0));
  cout<<matrix.transpose()<<endl;
}

while(ros::ok())
{
  if(movingRecordData){
    printf("movingDataRecord\n");
    std::ofstream origrnFTTxt(dir + "/moving_origen_ft.txt");
    std::ofstream gtcedFTTxt(dir + "/moving_gtced_ft.txt");
    std::ofstream robotPoseTxt(dir + "/moving_robot_pose.txt");
    std::ofstream robotSpeedTxt(dir + "/moving_robot_speed.txt");
    origrnFTTxt.precision(9);
    gtcedFTTxt.precision(9);
    robotPoseTxt.precision(9);
    robotSpeedTxt.precision(9);

    Eigen::Matrix<double,6,1> originFTData;
    Eigen::Matrix<double,6,1> GTCFTData;
    Eigen::Matrix<double,6,1> poseData;
    Eigen::Matrix<double,6,1> speedData;

    std::for_each(std::begin(poseList), std::end(poseList), [&](const Eigen::Matrix<double,6,1> movePoseData){
      bool reachTargetPose = false;
      std::cout << movePoseData.transpose() <<std::endl;
      robotMove->sendMoveLMsg(urConParPub, movePoseData);
      while(ros::ok() && !reachTargetPose){
        double cycleTime = 1.0 / CTRL_FREQ;
        auto t_start = std::chrono::high_resolution_clock::now();

        speedData = robotData->getRobotData("actualFlangeSpeed");
        poseData = robotData->getRobotData("actualFlangePose");
        originFTData = FTData->getFTData("origen_ft");
        GTCFTData = originFTData - gravityCompensation(poseData, compensationArgs);

        origrnFTTxt << originFTData.transpose() << endl;
        gtcedFTTxt << GTCFTData.transpose() << endl;
        robotPoseTxt << poseData.transpose() << endl;
        robotSpeedTxt << speedData.transpose() << endl;

        if((poseData.block<3, 1>(0, 0) - movePoseData.block<3, 1>(0, 0)).cwiseAbs().sum() < 0.001){
          reachTargetPose = true;
        }else{
          reachTargetPose = false;
//            robotMove->sendMoveLMsg(urConParPub, movePoseData);
//            cout << "poseERROR" << (poseData - movePoseData).cwiseAbs().sum() << endl;
        }

//          cout << "movePoseData" << movePoseData.transpose() << endl;
//          cout << "pose" << poseData.transpose() << endl;
//          cout << "没有重力补偿的力传感器数据为：" << originFTData.transpose() << endl;
//          cout << "重力补偿后的力传感器数据为：" << GTCFTData.transpose() << endl << endl;

        auto t_stop = std::chrono::high_resolution_clock::now();
        auto t_duration = std::chrono::duration<double>(t_stop - t_start);
        if (t_duration.count() < cycleTime)
        {
          std::this_thread::sleep_for(std::chrono::duration<double>(cycleTime - t_duration.count()));
        }
      }
    });

    origrnFTTxt.close();
    gtcedFTTxt.close();
    robotPoseTxt.close();
    robotSpeedTxt.close();
    movingRecordData = false;
  }else{
    usleep(100000);
  }
}
}


void keyCycleDetection(Robot_Move &robotMove, const ros::Publisher &urConParPub)
{
int num = 0;
 while(ros::ok())
 {
   keyValue = scanKeyboard();
   if(keyValue == SPACE)
   {
     robotMove.sendteachMsg(urConParPub,true);
     teachMode = true;
     needRecordData = true;
   }else if(keyValue == ENTER){
     num++;
     if(num > 3){
       num = 0;
       movingRecordData = true;
     }
   }
   else{
     robotMove.sendteachMsg(urConParPub,false);
     teachMode = false;
   }
 }
}


//**********************************************main**********************************************/

int main(int argc, char **argv)
{
//  Process_FT_Data KW_FT;
Process_Robot_Data URData;
Robot_Move URMove;
Transfer_Matrix transM;

Metrical_Data metricalData;
Gravity_Compensation_Args compensationArgs;

std::vector<double> tmpVector = T_FLANGE_FT;
R_flange_ft =  Eigen::Map<Eigen::MatrixXd>(&tmpVector[0], 4, 4).block<3, 3>(0, 0);

ros::init(argc, argv, "tool_gravity_compensation");
ros::NodeHandle n;
Process_FT_Data KW_FT(n,"/ft_data","/processed_ft_data",Z_);
//  ros::Subscriber FTDataSub = n.subscribe("ft_data", 100, &Process_FT_Data::ftDataCallback, &KW_FT);
ros::Subscriber URDataSub = n.subscribe("ur_data", 100, &Process_Robot_Data::robotDataCallback, &URData);
ros::Publisher  urConParPub = n.advertise<hrc::RobotControl>("ur_control", 100);
ros::Rate loop_rate(CTRL_FREQ);
ros::AsyncSpinner AS(1);
AS.start();

int i = 0;
std::vector<double> initPoseVector = INITPOSE;
Eigen::Matrix<double,6,1> initPose ;
initPose << Eigen::Map<Eigen::MatrixXd>(&initPoseVector[0], 6, 1);

while((URData.getRobotData("actualFlangePose") - initPose).cwiseAbs().sum() > 0.002 && ros::ok()){
  usleep(1000);
  if(i++ > 6000){
    printf("机器人没有到达指定初始位置\n");
    return 0;
  }
}

//数据采集
while(ros::ok() && collectionOver == false){
  if(needCollectData == false || teachMode == true)
  {
    keyValue = scanKeyboard();
  }
//    printf(":%d\n",keyValue);
  if(keyValue == SPACE)
  {
    URMove.sendteachMsg(urConParPub,true);
    teachMode = true;
    needCollectData = true;
  }else if(keyValue == ENTER){
    collectionOver = true;
  }
  else if(keyValue == ESC){
    collectionOver = true;
    metricalData.poseNum = 0;
     printf("停止重力补偿，使用默认参数\n");
     break;
  }else{
    URMove.sendteachMsg(urConParPub,false);
    teachMode = false;
  }
  if(teachMode == false && needCollectData == true)
  {
    dataCollection(&KW_FT, &URData, metricalData);
    if(metricalData.poseNum<6){
      printf("已经记录了%ld组数据，至少还需要%ld组数据\n",metricalData.poseNum,6-metricalData.poseNum);
    }else{
      printf("已经记录了组%ld数据\n",metricalData.poseNum);
    }
    needCollectData = false;
  }
}
if(metricalData.poseNum == 0){}
else if(metricalData.poseNum<6){
  cout << "记录数据小于6组，最小二乘拟合失败" << endl;
  return 0;
}else{
  cout << "开始重力补偿参数计算" << endl;
  dataFormat(metricalData);
  auto a = matrixLeastSquares(metricalData.R, metricalData.F31);
  auto b = matrixLeastSquares(metricalData.F36, metricalData.T);
  gravityCompensationArgs(a, b, compensationArgs);
}

  cout<<"G"<<endl<<compensationArgs.G<<endl;
  cout<<"P"<<endl<<compensationArgs.P.transpose()<<endl;
  cout<<"U"<<endl<<compensationArgs.U<<endl;
  cout<<"V"<<endl<<compensationArgs.V<<endl;
  cout<<"F0"<<endl<<compensationArgs.F0.transpose()<<endl;
  cout<<"T0"<<endl<<compensationArgs.T0.transpose()<<endl;


  time_t currtime = time(nullptr);
  std::string dir = "/./home/cym/catkin_ws/record/gtc/";
  std::string sTime = dateTime_string(currtime);
  dir += sTime;
  createFolders(dir.c_str());

  std::ofstream tgcArgsTxt(dir + "/TGC_args.txt");
  tgcArgsTxt.precision(9);
  tgcArgsTxt << "G" << endl << compensationArgs.G << endl << endl;
  tgcArgsTxt << "P" << endl << compensationArgs.P.transpose() << endl << endl;
  tgcArgsTxt << "U" << endl << compensationArgs.U << endl << endl;
  tgcArgsTxt << "V" << endl << compensationArgs.V << endl << endl;
  tgcArgsTxt << "a" << endl << compensationArgs.A.transpose() << endl << endl;
  tgcArgsTxt << "F0" << endl << compensationArgs.F0.transpose() << endl << endl;
  tgcArgsTxt << "T0" << endl << compensationArgs.T0.transpose() << endl << endl;
  tgcArgsTxt.close();

  boost::thread t1(&keyCycleDetection, URMove, urConParPub);
  boost::thread t2(&staticDataRecord, dir,&KW_FT, &URData, compensationArgs);
  boost::thread t3(&movingDataRecord, &URMove, urConParPub, dir,&KW_FT, &URData, compensationArgs);
  t1.join();
  t2.join();
  t3.join();
while(ros::ok())
{
  loop_rate.sleep();
}
return 0;
}
