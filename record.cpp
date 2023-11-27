#include <unistd.h>
#include "ros/ros.h"
#include <deque>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <hrc/RobotControl.h>
#include <hrc/RobotData.h>
#include <virtual_fixture.h>
#include <config.h>
#include <record_node.h>
#include <generic_api.h>
#include <controller.h>

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

  void urDataCallback(const hrc::RobotData &msg);
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
  void ftDataCallback(const hrc::ProcessedFTData &msg);
};

class Record_Ctrl_Data{
public:
  double beginTime;
  long int updateFlag = 0;
  Eigen::Matrix<double,1,8> poseData;
  std::deque<Eigen::Matrix<double,1,8>> poseDataDeque;
  Eigen::Matrix<double,1,8> speedData;
  std::deque<Eigen::Matrix<double,1,8>> speedDataDeque;
  void urContralCallback(const hrc::RobotControl &msg);
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

int main(int argc, char **argv)
{
  Record_FT_Data FTRec;
  Record_Robot_Data URRec;
  Record_Ctrl_Data URCtrlRec;
  Record_Mode_Data modeRec;

  ros::init(argc, argv, "record_data");
  ros::NodeHandle n;
  ros::Subscriber FTDataSub = n.subscribe("/processed_ft_data", 1000, &Record_FT_Data::ftDataCallback, &FTRec);
  ros::Subscriber URDataSub = n.subscribe("ur_data", 1000, &Record_Robot_Data::urDataCallback, &URRec);
  ros::Subscriber URCtrlRecSub = n.subscribe("ur_control", 1000, &Record_Ctrl_Data::urContralCallback, &URCtrlRec);
  ros::Subscriber ctrlModeSub = n.subscribe("/mode", 1000, &Record_Mode_Data::modeDataCallback, &modeRec);

  std::string dir;
  ros::param::get("/config/record_dir", dir);
//  std::cout << dir << std::endl;

  time_t currtime = time(nullptr);
  std::string sTime = dateTime_string(currtime);
  dir += sTime;
  createFolders(dir.c_str());
  std::ofstream actPoseTxt(dir + "/actualTCPPose.txt");
  std::ofstream tgtPoseTxt(dir + "/targetTCPPose.txt");
  std::ofstream actSpeedTxt(dir + "/actualTCPSpeed.txt");
  std::ofstream tgtSpeedTxt(dir + "/targetTCPSpeed.txt");
  std::ofstream actAccelTxt(dir + "/actualToolAccel.txt");
  std::ofstream ctrlPoseTxt(dir + "/ctrlPoseData.txt");
  std::ofstream ctrlSpeedTxt(dir + "/ctrlSpeedData.txt");


  std::ofstream origen_ft_Txt(dir + "/origen_ft.txt");
  std::ofstream filtered_ft_Txt(dir + "/filtered_ft.txt");
  std::ofstream origen_gtced_ft_Txt(dir + "/origen_gtced_ft.txt");
  std::ofstream filtered_gtced_ft_Txt(dir + "/filtered_gtced_ft.txt");
  std::ofstream origen_world_Txt(dir + "/origen_world.txt");
  std::ofstream filtered_world_Txt(dir + "/filtered_world.txt");
  std::ofstream origen_gtced_world_Txt(dir + "/origen_gtced_world.txt");
  std::ofstream filtered_gtced_world_Txt(dir + "/filtered_gtced_world.txt");

  std::ofstream mode_Txt(dir + "/mode.txt");

  actPoseTxt.precision(9);
  tgtPoseTxt.precision(9);
  actSpeedTxt.precision(9);
  tgtSpeedTxt.precision(9);
  actAccelTxt.precision(9);
  ctrlPoseTxt.precision(9);
  ctrlSpeedTxt.precision(9);
  origen_ft_Txt.precision(9);
  filtered_ft_Txt.precision(9);
  origen_gtced_ft_Txt.precision(9);
  filtered_gtced_ft_Txt.precision(9);
  origen_world_Txt.precision(9);
  filtered_world_Txt.precision(9);
  origen_gtced_world_Txt.precision(9);
  filtered_gtced_world_Txt.precision(9);
  mode_Txt.precision(9);

  ros::Time beginTime = ros::Time::now();
  modeRec.beginTime = FTRec.beginTime = URRec.beginTime = URCtrlRec.beginTime = beginTime.sec + static_cast<double>(beginTime.nsec)/1000000000.0;

  ros::AsyncSpinner spinner(1); // Use 1 threads
  spinner.start();
  ros::Rate loop_rate(1000);

  while(ros::ok())
  {
    //    printf("size of deque = %ld\n",FTRec.FTDeque.size());
    if(FTRec.updateFlag != 0)
    {
      while(FTRec.origen_ft_Deque.size() != 0){
        origen_ft_Txt << FTRec.origen_ft_Deque.front() << std::endl;
        FTRec.origen_ft_Deque.pop_front();
      }
      while(FTRec.filtered_ft_Deque.size() != 0){
        filtered_ft_Txt << FTRec.filtered_ft_Deque.front() << std::endl;
        FTRec.filtered_ft_Deque.pop_front();
      }
      while(FTRec.origen_gtced_ft_Deque.size() != 0){
        origen_gtced_ft_Txt << FTRec.origen_gtced_ft_Deque.front() << std::endl;
        FTRec.origen_gtced_ft_Deque.pop_front();
      }
      while(FTRec.filtered_gtced_ft_Deque.size() != 0){
        filtered_gtced_ft_Txt << FTRec.filtered_gtced_ft_Deque.front() << std::endl;
        FTRec.filtered_gtced_ft_Deque.pop_front();
      }
      while(FTRec.origen_world_Deque.size() != 0){
        origen_world_Txt << FTRec.origen_world_Deque.front() << std::endl;
        FTRec.origen_world_Deque.pop_front();
      }
      while(FTRec.filtered_world_Deque.size() != 0){
        filtered_world_Txt << FTRec.filtered_world_Deque.front() << std::endl;
        FTRec.filtered_world_Deque.pop_front();
      }
      while(FTRec.origen_gtced_world_Deque.size() != 0){
        origen_gtced_world_Txt << FTRec.origen_gtced_world_Deque.front() << std::endl;
        FTRec.origen_gtced_world_Deque.pop_front();
      }
      while(FTRec.filtered_gtced_world_Deque.size() != 0){
        filtered_gtced_world_Txt << FTRec.filtered_gtced_world_Deque.front() << std::endl;
        FTRec.filtered_gtced_world_Deque.pop_front();
      }
      FTRec.updateFlag = 0;
    }
    if(URCtrlRec.updateFlag != 0)
    {
      while(URCtrlRec.poseDataDeque.size() != 0){
        ctrlPoseTxt << URCtrlRec.poseDataDeque.front() << std::endl;;
        URCtrlRec.poseDataDeque.pop_front();
      }
      while(URCtrlRec.speedDataDeque.size() != 0){
        ctrlSpeedTxt << URCtrlRec.speedDataDeque.front() << std::endl;;
        URCtrlRec.speedDataDeque.pop_front();
      }
      URCtrlRec.updateFlag = 0;
    }
    if(URRec.updateFlag != 0)
    {
      while(URRec.actualPoseDeque.size() != 0){
        actPoseTxt << URRec.actualPoseDeque.front() << std::endl;
        URRec.actualPoseDeque.pop_front();
      }
      while(URRec.targetPoseDeque.size() != 0){
        tgtPoseTxt << URRec.targetPoseDeque.front() << std::endl;
        URRec.targetPoseDeque.pop_front();
      }
      while(URRec.actualSpeedDeque.size() != 0){
        actSpeedTxt << URRec.actualSpeedDeque.front() << std::endl;
        URRec.actualSpeedDeque.pop_front();
      }
      while(URRec.targetSpeedDeque.size() != 0){
        tgtSpeedTxt << URRec.targetSpeedDeque.front() << std::endl;
        URRec.targetSpeedDeque.pop_front();
      }
      while(URRec.actualAccelDeque.size() != 0){
        actAccelTxt << URRec.actualAccelDeque.front() << std::endl;
        URRec.actualAccelDeque.pop_front();
      }
      URRec.updateFlag = 0;
    }
    if(modeRec.updateFlag != 0)
    {
      while(modeRec.modeDeque.size() != 0){
         mode_Txt << modeRec.timeDeque.front() << "  ";
        mode_Txt << modeRec.modeDeque.front() << std::endl;
        modeRec.modeDeque.pop_front();
        modeRec.timeDeque.pop_front();
      }
      URRec.updateFlag = 0;
    }
    loop_rate.sleep();
  }
  actPoseTxt.close();
  tgtPoseTxt.close();
  actSpeedTxt.close();
  tgtSpeedTxt.close();
  actAccelTxt.close();
  ctrlPoseTxt.close();
  ctrlSpeedTxt.close();
  origen_ft_Txt.close();
  filtered_ft_Txt.close();
  origen_gtced_ft_Txt.close();
  filtered_gtced_ft_Txt.close();
  origen_world_Txt.close();
  filtered_world_Txt.close();
  origen_gtced_world_Txt.close();
  filtered_gtced_world_Txt.close();
  mode_Txt.close();
  return 0;
}



void Record_Robot_Data::urDataCallback(const hrc::RobotData &msg)
{
  double time = msg.header.stamp.sec + static_cast< double>(msg.header.stamp.nsec)/1000000000.0 - beginTime;
  std::vector<double> tmp;
  tmp = msg.actualTCPPose;
  actualPose << msg.header.seq, time,
      Eigen::Map<Eigen::Matrix<double,1,6>>(&tmp[0],tmp.size());
  actualPoseDeque.push_back(actualPose);
  tmp.clear();
  tmp = msg.actualTCPPose;
  targetPose << msg.header.seq, time,
      Eigen::Map<Eigen::Matrix<double,1,6>>(&tmp[0],tmp.size());
  targetPoseDeque.push_back(targetPose);
  tmp.clear();
  tmp = msg.actualTCPSpeed;
  actualSpeed << msg.header.seq, time,
      Eigen::Map<Eigen::Matrix<double,1,6>>(&tmp[0],tmp.size());
  actualSpeedDeque.push_back(actualSpeed);
  tmp.clear();
  tmp = msg.targetTCPSpeed;
  targetSpeed << msg.header.seq, time,
      Eigen::Map<Eigen::Matrix<double,1,6>>(&tmp[0],tmp.size());
  targetSpeedDeque.push_back(targetSpeed);
  tmp.clear();
  tmp = msg.actualToolAccel;
  actualAccel << msg.header.seq, time,
      Eigen::Map<Eigen::Matrix<double,1,3>>(&tmp[0],tmp.size()),0,0,0;
  actualAccelDeque.push_back(actualAccel);
  if(updateFlag != 0){
    //      printf("UR的数据堵塞了%ld个\n",updateFlag);
  }
  updateFlag++;
}

void Record_FT_Data::ftDataCallback(const hrc::ProcessedFTData &msg)
{
  //    printf("%u\n",msg.header.seq);
  //    printf("%.9f\n",msg.header.stamp.sec + static_cast< double>(msg.header.stamp.nsec)/1000000000.0);
  double time = msg.header.stamp.sec + static_cast< double>(msg.header.stamp.nsec)/1000000000.0 - beginTime;
  std::vector<double> tmp;
  tmp = msg.origen_ft;
  origen_ft << msg.header.seq, time,
      Eigen::Map<Eigen::Matrix<double,1,6>>(&tmp[0],tmp.size());
  origen_ft_Deque.push_back(origen_ft);
  tmp.clear();

  tmp = msg.filtered_ft;
  filtered_ft << msg.header.seq, time,
      Eigen::Map<Eigen::Matrix<double,1,6>>(&tmp[0],tmp.size());
  filtered_ft_Deque.push_back(filtered_ft);
  tmp.clear();

  tmp = msg.origen_gtced_ft;
  origen_gtced_ft << msg.header.seq, time,
      Eigen::Map<Eigen::Matrix<double,1,6>>(&tmp[0],tmp.size());
  origen_gtced_ft_Deque.push_back(origen_gtced_ft);
  tmp.clear();

  tmp = msg.filtered_gtced_ft;
  filtered_gtced_ft << msg.header.seq, time,
      Eigen::Map<Eigen::Matrix<double,1,6>>(&tmp[0],tmp.size());
  filtered_gtced_ft_Deque.push_back(filtered_gtced_ft);
  tmp.clear();

  tmp = msg.origen_world;
  origen_world << msg.header.seq, time,
      Eigen::Map<Eigen::Matrix<double,1,6>>(&tmp[0],tmp.size());
  origen_world_Deque.push_back(origen_world);
  tmp.clear();

  tmp = msg.filtered_world;
  filtered_world << msg.header.seq, time,
      Eigen::Map<Eigen::Matrix<double,1,6>>(&tmp[0],tmp.size());
  filtered_world_Deque.push_back(filtered_world);
  tmp.clear();

  tmp = msg.origen_gtced_world;
  origen_gtced_world << msg.header.seq, time,
      Eigen::Map<Eigen::Matrix<double,1,6>>(&tmp[0],tmp.size());
  origen_gtced_world_Deque.push_back(origen_gtced_world);
  tmp.clear();

  tmp = msg.filtered_gtced_world;
  filtered_gtced_world << msg.header.seq, time,
      Eigen::Map<Eigen::Matrix<double,1,6>>(&tmp[0],tmp.size());
  filtered_gtced_world_Deque.push_back(filtered_gtced_world);
  tmp.clear();
  if(updateFlag != 0){
//          printf("FT的数据堵塞了%ld个\n",updateFlag);
  }
  updateFlag++;
}

void Record_Ctrl_Data::urContralCallback(const hrc::RobotControl &msg){
  double time = msg.header.stamp.sec + static_cast< double>(msg.header.stamp.nsec)/1000000000.0 - beginTime;
  poseData << msg.header.seq, time,
      msg.pose.x,
      msg.pose.y,
      msg.pose.z,
      msg.pose.Rx,
      msg.pose.Ry,
      msg.pose.Rz;
  poseDataDeque.push_back(poseData);

  speedData << msg.header.seq, time,
      msg.pose.x,
      msg.pose.y,
      msg.pose.z,
      msg.pose.Rx,
      msg.pose.Ry,
      msg.pose.Rz;
  speedDataDeque.push_back(speedData);
  if(updateFlag != 0){
    //      printf("UR的数据堵塞了%ld个\n",updateFlag);
  }
  updateFlag++;
}

void Record_Mode_Data::modeDataCallback(const std_msgs::Int32 &msg)
{
  mode = msg.data;
  modeDeque.push_back(mode);
  updateFlag++;
  timeDeque.push_back(ros::Time::now().sec + static_cast<double>(ros::Time::now().nsec)/1000000000.0 - beginTime);
}
