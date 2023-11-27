#include <ros/ros.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <deque>
#include <math.h>
#include <mutex>
#include <map>
#include <Eigen/Dense>

#include <arm_control/RobotControl.h>
#include <arm_control/process_robot_data.h>

Transfer_Matrix *transM_P = new Transfer_Matrix;
void Robot_Move::sendSpeedLMsg(ros::Publisher  urConParPub, Eigen::Matrix<double,6,1> speedL){
  arm_control::RobotControl msg;
  std::string str;

  msg.speed.x = speedL(0,0);
  msg.speed.y = speedL(1,0);
  msg.speed.z = speedL(2,0);
  msg.speed.Rx = speedL(3,0);
  msg.speed.Ry = speedL(4,0);
  msg.speed.Rz = speedL(5,0);
  msg.header.stamp = ros::Time::now();
  msg.mode = str.assign("speedL");
  urConParPub.publish(msg);
}

void Robot_Move::sendPoseMsg(ros::Publisher  urConParPub, Eigen::Matrix<double,6,1> pose){
  arm_control::RobotControl msg;
  std::string str;
  msg.pose.x = pose(0,0);
  msg.pose.y = pose(1,0);
  msg.pose.z = pose(2,0);
  msg.pose.Rx = pose(3,0);
  msg.pose.Ry = pose(4,0);
  msg.pose.Rz = pose(5,0);
  msg.header.stamp = ros::Time::now();
  msg.mode = str.assign("servol");
  urConParPub.publish(msg);
}

void Robot_Move::sendMoveLMsg(ros::Publisher  urConParPub, Eigen::Matrix<double,6,1> pose){
  arm_control::RobotControl msg;
  std::string str;
  msg.pose.x = pose(0,0);
  msg.pose.y = pose(1,0);
  msg.pose.z = pose(2,0);
  msg.pose.Rx = pose(3,0);
  msg.pose.Ry = pose(4,0);
  msg.pose.Rz = pose(5,0);
  msg.header.stamp = ros::Time::now();
  msg.mode = str.assign("moveL");
  urConParPub.publish(msg);
}

//发送示教控制命令
void Robot_Move::sendteachMsg(ros::Publisher urConParPub,bool teachModeFlag){
  arm_control::RobotControl msg;
  msg.header.stamp = ros::Time::now();
  std::string str;
  if(teachModeFlag)
  {
    msg.mode = str.assign("teach");
  }else{
    msg.mode = str.assign("stop");
  }
  urConParPub.publish(msg);
}
//机器人消息的回调函数
void Process_Robot_Data::robotDataCallback(const arm_control::RobotData &msg)
{
  std::lock_guard<std::mutex> some_guard(robot_mutex);

  digitalInput = msg.digitalInputBits;

  std::vector<double> tmp;
  tmp = msg.actualTCPPose;
  actualFlangePose = Eigen::Map<Eigen::Matrix<double,6,1>>(&tmp[0],tmp.size());
  if(actualFlangePoseDeque.size() >= robotDataBuffer){
    actualFlangePoseDeque.pop_front();
  }
  actualFlangePoseDeque.push_back(actualFlangePose);
  tmp.clear();
  tmp = msg.targetTCPPose;
  targetFlangePose = Eigen::Map<Eigen::Matrix<double,6,1>>(&tmp[0],tmp.size());
  if(targetFlangePoseDeque.size() >= robotDataBuffer){
    targetFlangePoseDeque.pop_front();
  }
  targetFlangePoseDeque.push_back(targetFlangePose);
  tmp.clear();
  tmp = msg.actualTCPSpeed;
  actualFlangeSpeed = Eigen::Map<Eigen::Matrix<double,6,1>>(&tmp[0],tmp.size());
  if(actualFlangeSpeedDeque.size() >= robotDataBuffer){
    actualFlangeSpeedDeque.pop_front();
  }
  actualFlangeSpeedDeque.push_back(actualFlangeSpeed);
  tmp.clear();
  tmp = msg.targetTCPSpeed;
  targetFlangeSpeed = Eigen::Map<Eigen::Matrix<double,6,1>>(&tmp[0],tmp.size());
  if(targetFlangeSpeedDeque.size() >= robotDataBuffer){
    targetFlangeSpeedDeque.pop_front();
  }
  targetFlangeSpeedDeque.push_back(targetFlangeSpeed);
  tmp.clear();
  tmp = msg.actualToolAccel;
  actualFlangeAccel << Eigen::Map<Eigen::Matrix<double,3,1>>(&tmp[0],tmp.size()),0,0,0;
  if(actualFlangeAccelDeque.size() >= robotDataBuffer){
    actualFlangeAccelDeque.pop_front();
  }
  actualFlangeAccelDeque.push_back(actualFlangeAccel);

  transM_P->set_TR_robot_flange(actualFlangePose);

  //     printf("args.actualTCPPose = %2f\n",args.actualTCPPose[0]);
  //    printf("delay time ms = %2f\n", msgDelayTimeMs(msg.header.stamp));
  //    printf("actualTCPForce(0,0) = %2f\n", args.actualTCPForce(0,0));
}

uint64_t Process_Robot_Data::getDigitalInput(void)
{
  std::lock_guard<std::mutex> some_guard(robot_mutex);
  return digitalInput;

}

Eigen::Matrix<double,6,1> Process_Robot_Data::getRobotData(const char *dataType)
{
  std::lock_guard<std::mutex> some_guard(robot_mutex);
  Eigen::Matrix<double,6,1> tmpData;
  if (std::strcmp(dataType,"actualFlangePose") == 0)
  {
    tmpData = actualFlangePose;
  }
  else if (std::strcmp(dataType,"targetFlangePose") == 0)
  {
    tmpData = targetFlangePose;
  }
  else if (std::strcmp(dataType,"actualFlangeSpeed") == 0)
  {
    tmpData = actualFlangeSpeed;
  }
  else if (std::strcmp(dataType,"targetFlangeSpeed") == 0)
  {
    tmpData = targetFlangeSpeed;
  }
  else if (std::strcmp(dataType,"actualFlangeAccel") == 0)
  {
    tmpData << actualFlangeAccel;
  }
  else{
    printf("getRobotData函数错误输入\n");
  }
  return tmpData;
}

std::deque<Eigen::Matrix<double,6,1>> Process_Robot_Data::getRobotDataDeque(const char *dataType)
{
  std::lock_guard<std::mutex> some_guard(robot_mutex);
  std::deque<Eigen::Matrix<double,6,1>> tmpData;
  if (std::strcmp(dataType,"actualFlangePoseDeque") == 0)
  {
    tmpData = actualFlangePoseDeque;
  }
  else if (std::strcmp(dataType,"targetFlangePoseDeque") == 0)
  {
    tmpData = targetFlangePoseDeque;
  }
  else if (std::strcmp(dataType,"actualFlangeSpeedDeque") == 0)
  {
    tmpData = actualFlangeSpeedDeque;
  }
  else if (std::strcmp(dataType,"targetFlangeSpeedDeque") == 0)
  {
    tmpData = targetFlangeSpeedDeque;
  }
  else if (std::strcmp(dataType,"actualFlangeAccelDeque") == 0)
  {
    tmpData = actualFlangeAccelDeque;
  }
  else{
    printf("getRobotDataDqueue函数错误输入\n");
  }
  return tmpData;
}

//检测机器人是否到达目标位置
bool Process_Robot_Data::isReachTargetPose(Eigen::Matrix<double,6,1> targetPose, double error)
{
  return (getRobotData("actualFlangeSpeed").cwiseAbs().sum() + (getRobotData("actualFlangePose").block<3, 1>(0, 0) - targetPose.block<3, 1>(0, 0)).cwiseAbs().sum()) > error ? false : true;

}




Transfer_Matrix::Transfer_Matrix()
{
  std::vector<double> tmpVector = T_WORLD_ROBOT;
  T_world_robot =  Eigen::Map<Eigen::MatrixXd>(&tmpVector[0], 4, 4);
  R_world_robot = T_world_robot.block<3, 3>(0, 0);

  tmpVector = T_FLANGE_FT;
  T_flange_ft =  Eigen::Map<Eigen::MatrixXd>(&tmpVector[0], 4, 4);
  R_flange_ft = T_flange_ft.block<3, 3>(0, 0);

  tmpVector = T_FT_TCP;
  T_ft_tcp =  Eigen::Map<Eigen::MatrixXd>(&tmpVector[0], 4, 4);
  R_ft_tcp = T_ft_tcp.block<3, 3>(0, 0);
}

//分析机器人数据队列中的数据
Robot_Data_Analyze Process_Robot_Data::analyzeRobotDataQueue(const char *dataType){
  Eigen::Matrix<double,6,1> sum, accum;
  sum = accum = Eigen::MatrixXd::Zero(6,1);
  Robot_Data_Analyze Args;

  Args.Deque = getRobotDataDeque(dataType);


  std::for_each(std::begin(Args.Deque), std::end(Args.Deque), [&](const Eigen::Matrix<double,6,1> data){
    sum += data;
  });
  Args.mean = sum / Args.Deque.size();

  std::for_each(std::begin(Args.Deque), std::end(Args.Deque), [&](const Eigen::Matrix<double,6,1> data){
    accum += ((data - Args.mean).array() * (data - Args.mean).array()).matrix();
  });
  Args.stdev = sqrt(accum.array()/(Args.mean.size()-1)).matrix();
  return Args;
}

void Transfer_Matrix::set_TR_robot_flange(Eigen::Matrix<double,6,1> pose)
{
  R_robot_flange = quaternion_rotationMatrix(rotationVector_quaternion(pose.block<3, 1>(3, 0)));
  T_robot_flange.block<3,3>(0,0) = R_robot_flange;
  T_robot_flange.block<3,1>(0,3) = pose.block<3, 1>(0, 0);
}

//欧拉角转换为四元数
Eigen::Quaterniond euler_quaternion(Eigen::Vector3d eulerAngle)
{
  return Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitX());
}

//四元数转换为旋转矢量
Eigen::Matrix<double,3,1> quaternion_rotationVector(Eigen::Quaterniond quaternion)
{
  Eigen::Matrix<double,3,1> rotationVector;
  double len = quaternion.norm();
  double theat = acos(quaternion.w() / len) * 2;
  if(abs(sin(theat / 2)) < 0.0001){
    theat += theat > 0 ? 0.0001 : -0.0001;
  }
  rotationVector[0] = quaternion.x() / len / sin(theat / 2) * theat;
  rotationVector[1] = quaternion.y() / len / sin(theat / 2) * theat;
  rotationVector[2] = quaternion.z() / len / sin(theat / 2) * theat;
  return rotationVector;
}

//旋转矢量到四元数
Eigen::Quaterniond rotationVector_quaternion(Eigen::Matrix<double,3,1> rotationVector)
{
  Eigen::Quaterniond quaternion;
  double theat = rotationVector.norm();
  quaternion.w() = cos(theat / 2);
  if(abs(theat) < 0.0001){
    theat += theat > 0 ? 0.0001 : -0.0001;
  }
  quaternion.x() = rotationVector[0] / theat * sin(theat / 2);
  quaternion.y() = rotationVector[1] / theat * sin(theat / 2);
  quaternion.z() = rotationVector[2] / theat * sin(theat / 2);
  return quaternion;
}

//四元数转换为旋转矩阵
Eigen::Matrix3d quaternion_rotationMatrix(Eigen::Quaterniond quaternion)
{
  return quaternion.toRotationMatrix();
}
//旋转矩阵转换为欧拉角
//ZYX顺序，即先绕x轴roll,再绕y轴pitch,最后绕z轴yaw,0表示X轴,1表示Y轴,2表示Z轴
Eigen::Vector3d  rotationMatrix_euler(Eigen::Matrix3d rotationMatrix){
  return rotationMatrix.eulerAngles(2, 1, 0);
}

//旋转矩阵转换为欧拉ZYX/RPY
//ZYX顺序，即先绕x轴roll,再绕y轴pitch,最后绕z轴yaw,0表示X轴,1表示Y轴,2表示Z轴
Eigen::Vector3d  rotationMatrix_RPY(Eigen::Matrix3d rotationMatrix){
  return rotationMatrix.eulerAngles(2, 1, 0);
}

//欧拉ZYX/RPY转换为旋转矩阵
Eigen::Matrix3d RPY_rotationMatrix(Eigen::Vector3d RPY)
{
  Eigen::AngleAxisd rotation_vector_x(RPY(2), Eigen::Vector3d(1, 0, 0));
  Eigen::AngleAxisd rotation_vector_y(RPY(1), Eigen::Vector3d(0, 1, 0));
  Eigen::AngleAxisd rotation_vector_z(RPY(0), Eigen::Vector3d(0, 0, 1));
  Eigen::Matrix3d rotationMatrix =
      rotation_vector_z.matrix() * rotation_vector_y.matrix() * rotation_vector_x.matrix();

  return rotationMatrix;
}


//eulerXYZ转换为四元数
Eigen::Quaterniond eulerXYZ_quaternion(Eigen::Vector3d RPY)
{
  Eigen::AngleAxisd rotation_vector_x(RPY(0), Eigen::Vector3d(1, 0, 0));
  Eigen::AngleAxisd rotation_vector_y(RPY(1), Eigen::Vector3d(0, 1, 0));
  Eigen::AngleAxisd rotation_vector_z(RPY(2), Eigen::Vector3d(0, 0, 1));
  Eigen::Quaterniond quaternion(rotation_vector_z.matrix() * rotation_vector_y.matrix() * rotation_vector_x.matrix());
  return quaternion;
}

//旋转矢量转换为RPY
Eigen::Vector3d rotationVector_RPY(Eigen::Matrix<double,3,1> rotationVector)
{
  return rotationMatrix_RPY(quaternion_rotationMatrix(rotationVector_quaternion(rotationVector)));
}

//用轴角xyz表示两个四元数的差角 q1 - q2
Eigen::Vector3d quaternionsDiff_eulerXYZ(Eigen::Quaterniond q1, Eigen::Quaterniond q2)
{
  if(q1.coeffs().dot(q2.coeffs()) < 0.0){
    q1.coeffs() << -q1.coeffs();
  }
  Eigen::Quaterniond qErroe(q1 * q2.inverse());
  Eigen::Vector3d error_xyz;
  error_xyz << qErroe.x(), qErroe.y(), qErroe.z();
  if(error_xyz.norm() != 0.0)
  {
    error_xyz << error_xyz /error_xyz.norm() * 2 * acos(qErroe.w());
  }
  return error_xyz;
}

//用轴角xyz表示两个旋转矢量的差角 v1 - v2
Eigen::Vector3d rotationVectorDiff_eulerXYZ(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
  Eigen::Quaterniond q1,q2;
  q1 = rotationVector_quaternion(v1);
  q2 = rotationVector_quaternion(v2);
  return quaternionsDiff_eulerXYZ(q1,q2);
}


