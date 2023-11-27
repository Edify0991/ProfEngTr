#ifndef HRC_GTC_H
#define HRC_GTC_H

#include <deque>
#include <float.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/geometry.hpp>


#include <deque>
#include <float.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/geometry.hpp>

typedef struct
{
  unsigned long dataBuffer = 500;
  Eigen::Matrix<double,6,1> msgData;
  std::deque<Eigen::Matrix<double,6,1>> dataDeque;

}FT_Data;

typedef struct
{
unsigned long dataBuffer = 20;
Eigen::Matrix<double,6,1> pose;
Eigen::Matrix<double,6,1> speed;
std::deque<Eigen::Matrix<double,6,1>> poseDeque;
std::deque<Eigen::Matrix<double,6,1>> speedDeque;
}Robot_Data;


typedef struct
{
  unsigned long poseNum = 0;
  std::deque<Eigen::Matrix<double,6,1>> pose;
  std::deque<Eigen::Matrix<double,6,1>> FT;
  std::deque<Eigen::Matrix<double,3,3>> rotationMatrix;
  Eigen::Matrix<double,Eigen::Dynamic,6> R;
  Eigen::Matrix<double,Eigen::Dynamic,1> F31;
  Eigen::Matrix<double,Eigen::Dynamic,6> F36;
  Eigen::Matrix<double,Eigen::Dynamic,1> T;
  Eigen::Matrix<double,6,1> a;
  Eigen::Matrix<double,6,1> b;
}Metrical_Data;


//typedef struct
//{
//  double G;
//  double U;
//  double V;
//  Eigen::Matrix<double,3,1> F0;
//  Eigen::Matrix<double,3,1> T0;
//  Eigen::Matrix<double,3,1> P;
//  Eigen::Matrix<double,3,1> a;
//}Gravity_Compensation_Args;

#endif // HRC_GTC_H
