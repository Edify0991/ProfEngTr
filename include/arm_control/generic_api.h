#ifndef GENERIC_API_H
#define GENERIC_API_H
#include <unistd.h>
#include <ros/ros.h>
#include <deque>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <termios.h>

//将字符串转化为64位hash值
typedef std::uint64_t hash_t;
constexpr hash_t prime = 0x100000001B3ull;
constexpr hash_t basis = 0xCBF29CE484222325ull;
inline constexpr hash_t hashStrUint64(char const* str, hash_t last_value = basis)
{
  return *str ? hashStrUint64(str+1, (*str ^ last_value) * prime) : last_value;
}


//时间字符转time_t
time_t string_dateTime(std::string str);

//time_t转时间字符
std::string dateTime_string(time_t time);


/****************************************读txt文件里的数据***************************************/
//创建文件夹
void createFolders(const char* dir);

//对字符串inputString按tag字符分割
std::vector<std::string> split(std::string inputString,char tag);

//将读到的vector<vector<string>>转化为vector<vector<int>>或vector<vector<double>>
void strVector_doubleVector(std::vector<std::vector<double>> &doubleData, std::vector<std::vector<std::string>> strData);

//读取绝对路径为filePath的文件，文件中每行中的数值以tag字符分开
std::vector<std::vector<double>> readFile(char tag,std::string filePath);
/***************************************读txt文件里的数据***************************************/

//按键扫描，阻塞型
int scanKeyboard();

//与当前ros时间的差值
double msgDelayTimeMs(ros::Time gettime);

#endif // GENERIC_API_H
