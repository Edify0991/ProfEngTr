#include "arm_control/generic_api.h"

time_t string_dateTime(std::string str)
{
  char *cha = const_cast<char*>(str.data());             // 将string转换成char*。
  tm tm_;                                    // 定义tm结构体。
  int year, month, day, hour, minute, second;// 定义时间的各个int临时变量。
  sscanf(cha, "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second);// 将string存储的日期时间，转换为int临时变量。
  tm_.tm_year = year - 1900;                 // 年，由于tm结构体存储的是从1900年开始的时间，所以tm_year为int临时变量减去1900。
  tm_.tm_mon = month - 1;                    // 月，由于tm结构体的月份存储范围为0-11，所以tm_mon为int临时变量减去1。
  tm_.tm_mday = day;                         // 日。
  tm_.tm_hour = hour;                        // 时。
  tm_.tm_min = minute;                       // 分。
  tm_.tm_sec = second;                       // 秒。
  tm_.tm_isdst = 0;                          // 非夏令时。
  time_t t_ = mktime(&tm_);                  // 将tm结构体转换成time_t格式。
  return t_;                                 // 返回值。
}

std::string dateTime_string(time_t time)
{
  //  char m_padding [6];
  tm *tm_ = localtime(&time);                // 将time_t格式转换为tm结构体
  int year, month, day, hour, minute, second;// 定义时间的各个int临时变量。
  year = tm_->tm_year + 1900;                // 临时变量，年，由于tm结构体存储的是从1900年开始的时间，所以临时变量int为tm_year加上1900。
  month = tm_->tm_mon + 1;                   // 临时变量，月，由于tm结构体的月份存储范围为0-11，所以临时变量int为tm_mon加上1。
  day = tm_->tm_mday;                        // 临时变量，日。
  hour = tm_->tm_hour;                       // 临时变量，时。
  minute = tm_->tm_min;                      // 临时变量，分。
  second = tm_->tm_sec;                      // 临时变量，秒。
  char yearStr[5], monthStr[3], dayStr[3], hourStr[3], minuteStr[3], secondStr[3];// 定义时间的各个char*变量。
  sprintf(yearStr, "%d", year);              // 年。
  sprintf(monthStr, "%d", month);            // 月。
  sprintf(dayStr, "%d", day);                // 日。
  sprintf(hourStr, "%d", hour);              // 时。
  sprintf(minuteStr, "%d", minute);          // 分。
  if (minuteStr[1] == '\0')                  // 如果分为一位，如5，则需要转换字符串为两位，如05。
  {
    minuteStr[2] = '\0';
    minuteStr[1] = minuteStr[0];
    minuteStr[0] = '0';
  }
  sprintf(secondStr, "%d", second);          // 秒。
  if (secondStr[1] == '\0')                  // 如果秒为一位，如5，则需要转换字符串为两位，如05。
  {
    secondStr[2] = '\0';
    secondStr[1] = secondStr[0];
    secondStr[0] = '0';
  }
  char s[20];                                // 定义总日期时间char*变量。
  sprintf(s, "%s_%s_%s_%s:%s:%s", yearStr, monthStr, dayStr, hourStr, minuteStr, secondStr);// 将年月日时分秒合并。
  std::string str(s);                             // 定义string变量，并将总日期时间char*变量作为构造函数的参数传入。
  return str;                                // 返回转换日期时间后的string变量。
}




/****************************************读txt文件里的数据***************************************/

void createFolders(const char* dir){
  char order[100] = "mkdir -p ";
  strcat(order, dir);
  system(order);
}

//对字符串inputString按tag字符分割
std::vector<std::string> split(std::string inputString,char tag){
  long length = inputString.length();
  int start=0;//数值起始下标
  std::vector<std::string> line;
  for(int i=0;i<length;i++){
    if(inputString[i] == tag){//遇到tag字符
      std::string sub = inputString.substr(start,i-start);    //取inputString[start]-inputString[i]子串
      line.push_back(sub);//压入向量中
      start =  i+1;
    }else if(i==length-1){
      std::string sub = inputString.substr(start,i-start+1);//最后一个字符没有标点，需单独处理
      line.push_back(sub);//压入向量中
    }
  }
  return line;
}

//将读到的vector<vector<string>>转化为vector<vector<int>>或vector<vector<double>>
void strVector_doubleVector(std::vector<std::vector<double>> &doubleData, std::vector<std::vector<std::string>> strData){
  std::vector<std::vector<std::string>>::iterator iter =  strData.begin();
  for(;iter != strData.end();iter++){
    std::vector<std::string> line = *iter;
    std::vector<std::string>::iterator lineiter =  line.begin();
    std::vector<double> intline;
    for(;lineiter != line.end();lineiter++){
      const char *p = lineiter->c_str();
      intline.push_back(atof(p));
    }
    doubleData.push_back(intline);
  }
}

//读取绝对路径为filePath的文件，文件中每行中的数值以tag字符分开
std::vector<std::vector<double>> readFile(char tag,std::string filePath){
  std::ifstream fileReader;
  fileReader.open(filePath,std::ios::in);//以只读方式打开
  std::vector<std::vector<std::string>> strData;
  std::vector<std::vector<double>> doubleData;
  while(!fileReader.eof()){//未到文件末尾
    std::string linestring;
    getline(fileReader,linestring);//读取一行
    std::vector<std::string> line = split(linestring,tag);//分割每行,并放在line向量中
    strData.push_back(line);
  }
  strVector_doubleVector(doubleData, strData);
  return doubleData;
}
/***************************************读txt文件里的数据***************************************/

/******************************************按键检测************************************************/
//按键扫描，阻塞型
int scanKeyboard()
{
int in;
struct termios new_settings;
struct termios stored_settings;
tcgetattr(0,&stored_settings);
new_settings = stored_settings;
new_settings.c_lflag &= (~ICANON);
new_settings.c_cc[VTIME] = 0;
tcgetattr(0,&stored_settings);
new_settings.c_cc[VMIN] = 1;
tcsetattr(0,TCSANOW,&new_settings);

in = getchar();

tcsetattr(0,TCSANOW,&stored_settings);
return in;
}
/*******************************************按键检测**********************************************/


//与当前ros时间的差值
double msgDelayTimeMs(ros::Time gettime){
  ros::Time timeNow = ros::Time::now();
  double timeInterval;
  timeInterval = ((timeNow.sec - gettime.sec)*1000000000 + timeNow.nsec -gettime.nsec)/1000000.0;
  return timeInterval;
}
