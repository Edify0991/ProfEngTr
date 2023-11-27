#include <unistd.h>
#include <thread>
#include <chrono>
#include <math.h>
#include <mutex>
#include <ros/ros.h>
#include "arm_control/ur_move.h"
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include "arm_control/RobotControl.h"
#include "arm_control/record_node.h"
#include "arm_control/generic_api.h"
#include "arm_control/process_robot_data.h"
using namespace ur_rtde;
using namespace std::chrono;


class  UR_Control{
private:
    std::mutex urMove_mutex;
    RTDEControlInterface rtdeControl;
public:

    std::vector<double> targetPose;
    std::vector<double> targetSpeed;
    Move_Arg moveArg;
    UR_Control(std::string IPAdress = "127.0.0.1"):rtdeControl(IPAdress)
    {

    }
    //移动到初始位置
    void moveInit(void)
    {
        moveArg.mode = moveJ;
        moveArg.initPose = INITPOSE;
        targetPose = moveArg.initPose;
        rtdeControl.moveJ_IK(moveArg.initPose, moveArg.velocity, moveArg.acceleration);   //tool velocity and tool acceleration
        moveArg.mode = stop;
    }
    //机器人末端位置的位置消息回调函数
    void urContralCallback(const arm_control::RobotControl &msg)  //msg类型为RobotControl.msg
    {
        std::lock_guard<std::mutex> some_guard(urMove_mutex);
        targetPose.clear();
        targetPose.push_back(msg.pose.x);
        targetPose.push_back(msg.pose.y);
        targetPose.push_back(msg.pose.z);
        targetPose.push_back(msg.pose.Rx);
        targetPose.push_back(msg.pose.Ry);
        targetPose.push_back(msg.pose.Rz);
        targetSpeed.clear();
        targetSpeed.push_back(msg.speed.x);
        targetSpeed.push_back(msg.speed.y);
        targetSpeed.push_back(msg.speed.z);
        targetSpeed.push_back(msg.speed.Rx);
        targetSpeed.push_back(msg.speed.Ry);
        targetSpeed.push_back(msg.speed.Rz);
        const std::string mode_str = msg.mode;
        moveArg.lastMode = moveArg.mode;
        switch(hashStrUint64(mode_str.c_str())){
        case hashStrUint64("stop"):
        moveArg.mode = stop;
        break;
        case hashStrUint64("moveJ"):
        moveArg.mode = moveJ;
        break;
        case hashStrUint64("moveL"):
        moveArg.mode = moveL;
        break;
        case hashStrUint64("servol"):
        moveArg.mode = servol;
        break;
        case hashStrUint64("force"):
        moveArg.mode = force;
        break;
        case hashStrUint64("speedL"):
        moveArg.mode = speedL;
        break;
        case hashStrUint64("teach"):
        moveArg.mode = teach;
        break;
        default:
      moveArg.mode = stop;
    }
    moveArg.newPoseTarget = true;
    }

  //伺服移动
  void servolMove(void)
  {
    double servolTime = 1.0 / CTRL_FREQ;
    auto t_start = high_resolution_clock::now();
    rtdeControl.servoL(targetPose,
                       moveArg.servol.velocity,
                       moveArg.servol.acceleration,
                       servolTime,
                       moveArg.servol.lookaheadTime,
                       moveArg.servol.gain);
    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);
    if (t_duration.count() < servolTime)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>( servolTime - t_duration.count()));
    }
  }

  //关节位置控制移动
  void moveJMode(void)
  {
    auto t_start = high_resolution_clock::now();
    double cycleTime = 1.0 / CTRL_FREQ;
    rtdeControl.moveJ_IK(targetPose, moveArg.velocity, moveArg.acceleration);
    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);
    if (t_duration.count() < cycleTime)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(cycleTime - t_duration.count()));
    }
  }

  void moveLMode(void)
  {
    double cycleTime = 1.0 / CTRL_FREQ;
    auto t_start = high_resolution_clock::now();
    rtdeControl.moveL(targetPose, 0.2, 0.2);
    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);
    if (t_duration.count() < cycleTime)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(cycleTime - t_duration.count()));
    }
  }

  //force control mode, the function is blocked
  void forceMode(void)
  {
    double cycleTime = 1.0 / CTRL_FREQ;
    auto t_start = high_resolution_clock::now();
    rtdeControl.forceMode(moveArg.force.task_frame,
                          moveArg.force.selection_vector,
                          moveArg.force.wrench,
                          moveArg.force.force_type,
                          moveArg.force.limitsSpeed);
    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);
    if (t_duration.count() < cycleTime)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>(cycleTime - t_duration.count()));
    }
  }
  //笛卡尔速度
  void speedLMove(void)
  {
    double servolTime = 1.0 / CTRL_FREQ;
    auto t_start = high_resolution_clock::now();
    rtdeControl.speedL(targetSpeed,
                       moveArg.speedL.acceleration,
                       moveArg.speedL.returnTime);
    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);
    if (t_duration.count() < servolTime)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>( servolTime - t_duration.count()));
    }
  }

  //拖动试教模式开启
  void teachMode(void)
  {
    rtdeControl.teachMode();
  }

  //stop robot
  void stopMove(void)
  {
    double servolTime = 1.0 / CTRL_FREQ;
    auto t_start = high_resolution_clock::now();
    if(moveArg.lastMode == servol){
      rtdeControl.servoStop();
      rtdeControl.stopJ(1);
    }
    else if(moveArg.lastMode == force){
      rtdeControl.forceModeStop();
      rtdeControl.stopJ(1);
    }
    else if(moveArg.lastMode == moveJ){
      rtdeControl.stopJ(1);
    }
    else if(moveArg.lastMode == moveL){
      rtdeControl.stopJ(1);
    }
    else if(moveArg.lastMode == speedL){
      rtdeControl.stopJ(1);
    }else if(moveArg.lastMode == teach){
      rtdeControl.endTeachMode();
    }
    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);
    if (t_duration.count() < servolTime)
    {
      std::this_thread::sleep_for(std::chrono::duration<double>( servolTime - t_duration.count()));
    }
  }

  ~UR_Control()
  {
    rtdeControl.stopScript();
    delete &rtdeControl;
  }
};

int main(int argc, char* argv[])
{
#if USE_UR_SIM
  UR_Control urR("127.0.0.1");
#else
  UR_Control urR("192.168.0.104");
#endif
  ros::init(argc, argv, "ur_move");
  ros::NodeHandle n;
  ros::Subscriber URMoveSub = n.subscribe("ur_control", 100, &UR_Control::urContralCallback, &urR);   //特别注意Subscribe的第四个参数用法,这里的消息由hrc_vf_main与hrc_vf_main发布
  ros::AsyncSpinner AS(1);
  AS.start();

  urR.moveInit();
  printf("ur have moved to init pose\n");

  //    usleep(200000);

  while(ros::ok())
  {
    if(!urR.moveArg.newPoseTarget){
      usleep(static_cast<int>(1.0 / CTRL_FREQ * 1000000));    //static_cast相当于传统的C语言里的强制转换
      continue;
    }    //在进行第一次调用后，newPoseTarget被设为true
    urR.moveArg.newPoseTarget = false;
    switch(urR.moveArg.mode){
    case stop:
      urR.stopMove();
      break;
    case moveJ:
      urR.moveJMode();
      break;
    case moveL:
      urR.moveLMode();
      break;
    case servol:
      urR.servolMove();
      break;
    case force:
      urR.forceMode();
      break;
    case speedL:
      urR.speedLMove();
      break;
    case teach:
      urR.teachMode();
      break;
    default:
      break;
    }
  }
  delete &urR;
  return 0;
}
