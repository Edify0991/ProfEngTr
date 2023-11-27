#include "arm_control/process_ft_data.h"

Process_FT_Data::Process_FT_Data(ros::NodeHandle& node_handle, const std::string sub_name, const std::string pub_name): subName(sub_name),pubName(pub_name),n(node_handle)
{
  // 获取并处理力数据的函数单独开一个线程，与主线程并行
  // std::thread getftdata(getftOriData);
  // getftdata.detach();
  sub = n.subscribe<geometry_msgs::WrenchStamped>(subName, 128, boost::bind(&Process_FT_Data::ftDataCallback, this, _1));
  pub = n.advertise<arm_control::ProcessedFTData>(pubName, 128);  // 发布处理好的力数据

  double G(0);
  double U(0);
  double V(0);
  Eigen::Matrix<double,3,1> F0({0, 0, 0});
  Eigen::Matrix<double,3,1> T0({0, 0, 0});
  Eigen::Matrix<double,3,1> P({0, 0, 0});
  Eigen::Matrix<double,3,1> A({0, 0, 0});

  GCArgs.G = G;
  GCArgs.U = U;
  GCArgs.V = V;
  GCArgs.F0 = F0;
  GCArgs.T0 = T0;
  GCArgs.P = P;
  GCArgs.A = A;

  std::cout<<"重力补偿参数为：G-[" << GCArgs.G << "] U-" << GCArgs.U << " V-" << GCArgs.V
    << " F0-[" << GCArgs.F0 << "] T0-[" << GCArgs.T0 << "] P-" << GCArgs.P << " A-" << GCArgs.A << std::endl;
}


void Process_FT_Data::ftDataCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
  std::lock_guard<std::mutex> some_guard(ft_mutex);

  Eigen::Matrix<double,6,1> tmpFT;
  tmpFT << msg->wrench.force.x,
      msg->wrench.force.y,
      msg->wrench.force.z,
      msg->wrench.torque.x,
      msg->wrench.torque.y,
      msg->wrench.torque.z;

  if(origin_ft.size() >= FTDataBuffer) {
      origin_ft.pop_front();
      filtered_ft.pop_front();
      origin_gtced_ft.pop_front();
      filtered_gtced_ft.pop_front();
      origin_world.pop_front();
      filtered_world.pop_front();
      origin_gtced_world.pop_front();
      filtered_gtced_world.pop_front();
  }
  origin_ft.push_back(tmpFT);

  //力传感器滤波
  tmpFT = filter(origin_ft, filtered_ft);
  filtered_ft.push_back(tmpFT);

  //重力补偿
  tmpFT = gravityCompensation(GCArgs, transM_P->R_robot_flange*transM_P->R_flange_ft);
  origin_gtced_ft.push_back(origin_ft.back() - tmpFT);
  filtered_gtced_ft.push_back(filtered_ft.back() - tmpFT);

  // 最终需要的是笛卡尔空间下的力
  origin_world.push_back(transFT_World(origin_world.back(), transM_P));
  filtered_world.push_back(transFT_World(filtered_ft.back(), transM_P));
  origin_gtced_world.push_back(transFT_World(origin_gtced_world.back(), transM_P));
  filtered_gtced_world.push_back(transFT_World(filtered_gtced_ft.back(), transM_P));

  pubFTMsg.origin_ft.resize(origin_ft.back().size());
  Eigen::Map<Eigen::Matrix<double, 6, 1>>(&pubFTMsg.origin_ft[0], origin_ft.back().size()) = origin_ft.back();  // 将double类型的 6x1 Matrix中的力传感器最新的数据赋值给pubFTMsg

  pubFTMsg.filtered_ft.resize(filtered_ft.back().size());
  Eigen::Map<Eigen::Matrix<double, 6, 1>>(&pubFTMsg.filtered_ft[0],filtered_ft.back().size()) = filtered_ft.back();

  pubFTMsg.origin_gtced_ft.resize(origin_gtced_ft.back().size());
  Eigen::Map<Eigen::Matrix<double,6,1>>(&pubFTMsg.origin_gtced_ft[0],origin_gtced_ft.back().size()) = origin_gtced_ft.back();

  pubFTMsg.filtered_gtced_ft.resize(filtered_gtced_ft.back().size());
  Eigen::Map<Eigen::Matrix<double,6,1>>(&pubFTMsg.filtered_gtced_ft[0],filtered_gtced_ft.back().size()) = filtered_gtced_ft.back();

  pubFTMsg.origin_world.resize(origin_world.back().size());
  Eigen::Map<Eigen::Matrix<double,6,1>>(&pubFTMsg.origin_world[0],origin_world.back().size()) = origin_world.back();

  pubFTMsg.filtered_world.resize(filtered_world.back().size());
  Eigen::Map<Eigen::Matrix<double,6,1>>(&pubFTMsg.filtered_world[0],filtered_world.back().size()) = filtered_world.back();

  pubFTMsg.origin_gtced_world.resize(origin_gtced_world.back().size());
  Eigen::Map<Eigen::Matrix<double,6,1>>(&pubFTMsg.origin_gtced_world[0],origin_gtced_world.back().size()) = origin_gtced_world.back();

  pubFTMsg.filtered_gtced_world.resize(filtered_gtced_world.back().size());
  Eigen::Map<Eigen::Matrix<double,6,1>>(&pubFTMsg.filtered_gtced_world[0],filtered_gtced_world.back().size()) = filtered_gtced_world.back();

  pubFTMsg.header.stamp = ros::Time::now();
  pub.publish(pubFTMsg);
}

std::deque<Eigen::Matrix<double,6,1>> Process_FT_Data::getFTDeque(const char *FTType)
{
  std::lock_guard<std::mutex> some_guard(ft_mutex);
  std::deque<Eigen::Matrix<double,6,1>> tmpFT;

  switch(hashStrUint64(FTType)){
    case hashStrUint64("origen_ft"):
      tmpFT = origin_ft;
      break;
    case hashStrUint64("filtered_ft"):
      tmpFT = filtered_ft;
      break;
    case hashStrUint64("origen_gtced_ft"):
      tmpFT = origin_gtced_ft;
      break;
    case hashStrUint64("filtered_gtced_ft"):
      tmpFT = filtered_gtced_ft;
      break;
    case hashStrUint64("origen_world"):
      tmpFT = origin_world;
      break;
    case hashStrUint64("filtered_world"):
      tmpFT = filtered_world;
      break;
    case hashStrUint64("origen_gtced_world"):
      tmpFT = origin_gtced_world;
      break;
    case hashStrUint64("filtered_gtced_world"):
      tmpFT = filtered_gtced_world;
      break;
    default:
      printf("getFTDeque函数错误输入\n");
      break;
  }
  return tmpFT;
}

Eigen::Matrix<double,6,1> Process_FT_Data::getFTData(const char *FTType)
{
  std::lock_guard<std::mutex> some_guard(ft_mutex);
  Eigen::Matrix<double,6,1> tmpFT;
  switch(hashStrUint64(FTType)){
  case hashStrUint64("origen_ft"):
    tmpFT = origin_ft.back();
    break;
  case hashStrUint64("filtered_ft"):
    tmpFT = filtered_ft.back();
    break;
  case hashStrUint64("origen_gtced_ft"):
    tmpFT = origin_gtced_ft.back();
    break;
  case hashStrUint64("filtered_gtced_ft"):
    tmpFT = filtered_gtced_ft.back();
    break;
  case hashStrUint64("origen_world"):
    tmpFT = origin_world.back();
    break;
  case hashStrUint64("filtered_world"):
    tmpFT = filtered_world.back();
    break;
  case hashStrUint64("origen_gtced_world"):
    tmpFT = origin_gtced_world.back();
    break;
  case hashStrUint64("filtered_gtced_world"):
    tmpFT = filtered_gtced_world.back();
    break;
  default:
    printf("getFT函数错误输入\n");
    break;
  }
  return tmpFT;
}

Eigen::Matrix<double,6,1> Process_FT_Data::transFT_World(const Eigen::Matrix<double,6,1>& FTData, Transfer_Matrix* T)
{
  Eigen::Matrix<double,6,1> tmpFT;
  tmpFT << T->R_world_robot * T->R_robot_flange * T->R_flange_ft * FTData.block<3, 1>(0, 0),
      T->R_world_robot * T->R_robot_flange * T->R_flange_ft * FTData.block<3, 1>(3, 0);
  return tmpFT;
}

Eigen::Matrix<double,6,1> Process_FT_Data::filter(
    const std::deque<Eigen::Matrix<double,6,1>> &originData,
    const std::deque<Eigen::Matrix<double,6,1>> &processedData,
    const std::vector<double> a,
    const std::vector<double> b) {
  Eigen::Matrix<double,6,1> tmpFT;
  tmpFT = Eigen::MatrixXd::Zero(6,1);
  if(originData.size() < a.size() || processedData.size() < b.size() - 1 ) {
    return tmpFT;
  }

  for(unsigned long i = 0; i < b.size(); i++) {
    tmpFT += *(b.begin() + i) * *(originData.end() - i - 1);
  }

  if(a.size() > 1) {
    for(unsigned long i = 1; i < a.size(); i++) {
      tmpFT -= *(a.begin() + i) * *(processedData.end() - i);
    }
  }
  return tmpFT;
}

// 工具,力传感器重力补偿
// 返回重力补偿值，用当前力传感器值减去重力补偿值即可得到重力补偿后的值
Eigen::Matrix<double,6,1> Process_FT_Data::gravityCompensation(
    const Gravity_Compensation_Args &GCArgs,
    const Eigen::Matrix<double,3,3> R_robot_ft) {
  Eigen::Matrix<double,6,1> FTc;
  Eigen::Matrix<double,3,3> rotationMatrix;
  Eigen::Matrix<double,3,1> gx, mgx;
  FTc = Eigen::Matrix<double,6,1>::Zero();

  gx = R_robot_ft.transpose() * GCArgs.A; // 传感器坐标系下的重力
  mgx << gx(2) * GCArgs.P(1) - gx(1) * GCArgs.P(2),
      gx(0) * GCArgs.P(2) - gx(2) * GCArgs.P(0),
      gx(1) * GCArgs.P(0) - gx(0) * GCArgs.P(1);
  FTc.block<3,1>(0,0) = GCArgs.F0 + gx;
  FTc.block<3,1>(3,0) = GCArgs.T0 + mgx;
  return FTc;
}

//分析力传感器数据队列中的数据
FT_Data_Analyze Process_FT_Data::analyzeFTDataQueue(const char *FTType){
  Eigen::Matrix<double,6,1> sum, accum;
  sum = accum = Eigen::MatrixXd::Zero(6,1);
  FT_Data_Analyze Args;

  Args.Deque = getFTDeque(FTType);


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

//校准力传感器
bool Process_FT_Data::setFTDateZeroError(const double maxStdevSum)
{
  std::lock_guard<std::mutex> some_guard(ft_mutex);
  std::deque<Eigen::Matrix<double,6,1>> tmpFTDeque;
  Eigen::Matrix<double,6,1> sum, mean, accum, stdev;
  sum = mean = accum = stdev = Eigen::MatrixXd::Zero(6,1);
  double stdevSum = 0;

  if(FTNeedCalibration == false){
    printf("FT sensor data don't need to Calibrate. \n");
    return true;
  }

  std::for_each(std::begin(origin_gtced_ft), std::end(origin_gtced_ft), [&](const Eigen::Matrix<double,6,1> data){
    sum += data;
  });

  mean = sum / origin_gtced_ft.size();

  std::for_each(std::begin(origin_gtced_ft), std::end(origin_gtced_ft), [&](const Eigen::Matrix<double,6,1> data){
    accum += ((data - mean).array() * (data - mean).array()).matrix();
  });
  stdev = sqrt(accum.array()/(origin_gtced_ft.size()-1)).matrix();
  stdevSum = stdev.rowwise().sum()(0,0);

  if(stdevSum < maxStdevSum){
    GCArgs.F0 += mean.block<3,1>(0,0);
    GCArgs.T0 += mean.block<3,1>(3,0);
    printf("FT sensor data have been Calibrated. \n");
    std::cout << "FT Date Zero Error = " << mean.transpose() << std::endl;
    FTNeedCalibration = false;
    return true;
  }else{
    printf("Stdev is too large to Calibrate FT sensor data. Stdev = %f \n",stdevSum);
    return false;
  }
}

// int main(int argc, char **argv) {
//   ros::init(argc, argv, "force_data");
//   ros::NodeHandle n;
//   ros::Subscriber L_wrench_from_sensor;
//   L_wrench_from_sensor = n.subscribe("/wrench", 10, &ftDataCallback);
//   ros::Publisher urDataPub = n.advertise<hrc::RobotData>("ur_data", 1);

//   //等待力传感器数据队列存满
//   while(KW_FT.getFTDeque("origen_world").size() < KW_FT.FTDataBuffer && ros::ok()) {
//     loop_rate.sleep();
//   }
//   ros::Rate loop_rate(128);
//   return 0;
// }