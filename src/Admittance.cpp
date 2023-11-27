/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:08:47 
 * @Last Modified by:   MingshanHe 
 * @Last Modified time: 2021-12-05 04:08:47 
 * @Licence: MIT Licence
 */
#include <arm_control/Admittance.h>

Admittance::Admittance(ros::NodeHandle &n,
    double frequency,
    std::string topic_arm_state,
    std::string topic_arm_command,
    std::string topic_wrench_state,
    std::vector<double> M,
    std::vector<double> D,
    std::vector<double> K,
    std::vector<double> desired_pose,
    std::string base_link,
    std::string end_link,
    double arm_max_vel,
    double arm_max_acc) :
  nh_(n), loop_rate_(frequency),
  M_(M.data()), D_(D.data()),K_(K.data()),desired_pose_(desired_pose.data()),
  arm_max_vel_(arm_max_vel), arm_max_acc_(arm_max_acc),
  UR10e_FT(n, "/ur_ftdata", "/processed_ft_data"){

    
  transM_P = &transM;
  // Process_FT_Data UR10e_FT(nh_, "/ur_ftdata", "/processed_ft_data");
  URDataSub = nh_.subscribe("ur_data", 100, &Process_Robot_Data::robotDataCallback, &URData);
  urConParPub = nh_.advertise<arm_control::RobotControl>("ur_control", 100);

  // 使用多线程，使得不同回调函数互不冲突
  ros::AsyncSpinner AS(2);
  AS.start();
  //* Subscribers
  // sub_arm_state_           = nh_.subscribe(topic_arm_state, 5, 
  //     &Admittance::state_arm_callback, this,ros::TransportHints().reliable().tcpNoDelay());
  // sub_wrench_state_        = nh_.subscribe(topic_wrench_state, 5,
  //     &Admittance::state_wrench_callback, this, ros::TransportHints().reliable().tcpNoDelay());
  
  //* Publishers
  pub_arm_cmd_              = nh_.advertise<geometry_msgs::Twist>(topic_arm_command, 5);

  // initializing the class variables
  arm_position_.setZero();
  arm_twist_.setZero();
  wrench_external_.setZero();
  desired_pose_position_ << desired_pose_.topRows(3);
  desired_pose_orientation_.coeffs() << desired_pose_.bottomRows(4)/desired_pose_.bottomRows(4).norm();

  // Init integrator
  arm_desired_twist_adm_.setZero();

  ft_arm_ready_ = false;
  base_world_ready_ = false;
  world_arm_ready_ = false;

  force_x_pre = 0;
  force_y_pre = 0;
  force_z_pre = 0;
}

//!-                   INITIALIZATION                    -!//

void Admittance::wait_for_transformations() {
  tf::TransformListener listener;
  Matrix6d rot_matrix;
  // Makes sure all TFs exists before enabling all transformations in the callbacks
  // while (!get_rotation_matrix(rot_matrix, listener, "world", base_link_)) {sleep(1);}
  base_world_ready_ = true;
  // while (!get_rotation_matrix(rot_matrix, listener, base_link_, "world")) {sleep(1);}
  world_arm_ready_ = true;
  // while (!get_rotation_matrix(rot_matrix, listener, base_link_, end_link_)) {sleep(1);}
  ft_arm_ready_ = true;
  ROS_INFO("The Force/Torque sensor is ready to use.");
}

//!-                    CONTROL LOOP                     -!//

void Admittance::run() {

  
    
    // 等待力传感器数据队列存满
    while(UR10e_FT.getFTDeque("origin_world").size() < UR10e_FT.FTDataBuffer && ros::ok()){
        loop_rate_.sleep();
    }

    //定义机器人初始位姿，速度
    std::vector<double> initPose = INITPOSE;
    pose << Eigen::Map<Eigen::MatrixXd>(&initPose[0], 6, 1);
    speed << 0.0,0.0,0.0,0.0,0.0,0.0;

    //等待机器人到达初始位姿
    int i = 0;
    while(!URData.isReachTargetPose(pose, 0.001) && ros::ok()) {
        usleep(1000);
        if(i++ > 6000) {
        printf("机器人没有到达指定初始位置\n");
        return;
        }
    }

    //等待力传感器完成校准
    while(UR10e_FT.FTNeedCalibration == true && ros::ok()){
        transM.set_TR_robot_flange(URData.getRobotData("targetFlangePose"));
        UR10e_FT.setFTDateZeroError(0.3);
    }

  ROS_INFO("Running the admittance control loop .................");

  while (nh_.ok()) {

    compute_admittance();

    send_commands_to_robot();

    ros::spinOnce();
    loop_rate_.sleep();
  }
}

//!-                Admittance Dynamics                  -!//

void Admittance::compute_admittance() {
  speed = URData.getRobotData("actualFlangeSpeed");
  pose = URData.getRobotData("actualFlangePose"); 
  arm_position_ << pose[0], pose[1], pose[2];
  Eigen::Vector3d eulerAngle(pose[3], pose[4], pose[5]);
  arm_orientation_ = euler_quaternion(eulerAngle);
  wrench_external_ = UR10e_FT.getFTData("filtered_gtced_ft");
  error.topRows(3) = arm_position_ - desired_pose_position_;
  if(desired_pose_orientation_.coeffs().dot(arm_orientation_.coeffs()) < 0.0)
  {
    arm_orientation_.coeffs() << -arm_orientation_.coeffs();
  }
  Eigen::Quaterniond quat_rot_err (arm_orientation_ * desired_pose_orientation_.inverse());
  if(quat_rot_err.coeffs().norm() > 1e-3)
  {
    quat_rot_err.coeffs() << quat_rot_err.coeffs()/quat_rot_err.coeffs().norm();
  }
  Eigen::AngleAxisd err_arm_des_orient(quat_rot_err);
  error.bottomRows(3) << err_arm_des_orient.axis() * err_arm_des_orient.angle();

  // Translation error w.r.t. desired equilibrium
  Vector6d coupling_wrench_arm;

  
  coupling_wrench_arm=  D_ * (arm_desired_twist_adm_) + K_ * error;
  arm_desired_accelaration = M_.inverse() * ( - coupling_wrench_arm  + wrench_external_);

  double a_acc_norm = (arm_desired_accelaration.segment(0, 3)).norm();

  if (a_acc_norm > arm_max_acc_) {
    ROS_WARN_STREAM_THROTTLE(1, "Admittance generates high arm accelaration!"
                              << " norm: " << a_acc_norm);
    arm_desired_accelaration.segment(0, 3) *= (arm_max_acc_ / a_acc_norm);
  }
  // Integrate for velocity based interface
  ros::Duration duration = loop_rate_.expectedCycleTime();
  arm_desired_twist_adm_ += arm_desired_accelaration * duration.toSec();

  Eigen::Matrix<double,6,1> speed;
  speed << arm_desired_twist_adm_[0], arm_desired_twist_adm_[1], arm_desired_twist_adm_[2], arm_desired_twist_adm_[3], arm_desired_twist_adm_[4], arm_desired_twist_adm_[5];
  URMove.sendSpeedLMsg(urConParPub, speed);
}

//!-                     CALLBACKS                       -!//

// void Admittance::state_arm_callback(
//   const cartesian_state_msgs::PoseTwistConstPtr msg) {
//   arm_position_ <<  msg->pose.position.x,
//                     msg->pose.position.y,
//                     msg->pose.position.z;

//   arm_orientation_.coeffs() <<  msg->pose.orientation.x,
//                                 msg->pose.orientation.y,
//                                 msg->pose.orientation.z,
//                                 msg->pose.orientation.w;

//   arm_twist_ << msg->twist.linear.x,
//                 msg->twist.linear.y,
//                 msg->twist.linear.z,
//                 msg->twist.angular.x,
//                 msg->twist.angular.y,
//                 msg->twist.angular.z;
// }

// void Admittance::state_wrench_callback(
//   const geometry_msgs::WrenchStampedConstPtr msg) {
//   Vector6d wrench_ft_frame;
//   Matrix6d rotation_ft_base;
//   if (ft_arm_ready_) {
//     wrench_ft_frame <<  msg->wrench.force.x,msg->wrench.force.y,msg->wrench.force.z,0,0,0;

//     float force_thres_lower_limit_ = 50;
//     float force_thres_upper_limit_ = 100;

//     // Low-Pass Filter for real robot
//     // if(fabs(wrench_ft_frame(0)) < force_thres_lower_limit_ || fabs(wrench_ft_frame(0)) > force_thres_upper_limit_){wrench_ft_frame(0) = 0;}
//     // else{
//     //   if(wrench_ft_frame(0) > 0){wrench_ft_frame(0) -= force_thres_lower_limit_;}
//     //   else{wrench_ft_frame(0) += force_thres_lower_limit_;}
//     //   wrench_ft_frame(0) = (1 - 0.2)*force_x_pre + 0.2*wrench_ft_frame(0);
//     //   force_x_pre = wrench_ft_frame(0);
//     // }
//     // if(fabs(wrench_ft_frame(1)) < force_thres_lower_limit_ || fabs(wrench_ft_frame(1)) > force_thres_upper_limit_){wrench_ft_frame(1) = 0;}
//     // else{
//     //   if(wrench_ft_frame(1) > 0){wrench_ft_frame(1) -= force_thres_lower_limit_;}
//     //   else{wrench_ft_frame(1) += force_thres_lower_limit_;}
//     //   wrench_ft_frame(1) = (1 - 0.2)*force_y_pre + 0.2*wrench_ft_frame(1);
//     //   force_y_pre = wrench_ft_frame(1);
//     // }
//     // if(fabs(wrench_ft_frame(2)) < force_thres_lower_limit_ || fabs(wrench_ft_frame(2)) > force_thres_upper_limit_){wrench_ft_frame(2) = 0;}
//     // else{
//     //   if(wrench_ft_frame(2) > 0){wrench_ft_frame(2) -= force_thres_lower_limit_;}
//     //   else{wrench_ft_frame(2) += force_thres_lower_limit_;}
//     //   wrench_ft_frame(2) = (1 - 0.2)*force_z_pre + 0.2*wrench_ft_frame(2);
//     //   force_z_pre = wrench_ft_frame(2);
//     // }
//     get_rotation_matrix(rotation_ft_base, listener_ft_, base_link_, end_link_);
//     wrench_external_ <<  rotation_ft_base * wrench_ft_frame;
//   }
// }

//!-               COMMANDING THE ROBOT                  -!//

void Admittance::send_commands_to_robot() {
  // double norm_vel_des = (arm_desired_twist_adm_.segment(0, 3)).norm();

  // if (norm_vel_des > arm_max_vel_) {
  //   ROS_WARN_STREAM_THROTTLE(1, "Admittance generate fast arm movements! velocity norm: " << norm_vel_des);

  //   arm_desired_twist_adm_.segment(0, 3) *= (arm_max_vel_ / norm_vel_des);

  // }
  geometry_msgs::Twist arm_twist_cmd;
  arm_twist_cmd.linear.x  = arm_desired_twist_adm_(0)*0.3;
  arm_twist_cmd.linear.y  = arm_desired_twist_adm_(1)*0.3;
  arm_twist_cmd.linear.z  = arm_desired_twist_adm_(2)*0.3;
  arm_twist_cmd.angular.x = arm_desired_twist_adm_(3)*0.3;
  arm_twist_cmd.angular.y = arm_desired_twist_adm_(4)*0.3;
  arm_twist_cmd.angular.z = arm_desired_twist_adm_(5)*0.3;

  // arm_twist_cmd.linear.x  = arm_desired_twist_adm_(0);
  // arm_twist_cmd.linear.y  = arm_desired_twist_adm_(1);
  // arm_twist_cmd.linear.z  = arm_desired_twist_adm_(2);
  // arm_twist_cmd.angular.x = arm_desired_twist_adm_(3);
  // arm_twist_cmd.angular.y = arm_desired_twist_adm_(4);
  // arm_twist_cmd.angular.z = arm_desired_twist_adm_(5);
  pub_arm_cmd_.publish(arm_twist_cmd);
}

//!-                    UTILIZATION                      -!//

// bool Admittance::get_rotation_matrix(Matrix6d & rotation_matrix,
//     tf::TransformListener & listener,
//     std::string from_frame,
//     std::string to_frame) {
//   tf::StampedTransform transform;
//   Matrix3d rotation_from_to;
//   try {
//     listener.lookupTransform(from_frame, to_frame,
//                             ros::Time(0), transform);
//     tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
//     rotation_matrix.setZero();
//     rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
//     rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
//   }
//   catch (tf::TransformException ex) {
//     rotation_matrix.setZero();
//     ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
//     return false;
//   }
//   return true;
// }

