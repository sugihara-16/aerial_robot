#include <twin_hammer/control/twin_hammer_controller.h>

using namespace aerial_robot_control;

TwinHammerController::TwinHammerController():
  PoseLinearController()
{
}

void TwinHammerController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                  double ctrl_loop_rate
                                  )
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  twin_hammer_model_ = boost::dynamic_pointer_cast<TwinHammerModel>(robot_model);

  target_base_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_ * 2, 0);

  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  // rosParamInit();
}

void TwinHammerController::controlCore()
{
  PoseLinearController::controlCore();
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
  target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(),target_acc_cog.y(),target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();
  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x,target_ang_acc_y,target_ang_acc_z);

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();
  
  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3*motor_num_);

  double mass_inv = 1/twin_hammer_model_->getMass();
  Eigen::Matrix3d inertia = twin_hammer_model_->getInertia<Eigen::Matrix3d>();
  Eigen::Matrix3d inertia_inv = (twin_hammer_model_->getInertia<Eigen::Matrix3d>()).inverse();
  std::vector<Eigen::Vector3d> rotors_origin_from_cog = twin_hammer_model_->getRotorsOriginFromCog<Eigen::Vector3d>();

  double t = ros::Time::now().toSec();

  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6,3);
  wrench_map.block(0,0,3,3) = Eigen::MatrixXd::Identity(3,3);
  int last_col = 0;

  for(int i=0; i<motor_num_; i++)
  {
    wrench_map.block(3,0,3,3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i));
    full_q_mat.middleCols(last_col, 3) = wrench_map;
    last_col += 3;
  }

  full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);
  Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat);
  target_vectoring_f_ = full_q_mat_inv * target_wrench_acc_cog;
  last_col = 0;

  // for(int i=0; i<full_q_mat_inv.rows(); i++){
  //   for(int j=0; j<full_q_mat_inv.cols(); j++){
  //     std::cout << full_q_mat_inv(i,j) << ",";
  //   }
  //   std::cout << std::endl;
  // }

  for(int i=0; i<motor_num_; i++)
  {
    Eigen::Vector3d f_i = target_vectoring_f_.segment(last_col,3);

    target_base_thrust_.at(i) = f_i.norm();

    double gimbal_i_roll = atan2(-f_i.y(), f_i.z());
    double gimbal_i_pitch = atan2(f_i.x(), -f_i.y() * sin(gimbal_i_roll) + f_i.z() * cos(gimbal_i_roll));
    target_gimbal_angles_.at(2*i) = gimbal_i_roll;
    target_gimbal_angles_.at(2*i+1) = gimbal_i_pitch;
    last_col += 3;
  } 
}

void TwinHammerController::sendCmd()
{
  PoseLinearController::sendCmd();
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);

  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.header.stamp = ros::Time::now();
  for(int i=0; i<motor_num_; i++)
  {
    gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
  }
  gimbal_control_pub_.publish(gimbal_control_msg);
}

// void TwinHammerController::rosParamInit()
// {
//   ros::NodeHandle control_nh(nh_, "controller");
//   getParam<int>(control_nh, "gimbal_dof", gimbal_dof_, 1);
//   getParam<bool>(control_nh, "gimbal_calc_in_fc", gimbal_calc_in_fc_, true);
//   getParam<bool>(control_nh, "i_term_rp_calc_in_pc", i_term_rp_calc_in_pc_, false);
// }


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::TwinHammerController, aerial_robot_control::ControlBase);
