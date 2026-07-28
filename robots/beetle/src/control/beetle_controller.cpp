#include <beetle/control/beetle_controller.h>

#include <cmath>

using namespace std;

namespace aerial_robot_control
{
  BeetleController::BeetleController():
    GimbalrotorController(),
    pd_wrench_comp_mode_(false),
    pre_module_state_(SEPARATED),
    des_wrench_pub_flag_(true),
    ff_inter_wrench_require_stamp_(false),
    ff_inter_wrench_timeout_(0.0)
  {
  }

  void BeetleController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                    double ctrl_loop_rate
                                    )
  {
    GimbalrotorController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
    wrench_pid_msg_.x.total.resize(1);
    wrench_pid_msg_.x.p_term.resize(1);
    wrench_pid_msg_.x.i_term.resize(1);
    wrench_pid_msg_.x.d_term.resize(1);
    wrench_pid_msg_.y.total.resize(1);
    wrench_pid_msg_.y.p_term.resize(1);
    wrench_pid_msg_.y.i_term.resize(1);
    wrench_pid_msg_.y.d_term.resize(1);
    wrench_pid_msg_.z.total.resize(1);
    wrench_pid_msg_.z.p_term.resize(1);
    wrench_pid_msg_.z.i_term.resize(1);
    wrench_pid_msg_.z.d_term.resize(1);
    wrench_pid_msg_.roll.total.resize(1);
    wrench_pid_msg_.roll.p_term.resize(1);
    wrench_pid_msg_.roll.i_term.resize(1);
    wrench_pid_msg_.roll.d_term.resize(1);
    wrench_pid_msg_.pitch.total.resize(1);
    wrench_pid_msg_.pitch.p_term.resize(1);
    wrench_pid_msg_.pitch.i_term.resize(1);
    wrench_pid_msg_.pitch.d_term.resize(1);
    wrench_pid_msg_.yaw.total.resize(1);
    wrench_pid_msg_.yaw.p_term.resize(1);
    wrench_pid_msg_.yaw.i_term.resize(1);
    wrench_pid_msg_.yaw.d_term.resize(1);

    beetle_robot_model_ = boost::dynamic_pointer_cast<BeetleRobotModel>(robot_model);
    beetle_navigator_ = boost::dynamic_pointer_cast<aerial_robot_navigation::BeetleNavigator>(navigator);
    external_wrench_lower_limit_ = Eigen::VectorXd::Zero(6);
    external_wrench_upper_limit_ = Eigen::VectorXd::Zero(6);
    rosParamInit();
    if(pd_wrench_comp_mode_) ROS_ERROR("PD & Wrench comp mode");
    external_wrench_compensation_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("external_wrench_compensation", 1);
    tagged_external_wrench_pub_ = nh_.advertise<beetle::TaggedWrench>("tagged_wrench", 1);
    whole_external_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("whole_wrench", 1);
    internal_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("internal_wrench", 1);
    wrench_comp_pid_pub_ = nh_.advertise<aerial_robot_msgs::PoseControlPid>("debug/wrench_comp/pid", 1);
    des_inter_wrench_pub_ = nh_.advertise<beetle::TaggedWrenches>("des_inter_wrench", 1, true);
    // Preserve the misspelled legacy topic while users migrate to des_inter_wrench.
    legacy_des_inter_wrench_pub_ = nh_.advertise<beetle::TaggedWrenches>("des_inter_wnrech", 1, true);
    int max_modules_num = beetle_navigator_->getMaxModuleNum();
    for(int i = 0; i < max_modules_num; i++){
      std::string module_name  = string("/") + beetle_navigator_->getMyName() + std::to_string(i+1);
      est_wrench_subs_.insert(make_pair(module_name, nh_.subscribe( module_name + string("/tagged_wrench"), 1, &BeetleController::estExternalWrenchCallback, this)));
      Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
      est_wrench_list_.insert(make_pair(i+1, wrench));
      inter_wrench_list_.insert(make_pair(i+1, wrench));
      wrench_comp_list_.insert(make_pair(i+1, wrench));
      ff_inter_wrench_list_.insert(make_pair(i+1, wrench));
      ff_inter_wrench_stamp_list_.insert(make_pair(i+1, ros::Time(0)));
      ff_inter_wrench_seq_list_.insert(make_pair(i+1, 0));
      ff_inter_wrench_has_stamp_list_.insert(make_pair(i+1, false));
      ff_inter_wrench_receive_time_list_.insert(make_pair(i+1, ros::WallTime(0)));
      ff_inter_wrench_subs_.insert(make_pair(module_name, nh_.subscribe( module_name + string("/ff_inter_wrench"), 1, &BeetleController::ffInterWrenchCallback, this)));
    }
    publishDesiredInteractionWrench();
    pid_controllers_.push_back(PID("f_x", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
    pid_controllers_.push_back(PID("f_y", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
    pid_controllers_.push_back(PID("f_z", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
    pid_controllers_.push_back(PID("t_x", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
    pid_controllers_.push_back(PID("t_y", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));
    pid_controllers_.push_back(PID("t_z", wrench_comp_p_gain_, wrench_comp_i_gain_, wrench_comp_d_gain_));

    ros::NodeHandle control_nh(nh_, "controller");
    ros::NodeHandle wrench_nh(control_nh, "wrench_comp");
    std::vector<int> indices = {FX, FY, FZ, TX, TY, TZ};
    pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(wrench_nh));
    pid_reconf_servers_.back()->setCallback(boost::bind(&BeetleController::cfgPidCallback, this, _1, _2, indices));

    prev_comp_update_time_ = -1;
  }

  void BeetleController::controlCore()
  {
    std::map<int, bool> assembly_flag = beetle_navigator_->getAssemblyFlags();
    int max_modules_num = beetle_navigator_->getMaxModuleNum();
    int module_state = beetle_navigator_-> getModuleState();
    if(beetle_navigator_->getControlFlag() &&
       module_state != SEPARATED){
      calcInteractionWrench();
    }else{
      std::lock_guard<std::mutex> lock(wrench_data_mutex_);
      for(int i = 0; i < max_modules_num; i++) est_wrench_list_[i+1] = Eigen::VectorXd::Zero(6);
      for(int i = 0; i < max_modules_num; i++){
          inter_wrench_list_[i+1] = Eigen::VectorXd::Zero(6);
          wrench_comp_list_[i+1] = Eigen::VectorXd::Zero(6);
        }
    }

    double mass_inv = 1 / beetle_robot_model_->getMass();
    Eigen::Matrix3d inertia_inv = (beetle_robot_model_->getInertia<Eigen::Matrix3d>()).inverse();
    int my_id = beetle_navigator_->getMyID();

    if(module_state == FOLLOWER &&
       pd_wrench_comp_mode_ &&
       beetle_navigator_->getControlFlag()&&
       !beetle_navigator_->pseudo_assembly_mode_){

      /* set proper gains for wrench comp */
      int module_num = 0;
      const auto estimated_wrenches = getEstimatedWrenchSnapshot();
      for(const auto & item : estimated_wrenches){
        if(assembly_flag[item.first]){
          module_num ++;
        }
      }
      std::vector<int> wrench_indices = {FX, FY, FZ, TX, TY, TZ};
      for(const auto& index: wrench_indices)
        {
          pid_controllers_.at(index).setPGain(wrench_comp_p_gain_ / std::pow(2, module_num -2) );
          pid_controllers_.at(index).setDGain(wrench_comp_d_gain_ / std::pow(2, module_num -2) );
          pid_controllers_.at(index).setIGain(wrench_comp_i_gain_ / std::pow(2, module_num -2) );
        }
      Eigen::VectorXd wrench_comp_term_cog = wrench_comp_list_[my_id]; // regarding cog
      Eigen::Matrix3d cog_rot;
      tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);
      Eigen::VectorXd wrench_comp_term = wrench_comp_term_cog; 
      wrench_comp_term.head(3) = cog_rot * wrench_comp_term.head(3); // regarding world

      /* current version: I term reconfig mehod */
      Eigen::VectorXd I_reconfig_acc_cog_term = Eigen::VectorXd::Zero(6);
      I_reconfig_acc_cog_term.head(3) = mass_inv * wrench_comp_term.head(3);
      I_reconfig_acc_cog_term.tail(3) = inertia_inv * wrench_comp_term.tail(3); //inavailable

      double IGain_Fx = pid_controllers_.at(X).getIGain();
      double IGain_Fy = pid_controllers_.at(Y).getIGain();
      double IGain_Fz = pid_controllers_.at(Z).getIGain();
      double IGain_Tx = pid_controllers_.at(ROLL).getIGain();
      double IGain_Ty = pid_controllers_.at(PITCH).getIGain();
      double IGain_Tz = pid_controllers_.at(YAW).getIGain();

      double du;
      if(prev_comp_update_time_ < 0){
        prev_comp_update_time_ = ros::Time::now().toSec();
        return;
      }else{
        du = ros::Time::now().toSec() - prev_comp_update_time_;
        prev_comp_update_time_ = ros::Time::now().toSec();
      }

      pid_controllers_.at(FX).updateWoVel(I_reconfig_acc_cog_term(0) / IGain_Fx, du);
      pid_controllers_.at(FY).updateWoVel(I_reconfig_acc_cog_term(1) / IGain_Fy, du);
      pid_controllers_.at(FZ).updateWoVel(I_reconfig_acc_cog_term(2) / IGain_Fz, du);
      pid_controllers_.at(TX).updateWoVel(I_reconfig_acc_cog_term(3) / IGain_Tx, du);
      pid_controllers_.at(TY).updateWoVel(I_reconfig_acc_cog_term(4) / IGain_Ty, du);
      pid_controllers_.at(TZ).updateWoVel(I_reconfig_acc_cog_term(5) / IGain_Tz, du);

      I_comp_Fx_ = pid_controllers_.at(FX).result();
      I_comp_Fy_ = pid_controllers_.at(FY).result();
      I_comp_Fz_ = pid_controllers_.at(FZ).result();
      I_comp_Tx_ = pid_controllers_.at(TX).result();
      I_comp_Ty_ = pid_controllers_.at(TY).result();
      I_comp_Tz_ = pid_controllers_.at(TZ).result();

      pid_controllers_.at(X).setICompTerm(I_comp_Fx_);
      pid_controllers_.at(Y).setICompTerm(I_comp_Fy_);
      pid_controllers_.at(Z).setICompTerm(I_comp_Fz_);
      pid_controllers_.at(ROLL).setICompTerm(I_comp_Tx_);
      pid_controllers_.at(PITCH).setICompTerm(I_comp_Ty_);
      pid_controllers_.at(YAW).setICompTerm(I_comp_Tz_);
      
      geometry_msgs::WrenchStamped wrench_msg;
      wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
      wrench_msg.wrench.force.x = I_reconfig_acc_cog_term(0);
      wrench_msg.wrench.force.y = I_reconfig_acc_cog_term(1);
      wrench_msg.wrench.force.z = I_reconfig_acc_cog_term(2);
      wrench_msg.wrench.torque.x = I_reconfig_acc_cog_term(3);
      wrench_msg.wrench.torque.y = I_reconfig_acc_cog_term(4);
      wrench_msg.wrench.torque.z = I_reconfig_acc_cog_term(5);
      external_wrench_compensation_pub_.publish(wrench_msg);

      /* publish wrench comp pid value*/
      wrench_pid_msg_.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
      wrench_pid_msg_.x.total.at(0) = pid_controllers_.at(FX).result();
      wrench_pid_msg_.x.p_term.at(0) = pid_controllers_.at(FX).getPTerm();
      wrench_pid_msg_.x.i_term.at(0) = pid_controllers_.at(FX).getITerm();
      wrench_pid_msg_.x.d_term.at(0) = pid_controllers_.at(FX).getDTerm();

      wrench_pid_msg_.y.total.at(0) = pid_controllers_.at(FY).result();
      wrench_pid_msg_.y.p_term.at(0) = pid_controllers_.at(FY).getPTerm();
      wrench_pid_msg_.y.i_term.at(0) = pid_controllers_.at(FY).getITerm();
      wrench_pid_msg_.y.d_term.at(0) = pid_controllers_.at(FY).getDTerm();

      wrench_pid_msg_.z.total.at(0) = pid_controllers_.at(FZ).result();
      wrench_pid_msg_.z.p_term.at(0) = pid_controllers_.at(FZ).getPTerm();
      wrench_pid_msg_.z.i_term.at(0) = pid_controllers_.at(FZ).getITerm();
      wrench_pid_msg_.z.d_term.at(0) = pid_controllers_.at(FZ).getDTerm();
      
      wrench_pid_msg_.roll.total.at(0) = pid_controllers_.at(TX).result();
      wrench_pid_msg_.roll.p_term.at(0) = pid_controllers_.at(TX).getPTerm();
      wrench_pid_msg_.roll.i_term.at(0) = pid_controllers_.at(TX).getITerm();
      wrench_pid_msg_.roll.d_term.at(0) = pid_controllers_.at(TX).getDTerm();

      wrench_pid_msg_.pitch.total.at(0) = pid_controllers_.at(TY).result();
      wrench_pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(TY).getPTerm();
      wrench_pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(TY).getITerm();
      wrench_pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(TY).getDTerm();

      wrench_pid_msg_.yaw.total.at(0) = pid_controllers_.at(TZ).result();
      wrench_pid_msg_.yaw.p_term.at(0) = pid_controllers_.at(TZ).getPTerm();
      wrench_pid_msg_.yaw.i_term.at(0) = pid_controllers_.at(TZ).getITerm();
      wrench_pid_msg_.yaw.d_term.at(0) = pid_controllers_.at(TZ).getDTerm();

      wrench_comp_pid_pub_.publish(wrench_pid_msg_);

    }else{
      pid_controllers_.at(FX).reset();
      pid_controllers_.at(FY).reset();
      pid_controllers_.at(FZ).reset();
      pid_controllers_.at(TX).reset();
      pid_controllers_.at(TY).reset();
      pid_controllers_.at(TZ).reset();
      pid_controllers_.at(X).setICompTerm(0.0);
      pid_controllers_.at(Y).setICompTerm(0.0);
      pid_controllers_.at(Z).setICompTerm(0.0);
      pid_controllers_.at(ROLL).setICompTerm(0.0);
      pid_controllers_.at(PITCH).setICompTerm(0.0);
      pid_controllers_.at(YAW).setICompTerm(0.0);
    }

    publishDesiredInteractionWrench();
      
    GimbalrotorController::controlCore();
    pre_module_state_ = module_state;
    
  }

  void BeetleController::reset()
  {
    GimbalrotorController::reset();
    pid_controllers_.at(FX).reset();
    pid_controllers_.at(FY).reset();
    pid_controllers_.at(FZ).reset();
    pid_controllers_.at(TX).reset();
    pid_controllers_.at(TY).reset();
    pid_controllers_.at(TZ).reset();
    pid_controllers_.at(X).setICompTerm(0.0);
    pid_controllers_.at(Y).setICompTerm(0.0);
    pid_controllers_.at(Z).setICompTerm(0.0);
    pid_controllers_.at(ROLL).setICompTerm(0.0);
    pid_controllers_.at(PITCH).setICompTerm(0.0);
    pid_controllers_.at(YAW).setICompTerm(0.0);
  }

  void BeetleController::calcInteractionWrench()
  {
    /* 1. calculate external wrench W_w for whole system (e.g. ground effects, model error and etc..)*/
    Eigen::VectorXd W_w = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd W_sum = Eigen::VectorXd::Zero(6);
    int module_num = 0;
    std::map<int, bool> assembly_flag = beetle_navigator_->getAssemblyFlags();

    const auto estimated_wrenches = getEstimatedWrenchSnapshot();
    const auto desired_wrenches = getFfInterWrenchSnapshot();
    for(const auto & item : estimated_wrenches){
      if(assembly_flag[item.first]){
      W_sum += item.second;
      module_num ++;
      }
    }

    if(!module_num) return;
    W_w = W_sum / module_num;
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    wrench_msg.wrench.force.x = W_w(0);
    wrench_msg.wrench.force.y = W_w(1);
    wrench_msg.wrench.force.z = W_w(2);
    wrench_msg.wrench.torque.x = W_w(3);
    wrench_msg.wrench.torque.y = W_w(4);
    wrench_msg.wrench.torque.z = W_w(5);
    whole_external_wrench_pub_.publish(wrench_msg);

    /* 2. calculate interactional wrench for each module*/
    Eigen::VectorXd left_inter_wrench = Eigen::VectorXd::Zero(6); //'left_inter_wrench' represents the wrench applied from right-side module to left-side module
    for(const auto & item : estimated_wrenches){
      if(assembly_flag[item.first]){
        Eigen::VectorXd right_inter_wrench = item.second - W_w + left_inter_wrench;
        inter_wrench_list_[item.first] = right_inter_wrench;
        left_inter_wrench = right_inter_wrench;
      }else{
        inter_wrench_list_[item.first] = Eigen::VectorXd::Zero(6);
      }
    }
    int my_id = beetle_navigator_->getMyID();
    wrench_msg.wrench.force.x = inter_wrench_list_[my_id](0);
    wrench_msg.wrench.force.y = inter_wrench_list_[my_id](1);
    wrench_msg.wrench.force.z = inter_wrench_list_[my_id](2);
    wrench_msg.wrench.torque.x = inter_wrench_list_[my_id](3);
    wrench_msg.wrench.torque.y = inter_wrench_list_[my_id](4);
    wrench_msg.wrench.torque.z = inter_wrench_list_[my_id](5);
    internal_wrench_pub_.publish(wrench_msg);
    /* 3. calculate wrench compensation term for each module*/
    int leader_id = beetle_navigator_->getLeaderID();
    /* 3.1. process from leader to left*/
    int right_module_id = leader_id;
    Eigen::VectorXd wrench_comp_sum_left = Eigen::VectorXd::Zero(6);
    for(int i = leader_id-1; i > 0; i--){
      if(assembly_flag[i]){
        wrench_comp_sum_left += -desired_wrenches.at(i) + inter_wrench_list_[i];
        // wrench_comp_list_[i] += wrench_comp_gain_ *  wrench_comp_sum_left;
        wrench_comp_list_[i] = wrench_comp_sum_left;
        right_module_id = i;
      }else{
        wrench_comp_list_[i] = Eigen::VectorXd::Zero(6);
      }
    }
    /* 3.2. process from leader to right*/
    int max_modules_num = beetle_navigator_->getMaxModuleNum();
    int left_module_id = leader_id;
    Eigen::VectorXd wrench_comp_sum_right = Eigen::VectorXd::Zero(6);
    for(int i = leader_id+1; i <= max_modules_num; i++){
      if(assembly_flag[i]){
        wrench_comp_sum_right += desired_wrenches.at(left_module_id) - inter_wrench_list_[left_module_id];
        // wrench_comp_list_[i] += wrench_comp_gain_ * wrench_comp_sum_right;
        wrench_comp_list_[i] = wrench_comp_sum_right;
        left_module_id = i;
      }else{
        wrench_comp_list_[i] = Eigen::VectorXd::Zero(6);
      }
    }
  }
  void BeetleController::rosParamInit()
  {
    GimbalrotorController::rosParamInit();
    ros::NodeHandle control_nh(nh_, "controller");
    getParam<bool>(control_nh, "pd_wrench_comp_mode", pd_wrench_comp_mode_, false);
    getParam<bool>(control_nh, "publish_desired_inter_wrench", des_wrench_pub_flag_, true);
    getParam<bool>(control_nh, "ff_inter_wrench_require_stamp", ff_inter_wrench_require_stamp_, false);
    getParam<double>(control_nh, "ff_inter_wrench_timeout", ff_inter_wrench_timeout_, 0.0);

    double external_force_upper_limit, external_force_lower_limit, external_torque_upper_limit, external_torque_lower_limit;
    getParam<double>(control_nh, "external_force_upper_limit", external_force_upper_limit, 0.5);
    getParam<double>(control_nh, "external_force_lower_limit", external_force_lower_limit, -0.5);
    getParam<double>(control_nh, "external_torque_upper_limit", external_torque_upper_limit, 0.01);
    getParam<double>(control_nh, "external_torque_lower_limit", external_torque_lower_limit, -0.01);
    external_wrench_upper_limit_.head(3) = Eigen::Vector3d::Constant(external_force_upper_limit);
    external_wrench_upper_limit_.tail(3) = Eigen::Vector3d::Constant(external_torque_upper_limit);
    external_wrench_lower_limit_.head(3) = Eigen::Vector3d::Constant(external_force_lower_limit);
    external_wrench_lower_limit_.tail(3) = Eigen::Vector3d::Constant(external_torque_lower_limit);
    ROS_INFO_STREAM("upper limit of external wrench : "<<external_wrench_upper_limit_.transpose());
    ROS_INFO_STREAM("lower limit of external wrench : "<<external_wrench_lower_limit_.transpose());

    getParam<double>(control_nh, "comp_term_update_freq", comp_term_update_freq_, 10);

    ros::NodeHandle wrench_nh(control_nh, "wrench_comp");
    getParam<double>(wrench_nh, "p_gain", wrench_comp_p_gain_, 0.1);
    getParam<double>(wrench_nh, "i_gain", wrench_comp_i_gain_, 0.005);
    getParam<double>(wrench_nh, "d_gain", wrench_comp_d_gain_, 0.07);
  }

  void BeetleController::externalWrenchEstimate()
  {
    const Eigen::VectorXd target_wrench_acc_cog = getTargetWrenchAccCog();

    if(navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE &&
       navigator_->getNaviState() != aerial_robot_navigation::TAKEOFF_STATE &&
       navigator_->getNaviState() != aerial_robot_navigation:: LAND_STATE)
      {
        prev_est_wrench_timestamp_ = 0;
        integrate_term_ = Eigen::VectorXd::Zero(6);
        return;
      }else if(target_wrench_acc_cog.size() == 0){
        ROS_WARN("Target wrench value for wrench estimation is not setted.");
        prev_est_wrench_timestamp_ = 0;
        integrate_term_ = Eigen::VectorXd::Zero(6);
        return;
      }

    Eigen::Vector3d vel_w, omega_cog; // workaround: use the filtered value
    auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::Imu>(estimator_->getImuHandler(0));
    tf::vectorTFToEigen(imu_handler->getFilteredVelCog(), vel_w);
    tf::vectorTFToEigen(imu_handler->getFilteredOmegaCog(), omega_cog);
    Eigen::Matrix3d cog_rot;
    tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);

    Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
    double mass = robot_model_->getMass();

    Eigen::VectorXd sum_momentum = Eigen::VectorXd::Zero(6);
    sum_momentum.head(3) = mass * vel_w;
    sum_momentum.tail(3) = inertia * omega_cog;

    Eigen::VectorXd target_wrench_cog = Eigen::VectorXd::Zero(6);
    target_wrench_cog.head(3) = mass * target_wrench_acc_cog.head(3);
    target_wrench_cog.tail(3) = inertia * target_wrench_acc_cog.tail(3);

    Eigen::MatrixXd J_t = Eigen::MatrixXd::Identity(6,6);
    J_t.topLeftCorner(3,3) = cog_rot;

    Eigen::VectorXd N = mass * robot_model_->getGravity();
    N.tail(3) = aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);

    if(prev_est_wrench_timestamp_ == 0)
      {
        prev_est_wrench_timestamp_ = ros::Time::now().toSec();
        init_sum_momentum_ = sum_momentum; // not good
      }

    double dt = ros::Time::now().toSec() - prev_est_wrench_timestamp_;

    integrate_term_ += (J_t * target_wrench_cog - N + est_external_wrench_) * dt;

    est_external_wrench_ = momentum_observer_matrix_ * (sum_momentum - init_sum_momentum_ - integrate_term_);

    Eigen::VectorXd est_external_wrench_cog = est_external_wrench_;
    est_external_wrench_cog.head(3) = cog_rot.inverse() * est_external_wrench_.head(3);

    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    wrench_msg.wrench.force.x = est_external_wrench_cog(0);
    wrench_msg.wrench.force.y = est_external_wrench_cog(1);
    wrench_msg.wrench.force.z = est_external_wrench_cog(2);
    wrench_msg.wrench.torque.x = est_external_wrench_cog(3);
    wrench_msg.wrench.torque.y = est_external_wrench_cog(4);
    wrench_msg.wrench.torque.z = est_external_wrench_cog(5);
    estimate_external_wrench_pub_.publish(wrench_msg);

    beetle::TaggedWrench tagged_wrench;
    tagged_wrench.index = beetle_navigator_->getMyID();
    tagged_wrench.wrench = wrench_msg;
    tagged_external_wrench_pub_.publish(tagged_wrench);

    prev_est_wrench_timestamp_ = ros::Time::now().toSec();
  }

  void BeetleController::estExternalWrenchCallback(const beetle::TaggedWrench & msg)
  {
    int id = msg.index;
    geometry_msgs::Wrench wrench_msg = msg.wrench.wrench;
    Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
    wrench(0) =  wrench_msg.force.x;
    wrench(1) =  wrench_msg.force.y;
    wrench(2) =  wrench_msg.force.z;
    wrench(3) =  wrench_msg.torque.x;
    wrench(4) =  wrench_msg.torque.y;
    wrench(5) =  wrench_msg.torque.z;
    if(!wrench.allFinite())
      {
        ROS_WARN_THROTTLE(1.0, "Rejecting a non-finite estimated wrench for module %d", id);
        return;
      }
    std::lock_guard<std::mutex> lock(wrench_data_mutex_);
    const auto entry = est_wrench_list_.find(id);
    if(entry == est_wrench_list_.end())
      {
        ROS_WARN_THROTTLE(1.0, "Rejecting an estimated wrench with invalid module id %d", id);
        return;
      }
    entry->second = wrench;
  }

  void BeetleController::ffInterWrenchCallback(const beetle::TaggedWrench & msg)
  {
    int id = msg.index;
    geometry_msgs::Wrench wrench_msg = msg.wrench.wrench;
    Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
    wrench(0) =  wrench_msg.force.x;
    wrench(1) =  wrench_msg.force.y;
    wrench(2) =  wrench_msg.force.z;
    wrench(3) =  wrench_msg.torque.x;
    wrench(4) =  wrench_msg.torque.y;
    wrench(5) =  wrench_msg.torque.z;
    if(!wrench.allFinite())
      {
        ROS_WARN_THROTTLE(1.0, "Rejecting a non-finite ff_inter_wrench for contact %d", id);
        return;
      }

    const ros::Time incoming_stamp = msg.wrench.header.stamp;
    const bool incoming_has_stamp = !incoming_stamp.isZero();
    if(ff_inter_wrench_require_stamp_ && !incoming_has_stamp)
      {
        ROS_WARN_THROTTLE(1.0,
                          "Rejecting unstamped ff_inter_wrench for contact %d; use the stamped command publisher",
                          id);
        return;
      }

    {
      std::lock_guard<std::mutex> lock(wrench_data_mutex_);
      const auto entry = ff_inter_wrench_list_.find(id);
      if(entry == ff_inter_wrench_list_.end())
        {
          ROS_WARN_THROTTLE(1.0, "Rejecting ff_inter_wrench with invalid contact id %d", id);
          return;
        }

      const bool previous_has_stamp = ff_inter_wrench_has_stamp_list_.at(id);
      if(incoming_has_stamp && previous_has_stamp)
        {
          const ros::Time previous_stamp = ff_inter_wrench_stamp_list_.at(id);
          const uint32_t previous_seq = ff_inter_wrench_seq_list_.at(id);
          if(incoming_stamp == previous_stamp && msg.wrench.header.seq == previous_seq)
            {
              if((entry->second - wrench).norm() > 1.0e-12)
                ROS_WARN_THROTTLE(1.0,
                                  "Ignoring conflicting ff_inter_wrench values with the same ordering key for contact %d",
                                  id);
              else
                ff_inter_wrench_receive_time_list_.at(id) = ros::WallTime::now();
              return;
            }
          if(incoming_stamp < previous_stamp ||
             (incoming_stamp == previous_stamp && msg.wrench.header.seq < previous_seq))
            {
              ROS_WARN_THROTTLE(1.0,
                                "Ignoring stale ff_inter_wrench for contact %d (stamp %.9f, latest %.9f)",
                                id, incoming_stamp.toSec(), previous_stamp.toSec());
              return;
            }
        }
      else if(!incoming_has_stamp && previous_has_stamp)
        {
          ROS_WARN_THROTTLE(1.0,
                            "Ignoring unstamped ff_inter_wrench for contact %d after stamped commands were received",
                            id);
          return;
        }

      entry->second = wrench;
      ff_inter_wrench_receive_time_list_.at(id) = ros::WallTime::now();
      if(incoming_has_stamp)
        {
          ff_inter_wrench_stamp_list_.at(id) = incoming_stamp;
          ff_inter_wrench_seq_list_.at(id) = msg.wrench.header.seq;
          ff_inter_wrench_has_stamp_list_.at(id) = true;
        }
    }
    publishDesiredInteractionWrench();
  }

  void BeetleController::setFfInterWrench(int id, const Eigen::VectorXd& desired_wrench)
  {
    if(desired_wrench.size() != 6 || !desired_wrench.allFinite()) return;
    {
      std::lock_guard<std::mutex> lock(wrench_data_mutex_);
      const auto entry = ff_inter_wrench_list_.find(id);
      if(entry == ff_inter_wrench_list_.end()) return;
      entry->second = desired_wrench;
      ff_inter_wrench_receive_time_list_.at(id) = ros::WallTime::now();
    }
    publishDesiredInteractionWrench();
  }

  std::map<int, Eigen::VectorXd> BeetleController::getEstimatedWrenchSnapshot() const
  {
    std::lock_guard<std::mutex> lock(wrench_data_mutex_);
    return est_wrench_list_;
  }

  std::map<int, Eigen::VectorXd> BeetleController::getFfInterWrenchSnapshot() const
  {
    std::lock_guard<std::mutex> lock(wrench_data_mutex_);
    std::map<int, Eigen::VectorXd> snapshot = ff_inter_wrench_list_;
    if(ff_inter_wrench_timeout_ <= 0.0) return snapshot;

    const ros::WallTime now = ros::WallTime::now();
    for(auto& item: snapshot)
      {
        const ros::WallTime received = ff_inter_wrench_receive_time_list_.at(item.first);
        if(!received.isZero() && (now - received).toSec() > ff_inter_wrench_timeout_)
          item.second.setZero();
      }
    return snapshot;
  }

  void BeetleController::publishDesiredInteractionWrench()
  {
    if(!des_wrench_pub_flag_) return;

    beetle::TaggedWrenches msg;
    {
      std::lock_guard<std::mutex> lock(wrench_data_mutex_);
      msg.tagged_wrenches.reserve(ff_inter_wrench_list_.size());
      const ros::WallTime now = ros::WallTime::now();
      for(const auto& item: ff_inter_wrench_list_)
      {
        beetle::TaggedWrench tagged;
        tagged.index = item.first;
        tagged.wrench.header.seq = ff_inter_wrench_seq_list_.at(item.first);
        tagged.wrench.header.stamp = ff_inter_wrench_has_stamp_list_.at(item.first)
          ? ff_inter_wrench_stamp_list_.at(item.first)
          : ros::Time(0);
        tagged.wrench.header.frame_id = "accepted_ff_inter_wrench";

        Eigen::VectorXd effective_wrench = item.second;
        const ros::WallTime received = ff_inter_wrench_receive_time_list_.at(item.first);
        if(ff_inter_wrench_timeout_ > 0.0 && !received.isZero() &&
           (now - received).toSec() > ff_inter_wrench_timeout_)
          effective_wrench.setZero();

        tagged.wrench.wrench.force.x = effective_wrench(0);
        tagged.wrench.wrench.force.y = effective_wrench(1);
        tagged.wrench.wrench.force.z = effective_wrench(2);
        tagged.wrench.wrench.torque.x = effective_wrench(3);
        tagged.wrench.wrench.torque.y = effective_wrench(4);
        tagged.wrench.wrench.torque.z = effective_wrench(5);
        msg.tagged_wrenches.push_back(tagged);
      }
    }
    des_inter_wrench_pub_.publish(msg);
    legacy_des_inter_wrench_pub_.publish(msg);
  }

} //namespace aerial_robot_controller

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::BeetleController, aerial_robot_control::ControlBase);
