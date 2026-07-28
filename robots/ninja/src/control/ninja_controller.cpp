#include <ninja/control/ninja_controller.h>

using namespace std;

namespace aerial_robot_control
{
  NinjaController::NinjaController():
    BeetleController(),
    joint_control_timestamp_(-1)
  {}
  void NinjaController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                    double ctrl_loop_rate
                                    )
  {
    BeetleController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
    ninja_navigator_ = boost::dynamic_pointer_cast<aerial_robot_navigation::NinjaNavigator>(navigator);
    ninja_robot_model_ = boost::dynamic_pointer_cast<NinjaRobotModel>(robot_model);
    pid_controllers_.push_back(PID("joint_pitch", joint_p_gain_, joint_i_gain_, joint_d_gain_));
    pid_controllers_.push_back(PID("joint_yaw", joint_p_gain_, joint_i_gain_, joint_d_gain_));
    // ninja_robot_model_->copyTreeStructure(ninja_robot_model_->getInitModuleTree(), module_tree_for_control_);

    pseudo_assembly_flag_sub_ = nh_.subscribe("/pseudo_assembly_flag",1,&NinjaController::pseudoAsmCallback, this);
    com_motion_pid_pub_ = nh_.advertise<aerial_robot_msgs::PoseControlPid>("debug/com_motion/pid", 1);

    com_motion_pid_msg_.x.total.resize(1);
    com_motion_pid_msg_.x.p_term.resize(1);
    com_motion_pid_msg_.x.i_term.resize(1);
    com_motion_pid_msg_.x.d_term.resize(1);
    com_motion_pid_msg_.y.total.resize(1);
    com_motion_pid_msg_.y.p_term.resize(1);
    com_motion_pid_msg_.y.i_term.resize(1);
    com_motion_pid_msg_.y.d_term.resize(1);
    com_motion_pid_msg_.z.total.resize(1);
    com_motion_pid_msg_.z.p_term.resize(1);
    com_motion_pid_msg_.z.i_term.resize(1);
    com_motion_pid_msg_.z.d_term.resize(1);
    com_motion_pid_msg_.roll.total.resize(1);
    com_motion_pid_msg_.roll.p_term.resize(1);
    com_motion_pid_msg_.roll.i_term.resize(1);
    com_motion_pid_msg_.roll.d_term.resize(1);
    com_motion_pid_msg_.pitch.total.resize(1);
    com_motion_pid_msg_.pitch.p_term.resize(1);
    com_motion_pid_msg_.pitch.i_term.resize(1);
    com_motion_pid_msg_.pitch.d_term.resize(1);
    com_motion_pid_msg_.yaw.total.resize(1);
    com_motion_pid_msg_.yaw.p_term.resize(1);
    com_motion_pid_msg_.yaw.i_term.resize(1);
    com_motion_pid_msg_.yaw.d_term.resize(1);
  }

  bool NinjaController::update()
  {
    if(!ninja_navigator_->getControlFlag())
      joint_control_timestamp_ = -1;
    else if(ninja_navigator_->getControlFlag() && joint_control_timestamp_ < 0)
      joint_control_timestamp_ = ros::Time::now().toSec();
    return GimbalrotorController::update();
  }

  void NinjaController::controlCore()
  {
    if(ninja_navigator_->getCurrentAssembled() && joint_control_timestamp_ > 0 && ninja_navigator_->getFreeJointFlag())
      {
        int my_id = ninja_navigator_->getMyID();
        int leader_id = ninja_navigator_->getLeaderID();
        std::vector<int> assembled_modules_ids = ninja_navigator_->getAssemblyIds();
        double du = ros::Time::now().toSec() - joint_control_timestamp_;
        std::vector<double> joint_errs =  ninja_navigator_->getJointPosErr();
        pid_controllers_.at(JOINT_TY).updateWoVel(joint_errs.at(0),du);
        pid_controllers_.at(JOINT_TZ).updateWoVel(joint_errs.at(1),du);
        Eigen::VectorXd joint_ff_wrench = Eigen::VectorXd::Zero(6);
        joint_ff_wrench.topRows(4) = Eigen::VectorXd::Zero(4);
        joint_ff_wrench(4) = pid_controllers_.at(JOINT_TY).result();
        joint_ff_wrench(5) = pid_controllers_.at(JOINT_TZ).result();
        if(my_id < leader_id)
          {
            setFfInterWrench(my_id,-joint_ff_wrench);
          }
        else if(my_id > leader_id)
          {
            int left_module_id = assembled_modules_ids[ninja_navigator_->getMyIndex() -1];
            setFfInterWrench(left_module_id, joint_ff_wrench);
          }
      }
    joint_control_timestamp_ = ros::Time::now().toSec();

    com_motion_pid_msg_.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    
    tf::Vector3 curr_com_pos = ninja_navigator_->getCurrComPos();
    tf::Vector3 curr_com_vel = ninja_navigator_->getCurrComVel();
    tf::Vector3 target_com_pos = ninja_navigator_->getTargetFinalPosCand();
    tf::Vector3 target_com_vel = ninja_navigator_->getTargetVelCand();
    tf::Vector3 com_pos_err = target_com_pos - curr_com_pos;
    tf::Vector3 com_vel_err = target_com_vel - curr_com_vel;

    tf::Vector3 curr_com_rpy = ninja_navigator_->getCurrComRPY();
    tf::Vector3 curr_com_omega = ninja_navigator_->getCurrComOmega();
    tf::Vector3 target_com_rpy = ninja_navigator_->getTargetFinalRPYCand();
    tf::Vector3 target_com_omega = ninja_navigator_->getTargetOmegaCand();
    tf::Vector3 com_rpy_err = target_com_rpy - curr_com_rpy;
    tf::Vector3 com_omega_err = target_com_omega - curr_com_omega;

    // com_motion_pid_msg_.x.total.at(0) = pid_controllers_.at(FX).result();
    // com_motion_pid_msg_.x.p_term.at(0) = pid_controllers_.at(FX).getPTerm();
    // com_motion_pid_msg_.x.i_term.at(0) = pid_controllers_.at(FX).getITerm();
    // com_motion_pid_msg_.x.d_term.at(0) = pid_controllers_.at(FX).getDTerm();
    com_motion_pid_msg_.x.target_p = target_com_pos.x();
    com_motion_pid_msg_.x.err_p = com_pos_err.x();
    com_motion_pid_msg_.x.target_d = target_com_vel.x();
    com_motion_pid_msg_.x.err_d = com_vel_err.x();

    // com_motion_pid_msg_.y.total.at(0) = pid_controllers_.at(FY).result();
    // com_motion_pid_msg_.y.p_term.at(0) = pid_controllers_.at(FY).getPTerm();
    // com_motion_pid_msg_.y.i_term.at(0) = pid_controllers_.at(FY).getITerm();
    // com_motion_pid_msg_.y.d_term.at(0) = pid_controllers_.at(FY).getDTerm();
    com_motion_pid_msg_.y.target_p = target_com_pos.y();
    com_motion_pid_msg_.y.err_p = com_pos_err.y();
    com_motion_pid_msg_.y.target_d = target_com_vel.y();
    com_motion_pid_msg_.y.err_d = com_vel_err.y();

    // com_motion_pid_msg_.z.total.at(0) = pid_controllers_.at(FZ).result();
    // com_motion_pid_msg_.z.p_term.at(0) = pid_controllers_.at(FZ).getPTerm();
    // com_motion_pid_msg_.z.i_term.at(0) = pid_controllers_.at(FZ).getITerm();
    // com_motion_pid_msg_.z.d_term.at(0) = pid_controllers_.at(FZ).getDTerm();
    com_motion_pid_msg_.z.target_p = target_com_pos.z();
    com_motion_pid_msg_.z.err_p = com_pos_err.z();
    com_motion_pid_msg_.z.target_d = target_com_vel.z();
    com_motion_pid_msg_.z.err_d = com_vel_err.z();

    // com_motion_pid_msg_.roll.total.at(0) = pid_controllers_.at(ROLL).result();
    // com_motion_pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
    // com_motion_pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
    // com_motion_pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
    com_motion_pid_msg_.roll.target_p = target_com_rpy.x();
    com_motion_pid_msg_.roll.err_p = com_rpy_err.x();
    com_motion_pid_msg_.roll.target_d = target_com_omega.x();
    com_motion_pid_msg_.roll.err_d = com_omega_err.x();

    // com_motion_pid_msg_.pitch.total.at(0) = pid_contpitchers_.at(PITCH).result();
    // com_motion_pid_msg_.pitch.p_term.at(0) = pid_contpitchers_.at(PITCH).getPTerm();
    // com_motion_pid_msg_.pitch.i_term.at(0) = pid_contpitchers_.at(PITCH).getITerm();
    // com_motion_pid_msg_.pitch.d_term.at(0) = pid_contpitchers_.at(PITCH).getDTerm();
    com_motion_pid_msg_.pitch.target_p = target_com_rpy.y();
    com_motion_pid_msg_.pitch.err_p = com_rpy_err.y();
    com_motion_pid_msg_.pitch.target_d = target_com_omega.y();
    com_motion_pid_msg_.pitch.err_d = com_omega_err.y();

    // com_motion_pid_msg_.yaw.total.at(0) = pid_contyawers_.at(YAW).result();
    // com_motion_pid_msg_.yaw.p_term.at(0) = pid_contyawers_.at(YAW).getPTerm();
    // com_motion_pid_msg_.yaw.i_term.at(0) = pid_contyawers_.at(YAW).getITerm();
    // com_motion_pid_msg_.yaw.d_term.at(0) = curr_com_rpy.z();
    com_motion_pid_msg_.yaw.target_p = target_com_rpy.z();
    com_motion_pid_msg_.yaw.err_p = com_rpy_err.z();
    com_motion_pid_msg_.yaw.target_d = target_com_omega.z();
    com_motion_pid_msg_.yaw.err_d = com_omega_err.z();
    //curr_com_rpy.z = target_com_rpy.z


    com_motion_pid_pub_.publish(com_motion_pid_msg_);

    BeetleController::controlCore();
  }

  void NinjaController::externalWrenchEstimate()
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
    // ROS_ERROR_STREAM(cog_rot);

    std::string my_name = ninja_navigator_->getMyName() + std::to_string(ninja_navigator_->getMyID());
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    wrench_msg.header.frame_id =my_name + "/cog";
    wrench_msg.wrench.force.x = est_external_wrench_cog(0);
    wrench_msg.wrench.force.y = est_external_wrench_cog(1);
    wrench_msg.wrench.force.z = est_external_wrench_cog(2);
    wrench_msg.wrench.torque.x = est_external_wrench_cog(3);
    wrench_msg.wrench.torque.y = est_external_wrench_cog(4);
    wrench_msg.wrench.torque.z = est_external_wrench_cog(5);
    estimate_external_wrench_pub_.publish(wrench_msg);

    // Convert the complete wrench, including the moment-arm term, from COG to COM.
    const Eigen::Matrix<double, 6, 1> est_external_wrench_com =
      ninja_navigator_->getCog2ComWrenchXStar() * est_external_wrench_cog;

    geometry_msgs::WrenchStamped wrench_msg_com;
    wrench_msg_com.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    wrench_msg_com.header.frame_id = my_name + "/center_of_moving";
    wrench_msg_com.wrench.force.x = est_external_wrench_com(0);
    wrench_msg_com.wrench.force.y = est_external_wrench_com(1);
    wrench_msg_com.wrench.force.z = est_external_wrench_com(2);
    wrench_msg_com.wrench.torque.x = est_external_wrench_com(3);
    wrench_msg_com.wrench.torque.y = est_external_wrench_com(4);
    wrench_msg_com.wrench.torque.z = est_external_wrench_com(5);
    
    beetle::TaggedWrench tagged_wrench_com;
    tagged_wrench_com.index = ninja_navigator_->getMyID();
    tagged_wrench_com.wrench = wrench_msg_com;
    tagged_external_wrench_pub_.publish(tagged_wrench_com);

    prev_est_wrench_timestamp_ = ros::Time::now().toSec();
  }

  void NinjaController::calcInteractionWrench()
  {
    using Wrench = Eigen::Matrix<double, 6, 1>;
    using WrenchTransform = aerial_robot_navigation::WrenchTransform;
    using ModuleTransforms = aerial_robot_navigation::NinjaNavigator::OpenChainWrenchTransforms;

    const std::map<int, bool> assembly_flags = ninja_navigator_->getAssemblyFlags();
    const std::vector<int> ids = ninja_navigator_->getAssemblyIds();
    const std::map<int, Eigen::VectorXd> estimated_wrenches = getEstimatedWrenchSnapshot();
    const std::map<int, Eigen::VectorXd> desired_wrenches = getFfInterWrenchSnapshot();
    const int module_count = static_cast<int>(ids.size());
    const int my_id = ninja_navigator_->getMyID();
    if(module_count < 2) return;

    std::map<int, ModuleTransforms> transforms;
    if(!ninja_navigator_->getOpenChainWrenchTransforms(transforms))
      {
        ROS_ERROR_THROTTLE(1.0, "Failed to build open-chain wrench transforms");
        for(const int id: ids) wrench_comp_list_[id] = Wrench::Zero();
        return;
      }

    /* Common disturbance is represented as a per-module wrench in the common COM frame. */
    Wrench common_disturbance = Wrench::Zero();
    int valid_module_count = 0;
    for(const int id: ids)
      {
        const auto flag = assembly_flags.find(id);
        const auto wrench = estimated_wrenches.find(id);
        if(flag == assembly_flags.end() || !flag->second || wrench == estimated_wrenches.end()) continue;
        common_disturbance += wrench->second;
        ++valid_module_count;
      }
    if(valid_module_count == 0) return;
    common_disturbance /= static_cast<double>(valid_module_count);

    geometry_msgs::WrenchStamped whole_wrench_msg;
    whole_wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    whole_wrench_msg.header.frame_id = ninja_navigator_->getMyName()
      + std::to_string(my_id) + "/center_of_moving";
    whole_wrench_msg.wrench.force.x = common_disturbance(0);
    whole_wrench_msg.wrench.force.y = common_disturbance(1);
    whole_wrench_msg.wrench.force.z = common_disturbance(2);
    whole_wrench_msg.wrench.torque.x = common_disturbance(3);
    whole_wrench_msg.wrench.torque.y = common_disturbance(4);
    whole_wrench_msg.wrench.torque.z = common_disturbance(5);
    whole_external_wrench_pub_.publish(whole_wrench_msg);

    /*
     * Open-chain contact estimation. q_i is the wrench exerted by module i on
     * module i+1, expressed at module i's right dock D_i. Only N-1 physical
     * contacts are unknown; no virtual terminal contact or KKT constraint is used.
     */
    const int contact_count = module_count - 1;
    Eigen::MatrixXd contact_matrix = Eigen::MatrixXd::Zero(6 * module_count,
                                                            6 * contact_count);
    Eigen::VectorXd module_residual = Eigen::VectorXd::Zero(6 * module_count);
    for(int i = 0; i < module_count; ++i)
      {
        const int id = ids.at(i);
        const ModuleTransforms& xs = transforms.at(id);
        const auto estimated = estimated_wrenches.find(id);
        Wrench estimated_base = Wrench::Zero();
        if(estimated != estimated_wrenches.end()) estimated_base = estimated->second;
        module_residual.segment<6>(6 * i) =
          xs.Ci_from_Base * (estimated_base - common_disturbance);

        if(i > 0)
          contact_matrix.block<6, 6>(6 * i, 6 * (i - 1)) = xs.Ci_from_Dim1;
        if(i < contact_count)
          contact_matrix.block<6, 6>(6 * i, 6 * i) = -xs.Ci_from_Di;
      }

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> contact_solver(contact_matrix);
    if(contact_solver.rank() < 6 * contact_count)
      {
        ROS_ERROR_THROTTLE(1.0, "Open-chain contact estimation matrix is rank deficient");
        for(const int id: ids) wrench_comp_list_[id] = Wrench::Zero();
        return;
      }

    const Eigen::VectorXd contact_wrenches = contact_solver.solve(module_residual);
    if(!contact_wrenches.allFinite())
      {
        ROS_ERROR_THROTTLE(1.0, "Open-chain contact estimation produced invalid values");
        for(const int id: ids) wrench_comp_list_[id] = Wrench::Zero();
        return;
      }

    for(const int id: ids) inter_wrench_list_[id] = Wrench::Zero();
    for(int i = 0; i < contact_count; ++i)
      inter_wrench_list_[ids.at(i)] = contact_wrenches.segment<6>(6 * i);

    const double residual_norm = (contact_matrix * contact_wrenches - module_residual).norm();
    const double data_norm = std::max(1.0, module_residual.norm());
    if(residual_norm / data_norm > 0.1)
      ROS_WARN_STREAM_THROTTLE(1.0, "Large open-chain contact estimation residual: "
                               << residual_norm / data_norm);

    const Wrench my_contact = inter_wrench_list_.at(my_id);
    geometry_msgs::WrenchStamped contact_msg;
    contact_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    contact_msg.header.frame_id = ninja_navigator_->getMyName()
      + std::to_string(my_id) + "/yaw_connect_point";
    contact_msg.wrench.force.x = my_contact(0);
    contact_msg.wrench.force.y = my_contact(1);
    contact_msg.wrench.force.z = my_contact(2);
    contact_msg.wrench.torque.x = my_contact(3);
    contact_msg.wrench.torque.y = my_contact(4);
    contact_msg.wrench.torque.z = my_contact(5);
    internal_wrench_pub_.publish(contact_msg);

    /*
     * Open-chain compensation. Anchor the leader command at zero and propagate
     * the N-1 physical contact errors toward both ends of the chain.
     */
    const int leader_id = ninja_navigator_->getLeaderID();
    const auto leader = std::find(ids.begin(), ids.end(), leader_id);
    if(leader == ids.end())
      {
        ROS_ERROR_THROTTLE(1.0, "Leader is not part of the assembled open chain");
        for(const int id: ids) wrench_comp_list_[id] = Wrench::Zero();
        return;
      }
    const int leader_index = static_cast<int>(std::distance(ids.begin(), leader));

    std::vector<Wrench> module_compensation(module_count, Wrench::Zero());
    auto contactErrorInCog = [&](int contact_index) -> Wrench
      {
        const int contact_id = ids.at(contact_index);
        Wrench desired = Wrench::Zero();
        const auto desired_it = desired_wrenches.find(contact_id);
        if(desired_it != desired_wrenches.end()) desired = desired_it->second;
        const Wrench current = inter_wrench_list_.at(contact_id);
        return Wrench(transforms.at(contact_id).Ci_from_Di * (desired - current));
      };

    for(int i = leader_index; i < contact_count; ++i)
      {
        const Wrench delta = contactErrorInCog(i);
        const WrenchTransform& current_from_next = transforms.at(ids.at(i)).Ci_from_Cip1;
        module_compensation.at(i + 1) =
          current_from_next.fullPivLu().solve(module_compensation.at(i) - delta);
      }

    for(int i = leader_index - 1; i >= 0; --i)
      {
        const Wrench delta = contactErrorInCog(i);
        const WrenchTransform& current_from_next = transforms.at(ids.at(i)).Ci_from_Cip1;
        module_compensation.at(i) = delta + current_from_next * module_compensation.at(i + 1);
      }

    for(int i = 0; i < module_count; ++i)
      {
        if(!module_compensation.at(i).allFinite())
          {
            ROS_ERROR_THROTTLE(1.0, "Open-chain compensation produced invalid values");
            for(const int id: ids) wrench_comp_list_[id] = Wrench::Zero();
            return;
          }
        wrench_comp_list_[ids.at(i)] = module_compensation.at(i);
      }
  }

  void NinjaController::pseudoAsmCallback(const std_msgs::BoolConstPtr & msg)
  {
    ninja_navigator_->pseudo_assembly_mode_ = msg->data;
    wrench_comp_p_gain_ = 0;
    wrench_comp_d_gain_ = 0;
    wrench_comp_i_gain_ = 0;
    if(msg->data)
      ROS_WARN_STREAM("Pseudo assembly mode ON!");
    else
      ROS_WARN_STREAM("Pseudo assembly mode OFF!");
  }
  

  void NinjaController::rosParamInit()
  {
    BeetleController::rosParamInit();
    ros::NodeHandle control_nh(nh_, "controller");
    ros::NodeHandle joint_nh(control_nh, "joint_comp");
    getParam<double>(joint_nh, "p_gain", joint_p_gain_, 0.1);
    getParam<double>(joint_nh, "i_gain", joint_i_gain_, 0.005);
    getParam<double>(joint_nh, "d_gain", joint_d_gain_, 0.07);  
  }

  void NinjaController::reset()
  {
    BeetleController::reset();
    pid_controllers_.at(JOINT_TY).reset();
    pid_controllers_.at(JOINT_TZ).reset();
    for(const auto & id : ninja_navigator_->getAssemblyIds())
      {
        Eigen::VectorXd reset_ff_wrench = Eigen::VectorXd::Zero(6);
        setFfInterWrench(id,reset_ff_wrench); 
      }
    joint_control_timestamp_ = -1;
  }
} //namespace aerial_robot_controller

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::NinjaController, aerial_robot_control::ControlBase);
