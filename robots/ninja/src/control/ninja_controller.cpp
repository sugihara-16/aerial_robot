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
    closed_loop_flag_sub_ = nh_.subscribe("/closed_loop_flag",1,&NinjaController::closedLoopCallback, this);
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

    //convert extimated external wrench from cog to com coordinates
    const Eigen::Matrix<double, 6, 1> W_cog = est_external_wrench_cog;
    const Eigen::Matrix<double, 6, 6> Xstar_com_from_cog = ninja_navigator_->getCog2ComWrenchXStar();
    const Eigen::Matrix<double, 6, 1> W_com = Xstar_com_from_cog * W_cog;
    Eigen::VectorXd est_external_wrench_com(6);
    est_external_wrench_com = W_com;
    
    geometry_msgs::WrenchStamped wrench_msg_com;
    wrench_msg_com.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    wrench_msg_com.header.frame_id = my_name + "/cog";
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
    // ===== 0) Basic Data =====
    const auto assembly_flag = ninja_navigator_->getAssemblyFlags();
    std::vector<int> ids     = ninja_navigator_->getAssemblyIds();  // C1..CN
    const int N = static_cast<int>(ids.size());
    const int my_id = ninja_navigator_->getMyID();
    const bool is_closed = ninja_navigator_->closed_loop_flag_;
    if (N == 0) return;
    
    const int  leader_id = ninja_navigator_->getLeaderID();
    const auto xsmap     = ninja_navigator_->getContactXstarsSnapshot(); 
    // xs.Phi_Ci_Di   = ^Ci X*_{Di}
    // xs.Psi_Ci_Dip1 = ^Ci X*_{D_{i+1}}
    // xs.Ci_from_Base= ^Ci X*_{Base} (Base=com)
    
    // ===== 1) Estimation for disturbance wrench =====
    Eigen::Matrix<double,6,1> Wdist_base = Eigen::Matrix<double,6,1>::Zero();
    int cnt = 0;
    for (const auto& kv : est_wrench_list_) {
      const int id = kv.first;
      if (!assembly_flag.count(id) || !assembly_flag.at(id)) continue;
      Wdist_base += kv.second;  // ^Base
      ++cnt;
    }
    if (cnt > 0) Wdist_base /= static_cast<double>(cnt);
    
    // publish: whole external wrench
    {
      geometry_msgs::WrenchStamped ws;
      ws.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
      ws.header.frame_id = ninja_navigator_->getMyName() + "/center_of_moving";
      ws.wrench.force.x  = Wdist_base(0);
      ws.wrench.force.y  = Wdist_base(1);
      ws.wrench.force.z  = Wdist_base(2);
      ws.wrench.torque.x = Wdist_base(3);
      ws.wrench.torque.y = Wdist_base(4);
      ws.wrench.torque.z = Wdist_base(5);
      whole_external_wrench_pub_.publish(ws);
    }
    
    // ===== 2) Build Estimateion System (Aw=d) =====
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6*N, 6*N);
    Eigen::VectorXd d = Eigen::VectorXd::Zero(6*N);
    
    for (int i = 0; i < N; ++i) {
      const int id_i = ids[i];
      const auto& xs = xsmap.at(id_i);
      
      // d_i = ^Ci Wext_i - (^Ci X*_{Base}) ^Base Wdist
      Eigen::Matrix<double,6,1> Wext_Ci = Eigen::Matrix<double,6,1>::Zero();
      if (est_wrench_list_.count(id_i)) {
        Wext_Ci = xs.Ci_from_Base * est_wrench_list_.at(id_i);
      }
      const Eigen::Matrix<double,6,1> Wdist_Ci = xs.Ci_from_Base * Wdist_base;
      d.segment<6>(6*i) = Wext_Ci - Wdist_Ci;
      
      // A
      A.block<6,6>(6*i, 6*i) = xs.Phi_Ci_Di;
      if (i >= 1) {
        A.block<6,6>(6*i, 6*(i-1)) -= xs.Xi_Ci_Dim1;
      } else if (is_closed) {
        A.block<6,6>(6*i, 6*(N-1)) -= xs.Xi_Ci_Dim1;
      }
    }
    
    // solve (KKT or smoothing)
    Eigen::VectorXd w_ctc = Eigen::VectorXd::Zero(6*N);
    if (!is_closed) {
      Eigen::MatrixXd E = Eigen::MatrixXd::Zero(6, 6*N);
      E.block<6,6>(0, 6*(N-1)) = Eigen::Matrix<double,6,6>::Identity();
      const int n = 6*N, p = 6;
      Eigen::MatrixXd KKT = Eigen::MatrixXd::Zero(n+p, n+p);
      KKT.block(0,0,n,n) = A.transpose()*A;
      KKT.block(0,n,n,p) = E.transpose();
      KKT.block(n,0,p,n) = E;
      Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n+p);
      rhs.segment(0,n) = A.transpose() * d;
      Eigen::VectorXd sol = KKT.fullPivLu().solve(rhs);
      w_ctc = sol.segment(0,n);
    } else {
      const int n = 6*N;
      if (!prev_ctc_valid_ || prev_ctc_wrench_.size() != n) {
        prev_ctc_wrench_ = Eigen::VectorXd::Zero(n);
        prev_ctc_valid_  = true;
      }
      Eigen::MatrixXd LHS = A.transpose()*A + rho_ctc_ * Eigen::MatrixXd::Identity(n,n);
      Eigen::VectorXd RHS = A.transpose()*d + rho_ctc_ * prev_ctc_wrench_;
      w_ctc = LHS.ldlt().solve(RHS);
      prev_ctc_wrench_ = w_ctc;
    }
    
    for (int i = 0; i < N; ++i) {
      const int id = ids[i];
      // make it ACTION-side: wrench that module i EXERTS ON (i+1) at frame Di
      inter_wrench_list_[id] = - w_ctc.segment<6>(6*i); // ^Di, now i→i+1
    }
    
    // publish my module
    {
      const Eigen::Matrix<double,6,1>& Wctc_Di = inter_wrench_list_[my_id];
      geometry_msgs::WrenchStamped m;
      m.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
      m.header.frame_id = ninja_navigator_->getMyName() + std::to_string(my_id) + "/yaw_connect_point";
      m.wrench.force.x = Wctc_Di(0);
      m.wrench.force.y = Wctc_Di(1);
      m.wrench.force.z = Wctc_Di(2);
      m.wrench.torque.x = Wctc_Di(3);
      m.wrench.torque.y = Wctc_Di(4);
      m.wrench.torque.z = Wctc_Di(5);
      internal_wrench_pub_.publish(m);
    }
    
    // ===== 3) Build Compensation System(Hw=Δ) =====
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6*N, 6*N);
    Eigen::VectorXd Delta = Eigen::VectorXd::Zero(6*N);
    
    for (int i = 0; i < N; ++i) {
      const int id_i = ids[i];
      const auto& xs = xsmap.at(id_i);
      
      // Δ_i = des_Di - cur_Di
      Eigen::Matrix<double,6,1> des_Di = Eigen::Matrix<double,6,1>::Zero();
      if (ff_inter_wrench_list_.count(id_i)) {
        des_Di = ff_inter_wrench_list_.at(id_i); // already ^Di
      }
      const Eigen::Matrix<double,6,1> cur_Di = inter_wrench_list_[id_i]; // ^Di
      // Delta.segment<6>(6*i) = des_Di - cur_Di;

      // Δ_i (in ^Ci) = Φ_i * (des_Di - cur_Di)
      const Eigen::Matrix<double,6,1> delta_Di = des_Di - cur_Di;      // ^Di
      const Eigen::Matrix<double,6,1> delta_Ci = xs.Phi_Ci_Di * delta_Di; // ^Ci
      Delta.segment<6>(6*i) = delta_Ci;

      // H: +Φ_i w_i − Ψ_i w_{i+1}
      H.block<6,6>(6*i, 6*i) = xs.Phi_Ci_Di;
      if (i < N-1) {
        H.block<6,6>(6*i, 6*(i+1)) -= xs.Psi_Ci_Dip1;
      } else if (is_closed) {
        H.block<6,6>(6*i, 0) -= xs.Psi_Ci_Dip1; // wrap
      }
    }
    
    // Constraint: Ew=0 (Open and Close)
    Eigen::MatrixXd E = Eigen::MatrixXd::Zero(6, 6*N);
    const int leader_idx = static_cast<int>(
                                            std::distance(ids.begin(), std::find(ids.begin(), ids.end(), leader_id)));
    E.block<6,6>(0, 6*leader_idx) = Eigen::Matrix<double,6,6>::Identity();
    
    // KKT solve
    {
      const int n = 6*N, p = 6;
      Eigen::MatrixXd KKT = Eigen::MatrixXd::Zero(n+p, n+p);
      KKT.block(0,0,n,n) = H.transpose()*H;
      KKT.block(0,n,n,p) = E.transpose();
      KKT.block(n,0,p,n) = E;
      Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n+p);
      rhs.segment(0,n) = H.transpose() * Delta;
      
      Eigen::VectorXd sol = KKT.fullPivLu().solve(rhs);
      Eigen::VectorXd wff = sol.segment(0,n); // ^Di
      
      for (int i = 0; i < N; ++i) {
        const int id = ids[i];
        const auto& xs = xsmap.at(id);
        const Eigen::Matrix<double,6,1> wff_Di = wff.segment<6>(6*i);
        const Eigen::Matrix<double,6,1> wff_Ci = xs.Phi_Ci_Di * wff_Di; // ^Ci
        wrench_comp_list_[id] = wff_Ci;
      }
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

  void NinjaController::closedLoopCallback(const std_msgs::BoolConstPtr & msg)
  {
    ninja_navigator_->closed_loop_flag_ = msg->data;
    if(msg->data)
      ROS_WARN_STREAM("Closed loop mode ON");
    else
      ROS_WARN_STREAM("Closed loop mode OFF!");
  }
  

  void NinjaController::rosParamInit()
  {
    BeetleController::rosParamInit();
    ros::NodeHandle control_nh(nh_, "controller");
    ros::NodeHandle joint_nh(control_nh, "joint_comp");
    getParam<double>(joint_nh, "p_gain", joint_p_gain_, 0.1);
    getParam<double>(joint_nh, "i_gain", joint_i_gain_, 0.005);
    getParam<double>(joint_nh, "d_gain", joint_d_gain_, 0.07);

    getParam<double>(control_nh, "smooth_rho", rho_ctc_, 0.1);
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
