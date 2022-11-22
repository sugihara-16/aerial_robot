#include <assemble_quadrotors/control/assemble_controller.h>

using namespace aerial_robot_control;

void AssembleController::initialize(ros::NodeHandle nh,
                                    ros::NodeHandle nhp,
                                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                    double ctrl_loop_rate)
{
  assemble_robot_model_ = boost::dynamic_pointer_cast<AssembleTiltedRobotModel>(robot_model);
  navigator_ = navigator;

  nh.param("airframe", airframe_, std::string("male"));
  nh.param("initial_assemble", current_assemble_, false);
  //initialize paramaters
  assemble_nh_ = ros::NodeHandle(nh_, "assemble");
  dessemble_nh_ = ros::NodeHandle(nh_, airframe_);

  desired_baselink_rot_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);

  ros::NodeHandle assemble_control_nh = ros::NodeHandle(nh_, "assemble/controller");
  assemble_control_nh.param("torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.1);
  assemble_control_nh.param("wrench_allocation_matrix_pub_interval", wrench_allocation_matrix_pub_interval_, 0.1);

  assemble_mode_controller_ = boost::make_shared<FullyActuatedController>();
  dessemble_mode_controller_ = boost::make_shared<HydrusTiltedLQIController>();
  current_assemble_ = assemble_robot_model_->isAssemble();

  assemble_robot_model_->assemble(); //switching robot model
  assemble_mode_controller_->initialize(assemble_nh_, nhp, assemble_robot_model_, estimator, navigator_, ctrl_loop_rate);
  assemble_robot_model_->dessemble(); //switching robot model
  dessemble_mode_controller_->initialize(dessemble_nh_, nhp, assemble_robot_model_, estimator, navigator_, ctrl_loop_rate);
  dessemble_mode_controller_->optimalGain(); // calculate LQI gain for once


  //adjust robot model for true state
  if(current_assemble_)
    {
      assemble_robot_model_->assemble();
    }
  else
    {
      assemble_robot_model_->dessemble();
    }

  send_once_flag_ = true;
}

//override
bool AssembleController::update(){
  if(assemble_robot_model_->isAssemble()){
    if(!current_assemble_) {
      send_once_flag_ = true;
      current_assemble_ = true;
      // set new target pos in current mode
      navigator_->setTargetXyFromCurrentState();
      navigator_->setTargetYawFromCurrentState();
      // set current errI for gravity compensation
      double current_ErrI = dessemble_mode_controller_->getCurrentZErrI();
      ROS_INFO("now z iterm is %f",current_ErrI);
      assemble_mode_controller_->setCurrentZErrI(current_ErrI);
    }
    if(!assemble_mode_controller_->ControlBase::update()) return false;
    if(!current_assemble_) {
      send_once_flag_ = true;
      current_assemble_ = true;
      // set new target pos in current mode
      navigator_->setTargetXyFromCurrentState();
      navigator_->setTargetYawFromCurrentState();
      // set current errI and ITerm for gravity compensation
      double current_ITerm = dessemble_mode_controller_->getCurrentZITerm();
      assemble_mode_controller_->setCurrentZITerm(current_ITerm);
      double current_ErrZ = dessemble_mode_controller_->getCurrentZErrI();
      assemble_mode_controller_->setCurrentZErrI(current_ErrZ);
    }
    assemble_mode_controller_->controlCore();
  }else{
    if(!dessemble_mode_controller_->ControlBase::update()) return false;
    if(current_assemble_) {
      send_once_flag_ = true;
      current_assemble_ = false;
      // set new target pos in current mode
      navigator_->setTargetXyFromCurrentState();
      navigator_->setTargetYawFromCurrentState();
      // set current errI for gravity compensation
      double current_ITerm = assemble_mode_controller_->getCurrentZITerm();
      dessemble_mode_controller_->setCurrentZITerm(current_ITerm);
      double current_ErrZ = assemble_mode_controller_->getCurrentZErrI();
      dessemble_mode_controller_->setCurrentZErrI(current_ErrZ);
    }
    dessemble_mode_controller_->controlCore();
  }
  sendCmd();
}

void AssembleController::sendCmd(){
  if(assemble_robot_model_->isAssemble()){
    assemble_mode_controller_->PoseLinearController::sendCmd();
    spinal::FourAxisCommand flight_command_data;
    flight_command_data.angles[0] = navigator_->getTargetRPY().x(); //probrem
    flight_command_data.angles[1] = navigator_->getTargetRPY().y();
    flight_command_data.angles[2] = assemble_mode_controller_->getCandidateYawTerm() ;
    // choose correct 4 elements
    auto all_target_base_thrust = assemble_mode_controller_->getTargetBaseThrust(); // 8 elements
    // if(airframe_ == "male") {
    //   std::vector<float> target_base_thrust {all_target_base_thrust.begin(), all_target_base_thrust.begin()+4};
    //   flight_command_data.base_thrust = target_base_thrust;
    // }else{
    //   std::vector<float> target_base_thrust {all_target_base_thrust.begin() + 4, all_target_base_thrust.begin()+8};
    //   flight_command_data.base_thrust = target_base_thrust;
    // }
    // flight_cmd_pub_.publish(flight_command_data);
    flight_command_data.base_thrust = all_target_base_thrust;
    flight_cmd_pub_.publish(flight_command_data);

    // send command for once
    // if (!send_once_flag_) return;

    // send truncated P matrix
    // copy from  sendTorqueAllocationMatrixInv(); in fully_actuated_controller.cpp
    spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
    // torque_allocation_matrix_inv_msg.rows.resize(4); //resize row of torque_allocation_matrix_inv to 4 for quadrotor
    torque_allocation_matrix_inv_msg.rows.resize(8);
    Eigen::MatrixXd torque_allocation_matrix_inv = assemble_mode_controller_->getQMatInv().rightCols(3);
    if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
      ROS_ERROR("Torque Allocation Matrix overflow");

    int offset = 0;
    // if (airframe_ == "female") offset = 4;

    for (unsigned int i = 0; i < 8; i++)
      {
        torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i + offset ,0) * 1000;
        torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i + offset ,1) * 1000;
        torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i + offset ,2) * 1000;
      }
    torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);

    // send horizontal CoG frame after switching from dessemble mode
    spinal::DesireCoord coord_msg;
    coord_msg.roll = 0;
    coord_msg.pitch = 0;
    desired_baselink_rot_pub_.publish(coord_msg);

    send_once_flag_ = false;
  }

  else {
    dessemble_mode_controller_->PoseLinearController::sendCmd();
    spinal::FourAxisCommand flight_command_data;
    flight_command_data.angles[0] = dessemble_mode_controller_->getTargetRoll();
    flight_command_data.angles[1] = dessemble_mode_controller_->getTargetPitch();
    flight_command_data.angles[2] = dessemble_mode_controller_->getCandidateYawTerm();
    flight_command_data.base_thrust = dessemble_mode_controller_->getTargetBaseThrust();
    flight_cmd_pub_.publish(flight_command_data);

    // send command for once
    // if (!send_once_flag_) return;

    // send LQI gain
    dessemble_mode_controller_->publishGain();

    // send tilted CoG frame after switching from dessemble mode
    double roll,pitch, yaw;
    assemble_robot_model_->getCogDesireOrientation<KDL::Rotation>().GetRPY(roll, pitch, yaw);

    spinal::DesireCoord coord_msg;
    coord_msg.roll = roll;
    coord_msg.pitch = pitch;
    desired_baselink_rot_pub_.publish(coord_msg);

    send_once_flag_ = false;
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::AssembleController, aerial_robot_control::ControlBase);
