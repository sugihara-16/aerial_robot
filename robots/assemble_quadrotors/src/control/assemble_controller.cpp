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

  nh.param("airframe", airframe_, std::string("male"));
  nh.param("initial_assemble", current_assemble_, false);
  //initialize paramaters
  assemble_nh_ = ros::NodeHandle(nh_, "assemble");
  dessemble_nh_ = ros::NodeHandle(nh_, airframe_);

  ros::NodeHandle assemble_control_nh = ros::NodeHandle(nh_, "assemble/controller");
  assemble_control_nh.param("torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.1);
  assemble_control_nh.param("wrench_allocation_matrix_pub_interval", wrench_allocation_matrix_pub_interval_, 0.1);

  assemble_mode_controller_ = boost::make_shared<FullyActuatedController>();
  dessemble_mode_controller_ = boost::make_shared<HydrusTiltedLQIController>();
  current_assemble_ = assemble_robot_model_->isAssemble();

  assemble_robot_model_->assemble(); //switching robot model
  assemble_mode_controller_->initialize(assemble_nh_, nhp, assemble_robot_model_, estimator, navigator, ctrl_loop_rate);
  assemble_robot_model_->dessemble(); //switching robot model
  dessemble_mode_controller_->initialize(dessemble_nh_, nhp, assemble_robot_model_, estimator, navigator, ctrl_loop_rate);

  //adjust robot model for true state
  if(current_assemble_)
    {
      assemble_robot_model_->assemble();
    }
  else
    {
      assemble_robot_model_->dessemble();
    }

}

//override
bool AssembleController::update(){
  if(assemble_robot_model_->isAssemble()){
    if(!assemble_mode_controller_->ControlBase::update()) return false;
    assemble_mode_controller_->controlCore();
  }else{
    if(!dessemble_mode_controller_->ControlBase::update()) return false;
    dessemble_mode_controller_->controlCore();
  }
  sendCmd();
}

void AssembleController::sendCmd(){
  if(assemble_robot_model_->isAssemble()){
    assemble_mode_controller_->PoseLinearController::sendCmd();
    spinal::FourAxisCommand flight_command_data;
    flight_command_data.angles[0] = navigator_->getTargetRPY().x();
    flight_command_data.angles[1] = navigator_->getTargetRPY().y();
    flight_command_data.angles[2] = assemble_mode_controller_->getCandidateYawTerm() ;
    // choose correct 4 elements
    auto all_target_base_thrust = assemble_mode_controller_->getTargetBaseThrust(); // 8 elements
    if(airframe_ == "male") {
      std::vector<float> target_base_thrust {all_target_base_thrust.begin(), all_target_base_thrust.begin()+4};
      flight_command_data.base_thrust = target_base_thrust;
    }else{
      std::vector<float> target_base_thrust {all_target_base_thrust.begin() + 5, all_target_base_thrust.begin()+8};
      flight_command_data.base_thrust = target_base_thrust;
    }
    flight_cmd_pub_.publish(flight_command_data);
    // send truncated P matrix
    // copy from  sendTorqueAllocationMatrixInv(); in fully_actuated_controller.cpp
    if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
      {
        torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

        spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
        torque_allocation_matrix_inv_msg.rows.resize(4); //resize row of torque_allocation_matrix_inv to 4 for quadrotor
        Eigen::MatrixXd torque_allocation_matrix_inv = assemble_mode_controller_->getQMatInv().rightCols(3);
        if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
          ROS_ERROR("Torque Allocation Matrix overflow");

        int offset = 0;
        if (airframe_ == "female") offset = 4;

        for (unsigned int i = 0; i < 4; i++)
          {
            torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i + offset ,0) * 1000;
            torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i + offset ,1) * 1000;
            torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i + offset ,2) * 1000;
          }
        torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
      }
  }

  else {
    dessemble_mode_controller_->PoseLinearController::sendCmd();
    spinal::FourAxisCommand flight_command_data;
    flight_command_data.angles[0] = dessemble_mode_controller_->getTargetRoll();
    flight_command_data.angles[1] = dessemble_mode_controller_->getTargetPitch();
    flight_command_data.angles[2] = dessemble_mode_controller_->getCandidateYawTerm();
    flight_command_data.base_thrust = dessemble_mode_controller_->getTargetBaseThrust();
    flight_cmd_pub_.publish(flight_command_data);
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::AssembleController, aerial_robot_control::ControlBase);
