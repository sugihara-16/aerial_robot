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

  airframe_ = nh.getParam("airframe", airframe_);

  //initialize controller according to robot state
  if(assemble_robot_model_->isAssemble()){
    current_assemble_ = true;
    //adjust NodeHandle according to robot state
    ros::NodeHandle temp_nh(nh, "assemble");
    nh = temp_nh;
    FullyActuatedController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
    ROS_INFO("assemble controller is activated!");
  }else{
    current_assemble_ = false;
    //adjust NodeHandle according to robot state
    ros::NodeHandle temp_nh(nh, airframe_);
    nh = temp_nh;

    HydrusTiltedLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

    ROS_INFO("%s controller is activated", airframe_.c_str());
  }
  nh_ = nh;
  nhp_ = nhp;
  estimator_  = estimator;
  robot_model_ = robot_model;
  navigator_ = navigator;
  ctrl_loop_du_ = ctrl_loop_rate;
}

//override

void AssembleController::controlCore(){
  if(assemble_robot_model_->isAssemble()){
    FullyActuatedController::controlCore();
  }else{
    HydrusTiltedLQIController::controlCore();
  }
}

void AssembleController::sendCmd(){
  if(assemble_robot_model_->isAssemble()){
    FullyActuatedController::sendCmd();
  }else{
    HydrusTiltedLQIController::sendCmd();
  }
}

bool AssembleController::update(){
  if(assemble_robot_model_->isAssemble()){
    if(!current_assemble_)
      {
        current_assemble_ = true;
        //change controller
        FullyActuatedController::initialize(nh_, nhp_, robot_model_, estimator_, navigator_, ctrl_loop_du_);
      }
    return FullyActuatedController::update();
  }else{
    if(current_assemble_)
      {
        current_assemble_ = false;
        //change controller
        HydrusTiltedLQIController::initialize(nh_, nhp_, robot_model_, estimator_, navigator_, ctrl_loop_du_);
      }
    return HydrusTiltedLQIController::update();
  }
}


void AssembleController::rosParamInit(){
  if(assemble_robot_model_->isAssemble()){
    FullyActuatedController::rosParamInit();
  }else{
    HydrusTiltedLQIController::rosParamInit();
  }
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::AssembleController, aerial_robot_control::ControlBase);
