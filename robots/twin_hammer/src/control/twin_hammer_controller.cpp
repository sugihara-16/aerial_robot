#include <twin_hammer/control/twin_hammer_controller.h>

using namespace aerial_robot_control;

TwinHammerController::TwinHammerController():
  GimbalrotorController()
{
}

void TwinHammerController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                  double ctrl_loop_rate
                                  )
{
  GimbalrotorController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  rosParamInit();
}

void TwinHammerController::controlCore()
{
  GimbalrotorController::controlCore();
}

void TwinHammerController::reset()
{
  GimbalrotorController::reset();
}

void TwinHammerController::rosParamInit()
{
  GimbalrotorController::rosParamInit();
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::TwinHammerController, aerial_robot_control::ControlBase);
