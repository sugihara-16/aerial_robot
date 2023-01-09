#include <assemble_quadrotors/navigation/assemble_flight_navigation.h>

namespace aerial_robot_navigation
{
  void AssembleNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator)
  {
    /* initialize the flight control */
    BaseNavigator::initialize(nh, nhp, robot_model, estimator);

    switching_flag_sub_ = nh_.subscribe("switching_flag", 1, &AssembleNavigator::switchingAircraftCallback, this);

    assemble_robot_model_ = boost::dynamic_pointer_cast<AssembleTiltedRobotModel>(robot_model);

    nh.param("airframe", airframe_, std::string("male"));

  }


  void AssembleNavigator::switchingAircraftCallback(const std_msgs::Empty& msg)
  {
    if(assemble_robot_model_->isAssemble())
      {
        ROS_INFO("Switched to dessemble navigation");
        assemble_robot_model_->dessemble();
      }
    else
      {
        ROS_INFO("Switched to assemble navigation");
        assemble_robot_model_->assemble();
      }
  }

  void AssembleNavigator::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
  {
    BaseNavigator::joyStickControl(joy_msg);
    if( assemble_robot_model_->isAssemble() && BaseNavigator::getXyControlMode() == VEL_CONTROL_MODE){
      // only navigate with local frame when assemble
      if(BaseNavigator::getControlframe() == WORLD_FRAME){
        BaseNavigator::setTargetVelX(0);
        BaseNavigator::setTargetVelY(0);
        BaseNavigator::setTargetVelZ(0);
      }
      //In assemble state, target velocity should be inveted if the airframe is female.
      if(airframe_ == "female"){
        tf::Vector3 target_vel = BaseNavigator::getTargetVel();
        target_vel = target_vel * (-1);
        BaseNavigator::setTargetVelX(target_vel.x());
        BaseNavigator::setTargetVelY(target_vel.y());
        BaseNavigator::setTargetVelZ(target_vel.z());
      }
    }

  }
}
/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::AssembleNavigator, aerial_robot_navigation::BaseNavigator);
