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
};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::AssembleNavigator, aerial_robot_navigation::BaseNavigator);
