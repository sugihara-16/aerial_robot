#include <assemble_quadrotors/model/assemble_robot_model.h>

AssembleTiltedRobotModel::AssembleTiltedRobotModel(bool init_with_rosparam,
                           bool verbose,
                           double fc_t_min_thre,
                           double epsilon):
  HydrusTiltedRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon),
  assemble_mode_(false),
  dessemble_mode_(true),
  initial_assemble_("dessemble")
{
  // initialize frame mode
  getParamFromRos();
  if(initial_assemble_)
    {
      ROS_INFO("assemble mode");
      assemble_mode_ = true;
      dessemble_mode_ = false;
    }
  else
    {
      ROS_INFO("dessemble mode");
      assemble_mode_ = false;
      dessemble_mode_ = true;
    }
}



void AssembleTiltedRobotModel::assemble()
{
  assemble_mode_ = true;
  dessemble_mode_ = false;

  // switch urdf mode
  initializeRotorNum();
  kinematicsInit("assemble_robot");
  staticsInit("assemble_robot");
  ROS_INFO("switched to assemble model");
  aerial_robot_model::RobotModel::updateRobotModel(); // update robot model instantly

}

void AssembleTiltedRobotModel::dessemble()
{
  assemble_mode_ = false;
  dessemble_mode_ = true;

  // switch urdf mode
  initializeRotorNum();
  kinematicsInit("dessemble_robot");
  staticsInit("dessemble_robot");

  HydrusTiltedRobotModel::updateRobotModel(); // update robot model instantly
  ROS_INFO("switched to dessemble model");
}

void AssembleTiltedRobotModel::getParamFromRos()
{
  ros::NodeHandle nh;
  nh.param("initial_assemble", initial_assemble_, false);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(AssembleTiltedRobotModel, aerial_robot_model::RobotModel);
