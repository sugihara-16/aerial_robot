#include <assemble_quadrotors/model/assemble_robot_model.h>

AssembleTiltedRobotModel::AssembleTiltedRobotModel(bool init_with_rosparam,
                           bool verbose,
                           double fc_t_min_thre,
                           double epsilon):
  HydrusTiltedRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon),
  assemble_mode_(false),
  dessemble_mode_(true),
  initial_assemble_(false),
  controller_lock_(false)
{
  // initialize frame mode
  getParamFromRos();
  if(initial_assemble_)
    {
      assemble();
    }
  else
    {
      dessemble();
    }
}

void AssembleTiltedRobotModel::assemble()
{
  controller_lock_ = true;
  //initialize urdf model
  urdf::Model empty_model;
  aerial_robot_model::RobotModel::setUrdfModel(empty_model);
  // switch urdf mode
  initializeRotorNum();
  kinematicsInit("assemble_robot");
  stabilityInit();
  staticsInit("assemble_robot");
  ROS_INFO("Switched to assemble model");
  // int rotor_num = aerial_robot_model::RobotModel::getRotorNum();
  // ROS_INFO("Rotor num is %d",rotor_num);
  initializeRviz("assemble_robot");
  aerial_robot_model::RobotModel::updateRobotModel(); // update robot model instantly

  // switch the model in the end
  assemble_mode_ = true;
  dessemble_mode_ = false;

  controller_lock_ = false;
}

void AssembleTiltedRobotModel::dessemble()
{
  controller_lock_ = true;
  //initialize urdf model
  urdf::Model empty_model;
  aerial_robot_model::RobotModel::setUrdfModel(empty_model);
  // switch urdf mode
  initializeRotorNum();
  kinematicsInit("dessemble_robot");
  stabilityInit();
  staticsInit("dessemble_robot");
  ROS_INFO("Switched to dessemble model");
  // int rotor_num = aerial_robot_model::RobotModel::getRotorNum();
  // ROS_INFO("Rotor num is %d",rotor_num);
  initializeRviz("dessemble_robot");
  HydrusTiltedRobotModel::updateRobotModel(); // update robot model instantly

  // switch the mode in the end
  assemble_mode_ = false;
  dessemble_mode_ = true;
  controller_lock_ = false;
}

void AssembleTiltedRobotModel::getParamFromRos()
{
  ros::NodeHandle nh;
  nh.param("initial_assemble", initial_assemble_, false);
}

void AssembleTiltedRobotModel::initializeRviz(const std::string param){
  ros::NodeHandle nh;
  std::string xml_string;
  std::string robot_description = "robot_description";
  nh.getParam(param,xml_string);
  nh.setParam(robot_description.c_str(),xml_string.c_str());
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(AssembleTiltedRobotModel, aerial_robot_model::RobotModel);
