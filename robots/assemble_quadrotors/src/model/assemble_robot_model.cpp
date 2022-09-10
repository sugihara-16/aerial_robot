#include <assemble_quadrotors/model/assemble_robot_model.h>

AssembleTiltedRobotModel::AssembleTiltedRobotModel(bool init_with_rosparam,
                           bool verbose,
                           double fc_t_min_thre,
                           double epsilon):
  HydrusTiltedRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon),
  assemble_mode_(false),
  dessemble_mode_(true)
{
  kinematicsInit();
  staticsInit();
  stabilityInit();
}



void AssembleTiltedRobotModel::assemble()
{
  assemble_mode_ = true;
  dessemble_mode_ = false;

  // switch urdf mode
    kinematicsInit("assemble_robot_model");
    staticsInit("assemble_robot_model");

    aerial_robot_model::RobotModel::updateRobotModel(); // update robot model instantly
}

void AssembleTiltedRobotModel::dessemble()
{
  assemble_mode_ = false;
  dessemble_mode_ = true;

  // switch urdf mode
    kinematicsInit("dessemble_model");
    staticsInit("dessemble_robot_model");

    HydrusTiltedRobotModel::updateRobotModel(); // update robot model instantly
}
