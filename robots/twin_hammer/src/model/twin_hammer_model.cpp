#include <twin_hammer/model/twin_hammer_model.h>

TwinHammerModel::TwinHammerModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  GimbalrotorRobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon),
  tfBuffer_(),
  tfListener_(tfBuffer_)
{
}

void TwinHammerModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  // GimbalrotorRobotModel::updateRobotModelImpl(joint_positions);
  // RobotModel::updateRobotModelImpl(joint_positions);
  // const auto seg_tf_map = getSegmentsTf();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(TwinHammerModel, aerial_robot_model::RobotModel); 