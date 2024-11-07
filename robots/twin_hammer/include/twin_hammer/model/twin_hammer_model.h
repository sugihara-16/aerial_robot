// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/model/gimbalrotor_robot_model.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
using namespace aerial_robot_model;

class TwinHammerModel : public GimbalrotorRobotModel{
public:
  TwinHammerModel(bool init_with_rosparam = true,
                    bool verbose = false,
                    double fc_t_min_thre = 0,
                    double epsilon = 10);
  virtual ~TwinHammerModel() = default;

private:
  ros::NodeHandle nh_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformBroadcaster br_;

protected:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
};

