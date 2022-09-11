// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <assemble_quadrotors/sensor/mocap.h>

using namespace Eigen;
using namespace std;


// override
void AssembleMocap::initialize(ros::NodeHandle nh,
                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                string sensor_name, int index)
{
  Mocap::initialize(nh, robot_model, estimator, sensor_name, index);

  std::string assemble_topic_name; // topic: /assemble/mocap/pose
  getParam<std::string>("assemble_mocap_sub_name", assemble_topic_name, std::string("pose"));
  assemble_mocap_sub_ = nh_.subscribe(assemble_topic_name, 1, &Mocap::assemblePoseCallback, this);

  std::string unit_topic_name; // topic: /assemble_quadrotors${id}/mocap/pose
  getParam<std::string>("unit_mocap_sub_name", unit_topic_name, std::string("pose"));
  unit_mocap_sub_ = nh_.subscribe(unit_topic_name, 1, &Mocap::unitPoseCallback, this);

  assemble_robot_model_ = boost::dynamic_pointer_cast<aerial_robot_model::AssembleRoboModel>(robot_model_);
}


void assemblePoseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{
  bool assemble_flag = assemble_robot_model_->isAssemble();

  if (!assemble_flag) return;

  Mocap::poseCallback(msg);
}

void unitPoseCallback(const geometry_msgs::PoseStampedConstPtr & msg)
{
  bool dessemble_flag = assemble_robot_model_->isDessemble();

  if (assmeble_flag_) return;

  Mocap::poseCallback(msg);
}




/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Mocap, sensor_plugin::SensorBase);













