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

#pragma once

#include <aerial_robot_estimation/sensor/base_plugin.h>
#include <geometry_msgs/PoseStamped.h>
#include <kalman_filter/kf_pos_vel_acc_plugin.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

using namespace Eigen;
using namespace std;

namespace sensor_plugin
{

  class Mocap :public sensor_plugin::SensorBase
    {
    public:
      void initialize(ros::NodeHandle nh,
                      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                      string sensor_name, int index) override;

      ~Mocap() {}
      Mocap();

      static constexpr int TIME_SYNC_CALIB_COUNT = 10;

    private:
      /* ros */
      ros::Subscriber mocap_sub_, ground_truth_sub_;

      /* ros param */
      double sample_freq_;
      double cutoff_pos_freq_;
      double cutoff_vel_freq_;

      double pos_noise_sigma_, angle_noise_sigma_, acc_bias_noise_sigma_;

      IirFilter lpf_pos_; /* x, y, z */
      IirFilter lpf_vel_; /* x, y, z */
      IirFilter lpf_angular_; /* yaw angular velocity */

      tf::Vector3 raw_pos_, raw_vel_;
      tf::Vector3 pos_, vel_;

      tf::Vector3 prev_raw_pos_, prev_raw_vel_;
      tf::Quaternion prev_raw_q_;

      bool receive_groundtruth_odom_;

      /* ros msg */
      aerial_robot_msgs::States ground_truth_pose_;

      void poseCallback(const geometry_msgs::PoseStampedConstPtr & msg);
      void setGroundTruthPosVel(tf::Vector3 baselink_pos, tf::Vector3 baselink_vel);
      void groundTruthCallback(const nav_msgs::OdometryConstPtr & msg);
      void rosParamInit();
      void init(tf::Vector3 init_pos);
      void estimateProcess(ros::Time stamp);
    };
};





