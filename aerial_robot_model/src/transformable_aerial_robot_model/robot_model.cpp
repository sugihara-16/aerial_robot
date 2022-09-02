#include <aerial_robot_model/transformable_aerial_robot_model.h>

namespace aerial_robot_model {

  namespace transformable {

    RobotModel::RobotModel(bool init_with_rosparam, bool verbose, double fc_f_min_thre, double fc_t_min_thre, double epsilon):
      aerial_robot_model::RobotModel(init_with_rosparam, verbose, fc_f_min_thre, fc_t_min_thre, epsilon),
      joint_num_(0)
    {
    }

    KDL::JntArray RobotModel::jointMsgToKdl(const sensor_msgs::JointState& state) const
    {
      KDL::JntArray joint_positions(tree_.getNrOfJoints());
      for(unsigned int i = 0; i < state.position.size(); ++i)
        {
          auto itr = joint_index_map_.find(state.name[i]);
          if(itr != joint_index_map_.end()) joint_positions(itr->second) = state.position[i];
        }
      return joint_positions;
    }

    sensor_msgs::JointState RobotModel::kdlJointToMsg(const KDL::JntArray& joint_positions) const
    {
      sensor_msgs::JointState state;
      state.name.reserve(joint_index_map_.size());
      state.position.reserve(joint_index_map_.size());
      for(const auto& actuator : joint_index_map_)
        {
          state.name.push_back(actuator.first);
          state.position.push_back(joint_positions(actuator.second));
        }
      return state;
    }

    void RobotModel::updateJacobians()
    {
      updateJacobians(getJointPositions(), false);
    }

    void RobotModel::updateJacobians(const KDL::JntArray& joint_positions, bool update_model)
    {
      if(update_model) updateRobotModel(joint_positions);

      calcCoGMomentumJacobian(); // should be processed first

      calcBasicKinematicsJacobian(); // need cog_jacobian_

      calcLambdaJacobian();

      calcJointTorque(false);

      calcJointTorqueJacobian();

      calcFeasibleControlJacobian();
    }


    void RobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
    {
      joint_positions_ = joint_positions;

      KDL::RigidBodyInertia link_inertia = KDL::RigidBodyInertia::Zero();
      const auto seg_tf_map = fullForwardKinematics(joint_positions);
      setSegmentsTf(seg_tf_map);

      for(const auto& inertia : inertia_map_)
        {
          KDL::Frame f = seg_tf_map.at(inertia.first);
          link_inertia = link_inertia + f * inertia.second;

          /* process for the extra module */
          for(const auto& extra : extra_module_map_)
            {
              if(extra.second.getName() == inertia.first)
                {
                  link_inertia = link_inertia + f * (extra.second.getFrameToTip() * extra.second.getInertia());
                }
            }
        }

      /* CoG */
      KDL::Frame f_baselink = seg_tf_map.at(baselink_);
      KDL::Frame cog;
      cog.M = f_baselink.M * cog_desire_orientation_.Inverse();
      cog.p = link_inertia.getCOG();
      setCog(cog);
      mass_ = link_inertia.getMass();

      setInertia((cog.Inverse() * link_inertia).getRotationalInertia());
      setCog2Baselink(cog.Inverse() * f_baselink);

      /* thrust point based on COG */
      std::vector<KDL::Vector> rotors_origin_from_cog, rotors_normal_from_cog;
      for(int i = 0; i < rotor_num_; ++i)
        {
          std::string rotor = thrust_link_ + std::to_string(i + 1);
          KDL::Frame f = seg_tf_map.at(rotor);
          if(verbose_) ROS_WARN(" %s : [%f, %f, %f]", rotor.c_str(), f.p.x(), f.p.y(), f.p.z());
          rotors_origin_from_cog.push_back((cog.Inverse() * f).p);
          rotors_normal_from_cog.push_back((cog.Inverse() * f).M * KDL::Vector(0, 0, 1));
        }
      setRotorsNormalFromCog(rotors_normal_from_cog);
      setRotorsOriginFromCog(rotors_origin_from_cog);

      /* statics */
      calcStaticThrust();
      calcFeasibleControlFDists();
      calcFeasibleControlTDists();
    }
  }
} //namespace aerial_robot_model

