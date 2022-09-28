#include <aerial_robot_model/kdl_test.h>

namespace aerial_robot_model {

  RobotModel::RobotModel():
    baselink_("fc"),
    thrust_link_("thrust"),
    rotor_num_(0),
    joint_num_(0),
    mass_(0),
    initialized_(false)
  {
    gravity_.resize(6);
    gravity_ <<  0, 0, 9.80665, 0, 0, 0;
    gravity_3d_.resize(3);
    gravity_3d_ << 0, 0, 9.80665;

    kinematicsInit();
    ROS_INFO("kdl_test is activated");
  }

  void RobotModel::kinematicsInit(std::string robot_description)
  {
    /* robot model */
    if (!model_.initParam(robot_description))
      {
        ROS_ERROR("Failed to extract urdf model from rosparam");
        return;
      }
    if (!kdl_parser::treeFromUrdfModel(model_, tree_))
      {
        ROS_ERROR("Failed to extract kdl tree from xml robot description");
        return;
      }
    /* get baselink and thrust_link from robot model */
    auto robot_model_xml = getRobotModelXml(robot_description);
    TiXmlElement* baselink_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("baselink");
    if(!baselink_attr)
      ROS_DEBUG("Can not get baselink attribute from urdf model");
    else
      baselink_ = std::string(baselink_attr->Attribute("name"));

    TiXmlElement* thrust_link_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("thrust_link");
    if(!thrust_link_attr)
      ROS_DEBUG("Can not get thrust_link attribute from urdf model");
    else
      thrust_link_ = std::string(thrust_link_attr->Attribute("name"));

    if(!model_.getLink(baselink_))
      {
        ROS_ERROR_STREAM("Can not find the link named '" << baselink_ << "' in urdf model");
        return;
      }
    bool found_thrust_link = false;
    std::vector<urdf::LinkSharedPtr> urdf_links;
    model_.getLinks(urdf_links);
    for(const auto& link: urdf_links)
      {
        if(link->name.find(thrust_link_.c_str()) != std::string::npos)
          found_thrust_link = true;
      }
    if(!found_thrust_link)
      {
        ROS_ERROR_STREAM("Can not find the link named '" << baselink_ << "' in urdf model");
        return;
      }

    inertialSetup(tree_.getRootSegment()->second);
    makeJointSegmentMap();

    rotors_origin_from_cog_.resize(rotor_num_);
    rotors_normal_from_cog_.resize(rotor_num_);
  }

  KDL::RigidBodyInertia RobotModel::inertialSetup(const KDL::TreeElement& tree_element)
  {
    const KDL::Segment current_seg = GetTreeElementSegment(tree_element);

    KDL::RigidBodyInertia current_seg_inertia = current_seg.getInertia();
    if(verbose_) ROS_WARN_STREAM("segment " <<  current_seg.getName() << ", mass is: " << current_seg_inertia.getMass());

    /* check whether this can be a base inertia segment (i.e. link) */
    /* 1. for the "root" parent link (i.e. link1) */
    if(current_seg.getName().find("root") != std::string::npos)
      {
        assert(inertia_map_.size() == 0);
        assert(GetTreeElementChildren(tree_element).size() == 1);

        const KDL::Segment& child_seg = GetTreeElementSegment(GetTreeElementChildren(tree_element).at(0)->second);
        inertia_map_.insert(std::make_pair(child_seg.getName(), child_seg.getInertia()));
        if(verbose_) ROS_WARN("Add root link: %s", child_seg.getName().c_str());

      }
    /* 2. for segment that has joint with parent segment */
    if (current_seg.getJoint().getType() != KDL::Joint::None)
      {
        /* add the new inertia base (child) link if the joint is not a rotor */
        if(current_seg.getJoint().getName().find("rotor") == std::string::npos)
          {
            /* create a new inertia base link */
            inertia_map_.insert(std::make_pair(current_seg.getName(), current_seg_inertia));
            joint_index_map_.insert(std::make_pair(current_seg.getJoint().getName(), tree_element.q_nr));
            joint_names_.push_back(current_seg.getJoint().getName());
            joint_indices_.push_back(tree_element.q_nr);
            joint_parent_link_names_.push_back(GetTreeElementParent(tree_element)->first);

            if(verbose_) ROS_WARN("Add new inertia base link: %s", current_seg.getName().c_str());
          }
      }
    /* special process for rotor */
    if(current_seg.getJoint().getName().find("rotor") != std::string::npos)
      {
        /* add the rotor direction */
        auto urdf_joint =  model_.getJoint(current_seg.getJoint().getName());
        if(urdf_joint->type == urdf::Joint::CONTINUOUS)
          {
            if(verbose_) ROS_WARN("joint name: %s, z axis: %f", current_seg.getJoint().getName().c_str(), urdf_joint->axis.z);
            rotor_direction_.insert(std::make_pair(std::atoi(current_seg.getJoint().getName().substr(5).c_str()), urdf_joint->axis.z));
          }
      }

    /* recursion process for children segment */
    for (const auto& elem: GetTreeElementChildren(tree_element))
      {
        const KDL::Segment& child_seg = GetTreeElementSegment(elem->second);
        KDL::RigidBodyInertia child_seg_inertia = child_seg.getFrameToTip() *  inertialSetup(elem->second);
        KDL::RigidBodyInertia current_seg_inertia_old = current_seg_inertia;
        current_seg_inertia = current_seg_inertia_old + child_seg_inertia;

        if(verbose_) ROS_WARN("Add new child segment %s to direct segment: %s", child_seg.getName().c_str(), current_seg.getName().c_str());
      }

    /* count the rotor */
    if(current_seg.getName().find(thrust_link_.c_str()) != std::string::npos) rotor_num_++;
    /* update the inertia if the segment is base */
    if (inertia_map_.find(current_seg.getName()) != inertia_map_.end())
      {
        inertia_map_.at(current_seg.getName()) = current_seg_inertia;

        if(verbose_) ROS_WARN("Total mass of base segment %s is %f", current_seg.getName().c_str(),
                              inertia_map_.at(current_seg.getName()).getMass());
        current_seg_inertia = KDL::RigidBodyInertia::Zero();
      }

    return current_seg_inertia;
  }

  void RobotModel::makeJointSegmentMap()
  {
    joint_segment_map_.clear();
    for (const auto joint_index : joint_index_map_) {
      std::vector<std::string> empty_vec;
      joint_segment_map_[joint_index.first] = empty_vec;
    }

    std::vector<std::string> current_joints;
    jointSegmentSetupRecursive(getTree().getRootSegment()->second, current_joints);
  }

  void RobotModel::jointSegmentSetupRecursive(const KDL::TreeElement& tree_element, std::vector<std::string> current_joints)
  {
    const auto inertia_map = getInertiaMap();
    const KDL::Segment current_seg = GetTreeElementSegment(tree_element);
    bool add_joint_flag = false;

    // if this segment has a real joint except rotor
    if (current_seg.getJoint().getType() != KDL::Joint::None && current_seg.getJoint().getName().find("rotor") == std::string::npos) {
      std::string focused_joint = current_seg.getJoint().getName();
      joint_hierachy_.insert(std::make_pair(focused_joint, current_joints.size()));
      current_joints.push_back(focused_joint);
      bool add_joint_flag = true;
      joint_num_++;
    }

    // if this segment is a real segment (= not having fixed joint)
    if (inertia_map.find(current_seg.getName()) != inertia_map.end() || current_seg.getName().find("thrust") != std::string::npos) {
      for (const auto& cj : current_joints) {
        joint_segment_map_.at(cj).push_back(current_seg.getName());
      }
    }

    // recursive process
    for (const auto& elem: GetTreeElementChildren(tree_element)) {
      jointSegmentSetupRecursive(elem->second, current_joints);
    }

    return;
  }


  TiXmlDocument RobotModel::getRobotModelXml(const std::string param, ros::NodeHandle nh)
  {
    std::string xml_string;
    TiXmlDocument xml_doc;

    if (!nh.hasParam(param))
      {
        ROS_ERROR("Could not find parameter %s on parameter server with namespace '%s'", param.c_str(), nh.getNamespace().c_str());
        return xml_doc;
      }

    nh.getParam(param, xml_string);
    xml_doc.Parse(xml_string.c_str());

    return xml_doc;
  }

} //namespace aerial_robot_model

int main (int argc, char **argv)
{
  ros::init (argc, argv, "kdl_test");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~"); // node handle with private namespace
  ROS_INFO("kdl_test is activated");

  aerial_robot_model::RobotModel robot_model;

  return 0;
}
