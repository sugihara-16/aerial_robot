// -*- mode: c++ -*-

#pragma once
#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <twin_hammer/model/twin_hammer_model.h>
// #include <gimbalrotor/control/gimbalrotor_controller.h>
namespace aerial_robot_control
{
  class TwinHammerController: public PoseLinearController
  {
  public:
    TwinHammerController();
    ~TwinHammerController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate
                    ) override;

  private:
    boost::shared_ptr<TwinHammerModel> twin_hammer_model_;
    ros::Publisher flight_cmd_pub_;
    ros::Publisher gimbal_control_pub_;
    ros::Subscriber haptics_switch_sub_;
    ros::Subscriber haptics_wrench_sub_;
    std::vector<float> target_base_thrust_;
    std::vector<double> target_gimbal_angles_;
    Eigen::VectorXd target_vectoring_f_;
    bool use_haptics_flag_;
    bool haptics_switch_;
    Eigen::Vector3d haptics_force_;
    Eigen::Vector3d haptics_torque_;
    void sendCmd() override;
    void HapticsSwitchCallback(std_msgs::Int8 msg);
    void HapticsWrenchCallback(geometry_msgs::WrenchStamped msg);

  protected:
    void controlCore() override;
    void rosParamInit() ;
  };
};
