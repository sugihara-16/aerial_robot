// -*- mode: c++ -*-

#pragma once
#include <twin_hammer/model/twin_hammer_model.h>
#include <gimbalrotor/control/gimbalrotor_controller.h>

namespace aerial_robot_control
{
  class TwinHammerController: public GimbalrotorController
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
    void controlCore() override;

  protected:
    void rosParamInit() override;
    void reset() override;
  };
};
