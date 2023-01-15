
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

#include <assemble_quadrotors/optimal_design.h>

int cnt = 0;

enum mechanics_parm{
  FX,
  FZ,
  FY,
  TX,
  TY,
  TZ
};

void calcRotorConfiguration(const std::vector<double>& x_thrust,const std::vector<double>& x, const int unit_rotor_num, const double pos_bound, const double m_f_rate, std::vector<Eigen::Vector3d>& p, std::vector<Eigen::Vector3d>& u, std::vector<Eigen::Vector3d>& v, std::vector<double>& direct, const bool unit_calc)
{
  p.clear();
  u.clear();
  v.clear();
  direct.clear();

  // determine the origin point by the type of calclation
  Eigen::Vector3d  unit1_center_pos;
  int calc_num;

  if(unit_calc){
    unit1_center_pos(0) = 0;
    unit1_center_pos(1) = 0;
    unit1_center_pos(2) = 0;
  }else{
    unit1_center_pos(0) = pos_bound;
    unit1_center_pos(1) = 0;
    unit1_center_pos(2) = 0;
  }
    // x: [ang1, ang2] * n/2

  for(int i = 0; i < unit_rotor_num; i++) {
    double direct_i = std::pow(-1, i);
    Eigen::Vector3d p_i;
    Eigen::Vector3d p_i_pair;
    switch(i) {
    case 0:
      p_i = unit1_center_pos + Eigen::Vector3d(-pos_bound/2, -pos_bound/2, 0);
      break;
    case 1:
      p_i = unit1_center_pos + Eigen::Vector3d(+pos_bound/2, -pos_bound/2, 0);
      break;
    case 2:
      p_i = unit1_center_pos + Eigen::Vector3d(+pos_bound/2, +pos_bound/2, 0);
      break;
    case 3:
      p_i = unit1_center_pos + Eigen::Vector3d(-pos_bound/2, +pos_bound/2, 0);
      break;
    default:
      break;
    }

    // http://fnorio.com/0098spherical_trigonometry1/spherical_trigonometry1.html
    double ang1 = x.at(i*2);
    double ang2 = x.at(i*2+1);
    Eigen::Vector3d u_i(sin(ang1) * cos(ang2), sin(ang1) * sin(ang2), cos(ang1));
    Eigen::Vector3d v_i = p_i.cross(u_i) - m_f_rate * direct_i * u_i;
    //std::cout<< "m_f_rate: " << m_f_rate <<  "; pi: " << p_i.transpose() << "; u_i: " << u_i.transpose() << "; vi: " << v_i.transpose() << std::endl;
    p.push_back(p_i); // append rotor position
    u.push_back(u_i); // append rotor force normal
    v.push_back(v_i); // append rotor torque normal
    direct.push_back(direct_i); // append propeller rotating direction

    // the mirror rotor
    // simplest rule: rotate the unit around the z axis
    if(!unit_calc){
      double direct_i_mirror = std::pow(-1, i); // append propeller rotating direction (same direction)
      Eigen::Vector3d p_i_mirror = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * p_i;
      Eigen::Vector3d u_i_mirror = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * u_i;
      Eigen::Vector3d v_i_mirror = p_i_mirror.cross(u_i_mirror) - m_f_rate * direct_i_mirror * u_i_mirror;
      p.push_back(p_i_mirror);
      u.push_back(u_i_mirror);
      v.push_back(v_i_mirror);
      direct.push_back(direct_i_mirror);
    }
  }

}


double objectiveFunc(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  OptimalDesign *planner = reinterpret_cast<OptimalDesign*>(ptr);
  std::vector<double> opt_angle = planner->test_x_;
  // get the p, u, v and direction
  std::vector<Eigen::Vector3d> p;
  std::vector<Eigen::Vector3d> u;
  std::vector<Eigen::Vector3d> v;
  std::vector<double> direct;

  calcRotorConfiguration(x,planner->test_x_, planner->unit_rotor_num_, planner->pos_bound_, planner->m_f_rate_, p, u, v, direct, false);

  double fx_sum = 0;
  double fy_sum = 0;
  double tx_sum = 0;
  double ty_sum = 0;
  double tz_sum = 0;
  double fz_sum = 0;
  for(int i = 0; i< planner->unit_rotor_num_*2; i++){
    fx_sum += u[i][0] * x[i] ;
    fy_sum += u[i][1] * x[i] ;
    fz_sum += x[i] * cos(opt_angle[i%4*2]);
    tx_sum += v[i][0] * x[i] ;
    ty_sum += v[i][1] * x[i] ;
    tz_sum += v[i][2] * x[i] ;
  }


  if (cnt % 10000 == 0) {
    // std::cout << "fx_sum" << fx_sum << std::endl;
    // std::cout << "fy_sum" << fy_sum << std::endl;
    // std::cout << "fz_sum" << fy_sum << std::endl;
    // std::cout << "tx_sum" << tx_sum << std::endl;
    // std::cout << "ty_sum" << ty_sum << std::endl;
    // std::cout << "tz_sum" << tz_sum << std::endl;
  }

   cnt++;

  switch(planner->target_axis_) {
  case FX:
    return fx_sum;
    break;
  case FY:
    return fy_sum;
    break;
  case TX:
    return tx_sum;
    break;
  case TY:
    return ty_sum;
    break;
  case TZ:
    return tz_sum;
    break;
  default:
    std::cout<<"given target axis is invalid."<<std::endl;
    break;
  }

}

// set the  bounds by constrains of unit rotor torque
double transZForceConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  OptimalDesign *planner = reinterpret_cast<OptimalDesign*>(ptr);
  std::vector<double> opt_angle = planner->test_x_;
  double fz_sum = 0;
  for(int i = 0; i< planner->unit_rotor_num_*2; i++){
    fz_sum += x[i] * cos(opt_angle[i%4*2]);
  }
  if (cnt % 10000 == 0) {
    std::cout<<"fz_trans :"<<fz_sum<<std::endl;
  }
  return ( fz_sum- 9.8 * planner->unit_mass_*2);
}

double transXyForceConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  OptimalDesign *planner = reinterpret_cast<OptimalDesign*>(ptr);

  // get the p, u, v and direction
  std::vector<Eigen::Vector3d> p;
  std::vector<Eigen::Vector3d> u;
  std::vector<Eigen::Vector3d> v;
  std::vector<double> direct;

  calcRotorConfiguration(x,planner->test_x_, planner->unit_rotor_num_, planner->pos_bound_, planner->m_f_rate_, p, u, v, direct, true);

  double fx_sum = 0;
  double fy_sum = 0;
  for(int i = 0; i< planner->unit_rotor_num_*2; i++){
    fx_sum += u[i][0] * x[i];
    fy_sum += u[i][1] * x[i];
  }

  switch(planner->target_axis_) {
  case FX:
    return (pow(fy_sum,2) - 0.1);
    break;
  case FY:
    return (pow(fx_sum, 2) - 0.1);
    break;
  default:
    return ( (pow(fx_sum, 2) + pow(fy_sum,2) - 0.1));
    break;
  }
}

double torqueConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  OptimalDesign *planner = reinterpret_cast<OptimalDesign*>(ptr);

  // get the p, u, v and direction
  std::vector<Eigen::Vector3d> p;
  std::vector<Eigen::Vector3d> u;
  std::vector<Eigen::Vector3d> v;
  std::vector<double> direct;

  calcRotorConfiguration(x,planner->test_x_, planner->unit_rotor_num_, planner->pos_bound_, planner->m_f_rate_, p, u, v, direct, true);

  double tx_sum = 0;
  double ty_sum = 0;
  double tz_sum = 0;
  for(int i = 0; i< planner->unit_rotor_num_*2; i++){
    tx_sum += v[i][0] * x[i] ;
    ty_sum += v[i][1] * x[i] ;
    tz_sum += v[i][2] * x[i] ;
  }

  switch(planner->target_axis_) {
  case TX:
    return (pow(ty_sum,2) + pow(tz_sum,2) - 1e-03);
    break;
  case TY:
    return (pow(tx_sum,2) + pow(tz_sum,2) - 1e-03);
    break;
  case TZ:
    return (pow(tx_sum,2) + pow(ty_sum,2) - 1e-03);
    break;
  default:
    return (pow(tx_sum,2) + pow(ty_sum,2) + pow(tz_sum,2) - 1e-03);
    break;
  }
}



OptimalDesign::OptimalDesign(ros::NodeHandle nh, ros::NodeHandle nhp)
{
  // intialize variables
  nhp.param("unit_rotor_num", unit_rotor_num_, 4);
  nhp.param("unit_mass", unit_mass_, 1.2); // [kg]
  nhp.param("max_thrust", max_thrust_, 10.0); // [N]
  nhp.param("fc_fmin_weight", fc_f_min_weight_, 1.0);
  nhp.param("fc_tmin_weight", fc_t_min_weight_, 1.0);
  nhp.param("pos_bound", pos_bound_, 0.22); // [m]
  nhp.param("m_f_rate", m_f_rate_, -0.011); // [Nm/N]
  nhp.param("test_mode", test_mode_, true);
  nhp.param("unit_mode", unit_mode_, false);
  nhp.param("test_x", test_x_, {0.523525,0.73108,0.523535,-2.08113,0.523595,2.4662,0.348489,-0.673131});
  nhp.param("target_axis", target_axis_, 0);

  units_num_ = 2; // we only consider two units

  nlopt::opt optimizer_solver(nlopt::GN_ISRES, unit_rotor_num_ * 2); // chose the proper optimization model
  optimizer_solver.set_max_objective(objectiveFunc, this); // register "this" as the second arg in objectiveFunc

  //set bounds
  std::vector<double> lb(unit_rotor_num_ * 2);
  std::vector<double> ub(unit_rotor_num_ * 2);
  for(int i = 0; i < unit_rotor_num_; i++) {
    lb.at(2 * i) = 0;
    ub.at(2 * i) = 10.0;

    lb.at(2 * i + 1) = 0; // lower bound of ang1
    ub.at(2 * i + 1) = 10.0; // upper bound of ang1
  }
  optimizer_solver.set_lower_bounds(lb);
  optimizer_solver.set_upper_bounds(ub);

  optimizer_solver.add_inequality_constraint(transXyForceConstraint, this, 1e-8);
  optimizer_solver.add_inequality_constraint(transZForceConstraint, this, 1e-8);
  // optimizer_solver.add_inequality_constraint(torqueConstraint, this, 1e-8);


  optimizer_solver.set_xtol_rel(1e-4); //1e-4
  int max_eval;
  nhp.param("max_eval", max_eval, 100000);
  optimizer_solver.set_maxeval(max_eval);
  double max_val;
  std::vector<double> opt_x(unit_rotor_num_ * 2,  0.01);
  std::vector<double> opt_thrust(unit_rotor_num_ * 2,  0.01);
  bool search_flag;
  nhp.param("search_flag", search_flag, true);
  opt_x = test_x_;
  if (search_flag) {
    nlopt::result result = optimizer_solver.optimize(opt_thrust, max_val);
    if (result != nlopt::SUCCESS)
      ROS_WARN_STREAM("the optimize solution does not succeed, result is " << result);
  }

  //output

  for (const auto &item : opt_thrust) {
    std::cout << item << "; ";
  }
  std::cout << std::endl;

  std::vector<Eigen::Vector3d> p;
  std::vector<Eigen::Vector3d> u;
  std::vector<Eigen::Vector3d> v;
  std::vector<double> direct;

  calcRotorConfiguration(opt_thrust,test_x_, unit_rotor_num_, pos_bound_,m_f_rate_, p, u, v, direct, false);

  // double fz_sum = 0;
  // for(int i = 0; i< unit_rotor_num_*2; i++){
  //   fz_sum += opt_thrust[i] * cos(test_x_[i%4*2]);
  // }

  // std::cout<<"fz_sum :"<<fz_sum<<std::endl;

  double fx_sum = 0;
  double fy_sum = 0;
  double tx_sum = 0;
  double ty_sum = 0;
  double tz_sum = 0;
  double fz_sum = 0;
  for(int i = 0; i< unit_rotor_num_*2; i++){
    fx_sum += u[i][0] * opt_thrust[i] ;
    fy_sum += u[i][1] * opt_thrust[i] ;
    fz_sum += opt_thrust[i] * cos(test_x_[i%4*2]);
    tx_sum += v[i][0] * opt_thrust[i] ;
    ty_sum += v[i][1] * opt_thrust[i] ;
    tz_sum += v[i][2] * opt_thrust[i] ;
  }

  std::cout << "fx_sum: " << fx_sum << std::endl;
  std::cout << "fy_sum: " << fy_sum << std::endl;
  std::cout << "fz_sum: " << fz_sum << std::endl;
  std::cout << "tx_sum: " << tx_sum << std::endl;
  std::cout << "ty_sum: " << ty_sum << std::endl;
  std::cout << "tz_sum: " << tz_sum << std::endl;


  ros::Duration(1.0).sleep();
}


int main (int argc, char **argv)
{
  ros::init (argc, argv, "optimal_thrust_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~"); // node handle with private namespace

  OptimalDesign optimizer(nh, nhp);

  return 0;
}
