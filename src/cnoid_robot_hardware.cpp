/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Yohei Kakiuchi (JSK lab.)
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
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
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

/*
 Author: Yohei Kakiuchi
*/

#include "cnoid_robot_hardware.h"
#include <urdf/model.h>

#include <thread>

namespace cnoid_robot_hardware
{

bool CnoidRobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)// add joint list
{
  // reading paramerters

  //prev_ref_positions_.resize(number_of_angles_);

  std::string model_str;
  if (!root_nh.getParam("robot_description", model_str)) {
    ROS_ERROR("Failed to get model from robot_description");
    return false;
  }
  urdf::Model model;
  if (!model.initString(model_str)) {
    ROS_ERROR("Failed to parse robot_description");
    return false;
  }
#if 0
  ROS_WARN("model: %d %d", model.joints_.size(),
           model.links_.size());
  for (std::map<std::string, urdf::JointSharedPtr>::iterator joint = model.joints_.begin();joint != model.joints_.end(); joint++)
    {
      ROS_WARN("j: %s, %s", joint->first.c_str(),
               joint->second->name.c_str());
    }
#endif

  // set number of angles from model
  number_of_angles_ = use_joints.size();
  // joint_names_.resize(number_of_angles_);
  joint_types_.resize(number_of_angles_);
  joint_lower_limits_.resize(number_of_angles_);
  joint_upper_limits_.resize(number_of_angles_);
  joint_effort_limits_.resize(number_of_angles_);
  joint_control_methods_.resize(number_of_angles_);
  //pid_controllers_.resize(number_of_angles_);
  joint_position_.resize(number_of_angles_);
  joint_velocity_.resize(number_of_angles_);
  joint_effort_.resize(number_of_angles_);
  joint_effort_command_.resize(number_of_angles_);
  joint_position_command_.resize(number_of_angles_);
  joint_velocity_command_.resize(number_of_angles_);

  // Initialize values
  for(unsigned int j = 0; j < number_of_angles_; j++) {
    std::string jointname = use_joints[j];
    cnoid::Link* joint = cnoid_body->link(jointname);
    // Add data from transmission
    joint_position_[j]         = joint->q(); // initialize
    joint_position_command_[j] = joint->q(); //
    joint_velocity_[j]         = joint->dq();
    joint_velocity_command_[j] = joint->dq();
    joint_effort_[j]           = joint->u();  // N/m for continuous joints
    joint_effort_command_[j]   = joint->u();

    ROS_INFO("joint: %s / [initial] q: %f, dq: %f, u: %f / [gain] P: %f, D: %f",
             jointname.c_str(), joint->q(), joint->dq(), joint->u(),
             p_gain[j], d_gain[j]);

    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        jointname, &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

    joint_control_methods_[j] = POSITION;
    hardware_interface::JointHandle joint_handle =
      hardware_interface::JointHandle(js_interface_.getHandle(jointname),
                                      &joint_position_command_[j]);
    pj_interface_.registerHandle(joint_handle);

    joint_limits_interface::JointLimits limits;
    const bool urdf_limits_ok = joint_limits_interface::getJointLimits(model.getJoint(jointname), limits);
    if (!urdf_limits_ok) {
      ROS_WARN("urdf limits of joint %s is not defined", jointname.c_str());
    }
    // Register handle in joint limits interface
    joint_limits_interface::PositionJointSaturationHandle
      limits_handle(joint_handle, // We read the state and read/write the command
                    limits);       // Limits spec
    pj_sat_interface_.registerHandle(limits_handle);
  }

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&pj_interface_);
  //registerInterface(&vj_interface_);
  //registerInterface(&ej_interface_);

  return true;
}

void CnoidRobotHW::read(const ros::Time& time, const ros::Duration& period)
{
  // copy choreonoid body from ...
  for(int i = 0; i < use_joints.size(); ++i) {
    cnoid::Link* joint = cnoid_body->link(use_joints[i]);
    joint_position_[i] = joint->q();
    joint_velocity_[i] = joint->dq();
    joint_effort_[i]   = joint->u();
  }
  return;
}

void CnoidRobotHW::write(const ros::Time& time, const ros::Duration& period)
{
  // write choreonoid body to ...
  /// command ???
  // flipper: 4
  // jaco:    6
  // hand:    3
  //ROS_INFO("tm: %f", time.toSec());
  for(int i = 0; i < use_joints.size(); ++i) {
    cnoid::Link* joint = cnoid_body->link(use_joints[i]);
    double tq;
    double pgain = p_gain[i];
    //double igain = i_gain[i];
    double dgain = d_gain[i];

    tq = pgain*(joint_position_command_[i] - joint->q()) - dgain*joint->dq();

    //ROS_INFO("%f = %f %f %f", tq, joint_position_command_[i], joint->q(), joint->dq());
    joint->u() = tq;
    //joint->u() = 0.0;
  }
  return;
}
}
