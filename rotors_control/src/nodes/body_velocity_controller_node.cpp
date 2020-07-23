/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2020 Sungwook Cho, VASRL, Cheongju University, Cheongju, South Korea
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "body_velocity_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

BodyVelocityControllerNode::BodyVelocityControllerNode(
  const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  :nh_(nh),
   private_nh_(private_nh){
  InitializeParams();

  cmd_bodyvel_sub_ = nh_.subscribe("/firefly/command/body_velocity_cmd", 1,
                                     &BodyVelocityControllerNode::BodyVelCmdCallback, this);
  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &BodyVelocityControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);  

}

BodyVelocityControllerNode::~BodyVelocityControllerNode() {
}

void BodyVelocityControllerNode::InitializeParams() {

  // Read parameters from rosparam.
  GetRosParameter(private_nh_, "p_gain/vbx",
                  body_velocity_controller_.controller_parameters_.p_gain_.x(),
                  &body_velocity_controller_.controller_parameters_.p_gain_.x());
  GetRosParameter(private_nh_, "p_gain/vby",
                  body_velocity_controller_.controller_parameters_.p_gain_.y(),
                  &body_velocity_controller_.controller_parameters_.p_gain_.y());
  GetRosParameter(private_nh_, "p_gain/vbz",
                  body_velocity_controller_.controller_parameters_.p_gain_.z(),
                  &body_velocity_controller_.controller_parameters_.p_gain_.z());
  GetRosParameter(private_nh_, "i_gain/vbx",
                  body_velocity_controller_.controller_parameters_.i_gain_.x(),
                  &body_velocity_controller_.controller_parameters_.i_gain_.x());
  GetRosParameter(private_nh_, "i_gain/vby",
                  body_velocity_controller_.controller_parameters_.i_gain_.y(),
                  &body_velocity_controller_.controller_parameters_.i_gain_.y());
  GetRosParameter(private_nh_, "i_gain/vbz",
                  body_velocity_controller_.controller_parameters_.i_gain_.z(),
                  &body_velocity_controller_.controller_parameters_.i_gain_.z());
  GetRosParameter(private_nh_, "attitude_gain/x",
                  body_velocity_controller_.controller_parameters_.attitude_gain_.x(),
                  &body_velocity_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(private_nh_, "attitude_gain/y",
                  body_velocity_controller_.controller_parameters_.attitude_gain_.y(),
                  &body_velocity_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(private_nh_, "attitude_gain/z",
                  body_velocity_controller_.controller_parameters_.attitude_gain_.z(),
                  &body_velocity_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(private_nh_, "angular_rate_gain/x",
                  body_velocity_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &body_velocity_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(private_nh_, "angular_rate_gain/y",
                  body_velocity_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &body_velocity_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(private_nh_, "angular_rate_gain/z",
                  body_velocity_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &body_velocity_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(private_nh_, &body_velocity_controller_.vehicle_parameters_);
  body_velocity_controller_.InitializeParameters(); 

}
void BodyVelocityControllerNode::Publish() {
}

void BodyVelocityControllerNode::BodyVelCmdCallback(
  const mav_msgs::RollPitchYawrateThrustConstPtr& body_vel_cmd_reference_msg) {

}

void BodyVelocityControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "body_velocity_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rotors_control::BodyVelocityControllerNode body_velocity_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}
