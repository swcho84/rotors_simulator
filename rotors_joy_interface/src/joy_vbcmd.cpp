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
#include "rotors_joy_interface/joy_vbcmd.h"

JoyVbCmd::JoyVbCmd() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ctrl_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust> (
    "/command/body_velocity_cmd", 1);

  namespace_ = nh_.getNamespace();
  joy_sub_ = nh_.subscribe("/joy", 1, &JoyVbCmd::JoyVbCmdCallback, this);    

  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust.x = 0;
  control_msg_.thrust.y = 0;
  control_msg_.thrust.z = 0;

  current_yaw_ang_ = 0;

  // for xbox360, mode 1
  pnh.param("axis_vby_", axes_.vby, 2);
  pnh.param("axis_vbx_", axes_.vbx, 1);
  pnh.param("axis_vbz_", axes_.vbz, 3);
  pnh.param("axis_yawang_", axes_.yawang, 0);

  pnh.param("axis_direction_roll", axes_.vby_direction, -1);
  pnh.param("axis_direction_pitch", axes_.vbx_direction, 1);
  pnh.param("axis_direction_yaw", axes_.yawang_direction, 1);
  pnh.param("axis_direction_thrust", axes_.vbz_direction, 1);

  pnh.param("max_vbx", max_.vbx, 2.0);  // [m/s]
  pnh.param("max_vby", max_.vby, 2.0);  // [m/s]
  pnh.param("max_vbz", max_.vbz, 1.0);  // [m/s]
}

void JoyVbCmd::JoyVbCmdCallback(const sensor_msgs::JoyConstPtr& msg) {
  current_joy_ = *msg;
  control_msg_.roll = msg->axes[axes_.vby] * max_.vby * axes_.vby_direction;
  control_msg_.pitch = msg->axes[axes_.vbx] * max_.vbx * axes_.vbx_direction;
  control_msg_.thrust.z = msg->axes[axes_.vbz] * max_.vbz * axes_.vbz_direction;

  // -1.0: yaw left turn, +1.0: yaw right turn
  control_msg_.yaw_rate = msg->axes[axes_.yawang] * axes_.yawang_direction * M_PI;

  ros::Time update_time = ros::Time::now();
  control_msg_.header.stamp = update_time;
  control_msg_.header.frame_id = "rotors_joy_frame";
  Publish();
}

void JoyVbCmd::Publish() {
  ctrl_pub_.publish(control_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "rotors_joy_vbcmd_interface");
  JoyVbCmd joyVbCmd;

  ros::spin();

  return 0;
}
