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
#ifndef ROTORS_CONTROL_BODY_VELOCITY_CONTROLLER_NODE_H
#define ROTORS_CONTROL_BODY_VELOCITY_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>

#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "rotors_control/common.h"
#include "rotors_control/body_velocity_controller.h"

namespace rotors_control {

class BodyVelocityControllerNode {
 public:
  BodyVelocityControllerNode();
  ~BodyVelocityControllerNode();

  void InitializeParams();
  void Publish();

 private:

  BodyVelocityController body_velocity_controller_;

  std::string namespace_;

  // subscribers
};
}

#endif // ROTORS_CONTROL_BODY_VELOCITY_CONTROLLER_NODE_H
