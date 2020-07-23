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
#include "body_velocity_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

BodyVelocityControllerNode::BodyVelocityControllerNode() {
  InitializeParams();

  ros::NodeHandle nh;

}

BodyVelocityControllerNode::~BodyVelocityControllerNode() { }

void BodyVelocityControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

}
void BodyVelocityControllerNode::Publish() {
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "body_velocity_controller_node");

  rotors_control::BodyVelocityControllerNode body_velocity_controller_node;

  ros::spin();

  return 0;
}
