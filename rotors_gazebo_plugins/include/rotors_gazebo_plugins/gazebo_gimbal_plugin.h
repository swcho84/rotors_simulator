/*
 * Copyright 2016 Pavel Vechersky, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 * Copyright 2020 Sungwook Cho <swcho84@cju.ac.kr>, Cheongju university, South Korea
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef ROTORS_GAZEBO_PLUGINS_GIMBAL_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_GIMBAL_PLUGIN_H

#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <gazebo/sensors/sensors.hh>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include "rotors_gazebo_plugins/SetFloat32.h"

#include "common.h"

namespace gazebo
{
class BottomGimbalPlugin : public ModelPlugin
{
public:
  BottomGimbalPlugin()
      : ModelPlugin() {}
  virtual ~BottomGimbalPlugin();

protected:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
  void OnUpdate(const common::UpdateInfo & /*_info*/);

private:
  sdf::ElementPtr sdf_;
  std::vector<event::ConnectionPtr> connections_;

  transport::NodePtr node_;
  physics::ModelPtr model_;
  physics::LinkPtr linkBase_;
  physics::LinkPtr linkPitch_;
  common::PID pidPitch_;

  double dPitchCmd_;
  event::ConnectionPtr updateConnection_;

  ros::ServiceServer srvGimbalPitchCmd_;
  ros::Publisher pubGimbalInfo_;

  bool SrvServerCallback(rotors_gazebo_plugins::SetFloat32Request &req, rotors_gazebo_plugins::SetFloat32Response &resp);
  void PublishGimbalInfo(double pitch);
};
} // namespace gazebo
#endif
