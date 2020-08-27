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

#ifndef ROTORS_GAZEBO_PLUGINS_LEDDAR_VU8_PLUGIN_H
#define ROTORS_GAZEBO_PLUGINS_LEDDAR_VU8_PLUGIN_H

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>

// msgs
#include "rotors_gazebo_plugins/VuChunk.h"
#include "rotors_gazebo_plugins/VuInfo.h"

// ros
#include <ros/ros.h>

#include "common.h"

#define PIf 3.14159265359f
#define D2R PIf / 180.0f

namespace gazebo
{
class GAZEBO_VISIBLE LidarLeddarVu8Plugin : public SensorPlugin
{
public:
  LidarLeddarVu8Plugin();
  virtual ~LidarLeddarVu8Plugin();
  virtual void OnNewLaserScans();
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

protected:
  physics::WorldPtr world;

private:
  float angle_segments[8];

  std::string namespace_;
  std::string lidar_topic;

  float min_angle;
  float max_angle;
  float min_range;
  float max_range;

  sensors::RaySensorPtr parentSensor;

  ros::NodeHandle ros_nh_;
  ros::Publisher lidar_pub_;

  event::ConnectionPtr newLaserScansConnection;
};
} // namespace gazebo
#endif
