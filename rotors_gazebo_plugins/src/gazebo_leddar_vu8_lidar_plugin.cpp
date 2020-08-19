/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
 *
*/
#include "rotors_gazebo_plugins/gazebo_leddar_vu8_lidar_plugin.h"

using namespace gazebo;
using namespace std;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(LidarLeddarVu8Plugin)

LidarLeddarVu8Plugin::LidarLeddarVu8Plugin() {}

LidarLeddarVu8Plugin::~LidarLeddarVu8Plugin()
{
  this->newLaserScansConnection.reset();

  this->parentSensor.reset();
  this->world.reset();
}

void LidarLeddarVu8Plugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  if (kPrintOnPluginLoad)
  {
    gzdbg << __FUNCTION__ << "() called." << std::endl;
  }

  // Get then name of the parent sensor
  this->parentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parentSensor)
    gzthrow("RayPlugin requires a Ray Sensor as its parent");

  this->world = physics::get_world(this->parentSensor->WorldName());
  this->newLaserScansConnection = this->parentSensor->LaserShape()->ConnectNewLaserScans(boost::bind(&LidarLeddarVu8Plugin::OnNewLaserScans, this));

  getSdfParam<std::string>(_sdf, "lidar_topic", lidar_topic, "/lidar_default/info");
  getSdfParam<float>(_sdf, "min_range", min_range, 0.5);
  getSdfParam<float>(_sdf, "max_range", max_range, 30.0);
  getSdfParam<float>(_sdf, "min_angle", min_angle, -0.418879);
  getSdfParam<float>(_sdf, "max_angle", max_angle, 0.418879);

	// simulated, left-to-right mode
  float segment_angle = (max_angle - min_angle) / 8;
  for (int i = 0; i < 8; i++) {
    angle_segments[i] = min_angle + (segment_angle / 2) + (segment_angle * i);
  }

  // ROS
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  
  lidar_pub_ = ros_nh_.advertise<rotors_gazebo_plugins::VuInfo>(lidar_topic, 10);
}

void LidarLeddarVu8Plugin::OnNewLaserScans()
{
  rotors_gazebo_plugins::VuInfo vuInfo;
  rotors_gazebo_plugins::VuChunk vuChunk[8];
  std::vector<double> ranges;
  
  parentSensor->Ranges(ranges); // 240
  int rangeCount = parentSensor->RangeCount();
  if (ranges.size() != rangeCount) {
    return;
  }

  int resolution = rangeCount / 8; // 30
  for (int chunkIdx = 0; chunkIdx < 8; ++chunkIdx) {
    // find min
    float ray_range = max_range;    
    for (int rayIdx = 0; rayIdx < resolution; ++rayIdx) {
      int idx = chunkIdx * 30 + rayIdx;
      if (ray_range > ranges[idx]) {
        ray_range = ranges[idx];
      }
    }

    // fill vuChunk
    if (ray_range > min_range && ray_range < max_range) {
      vuChunk[chunkIdx].segment_status = true;
      vuChunk[chunkIdx].angle = angle_segments[chunkIdx];
      vuChunk[chunkIdx].range = ray_range;
      vuChunk[chunkIdx].xrel = ray_range * cos(angle_segments[chunkIdx]);
      vuChunk[chunkIdx].yrel = ray_range * sin(angle_segments[chunkIdx]);
    } else {
      vuChunk[chunkIdx].segment_status = false;
      vuChunk[chunkIdx].angle = angle_segments[chunkIdx];
      vuChunk[chunkIdx].range = -1;
    }
    vuInfo.vu_chunk_array.push_back(vuChunk[chunkIdx]);
  }

  vuInfo.lidar_status = true;
  lidar_pub_.publish(vuInfo);
}
