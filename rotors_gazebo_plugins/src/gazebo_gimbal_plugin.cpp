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

#include "rotors_gazebo_plugins/gazebo_gimbal_plugin.h"

namespace gazebo
{

BottomGimbalPlugin::~BottomGimbalPlugin()
{
}

void BottomGimbalPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  model_ = model;

  node_ = transport::NodePtr(new transport::Node());
  node_->Init();

  dPitchCmd_ = 0.0;
  pidPitch_.Init(0.2, 0, 0, 1, -1, 10, -10);

  getSdfParam<std::string>(sdf, "linkBaseName", strLinkBase_, strLinkBase_, true);  
  getSdfParam<std::string>(sdf, "linkPitchName", strLinkPitch_, strLinkPitch_, true);    

  linkBase_ = model_->GetLink(strLinkBase_);
  linkPitch_ = model_->GetLink(strLinkPitch_);  

  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&BottomGimbalPlugin::OnUpdate, this, _1));
  gzmsg << "BottomGimbalPlugin::Init" << std::endl;

  // ROS
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ros::NodeHandle nh;
  srvGimbalPitchCmd_ = nh.advertiseService("/firefly/gimbal/set/pitch_cmd", &BottomGimbalPlugin::SrvServerCallback, this);  
  pubGimbalInfo_ = nh.advertise<std_msgs::Float32MultiArray>("/firefly/bottom_gimbal/attitude", 1);  

  subRawPointCloud2_ = nh.subscribe ("/firefly/vi_sensor/camera_depth/depth/points", 1, &BottomGimbalPlugin::CbPointCloud2, this);
  pubStabilizedPointCloud2_ = nh.advertise<sensor_msgs::PointCloud2> ("/firefly/vi_sensor/camera_depth/depth/points_stabilized", 1);
}

void BottomGimbalPlugin::OnUpdate(const common::UpdateInfo  &_info)
{
  if (!linkPitch_)
  {
    ROS_INFO("Please check gimbal_base_link..%d,%d", (int)(!linkBase_), (int)(!linkPitch_));
    return;
  }

  // roll axis: just stabilization, pitch axis: with command
  // with z-axis offset: 0.315 (need to tune)
  ignition::math::Pose3d baseLinkPose = linkBase_->WorldPose();
  ignition::math::Pose3d originAngle(
      baseLinkPose.Pos().X(),
      baseLinkPose.Pos().Y(),
      baseLinkPose.Pos().Z() - 0.025,
      0.0, 
      -1 * (dPitchCmd_ / 180 * 3.141592),
      baseLinkPose.Rot().Yaw());
  ignition::math::Vector3d originForce(0.0, 0.0, 0.0);

  linkPitch_->SetWorldPose(originAngle);
  linkPitch_->SetForce(originForce);

  // calculating stabilized pointcloud
/*
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, baseLinkPose.Rot().Yaw());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/firefly/gimbal_base_link", "/world"));

  sensor_msgs::PointCloud ptCloudOutput;
  for(size_t i = 0;i < ptCloudInput_.points.size();i++)
  {
    geometry_msgs::Point32 point = ptCloudInput_.points.at(i);
    tf::Vector3 vecRawPt(point.x,point.y,point.z);
    tf::Vector3 vecResPt = (transform) * (vecRawPt);
    geometry_msgs::Point32 T_point;
    T_point.x = vecResPt.getX();
    T_point.y = vecResPt.getY();
    T_point.z = vecResPt.getZ();
    ptCloudOutput.points.push_back(T_point);
  }

	// publishing stabilized pointcloud2
  sensor_msgs::PointCloud2 ptCloud2Res;
  sensor_msgs::convertPointCloudToPointCloud2(ptCloudOutput, ptCloud2Res);
  pubStabilizedPointCloud2_.publish(ptCloud2Res);
*/
  PublishGimbalInfo(dPitchCmd_);
}

bool BottomGimbalPlugin::SrvServerCallback(rotors_gazebo_plugins::SetFloat32Request &req, rotors_gazebo_plugins::SetFloat32Response &resp)
{
  dPitchCmd_ = req.data;
  resp.success = static_cast<unsigned char>(true);
  return true;
}

void BottomGimbalPlugin::PublishGimbalInfo(double pitch)
{
  std_msgs::Float32MultiArray msgGimbalInfo;
  msgGimbalInfo.data.resize(3);
  msgGimbalInfo.data[0] = 0.0;  // just stabilized
  msgGimbalInfo.data[1] = (float)(pitch);
  msgGimbalInfo.data[2] = 0.0;  // not used, just use the vehicle`s heading
  pubGimbalInfo_.publish(msgGimbalInfo);
}

void BottomGimbalPlugin::CbPointCloud2(const sensor_msgs::PointCloud2ConstPtr& ptCloud2Raw)
{
  //sensor_msgs::convertPointCloud2ToPointCloud(*ptCloud2Raw, ptCloudInput_);
}

GZ_REGISTER_MODEL_PLUGIN(BottomGimbalPlugin);

} // namespace gazebo
