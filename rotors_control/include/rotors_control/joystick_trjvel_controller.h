/*
 * Copyright 2020 Sungwook Cho, Cheongju Univ., South Korea
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

#ifndef ROTORS_CONTROL_JOYSTICK_TRJVEL_CONTROLLER_H
#define ROTORS_CONTROL_JOYSTICK_TRJVEL_CONTROLLER_H

// using vector type data
#include <iostream>
#include <string>
#include <stdio.h>
#include <signal.h>
#include <ctime>
#include <vector>
#include <dirent.h>
#include <fstream>

// essential header for ROS-OpenCV operation
#include <ros/ros.h>

// for using trajectory_msgs/MultiDOFJointTrajectory msg
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

// for using joystick
#include <sensor_msgs/Joy.h>

// for using eigen library
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// for using tf w.r.t the quaternion
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

// for using custom msg 
#include "rotors_control/NedPosEulerAtt.h"

#define PI          3.141592
#define R2D         180.0/PI
#define D2R         PI/180.0

// for xbox360 wired joystick
#define YAWAXIS     0
#define XNAXIS      1
#define YEAXIS      2
#define ZDAxis      3
#define YAWAXISDIR  1
#define XNAXISDIR   1
#define YEAXISDIR   1
#define ZDAXISDIR   1
#define XNPOSRES    2.0
#define YEPOSRES    2.0
#define ZDPOSRES    1.0

using namespace std;
using namespace ros;
using namespace Eigen;

class JoyTrjVelCntl
{
public:
  JoyTrjVelCntl();
  ~JoyTrjVelCntl();

  void MainLoop();

  string strJoyCntlName;  

private:
  NodeHandle nh_;

  Subscriber subJoyInfo_;
  void CbJoyInfo(const sensor_msgs::JoyConstPtr& msg);
  Eigen::Vector4d joyPose_;
  bool bCurrUseJoyConLoop_;
  bool bCurrUseExtGuidLoop_;
  bool bPrevUseJoyConLoop_;
  bool bPrevUseExtGuidLoop_;

  Subscriber subPoseInfo_;
  void CbPoseInfo(const geometry_msgs::PoseConstPtr& msg);
  Eigen::Vector3d mavPosEnu_;
  Eigen::Vector3d mavPosNed_;  
  Eigen::Vector3d mavEulerAtt_;
  double dHeightRef_;
  double dYawAngRef_;

  Publisher pubJoyTrjVelInfo_;
  void GenJoyConInfo();
  void GenExtGuideConInfo();

  Publisher pubNedPosEulerAttInfo_;
  
  Quaterniond CalcQuaternionFromYPREulerAng(Vector3d euler);
  Vector3d CalcYPREulerAngFromQuaternion(Quaterniond q);
  Matrix3d CalcDcmNtoB(Vector3d eulerAtt);
  Matrix3d CalcDcmBtoN(Vector3d eulerAtt);
  Matrix3d CalcDcmEuler321(Vector3d eulerAtt);
  Vector3d ConvertPosFromEnuToNed(Vector3d posEnu);
  double wrap_d(double _angle);
};

#endif // ROTORS_CONTROL_JOYSTICK_TRJVEL_CONTROLLER_H