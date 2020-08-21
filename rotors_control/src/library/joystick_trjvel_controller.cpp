#include "rotors_control/joystick_trjvel_controller.h"

using namespace std;
using namespace ros;
using namespace Eigen;

JoyTrjVelCntl::JoyTrjVelCntl()
{
  // assigning subscriber and publisher
  subJoyInfo_ = nh_.subscribe("/joy", 1, &JoyTrjVelCntl::CbJoyInfo, this);
  subPoseInfo_ = nh_.subscribe("/firefly/odometry_sensor1/pose", 1, &JoyTrjVelCntl::CbPoseInfo, this);
  pubJoyTrjVelInfo_ = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/firefly/command/trajectory", 1);

  // initializing joystick pose information
  for (unsigned int i = 0; i < 4; i++)
    joyPose_(i) = 0.0;

  // initializing mav pose information
  for (unsigned int i = 0; i < 3; i++)
  {
    mavPos_(i) = 0.0;
    mavAtt_(i) = 0.0;
  }

  dHeightRef_ = 0.0;
  dYawAngRef_ = 0.0;

  bCurrUseJoyConLoop_ = false;
  bCurrUseExtGuidLoop_ = false;  
  bPrevUseJoyConLoop_ = false;
  bPrevUseExtGuidLoop_ = false;    
}

JoyTrjVelCntl::~JoyTrjVelCntl()
{

}

void JoyTrjVelCntl::MainLoop()
{
  if (bCurrUseJoyConLoop_)
  {
    GenJoyConInfo();
    return;
  }
  
  if (bCurrUseExtGuidLoop_)
  {
    ROS_INFO("external guidance control input loop...");
    return;
  }

  ROS_INFO_DELAYED_THROTTLE(10, "waiting controller setup(joystick, external guidance loop)...");
  return;
}

void JoyTrjVelCntl::GenJoyConInfo()
{
  trajectory_msgs::MultiDOFJointTrajectory msgJoyTrjVelInfo;
  trajectory_msgs::MultiDOFJointTrajectoryPoint msgJoyVelInfo;
  geometry_msgs::Transform msgCurrPose;
  geometry_msgs::Twist msgCurrJoyVel;
  geometry_msgs::Twist msgCurrZeroAcc;
  msgJoyTrjVelInfo.points.clear();
  msgJoyVelInfo.transforms.clear();
  msgJoyVelInfo.velocities.clear();
  msgJoyVelInfo.accelerations.clear();

  // assigning zero state
  msgCurrPose.translation.x = 0.0;
  msgCurrPose.translation.y = 0.0; 
  msgCurrJoyVel.angular.x = 0.0;
  msgCurrJoyVel.angular.y = 0.0;
  msgCurrJoyVel.angular.z = 0.0;  
  msgCurrZeroAcc.linear.x = 0.0;
  msgCurrZeroAcc.linear.y = 0.0;
  msgCurrZeroAcc.linear.z = 0.0;
  msgCurrZeroAcc.angular.x = 0.0;
  msgCurrZeroAcc.angular.y = 0.0;
  msgCurrZeroAcc.angular.z = 0.0;  
  
  // assigning velocity control using joystick, body axis, translation
  Vector3d bodyVelCmd;
  bodyVelCmd(0) = (XNPOSRES) * (joyPose_(1));
  bodyVelCmd(1) = (YEPOSRES) * (joyPose_(2));
  bodyVelCmd(2) = 0.0;
  Matrix3d DcmNtoB;
  Matrix3d DcmBtoN;
  DcmNtoB = CalcDcmNtoB(mavAtt_);
  DcmBtoN = CalcDcmBtoN(mavAtt_);
  Vector3d nedVelCmd;
  nedVelCmd = (DcmBtoN) * (bodyVelCmd);
  msgCurrJoyVel.linear.x = nedVelCmd(0);
  msgCurrJoyVel.linear.y = nedVelCmd(1);
  msgCurrJoyVel.linear.z = (ZDPOSRES) * (joyPose_(3));
  dHeightRef_ += (msgCurrJoyVel.linear.z) * (0.01);
  dYawAngRef_ += (joyPose_(0)) * (0.001);
  msgCurrPose.translation.z = dHeightRef_;

  // assigning yaw angle control using joystick, rotation
  Vector3d eulerAng;
  eulerAng(0) = 0.0;
  eulerAng(1) = 0.0;
  eulerAng(2) = wrap_d((dYawAngRef_) * (PI));
  Quaterniond rotQuat;
  rotQuat = CalcQuaternionFromYPREulerAng(eulerAng);
  msgCurrPose.rotation.x = rotQuat.x();
  msgCurrPose.rotation.y = rotQuat.y();
  msgCurrPose.rotation.z = rotQuat.z();
  msgCurrPose.rotation.w = rotQuat.w();  

  // pushing data for single trajectory point
  msgJoyVelInfo.transforms.push_back(msgCurrPose);
  msgJoyVelInfo.velocities.push_back(msgCurrJoyVel);
  msgJoyVelInfo.accelerations.push_back(msgCurrZeroAcc);
  msgJoyTrjVelInfo.points.push_back(msgJoyVelInfo);
  pubJoyTrjVelInfo_.publish(msgJoyTrjVelInfo);
}

void JoyTrjVelCntl::CbJoyInfo(const sensor_msgs::JoyConstPtr& msg)
{
  // 0: yaw, 1: XbVel, 2: YbVel, 3: ZbVel
  joyPose_(0) = (msg->axes[YAWAXIS]) * (YAWAXISDIR);
  joyPose_(1) = (msg->axes[XNAXIS]) * (XNAXISDIR);
  joyPose_(2) = (msg->axes[YEAXIS]) * (YEAXISDIR);
  joyPose_(3) = (msg->axes[ZDAxis]) * (ZDAXISDIR);

  // using button A, B, X w.r.t xbox360
  if ((msg->buttons[0] == 1) && (msg->buttons[1] == 0))
  {
    // using joystick control input
    bCurrUseJoyConLoop_ = true;
    bCurrUseExtGuidLoop_ = false;    
  }
  else if ((msg->buttons[0] == 0) && (msg->buttons[1] == 1))
  {
    // using guidance control input
    bCurrUseJoyConLoop_ = false;
    bCurrUseExtGuidLoop_ = true;  
  }
  else
  {
    // staying the flag type
    bCurrUseJoyConLoop_ = bPrevUseJoyConLoop_;
    bCurrUseExtGuidLoop_ = bPrevUseExtGuidLoop_;
    
    // off the external input, just hovering
    if (msg->buttons[2] == 1)
    {
      bCurrUseJoyConLoop_ = false;
      bCurrUseExtGuidLoop_ = false;        
    }    
  }
  
  // saving the previous data
  bPrevUseJoyConLoop_ = bCurrUseJoyConLoop_;
  bPrevUseExtGuidLoop_ = bCurrUseExtGuidLoop_;
}

void JoyTrjVelCntl::CbPoseInfo(const geometry_msgs::PoseConstPtr& msg) 
{
  // enu data
  mavPos_(0) = msg->position.x;
  mavPos_(1) = msg->position.y;
  mavPos_(2) = msg->position.z;  

  // attitude data, 3-2-1 Euler angle
  Quaterniond qAtt;
  Vector3d eulerAng;
  qAtt.x() = msg->orientation.x;
  qAtt.y() = msg->orientation.y;
  qAtt.z() = msg->orientation.z;
  qAtt.w() = msg->orientation.w;
  eulerAng = CalcYPREulerAngFromQuaternion(qAtt);
  mavAtt_(0) = wrap_d(eulerAng(0));
  mavAtt_(1) = wrap_d(eulerAng(1));
  mavAtt_(2) = wrap_d(eulerAng(2));
}

// converting the Euler angle(3-2-1, ZYX, YPR) [rad] to the quaternion
Quaterniond JoyTrjVelCntl::CalcQuaternionFromYPREulerAng(Vector3d euler)
{
  tf2::Matrix3x3 matQuat;
  tf2::Quaternion quat;
  Quaterniond result;
  matQuat.setEulerYPR(euler(2), euler(1), euler(0));
  matQuat.getRotation(quat);
  result.vec().x() = (double)(quat.x());
  result.vec().y() = (double)(quat.y());
  result.vec().z() = (double)(quat.z());
  result.w() = (double)(quat.w());
  return result;
}

// converting the quaternion to the Euler angle(3-2-1, ZYX, YPR) [rad]
Vector3d JoyTrjVelCntl::CalcYPREulerAngFromQuaternion(Quaterniond q)
{
  tf2::Quaternion quat(q.x(), q.y(), q.z(), q.w());
  tf2::Matrix3x3 matQuat(quat);
  Vector3d result;
  double dYaw, dPitch, dRoll = 0.0;
  matQuat.getEulerYPR(dYaw, dPitch, dRoll);
  result(0) = wrap_d(dRoll);
  result(1) = wrap_d(dPitch);
  result(2) = wrap_d(dYaw);
  return result;
}

// calculating DCM, from NED to Body, using Euler angle (3->2->1)
// only 3-2-1 convention
// [          cy*cz,          cy*sz,            -sy]
// [ sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx]
// [ sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx]  
Matrix3d JoyTrjVelCntl::CalcDcmNtoB(Vector3d eulerAtt)
{
  Matrix3d result;
  double cx = cos(eulerAtt(0));
  double cy = cos(eulerAtt(1));
  double cz = cos(eulerAtt(2));
  double sx = sin(eulerAtt(0));  
  double sy = sin(eulerAtt(1));  
  double sz = sin(eulerAtt(2));    
  result(0, 0) = cy*cz;
  result(0, 1) = cy*sz;
  result(0, 2) = -sy;
  result(1, 0) = sy*sx*cz - sz*cx;
  result(1, 1) = sy*sx*sz + cz*cx;
  result(1, 2) = cy*sx;
  result(2, 0) = sy*cx*cz + sz*sx;
  result(2, 1) = sy*cx*sz - cz*sx;
  result(2, 2) = cy*cx;
  return result;
}

// calculating DCM, from Body to NED, using Euler angle (3->2->1)
Matrix3d JoyTrjVelCntl::CalcDcmBtoN(Vector3d eulerAtt)
{
  Matrix3d result;
  Matrix3d DcmNtoB;
  DcmNtoB = CalcDcmNtoB(eulerAtt);
  result = DcmNtoB.inverse();
  return result;
}

// wrap-up function, angle between -PI and PI
double JoyTrjVelCntl::wrap_d(double _angle)
{
  _angle = fmod(_angle, 2.0 * PI);

  if (_angle < -PI)
  {
    _angle += 2.0 * PI;
  }
  else if (_angle > PI)
  {
    _angle -= 2.0 * PI;
  }
  else
  {
    _angle = _angle;
  }

  return _angle;
}