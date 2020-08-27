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
  pubNedPosEulerAttInfo_ = nh_.advertise<rotors_control::NedPosEulerAtt>("/firefly/odometry_sensor1/pos_ned_att_euler", 1);

  // initializing joystick pose information
  for (unsigned int i = 0; i < 4; i++)
    joyPose_(i) = 0.0;

  // initializing mav pose information
  for (unsigned int i = 0; i < 3; i++)
  {
    mavPosEnu_(i) = 0.0;
    mavPosNed_(i) = 0.0;
    mavEulerAtt_(i) = 0.0;
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
    GenExtGuideConInfo();
    return;
  }

  ROS_INFO_DELAYED_THROTTLE(10, "waiting controller setup(joystick, external guidance loop)...");
  return;
}

void JoyTrjVelCntl::GenExtGuideConInfo()
{
  ROS_INFO_DELAYED_THROTTLE(10, "external guidance control input loop...");
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
  DcmNtoB = CalcDcmNtoB(mavEulerAtt_);
  DcmBtoN = CalcDcmBtoN(mavEulerAtt_);
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
  // converting enu data to ned data
  mavPosEnu_(0) = msg->position.x;
  mavPosEnu_(1) = msg->position.y;
  mavPosEnu_(2) = msg->position.z;
  mavPosNed_ = ConvertPosFromEnuToNed(mavPosEnu_);    

  // attitude data, 3-2-1 Euler angle
  Quaterniond qAtt;
  Vector3d eulerAng;
  qAtt.x() = msg->orientation.x;
  qAtt.y() = msg->orientation.y;
  qAtt.z() = msg->orientation.z;
  qAtt.w() = msg->orientation.w;
  eulerAng = CalcYPREulerAngFromQuaternion(qAtt);
  mavEulerAtt_(0) = wrap_d(eulerAng(0));
  mavEulerAtt_(1) = wrap_d(eulerAng(1));
  mavEulerAtt_(2) = wrap_d(eulerAng(2));

  // publishing the result, domain: aerospace control side
  rotors_control::NedPosEulerAtt msgNedPosEulerAtt;
  msgNedPosEulerAtt.rosTime = (double)(ros::Time::now().toSec());
  msgNedPosEulerAtt.Xn = mavPosNed_(0);
  msgNedPosEulerAtt.Ye = mavPosNed_(1);
  msgNedPosEulerAtt.Zd = mavPosNed_(2);
  msgNedPosEulerAtt.rollPhi = mavEulerAtt_(0);
  msgNedPosEulerAtt.pitchTheta = mavEulerAtt_(1);
  msgNedPosEulerAtt.yawPsi = ((-1.0)*(mavEulerAtt_(2))) + ((0.5) * (PI));
  pubNedPosEulerAttInfo_.publish(msgNedPosEulerAtt);
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

// calculating DCM, Euler angle, 321 conversion (ref:from NED to Body, using Euler angle (3->2->1))
// only 3-2-1 convention
// [          cy*cz,          cy*sz,            -sy]
// [ sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx]
// [ sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx]
Matrix3d JoyTrjVelCntl::CalcDcmEuler321(Vector3d eulerAtt)
{
  return CalcDcmNtoB(eulerAtt);
}

// converting from the position w.r.t ENU frame to the position w.r.t NED frame
Vector3d JoyTrjVelCntl::ConvertPosFromEnuToNed(Vector3d posEnu)
{
  // tested(ok)
  Vector3d result;
  Vector3d attForEnuToNed;
  Matrix3d dcmForEnuToNed;
  attForEnuToNed(0) = (-180.0) * (D2R);
  attForEnuToNed(1) = (0.0) * (D2R);
  attForEnuToNed(2) = (90.0) * (D2R);
  dcmForEnuToNed = CalcDcmEuler321(attForEnuToNed);
  result = (dcmForEnuToNed) * (posEnu);
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