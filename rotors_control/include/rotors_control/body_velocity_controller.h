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
#ifndef ROTORS_CONTROL_BODY_VELOCITY_CONTROLLER_H
#define ROTORS_CONTROL_BODY_VELOCITY_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

namespace rotors_control {

// Default values for the lee position controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultPGain = Eigen::Vector3d(1.0, 1.0, 1.0);
static const Eigen::Vector3d kDefaultIGain = Eigen::Vector3d(0.5, 0.5, 0.5);
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 3, 0.035);
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.025);

class BodyVelocityControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  BodyVelocityControllerParameters()
      : p_gain_(kDefaultPGain),
        i_gain_(kDefaultIGain),
        attitude_gain_(kDefaultAttitudeGain),
        angular_rate_gain_(kDefaultAngularRateGain) {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  Eigen::Matrix4Xd allocation_matrix_;
  Eigen::Vector3d p_gain_;
  Eigen::Vector3d i_gain_;
  Eigen::Vector3d attitude_gain_;
  Eigen::Vector3d angular_rate_gain_;
  RotorConfiguration rotor_configuration_;
};

class BodyVelocityController {
 public:
  BodyVelocityController();
  ~BodyVelocityController();
  void InitializeParameters();

  BodyVelocityControllerParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
};
}

#endif // ROTORS_CONTROL_BODY_VELOCITY_CONTROLLER_H
