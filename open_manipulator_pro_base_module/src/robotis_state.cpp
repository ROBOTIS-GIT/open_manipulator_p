/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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
*******************************************************************************/

#include "open_manipulator_pro_base_module/robotis_state.h"

using namespace open_manipulator_pro;

RobotisState::RobotisState()
{
  is_moving_ = false;

  cnt_      = 0;
  mov_time_ = 1.0;
  smp_time_ = 0.008;
  all_time_steps_ = int(mov_time_ / smp_time_) + 1;

  calc_joint_tra_ = Eigen::MatrixXd::Zero(all_time_steps_, MAX_JOINT_ID + 1);
  calc_task_tra_  = Eigen::MatrixXd::Zero(all_time_steps_, 3);

  joint_ini_pose_ = Eigen::MatrixXd::Zero( MAX_JOINT_ID + 1, 1);

  // for inverse kinematics;
  ik_solve_ = false;

  ik_target_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);

  ik_start_rotation_  = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);
  ik_target_rotation_ = robotis_framework::convertRPYToRotation(0.0, 0.0, 0.0);

  ik_id_start_  = 0;
  ik_id_end_    = 0;
}

RobotisState::~RobotisState()
{
}

void RobotisState::setInverseKinematics(int cnt, Eigen::MatrixXd start_rotation)
{
  for (int dim = 0; dim < 3; dim++)
    ik_target_position_.coeffRef(dim, 0) = calc_task_tra_.coeff(cnt, dim);

  Eigen::Quaterniond start_quaternion = robotis_framework::convertRotationToQuaternion(start_rotation);

  Eigen::Quaterniond target_quaternion(kinematics_pose_msg_.pose.orientation.w,
                                       kinematics_pose_msg_.pose.orientation.x,
                                       kinematics_pose_msg_.pose.orientation.y,
                                       kinematics_pose_msg_.pose.orientation.z);

  double count = (double) cnt / (double) all_time_steps_;

  Eigen::Quaterniond quaternion = start_quaternion.slerp(count, target_quaternion);

  ik_target_rotation_ = robotis_framework::convertQuaternionToRotation(quaternion);
}

