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

#ifndef MANIPULATOR_BASE_MODULE_ROBOTIS_STATE_H_
#define MANIPULATOR_BASE_MODULE_ROBOTIS_STATE_H_

#include <eigen3/Eigen/Eigen>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "robotis_math/robotis_math.h"

#include "open_manipulator_pro_base_module_msgs/JointPose.h"
#include "open_manipulator_pro_base_module_msgs/KinematicsPose.h"
#include "open_manipulator_pro_kinematics_dynamics/open_manipulator_pro_kinematics_dynamics.h"

namespace open_manipulator_pro
{

class RobotisState
{
public:
  RobotisState();
  ~RobotisState();

  bool    is_moving_;

  // trajectory
  int     cnt_;
  int     all_time_steps_;
  double  mov_time_;
  double  smp_time_;

  Eigen::MatrixXd calc_joint_tra_;
  Eigen::MatrixXd calc_task_tra_;

  Eigen::MatrixXd joint_ini_pose_;

  // msgs
  open_manipulator_pro_base_module_msgs::JointPose joint_pose_msg_;
  open_manipulator_pro_base_module_msgs::KinematicsPose kinematics_pose_msg_;

  // inverse kinematics
  bool ik_solve_;
  Eigen::MatrixXd ik_target_position_;
  Eigen::MatrixXd ik_start_rotation_, ik_target_rotation_;
  int ik_id_start_, ik_id_end_;

  void setInverseKinematics(int cnt, Eigen::MatrixXd start_rotation);
};

}

#endif /* MANIPULATOR_BASE_MODULE_ROBOTIS_STATE_H_ */
