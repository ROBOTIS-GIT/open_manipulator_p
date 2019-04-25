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

#ifndef MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_H_
#define MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_H_

#include <vector>

#include "link_data.h"
#include "open_manipulator_pro_kinematics_dynamics_define.h"

namespace open_manipulator_pro
{

enum TreeSelect
{
  ARM
};

class ManipulatorKinematicsDynamics
{
public:
  ManipulatorKinematicsDynamics();
  ManipulatorKinematicsDynamics(TreeSelect tree);
  ~ManipulatorKinematicsDynamics();

  LinkData *manipulator_link_data_[ ALL_JOINT_ID + 1];

  std::vector<int> findRoute(int to);
  std::vector<int> findRoute(int from, int to);

  double totalMass(int joint_ID);
  Eigen::MatrixXd calcMC(int joint_ID);
  Eigen::MatrixXd calcCOM(Eigen::MatrixXd MC);

  void forwardKinematics(int joint_ID);

  Eigen::MatrixXd calcJacobian(std::vector<int> idx);
  Eigen::MatrixXd calcJacobianCOM(std::vector<int> idx);
  Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                            Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);

  bool inverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                         int max_iter, double ik_err);
  bool inverseKinematics(int from, int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                         int max_iter, double ik_err);
};

}

#endif /* MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_H_ */
