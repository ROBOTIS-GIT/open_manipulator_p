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

#include <iostream>
#include "open_manipulator_pro_kinematics_dynamics/open_manipulator_pro_kinematics_dynamics.h"

namespace open_manipulator_pro
{

ManipulatorKinematicsDynamics::ManipulatorKinematicsDynamics()
{
}
ManipulatorKinematicsDynamics::~ManipulatorKinematicsDynamics()
{
}

ManipulatorKinematicsDynamics::ManipulatorKinematicsDynamics(TreeSelect tree)
{
  for (int id = 0; id <= ALL_JOINT_ID; id++)
    manipulator_link_data_[id] = new LinkData();

  if (tree == ARM)
  {
    manipulator_link_data_[0]->name_    = "base";
    manipulator_link_data_[0]->parent_  = -1;
    manipulator_link_data_[0]->sibling_ = -1;
    manipulator_link_data_[0]->child_   = 1;
    manipulator_link_data_[0]->mass_    = 0.0;
    manipulator_link_data_[0]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[0]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[0]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[0]->joint_limit_max_   = 100.0;
    manipulator_link_data_[0]->joint_limit_min_   = -100.0;
    manipulator_link_data_[0]->inertia_           = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    manipulator_link_data_[1]->name_    = "joint1";
    manipulator_link_data_[1]->parent_  = 0;
    manipulator_link_data_[1]->sibling_ = -1;
    manipulator_link_data_[1]->child_   = 2;
    manipulator_link_data_[1]->mass_    = 0.85644;
    manipulator_link_data_[1]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.126);
    manipulator_link_data_[1]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 0.0, 1.0);
    manipulator_link_data_[1]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[1]->joint_limit_max_   = 0.5 * M_PI;
    manipulator_link_data_[1]->joint_limit_min_   = -0.5 * M_PI;
    manipulator_link_data_[1]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[2]->name_    = "joint2";
    manipulator_link_data_[2]->parent_  = 1;
    manipulator_link_data_[2]->sibling_ = -1;
    manipulator_link_data_[2]->child_   = 3;
    manipulator_link_data_[2]->mass_    = 0.94658;
    manipulator_link_data_[2]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.06900, 0.033);
    manipulator_link_data_[2]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    manipulator_link_data_[2]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[2]->joint_limit_max_   = 0.5 * M_PI;
    manipulator_link_data_[2]->joint_limit_min_   = -0.5 * M_PI;
    manipulator_link_data_[2]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[3]->name_    = "joint3";
    manipulator_link_data_[3]->parent_  = 2;
    manipulator_link_data_[3]->sibling_ = -1;
    manipulator_link_data_[3]->child_   = 4;
    manipulator_link_data_[3]->mass_    = 1.30260;
    manipulator_link_data_[3]->relative_position_ = robotis_framework::getTransitionXYZ(0.03000, -0.01150, 0.26400);
    manipulator_link_data_[3]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    manipulator_link_data_[3]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[3]->joint_limit_max_   = 0.5 * M_PI;
    manipulator_link_data_[3]->joint_limit_min_   = -0.5 * M_PI;
    manipulator_link_data_[3]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[4]->name_    = "joint4";
    manipulator_link_data_[4]->parent_  = 3;
    manipulator_link_data_[4]->sibling_ = -1;
    manipulator_link_data_[4]->child_   = 5;
    manipulator_link_data_[4]->mass_    = 1.236;
    manipulator_link_data_[4]->relative_position_ = robotis_framework::getTransitionXYZ(0.19500, -0.05750, 0.03000);
    manipulator_link_data_[4]->joint_axis_        = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    manipulator_link_data_[4]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[4]->joint_limit_max_   = 0.5 * M_PI;
    manipulator_link_data_[4]->joint_limit_min_   = -0.5 * M_PI;
    manipulator_link_data_[4]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[5]->name_    = "joint5";
    manipulator_link_data_[5]->parent_  = 4;
    manipulator_link_data_[5]->sibling_ = -1;
    manipulator_link_data_[5]->child_   = 6;
    manipulator_link_data_[5]->mass_    = 0.491;
    manipulator_link_data_[5]->relative_position_ = robotis_framework::getTransitionXYZ(0.06300, 0.04500, 0.00000);
    manipulator_link_data_[5]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 1.0, 0.0);
    manipulator_link_data_[5]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[5]->joint_limit_max_   = 0.5 * M_PI;
    manipulator_link_data_[5]->joint_limit_min_   = -0.5 * M_PI;
    manipulator_link_data_[5]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[6]->name_    = "joint6";
    manipulator_link_data_[6]->parent_  = 5;
    manipulator_link_data_[6]->sibling_ = -1;
    manipulator_link_data_[6]->child_   = 7;
    manipulator_link_data_[6]->mass_    = 0.454;
    manipulator_link_data_[6]->relative_position_ = robotis_framework::getTransitionXYZ(0.12300, -0.04500, 0.00000);
    manipulator_link_data_[6]->joint_axis_        = robotis_framework::getTransitionXYZ(1.0, 0.0, 0.0);
    manipulator_link_data_[6]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[6]->joint_limit_max_   = 0.5 * M_PI;
    manipulator_link_data_[6]->joint_limit_min_   = -0.5 * M_PI;
    manipulator_link_data_[6]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[7]->name_    = "end";
    manipulator_link_data_[7]->parent_  = 6;
    manipulator_link_data_[7]->sibling_ = -1;
    manipulator_link_data_[7]->child_   = -1;
    manipulator_link_data_[7]->mass_    = 0.0;
    manipulator_link_data_[7]->relative_position_ = robotis_framework::getTransitionXYZ(0.0115, 0.0, 0.0);
    manipulator_link_data_[7]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[7]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[7]->joint_limit_max_   = 100.0;
    manipulator_link_data_[7]->joint_limit_min_   = -100.0;
    manipulator_link_data_[7]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);
  }
}

std::vector<int> ManipulatorKinematicsDynamics::findRoute(int to)
{
  int id = manipulator_link_data_[to]->parent_;

  std::vector<int> idx;

  if (id == 0)
  {
    idx.push_back(0);
    idx.push_back(to);
  }
  else
  {
    idx = findRoute(id);
    idx.push_back(to);
  }

  return idx;
}

std::vector<int> ManipulatorKinematicsDynamics::findRoute(int from, int to)
{
  int id = manipulator_link_data_[to]->parent_;

  std::vector<int> idx;

  if (id == from)
  {
    idx.push_back(from);
    idx.push_back(to);
  }
  else if (id != 0)
  {
    idx = findRoute(from, id);
    idx.push_back(to);
  }

  return idx;
}

double ManipulatorKinematicsDynamics::totalMass(int joint_ID)
{
  double mass;

  if (joint_ID == -1)
    mass = 0.0;
  else
    mass = manipulator_link_data_[joint_ID]->mass_ + totalMass(manipulator_link_data_[joint_ID]->sibling_)
                                                   + totalMass(manipulator_link_data_[joint_ID]->child_);

  return mass;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcMC(int joint_ID)
{
  Eigen::MatrixXd mc(3, 1);

  if (joint_ID == -1)
    mc = Eigen::MatrixXd::Zero(3, 1);
  else
  {
    mc = manipulator_link_data_[joint_ID]->mass_ * (manipulator_link_data_[joint_ID]->orientation_
        * manipulator_link_data_[joint_ID]->center_of_mass_ + manipulator_link_data_[joint_ID]->position_);
    mc = mc + calcMC(manipulator_link_data_[joint_ID]->sibling_) + calcMC(manipulator_link_data_[joint_ID]->child_);
  }

  return mc;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcCOM(Eigen::MatrixXd MC)
{
  double mass;
  Eigen::MatrixXd COM(3, 1);

  mass = totalMass(0);

  COM = MC / mass;

  return COM;
}

void ManipulatorKinematicsDynamics::forwardKinematics(int joint_ID)
{
  if (joint_ID == -1)
    return;

  manipulator_link_data_[0]->position_ = Eigen::MatrixXd::Zero(3, 1);
  manipulator_link_data_[0]->orientation_ = robotis_framework::calcRodrigues(
                                                robotis_framework::calcHatto(manipulator_link_data_[0]->joint_axis_),
                                                manipulator_link_data_[0]->joint_angle_
                                            );

  if (joint_ID != 0)
  {
    int parent = manipulator_link_data_[joint_ID]->parent_;

    manipulator_link_data_[joint_ID]->position_ = manipulator_link_data_[parent]->orientation_
                                                  * manipulator_link_data_[joint_ID]->relative_position_
                                                  + manipulator_link_data_[parent]->position_;
    manipulator_link_data_[joint_ID]->orientation_ = manipulator_link_data_[parent]->orientation_
                                                    * robotis_framework::calcRodrigues(robotis_framework::calcHatto(manipulator_link_data_[joint_ID]->joint_axis_),
                                                                                       manipulator_link_data_[joint_ID]->joint_angle_);

    manipulator_link_data_[joint_ID]->transformation_.block<3, 1>(0, 3) = manipulator_link_data_[joint_ID]->position_;
    manipulator_link_data_[joint_ID]->transformation_.block<3, 3>(0, 0) = manipulator_link_data_[joint_ID]->orientation_;
  }

  forwardKinematics(manipulator_link_data_[joint_ID]->sibling_);
  forwardKinematics(manipulator_link_data_[joint_ID]->child_);
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcJacobian(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size - 1;

  Eigen::MatrixXd tar_position = manipulator_link_data_[idx[end]]->position_;
  Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(6, idx_size);

  for (int index = 0; index < idx_size; index++)
  {
    int id = idx[index];

    Eigen::MatrixXd tar_orientation = manipulator_link_data_[id]->orientation_ * manipulator_link_data_[id]->joint_axis_;

    Jacobian.block(0, index, 3, 1) = robotis_framework::calcCross(tar_orientation,
                                                                  tar_position - manipulator_link_data_[id]->position_);
    Jacobian.block(3, index, 3, 1) = tar_orientation;
  }

  return Jacobian;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcJacobianCOM(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size - 1;

  Eigen::MatrixXd target_position = manipulator_link_data_[idx[end]]->position_;
  Eigen::MatrixXd jacobianCOM = Eigen::MatrixXd::Zero(6, idx_size);

  for (int index = 0; index < idx_size; index++)
  {
    int     id    = idx[index];
    double  mass  = totalMass(id);

    Eigen::MatrixXd og = calcMC(id) / mass - manipulator_link_data_[id]->position_;
    Eigen::MatrixXd target_orientation = manipulator_link_data_[id]->orientation_ * manipulator_link_data_[id]->joint_axis_;

    jacobianCOM.block(0, index, 3, 1) = robotis_framework::calcCross(target_orientation, og);
    jacobianCOM.block(3, index, 3, 1) = target_orientation;
  }

  return jacobianCOM;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                                                         Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation)
{
  Eigen::MatrixXd pos_err = tar_position - curr_position;
  Eigen::MatrixXd ori_err1 = curr_orientation.inverse() * tar_orientation;
  Eigen::MatrixXd ori_err2 = curr_orientation * robotis_framework::convertRotToOmega(ori_err1);

  Eigen::MatrixXd err = Eigen::MatrixXd::Zero(6, 1);
  err.block(0, 0, 3, 1) = pos_err;
  err.block(3, 0, 3, 1) = ori_err2;

  return err;
}

bool ManipulatorKinematicsDynamics::inverseKinematics(int to, Eigen::MatrixXd tar_position,
    Eigen::MatrixXd tar_orientation, int max_iter, double ik_err)
{
  bool ik_success     = false;
  bool limit_success  = false;

  forwardKinematics(0);

  std::vector<int> idx = findRoute(to);

  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position     = manipulator_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation  = manipulator_link_data_[to]->orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
      ik_success = false;

    Eigen::MatrixXd jacobian2 = jacobian * jacobian.transpose();
    Eigen::MatrixXd jacobian3 = jacobian.transpose() * jacobian2.inverse();

    Eigen::MatrixXd _delta_angle = jacobian3 * err;

    for (int id = 0; id < idx.size(); id++)
    {
      int joint_num = idx[id];
      manipulator_link_data_[joint_num]->joint_angle_ += _delta_angle.coeff(id);
    }

    forwardKinematics(0);
  }

  for (int id = 0; id < idx.size(); id++)
  {
    int joint_num = idx[id];

    if (manipulator_link_data_[joint_num]->joint_angle_ >= manipulator_link_data_[joint_num]->joint_limit_max_)
    {
      limit_success = false;
      break;
    }
    else if (manipulator_link_data_[joint_num]->joint_angle_ <= manipulator_link_data_[joint_num]->joint_limit_min_)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

bool ManipulatorKinematicsDynamics::inverseKinematics(int from, int to, Eigen::MatrixXd tar_position,
                                                      Eigen::MatrixXd tar_orientation, int max_iter, double ik_err)
{
  bool ik_success = false;
  bool limit_success = false;

  forwardKinematics(0);

  std::vector<int> idx = findRoute(from, to);

  for (int iter = 0; iter < max_iter; iter++)
  {
    Eigen::MatrixXd jacobian = calcJacobian(idx);

    Eigen::MatrixXd curr_position     = manipulator_link_data_[to]->position_;
    Eigen::MatrixXd curr_orientation  = manipulator_link_data_[to]->orientation_;

    Eigen::MatrixXd err = calcVWerr(tar_position, curr_position, tar_orientation, curr_orientation);

    if (err.norm() < ik_err)
    {
      ik_success = true;
      break;
    }
    else
    {
      ik_success = false;
    }

    Eigen::MatrixXd jacobian2 = jacobian * jacobian.transpose();
    Eigen::MatrixXd jacobian3 = jacobian.transpose() * jacobian2.inverse();

    Eigen::MatrixXd delta_angle = jacobian3 * err;

    for (int id = 0; id < idx.size(); id++)
    {
      int joint_num = idx[id];
      manipulator_link_data_[joint_num]->joint_angle_ += delta_angle.coeff(id);
    }

    forwardKinematics(0);
  }

  for (int id = 0; id < idx.size(); id++)
  {
    int joint_num = idx[id];

    if (manipulator_link_data_[joint_num]->joint_angle_ >= manipulator_link_data_[joint_num]->joint_limit_max_)
    {
      limit_success = false;
      break;
    }
    else if (manipulator_link_data_[joint_num]->joint_angle_ <= manipulator_link_data_[joint_num]->joint_limit_min_)
    {
      limit_success = false;
      break;
    }
    else
      limit_success = true;
  }

  if (ik_success == true && limit_success == true)
    return true;
  else
    return false;
}

}
