/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "../include/open_manipulator_p_libs/kinematics.h"

using namespace robotis_manipulator;
using namespace kinematics;


/*****************************************************************************
** Kinematics Solver Using Chain Rule and Jacobian
*****************************************************************************/
void SolverUsingCRAndJacobian::setOption(const void *arg){}

Eigen::MatrixXd SolverUsingCRAndJacobian::jacobian(Manipulator *manipulator, Name tool_name)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

  Eigen::Vector3d joint_axis = Eigen::Vector3d::Zero(3);

  Eigen::Vector3d position_changed = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d orientation_changed = Eigen::Vector3d::Zero(3);
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

  //////////////////////////////////////////////////////////////////////////////////

  int8_t index = 0;
  Name my_name =  manipulator->getWorldChildName();

  for (int8_t size = 0; size < manipulator->getDOF(); size++)
  {
    Name parent_name = manipulator->getComponentParentName(my_name);
    if (parent_name == manipulator->getWorldName())
    {
      joint_axis = manipulator->getWorldOrientation() * manipulator->getAxis(my_name);
    }
    else
    {
      joint_axis = manipulator->getComponentOrientationFromWorld(parent_name) * manipulator->getAxis(my_name);
    }

    position_changed = math::skewSymmetricMatrix(joint_axis) *
                       (manipulator->getComponentPositionFromWorld(tool_name) - manipulator->getComponentPositionFromWorld(my_name));
    orientation_changed = joint_axis;

    pose_changed << position_changed(0),
        position_changed(1),
        position_changed(2),
        orientation_changed(0),
        orientation_changed(1),
        orientation_changed(2);

    jacobian.col(index) = pose_changed;
    index++;
    my_name = manipulator->getComponentChildName(my_name).at(0); // Get Child name which has active joint
  }
  return jacobian;
}

void SolverUsingCRAndJacobian::solveForwardKinematics(Manipulator *manipulator)
{
  forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

bool SolverUsingCRAndJacobian::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  return inverseSolverUsingJacobian(manipulator, tool_name, target_pose, goal_joint_value);
}


//private
void SolverUsingCRAndJacobian::forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = manipulator->getComponentParentName(my_name);
  int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  Pose parent_pose_value;
  Pose my_pose_value;

  //Get Parent Pose
  if (parent_name == manipulator->getWorldName())
  {
    parent_pose_value = manipulator->getWorldPose();
  }
  else
  {
    parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
  }

  //position
  my_pose_value.kinematic.position = parent_pose_value.kinematic.position
                                   + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));
  //orientation
  my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * manipulator->getComponentRelativeOrientationFromParent(my_name) * math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name)); 
  //linear velocity
  my_pose_value.dynamic.linear.velocity = math::vector3(0.0, 0.0, 0.0);
  //angular velocity
  my_pose_value.dynamic.angular.velocity = math::vector3(0.0, 0.0, 0.0);
  //linear acceleration
  my_pose_value.dynamic.linear.acceleration = math::vector3(0.0, 0.0, 0.0);
  //angular acceleration
  my_pose_value.dynamic.angular.acceleration = math::vector3(0.0, 0.0, 0.0);

  manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = manipulator->getComponentChildName(my_name).at(index);
    forwardSolverUsingChainRule(manipulator, child_name);
  }
}

bool SolverUsingCRAndJacobian::inverseSolverUsingJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  const double lambda = 0.7;
  const int8_t iteration = 10;

  Manipulator _manipulator = *manipulator;

  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, _manipulator.getDOF());

  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd delta_angle = Eigen::VectorXd::Zero(_manipulator.getDOF());

  for (int8_t count = 0; count < iteration; count++)
  { 
    // Forward kinematics solve
    solveForwardKinematics(&_manipulator);
    // Get jacobian
    jacobian = this->jacobian(&_manipulator, tool_name);

    // Pose Difference
    pose_changed = math::poseDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name),
                                           target_pose.kinematic.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));

    // Pose sovler success
    if (pose_changed.norm() < 1E-6)
    {
      *goal_joint_value = _manipulator.getAllActiveJointValue();
      for(int8_t index = 0; index < _manipulator.getDOF(); index++)
      {
        goal_joint_value->at(index).velocity = 0.0;
        goal_joint_value->at(index).acceleration = 0.0;
        goal_joint_value->at(index).effort = 0.0;
      }
      return true;
    }

    // Get delta angle
    ColPivHouseholderQR<MatrixXd> dec(jacobian);
    delta_angle = lambda * dec.solve(pose_changed);

    // Set changed angle
    std::vector<double> changed_angle;
    for(int8_t index = 0; index < _manipulator.getDOF(); index++)
      changed_angle.push_back(_manipulator.getAllActiveJointPosition().at(index) + delta_angle(index));
    _manipulator.setAllActiveJointPosition(changed_angle);
  }
  *goal_joint_value = {};
  return false;
}


/*****************************************************************************
** Kinematics Solver Using Chain Rule and Singularity Robust Jacobian
*****************************************************************************/
void SolverUsingCRAndSRJacobian::setOption(const void *arg){}

Eigen::MatrixXd SolverUsingCRAndSRJacobian::jacobian(Manipulator *manipulator, Name tool_name)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

  Eigen::Vector3d joint_axis = Eigen::Vector3d::Zero(3);

  Eigen::Vector3d position_changed = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d orientation_changed = Eigen::Vector3d::Zero(3);
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

  //////////////////////////////////////////////////////////////////////////////////

  int8_t index = 0;
  Name my_name =  manipulator->getWorldChildName();

  for (int8_t size = 0; size < manipulator->getDOF(); size++)
  {
    Name parent_name = manipulator->getComponentParentName(my_name);
    if (parent_name == manipulator->getWorldName())
    {
      joint_axis = manipulator->getWorldOrientation() * manipulator->getAxis(my_name);
    }
    else
    {
      joint_axis = manipulator->getComponentOrientationFromWorld(parent_name) * manipulator->getAxis(my_name);
    }

    position_changed = math::skewSymmetricMatrix(joint_axis) *
                       (manipulator->getComponentPositionFromWorld(tool_name) - manipulator->getComponentPositionFromWorld(my_name));
    orientation_changed = joint_axis;

    pose_changed << position_changed(0),
        position_changed(1),
        position_changed(2),
        orientation_changed(0),
        orientation_changed(1),
        orientation_changed(2);

    jacobian.col(index) = pose_changed;
    index++;
    my_name = manipulator->getComponentChildName(my_name).at(0); // Get Child name which has active joint
  }
  return jacobian;
}

void SolverUsingCRAndSRJacobian::solveForwardKinematics(Manipulator *manipulator)
{
  forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

bool SolverUsingCRAndSRJacobian::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  return inverseSolverUsingSRJacobian(manipulator, tool_name, target_pose, goal_joint_value);
}


// Private
void SolverUsingCRAndSRJacobian::forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = manipulator->getComponentParentName(my_name);
  int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  Pose parent_pose_value;
  Pose my_pose_value;

  // Get Parent Pose
  if (parent_name == manipulator->getWorldName())
  {
    parent_pose_value = manipulator->getWorldPose();
  }
  else
  {
    parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
  }

  // Position
  my_pose_value.kinematic.position = parent_pose_value.kinematic.position
                                   + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));
  // Orientation
  my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * manipulator->getComponentRelativeOrientationFromParent(my_name) * math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name)); 
  // Linear velocity
  my_pose_value.dynamic.linear.velocity = math::vector3(0.0, 0.0, 0.0);
  // Angular velocity
  my_pose_value.dynamic.angular.velocity = math::vector3(0.0, 0.0, 0.0);
  // Linear acceleration
  my_pose_value.dynamic.linear.acceleration = math::vector3(0.0, 0.0, 0.0);
  // Angular acceleration
  my_pose_value.dynamic.angular.acceleration = math::vector3(0.0, 0.0, 0.0);

  manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = manipulator->getComponentChildName(my_name).at(index);
    forwardSolverUsingChainRule(manipulator, child_name);
  }
}

bool SolverUsingCRAndSRJacobian::inverseSolverUsingSRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  //manipulator
  Manipulator _manipulator = *manipulator;

  //solver parameter
  double lambda = 0.0;
  const double param = 0.002;
  const int8_t iteration = 5;

  const double gamma = 0.5;             //rollback delta

  //sr sovler parameter
  double wn_pos = 1 / 0.3;
  double wn_ang = 1 / (2 * M_PI);
  double pre_Ek = 0.0;
  double new_Ek = 0.0;

  Eigen::MatrixXd We(6, 6);
  We << wn_pos, 0, 0, 0, 0, 0,
      0, wn_pos, 0, 0, 0, 0,
      0, 0, wn_pos, 0, 0, 0,
      0, 0, 0, wn_ang, 0, 0,
      0, 0, 0, 0, wn_ang, 0,
      0, 0, 0, 0, 0, wn_ang;

  Eigen::MatrixXd Wn = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //jacobian
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, _manipulator.getDOF());
  Eigen::MatrixXd sr_jacobian = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //delta parameter
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd angle_changed = Eigen::VectorXd::Zero(_manipulator.getDOF());    //delta angle (dq)
  Eigen::VectorXd gerr(_manipulator.getDOF());

  //angle parameter
  std::vector<double> present_angle;                                               //angle (q)
  std::vector<double> set_angle;                                                   //set angle (q + dq)

  ////////////////////////////solving//////////////////////////////////

  solveForwardKinematics(&_manipulator);
  //////////////checking dx///////////////
  pose_changed = math::poseDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name), target_pose.kinematic.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));
  pre_Ek = pose_changed.transpose() * We * pose_changed;
  ///////////////////////////////////////

  /////////////////////////////debug/////////////////////////////////
  #if defined(KINEMATICS_DEBUG)
  Eigen::Vector3d target_orientation_rpy = math::convertRotationToRPY(target_pose.orientation);
  Eigen::VectorXd debug_target_pose(6);
  for(int t=0; t<3; t++)
    debug_target_pose(t) = target_pose.position(t);
  for(int t=0; t<3; t++)
    debug_target_pose(t+3) = target_orientation_rpy(t);

  Eigen::Vector3d present_position = _manipulator.getComponentPositionFromWorld(tool_name);
  Eigen::MatrixXd present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
  Eigen::Vector3d present_orientation_rpy = math::convertRotationToRPY(present_orientation);
  Eigen::VectorXd debug_present_pose(6);
  for(int t=0; t<3; t++)
    debug_present_pose(t) = present_position(t);
  for(int t=0; t<3; t++)
    debug_present_pose(t+3) = present_orientation_rpy(t);

  log::println("------------------------------------");
  log::warn("iter : first");
  log::warn("Ek : ", pre_Ek*1000000000000);
  log::println("tar_pose");
  log::println_VECTOR(debug_target_pose,16);
  log::println("pre_pose");
  log::println_VECTOR(debug_present_pose,16);
  log::println("delta_pose");
  log::println_VECTOR(debug_target_pose-debug_present_pose,16);
  #endif
  ////////////////////////////debug//////////////////////////////////

  //////////////////////////solving loop///////////////////////////////
  for (int8_t count = 0; count < iteration; count++)
  {
    //////////solve using jacobian//////////
    jacobian = this->jacobian(&_manipulator, tool_name);
    lambda = pre_Ek + param;

    sr_jacobian = (jacobian.transpose() * We * jacobian) + (lambda * Wn);     //calculate sr_jacobian (J^T*we*J + lamda*Wn)
    gerr = jacobian.transpose() * We * pose_changed;                          //calculate gerr (J^T*we) dx

    ColPivHouseholderQR<Eigen::MatrixXd> dec(sr_jacobian);                    //solving (get dq)
    angle_changed = dec.solve(gerr);                                          //(J^T*we) * dx = (J^T*we*J + lamda*Wn) * dq

    present_angle = _manipulator.getAllActiveJointPosition();
    set_angle.clear();
    for (int8_t index = 0; index < _manipulator.getDOF(); index++)
      set_angle.push_back(present_angle.at(index) + angle_changed(index));
    _manipulator.setAllActiveJointPosition(set_angle);
    solveForwardKinematics(&_manipulator);
    ////////////////////////////////////////

    //////////////checking dx///////////////
    pose_changed = math::poseDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name), target_pose.kinematic.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));
    new_Ek = pose_changed.transpose() * We * pose_changed;
    ////////////////////////////////////////

    /////////////////////////////debug/////////////////////////////////
    #if defined(KINEMATICS_DEBUG)
    present_position = _manipulator.getComponentPositionFromWorld(tool_name);
    present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
    present_orientation_rpy = math::convertRotationToRPY(present_orientation);
    for(int t=0; t<3; t++)
      debug_present_pose(t) = present_position(t);
    for(int t=0; t<3; t++)
      debug_present_pose(t+3) = present_orientation_rpy(t);
    log::warn("iter : ", count,0);
    log::warn("Ek : ", new_Ek*1000000000000);
    log::println("tar_pose");
    log::println_VECTOR(debug_target_pose,16);
    log::println("pre_pose");
    log::println_VECTOR(debug_present_pose,16);
    log::println("delta_pose");
    log::println_VECTOR(debug_target_pose-debug_present_pose,16);
    #endif
    ////////////////////////////debug//////////////////////////////////

    if (new_Ek < 1E-12)
    {
      /////////////////////////////debug/////////////////////////////////
      #if defined(KINEMATICS_DEBUG)
      log::warn("iter : ", count,0);
      log::warn("Ek : ", new_Ek*1000000000000);
      log::error("Success");
      log::println("------------------------------------");
      #endif
      //////////////////////////debug//////////////////////////////////
      *goal_joint_value = _manipulator.getAllActiveJointValue();
      for(int8_t index = 0; index < _manipulator.getDOF(); index++)
      {
        goal_joint_value->at(index).velocity = 0.0;
        goal_joint_value->at(index).acceleration = 0.0;
        goal_joint_value->at(index).effort = 0.0;
      }
      return true;
    }
    else if (new_Ek < pre_Ek)
    {
      pre_Ek = new_Ek;
    }
    else
    {
      present_angle = _manipulator.getAllActiveJointPosition();
      for (int8_t index = 0; index < _manipulator.getDOF(); index++)
        set_angle.push_back(present_angle.at(index) - (gamma * angle_changed(index)));
      _manipulator.setAllActiveJointPosition(set_angle);

      solveForwardKinematics(&_manipulator);
    }
  }
  log::error("[sr]fail to solve inverse kinematics (please change the solver)");
  *goal_joint_value = {};
  return false;
}


/*****************************************************************************
** Kinematics Solver Using Chain Rule and Singularity Robust Position Only Jacobian
*****************************************************************************/
void SolverUsingCRAndSRPositionOnlyJacobian::setOption(const void *arg){}

Eigen::MatrixXd SolverUsingCRAndSRPositionOnlyJacobian::jacobian(Manipulator *manipulator, Name tool_name)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

  Eigen::Vector3d joint_axis = Eigen::Vector3d::Zero(3);

  Eigen::Vector3d position_changed = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d orientation_changed = Eigen::Vector3d::Zero(3);
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

  //////////////////////////////////////////////////////////////////////////////////

  int8_t index = 0;
  Name my_name =  manipulator->getWorldChildName();

  for (int8_t size = 0; size < manipulator->getDOF(); size++)
  {
    Name parent_name = manipulator->getComponentParentName(my_name);
    if (parent_name == manipulator->getWorldName())
    {
      joint_axis = manipulator->getWorldOrientation() * manipulator->getAxis(my_name);
    }
    else
    {
      joint_axis = manipulator->getComponentOrientationFromWorld(parent_name) * manipulator->getAxis(my_name);
    }

    position_changed = math::skewSymmetricMatrix(joint_axis) *
                       (manipulator->getComponentPositionFromWorld(tool_name) - manipulator->getComponentPositionFromWorld(my_name));
    orientation_changed = joint_axis;

    pose_changed << position_changed(0),
        position_changed(1),
        position_changed(2),
        orientation_changed(0),
        orientation_changed(1),
        orientation_changed(2);

    jacobian.col(index) = pose_changed;
    index++;
    my_name = manipulator->getComponentChildName(my_name).at(0); // Get Child name which has active joint
  }
  return jacobian;
}

void SolverUsingCRAndSRPositionOnlyJacobian::solveForwardKinematics(Manipulator *manipulator)
{
  forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

bool SolverUsingCRAndSRPositionOnlyJacobian::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  return inverseSolverUsingPositionOnlySRJacobian(manipulator, tool_name, target_pose, goal_joint_value);
}


//private
void SolverUsingCRAndSRPositionOnlyJacobian::forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = manipulator->getComponentParentName(my_name);
  int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  Pose parent_pose_value;
  Pose my_pose_value;

  //Get Parent Pose
  if (parent_name == manipulator->getWorldName())
  {
    parent_pose_value = manipulator->getWorldPose();
  }
  else
  {
    parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
  }

  //position
  my_pose_value.kinematic.position = parent_pose_value.kinematic.position
                                   + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));
  //orientation
  my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * manipulator->getComponentRelativeOrientationFromParent(my_name) * math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name)); 
  //linear velocity
  my_pose_value.dynamic.linear.velocity = math::vector3(0.0, 0.0, 0.0);
  //angular velocity
  my_pose_value.dynamic.angular.velocity = math::vector3(0.0, 0.0, 0.0);
  //linear acceleration
  my_pose_value.dynamic.linear.acceleration = math::vector3(0.0, 0.0, 0.0);
  //angular acceleration
  my_pose_value.dynamic.angular.acceleration = math::vector3(0.0, 0.0, 0.0);

  manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = manipulator->getComponentChildName(my_name).at(index);
    forwardSolverUsingChainRule(manipulator, child_name);
  }
}

bool SolverUsingCRAndSRPositionOnlyJacobian::inverseSolverUsingPositionOnlySRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  //manipulator
  Manipulator _manipulator = *manipulator;

  //solver parameter
  double lambda = 0.0;
  const double param = 0.002;
  const int8_t iteration = 10;

  const double gamma = 0.5;             //rollback delta

  //jacobian
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, _manipulator.getDOF());
  Eigen::MatrixXd position_jacobian = Eigen::MatrixXd::Identity(3, _manipulator.getDOF());
  Eigen::MatrixXd sr_jacobian = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //delta parameter
  Eigen::Vector3d position_changed = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd angle_changed = Eigen::VectorXd::Zero(_manipulator.getDOF());    //delta angle (dq)
  Eigen::VectorXd gerr(_manipulator.getDOF());

  //sr sovler parameter
  double wn_pos = 1 / 0.3;
  double pre_Ek = 0.0;
  double new_Ek = 0.0;

  Eigen::MatrixXd We(3, 3);
  We << wn_pos, 0, 0,
      0, wn_pos, 0,
      0, 0, wn_pos;

  Eigen::MatrixXd Wn = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //angle parameter
  std::vector<double> present_angle;                                               //angle (q)
  std::vector<double> set_angle;                                                   //set angle (q + dq)

  ////////////////////////////solving//////////////////////////////////

  solveForwardKinematics(&_manipulator);
  //////////////checking dx///////////////
  position_changed = math::positionDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name));
  pre_Ek = position_changed.transpose() * We * position_changed;
  ///////////////////////////////////////

  /////////////////////////////debug/////////////////////////////////
  #if defined(KINEMATICS_DEBUG)
  Eigen::Vector3d target_orientation_rpy = math::convertRotationToRPY(target_pose.orientation);
  Eigen::VectorXd debug_target_pose(6);
  for(int t=0; t<3; t++)
    debug_target_pose(t) = target_pose.position(t);
  for(int t=0; t<3; t++)
    debug_target_pose(t+3) = target_orientation_rpy(t);

  Eigen::Vector3d present_position = _manipulator.getComponentPositionFromWorld(tool_name);
  Eigen::MatrixXd present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
  Eigen::Vector3d present_orientation_rpy = math::convertRotationToRPY(present_orientation);
  Eigen::VectorXd debug_present_pose(6);
  for(int t=0; t<3; t++)
    debug_present_pose(t) = present_position(t);
  for(int t=0; t<3; t++)
    debug_present_pose(t+3) = present_orientation_rpy(t);

  log::println("------------------------------------");
  log::warn("iter : first");
  log::warn("Ek : ", pre_Ek*1000000000000);
  log::println("tar_pose");
  log::println_VECTOR(debug_target_pose,16);
  log::println("pre_pose");
  log::println_VECTOR(debug_present_pose,16);
  log::println("delta_pose");
  log::println_VECTOR(debug_target_pose-debug_present_pose,16);
  #endif
  ////////////////////////////debug//////////////////////////////////

  //////////////////////////solving loop///////////////////////////////
  for (int8_t count = 0; count < iteration; count++)
  {
    //////////solve using jacobian//////////
    jacobian = this->jacobian(&_manipulator, tool_name);
    position_jacobian.row(0) = jacobian.row(0);
    position_jacobian.row(1) = jacobian.row(1);
    position_jacobian.row(2) = jacobian.row(2);
    lambda = pre_Ek + param;

    sr_jacobian = (position_jacobian.transpose() * We * position_jacobian) + (lambda * Wn);     //calculate sr_jacobian (J^T*we*J + lamda*Wn)
    gerr = position_jacobian.transpose() * We * position_changed;                                //calculate gerr (J^T*we) dx

    ColPivHouseholderQR<Eigen::MatrixXd> dec(sr_jacobian);                    //solving (get dq)
    angle_changed = dec.solve(gerr);                                          //(J^T*we) * dx = (J^T*we*J + lamda*Wn) * dq

    present_angle = _manipulator.getAllActiveJointPosition();
    set_angle.clear();
    for (int8_t index = 0; index < _manipulator.getDOF(); index++)
      set_angle.push_back(_manipulator.getAllActiveJointPosition().at(index) + angle_changed(index));
    _manipulator.setAllActiveJointPosition(set_angle);
    solveForwardKinematics(&_manipulator);
    ////////////////////////////////////////

    //////////////checking dx///////////////
    position_changed = math::positionDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name));
    new_Ek = position_changed.transpose() * We * position_changed;
    ////////////////////////////////////////

    /////////////////////////////debug/////////////////////////////////
    #if defined(KINEMATICS_DEBUG)
    present_position = _manipulator.getComponentPositionFromWorld(tool_name);
    present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
    present_orientation_rpy = math::convertRotationToRPY(present_orientation);
    for(int t=0; t<3; t++)
      debug_present_pose(t) = present_position(t);
    for(int t=0; t<3; t++)
      debug_present_pose(t+3) = present_orientation_rpy(t);
    log::warn("iter : ", count,0);
    log::warn("Ek : ", new_Ek*1000000000000);
    log::println("tar_pose");
    log::println_VECTOR(debug_target_pose,16);
    log::println("pre_pose");
    log::println_VECTOR(debug_present_pose,16);
    log::println("delta_pose");
    log::println_VECTOR(debug_target_pose-debug_present_pose,16);
    #endif
    ////////////////////////////debug//////////////////////////////////

    if (new_Ek < 1E-12)
    {
      /////////////////////////////debug/////////////////////////////////
      #if defined(KINEMATICS_DEBUG)
      log::warn("iter : ", count,0);
      log::warn("Ek : ", new_Ek*1000000000000);
      log::error("IK Success");
      log::println("------------------------------------");
      #endif
      //////////////////////////debug//////////////////////////////////
      *goal_joint_value = _manipulator.getAllActiveJointValue();
      for(int8_t index = 0; index < _manipulator.getDOF(); index++)
      {
        goal_joint_value->at(index).velocity = 0.0;
        goal_joint_value->at(index).acceleration = 0.0;
        goal_joint_value->at(index).effort = 0.0;
      }
      return true;
    }
    else if (new_Ek < pre_Ek)
    {
      pre_Ek = new_Ek;
    }
    else
    {
      present_angle = _manipulator.getAllActiveJointPosition();
      for (int8_t index = 0; index < _manipulator.getDOF(); index++)
        set_angle.push_back(_manipulator.getAllActiveJointPosition().at(index) - (gamma * angle_changed(index)));
      _manipulator.setAllActiveJointPosition(set_angle);

      solveForwardKinematics(&_manipulator);
    }
  }
  log::error("[position_only]fail to solve inverse kinematics (please change the solver)");
  *goal_joint_value = {};
  return false;
}


/*****************************************************************************
** Kinematics Solver Customized for OpenManipulator Chain
*****************************************************************************/
void SolverCustomizedforOMChain::setOption(const void *arg){}

Eigen::MatrixXd SolverCustomizedforOMChain::jacobian(Manipulator *manipulator, Name tool_name)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());

  Eigen::Vector3d joint_axis = Eigen::Vector3d::Zero(3);

  Eigen::Vector3d position_changed = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d orientation_changed = Eigen::Vector3d::Zero(3);
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

  //////////////////////////////////////////////////////////////////////////////////

  int8_t index = 0;
  Name my_name =  manipulator->getWorldChildName();

  for (int8_t size = 0; size < manipulator->getDOF(); size++)
  {
    Name parent_name = manipulator->getComponentParentName(my_name);
    if (parent_name == manipulator->getWorldName())
    {
      joint_axis = manipulator->getWorldOrientation() * manipulator->getAxis(my_name);
    }
    else
    {
      joint_axis = manipulator->getComponentOrientationFromWorld(parent_name) * manipulator->getAxis(my_name);
    }

    position_changed = math::skewSymmetricMatrix(joint_axis) *
                       (manipulator->getComponentPositionFromWorld(tool_name) - manipulator->getComponentPositionFromWorld(my_name));
    orientation_changed = joint_axis;

    pose_changed << position_changed(0),
        position_changed(1),
        position_changed(2),
        orientation_changed(0),
        orientation_changed(1),
        orientation_changed(2);

    jacobian.col(index) = pose_changed;
    index++;
    my_name = manipulator->getComponentChildName(my_name).at(0); // Get Child name which has active joint
  }
  return jacobian;
}

void SolverCustomizedforOMChain::solveForwardKinematics(Manipulator *manipulator)
{
  forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

bool SolverCustomizedforOMChain::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  return chainCustomInverseKinematics(manipulator, tool_name, target_pose, goal_joint_value);
}


//private
void SolverCustomizedforOMChain::forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = manipulator->getComponentParentName(my_name);
  int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  Pose parent_pose_value;
  Pose my_pose_value;

  //Get Parent Pose
  if (parent_name == manipulator->getWorldName())
  {
    parent_pose_value = manipulator->getWorldPose();
  }
  else
  {
    parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
  }

  //position
  my_pose_value.kinematic.position = parent_pose_value.kinematic.position
                                   + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));
  //orientation
  my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * manipulator->getComponentRelativeOrientationFromParent(my_name) * math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name)); 
  //linear velocity
  my_pose_value.dynamic.linear.velocity = math::vector3(0.0, 0.0, 0.0);
  //angular velocity
  my_pose_value.dynamic.angular.velocity = math::vector3(0.0, 0.0, 0.0);
  //linear acceleration
  my_pose_value.dynamic.linear.acceleration = math::vector3(0.0, 0.0, 0.0);
  //angular acceleration
  my_pose_value.dynamic.angular.acceleration = math::vector3(0.0, 0.0, 0.0);

  manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = manipulator->getComponentChildName(my_name).at(index);
    forwardSolverUsingChainRule(manipulator, child_name);
  }
}

bool SolverCustomizedforOMChain::chainCustomInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  //manipulator
  Manipulator _manipulator = *manipulator;

  //solver parameter
  double lambda = 0.0;
  const double param = 0.002;
  const int8_t iteration = 10;

  const double gamma = 0.5;             //rollback delta

  //sr sovler parameter
  double wn_pos = 1 / 0.3;
  double wn_ang = 1 / (2 * M_PI);
  double pre_Ek = 0.0;
  double new_Ek = 0.0;

  Eigen::MatrixXd We(6, 6);
  We << wn_pos, 0, 0, 0, 0, 0,
      0, wn_pos, 0, 0, 0, 0,
      0, 0, wn_pos, 0, 0, 0,
      0, 0, 0, wn_ang, 0, 0,
      0, 0, 0, 0, wn_ang, 0,
      0, 0, 0, 0, 0, wn_ang;

  Eigen::MatrixXd Wn = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //jacobian
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, _manipulator.getDOF());
  Eigen::MatrixXd sr_jacobian = Eigen::MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  //delta parameter
  Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd angle_changed = Eigen::VectorXd::Zero(_manipulator.getDOF());    //delta angle (dq)
  Eigen::VectorXd gerr(_manipulator.getDOF());

  //angle parameter
  std::vector<double> present_angle;                                               //angle (q)
  std::vector<double> set_angle;                                                   //set angle (q + dq)

  ////////////////////////////solving//////////////////////////////////

  solveForwardKinematics(&_manipulator);

  //////////////make target ori//////////  //only OpenManipulator Chain
  Eigen::Matrix3d present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
  Eigen::Vector3d present_orientation_rpy = math::convertRotationMatrixToRPYVector(present_orientation);
  Eigen::Matrix3d target_orientation = target_pose.kinematic.orientation;
  Eigen::Vector3d target_orientation_rpy = math::convertRotationMatrixToRPYVector(target_orientation);

  Eigen::Vector3d joint1_rlative_position = _manipulator.getComponentRelativePositionFromParent(_manipulator.getWorldChildName());
  Eigen::Vector3d target_position_from_joint1 = target_pose.kinematic.position - joint1_rlative_position;

  target_orientation_rpy(0) = present_orientation_rpy(0);
  target_orientation_rpy(1) = target_orientation_rpy(1);
  target_orientation_rpy(2) = atan2(target_position_from_joint1(1) ,target_position_from_joint1(0));

  target_pose.kinematic.orientation = math::convertRPYToRotationMatrix(target_orientation_rpy(0), target_orientation_rpy(1), target_orientation_rpy(2));
  ///////////////////////////////////////

  //////////////checking dx///////////////
  pose_changed = math::poseDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name), target_pose.kinematic.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));
  pre_Ek = pose_changed.transpose() * We * pose_changed;
  ///////////////////////////////////////

  /////////////////////////////debug/////////////////////////////////
  #if defined(KINEMATICS_DEBUG)
  Eigen::VectorXd debug_target_pose(6);
  for(int t=0; t<3; t++)
    debug_target_pose(t) = target_pose.position(t);
  for(int t=0; t<3; t++)
    debug_target_pose(t+3) = target_orientation_rpy(t);

  Eigen::Vector3d present_position = _manipulator.getComponentPositionFromWorld(tool_name);
  Eigen::VectorXd debug_present_pose(6);
  for(int t=0; t<3; t++)
    debug_present_pose(t) = present_position(t);
  for(int t=0; t<3; t++)
    debug_present_pose(t+3) = present_orientation_rpy(t);

  log::println("------------------------------------");
  log::warn("iter : first");
  log::warn("Ek : ", pre_Ek*1000000000000);
  log::println("tar_pose");
  log::println_VECTOR(debug_target_pose,16);
  log::println("pre_pose");
  log::println_VECTOR(debug_present_pose,16);
  log::println("delta_pose");
  log::println_VECTOR(debug_target_pose-debug_present_pose,16);
  #endif
  ////////////////////////////debug//////////////////////////////////

  //////////////////////////solving loop///////////////////////////////
  for (int8_t count = 0; count < iteration; count++)
  {
    //////////solve using jacobian//////////
    jacobian = this->jacobian(&_manipulator, tool_name);
    lambda = pre_Ek + param;

    sr_jacobian = (jacobian.transpose() * We * jacobian) + (lambda * Wn);     //calculate sr_jacobian (J^T*we*J + lamda*Wn)
    gerr = jacobian.transpose() * We * pose_changed;                          //calculate gerr (J^T*we) dx

    ColPivHouseholderQR<Eigen::MatrixXd> dec(sr_jacobian);                    //solving (get dq)
    angle_changed = dec.solve(gerr);                                          //(J^T*we) * dx = (J^T*we*J + lamda*Wn) * dq

    present_angle = _manipulator.getAllActiveJointPosition();
    set_angle.clear();
    for (int8_t index = 0; index < _manipulator.getDOF(); index++)
      set_angle.push_back(present_angle.at(index) + angle_changed(index));
    _manipulator.setAllActiveJointPosition(set_angle);
    solveForwardKinematics(&_manipulator);
    ////////////////////////////////////////

    //////////////checking dx///////////////
    pose_changed = math::poseDifference(target_pose.kinematic.position, _manipulator.getComponentPositionFromWorld(tool_name), target_pose.kinematic.orientation, _manipulator.getComponentOrientationFromWorld(tool_name));
    new_Ek = pose_changed.transpose() * We * pose_changed;
    ////////////////////////////////////////

    /////////////////////////////debug/////////////////////////////////
    #if defined(KINEMATICS_DEBUG)
    present_position = _manipulator.getComponentPositionFromWorld(tool_name);
    present_orientation = _manipulator.getComponentOrientationFromWorld(tool_name);
    present_orientation_rpy = math::convertRotationToRPY(present_orientation);
    for(int t=0; t<3; t++)
      debug_present_pose(t) = present_position(t);
    for(int t=0; t<3; t++)
      debug_present_pose(t+3) = present_orientation_rpy(t);
    log::warn("iter : ", count,0);
    log::warn("Ek : ", new_Ek*1000000000000);
    log::println("tar_pose");
    log::println_VECTOR(debug_target_pose,16);
    log::println("pre_pose");
    log::println_VECTOR(debug_present_pose,16);
    log::println("delta_pose");
    log::println_VECTOR(debug_target_pose-debug_present_pose,16);
    #endif
    ////////////////////////////debug//////////////////////////////////

    if (new_Ek < 1E-12)
    {
      /////////////////////////////debug/////////////////////////////////
      #if defined(KINEMATICS_DEBUG)
      log::warn("iter : ", count,0);
      log::warn("Ek : ", new_Ek*1000000000000);
      log::error("Success");
      log::println("------------------------------------");
      #endif
      //////////////////////////debug//////////////////////////////////
      *goal_joint_value = _manipulator.getAllActiveJointValue();
      for(int8_t index = 0; index < _manipulator.getDOF(); index++)
      {
        goal_joint_value->at(index).velocity = 0.0;
        goal_joint_value->at(index).acceleration = 0.0;
        goal_joint_value->at(index).effort = 0.0;
      }
      return true;
    }
    else if (new_Ek < pre_Ek)
    {
      pre_Ek = new_Ek;
    }
    else
    {
      present_angle = _manipulator.getAllActiveJointPosition();
      for (int8_t index = 0; index < _manipulator.getDOF(); index++)
        set_angle.push_back(present_angle.at(index) - (gamma * angle_changed(index)));
      _manipulator.setAllActiveJointPosition(set_angle);

      solveForwardKinematics(&_manipulator);
    }
  }
  log::error("[OpenManipulator Chain Custom]fail to solve inverse kinematics");
  *goal_joint_value = {};
  return false;
}


/*****************************************************************************
** Kinematics Solver Using Geometry Approach
*****************************************************************************/
void SolverUsingCRAndGeometry::setOption(const void *arg)
{
  with_gripper_ = arg;
}

Eigen::MatrixXd SolverUsingCRAndGeometry::jacobian(Manipulator *manipulator, Name tool_name)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, manipulator->getDOF());
  return jacobian;
}

void SolverUsingCRAndGeometry::solveForwardKinematics(Manipulator *manipulator)
{
  forwardSolverUsingChainRule(manipulator, manipulator->getWorldChildName());
}

bool SolverUsingCRAndGeometry::solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  return inverseSolverUsingGeometry(manipulator, tool_name, target_pose, goal_joint_value);
}


//private
void SolverUsingCRAndGeometry::forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name)
{
  Name my_name = component_name;
  Name parent_name = manipulator->getComponentParentName(my_name);
  int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  Pose parent_pose_value;
  Pose my_pose_value;

  //Get Parent Pose
  if (parent_name == manipulator->getWorldName())
  {
    parent_pose_value = manipulator->getWorldPose();
  }
  else
  {
    parent_pose_value = manipulator->getComponentPoseFromWorld(parent_name);
  }

  //position
  my_pose_value.kinematic.position = parent_pose_value.kinematic.position
                                   + (parent_pose_value.kinematic.orientation * manipulator->getComponentRelativePositionFromParent(my_name));
  //orientation
  my_pose_value.kinematic.orientation = parent_pose_value.kinematic.orientation * manipulator->getComponentRelativeOrientationFromParent(my_name) * math::rodriguesRotationMatrix(manipulator->getAxis(my_name), manipulator->getJointPosition(my_name)); 
  //linear velocity
  my_pose_value.dynamic.linear.velocity = math::vector3(0.0, 0.0, 0.0);
  //angular velocity
  my_pose_value.dynamic.angular.velocity = math::vector3(0.0, 0.0, 0.0);
  //linear acceleration
  my_pose_value.dynamic.linear.acceleration = math::vector3(0.0, 0.0, 0.0);
  //angular acceleration
  my_pose_value.dynamic.angular.acceleration = math::vector3(0.0, 0.0, 0.0);

  manipulator->setComponentPoseFromWorld(my_name, my_pose_value);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = manipulator->getComponentChildName(my_name).at(index);
    forwardSolverUsingChainRule(manipulator, child_name);
  }
}

bool SolverUsingCRAndGeometry::inverseSolverUsingGeometry(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue> *goal_joint_value)
{
  Manipulator _manipulator = *manipulator;
  JointValue target_angle[6];
  std::vector<JointValue> target_angle_vector;

  //// Position
  // Compute Joint 1 Angle
  Eigen::VectorXd position = Eigen::VectorXd::Zero(3);
  Eigen::MatrixXd orientation = Eigen::MatrixXd::Zero(3,3);
  position = target_pose.kinematic.position;
  orientation = target_pose.kinematic.orientation;
  double d6 = 0.123;

  if (with_gripper_)
  {
    auto tool_length = _manipulator.getComponentRelativePositionFromParent(tool_name);
    d6 += tool_length(0);
  }
  Eigen::Vector3d position_2 = Eigen::VectorXd::Zero(3);
  position_2 << orientation(0,0), orientation(1,0), orientation(2,0);
  Eigen::Vector3d position_3 = Eigen::VectorXd::Zero(3);
  position_3 = position - d6*position_2;
  if (position_3(0) == 0) position_3(0) = 0.001;
  target_angle[0].position = atan(position_3(1) / position_3(0));

  // Compute Joint 3 Angle
  Eigen::VectorXd position3 = Eigen::VectorXd::Zero(3); 
  Eigen::VectorXd position3_2 = Eigen::VectorXd::Zero(3); 
  position3 << 0.0, 0.0, 0.126;
  position3_2 << 0.0, 0.0, 0.033;
  Eigen::MatrixXd orientation3 = math::convertRPYToRotationMatrix(0,0,target_angle[0].position);
  Eigen::VectorXd position3_3 = Eigen::VectorXd::Zero(3); 
  position3_3 = position3 + orientation3 * position3_2;
  Eigen::VectorXd position3_4 = Eigen::VectorXd::Zero(3); 
  position3_4 = position_3 - position3_3;
  double l1 = sqrt(0.264*0.264 + 0.030*0.030);
  double l2 = sqrt(0.030*0.030 + 0.258*0.258);
  double phi = acos((l1*l1 + l2*l2 - position3_4.norm()*position3_4.norm()) / (2*l1*l2));
  double alpha1 = atan2(0.030, 0.264);
  double alpha2 = atan2(0.258, 0.030);
  target_angle[2].position = PI - (phi-alpha1) - alpha2;

  // Compute Joint 2 Angle
  Eigen::VectorXd position2 = Eigen::VectorXd::Zero(3); 
  Eigen::MatrixXd orientation2 = math::convertRPYToRotationMatrix(0,0,target_angle[0].position);
  position2 = orientation2.inverse() * position3_4;
  double beta1 = atan2(position2(2), position2(0));
  double beta2 = acos((l1*l1 + position3_4.norm()*position3_4.norm() - l2*l2) / (2*l1*position3_4.norm()));
  if (position3_4(2) > 0) target_angle[1].position = (PI/2-alpha1) - fabs(beta1) - beta2;
  else target_angle[1].position = (PI/2-alpha1) + fabs(beta1) - beta2;

  //// Orientation
  // Compute Joint 4, 5, 6 Angles
  Eigen::MatrixXd orientation_to_joint3 = math::convertRPYToRotationMatrix(0,0,target_angle[0].position)
                                        * math::convertRPYToRotationMatrix(0,target_angle[1].position,0)
                                        * math::convertRPYToRotationMatrix(0,target_angle[2].position,0);
  Eigen::MatrixXd orientation_def = orientation_to_joint3.transpose() * orientation;

  if(orientation_def(0,0) <  1.0)
  {
    if(orientation_def(0,0) >  -1.0)
    { 
      double joint4_angle_present = _manipulator.getJointPosition("joint4");
      double joint4_angle_temp_1 = atan2(orientation_def(1,0), -orientation_def(2,0));
      double joint4_angle_temp_2 = atan2(-orientation_def(1,0), orientation_def(2,0));

      if(fabs(joint4_angle_temp_1-joint4_angle_present) < fabs(joint4_angle_temp_2-joint4_angle_present))
      {
        // log::println("joint4_angle_temp_1", fabs(joint4_angle_present-joint4_angle_temp_1));
        target_angle[3].position = joint4_angle_temp_1;
        target_angle[4].position = acos(orientation_def(0,0));
        target_angle[5].position = atan2(orientation_def(0,1), orientation_def(0,2));
      }
      else
      {
        // log::println("joint4_angle_temp_2", fabs(joint4_angle_present-joint4_angle_temp_2));
        target_angle[3].position = joint4_angle_temp_2;
        target_angle[4].position = -acos(orientation_def(0,0));
        target_angle[5].position = atan2(-orientation_def(0,1), -orientation_def(0,2));
      }
    }
    else        // R(0,0) = -1
    {
      target_angle[3].position = _manipulator.getJointPosition("joint4");
      target_angle[4].position = PI;
      target_angle[5].position = atan2(-orientation_def(1,2), orientation_def(1,1))+target_angle[3].position;
    }
  }
  else          // R(0,0) = 1
  {
    target_angle[3].position = _manipulator.getJointPosition("joint4");
    target_angle[4].position = 0.0;
    target_angle[5].position = atan2(-orientation_def(1,2), orientation_def(1,1))-target_angle[3].position;
  }  
  
  // log::println("------------------------------------");
  // log::println("End-effector Pose : ");
  // log::println("position1: ", target_angle[0].position);
  // log::println("position2: ", target_angle[1].position);
  // log::println("position3: ", target_angle[2].position);
  // log::println("position5: ", target_angle[4].position);
  // log::println("position4: ", target_angle[3].position);
  // log::println("position6: ", target_angle[5].position);
  // log::println("------------------------------------");

  if(std::isnan(target_angle[0].position) ||
    std::isnan(target_angle[1].position) ||
    std::isnan(target_angle[2].position) ||
    std::isnan(target_angle[3].position) ||
    std::isnan(target_angle[4].position) ||
    std::isnan(target_angle[5].position) )
  {
    log::error("Target angle value is NAN!!");
  return false;
  }


  if(std::isinf(target_angle[0].position) ||
    std::isinf(target_angle[1].position) ||
    std::isinf(target_angle[2].position) ||
    std::isinf(target_angle[3].position) ||
    std::isinf(target_angle[4].position) ||
    std::isinf(target_angle[5].position) )
  {
    log::error("Target angle value is INF!!");
  return false;
  }

  target_angle_vector.push_back(target_angle[0]);
  target_angle_vector.push_back(target_angle[1]);
  target_angle_vector.push_back(target_angle[2]);
  target_angle_vector.push_back(target_angle[3]);
  target_angle_vector.push_back(target_angle[4]);
  target_angle_vector.push_back(target_angle[5]);

  *goal_joint_value = target_angle_vector;

  return true;
}
