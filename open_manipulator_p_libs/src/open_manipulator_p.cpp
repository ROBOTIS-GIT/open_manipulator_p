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

#include "../include/open_manipulator_p_libs/open_manipulator_p.h"

OpenManipulator::OpenManipulator() {}

OpenManipulator::~OpenManipulator()
{
  delete kinematics_;
  delete actuator_;
  delete tool_;
  for(uint8_t index = 0; index < CUSTOM_TRAJECTORY_SIZE; index++)
    delete custom_trajectory_[index];
}

void OpenManipulator::initOpenManipulator(bool using_actual_robot_state, STRING usb_port, STRING baud_rate, float control_loop_time, bool with_gripper)
{
  /*****************************************************************************
  ** Initialize Manipulator Parameter
  *****************************************************************************/
  addWorld("world",    // world name
           "joint1");  // child name

  addJoint("joint1",   // my name
           "world",    // parent name
           "joint2",   // child name
            math::vector3(0.0, 0.0, 0.126),                  // relative position
            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
            Z_AXIS,    // axis of rotation
            1,         // actuator id
            M_PI,      // max joint limit (3.14 rad)
            -M_PI,     // min joint limit (-3.14 rad)
            1.0,       // coefficient
            9.8406837e-02,                                                        // mass
            math::inertiaMatrix(3.4543422e-05, -1.6031095e-08, -3.8375155e-07,
                                3.2689329e-05, 2.8511935e-08,
                                1.8850320e-05),                                   // inertial tensor
            math::vector3(-3.0184870e-04, 5.4043684e-04, 0.018 + 2.9433464e-02)   // COM
            );

  addJoint("joint2",   // my name
            "joint1",  // parent name
            "joint3",  // child name
            math::vector3(0.0, 0.0, 0.033),                  // relative position
            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
            Y_AXIS,    // axis of rotation
            2,         // actuator id
            M_PI,      // max joint limit (3.14 rad)
            -M_PI,     // min joint limit (-3.14 rad)
            1.0,       // coefficient
            1.3850917e-01,                                                        // mass
            math::inertiaMatrix(3.3055381e-04, 9.7940978e-08, -3.8505711e-05,
                                3.4290447e-04, -1.5717516e-06,
                                6.0346498e-05),                                   // inertial tensor
            math::vector3(1.0308393e-02, 3.7743363e-04, 1.0170197e-01)            // COM
            );

  addJoint("joint3",   // my name
            "joint2",  // parent name
            "joint4",  // child name
            math::vector3(0.030, 0.0, 0.264),                // relative position
            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
            Y_AXIS,    // axis of rotation
            3,         // actuator id
            M_PI,      // max joint limit (3.14 rad)
            -M_PI,     // min joint limit (-3.14 rad)
            1.0,       // coefficient
            1.3274562e-01,                                                        // mass
            math::inertiaMatrix(3.0654178e-05, -1.2764155e-06, -2.6874417e-07,
                                2.4230292e-04, 1.1559550e-08,
                                2.5155057e-04),                                   // inertial tensor
            math::vector3(9.0909590e-02, 3.8929816e-04, 2.2413279e-04)            // COM
            );

  addJoint("joint4",   // my name
            "joint3",  // parent name
            "joint5",  // child name
            math::vector3(0.195, 0.0, 0.030),                // relative position
            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
            X_AXIS,    // axis of rotation
            4,         // actuator id
            M_PI,      // max joint limit (3.14 rad)
            -M_PI,     // min joint limit (-3.14 rad)
            1.0,       // coefficient
            1.4327573e-01,                                                        // mass
            math::inertiaMatrix(8.0870749e-05, 0.0, -1.0157896e-06,
                                7.5980465e-05, 0.0,
                                9.3127351e-05),                                   // inertial tensor
            math::vector3(4.4206755e-02, 3.6839985e-07, 8.9142216e-03)            // COM
            );

  addJoint("joint5",   // my name
            "joint4",  // parent name
            "joint6",  // child name
            math::vector3(0.063, 0.0, 0.0),                  // relative position
            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
            Y_AXIS,    // axis of rotation
            5,         // actuator id
            M_PI,      // max joint limit (3.14 rad)
            -M_PI,     // min joint limit (-3.14 rad)
            1.0,       // coefficient
            1.4327573e-01,                                                        // mass
            math::inertiaMatrix(8.0870749e-05, 0.0, -1.0157896e-06,
                                7.5980465e-05, 0.0,
                                9.3127351e-05),                                   // inertial tensor
            math::vector3(4.4206755e-02, 3.6839985e-07, 8.9142216e-03)            // COM
            );

  addJoint("joint6",   // my name
            "joint5",  // parent name
            "gripper", // child name
            math::vector3(0.123, 0.0, 0.0),                  // relative position
            math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
            X_AXIS,    // axis of rotation
            6,         // actuator id
            M_PI,      // max joint limit (3.14 rad)
            -M_PI,     // min joint limit (-3.14 rad)
            1.0,       // coefficient
            1.4327573e-01,                                                        // mass
            math::inertiaMatrix(8.0870749e-05, 0.0, -1.0157896e-06,
                                7.5980465e-05, 0.0,
                                9.3127351e-05),                                   // inertial tensor
            math::vector3(4.4206755e-02, 3.6839985e-07, 8.9142216e-03)            // COM
            );

  int gripper_id = -1;
  double gripper_len = 0.0;
  if (with_gripper)
  {
    gripper_id = 7;
    gripper_len = 0.1223;
  }
  addTool("gripper",  // my name
          "joint6",   // parent name
          math::vector3(gripper_len, 0.0, 0.0),               // relative position
          // math::vector3(0.150, 0.0, 0.0),                  // relative position
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
          gripper_id, // actuator id
          1.1351,     // max gripper limit (0.01 m)
          -0.001,     // min gripper limit (-0.01 m)
          1.0,        // Change unit from `meter` to `radian`
          3.2218127e-02 * 2,                                                    // mass
          math::inertiaMatrix(9.5568826e-06, 2.8424644e-06, -3.2829197e-10,
                              2.2552871e-05, -3.1463634e-10,
                              1.7605306e-05),                                   // inertial tensor
          math::vector3(0.028 + 8.3720668e-03, 0.0246, -4.2836895e-07)          // COM
          );

  /*****************************************************************************
  ** Initialize Kinematics 
  *****************************************************************************/
  kinematics_ = new kinematics::SolverUsingCRAndGeometry();

  addKinematics(kinematics_);

  // Set kinematics arguments
  void *p_with_gripper = &with_gripper;
  setKinematicsOption(p_with_gripper);

  if (using_actual_robot_state)
  {
    /*****************************************************************************
    ** Initialize Joint Actuator
    *****************************************************************************/
    actuator_ = new dynamixel::JointDynamixel();
    // actuator_ = new dynamixel::JointDynamixelProfileControl(control_loop_time);

    // Set communication arguments
    STRING dxl_comm_arg[2] = {usb_port, baud_rate};
    void *p_dxl_comm_arg = &dxl_comm_arg;

    // Set joint actuator id
    std::vector<uint8_t> jointDxlId;
    jointDxlId.push_back(1);
    jointDxlId.push_back(2);
    jointDxlId.push_back(3);
    jointDxlId.push_back(4);
    jointDxlId.push_back(5);
    jointDxlId.push_back(6);
    addJointActuator(JOINT_DYNAMIXEL, actuator_, jointDxlId, p_dxl_comm_arg);

    // Set joint actuator control mode
    STRING joint_dxl_mode_arg = "position_mode";
    void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
    setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);

    /*****************************************************************************
    ** Initialize Tool Actuator
    *****************************************************************************/
    if (with_gripper) 
    {
      tool_ = new dynamixel::GripperDynamixel();

      uint8_t gripperDxlId = 7;
      addToolActuator(TOOL_DYNAMIXEL, tool_, gripperDxlId, p_dxl_comm_arg);

      // Set gripper actuator control mode
      STRING gripper_dxl_mode_arg = "current_based_position_mode";
      void *p_gripper_dxl_mode_arg = &gripper_dxl_mode_arg;
      setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_mode_arg);

      // Set gripper actuator parameter
      // STRING gripper_dxl_opt_arg[2];
      // void *p_gripper_dxl_opt_arg = &gripper_dxl_opt_arg;
      // gripper_dxl_opt_arg[0] = "Profile_Acceleration";
      // gripper_dxl_opt_arg[1] = "20";
      // setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

      // gripper_dxl_opt_arg[0] = "Profile_Velocity";
      // gripper_dxl_opt_arg[1] = "20";
      // setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);
    }

    // Enable All Actuators 
    enableAllActuator();

    // Receive current angles from all actuators 
    receiveAllJointActuatorValue();

    if (with_gripper) receiveAllToolActuatorValue();
  }

  /*****************************************************************************
  ** Initialize Custom Trajectory
  *****************************************************************************/
  custom_trajectory_[0] = new custom_trajectory::Line();
  custom_trajectory_[1] = new custom_trajectory::Circle();
  custom_trajectory_[2] = new custom_trajectory::Rhombus();
  custom_trajectory_[3] = new custom_trajectory::Heart();

  addCustomTrajectory(CUSTOM_TRAJECTORY_LINE, custom_trajectory_[0]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, custom_trajectory_[1]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, custom_trajectory_[2]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_HEART, custom_trajectory_[3]);
}

void OpenManipulator::processOpenManipulator(double present_time, bool using_actual_robot_state, bool with_gripper)
{
  // Planning (ik)
  JointWaypoint goal_joint_value = getJointGoalValueFromTrajectory(present_time);
  JointWaypoint goal_tool_value;
  
  static double init_tool_value = getToolGoalValue().at(0).position;
  static bool onoff = false;
  if (with_gripper) 
  {    
    // switch to on if the value changed different from the init value
    if (init_tool_value != getToolGoalValue().at(0).position)
      onoff = true;

    if (!onoff)
      goal_tool_value = getToolGoalValue();
    else
      goal_tool_value = distanceToAngle(getToolGoalValue());
  }

  // Control (motor)
  receiveAllJointActuatorValue();
  if (with_gripper) 
  {
    std::vector<Name> tool_component_name;
    tool_component_name = getManipulator()-> getAllToolComponentName();

    if (using_actual_robot_state)
    {
      getManipulator()->setJointValue(tool_component_name.at(0), 
                                      angleToDistance(receiveAllToolActuatorValue()).at(0));
    }
  }
  
  if (goal_joint_value.size() != 0) sendAllJointActuatorValue(goal_joint_value);
  if (with_gripper) {if (goal_tool_value.size() != 0) {sendAllToolActuatorValue(goal_tool_value);}}

  // Perception (fk)
  solveForwardKinematics();
}

JointWaypoint OpenManipulator::distanceToAngle(JointWaypoint distance)
{
  // distance (m) -> angle (rad) 
  double angle = distance.at(0).position; 
  // double angle = 1.135 - distance.at(0).position; // / 0.109 * 1.135;

  JointValue result;
  result.position = angle;

  JointWaypoint result_vector;
  result_vector.push_back(result);

  return result_vector;
}

JointWaypoint OpenManipulator::angleToDistance(JointWaypoint angle)
{
  // angle (rad) -> distance (m) 
  double distance = angle.at(0).position;
  // double distance = 1.135 - angle.at(0).position; /// 1.135 * 0.109;

  JointValue result;
  result.position = distance;

  JointWaypoint result_vector;
  result_vector.push_back(result);

  return result_vector;
}
