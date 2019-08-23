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

/* Authors: Ryan Shim */

#include "open_manipulator_pro_teleop/open_manipulator_pro_teleop_keyboard.hpp"

namespace open_manipulator_pro_teleop_keyboard
{
OpenManipulatorProTeleopKeyboard::OpenManipulatorProTeleopKeyboard()
: Node("open_manipulator_pro_teleop_keyboard")
{
  /************************************************************
  ** Initialise ROS Parameters
  ************************************************************/
  this->declare_parameter(
    "use_gripper",
    rclcpp::ParameterValue(false),
    rcl_interfaces::msg::ParameterDescriptor());

  this->get_parameter("use_gripper", use_gripper_);

  /********************************************************************************
  ** Initialise joint angle and kinematic position size 
  ********************************************************************************/
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  /********************************************************************************
  ** Initialise Subscribers
  ********************************************************************************/
  joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "open_manipulator_pro/joint_states", 10, std::bind(&OpenManipulatorProTeleopKeyboard::joint_states_callback, this, std::placeholders::_1));
  kinematics_pose_sub_ = this->create_subscription<open_manipulator_msgs::msg::KinematicsPose>(
    "open_manipulator_pro/kinematics_pose", 10, std::bind(&OpenManipulatorProTeleopKeyboard::kinematics_pose_callback, this, std::placeholders::_1));

  /********************************************************************************
  ** Initialise Clients
  ********************************************************************************/
  goal_joint_space_path_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("open_manipulator_pro/goal_joint_space_path");
  goal_tool_control_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("open_manipulator_pro/goal_tool_control");
  goal_task_space_path_from_present_position_only_client_ = this->create_client<open_manipulator_msgs::srv::SetKinematicsPose>("open_manipulator_pro/goal_task_space_path_from_present_position_only");
  goal_joint_space_path_from_present_client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>("open_manipulator_pro/goal_joint_space_path_from_present");

  RCLCPP_INFO(this->get_logger(), "OpenManipulator Teleop Keyboard Initialised");

  /********************************************************************************
  ** Display in terminal
  ********************************************************************************/
  printf("OpenManipulator teleoperation using keyboard start");
  this->disable_waiting_for_enter();

  auto period = std::chrono::milliseconds(10); 
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period), 
    std::bind(&OpenManipulatorProTeleopKeyboard::display_callback, this));
}

OpenManipulatorProTeleopKeyboard::~OpenManipulatorProTeleopKeyboard() 
{
  this->restore_terminal_settings();
  RCLCPP_INFO(this->get_logger(), "OpenManipulator Teleop Keyboard Terminated");
}

/********************************************************************************
** Callback Functions
********************************************************************************/
void OpenManipulatorProTeleopKeyboard::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for(std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint5"))  temp_angle.at(4) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint6"))  temp_angle.at(5) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void OpenManipulatorProTeleopKeyboard::kinematics_pose_callback(const open_manipulator_msgs::msg::KinematicsPose::SharedPtr msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

/********************************************************************************
** Callback Functions and Relevant Functions
********************************************************************************/
void OpenManipulatorProTeleopKeyboard::set_goal(char ch)
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(4, 0.0);

  if(ch == 'w' || ch == 'W')
  {
    printf("input : w \tincrease(++) x axis in task space\n");
    goalPose.at(0) = DELTA;
    set_task_space_path_from_present_position_only(goalPose, PATH_TIME);
  }
  else if(ch == 's' || ch == 'S')
  {
    printf("input : s \tdecrease(--) x axis in task space\n");
    goalPose.at(0) = -DELTA;
    set_task_space_path_from_present_position_only(goalPose, PATH_TIME);
  }
  else if(ch == 'a' || ch == 'A')
  {
    printf("input : a \tincrease(++) y axis in task space\n");
    goalPose.at(1) = DELTA;
    set_task_space_path_from_present_position_only(goalPose, PATH_TIME);
  }
  else if(ch == 'd' || ch == 'D')
  {
    printf("input : d \tdecrease(--) y axis in task space\n");
    goalPose.at(1) = -DELTA;
    set_task_space_path_from_present_position_only(goalPose, PATH_TIME);
  }
  else if(ch == 'z' || ch == 'Z')
  {
    printf("input : z \tincrease(++) z axis in task space\n");
    goalPose.at(2) = DELTA;
    set_task_space_path_from_present_position_only(goalPose, PATH_TIME);
  }
  else if(ch == 'x' || ch == 'X')
  {
    printf("input : x \tdecrease(--) z axis in task space\n");
    goalPose.at(2) = -DELTA;
    set_task_space_path_from_present_position_only(goalPose, PATH_TIME);
  }
  else if(ch == 'y' || ch == 'Y')
  {
    printf("input : y \tincrease(++) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    set_joint_space_path_from_present(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'h' || ch == 'H')
  {
    printf("input : h \tdecrease(--) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = -JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    set_joint_space_path_from_present(joint_name, goalJoint, PATH_TIME);
  }

  else if(ch == 'u' || ch == 'U')
  {
    printf("input : u \tincrease(++) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) = JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    set_joint_space_path_from_present(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'j' || ch == 'J')
  {
    printf("input : j \tdecrease(--) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) = -JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    set_joint_space_path_from_present(joint_name, goalJoint, PATH_TIME);
  }

  else if(ch == 'i' || ch == 'I')
  {
    printf("input : i \tincrease(++) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) = JOINT_DELTA;
    joint_name.push_back("joint4");
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    set_joint_space_path_from_present(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'k' || ch == 'K')
  {
    printf("input : k \tdecrease(--) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) = -JOINT_DELTA;
    joint_name.push_back("joint4");
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    set_joint_space_path_from_present(joint_name, goalJoint, PATH_TIME);
  }

  else if(ch == 'o' || ch == 'O')
  {
    printf("input : o \tincrease(++) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) = JOINT_DELTA;
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    set_joint_space_path_from_present(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'l' || ch == 'L')
  {
    printf("input : l \tdecrease(--) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) = -JOINT_DELTA;
    joint_name.push_back("joint5");
    joint_name.push_back("joint6");
    set_joint_space_path_from_present(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'v' || ch == 'V')
  {
    if (use_gripper_)
    {
      printf("input : v \topen gripper\n");
      std::vector<double> joint_angle;
      joint_angle.push_back(1.135);
      set_tool_control(joint_angle);
    }
  }
  else if(ch == 'b' || ch == 'B')
  {
    if (use_gripper_)
    {
      printf("input : b \tclose gripper\n");
      std::vector<double> joint_angle;
      joint_angle.push_back(0.0);
      set_tool_control(joint_angle);
    }
  }
  else if(ch == '2')
  {
    printf("input : 2 \thome pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-PI/4);
    joint_name.push_back("joint3"); joint_angle.push_back(PI/8);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    joint_name.push_back("joint5"); joint_angle.push_back(PI/8);
    joint_name.push_back("joint6"); joint_angle.push_back(0.0);
    set_joint_space_path(joint_name, joint_angle, path_time);
  }
  else if(ch == '1')
  {
    printf("input : 1 \tinit pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    joint_name.push_back("joint5"); joint_angle.push_back(0.0);
    joint_name.push_back("joint6"); joint_angle.push_back(0.0);
    set_joint_space_path(joint_name, joint_angle, path_time);
  }
  else if(ch == 'q')
  {
    printf("Pressed Quit \n");
    rclcpp::shutdown();
  }
  // else
  //   RCLCPP_ERROR(this->get_logger(), "Wrong Input :(");
}

bool OpenManipulatorProTeleopKeyboard::set_joint_space_path(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name = joint_name;
  request->joint_position.position = joint_angle;
  request->path_time = path_time;
  
  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto result = future.get();
      return result->is_planned;
  };
  auto future_result = goal_joint_space_path_client_->async_send_request(request, response_received_callback);

  return false;
}

bool OpenManipulatorProTeleopKeyboard::set_tool_control(std::vector<double> joint_angle)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name.push_back("gripper");
  request->joint_position.position = joint_angle;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto result = future.get();
      return result->is_planned;
  };
  auto future_result = goal_tool_control_client_->async_send_request(request, response_received_callback);

  return false;
}

bool OpenManipulatorProTeleopKeyboard::set_task_space_path_from_present_position_only(std::vector<double> kinematics_pose, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetKinematicsPose::Request>();
  request->planning_group = "gripper";
  request->kinematics_pose.pose.position.x = kinematics_pose.at(0);
  request->kinematics_pose.pose.position.y = kinematics_pose.at(1);
  request->kinematics_pose.pose.position.z = kinematics_pose.at(2);
  request->path_time = path_time;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetKinematicsPose>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto result = future.get();
      return result->is_planned;
  };
  auto future_result = goal_task_space_path_from_present_position_only_client_->async_send_request(request, response_received_callback);

  return false;
}

bool OpenManipulatorProTeleopKeyboard::set_joint_space_path_from_present(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
  request->joint_position.joint_name = joint_name;
  request->joint_position.position = joint_angle;
  request->path_time = path_time;

  using ServiceResponseFuture = rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      auto result = future.get();
      return result->is_planned;
  };
  auto future_result = goal_joint_space_path_from_present_client_->async_send_request(request, response_received_callback);

  return false;
}

/********************************************************************************
** Other Functions
********************************************************************************/
void OpenManipulatorProTeleopKeyboard::print_text()
{
  printf("\n");
  printf("---------------------------------\n");
  printf("Control Your OpenManipulator-PRO!\n");
  printf("---------------------------------\n");
  printf("w : increase x axis in task space\n");
  printf("s : decrease x axis in task space\n");
  printf("a : increase y axis in task space\n");
  printf("d : decrease y axis in task space\n");
  printf("z : increase z axis in task space\n");
  printf("x : decrease z axis in task space\n");
  printf("\n");
  printf("r : increase joint 1 angle\n");
  printf("f : decrease joint 1 angle\n");
  printf("t : increase joint 2 angle\n");
  printf("g : decrease joint 2 angle\n");
  printf("y : increase joint 3 angle\n");
  printf("h : decrease joint 3 angle\n");
  printf("u : increase joint 4 angle\n");
  printf("j : decrease joint 4 angle\n");
  printf("i : increase joint 5 angle\n");
  printf("k : decrease joint 5 angle\n");
  printf("o : increase joint 6 angle\n");
  printf("l : decrease joint 6 angle\n");
  printf("       \n");
  if (use_gripper_)
  {
    printf("\n");
    printf("v : gripper open\n");
    printf("b : gripper close\n");    
  }
  printf("1 : init pose\n");
  printf("2 : home pose\n");
  printf("       \n");
  printf("q to quit\n");
  printf("-------------------------------------------------------------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf J5: %.3lf J6: %.3lf\n",
         get_present_joint_angle().at(0),
         get_present_joint_angle().at(1),
         get_present_joint_angle().at(2),
         get_present_joint_angle().at(3));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         get_present_kinematics_pose().at(0),
         get_present_kinematics_pose().at(1),
         get_present_kinematics_pose().at(2));
  printf("---------------------------\n");  
}

std::vector<double> OpenManipulatorProTeleopKeyboard::get_present_joint_angle()
{
  return present_joint_angle_;
}

std::vector<double> OpenManipulatorProTeleopKeyboard::get_present_kinematics_pose()
{
  return present_kinematic_position_;
}

void OpenManipulatorProTeleopKeyboard::restore_terminal_settings()
{
  tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void OpenManipulatorProTeleopKeyboard::disable_waiting_for_enter()
{
  struct termios newt;

  tcgetattr(0, &oldt_);             /* Save terminal settings */
  newt = oldt_;                     /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO); /* Change settings */
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &newt);     /* Apply settings */
  
}

void OpenManipulatorProTeleopKeyboard::display_callback()  
{
  this->print_text();  
  
  char ch = std::getchar();
  this->set_goal(ch);
}
}  // namespace open_manipulator_pro_teleop_keyboard

/********************************************************************************
** Main
********************************************************************************/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<open_manipulator_pro_teleop_keyboard::OpenManipulatorProTeleopKeyboard>());

  rclcpp::shutdown();

  return 0;
}
