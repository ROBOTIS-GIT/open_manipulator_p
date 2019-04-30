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

/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include "open_manipulator_pro_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace open_manipulator_pro_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc_(argc),
  init_argv_(argv)
{}

QNode::~QNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init() {
  ros::init(init_argc_,init_argv_,"open_manipulator_pro_gui");

  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  // Add your ros communications here.
  ini_pose_msg_pub_ = n.advertise<std_msgs::String>("/robotis/base/ini_pose_msg", 0);
  set_mode_msg_pub_ = n.advertise<std_msgs::String>("/robotis/base/set_mode_msg", 0);

  joint_pose_msg_pub_ = n.advertise<open_manipulator_pro_base_module_msgs::JointPose>("/robotis/base/joint_pose_msg", 0);
  kinematics_pose_msg_pub_ = n.advertise<open_manipulator_pro_base_module_msgs::KinematicsPose>("/robotis/base/kinematics_pose_msg", 0);

  get_joint_pose_client_ = n.serviceClient<open_manipulator_pro_base_module_msgs::GetJointPose>("/robotis/base/get_joint_pose", 0);
  get_kinematics_pose_client_ = n.serviceClient<open_manipulator_pro_base_module_msgs::GetKinematicsPose>("/robotis/base/get_kinematics_pose", 0);

  status_msg_sub_ = n.subscribe("/robotis/status", 10, &QNode::statusMsgCallback, this);

  start();
  return true;
}

void QNode::run() {

  ros::Rate loop_rate(50);

  while ( ros::ok() )
  {
    ros::spinOnce();

    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::statusMsgCallback(const robotis_controller_msgs::StatusMsg::ConstPtr &msg)
{
  log((LogLevel) msg->type, msg->status_msg, msg->module_name);
}

void QNode::log( const LogLevel &level, const std::string &msg, std::string sender)
{
  logging_model_.insertRows(logging_model_.rowCount(),1);
  std::stringstream logging_model_msg;

  std::stringstream _sender;
  _sender << "[" << sender << "] ";

  switch ( level ) {
  case(Debug) : {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << _sender.str() << msg;
    break;
  }
  case(Info) : {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << _sender.str() << msg;
    break;
  }
  case(Warn) : {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[WARN] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  case(Error) : {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "<ERROR> [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  case(Fatal) : {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << _sender.str() <<msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(logging_model_.rowCount()-1),new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::sendJointPoseMsg( open_manipulator_pro_base_module_msgs::JointPose msg )
{
  joint_pose_msg_pub_.publish( msg );

  log( Info , "Send Joint Pose Msg" );
}

void QNode::sendKinematicsPoseMsg( open_manipulator_pro_base_module_msgs::KinematicsPose msg )
{
  kinematics_pose_msg_pub_.publish( msg );

  log( Info , "Send Kinematics Pose Msg" );
}

void QNode::sendIniPoseMsg( std_msgs::String msg )
{
  ini_pose_msg_pub_.publish ( msg );

  log( Info , "Go to Manipulator Initial Pose" );
}

void QNode::sendSetModeMsg( std_msgs::String msg )
{
  set_mode_msg_pub_.publish ( msg );

  log( Info , "Set BaseModule" );
}

void QNode::getJointPose( std::vector<std::string> joint_name )
{
  log( Info , "Get Current Joint Pose" );

  open_manipulator_pro_base_module_msgs::GetJointPose _get_joint_pose;

  // request
  for ( int _id = 0; _id < joint_name.size(); _id++ )
    _get_joint_pose.request.joint_name.push_back( joint_name[ _id ] );

  // response
  if ( get_joint_pose_client_.call ( _get_joint_pose ) )
  {
    open_manipulator_pro_base_module_msgs::JointPose _joint_pose;

    for ( int _id = 0; _id < _get_joint_pose.response.joint_name.size(); _id++ )
    {
      _joint_pose.name.push_back( _get_joint_pose.response.joint_name[ _id ] );
      _joint_pose.value.push_back( _get_joint_pose.response.joint_value[ _id ]);
    }

    Q_EMIT updateCurrentJointPose( _joint_pose );
  }
  else
    log(Error, "fail to get joint pose.");
}

void QNode::getKinematicsPose ( std::string group_name )
{
  log( Info , "Get Current Kinematics Pose" );

  open_manipulator_pro_base_module_msgs::GetKinematicsPose _get_kinematics_pose;

  // request
  _get_kinematics_pose.request.group_name = group_name;

  // response
  if ( get_kinematics_pose_client_.call( _get_kinematics_pose ) )
  {
    open_manipulator_pro_base_module_msgs::KinematicsPose _kinematcis_pose;

    _kinematcis_pose.name = _get_kinematics_pose.request.group_name;
    _kinematcis_pose.pose = _get_kinematics_pose.response.group_pose;

    Q_EMIT updateCurrentKinematicsPose( _kinematcis_pose );
  }
  else
    log(Error, "fail to get kinematcis pose.");
}

}  // namespace open_manipulator_pro_gui
