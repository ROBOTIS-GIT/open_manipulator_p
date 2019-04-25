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
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "open_manipulator_pro_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace open_manipulator_pro_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  joint_name.push_back("joint1");
  joint_name.push_back("joint2");
  joint_name.push_back("joint3");
  joint_name.push_back("joint4");
  joint_name.push_back("joint5");
  joint_name.push_back("joint6");

  /*********************
    ** Logging
    **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  joint_spinbox.append( ui.joint1_spinbox );
  joint_spinbox.append( ui.joint2_spinbox );
  joint_spinbox.append( ui.joint3_spinbox );
  joint_spinbox.append( ui.joint4_spinbox );
  joint_spinbox.append( ui.joint5_spinbox );
  joint_spinbox.append( ui.joint6_spinbox );

  /****************************
    ** Connect
    ****************************/

  qRegisterMetaType<open_manipulator_pro_base_module_msgs::JointPose>("open_manipulator_pro_base_module_msgs::JointPose");
  QObject::connect(&qnode, SIGNAL(updateCurrentJointPose(open_manipulator_pro_base_module_msgs::JointPose)), this, SLOT(updateCurrJointPoseSpinbox(open_manipulator_pro_base_module_msgs::JointPose)));

  qRegisterMetaType<open_manipulator_pro_base_module_msgs::KinematicsPose>("open_manipulator_pro_base_module_msgs::KinematicsPose");
  QObject::connect(&qnode, SIGNAL(updateCurrentKinematicsPose(open_manipulator_pro_base_module_msgs::KinematicsPose)), this, SLOT(updateCurrKinematicsPoseSpinbox(open_manipulator_pro_base_module_msgs::KinematicsPose)));

  /*********************
    ** Auto Start
    **********************/
  qnode.init();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::on_curr_joint_button_clicked( bool check )
{
  qnode.getJointPose( joint_name );
}

void MainWindow::on_des_joint_button_clicked( bool check )
{
  open_manipulator_pro_base_module_msgs::JointPose msg;

  for ( int _id = 0; _id < joint_spinbox.size(); _id++ )
  {
    msg.name.push_back( joint_name[ _id ] );
    msg.value.push_back( ((QDoubleSpinBox *) joint_spinbox[ _id ])->value() * M_PI / 180.0 );
  }

  qnode.sendJointPoseMsg( msg );
}

void MainWindow::on_curr_pos_button_clicked( bool check )
{
  std::string group_name = "arm";

  qnode.getKinematicsPose( group_name );
}

void MainWindow::on_des_pos_button_clicked( bool check )
{
  open_manipulator_pro_base_module_msgs::KinematicsPose msg;

  msg.name = "arm";

  msg.pose.position.x = ui.pos_x_spinbox->value();
  msg.pose.position.y = ui.pos_y_spinbox->value();
  msg.pose.position.z = ui.pos_z_spinbox->value();

  double roll = ui.ori_roll_spinbox->value() * M_PI / 180.0;
  double pitch = ui.ori_pitch_spinbox->value() * M_PI / 180.0;
  double yaw = ui.ori_yaw_spinbox->value() * M_PI / 180.0;

  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();

  qnode.sendKinematicsPoseMsg( msg );
}

void MainWindow::on_ini_pose_button_clicked( bool check )
{
  std_msgs::String msg;

  msg.data ="ini_pose";

  qnode.sendIniPoseMsg( msg );
}

void MainWindow::on_set_mode_button_clicked( bool check )
{
  std_msgs::String msg;

  msg.data ="set_mode";

  qnode.sendSetModeMsg( msg );
}

void MainWindow::updateCurrJointPoseSpinbox( open_manipulator_pro_base_module_msgs::JointPose msg )
{
  for ( int _name_index = 0; _name_index < msg.name.size(); _name_index++ )
  {
    for ( int _id = 0; _id < joint_name.size();  _id++ )
    {
      if ( msg.name[ _id ] == joint_name[ _name_index ] )
      {
        ((QDoubleSpinBox *) joint_spinbox[ _name_index ])->setValue( msg.value[ _id ] * 180.0 / M_PI );
        break;
      }
    }
  }
}

void MainWindow::updateCurrKinematicsPoseSpinbox( open_manipulator_pro_base_module_msgs::KinematicsPose msg )
{
  ui.pos_x_spinbox->setValue( msg.pose.position.x );
  ui.pos_y_spinbox->setValue( msg.pose.position.y );
  ui.pos_z_spinbox->setValue( msg.pose.position.z );

  Eigen::Quaterniond QR( msg.pose.orientation.w , msg.pose.orientation.x , msg.pose.orientation.y , msg.pose.orientation.z );

  Eigen::MatrixXd rpy = quaternion2rpy( QR );

  double roll = rpy.coeff( 0 , 0 ) * 180.0 / M_PI;
  double pitch = rpy.coeff( 1 , 0 ) * 180.0 / M_PI;
  double yaw = rpy.coeff( 2, 0 ) * 180.0 /M_PI;

  ui.ori_roll_spinbox->setValue( roll );
  ui.ori_pitch_spinbox->setValue( pitch );
  ui.ori_yaw_spinbox->setValue( yaw );
}

Eigen::MatrixXd MainWindow::rotationX( double angle )
{
  Eigen::MatrixXd _rotation( 3 , 3 );

  _rotation << 1.0,          0.0,           0.0,
      0.0, cos( angle ), -sin( angle ),
      0.0, sin( angle ),  cos( angle );

  return _rotation;
}

Eigen::MatrixXd MainWindow::rotationY( double angle )
{
  Eigen::MatrixXd _rotation( 3 , 3 );

  _rotation << cos( angle ), 0.0, sin( angle ),
      0.0, 1.0, 	     0.0,
      -sin( angle ), 0.0, cos( angle );

  return _rotation;
}

Eigen::MatrixXd MainWindow::rotationZ( double angle )
{
  Eigen::MatrixXd _rotation(3,3);

  _rotation << cos( angle ), -sin( angle ), 0.0,
      sin( angle ),  cos( angle ), 0.0,
      0.0,           0.0, 1.0;

  return _rotation;
}

Eigen::MatrixXd MainWindow::rotation2rpy( Eigen::MatrixXd rotation )
{
  Eigen::MatrixXd _rpy = Eigen::MatrixXd::Zero( 3 , 1 );

  _rpy.coeffRef( 0 , 0 ) = atan2( rotation.coeff( 2 , 1 ), rotation.coeff( 2 , 2 ) );
  _rpy.coeffRef( 1 , 0 ) = atan2( -rotation.coeff( 2 , 0 ), sqrt( pow( rotation.coeff( 2 , 1 ) , 2 ) + pow( rotation.coeff( 2 , 2 ) , 2 ) ) );
  _rpy.coeffRef( 2 , 0 ) = atan2 ( rotation.coeff( 1 , 0 ) , rotation.coeff( 0 , 0 ) );

  return _rpy;
}

Eigen::MatrixXd MainWindow::rpy2rotation( double roll, double pitch, double yaw )
{
  Eigen::MatrixXd _rotation = rotationZ( yaw ) * rotationY( pitch ) * rotationX( roll );

  return _rotation;
}

Eigen::Quaterniond MainWindow::rpy2quaternion( double roll, double pitch, double yaw )
{
  Eigen::MatrixXd _rotation = rpy2rotation( roll, pitch, yaw );

  Eigen::Matrix3d _rotation3d;
  _rotation3d = _rotation.block( 0 , 0 , 3 , 3 );

  Eigen::Quaterniond _quaternion;

  _quaternion = _rotation3d;

  return _quaternion;
}

Eigen::Quaterniond MainWindow::rotation2quaternion( Eigen::MatrixXd rotation )
{
  Eigen::Matrix3d _rotation3d;

  _rotation3d = rotation.block( 0 , 0 , 3 , 3 );

  Eigen::Quaterniond _quaternion;
  _quaternion = _rotation3d;

  return _quaternion;
}

Eigen::MatrixXd MainWindow::quaternion2rpy( Eigen::Quaterniond quaternion )
{
  Eigen::MatrixXd _rpy = rotation2rpy( quaternion.toRotationMatrix() );

  return _rpy;
}

Eigen::MatrixXd MainWindow::quaternion2rotation( Eigen::Quaterniond quaternion )
{
  Eigen::MatrixXd _rotation = quaternion.toRotationMatrix();

  return _rotation;
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
  ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Robotis</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

}  // namespace open_manipulator_pro_gui

