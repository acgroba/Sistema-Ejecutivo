/*!*******************************************************************************************
 *  \file       behavior_follow_path.h
 *  \brief      Behavior Follow path implementation file.
 *  \details    This file implements the follow path class.
 *  \authors    Alberto Rodelgo Perales
 *  \copyright  Copyright (c) 2018 Universidad Politecnica de Madrid
 *              All rights reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef FOLLOW_PATH_H
#define FOLLOW_PATH_H

// System
#include <string>
#include "math.h"

// ROS
#include "std_srvs/Empty.h"
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <aerostack_msgs/BehaviorActivationFinished.h>
#include <aerostack_msgs/RequestProcesses.h>
#include <cmath>
// Aerostack msgs
#include <aerostack_msgs/BehaviorActivationFinished.h>
#include <droneMsgsROS/ConsultBelief.h>
#include <droneMsgsROS/droneCommand.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "mav_msgs/RollPitchYawrateThrust.h"
#include "droneMsgsROS/droneStatus.h"
#include <nav_msgs/Path.h>
// Aerostack libraries
#include <behavior_execution_controller.h>
#include "aerostack_msgs/SetControlMode.h"
#include <yaml-cpp/yaml.h>

const int MAX_DISTANCE = 1000; //maximum meters allowed

namespace quadrotor_motion_with_pid_control
{
class BehaviorFollowPath : public BehaviorExecutionController
{
  // Constructor
public:
  BehaviorFollowPath();
  ~BehaviorFollowPath();

private:
  // Congfig variables
  std::string self_localization_speed_str;
  std::string command_high_level_str;
  std::string motion_reference_pose_str;
  std::string self_localization_pose_str;  
  std::string motion_reference_speed_str;
  std::string status_str;
  std::string set_control_mode_srv;
  std::string motion_reference_path_str;


  ros::NodeHandle node_handle;
  std::string nspace; 
  // Communication variables
  ros::ServiceClient request_processes_activation_cli;
  ros::ServiceClient request_processes_deactivation_cli;
  // Subscriber
  ros::Subscriber self_localization_speed_sub; 
  ros::Subscriber self_localization_pose_sub;   
  ros::Subscriber motion_reference_pose_sub;
  ros::Subscriber status_sub;
  //Publishers
  ros::Publisher command_high_level_pub;
  ros::Publisher motion_reference_speed_pub;
  ros::Publisher motion_reference_path_pub;
  //Service
  ros::ServiceClient setControlModeClientSrv;
  aerostack_msgs::RequestProcesses request_processes_srv;

  // Messages
  geometry_msgs::TwistStamped estimated_speed_msg;
  droneMsgsROS::droneStatus status_msg;
  droneMsgsROS::droneCommand high_level_command;
  geometry_msgs::TwistStamped motion_reference_speed;
  nav_msgs::Path reference_path;
  geometry_msgs::PoseStamped last_path_point;
  geometry_msgs::PoseStamped estimated_pose_msg;
  geometry_msgs::PoseStamped reference_pose;
  geometry_msgs::PoseStamped last_target_pose;
  geometry_msgs::PoseStamped current_target_pose;

  bool received_speed;
  int path_index;
  std::string direction;
  bool execute;
private:
  // BehaviorExecutionController
  void onConfigure();
  void onActivate();
  void onDeactivate();
  void onExecute();
  bool checkSituation();
  void checkGoal();
  void checkProgress();
  void checkProcesses();

  bool setControlMode(int new_control_mode);
  bool checkQuadrotorStopped();
public: 
// Callbacks
void selfLocalizationSpeedCallBack(const geometry_msgs::TwistStamped &msg);
void selfLocalizationPoseCallBack(const geometry_msgs::PoseStamped &msg);
void motionReferencePoseCallBack(const geometry_msgs::PoseStamped &msg);
void statusCallBack(const droneMsgsROS::droneStatus &msg);
};
}

#endif
