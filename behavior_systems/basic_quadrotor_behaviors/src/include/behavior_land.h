/*!*******************************************************************************************
 *  \file       behavior_land.h
 *  \brief      behavior land definition file.
 *  \details     This file contains the BehaviorLand declaration. To obtain more information about
 *              it's definition consult the behavior_land.cpp file.
 *  \authors    Rafael Artiñano Muñoz
 *  \copyright  Copyright (c) 2018 Universidad Politecnica de Madrid
 *              All Rights Reserved
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

#ifndef LAND_H
#define LAND_H

// System
#include <string>
#include <thread>
#include <tuple>
// ROS
#include "std_srvs/Empty.h"
#include <geometry_msgs/Vector3Stamped.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

// Aerostack msgs

#include <aerostack_msgs/BehaviorActivationFinished.h>
//#include <droneMsgsROS/ConsultBelief.h>
#include <droneMsgsROS/droneCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <droneMsgsROS/dronePose.h>
#include <droneMsgsROS/battery.h>

// Aerostack libraries
#include <behavior_execution_controller.h>

namespace basic_quadrotor_behaviors
{
class BehaviorLand : public BehaviorExecutionController
{
  // Constructor
public:
  BehaviorLand();
  ~BehaviorLand();

private:
  ros::NodeHandle nh;
  std::string nspace; 

  // Congfig variables
  std::string estimated_pose_str;
  std::string controllers_str;
  std::string execute_query_srv;
  // ------state_dron-------
  std::string pose_topic;
  bool isLanded;

  // Communication variables
  ros::Subscriber estimated_pose_sub;
  ros::Subscriber pose_subscriber;
  ros::Subscriber battery_subscriber;
  ros::Publisher controllers_pub;
 // ros::ServiceClient query_client;

  // Messages
   geometry_msgs::PoseStamped estimated_pose_msg;


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

public: // Callbacks
  void estimatedPoseCallBack(const geometry_msgs::PoseStamped&);
  void poseCallback(const droneMsgsROS::dronePose& pose);

};
}

#endif
