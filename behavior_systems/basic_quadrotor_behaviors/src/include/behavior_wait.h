/*!*******************************************************************************************
 *  \file       behavior_wait.h
 *  \brief      behavior wait definition file.
 *  \details     This file contains the BehaviorWait declaration. To obtain more information about
 *              it's definition consult the behavior_wait.cpp file.
 *  \authors    Rafael Artiñano Muñoz, Abraham Carrera Groba
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
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

#ifndef WAIT_H
#define WAIT_H

// System
#include <string>
#include <thread>
#include <tuple>
// ROS
#include "std_srvs/Empty.h"
#include <nodelet/nodelet.h>
#include <ros/ros.h>

// Aerostack msgs
#include <aerostack_msgs/BehaviorActivationFinished.h>
#include <droneMsgsROS/droneSpeeds.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <yaml-cpp/yaml.h>
// Aerostack libraries
#include <behavior_execution_controller.h>
#include <droneMsgsROS/battery.h>
#include <droneMsgsROS/droneStatus.h>

namespace basic_quadrotor_behaviors
{
class BehaviorWait : public BehaviorExecutionController
{
  // Constructor
public:
  BehaviorWait();
  ~BehaviorWait();

private:
  ros::NodeHandle nh;
  std::string nspace; 
  // Communication variables
  ros::Subscriber battery_subscriber;
  ros::Subscriber status_subscriber;


  ros::Timer timer;
  bool timer_msg;
  bool timeout_end;
  float timeout;

  std::string pose_topic;
  bool isLanded;

  std::string battery_topic; 
  const double BATTERY_LOW_THRESHOLD = 25;
  bool isLow;

  std::string status_topic; 

private:
  // BehaviorExecutionController
  bool checkSituation();
  void checkGoal();
  void checkProgress();
  void checkProcesses();

  void onConfigure();
  void onActivate();
  void onDeactivate();
  void onExecute();

public: // Callbacks
  void timerCallback(const ros::TimerEvent&);
  void poseCallback(const droneMsgsROS::droneStatus& pose);
  void batteryCallback(const droneMsgsROS::battery& battery);

};
}

#endif
