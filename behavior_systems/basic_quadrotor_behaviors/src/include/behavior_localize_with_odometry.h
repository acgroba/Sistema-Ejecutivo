/*!*******************************************************************************************
 *  \file       behavior_localize_with_odometry.h
 *  \brief      behavior localize with odometry definition file.
 *  \details     This file contains the BehaviorLocalizeWithOdometry declaration. To obtain more information about
 *              it's definition consult the behavior_localize_with_odometry.cpp file.
 *  \authors    Abraham Carrera Groba, Alberto Camporredondo
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
*********************************************************************************************/

#ifndef LOCALIZE_WITH_ODOMETRY_H
#define LOCALIZE_WITH_ODOMETRY_H

// System
#include <map>
#include <string>
#include <thread>
#include <tuple>
// ROS
#include "std_srvs/Empty.h"
#include <nodelet/nodelet.h>
#include <ros/ros.h>

// Aerostack msgs
#include <aerostack_msgs/BehaviorActivationFinished.h>
#include <aerostack_msgs/RequestProcesses.h>

// Aerostack libraries
#include <behavior_execution_controller.h>

namespace basic_quadrotor_behaviors
{
class BehaviorLocalizeWithOdometry : public BehaviorExecutionController
{
  // Constructor
public:
  BehaviorLocalizeWithOdometry();
  ~BehaviorLocalizeWithOdometry();

private:
  ros::NodeHandle nh;
  std::string nspace; 

  // Congfig variables
  std::string odometry_str;
  std::vector<std::string> processes;

  // Communication variables
  ros::ServiceClient request_processes_activation_cli;
  ros::ServiceClient request_processes_deactivation_cli;
  ros::ServiceClient odometry_srv;


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

};
}

#endif
