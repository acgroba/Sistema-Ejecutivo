/*!*******************************************************************************************
 *  \file       behavior_localize_with_odometry.cpp
 *  \brief      BehaviorLocalizeWithOdometry implementation file.
 *  \details    This file implements the BehaviorLocalizeWithOdometry class.
 *  \authors    Abraham Carrera Groba, Alberto Camporredondo Portela
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

#include "../include/behavior_localize_with_odometry.h"
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace basic_quadrotor_behaviors
{
BehaviorLocalizeWithOdometry::BehaviorLocalizeWithOdometry() : BehaviorExecutionController() {
 setName("localize_with_odometry"); 
 setExecutionGoal(ExecutionGoals::KEEP_RUNNING); 
}

BehaviorLocalizeWithOdometry::~BehaviorLocalizeWithOdometry() {}

bool BehaviorLocalizeWithOdometry::checkSituation() 
{ 
 return true; 
}

void BehaviorLocalizeWithOdometry::checkGoal() 
{ 

}

void BehaviorLocalizeWithOdometry::checkProgress() 
{ 
 
}

void BehaviorLocalizeWithOdometry::checkProcesses() 
{ 
 
}

void BehaviorLocalizeWithOdometry::onConfigure()
{
  nh = getNodeHandle();
  nspace = getNamespace(); 
  
  ros::param::get("~odometry_str", odometry_str);
}

void BehaviorLocalizeWithOdometry::onActivate()
{
  // Start processes
  request_processes_activation_cli =
      nh.serviceClient<aerostack_msgs::RequestProcesses>("/" + nspace + "/" + "start_processes");
  request_processes_deactivation_cli =
      nh.serviceClient<aerostack_msgs::RequestProcesses>("/" + nspace + "/" + "stop_processes");

  processes = {"droneOdometryStateEstimator", "self_localization_selector_process", "droneLocalizer",
               "droneObstacleProcessor"};

  aerostack_msgs::RequestProcesses request_processes_srv;
  for (std::string process : processes)
  {
    request_processes_srv.request.processes.push_back(process);
  }

  request_processes_activation_cli.call(request_processes_srv);

  if (!request_processes_srv.response.acknowledge)
  {
    setErrorMessage("Processes for Behavior: [ " + ros::this_node::getName() + " ] couldn't be started");
  }

  // Activate communications
  odometry_srv = nh.serviceClient<std_srvs::Empty>(odometry_str);

  // Behavior implementation
  std_srvs::Empty empty;
  odometry_srv.call(empty);
}

void BehaviorLocalizeWithOdometry::onDeactivate()
{
  aerostack_msgs::RequestProcesses request_processes_srv;
  for (std::string process : processes)
  {
    request_processes_srv.request.processes.push_back(process);
  }

  request_processes_deactivation_cli.call(request_processes_srv);

  if (!request_processes_srv.response.acknowledge)
  {
    setErrorMessage("Processes for Behavior: [ " + ros::this_node::getName() + " ] couldn't be stopped");
  }
  odometry_srv.shutdown();
  request_processes_activation_cli.shutdown();
  request_processes_deactivation_cli.shutdown();
}

void BehaviorLocalizeWithOdometry::onExecute()
{
  
}

}
PLUGINLIB_EXPORT_CLASS(basic_quadrotor_behaviors::BehaviorLocalizeWithOdometry, nodelet::Nodelet)
