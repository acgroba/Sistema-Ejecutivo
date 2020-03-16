/*!*******************************************************************************************
 *  \file       behavior_wait.cpp
 *  \brief      Behavior BehaviorWait implementation file.
 *  \details    This file implements the BehaviorWait class.
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

#include "../include/behavior_wait.h"
#include <pluginlib/class_list_macros.h>

namespace basic_quadrotor_behaviors
{
BehaviorWait::BehaviorWait() : BehaviorExecutionController() 
{ 
  setName("wait");
  
 }

BehaviorWait::~BehaviorWait() {}

bool BehaviorWait::checkSituation()
{
    
	if(isLow)
  {
    setErrorMessage("Error: Battery low, unable to perform action");
    
    return false;
  }
  if (isLanded)
  {
    setErrorMessage("Error: Drone landed");

    return false;
  }
  return true;
}

void BehaviorWait::poseCallback(const  droneMsgsROS::droneStatus& pose) {
 
  if(pose.status == droneMsgsROS::droneStatus::LANDED) {
    isLanded = true;
  }
}
void BehaviorWait::batteryCallback(const droneMsgsROS::battery& battery) {
	
	if(battery.batteryPercent < BATTERY_LOW_THRESHOLD) {
		isLow=true;
	}
}
void BehaviorWait::checkGoal()
{
    if(timeout_end && timer_msg)
    {
      BehaviorExecutionController::setTerminationCause(aerostack_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
    }
}

void BehaviorWait::checkProgress()
{ 

}

void BehaviorWait::checkProcesses() 
{ 
 
}

void BehaviorWait::onConfigure()
{
  nh = getNodeHandle();
  nspace = getNamespace();

  ros::param::get("~battery_topic", battery_topic);
  ros::param::get("~status_topic", status_topic); 

  timer_msg=false;
  isLow=false;
  isLanded = false;
  //battery topic
  battery_subscriber = nh.subscribe("/" + nspace + "/" + battery_topic, 1, &BehaviorWait::batteryCallback, this);
  //status topic
  status_subscriber = nh.subscribe("/" + nspace + "/" + status_topic, 1, &BehaviorWait::poseCallback, this);
}

void BehaviorWait::onActivate()
{
  timer_msg=false;
  timeout_end=false;
  /*behavior implementation*/
  std::string arguments = getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  if(config_file["duration"].IsDefined())
  {
    timeout=config_file["duration"].as<float>();
    timeout_end=true;
    timer = nh.createTimer(ros::Duration(timeout), &BehaviorWait::timerCallback, this);
  }
}

void BehaviorWait::onDeactivate()
{
 // pose_subscriber.shutdown();
 // battery_subscriber.shutdown();
}

void BehaviorWait::onExecute()
{

}


void BehaviorWait::timerCallback(const ros::TimerEvent& event)
{
  timer_msg=true;
}
}
PLUGINLIB_EXPORT_CLASS(basic_quadrotor_behaviors::BehaviorWait, nodelet::Nodelet)
