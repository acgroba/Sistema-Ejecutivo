/*!*******************************************************************************************
 *  \file       behavior_land.cpp
 *  \brief      Behavior Land implementation file.
 *  \details    This file implements the behaviorLand class.
 *  \authors    Rafael Artiñano Muñoz
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

#include "../include/behavior_land.h"
#include <pluginlib/class_list_macros.h>

namespace basic_quadrotor_behaviors
{
BehaviorLand::BehaviorLand() : BehaviorExecutionController() { setName("land"); }

BehaviorLand::~BehaviorLand() {}

bool BehaviorLand::checkSituation()
{
  if (isLanded)
  {
    setErrorMessage("Error: Drone landed");
    return false;
  }
  return true;
}




void BehaviorLand::poseCallback(const droneMsgsROS::dronePose& pose) {
 
  if(pose.z < 0.1) {
    isLanded = true;
  }
}

void BehaviorLand::checkGoal()
{

  double precision_land_pose = 0.1;


    if(estimated_pose_msg.pose.position.z < precision_land_pose) {
      BehaviorExecutionController::setTerminationCause(aerostack_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
    }
   

}

void BehaviorLand::checkProgress() 
{  

}

void BehaviorLand::checkProcesses() 
{ 
 
}

 
void BehaviorLand::onConfigure()
{
  nh = getNodeHandle();
  nspace = getNamespace(); 

  ros::param::get("~estimated_pose_topic", estimated_pose_str);
  ros::param::get("~controllers_topic", controllers_str);
  ros::param::get("~pose_topic", pose_topic);

  isLanded = false;
}

void BehaviorLand::onActivate()
{
  // Activate communications
  estimated_pose_sub = nh.subscribe("/" + nspace + "/" + estimated_pose_str, 1000, &BehaviorLand::estimatedPoseCallBack, this);
  pose_subscriber = nh.subscribe("/" + nspace + "/" + pose_topic, 1, &BehaviorLand::poseCallback, this);
  controllers_pub = nh.advertise<droneMsgsROS::droneCommand>("/" + nspace + "/" + controllers_str, 1, true);
  // Behavior implementation
  std_msgs::Header header;
  header.frame_id = "behavior_land";

  droneMsgsROS::droneCommand msg;
  msg.header = header;
  msg.command = droneMsgsROS::droneCommand::LAND;
  controllers_pub.publish(msg);

  estimated_pose_msg.pose.position.z = 1;
}

void BehaviorLand::onDeactivate()
{

  std_msgs::Header header;
  header.frame_id = "behavior_land";

  droneMsgsROS::droneCommand msg;
  msg.header = header;
  msg.command = droneMsgsROS::droneCommand::HOVER;
  controllers_pub.publish(msg);
  estimated_pose_sub.shutdown();
  controllers_pub.shutdown();
  pose_subscriber.shutdown();
}

void BehaviorLand::onExecute()
{
  
}


// Custom topic Callbacks
void BehaviorLand::estimatedPoseCallBack(const  geometry_msgs::PoseStamped& msg)
{
  estimated_pose_msg=msg;
}

}
PLUGINLIB_EXPORT_CLASS(basic_quadrotor_behaviors::BehaviorLand, nodelet::Nodelet)
