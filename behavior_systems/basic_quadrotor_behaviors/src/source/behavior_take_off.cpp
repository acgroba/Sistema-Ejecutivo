/*!*******************************************************************************************
 *  \file       behavior_take_off.cpp
 *  \brief      Behavior TakeOff implementation file.
 *  \details    This file implements the BehaviorTakeOff class.
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

#include "../include/behavior_take_off.h"
#include <pluginlib/class_list_macros.h>

namespace basic_quadrotor_behaviors
{
BehaviorTakeOff::BehaviorTakeOff() : BehaviorExecutionController() 
{ 
  setName("take_off"); 
}

BehaviorTakeOff::~BehaviorTakeOff() {}

void BehaviorTakeOff::onConfigure()
{ 
  nh = getNodeHandle();
  nspace = getNamespace();
  
  ros::param::get("~estimated_pose_topic", estimated_pose_str);
  ros::param::get("~controllers_topic", controllers_str);
  ros::param::get("~rotation_angles_topic", rotation_angles_str);
  ros::param::get("~initialize_yaw_srv", initialize_yaw_str);
  ros::param::get("~battery_topic", battery_topic);
  ros::param::get("~pose_topic", pose_topic);

  isLow=false;
  isFlying=false;
}

void BehaviorTakeOff::onActivate()
{
  // Activate communications
  estimated_pose_sub = nh.subscribe("/" + nspace + "/" + estimated_pose_str, 1000,
                                             &BehaviorTakeOff::estimatedPoseCallBack, this);
  rotation_angles_sub = nh.subscribe("/" + nspace + "/" + rotation_angles_str, 1000,
                                              &BehaviorTakeOff::rotationAnglesCallback, this);
  //battery topic
  battery_subscriber = nh.subscribe("/" + nspace + "/" + battery_topic, 1, &BehaviorTakeOff::batteryCallback, this);
  //flight topic
  pose_subscriber = nh.subscribe("/" + nspace + "/" + pose_topic, 1, &BehaviorTakeOff::poseCallback, this);

  initialize_yaw_cli =
      nh.serviceClient<droneMsgsROS::setInitDroneYaw_srv_type>("/" + nspace + "/" + initialize_yaw_str);
  controllers_pub = nh.advertise<droneMsgsROS::droneCommand>("/" + nspace + "/" + controllers_str, 1, true);

  // Behavior implementation
  std_msgs::Header header;
  header.frame_id = "behavior_take_off";

  droneMsgsROS::droneCommand msg;
  msg.header = header;
  msg.command = droneMsgsROS::droneCommand::TAKE_OFF;
  controllers_pub.publish(msg);
  estimated_pose_msg.pose.position.z = 0;
  first_position = false;
}

void BehaviorTakeOff::onDeactivate()
{
  std_msgs::Header header;
  header.frame_id = "behavior_take_off";

  droneMsgsROS::droneCommand msg;
  msg.header = header;
  msg.command = droneMsgsROS::droneCommand::HOVER;

  controllers_pub.publish(msg);

  estimated_pose_sub.shutdown();
  rotation_angles_sub.shutdown();
  controllers_pub.shutdown();
  initialize_yaw_cli.shutdown();
 // pose_subscriber.shutdown();
 // battery_subscriber.shutdown();

}

void BehaviorTakeOff::onExecute()
{
  
}

bool BehaviorTakeOff::checkSituation()
{
	if(isLow)
  {
    setErrorMessage("Error: Battery low, unable to perform action");
    return false;
  }
  if (isFlying)
  {
    setErrorMessage("Error: Already flying");
    return false;
  }
  return true;
}

void BehaviorTakeOff::batteryCallback(const droneMsgsROS::battery& battery) {

	if(battery.batteryPercent < BATTERY_LOW_THRESHOLD) {
		isLow=true;
	}
}


void BehaviorTakeOff::poseCallback(const droneMsgsROS::dronePose& pose) {

	if(pose.z > 0.1) {
		isFlying = true;
	}
} 

void BehaviorTakeOff::checkGoal()
{
  double precision_take_off = 0.1;
  // Check achievement
  if (std::abs(std::abs(estimated_pose_msg.pose.position.z) - 0.7) < precision_take_off)
  {
    BehaviorExecutionController::setTerminationCause(aerostack_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
    droneMsgsROS::setInitDroneYaw_srv_type init_yaw_msg;
    init_yaw_msg.request.yaw_droneLMrT_telemetry_rad = (rotation_angles_msg.vector.z) * (M_PI / 180.0);
    initialize_yaw_cli.call(init_yaw_msg);

    droneMsgsROS::droneCommand msg;
    msg.command = droneMsgsROS::droneCommand::HOVER;
    controllers_pub.publish(msg);

  }

}

void BehaviorTakeOff::checkProgress() 
{ 
 
}


void BehaviorTakeOff::checkProcesses() 
{ 
 
}

// Custom topic Callbacks
void BehaviorTakeOff::estimatedPoseCallBack(const geometry_msgs::PoseStamped &message)
{
  if (!first_position)

    estimated_pose_msg = message;
}

void BehaviorTakeOff::rotationAnglesCallback(const geometry_msgs::Vector3Stamped &message)
{
  rotation_angles_msg = message;
}
}
PLUGINLIB_EXPORT_CLASS(basic_quadrotor_behaviors::BehaviorTakeOff, nodelet::Nodelet)
