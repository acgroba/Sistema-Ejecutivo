/*!*******************************************************************************************
 *  \file       behavior_keep_hovering_with_pid_control.cpp
 *  \brief      Behavior Keep Hovering implementation file.
 *  \details    This file implements the keep hovering class.
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

#include "../include/behavior_keep_hovering_with_pid_control.h"
#include <pluginlib/class_list_macros.h>

namespace quadrotor_motion_with_pid_control
{
BehaviorKeepHoveringWithPidControl::BehaviorKeepHoveringWithPidControl() : BehaviorExecutionController() { 
  setName("keep_hovering_with_pid_control"); 
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING); 
}

BehaviorKeepHoveringWithPidControl::~BehaviorKeepHoveringWithPidControl() {}

void BehaviorKeepHoveringWithPidControl::onConfigure()
{
  node_handle = getNodeHandle();
  nspace = getNamespace();  
  node_handle.param<std::string>("self_localization_speed_topic", self_localization_speed_str, "self_localization/speed");
  node_handle.param<std::string>("estimated_pose_topic", self_localization_pose_str, "self_localization/pose");
  node_handle.param<std::string>("controllers_topic", command_high_level_str, "command/high_level");
  node_handle.param<std::string>("motion_reference_speed_topic", motion_reference_speed_str,"motion_reference/speed");
  node_handle.param<std::string>("motion_reference_pose_topic", motion_reference_pose_str,"motion_reference/pose");
  node_handle.param<std::string>("set_control_mode_service_name",set_control_mode_srv,"set_control_mode");
  node_handle.param<std::string>("status_topic",status_str,"status");

  // Start processes
  request_processes_activation_cli =
      node_handle.serviceClient<aerostack_msgs::RequestProcesses>("/" + nspace + "/" + "start_processes");
  request_processes_deactivation_cli =
      node_handle.serviceClient<aerostack_msgs::RequestProcesses>("/" + nspace + "/" + "stop_processes");

  request_processes_srv.request.processes = {"quadrotor_pid_controller_process"};

  //Subscriber
  status_sub = node_handle.subscribe("/" + nspace + "/"+status_str, 1, &BehaviorKeepHoveringWithPidControl::statusCallBack, this);
}

bool BehaviorKeepHoveringWithPidControl::checkSituation()
{
  //Quadrotor is FLYING
  if (status_msg.status == droneMsgsROS::droneStatus::FLYING){
    return true;
  }else{
    setErrorMessage("Error: Drone is not flying");
    std::cout<<"Error: Drone is not flying"<<std::endl;
    return false;
  }
}

void BehaviorKeepHoveringWithPidControl::onActivate()
{
  request_processes_activation_cli.call(request_processes_srv);
  if (!request_processes_srv.response.acknowledge) setErrorMessage("Processes for Behavior: [ " + ros::this_node::getName() + " ] couldn't be started");

  estimated_speed_msg = *ros::topic::waitForMessage<geometry_msgs::TwistStamped>("/" + nspace + "/"+self_localization_speed_str, node_handle, ros::Duration(1));
  //Subscribers
  self_localization_speed_sub = node_handle.subscribe("/" + nspace + "/"+self_localization_speed_str, 1, &BehaviorKeepHoveringWithPidControl::selfLocalizationSpeedCallBack, this);
  self_localization_pose_sub = node_handle.subscribe("/" + nspace + "/"+self_localization_pose_str, 1, &BehaviorKeepHoveringWithPidControl::selfLocalizationPoseCallBack, this);
  //Publishers
  motion_reference_speed_pub = node_handle.advertise<geometry_msgs::TwistStamped>("/" + nspace + "/"+motion_reference_speed_str,1, true);
  command_high_level_pub = node_handle.advertise<droneMsgsROS::droneCommand>("/" + nspace + "/"+command_high_level_str, 1, true);
  
  motion_reference_pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("/" + nspace + "/"+motion_reference_pose_str, 1,true);
  //Service
  setControlModeClientSrv = node_handle.serviceClient<aerostack_msgs::SetControlMode>("/" + nspace + "/"+set_control_mode_srv);
  ros::Duration(4).sleep();

  
  quadrotor_moving = true;
  received_speed = false;
  droneMsgsROS::droneCommand high_level_command;
  geometry_msgs::TwistStamped motion_reference_speed;
  std_msgs::Header header;

  motion_reference_speed.twist.linear.x = 0.0;
  motion_reference_speed.twist.linear.y = 0.0;
  motion_reference_speed.twist.linear.z = 0.0;
  motion_reference_speed.twist.angular.x = 0.0;
  motion_reference_speed.twist.angular.y = 0.0;
  motion_reference_speed.twist.angular.z = 0.0;
  motion_reference_speed_pub.publish(motion_reference_speed);
  header.frame_id = "behavior_keep_hovering_with_pid_control";  
  high_level_command.header = header;
  high_level_command.command = droneMsgsROS::droneCommand::HOVER;
  command_high_level_pub.publish(high_level_command);  
}

void BehaviorKeepHoveringWithPidControl::onDeactivate()
{
  std_msgs::Header header;
  header.frame_id = "behavior_keep_hovering_with_pid_control";

  droneMsgsROS::droneCommand msg;
  msg.header = header;
  msg.command = droneMsgsROS::droneCommand::HOVER;
  command_high_level_pub.publish(msg);  

  request_processes_deactivation_cli.call(request_processes_srv);
  if (!request_processes_srv.response.acknowledge) setErrorMessage("Processes for Behavior: [ " + ros::this_node::getName() + " ] couldn't be stopped");

  setControlModeClientSrv.shutdown();
  self_localization_pose_sub.shutdown();
  self_localization_speed_sub.shutdown();
  command_high_level_pub.shutdown();
  //motion_reference_speed_pub.shutdown();
  motion_reference_pose_pub.shutdown();
}

void BehaviorKeepHoveringWithPidControl::onExecute()
{
  if (quadrotor_moving){
    if (received_speed && checkQuadrotorStopped()){
      quadrotor_moving = false;
      reference_pose  = estimated_pose_msg;
      setControlMode(aerostack_msgs::QuadrotorPidControllerMode::POSE);
      motion_reference_pose_pub.publish(reference_pose);
    }  
  } 
}

void BehaviorKeepHoveringWithPidControl::checkGoal(){}


void BehaviorKeepHoveringWithPidControl::checkProgress() {
  if (!quadrotor_moving){
    distance = sqrt(pow(estimated_pose_msg.pose.position.x-reference_pose.pose.position.x,2)+
                    pow(estimated_pose_msg.pose.position.y-reference_pose.pose.position.y,2)+
                    pow(estimated_pose_msg.pose.position.z-reference_pose.pose.position.z,2));

    if (distance > 1) BehaviorExecutionController::setTerminationCause(aerostack_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
  }
}


void BehaviorKeepHoveringWithPidControl::checkProcesses() 
{ 
 
}

bool BehaviorKeepHoveringWithPidControl::checkQuadrotorStopped()
{

  if (abs(estimated_speed_msg.twist.linear.x) <= 0.1 && abs(estimated_speed_msg.twist.linear.y) <= 0.1 && abs(estimated_speed_msg.twist.linear.z) <= 0.1 &&
      abs(estimated_speed_msg.twist.angular.x) <= 0.1 && abs(estimated_speed_msg.twist.angular.y) <= 0.1 && abs(estimated_speed_msg.twist.angular.z) <= 0.1 ){
      return true;
  }else{
    return false;
  }
}

//Set control mode
bool BehaviorKeepHoveringWithPidControl::setControlMode(int new_control_mode){
  // Prepare service message
  aerostack_msgs::SetControlMode setControlModeSrv;
  setControlModeSrv.request.controlMode.command = new_control_mode;
  // use service
  if (setControlModeClientSrv.call(setControlModeSrv)){
    return setControlModeSrv.response.ack;
  }else{
    return false;
  }
}

void BehaviorKeepHoveringWithPidControl::selfLocalizationSpeedCallBack(const geometry_msgs::TwistStamped &msg){
  estimated_speed_msg = msg; received_speed = true;
}
void BehaviorKeepHoveringWithPidControl::selfLocalizationPoseCallBack(const geometry_msgs::PoseStamped &msg){
  estimated_pose_msg = msg;
}
void BehaviorKeepHoveringWithPidControl::statusCallBack(const droneMsgsROS::droneStatus &msg){
  status_msg = msg;
}

}
PLUGINLIB_EXPORT_CLASS(quadrotor_motion_with_pid_control::BehaviorKeepHoveringWithPidControl, nodelet::Nodelet)