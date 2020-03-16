/*!*******************************************************************************************
 *  \file       behavior_keep_moving_with_pid_control.cpp
 *  \brief      Behavior Keep Moving implementation file.
 *  \details    This file implements the keep moving class.
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

#include "../include/behavior_keep_moving_with_pid_control.h"
#include <pluginlib/class_list_macros.h>

namespace quadrotor_motion_with_pid_control
{
BehaviorKeepMovingWithPidControl::BehaviorKeepMovingWithPidControl() : BehaviorExecutionController() { 
  setName("keep_moving_with_pid_control"); 
  setExecutionGoal(ExecutionGoals::KEEP_RUNNING); 
}

BehaviorKeepMovingWithPidControl::~BehaviorKeepMovingWithPidControl() {}

void BehaviorKeepMovingWithPidControl::onConfigure()
{
  node_handle = getNodeHandle();
  nspace = getNamespace();  
  node_handle.param<std::string>("self_localization_speed_topic", self_localization_speed_str, "self_localization/speed");
  node_handle.param<std::string>("controllers_topic", command_high_level_str, "command/high_level");
  node_handle.param<std::string>("motion_reference_speed_topic", motion_reference_speed_str,"motion_reference/speed");
  node_handle.param<std::string>("set_control_mode_service_name",set_control_mode_srv,"set_control_mode");
  node_handle.param<std::string>("status_topic",status_str,"status");

  // Start processes
  request_processes_activation_cli =
      node_handle.serviceClient<aerostack_msgs::RequestProcesses>("/" + nspace + "/" + "start_processes");
  request_processes_deactivation_cli =
      node_handle.serviceClient<aerostack_msgs::RequestProcesses>("/" + nspace + "/" + "stop_processes");

  request_processes_srv.request.processes = {"quadrotor_pid_controller_process"};

  //Subscriber
  status_sub = node_handle.subscribe("/" + nspace + "/"+status_str, 1, &BehaviorKeepMovingWithPidControl::statusCallBack, this);
}

bool BehaviorKeepMovingWithPidControl::checkSituation()
{
  //Quadrotor is FLYING
  if (status_msg.status != droneMsgsROS::droneStatus::LANDED){
    return true;
  }else{
    setErrorMessage("Error: Drone is landed");
    std::cout<<"Error: Drone is landed"<<std::endl;
    return false;
  }
}

void BehaviorKeepMovingWithPidControl::checkGoal(){}


void BehaviorKeepMovingWithPidControl::checkProgress() {

    if(received_speed && direction == "LEFT")
    {
        if (!(estimated_speed_msg.twist.linear.x<0 && estimated_speed_msg.twist.linear.y<0.0001)) BehaviorExecutionController::setTerminationCause(aerostack_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
    }
    else if(received_speed && direction == "RIGHT")
    {
        if (!(estimated_speed_msg.twist.linear.x>0 && estimated_speed_msg.twist.linear.y<0.0001)) BehaviorExecutionController::setTerminationCause(aerostack_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
    }
    else if(received_speed && direction == "FORWARD")
    {
        if (!(estimated_speed_msg.twist.linear.x<0.0001 && estimated_speed_msg.twist.linear.y>0)) BehaviorExecutionController::setTerminationCause(aerostack_msgs::BehaviorActivationFinished::WRONG_PROGRESS);

    }
    else if(received_speed && direction == "BACKWARD")
    {
        if (!(estimated_speed_msg.twist.linear.x<0.0001 && estimated_speed_msg.twist.linear.y<0)) BehaviorExecutionController::setTerminationCause(aerostack_msgs::BehaviorActivationFinished::WRONG_PROGRESS);

    }
}

void BehaviorKeepMovingWithPidControl::checkProcesses() 
{ 
 
}

void BehaviorKeepMovingWithPidControl::onExecute() 
{ 
 
}

//Set control mode
bool BehaviorKeepMovingWithPidControl::setControlMode(int new_control_mode){
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

void BehaviorKeepMovingWithPidControl::onActivate()
{

  request_processes_activation_cli.call(request_processes_srv);
  if (!request_processes_srv.response.acknowledge) setErrorMessage("Processes for Behavior: [ " + ros::this_node::getName() + " ] couldn't be started");

  estimated_speed_msg = *ros::topic::waitForMessage<geometry_msgs::TwistStamped>("/" + nspace + "/"+self_localization_speed_str, node_handle, ros::Duration(1));

  //Subscribers
  self_localization_speed_sub = node_handle.subscribe("/" + nspace + "/"+self_localization_speed_str, 1, &BehaviorKeepMovingWithPidControl::selfLocalizationSpeedCallBack, this);
  //Publishers
  motion_reference_speed_pub = node_handle.advertise<geometry_msgs::TwistStamped>("/" + nspace + "/"+motion_reference_speed_str,1, true);
  command_high_level_pub = node_handle.advertise<droneMsgsROS::droneCommand>("/" + nspace + "/"+command_high_level_str, 1, true);
  
  //Service
  setControlModeClientSrv = node_handle.serviceClient<aerostack_msgs::SetControlMode>("/" + nspace + "/"+set_control_mode_srv);
  ros::Duration(4).sleep();
  received_speed = false;
  std_msgs::Header header;

  //Get arguments
  std::string arguments=getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  direction = config_file["direction"].as<std::string>();
  double speed = config_file["speed"].as<double>();  
    if(direction == "LEFT")
    {
      motion_reference_speed.twist.linear.x=-speed;
      motion_reference_speed.twist.linear.y=0;
      motion_reference_speed.twist.linear.z=0;

    }
    else if(direction == "RIGHT")
    {
      motion_reference_speed.twist.linear.x=speed;
      motion_reference_speed.twist.linear.y=0;
      motion_reference_speed.twist.linear.z=0;

    }
    else if(direction == "FORWARD")
    {
      motion_reference_speed.twist.linear.x=0;
      motion_reference_speed.twist.linear.y=speed;
      motion_reference_speed.twist.linear.z=0;
    }
    else if(direction == "BACKWARD")
    {
      motion_reference_speed.twist.linear.x=0;
      motion_reference_speed.twist.linear.y=-speed;
      motion_reference_speed.twist.linear.z=0;
    }

    setControlMode(aerostack_msgs::QuadrotorPidControllerMode::SPEED);
    motion_reference_speed_pub.publish(motion_reference_speed);
    header.frame_id = "behavior_keep_moving_with_pid_control";  
    high_level_command.header = header;
    high_level_command.command = droneMsgsROS::droneCommand::MOVE;
    command_high_level_pub.publish(high_level_command); 
}

void BehaviorKeepMovingWithPidControl::onDeactivate()
{
  std_msgs::Header header;
  header.frame_id = "behavior_keep_moving_with_pid_control";

  droneMsgsROS::droneCommand msg;
  msg.header = header;
  msg.command = droneMsgsROS::droneCommand::HOVER;
  motion_reference_speed.twist.linear.x=0;
  motion_reference_speed.twist.linear.y=0;
  motion_reference_speed.twist.linear.z=0;
  motion_reference_speed_pub.publish(motion_reference_speed);
  command_high_level_pub.publish(msg);

  request_processes_deactivation_cli.call(request_processes_srv);
  if (!request_processes_srv.response.acknowledge) setErrorMessage("Processes for Behavior: [ " + ros::this_node::getName() + " ] couldn't be stopped");

  setControlModeClientSrv.shutdown();
  self_localization_speed_sub.shutdown();
  command_high_level_pub.shutdown();
  motion_reference_speed_pub.shutdown();
}
void BehaviorKeepMovingWithPidControl::selfLocalizationSpeedCallBack(const geometry_msgs::TwistStamped &msg){
  estimated_speed_msg = msg; received_speed = true;
}
void BehaviorKeepMovingWithPidControl::statusCallBack(const droneMsgsROS::droneStatus &msg){
  status_msg = msg;
}
}
PLUGINLIB_EXPORT_CLASS(quadrotor_motion_with_pid_control::BehaviorKeepMovingWithPidControl, nodelet::Nodelet)
