/*!*******************************************************************************************
 *  \file       behavior_follow_path.cpp
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

#include "../include/behavior_follow_path.h"
#include <pluginlib/class_list_macros.h>

namespace quadrotor_motion_with_pid_control
{
BehaviorFollowPath::BehaviorFollowPath() : BehaviorExecutionController() { 
  setName("follow_path");
  setExecutionGoal(ExecutionGoals::ACHIEVE_GOAL);
}

BehaviorFollowPath::~BehaviorFollowPath() {}

void BehaviorFollowPath::onConfigure()
{
  node_handle = getNodeHandle();
  nspace = getNamespace();
  node_handle.param<std::string>("estimated_speed_topic", self_localization_speed_str, "self_localization/speed");
  node_handle.param<std::string>("estimated_pose_topic", self_localization_pose_str, "self_localization/pose");
  node_handle.param<std::string>("controllers_topic", command_high_level_str, "command/high_level");
  node_handle.param<std::string>("motion_reference_speed_topic", motion_reference_speed_str,"motion_reference/speed");
  node_handle.param<std::string>("motion_reference_pose_topic", motion_reference_pose_str,"motion_reference/pose");
  node_handle.param<std::string>("motion_reference_path_topic", motion_reference_path_str,"motion_reference/path");
  
  node_handle.param<std::string>("set_control_mode_service_name",set_control_mode_srv,"set_control_mode");
  node_handle.param<std::string>("status_topic",status_str,"status");

  // Start processes
  request_processes_activation_cli =
      node_handle.serviceClient<aerostack_msgs::RequestProcesses>("/" + nspace + "/" + "start_processes");
  request_processes_deactivation_cli =
      node_handle.serviceClient<aerostack_msgs::RequestProcesses>("/" + nspace + "/" + "stop_processes");

  request_processes_srv.request.processes = {"quadrotor_pid_controller_process","path_tracker_process"};

  //Subscriber
  status_sub = node_handle.subscribe("/" + nspace + "/"+status_str, 1, &BehaviorFollowPath::statusCallBack, this);
}

bool BehaviorFollowPath::checkSituation()
{
  //Quadrotor is FLYING
  if (status_msg.status == droneMsgsROS::droneStatus::LANDED){
    setErrorMessage("Error: Drone is landed");
    std::cout<<"Error: Drone is landed"<<std::endl;
    return false;
  }
return true;
}

void BehaviorFollowPath::checkGoal(){ 
  if(abs(sqrt(pow(last_path_point.pose.position.x-estimated_pose_msg.pose.position.x,2)+
              pow(last_path_point.pose.position.y-estimated_pose_msg.pose.position.y,2)+
              pow(last_path_point.pose.position.z-estimated_pose_msg.pose.position.z,2)))<0.1 && checkQuadrotorStopped() && path_index == reference_path.poses.size()){
    //ros::Duration(5).sleep();
    BehaviorExecutionController::setTerminationCause(aerostack_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
  } 
}

void BehaviorFollowPath::onExecute()
{
  
}

void BehaviorFollowPath::checkProgress() {
  if (!execute) BehaviorExecutionController::setTerminationCause(aerostack_msgs::BehaviorActivationFinished::WRONG_PROGRESS);

  //Quadrotor is too far from the target and it is not moving
  last_target_pose = current_target_pose;
  float targets_distance = abs(sqrt(pow(last_target_pose.pose.position.x-current_target_pose.pose.position.x,2)+pow(last_target_pose.pose.position.y-current_target_pose.pose.position.y,2)+pow(last_target_pose.pose.position.z-current_target_pose.pose.position.z,2)));
  float quadrotor_distance = abs(sqrt(pow(current_target_pose.pose.position.x-estimated_pose_msg.pose.position.x,2)+pow(current_target_pose.pose.position.y-estimated_pose_msg.pose.position.y,2)+pow(current_target_pose.pose.position.z-estimated_pose_msg.pose.position.z,2)));
  if(path_index > 1 && targets_distance > quadrotor_distance * 2 && checkQuadrotorStopped()) BehaviorExecutionController::setTerminationCause(aerostack_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
}

//Set control mode
bool BehaviorFollowPath::setControlMode(int new_control_mode){
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



void BehaviorFollowPath::onActivate()
{
  request_processes_activation_cli.call(request_processes_srv);
  if (!request_processes_srv.response.acknowledge) setErrorMessage("Processes for Behavior: [ " + ros::this_node::getName() + " ] couldn't be started");
  //estimated_speed_msg = *ros::topic::waitForMessage<geometry_msgs::TwistStamped>("/" + nspace + "/"+self_localization_speed_str, node_handle, ros::Duration(1));
  //Subscribers
  self_localization_speed_sub = node_handle.subscribe("/" + nspace + "/"+self_localization_speed_str, 1, &BehaviorFollowPath::selfLocalizationSpeedCallBack, this);
  self_localization_pose_sub = node_handle.subscribe("/" + nspace + "/"+self_localization_pose_str, 1, &BehaviorFollowPath::selfLocalizationPoseCallBack, this);
  motion_reference_pose_sub = node_handle.subscribe("/" + nspace + "/"+motion_reference_pose_str, 1, &BehaviorFollowPath::motionReferencePoseCallBack, this);
  //Publishers
  motion_reference_speed_pub = node_handle.advertise<geometry_msgs::TwistStamped>("/" + nspace + "/"+motion_reference_speed_str,1, true);
  command_high_level_pub = node_handle.advertise<droneMsgsROS::droneCommand>("/" + nspace + "/"+command_high_level_str, 1, true);
  

  //Service
  setControlModeClientSrv = node_handle.serviceClient<aerostack_msgs::SetControlMode>("/" + nspace + "/"+set_control_mode_srv);

  path_index = 0;
  execute = true;

  geometry_msgs::TwistStamped motion_reference_speed;

  //Checks if path distance is not too long and argument is defined
  std::string arguments = getParameters();
  YAML::Node config_file = YAML::Load(arguments);
  if(config_file["path"].IsDefined()){
    std::vector<std::vector<double>> points=config_file["path"].as<std::vector<std::vector<double>>>();
    geometry_msgs::PoseStamped path_point;
    float distance = 0;
    //First point
    path_point.pose.position.x = points[0][0];
    path_point.pose.position.y = points[0][1];
    path_point.pose.position.z = points[0][2];
    reference_path.poses.push_back(path_point);    
    for(int i=1;i<points.size();i++){
      path_point.pose.position.x = points[i][0];
      path_point.pose.position.y = points[i][1];
      path_point.pose.position.z = points[i][2];
      reference_path.poses.push_back(path_point);
      distance += abs(sqrt(pow(points[i-1][0]-points[i][0],2)+pow(points[i-1][1]-points[i][1],2)+pow(points[i-1][2]-points[i][2],2)));
    }
    if (distance > MAX_DISTANCE){
      setErrorMessage("Error: Path is too long");
      std::cout<<"Error: Path is too long"<<std::endl;    
      execute = false;     
      return; 
    }
    //Last point
    last_path_point.pose.position.x = points[points.size()-1][0];
    last_path_point.pose.position.y = points[points.size()-1][1];
    last_path_point.pose.position.z = points[points.size()-1][2];
  }else{
    setErrorMessage("Error: Path is not defined");
    std::cout<<"Error: Path is not defined"<<std::endl;    
    execute = false;   
    return;
  }

  //Hover
  motion_reference_speed.twist.linear.x = 0.0;
  motion_reference_speed.twist.linear.y = 0.0;
  motion_reference_speed.twist.linear.z = 0.0;
  motion_reference_speed.twist.angular.x = 0.0;
  motion_reference_speed.twist.angular.y = 0.0;
  motion_reference_speed.twist.angular.z = 0.0;
  motion_reference_speed_pub.publish(motion_reference_speed);
  //Change controller mode
  setControlMode(aerostack_msgs::QuadrotorPidControllerMode::SPEED_3D);
  //Send trajectory
  reference_path.header.frame_id = "behavior_follow_path";
  motion_reference_path_pub = node_handle.advertise<nav_msgs::Path>("/" + nspace + "/"+motion_reference_path_str, 1,true);
  motion_reference_path_pub.publish(reference_path);
  //MOVE
  high_level_command.header.frame_id = "behavior_follow_path";
  high_level_command.command = droneMsgsROS::droneCommand::MOVE;
  command_high_level_pub.publish(high_level_command);
}

void BehaviorFollowPath::onDeactivate()
{
  droneMsgsROS::droneCommand msg;
  msg.header.frame_id = "behavior_follow_path";
  msg.command = droneMsgsROS::droneCommand::HOVER;
  motion_reference_speed.twist.linear.x=0;
  motion_reference_speed.twist.linear.y=0;
  motion_reference_speed.twist.linear.z=0;
  motion_reference_speed_pub.publish(motion_reference_speed);
  command_high_level_pub.publish(msg);

  request_processes_deactivation_cli.call(request_processes_srv);
  if (!request_processes_srv.response.acknowledge) setErrorMessage("Processes for Behavior: [ " + ros::this_node::getName() + " ] couldn't be stopped");
  
  reference_path.poses={};
  setControlModeClientSrv.shutdown();
  self_localization_speed_sub.shutdown();
  command_high_level_pub.shutdown();
  motion_reference_speed_pub.shutdown();
}

bool BehaviorFollowPath::checkQuadrotorStopped()
{
  if (received_speed){
    if (abs(estimated_speed_msg.twist.linear.x) <= 0.15 && abs(estimated_speed_msg.twist.linear.y) <= 0.15 && abs(estimated_speed_msg.twist.linear.z) <= 0.15){
        return true;
    }else{
      return false;
    }    
  }else{
    return false;
  }
}

void BehaviorFollowPath::checkProcesses() 
{ 
 
}


// Callbacks
void BehaviorFollowPath::selfLocalizationSpeedCallBack(const geometry_msgs::TwistStamped &msg){ 
  estimated_speed_msg = msg; 
  received_speed = true;
}
void BehaviorFollowPath::selfLocalizationPoseCallBack(const geometry_msgs::PoseStamped &msg){
  estimated_pose_msg = msg;
}
void BehaviorFollowPath::motionReferencePoseCallBack(const geometry_msgs::PoseStamped &msg){
  if (current_target_pose.pose.position.x != msg.pose.position.x || current_target_pose.pose.position.y != msg.pose.position.y || current_target_pose.pose.position.z != msg.pose.position.z ){
      last_target_pose = current_target_pose;
      current_target_pose = msg;
      path_index++;
  }
}
void BehaviorFollowPath::statusCallBack(const droneMsgsROS::droneStatus &msg){
  status_msg = msg;
}

}
PLUGINLIB_EXPORT_CLASS(quadrotor_motion_with_pid_control::BehaviorFollowPath, nodelet::Nodelet)
