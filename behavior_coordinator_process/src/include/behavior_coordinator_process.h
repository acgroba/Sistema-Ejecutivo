/*!*********************************************************************************
 *  \file       behavior_coordinator_process.h
 *  \brief      BehaviorCoordinator definition file.
 *  \details    This file contains the BehaviorCoordinator declaration. To obtain more information about
 *              it's definition consult the behavior_coordinator_process.cpp file.
 *  \authors    Abraham Carrera Groba
 *  \copyright  Copyright (c) 2019 Universidad Politecnica de Madrid
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

#ifndef BEHAVIOR_COORDINATOR_PROCESS_H
#define BEHAVIOR_COORDINATOR_PROCESS_H

/*GNU/Linux*/
#include <chrono>
#include <thread>
#include <tuple>
#include <tuple>
#include <vector>

/*ROS*/
#include <ros/ros.h>

/*Aerostack Messages*/
#include <aerostack_msgs/ActivateBehavior.h>
#include <aerostack_msgs/BehaviorActivationFinished.h>
#include <aerostack_msgs/BehaviorCommandPriority.h>
#include <aerostack_msgs/CheckBehaviorFormat.h>
#include <aerostack_msgs/ConsultAvailableBehaviors.h>
#include <aerostack_msgs/ConsultIncompatibleBehaviors.h>
#include <aerostack_msgs/DeactivateBehavior.h>
#include <aerostack_msgs/ListOfBehaviors.h>
#include <aerostack_msgs/RequestBehaviorActivation.h>
#include <aerostack_msgs/RequestBehaviorDeactivation.h>
#include <aerostack_msgs/ExecutionRequest.h>
#include <std_msgs/Float64.h>

/*Aerostack framework*/
#include <argument_descriptor.h>
#include <behavior_activation_changes.h>
#include <behavior_catalog.h>
#include <behavior_descriptor.h>
#include <robot_process.h>
#include <ctime>
/*Class definition*/
class BehaviorCoordinator : public RobotProcess
{
private: /*Process variables*/
  std::map<std::string, BehaviorDescriptor> catalog;
  std::vector<BehaviorDescriptor> default_behaviors;
  int uid;
  BehaviorActivationChanges behavior_activation_changes;
  BehaviorCatalog behavior_catalog;


  /*Communication variables*/
  ros::NodeHandle node_handle;

  std::string activate_behavior_str;
  std::string deactivate_behavior_str;
  std::string behavior_activation_finished_str;
  std::string list_of_active_behaviors_str;
  std::string consult_available_behaviors_str;
  std::string consult_incompatible_behaviors_str;
  std::string check_behavior_format_str;
  std::string execution_request_str;

  std::string drone_id;
  std::string drone_id_namespace;
  std::string my_stack_directory;
  std::string behavior_catalog_path;
  std::string failure_cause;

  ros::ServiceServer activate_behavior_srv;
  ros::ServiceServer deactivate_behavior_srv;
  ros::ServiceServer consult_available_behaviors_srv;
  ros::ServiceServer consult_incompatible_behaviors_srv;
  ros::ServiceServer check_behavior_format_srv;
  ros::Subscriber behavior_activation_finished_sub;
  ros::Publisher list_of_active_behaviors_pub;
  ros::Publisher execution_request_pub;
public: /*Constructor & Destructor*/
  BehaviorCoordinator();
  ~BehaviorCoordinator();

private: /*RobotProcess*/
  void ownSetUp();
  void ownStart();
  void ownRun();
  void ownStop();

private: /*Process functions*/
  bool activate(BehaviorDescriptor behavior, std::string arguments, int priority);
  bool deactivate(int uid);
  void activate_all(std::vector<std::tuple<BehaviorDescriptor, std::string, int>> behaviors);
  void deactivate_all(std::vector<BehaviorDescriptor> behaviors);
  void activate_default();
  void deactivate_activation_conditions_not_present();
  void print_active_behaviors();
  void updateActiveBehaviorsList();
  void publishActiveBehaviorsList();
  std::tuple<bool, std::string, double> startBehavior(BehaviorDescriptor behavior, std::string arguments, int priority);
  std::tuple<bool, std::string> stopBehavior(BehaviorDescriptor behavior);

public: /*Callbacks*/
  bool activateBehaviorCallback(aerostack_msgs::RequestBehaviorActivation::Request &,
                                aerostack_msgs::RequestBehaviorActivation::Response &);
  bool deactivateBehaviorCallback(aerostack_msgs::RequestBehaviorDeactivation::Request &,
                                  aerostack_msgs::RequestBehaviorDeactivation::Response &);
  void behaviorActivationFinishedCallback(const aerostack_msgs::BehaviorActivationFinished &message);
  bool consultAvailableBehaviorsCallback(aerostack_msgs::ConsultAvailableBehaviors::Request &,
                                         aerostack_msgs::ConsultAvailableBehaviors::Response &);
  bool consultIncompatibleBehaviorsCallback(aerostack_msgs::ConsultIncompatibleBehaviors::Request &,
                                            aerostack_msgs::ConsultIncompatibleBehaviors::Response &);
  bool checkBehaviorFormatCallback(aerostack_msgs::CheckBehaviorFormat::Request &,
                                   aerostack_msgs::CheckBehaviorFormat::Response &);

  /*Process functions*/
  BehaviorActivationChanges getBehaviorActivationChanges();
};

#endif
