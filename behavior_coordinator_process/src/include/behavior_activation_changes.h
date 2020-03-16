/*!*********************************************************************************
 *  \file       behavior_activation_changes.h
 *  \brief      BehaviorActivationChanges definition file.
 *  \details    This file contains the BehaviorActivationChanges declaration.
 *              To obtain more information about it's definition consult
 *              the behavior_activation_changes.cpp file.
 *  \authors    Abraham Carrera Groba.
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

#ifndef BEHAVIOR_ACTIVATION_CHANGES_H
#define BEHAVIOR_ACTIVATION_CHANGES_H

/*GNU/Limux*/
#include <iterator>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

/*ROS*/
#include <ros/ros.h>

/*Aerostack framework*/
#include <behavior_descriptor.h>

/*Aerostack Messages*/
#include <aerostack_msgs/CheckSituation.h>

/*Class definition*/
class BehaviorActivationChanges
{
public: /*Constructor & Destructor*/
  BehaviorActivationChanges();
  BehaviorActivationChanges(std::vector<std::tuple<BehaviorDescriptor, std::string, int>> activation_list,
                            std::vector<BehaviorDescriptor> deactivation_list);
  ~BehaviorActivationChanges();

private: /*Process variables*/
  std::vector<std::tuple<BehaviorDescriptor, std::string, int>> activation_list;
  std::vector<BehaviorDescriptor> deactivation_list;
  std::map<std::string, std::tuple<BehaviorDescriptor, int, int,std::string>> active_behaviors;

  /*Communication variables*/
  ros::NodeHandle node_handle;
  std::string drone_id;
  std::string drone_id_namespace;
  std::string check_activation_conditions_str;
  std::string failure_cause;
  bool result;

public: /*Functionality*/
  /*Process functions*/
  bool updateListsToDeactivateBehavior(BehaviorDescriptor behavior);
  bool updateListsToActivateBehavior(BehaviorDescriptor behavior, std::string argugetments, int priority);
  std::pair<bool, std::string> checkActivationConditions(BehaviorDescriptor behavior);
  void setUp();
  /*Getters*/
  std::vector<std::tuple<BehaviorDescriptor, std::string, int>> getActivationList();
  std::vector<BehaviorDescriptor> getDeactivationList();
  std::map<std::string, std::tuple<BehaviorDescriptor, int, int,std::string>> getActiveBehaviors();
  int getPriority(BehaviorDescriptor behavior);
  bool isActive(BehaviorDescriptor behavior);
  bool isActiveByUid(int uid);
  int getUid(BehaviorDescriptor behavior);
  std::string getFailureCause();
  std::string getArguments(BehaviorDescriptor behavior);
  BehaviorDescriptor getBehaviorDescriptorByUid(int uid);
  std::string getBehaviorArgumentsByUid(int uid);
  int getBehaviorPriorityByUid(int uid);
  /*Setters*/
  void setActivationList(std::vector<std::tuple<BehaviorDescriptor, std::string, int>> activation_list);
  void setDeactivationList(std::vector<BehaviorDescriptor> deactivation_list);
  void addActiveBehavior(BehaviorDescriptor behavior, int uid, int priority,std::string arguments);
  void removeActiveBehavior(int uid);
};

#endif
