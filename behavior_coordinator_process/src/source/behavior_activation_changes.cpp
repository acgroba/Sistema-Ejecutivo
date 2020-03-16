/*!*******************************************************************************************
 *  \file       behavior_activation_changes.cpp
 *  \brief      BehaviorActivationChanges implementation file.
 *  \details    This file implements the BehaviorActivationChanges class.
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

#include "../include/behavior_activation_changes.h"

/*Constructors*/
BehaviorActivationChanges::BehaviorActivationChanges()
{
  this->activation_list = {};
  this->deactivation_list = {};
}

BehaviorActivationChanges::BehaviorActivationChanges(
    std::vector<std::tuple<BehaviorDescriptor, std::string, int>> activation_list,
    std::vector<BehaviorDescriptor> deactivation_list)
{
  this->activation_list = activation_list;
  this->deactivation_list = deactivation_list;
}

/*Destructor*/

BehaviorActivationChanges::~BehaviorActivationChanges() {}

/*Functionality*/


void BehaviorActivationChanges::setUp()
{
  ros::NodeHandle private_nh;
  private_nh.param<std::string>("drone_id", drone_id, "1");
  private_nh.param<std::string>("drone_id_namespace", drone_id_namespace, "drone" + drone_id);
  private_nh.param<std::string>("check_activation_conditions_cli", check_activation_conditions_str,
                                "check_activation_conditions");
}

bool BehaviorActivationChanges::updateListsToActivateBehavior(BehaviorDescriptor behavior, std::string arguments,
                                                              int priority)
{ 
    result = true;

  activation_list.push_back(std::make_tuple(behavior, arguments, priority));

  std::pair<bool, std::string> situation = checkActivationConditions(behavior);
  result = situation.first;
  if (result)
  {
    std::vector<BehaviorDescriptor> incompatibilities = behavior.getIncompatibilities();

    for (std::vector<BehaviorDescriptor>::iterator it = incompatibilities.begin(); it != incompatibilities.end(); it++)
    {

      BehaviorDescriptor conflict = *it;
      
      if (isActive(conflict))
      {
        
        if (getPriority(conflict) <= priority)
        { 
          if (!this->updateListsToDeactivateBehavior(conflict))
          { failure_cause="Could not deactivate incompatible behavior";
            result = false;
          }
        }
        else
        { failure_cause="Incompatible behavior with more priority present";
          result = false;
        }
      }
    }
    if (result == true)
    {
      std::vector<std::vector<BehaviorDescriptor>> disjunction_of_precedences = behavior.getDisjunctionOfPrecedences();

      bool conjunction_of_precedences_condition_satisfied = false;

      for (std::vector<std::vector<BehaviorDescriptor>>::iterator it = disjunction_of_precedences.begin();
           it != disjunction_of_precedences.end() && !conjunction_of_precedences_condition_satisfied; it++)
      {
        std::vector<BehaviorDescriptor> conjunction_of_precedences = *it;

        std::vector<std::tuple<BehaviorDescriptor, std::string, int>> temp_activation_list = activation_list;

        std::vector<BehaviorDescriptor> temp_deactivation_list = deactivation_list;

        conjunction_of_precedences_condition_satisfied = true;

        for (std::vector<BehaviorDescriptor>::iterator it2 = conjunction_of_precedences.begin();
             it2 != conjunction_of_precedences.end() && conjunction_of_precedences_condition_satisfied; it2++)
        {
          BehaviorDescriptor precedent_behavior = *it2;

          if (!isActive(precedent_behavior))
          {
            conjunction_of_precedences_condition_satisfied =
                updateListsToActivateBehavior(precedent_behavior, "", priority);
          }
        }

        if (!conjunction_of_precedences_condition_satisfied)
        { failure_cause="Precedences conditions could not be satisfied";
          this->setActivationList(temp_activation_list);
          this->setDeactivationList(temp_deactivation_list);
        }
      }
    }
  }
  else
  {   failure_cause="Activation conditions not present";
    //std::cout << "      !!!!!!!!ACTIVATION CONDITIONS NOT PRESENT!!!!!!!!" << std::endl;
  }

  return result;
}

bool BehaviorActivationChanges::updateListsToDeactivateBehavior(BehaviorDescriptor behavior)
{
   result = true;
  deactivation_list.push_back(behavior);

  for (std::map<std::string, std::tuple<BehaviorDescriptor, int, int,std::string>>::iterator it = active_behaviors.begin();
       result && it != active_behaviors.end(); it++)
  {
    BehaviorDescriptor active_behavior =std::get<0>((*it).second);

    std::vector<std::vector<BehaviorDescriptor>> active_behavior_disjunction_of_precedences =
        active_behavior.getDisjunctionOfPrecedences();

    bool all_behavior_in_conjunction_active = true;
    bool behavior_is_precedence = false;

    for (std::vector<std::vector<BehaviorDescriptor>>::iterator it2 =
             active_behavior_disjunction_of_precedences.begin();
         (it2 != active_behavior_disjunction_of_precedences.end()) &&
         !(all_behavior_in_conjunction_active && behavior_is_precedence);
         it2++)
    {
      std::vector<BehaviorDescriptor> active_behavior_conjunction_of_precedences = *it2;

      for (std::vector<BehaviorDescriptor>::iterator it3 = active_behavior_conjunction_of_precedences.begin();
           it3 != active_behavior_conjunction_of_precedences.end(); it3++)
      {

        BehaviorDescriptor active_behavior_precedence = *it3;

        if (active_behavior_precedence == behavior)
        {
          behavior_is_precedence = true;
        }
        else if (!isActive(active_behavior_precedence))
        {
          all_behavior_in_conjunction_active = false;
        }
      }
    }
    if (all_behavior_in_conjunction_active && behavior_is_precedence)
    {
      if (getPriority(active_behavior) > getPriority(behavior))
      {  failure_cause="Could not deactivate behavior with more priority";
        result = false;
      }
      else
      {

        updateListsToDeactivateBehavior(active_behavior);
      }
    }
  }
 
  return result;
}
std::pair<bool, std::string> BehaviorActivationChanges::checkActivationConditions(BehaviorDescriptor behavior)
{
  
  std::string behavior_name_lowercase = behavior.getName();
  std::transform(behavior_name_lowercase.begin(), behavior_name_lowercase.end(), behavior_name_lowercase.begin(),
                 ::tolower);


    std::string behavior_path;
  if (behavior.getSystem() != "") {
   behavior_path = "/drone" + drone_id + "/" + behavior.getSystem() + "/behavior_" +
                              behavior_name_lowercase + "/" + check_activation_conditions_str;
  }  else {
    behavior_path = "/drone" + drone_id + "/behavior_" +
                              behavior_name_lowercase + "/" + check_activation_conditions_str;
  }

  aerostack_msgs::CheckSituation activation_conditions_msg;
  ros::ServiceClient check_activation_conditions_cli =
      node_handle.serviceClient<aerostack_msgs::CheckSituation>(behavior_path);


  if (!check_activation_conditions_cli.call(activation_conditions_msg))
  {
    return std::make_pair(false, "         !!!!!!behavior [" + behavior.getName() +
                                     "] does not meet activation conditions!!!!!!");
  }
  return std::make_pair(activation_conditions_msg.response.situation_occurs, activation_conditions_msg.response.error_message);
}

bool BehaviorActivationChanges::isActive(BehaviorDescriptor behavior)
{

  return active_behaviors.find(behavior.getName()) !=active_behaviors.end();
}

bool BehaviorActivationChanges::isActiveByUid(int uid)
{

  bool result = false;

  for (std::map<std::string, std::tuple<BehaviorDescriptor, int, int,std::string>>::iterator it = active_behaviors.begin();
       it != active_behaviors.end() && !result; it++)
  {
     int active_uid = std::get<2>((*it).second);

    if (active_uid == uid)
    {
      result = true;
      break;
    }
  }
  return result;
}

int BehaviorActivationChanges::getPriority(BehaviorDescriptor behavior)
{

  return   std::get<1>(active_behaviors.find(behavior.getName())->second);

}


int BehaviorActivationChanges::getUid(BehaviorDescriptor behavior)
{

 return std::get<2>(active_behaviors.find(behavior.getName())->second);
}

std::string BehaviorActivationChanges::getArguments(BehaviorDescriptor behavior)
{

 return std::get<3>(active_behaviors.find(behavior.getName())->second);
}


std::string BehaviorActivationChanges::getBehaviorArgumentsByUid(int uid)
{
  std::string arguments;

  for (std::map<std::string, std::tuple<BehaviorDescriptor, int, int,std::string>>::iterator it = active_behaviors.begin();
       it != active_behaviors.end(); it++)
  {
     int active_uid = std::get<2>((*it).second);

    if (active_uid == uid)
    {
       arguments=std::get<3>((*it).second);
       break;
    }
  }
  return arguments;
}

int BehaviorActivationChanges::getBehaviorPriorityByUid(int uid)
{
  int priority;

  for (std::map<std::string, std::tuple<BehaviorDescriptor, int, int,std::string>>::iterator it = active_behaviors.begin();
       it != active_behaviors.end(); it++)
  {
     int active_uid = std::get<2>((*it).second);

    if (active_uid == uid)
    {
       priority=std::get<1>((*it).second);
       break;
    }
  }
  return priority;
}

BehaviorDescriptor BehaviorActivationChanges::getBehaviorDescriptorByUid(int uid)
{
  BehaviorDescriptor behavior;

  for (std::map<std::string, std::tuple<BehaviorDescriptor, int, int,std::string>>::iterator it = active_behaviors.begin();
       it != active_behaviors.end(); it++)
  {
     int active_uid = std::get<2>((*it).second);

    if (active_uid == uid)
    {
       behavior=std::get<0>((*it).second);
       break;
    }
  }
  return behavior;
}
void BehaviorActivationChanges::addActiveBehavior(BehaviorDescriptor behavior, int uid, int priority, std::string arguments)
{
  active_behaviors.insert({behavior.getName(), std::make_tuple(behavior, priority,uid,arguments)});
}

void BehaviorActivationChanges::removeActiveBehavior(int uid)
{
  for (std::map<std::string, std::tuple<BehaviorDescriptor, int, int,std::string>> ::iterator it = active_behaviors.begin();
       it != active_behaviors.end(); ++it)
  {
    int active_uid = std::get<2>((*it).second);
    if (uid == active_uid)
    {
      active_behaviors.erase((*it).first);
      break;
    }
  }
}

/*Getters*/
std::vector<std::tuple<BehaviorDescriptor, std::string, int>> BehaviorActivationChanges::getActivationList()
{
  return activation_list;
}

std::vector<BehaviorDescriptor> BehaviorActivationChanges::getDeactivationList() { return deactivation_list; }

  std::map<std::string, std::tuple<BehaviorDescriptor, int, int,std::string>>  BehaviorActivationChanges::getActiveBehaviors()
{
  return active_behaviors;
}

std::string  BehaviorActivationChanges::getFailureCause()
{
  return failure_cause;
}



/*Setters*/
void BehaviorActivationChanges::setActivationList(
    std::vector<std::tuple<BehaviorDescriptor, std::string, int>> activation_list)
{
  this->activation_list = activation_list;
}

void BehaviorActivationChanges::setDeactivationList(std::vector<BehaviorDescriptor> deactivation_list)
{
  this->deactivation_list = deactivation_list;
}
