/*!*******************************************************************************************
 *  \file       behavior_coordinator_process.cpp
 *  \brief      BehaviorCoordinator implementation file.
 *  \details    This file implements the BehaviorCoordinator class.
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

#include "../include/behavior_coordinator_process.h"

/*Constructor*/
BehaviorCoordinator::BehaviorCoordinator() { uid = 0; }

/*Destructor*/
BehaviorCoordinator::~BehaviorCoordinator() {}

/*
---------------
 Robot Process
---------------
*/
void BehaviorCoordinator::ownSetUp()
{
  ros::NodeHandle private_nh;

  private_nh.param<std::string>("drone_id", drone_id, "1");
  private_nh.param<std::string>("drone_id_namespace", drone_id_namespace, "drone" + drone_id);
  private_nh.param<std::string>("my_stack_directory", my_stack_directory,
                                "~/workspace/ros/quadrotor_stack_catkin/src/quadrotor_stack");
  private_nh.getParam("behavior_catalog_path", behavior_catalog_path);
  private_nh.param<std::string>("activate_behavior_srv", activate_behavior_str, "request_behavior_activation");
  private_nh.param<std::string>("deactivate_behavior_srv", deactivate_behavior_str, "request_behavior_deactivation");
  private_nh.param<std::string>("consult_available_behaviors_srv", consult_available_behaviors_str,
                                "consult_available_behaviors");
  private_nh.param<std::string>("consult_incompatible_behaviors_srv", consult_incompatible_behaviors_str,
                                "consult_incompatible_behaviors");
  private_nh.param<std::string>("behavior_activation_finished_topic", behavior_activation_finished_str,
                                "behavior_activation_finished");
  private_nh.param<std::string>("list_of_active_behaviors_topic", list_of_active_behaviors_str,
                                "list_of_active_behaviors");
  private_nh.param<std::string>("check_behavior_format_srv", check_behavior_format_str, "check_behavior_format");

  private_nh.param<std::string>("execution_request_topic", execution_request_str, "execution_request");

  bool catalog_loaded = behavior_catalog.loadConfiguration(behavior_catalog_path);

  if (!catalog_loaded)
  {
   // std::cout << "[ERROR]: Behavior catalog has not been loaded correctly" << std::endl;
  }
  else
  {
    ros::V_string available_nodes;
    ros::master::getNodes(available_nodes);
    behavior_catalog.checkBehaviors(available_nodes, drone_id_namespace);
    catalog = behavior_catalog.getBehaviorsInCatalog();
    default_behaviors = behavior_catalog.getDefaultBehaviors();

    //std::cout << "----------------------------------AVAILABLE BEHAVIORS----------------------------------" << std::endl;

    for (std::map<std::string, BehaviorDescriptor>::iterator it = catalog.begin(); it != catalog.end(); it++)
    {
     // std::cout << (*it).first << std::endl;
    }
  }

  behavior_activation_changes.setUp();
}

void BehaviorCoordinator::ownStart()
{
  activate_behavior_srv = node_handle.advertiseService("/" + drone_id_namespace + "/" + activate_behavior_str,
                                                       &BehaviorCoordinator::activateBehaviorCallback, this);
  deactivate_behavior_srv = node_handle.advertiseService("/" + drone_id_namespace + "/" + deactivate_behavior_str,
                                                         &BehaviorCoordinator::deactivateBehaviorCallback, this);
  consult_available_behaviors_srv = node_handle.advertiseService(
      consult_available_behaviors_str, &BehaviorCoordinator::consultAvailableBehaviorsCallback, this);
  consult_incompatible_behaviors_srv = node_handle.advertiseService(
      consult_incompatible_behaviors_str, &BehaviorCoordinator::consultIncompatibleBehaviorsCallback, this);
  behavior_activation_finished_sub =
      node_handle.subscribe("/" + drone_id_namespace + "/" + behavior_activation_finished_str, 100,
                            &BehaviorCoordinator::behaviorActivationFinishedCallback, this);
  list_of_active_behaviors_pub =
      node_handle.advertise<aerostack_msgs::ListOfBehaviors>(list_of_active_behaviors_str, 1);
 execution_request_pub =
      node_handle.advertise<aerostack_msgs::ExecutionRequest>("/" + drone_id_namespace + "/" +execution_request_str, 1);
  check_behavior_format_srv =
      node_handle.advertiseService(check_behavior_format_str, &BehaviorCoordinator::checkBehaviorFormatCallback, this);

  activate_default();
  publishActiveBehaviorsList();
}

void BehaviorCoordinator::ownStop()
{
  activate_behavior_srv.shutdown();
  deactivate_behavior_srv.shutdown();
  consult_available_behaviors_srv.shutdown();
  behavior_activation_finished_sub.shutdown();
  list_of_active_behaviors_pub.shutdown();
}

void BehaviorCoordinator::ownRun() {}

/*
-----------
 Funcionality
-----------
*/

bool BehaviorCoordinator::activate(BehaviorDescriptor behavior, std::string arguments, int priority)
{  
   
   
 
 // std::cout << "    >>>>>>>>>>>>>>>>>ACTIVATING BEHAVIOR>>>>>>>>>>>>>>>>> " << behavior.getName() << "[" << arguments
     //       << "]" << std::endl;
  if (behavior_activation_changes.isActive(behavior))
  { failure_cause="Behavior is already active";
   // std::cout << "      ···················BEHAVIOR IS ALREADY ACTIVE···················" << std::endl;
    return false;
  }
  else
  {
    behavior_activation_changes.setActivationList(std::vector<std::tuple<BehaviorDescriptor, std::string, int>>{});
    behavior_activation_changes.setDeactivationList(std::vector<BehaviorDescriptor>{});

    bool activation_result = behavior_activation_changes.updateListsToActivateBehavior(behavior, arguments, priority);

    if (activation_result)
    {
    //  std::cout << "      ············TRYING TO ACTIVATE COLATERAL ACTIVATIONS············" << std::endl;
      activate_all(behavior_activation_changes.getActivationList());
     // std::cout << "      ············TRYING TO DEACTIVATE COLATERAL DEACTIVATIONS········" << std::endl;
      deactivate_all(behavior_activation_changes.getDeactivationList());
      print_active_behaviors();
      return true;
    }
    else {
    print_active_behaviors();
    return false;
    }
    
  }
}

bool BehaviorCoordinator::deactivate(int uid)
{  
 
 // std::cout << "    >>>>>>>>>>>>>>>>>DEACTIVATING BEHAVIOR>>>>>>>>>>>>>>>>>" << uid << std::endl;

  behavior_activation_changes.setDeactivationList(std::vector<BehaviorDescriptor>{});
  BehaviorDescriptor behavior = behavior_activation_changes.getBehaviorDescriptorByUid(uid);

  if (!behavior_activation_changes.isActive(behavior))
  { failure_cause="Behavior is not active";
  //  std::cout << "      ···················BEHAVIOR IS NOT ACTIVE···················" << std::endl;
    return false;
  }
  else
  {

    bool deactivation_result = behavior_activation_changes.updateListsToDeactivateBehavior(behavior);

    if (deactivation_result)
    {
      deactivate_all(behavior_activation_changes.getDeactivationList());
      print_active_behaviors();
      return true;
    }
    else 
    {
    print_active_behaviors();
    return false;
    }
  }
}

void BehaviorCoordinator::activate_all(std::vector<std::tuple<BehaviorDescriptor, std::string, int>> behaviors)
{
  for (std::vector<std::tuple<BehaviorDescriptor, std::string, int>>::iterator it = behaviors.begin();
       it != behaviors.end(); it++)
  {
    BehaviorDescriptor behavior = std::get<0>((*it));
    std::string arguments = std::get<1>((*it));
    int priority = std::get<2>((*it));
    startBehavior(behavior, arguments, priority);
  }
}

void BehaviorCoordinator::deactivate_all(std::vector<BehaviorDescriptor> behaviors)
{ 
  for (std::vector<BehaviorDescriptor>::iterator it = behaviors.begin(); it != behaviors.end(); it++)
  { 
    BehaviorDescriptor behavior = *it;

    stopBehavior(behavior);
  }
}

void BehaviorCoordinator::activate_default()
{
  // std::cout << "    >>>>>>>>>>>>>>ACTIVATING DEFAULT BEHAVIORS>>>>>>>>>>>>>>" << std::endl;
  for (std::vector<BehaviorDescriptor>::iterator it = default_behaviors.begin(); it != default_behaviors.end(); it++)
  {
    BehaviorDescriptor default_behavior = *it;
    activate(default_behavior, "", 1);
  }
}

void BehaviorCoordinator::deactivate_activation_conditions_not_present()
{
  
  for (auto active_behavior : behavior_activation_changes.getActiveBehaviors())
  {
    BehaviorDescriptor behavior;
    behavior = std::get<0>(active_behavior.second);
    std::pair<bool, std::string> situation = behavior_activation_changes.checkActivationConditions(behavior);
    bool result = situation.first;
    if (!result) 
    { //std::cout << "    >>>DEACTIVATING BEHAVIOR "<<behavior.getName()<<" BECAUSE ACTIVATION CONDITIONS NOT PRESENT ANYMORE>>>" << std::endl;

     deactivate( std::get<2>(active_behavior.second));
    }
    
  }
}


void BehaviorCoordinator::print_active_behaviors()
{
  // std::cout << "----------------------------------ACTIVE BEHAVIORS----------------------------------" << std::endl;
   std::map<std::string, std::tuple<BehaviorDescriptor, int, int,std::string>>  active_behaviors =
      behavior_activation_changes.getActiveBehaviors();
  for (  std::map<std::string, std::tuple<BehaviorDescriptor, int, int,std::string>> ::iterator it = active_behaviors.begin();
       it != active_behaviors.end(); it++)
  {
    BehaviorDescriptor active_behavior = std::get<0>((*it).second);
   //  std::cout << active_behavior.getName() << "[" << std::get<2>((*it).second) << "] with priority " << std::get<1>((*it).second)
     //         << std::endl;
  }
}

void BehaviorCoordinator::publishActiveBehaviorsList()
{
  std::map<std::string, std::tuple<BehaviorDescriptor, int, int,std::string>> active_behaviors= behavior_activation_changes.getActiveBehaviors();
  aerostack_msgs::ListOfBehaviors active_behaviors_msg;

  for (auto active_behavior : active_behaviors)
  {
   
    	aerostack_msgs::BehaviorCommand behavior_command;
    	behavior_command.name = active_behavior.first;
    	behavior_command.arguments = std::get<3>(active_behavior.second);
    	active_behaviors_msg.behavior_commands.push_back(behavior_command);
    	active_behaviors_msg.behaviors.push_back(behavior_command.name);
  
  } 

  list_of_active_behaviors_pub.publish(active_behaviors_msg);
}

std::tuple<bool, std::string, double> BehaviorCoordinator::startBehavior(BehaviorDescriptor behavior,
                                                                         std::string arguments, int priority)
{ 
  aerostack_msgs::ExecutionRequest execution_request_msg;
  execution_request_msg.behavior_command.name=behavior.getName();
  execution_request_msg.behavior_command.arguments=arguments;
  execution_request_msg.behavior_command.priority=priority;
  execution_request_msg.request_type=aerostack_msgs::ExecutionRequest::ACTIVATE;

  std::string behavior_name_lowercase = behavior.getName();
  std::transform(behavior_name_lowercase.begin(), behavior_name_lowercase.end(), behavior_name_lowercase.begin(),
                 ::tolower);

 std::string behavior_path;
  if(behavior.getSystem() != "") {
   behavior_path = "/" + drone_id_namespace + "/" + behavior.getSystem() + "/behavior_" +
                              behavior_name_lowercase + "/activate_behavior";
  }
  else {
   behavior_path = "/" + drone_id_namespace +  "/behavior_" +
                              behavior_name_lowercase + "/activate_behavior";
  }

  ros::ServiceClient behavior_cli = node_handle.serviceClient<aerostack_msgs::ActivateBehavior>(behavior_path);

  aerostack_msgs::ActivateBehavior activate_behavior_msg;
  activate_behavior_msg.request.arguments = arguments;
  activate_behavior_msg.request.timeout = behavior.getTimeout();
	
  uid += 1;
  if (!behavior_cli.call(activate_behavior_msg))
  { failure_cause="Failed behavior execution controller activation";
    execution_request_msg.success=false;
    execution_request_msg.failure_cause=failure_cause;
    execution_request_pub.publish(execution_request_msg);
    failure_cause="";
   // std::cout << "             !!!!COULDN'T ACTIVATE BEHAVIOR!!!!" << std::endl;
    return std::make_tuple(false, "Behavior: [" + behavior.getName() + "] is not launched", 0);
  }
     
    execution_request_msg.success=true;
    execution_request_pub.publish(execution_request_msg);
  if (activate_behavior_msg.response.ack)
  {

    behavior_activation_changes.addActiveBehavior(behavior, uid, priority, arguments);
  }

  return std::make_tuple(activate_behavior_msg.response.ack, activate_behavior_msg.response.error_message, uid);
}

std::tuple<bool, std::string> BehaviorCoordinator::stopBehavior(BehaviorDescriptor behavior)
{  aerostack_msgs::ExecutionRequest execution_request_msg;
   int uid = behavior_activation_changes.getUid(behavior);
   execution_request_msg.behavior_command.name=behavior.getName();
   execution_request_msg.behavior_command.arguments= behavior_activation_changes.getBehaviorArgumentsByUid(uid);
   execution_request_msg.behavior_command.priority= behavior_activation_changes.getBehaviorPriorityByUid(uid);
   execution_request_msg.request_type=aerostack_msgs::ExecutionRequest::DEACTIVATE;
  std::string behavior_name = behavior.getName();
  std::transform(behavior_name.begin(), behavior_name.end(), behavior_name.begin(), ::tolower);
   std::string behavior_path;

     if(behavior.getSystem() != "") {

   behavior_path = "/" + drone_id_namespace + "/" + behavior.getSystem() + "/behavior_" + behavior_name + "/deactivate_behavior";
       }
  else {
   behavior_path = "/" + drone_id_namespace +  "/behavior_" +
                              behavior_name + "/deactivate_behavior";
  }


  ros::ServiceClient behavior_cli = node_handle.serviceClient<aerostack_msgs::DeactivateBehavior>(behavior_path);

  aerostack_msgs::DeactivateBehavior deactivate_msg;
  if (!behavior_cli.call(deactivate_msg)){
    failure_cause="Behavior: [" + behavior.getName() + "] is not launched";
    execution_request_msg.success=false;
    execution_request_msg.failure_cause=failure_cause;
    execution_request_pub.publish(execution_request_msg);
    failure_cause="";
    return std::make_tuple(false, "Behavior: [" + behavior.getName() + "] is not launched");
    }

  if (!deactivate_msg.response.ack) {
    failure_cause=deactivate_msg.response.error_message;
    execution_request_msg.success=false;
    execution_request_msg.failure_cause=failure_cause;
    execution_request_pub.publish(execution_request_msg);
    failure_cause="";
    return std::make_tuple(false, deactivate_msg.response.error_message);
}
    execution_request_msg.success=true;
    execution_request_pub.publish(execution_request_msg);

  behavior_activation_changes.removeActiveBehavior(uid);

  return std::make_tuple(true, "");
}

BehaviorActivationChanges BehaviorCoordinator::getBehaviorActivationChanges() { return behavior_activation_changes; }

/*
-----------
 Callbacks
-----------
*/
bool BehaviorCoordinator::activateBehaviorCallback(aerostack_msgs::RequestBehaviorActivation::Request &request,
                                                   aerostack_msgs::RequestBehaviorActivation::Response &response)
{ 

  BehaviorDescriptor behavior;
  for (std::map<std::string, BehaviorDescriptor>::iterator it = catalog.begin(); it != catalog.end(); it++)
  {
    if ((*it).first == request.behavior.name)
    {
      behavior = (*it).second;
    }
  }

  if (activate(behavior, request.behavior.arguments, request.behavior.priority))
  {
    activate_default();
    response.ack = true;
  }

  else
  {
   // std::cout << "          !!!!!!COULD NOT ACTIVATE!!!!!!" << behavior.getName() << std::endl;
    print_active_behaviors();
    response.ack = false;
  }

  publishActiveBehaviorsList();
  return true;
}

bool BehaviorCoordinator::deactivateBehaviorCallback(aerostack_msgs::RequestBehaviorDeactivation::Request &request,
                                                     aerostack_msgs::RequestBehaviorDeactivation::Response &response)
{
  int uid=-1;
  if (request.name != "") {

    BehaviorDescriptor behavior;
    behavior.setName(request.name);
    uid= behavior_activation_changes.getUid(behavior);
  }
  else {
     uid=request.behavior_uid;
  }
  if (deactivate(uid))
  {
    activate_default();
  }
  else
  {
   // std::cout << "          !!!!!!COULD NOT DEACTIVATE!!!!!!" << request.behavior_uid << std::endl;
    print_active_behaviors();
  }
  publishActiveBehaviorsList();
  return true;
}

bool BehaviorCoordinator::consultAvailableBehaviorsCallback(
    aerostack_msgs::ConsultAvailableBehaviors::Request &request,
    aerostack_msgs::ConsultAvailableBehaviors::Response &response)
{
  aerostack_msgs::ListOfBehaviors available_behaviors_msg;
  std::vector<BehaviorDescriptor> available_behaviors;

  available_behaviors = behavior_catalog.getBehaviors();

  std::for_each(available_behaviors.begin(), available_behaviors.end(),
                [&](BehaviorDescriptor behavior) { available_behaviors_msg.behaviors.push_back(behavior.getName()); });

  response.available_behaviors = available_behaviors_msg;
}

bool BehaviorCoordinator::consultIncompatibleBehaviorsCallback(
    aerostack_msgs::ConsultIncompatibleBehaviors::Request &request,
    aerostack_msgs::ConsultIncompatibleBehaviors::Response &response)
{
  bool found;
  std::string error_message;
  BehaviorDescriptor behavior_descriptor;
  std::tie(found, error_message, behavior_descriptor) = behavior_catalog.getBehaviorDescriptor(request.behavior.name);
  std::map<std::string, std::tuple<BehaviorDescriptor, int, int,std::string>> active_behaviors =  behavior_activation_changes.getActiveBehaviors();
  if (not found)
  {
    response.ack = false;
    response.error_message = "Behavior not found";
    return true;
  }

  std::vector<BehaviorDescriptor> incompatibilities = behavior_descriptor.getIncompatibilities();
  for (auto incompatibility : incompatibilities)
  {
    for (auto active_behavior : active_behaviors)
    {
      if (active_behavior.first == incompatibility.getName())
      {
        aerostack_msgs::BehaviorCommand behavior_command;
        behavior_command.name = active_behavior.first;
        behavior_command.arguments = std::get<3>(active_behavior.second);

        response.incompatible_behaviors.push_back(behavior_command);
        response.uids.push_back(std::get<1>(active_behavior.second));
      }
    }
  }

  response.ack = true;
  response.error_message = "";
  return true;
}

bool BehaviorCoordinator::checkBehaviorFormatCallback(aerostack_msgs::CheckBehaviorFormat::Request &request,
                                                      aerostack_msgs::CheckBehaviorFormat::Response &response)
{
  bool found;
  std::string error_message;
  BehaviorDescriptor behavior;
  std::tie(found, error_message, behavior) = behavior_catalog.getBehaviorDescriptor(request.behavior.name);

  if (!found)
  {
    response.ack = false;
    response.error_message = error_message;
    return true;
  }

  std::string argument = request.behavior.arguments;
  argument="\""+argument+"\"";
  YAML::Node node;
  try
  {
    node = YAML::Load(argument);
  }
  catch (YAML::Exception exception)
  {
   // std::cout << "Argument does not follow YAML standards " << std::endl;
    response.ack = false;
    response.error_message = "Arguments for behavior \"" + request.behavior.name + "\" does not follow YAML standards ";
    return true;
  }

  for (auto tag : node)
  {
    auto argument_name = tag.first.as<std::string>();

    ArgumentDescriptor argument_descriptor;
    bool found = false;
    for (ArgumentDescriptor argument : behavior.getArguments())
    {
      if (argument.check_name(argument_name))
      {
        argument_descriptor.setName(argument.getName());
        argument_descriptor.setAllowedValues(argument.getAllowedValues());
        argument_descriptor.setDimensions(argument.getDimensions());
        found = true;
      }
    }

    if (!found)
    {
      std::string error_message =
          "Incorrect argument \"" + argument_name + "\" for behavior \"" + request.behavior.name + "\"";
      response.ack = false;
      response.error_message = error_message;
      return true;
    }

    bool correct_format;
    std::string error_message;
    if (argument_name == "direction")
    {
      std::tie(correct_format, error_message) =
          argument_descriptor.check_string_format(node[argument_name].as<std::string>());
    }
    else
    {
      std::vector<std::string> params = node[argument_name].as<std::vector<std::string>>();
      bool has_variable = false;
      for (auto t : params)
      {
        
        if (t.find("+") != std::string::npos)
        {
          has_variable = true;
        }
      }

      if (has_variable)
      {
        std::tie(correct_format, error_message) = argument_descriptor.check_variable_format(params);
      }
      else
      {
        try
        {
          std::vector<double> coordinates = node[argument_name].as<std::vector<double>>();
          std::tie(correct_format, error_message) = argument_descriptor.check_int_format(coordinates);
        }
        catch (YAML::Exception e)
        {
          int number = node[argument_name].as<double>();
          std::tie(correct_format, error_message) = argument_descriptor.check_int_format(number);
        }
      }
    }

    if (!correct_format)
    {
      response.ack = false;
      response.error_message = error_message + "for behavior \"" + request.behavior.name + "\"";
      return true;
    }
  }
  response.ack = true;
  response.error_message = "";
  return true;
}

void BehaviorCoordinator::behaviorActivationFinishedCallback(const aerostack_msgs::BehaviorActivationFinished &message)
{
  // std::cout << "Behavior Activation Finished: " << message.name << " [" << message.termination_cause << "]"
      //      << std::endl;
  
  BehaviorDescriptor behavior;
  behavior.setName(message.name);
  if (behavior_activation_changes.isActive(behavior))
  {
    int uid = behavior_activation_changes.getUid(behavior);
    aerostack_msgs::ExecutionRequest execution_request_msg;
    execution_request_msg.behavior_command.name=behavior.getName();
    execution_request_msg.behavior_command.arguments= behavior_activation_changes.getBehaviorArgumentsByUid(uid);
    execution_request_msg.behavior_command.priority= behavior_activation_changes.getBehaviorPriorityByUid(uid);
    execution_request_msg.request_type=aerostack_msgs::ExecutionRequest::ACTIVATION_FINISHED;
    execution_request_msg.success=true;
    execution_request_pub.publish(execution_request_msg);
    behavior_activation_changes.removeActiveBehavior(uid);
  }
  deactivate_activation_conditions_not_present();
  activate_default();
  publishActiveBehaviorsList();
  print_active_behaviors();
}
