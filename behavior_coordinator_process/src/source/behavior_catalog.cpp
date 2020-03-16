/*!*******************************************************************************************
 *  \file       behavior_catalog.cpp
 *  \brief      BehaviorCatalog implementation file.
 *  \details    This file implements the BehaviorCatalog class.
 *  \authors    Abraham Carrera Groba, Alberto Camporredondo.
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

#define _TAGS_DEFINITION
#define _VARIABLE_DEFINITION
#include "../include/behavior_catalog.h"

BehaviorCatalog::BehaviorCatalog() {}

BehaviorCatalog::~BehaviorCatalog() {}

std::tuple<bool, YAML::Node> BehaviorCatalog::open(std::string path_file)
{
  if (behaviors_loaded.size() > 0)
  {
    std::cout << "[INFO]: configuration already loaded" << std::endl;
    return std::make_tuple(true, config_file); // config_file is the YAML::Node already loaded
  }
  std::cout << "[INFO]: loading configuration" << std::endl;

  std::cout << "[INFO]: behavior_catalog path: " << path_file << std::endl;
  YAML::Node yaml_node;
  try
  {
    yaml_node = YAML::LoadFile(path_file);
    if (yaml_node.IsNull())
    {
      std::cout << ERROR << "[ERROR]: There is a problem with the file" << STOP << std::endl;
      return std::make_tuple(false, yaml_node);
    }
  }
  catch (YAML::Exception exception)
  {
    prettify.printError(exception.what());
    return std::make_tuple(false, yaml_node);
  }
  return std::make_tuple(true, yaml_node);
}

bool BehaviorCatalog::loadConfiguration(std::string file_path)
{ 
  std::cout<<"Catalog: "<<file_path<<std::endl;
  bool fileIsOpen;
  std::tie(fileIsOpen, config_file) = BehaviorCatalog::open(file_path); // config_file file is declared globally

  if (fileIsOpen)
  {
    prettify.printTitle();
    // TODO: dibujar por pantalla inicio carga catalogo
  }
  else
  {
    return false;
  }
  /*Default behavior values*/
  if (config_file[tags.default_values].IsDefined())
  {
    /*Load default_values tag*/
    auto default_values = config_file[tags.default_values];
    prettify.printParent("INFO", "default_values");

    /*Category*/
    variables.default_value.category = default_values[tags.default_value.category].as<std::string>();
    prettify.printChild("INFO", "category: {" + variables.default_value.category + "}");

    /*Default*/
    variables.default_value.default_activation = default_values[tags.default_value.default_activation].as<bool>();
    prettify.printChild("INFO", "default: {" + std::to_string(variables.default_value.default_activation) + "}");

    /*Timeout*/
    variables.default_value.timeout = default_values[tags.default_value.timeout].as<int>();
    prettify.printChild("INFO", "timeout: {" + std::to_string(variables.default_value.timeout) + "}");
  }
  else
  {
    prettify.printParent("WARNING", "Tag {default_behavior_values} is not defined");
  }

  /*behavior descriptors*/
  if (config_file[tags.behavior_descriptors].IsDefined())
  {
    for (auto behavior : config_file[tags.behavior_descriptors])
    {
      /*Behavior name*/
      variables.behavior_descriptor.behavior_name = behavior[tags.behavior_descriptor.behavior_name].as<std::string>();
      prettify.printParent("INFO", "behavior: { " + variables.behavior_descriptor.behavior_name + " }");

      /*timeout*/
      if (behavior[tags.behavior_descriptor.timeout].IsDefined())
      {
        variables.behavior_descriptor.timeout = behavior[tags.behavior_descriptor.timeout].as<int>();
      }
      else
      {
        variables.behavior_descriptor.timeout = variables.default_value.timeout;
      }
      prettify.printChild("INFO", "timeout: {" + std::to_string(variables.behavior_descriptor.timeout) + "}");

      /*category activation*/
      if (behavior[tags.behavior_descriptor.category].IsDefined())
      {
        variables.behavior_descriptor.category = behavior[tags.behavior_descriptor.category].as<std::string>();
      }
      else
      {
        variables.behavior_descriptor.category = variables.default_value.category;
      }
      prettify.printChild("INFO", "category: {" + variables.behavior_descriptor.category + "}");

      /*category activation*/
      if (behavior[tags.behavior_descriptor.system].IsDefined())
      {
        variables.behavior_descriptor.system = behavior[tags.behavior_descriptor.system].as<std::string>();
        prettify.printChild("INFO", "system: {" + variables.behavior_descriptor.system + "}");
      }
      else {
         variables.behavior_descriptor.system = "";
      }

      /*default activation*/
      if (behavior[tags.behavior_descriptor.default_activation].IsDefined())
      {
        variables.behavior_descriptor.default_activation =
            behavior[tags.behavior_descriptor.default_activation].as<bool>();
      }
      else
      {
        variables.behavior_descriptor.default_activation = variables.default_value.default_activation;
      }
      prettify.printChild("INFO",
                          "default: {" + std::to_string(variables.behavior_descriptor.default_activation) + "}");

      /*arguments*/
      if (behavior[tags.behavior_descriptor.arguments].IsDefined())
      {
        for (auto argument : behavior[tags.behavior_descriptor.arguments])
        {
          std::string argument_name = argument[tags.behavior_descriptor.argument.argument_name].as<std::string>();
          std::vector<std::string> allowed_values;
          for (auto allow_value : argument[tags.behavior_descriptor.argument.allowed_values])
          {
            allowed_values.push_back(allow_value.as<std::string>());
          }
          if (argument[tags.behavior_descriptor.argument.dimensions].IsDefined())
          {
            int dimensions = argument[tags.behavior_descriptor.argument.dimensions].as<int>();
            variables.behavior_descriptor.arguments.push_back(
                ArgumentDescriptor(argument_name, allowed_values, dimensions));
          }
          else
          {
            variables.behavior_descriptor.arguments.push_back(ArgumentDescriptor(argument_name, allowed_values, 1));
          }
          prettify.printChild("INFO", "argument_name: {" + argument_name + "}");
        }
      }

      behaviors_loaded.insert(
          {variables.behavior_descriptor.behavior_name,
           BehaviorDescriptor(variables.behavior_descriptor.behavior_name, variables.behavior_descriptor.category,
                              variables.behavior_descriptor.system, variables.behavior_descriptor.default_activation,
                              variables.behavior_descriptor.timeout, std::vector<std::vector<BehaviorDescriptor>>{},
                              std::vector<BehaviorDescriptor>{}, variables.behavior_descriptor.arguments)});

      /*Cleaning variables to avoid possible errors*/
      variables.behavior_descriptor.behavior_name = "";
      variables.behavior_descriptor.category = "";
      variables.behavior_descriptor.default_activation = false;
      variables.behavior_descriptor.timeout = 0;

      variables.behavior_descriptor.arguments.clear();
    }
  }
  else
  {
    prettify.printParent("INFO", "Tag {behavior_descriptors} is not defined");
    return false;
  }

  /*constraints*/
  /*Exclusivity constraints*/
  if (config_file[tags.constraints.exclusivity_constraints].IsDefined())
  {

    prettify.printParent("INFO", "exclusivity_constraints");
    int list_number = 0;
    std::vector<std::vector<BehaviorDescriptor>> exclusion_lists;

    for (auto exclusive_list : config_file[tags.constraints.exclusivity_constraints])
    {
      std::vector<std::string> behavior_list;
      std::vector<BehaviorDescriptor> exclusion_list;

      for (auto behavior : exclusive_list[tags.constraints.exclusivity_constraint.mutually_exclusive])
      {
        behavior_list.push_back(behavior.as<std::string>());

        std::cout << behavior.as<std::string>() << std::endl;
        exclusion_list.push_back(behaviors_loaded.find(behavior.as<std::string>())->second);
      }

      exclusion_lists.push_back(exclusion_list);
      list_number++;
      prettify.printChildList("INFO", "mutually_exclusive " + std::to_string(list_number), behavior_list);
    }

    manageExclusivityConstraints(exclusion_lists);
  }
  else
  {
    prettify.printParent("INFO", "Tag {exclusivity_constraints} is not defined");
  }

  /*precedence_constraint*/
  if (config_file[tags.constraints.precedence_constraints].IsDefined())
  {
    prettify.printParent("INFO", "precedence_constraints");
    int list_number = 0;
    std::vector<std::pair<std::vector<BehaviorDescriptor>, std::vector<BehaviorDescriptor>>> precedence_constraints;
    for (auto precedence_list : config_file[tags.constraints.precedence_constraints])
    {

      std::vector<BehaviorDescriptor> first_behaviors = {};
      std::vector<BehaviorDescriptor> next_behaviors = {};

      for (auto behavior : precedence_list[tags.constraints.precedence_constraint.first])
      {

        first_behaviors.push_back(behaviors_loaded.find(behavior.as<std::string>())->second);
      }
      for (auto behavior : precedence_list[tags.constraints.precedence_constraint.next])
      {

        next_behaviors.push_back(behaviors_loaded.find(behavior.as<std::string>())->second);
      };

      precedence_constraints.push_back(std::make_pair(first_behaviors, next_behaviors));
    }

    managePrecedenceConstraints(precedence_constraints);
  }
  else
  {
    prettify.printParent("INFO", "Tag {precedence_constraints} is not defined");
  }

  prettify.printEnd("Behavior Configuration has been loaded");
  return true;
}

void BehaviorCatalog::manageExclusivityConstraints(std::vector<std::vector<BehaviorDescriptor>> exclusion_lists)
{
  for (auto exclusion_list : exclusion_lists)
  {
    for (auto behavior : exclusion_list)
    {
      behaviors_loaded.find(behavior.getName())->second.setIncompatibilities(exclusion_list);
    }
  }
}

std::vector<BehaviorDescriptor> BehaviorCatalog::getDefaultBehaviors()
{
  std::vector<BehaviorDescriptor> default_behaviors = {};

  for (auto behavior : behaviors_loaded)
  {
    if (behavior.second.isActivatedByDefault())
    {
      default_behaviors.push_back(behavior.second);
    }
  }
  return default_behaviors;
}
void BehaviorCatalog::print()
{
  for (auto behaviorr : behaviors_loaded)
  {
    BehaviorDescriptor behavior = behaviorr.second;
    std::cout << behaviorr.first << std::endl;
    std::cout << behavior.getName() << std::endl;
    std::cout << behavior.getCategory() << std::endl;
    std::cout << behavior.getName() << std::endl;
    std::cout << behavior.isActivatedByDefault() << std::endl;
    std::cout << behavior.getTimeout() << std::endl;
  }
}

void BehaviorCatalog::managePrecedenceConstraints(
    std::vector<std::pair<std::vector<BehaviorDescriptor>, std::vector<BehaviorDescriptor>>> precedence_constraints)
{
  for (auto precedence_constraint : precedence_constraints)
  {
    for (auto behavior : precedence_constraint.second)
    {
      behaviors_loaded.find(behavior.getName())->second.addDisjunctivePrecedences(precedence_constraint.first);
    }
  }
}

void BehaviorCatalog::eraseConfiguration() { behaviors_loaded.clear(); }
std::map<std::string, BehaviorDescriptor> BehaviorCatalog::getBehaviorsInCatalog() { return behaviors_loaded; }

std::vector<BehaviorDescriptor> BehaviorCatalog::getBehaviors()
{
  std::vector<BehaviorDescriptor> available_behaviors;

  for (auto behavior : behaviors_loaded)
  {
    available_behaviors.push_back(behavior.second);
  }

  return available_behaviors;
}

std::tuple<bool, std::string, BehaviorDescriptor> BehaviorCatalog::getBehaviorDescriptor(std::string name)
{
  std::transform(name.begin(), name.end(), name.begin(), ::toupper);
  if (behaviors_loaded.size() <= 0)
  {
    return std::make_tuple(false, "No config loaded", BehaviorDescriptor());
  }

  if (behaviors_loaded.count(name) == 0)
  {
    return std::make_tuple(false, name + " has not been found in the configuration", BehaviorDescriptor());
  }

  return std::make_tuple(true, "", behaviors_loaded.find(name)->second);
}

void BehaviorCatalog::checkBehaviors(ros::V_string available_behaviors, std::string node_namespace)
{
  std::vector<std::string> behaviors;
  std::for_each(available_behaviors.begin(), available_behaviors.end(), [&](std::string available_behaviors) {
    std::string target = "/" + node_namespace + "/";
    int target_size = target.size();
    // If this fails with gcc < 4.9, just use str match with plain /droneX/ string
    if (regex_match(available_behaviors, "\\/drone\\d\\/.*"))
    {
      // Follows /drone1/...
      std::string behavior = available_behaviors.substr(target_size);
      behaviors.push_back(behavior);
    }
    else
    {
      // Does not follows /drone1/...
      std::string behavior = available_behaviors.substr(1);
      behaviors.push_back(behavior);
    }
  });

  std::vector<std::string> behaviors_in_catalog;
  for (auto behavior_pair : behaviors_loaded)
  {
    std::string behavior = behavior_pair.first;
    std::transform(behavior.begin(), behavior.end(), behavior.begin(), ::tolower);
    behavior = "behavior_" + behavior;
    behaviors_in_catalog.push_back(behavior);
  }
  std::vector<std::string> unavailable_behaviors = checkUnavailability(behaviors, behaviors_in_catalog);

  if (unavailable_behaviors.size() > 0)
  {
    prettify.printWarningVector("The following behaviors are set in the catalog but not launched: ",
                                unavailable_behaviors);
  }
}

std::vector<std::string> BehaviorCatalog::checkUnavailability(std::vector<std::string> processes_launched,
                                                              std::vector<std::string> processes_in_catalog)
{
  std::vector<std::string> processes_unavailable;
  for (auto process_in_catalog : processes_in_catalog)
  {
    if (!processIsLaunched(process_in_catalog, processes_launched))
    {
      processes_unavailable.push_back(process_in_catalog);
    }
  }
  return processes_unavailable;
}

bool BehaviorCatalog::processIsLaunched(std::string process_in_catalog, std::vector<std::string> processes_launched)
{
  for (auto process_launched : processes_launched)
  {
    if (process_in_catalog == process_launched)
    {
      return true;
    }
  }
  return false;
}
