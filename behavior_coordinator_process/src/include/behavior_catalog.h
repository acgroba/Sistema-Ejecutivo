/*!*********************************************************************************
 *  \file       behavior_catalog.h
 *  \brief      BehaviorCatalog definition file.
 *  \details    This file contains the BehaviorCatalog declaration.
 *              To obtain more information about it's definition consult
 *              the behavior_catalog.cpp file.
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

#ifndef BEHAVIOR_CATALOG_H
#define BEHAVIOR_CATALOG_H

/*Gnu/Linux*/
#include <algorithm>
#include <cstdlib>
#include <map>
#include <string>
#include <tuple>
#include <utility>

// On gcc < 4.9 regex implementation is not stable
// see:
//  https://stackoverflow.com/questions/37411117/
//  terminate-called-after-throwing-an-instance-of-stdregex-error
// So, if gcc < 4.9 fallback to boost regexes
#ifndef __GNUC__
#include <boost/regex.hpp>
#elif __GNUC_PREREQ(4, 9)
#include <regex>
#else
#include <boost/regex.hpp>
#endif

/*ROS*/
#include <yaml-cpp/yaml.h>

/*Aerostack*/
#include "../include/behavior_descriptor.h"
#include "../include/prettify.h"

struct precedence_constraints_t
{
  std::vector<std::string> first;
  std::vector<std::string> next;
};

/*Class definition*/
class BehaviorCatalog
{
public: /*Constructor & Destructor*/
  BehaviorCatalog();
  ~BehaviorCatalog();

private: /*Process variables*/
  std::map<std::string, BehaviorDescriptor> behaviors_loaded;
  std::map<int, std::vector<std::string>> exclusivity_constraints;
  std::map<int, precedence_constraints_t> precedence_constraints;
  YAML::Node config_file;
  Prettify prettify;

public: /*Process functions*/
  /*Process Functionality*/
  bool loadConfiguration(std::string path_file);
  void eraseConfiguration();
  void print();
  void manageExclusivityConstraints(std::vector<std::vector<BehaviorDescriptor>> exclusion_lists);
  void managePrecedenceConstraints(
      std::vector<std::pair<std::vector<BehaviorDescriptor>, std::vector<BehaviorDescriptor>>> precedence_constraints);
  void checkBehaviors(ros::V_string available_behaviors, std::string node_namespace);
  std::vector<std::string> checkUnavailability(std::vector<std::string> processes_launched,
                                               std::vector<std::string> processes_in_catalog);
  bool processIsLaunched(std::string process_in_catalog, std::vector<std::string> processes_launched);

  /*Getters*/
  std::vector<BehaviorDescriptor> getDefaultBehaviors();
  std::vector<BehaviorDescriptor> getBehaviors();
  std::map<std::string, BehaviorDescriptor> getBehaviorsInCatalog();
  std::tuple<bool, std::string, BehaviorDescriptor> getBehaviorDescriptor(std::string name);

private:
  std::tuple<bool, YAML::Node> open(std::string path_file);

  bool regex_match(std::string str, std::string test)
  {
    bool match = false;
#ifndef __GNUC__
    boost::regex expr(test);
    match = boost::regex_match(str, expr);
#elif __GNUC_PREREQ(4, 9)
    match = std::regex_match(str, std::regex(test));
#else
    boost::regex expr(test);
    match = boost::regex_match(str, expr);
#endif
    return match;
  }
};

#ifdef _TAGS_DEFINITION
/*TAGS*/
struct tags_t
{
  /*TAGS: default_values*/
  std::string default_values = "default_values";
  struct default_value_t
  {
    std::string category = "category";
    std::string timeout = "timeout";
    std::string default_activation = "default";
  } default_value;

  /*TAGS: behavior_descriptors*/
  std::string behavior_descriptors = "behavior_descriptors";
  struct behavior_descriptor_t
  {
    std::string behavior_name = "behavior";
    std::string category = "category";
    std::string system = "system";
    std::string default_activation = "default";
    std::string timeout = "timeout";

    std::string arguments = "arguments";

    struct argument_t
    {
      std::string argument_name = "argument";
      std::string allowed_values = "allowed_values";
      std::string dimensions = "dimensions";
    } argument;
  } behavior_descriptor;

  /*TAGS: Constraints*/
  struct constraints_t
  {
    std::string exclusivity_constraints = "exclusivity_constraints";
    struct exclusivity_constraints_t
    {
      std::string mutually_exclusive = "mutually_exclusive";
    } exclusivity_constraint;

    std::string precedence_constraints = "precedence_constraints";
    struct precedence_constraints_t
    {
      std::string first = "active";
      std::string next = "before";
    } precedence_constraint;
  } constraints;

} tags;

#endif

#ifdef _VARIABLE_DEFINITION
/*Variables*/
struct variables_t
{
  /*Catalog variables: default_values*/
  struct default_value_variables_t
  {
    std::string category;
    bool default_activation;
    int timeout;
  } default_value;

  /*Catalog variables: behavior_descriptors*/
  struct behavior_descriptor_t
  {
    std::string behavior_name;
    std::string category;
    std::string system;
    bool default_activation;
    int timeout;
    std::vector<ArgumentDescriptor> arguments;
  } behavior_descriptor;

} variables;
#endif

#endif
