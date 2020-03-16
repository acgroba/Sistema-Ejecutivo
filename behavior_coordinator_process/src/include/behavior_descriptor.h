/*!*********************************************************************************
 *  \file       behavior_descriptor.h
 *  \brief      BehaviorDescriptor definition file.
 *  \details    This file contains the BehaviorCatalog declaration.
 *              To obtain more information about it's definition consult
 *              the behavior_descriptor.cpp file.
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

#ifndef BEHAVIOR_DESCRIPTOR_H
#define BEHAVIOR_DESCRIPTOR_H

/*GNU/Limux*/
#include <string>
#include <vector>

/*Messages*/
#include <aerostack_msgs/BehaviorCommand.h>

#include <argument_descriptor.h>

/*Class definition*/
class BehaviorDescriptor
{
public: /*Constructor & Destructor*/
  BehaviorDescriptor();
  BehaviorDescriptor(std::string name, std::string category, std::string system, bool default_activation, int timeout,
                     std::vector<std::vector<BehaviorDescriptor>> disjunction_of_precedences,
                     std::vector<BehaviorDescriptor> incompatibilities, std::vector<ArgumentDescriptor> arguments);
  ~BehaviorDescriptor();
  bool operator==(BehaviorDescriptor const &rhs) const { return this->name == rhs.name; }
  bool operator<(BehaviorDescriptor const &rhs) const { return this->name < rhs.name; }
private: /*Process variables*/
  std::string name;
  std::string category;
  std::string system;
  bool default_activation;
  int timeout;
  std::vector<std::vector<BehaviorDescriptor>> disjunction_of_precedences;
  std::vector<BehaviorDescriptor> incompatibilities;
  std::vector<ArgumentDescriptor> arguments;

public: /*Process functions*/
  /*Functionality*/
  aerostack_msgs::BehaviorCommand serialize();

  /*Getters*/
  std::string getName();
  std::string getCategory();
  std::string getSystem();
  bool isActivatedByDefault();
  int getTimeout();
  std::vector<std::vector<BehaviorDescriptor>> getDisjunctionOfPrecedences();
  std::vector<BehaviorDescriptor> getIncompatibilities();
  std::vector<ArgumentDescriptor> getArguments();

  /*Setters*/
  void setName(std::string name);
  void setCategory(std::string category);
  void setSystem(std::string system);
  void setDefaultActivation(bool default_activation);
  void setTimeout(int timeout);
  void setDisjunctionOfPrecedences(std::vector<std::vector<BehaviorDescriptor>> disjunction_of_precedences);
  void addDisjunctivePrecedences(std::vector<BehaviorDescriptor> disjunctive_precedences);
  void setIncompatibilities(std::vector<BehaviorDescriptor> incompatibilities);
  void setArguments(std::vector<ArgumentDescriptor> arguments);
};

#endif
