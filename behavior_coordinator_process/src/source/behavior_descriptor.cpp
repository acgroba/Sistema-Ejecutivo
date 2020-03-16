/*!*******************************************************************************************
 *  \file       behavior_descriptor.cpp
 *  \brief      BehaviorDescriptor implementation file.
 *  \details    This file implements the BehaviorDescriptor class.
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

#include "../include/behavior_descriptor.h"

/*Constructors*/
BehaviorDescriptor::BehaviorDescriptor() {}

BehaviorDescriptor::BehaviorDescriptor(std::string name, std::string category, std::string system,
                                       bool default_activation, int timeout,
                                       std::vector<std::vector<BehaviorDescriptor>> disjunction_of_precedences,
                                       std::vector<BehaviorDescriptor> incompatibilities,
                                       std::vector<ArgumentDescriptor> arguments)
{
  this->name = name;
  this->category = category;
  this->system = system;
  this->default_activation = default_activation;
  this->timeout = timeout;
  this->disjunction_of_precedences = disjunction_of_precedences;
  this->incompatibilities = incompatibilities;
  this->arguments = arguments;
}

/*Destructor*/
BehaviorDescriptor::~BehaviorDescriptor() {}

/*Functionality*/
aerostack_msgs::BehaviorCommand BehaviorDescriptor::serialize()
{
  aerostack_msgs::BehaviorCommand behavior_msg;
  behavior_msg.name = name;

  return behavior_msg;
}

/*Getters*/
std::string BehaviorDescriptor::getName() { return name; }

std::string BehaviorDescriptor::getCategory() { return category; }

std::string BehaviorDescriptor::getSystem() { return system; }

bool BehaviorDescriptor::isActivatedByDefault() { return default_activation; }

int BehaviorDescriptor::getTimeout() { return timeout; }

std::vector<std::vector<BehaviorDescriptor>> BehaviorDescriptor::getDisjunctionOfPrecedences()
{
  return disjunction_of_precedences;
}

std::vector<ArgumentDescriptor> BehaviorDescriptor::getArguments() { return arguments; }

std::vector<BehaviorDescriptor> BehaviorDescriptor::getIncompatibilities() { return incompatibilities; }

/*Setters*/
void BehaviorDescriptor::setName(std::string name) { this->name = name; }
void BehaviorDescriptor::setSystem(std::string system) { this->system = system; }
void BehaviorDescriptor::setDefaultActivation(bool default_activation)
{
  this->default_activation = default_activation;
}

void BehaviorDescriptor::setDisjunctionOfPrecedences(
    std::vector<std::vector<BehaviorDescriptor>> disjunction_of_precedences)
{
  this->disjunction_of_precedences = disjunction_of_precedences;
}

void BehaviorDescriptor::addDisjunctivePrecedences(std::vector<BehaviorDescriptor> disjunctive_precedences)
{
  int size = disjunction_of_precedences.size();

  if (size > 0)
  {
    for (auto disjunctive_precedence : disjunctive_precedences)
    {

      disjunction_of_precedences.resize(2 * size);
      std::copy_n(disjunction_of_precedences.begin(), size, disjunction_of_precedences.begin() + size);
      for (auto conjunction_of_precedences : disjunction_of_precedences)
      {
        conjunction_of_precedences.push_back(disjunctive_precedence);
      }
    }
  }
  else
  {
    for (auto disjunctive_precedence : disjunctive_precedences)
    {
      disjunction_of_precedences.push_back(std::vector<BehaviorDescriptor>{disjunctive_precedence});
    }
  }
}
void BehaviorDescriptor::setIncompatibilities(std::vector<BehaviorDescriptor> incompatibilities)
{
  this->incompatibilities = incompatibilities;
}
