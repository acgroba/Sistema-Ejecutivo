/*!*******************************************************************************************
 *  \file       argument_descriptor.cpp
 *  \brief      ArgumentDescriptor implementation file.
 *  \details    This file implements the ArgumentDescriptor class.
 *  \authors    Alberto Camporredondo.
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

#include "../include/argument_descriptor.h"

/*Constructors*/
ArgumentDescriptor::ArgumentDescriptor() {}

ArgumentDescriptor::ArgumentDescriptor(std::string name, std::vector<std::string> allowed_values, int dimensions)
{
  this->name = name;
  this->allowed_values = allowed_values;
  this->dimensions = dimensions;
}

/*Destructor*/
ArgumentDescriptor::~ArgumentDescriptor() {}

/*Functionality*/
bool ArgumentDescriptor::check_name(std::string argument_name) { return argument_name == name; }

std::tuple<bool, std::string> ArgumentDescriptor::check_string_format(std::string argument_value)
{
  bool found = false;
  std::for_each(allowed_values.begin(), allowed_values.end(), [&](std::string allowed_value) {
    if (allowed_value == argument_value)
    {
      found = true;
    }
  });
  if (!found)
    return std::make_tuple(false, "Argument value: \"" + argument_value + "\" not possible ");
  else
    return std::make_tuple(true, "");
}

std::tuple<bool, std::string> ArgumentDescriptor::check_int_format(double number)
{
  int argument_dimension = 1;

  if (argument_dimension != this->dimensions)
  {
    return std::make_tuple(false, "Dimensions \"" + std::to_string(argument_dimension) + "\" do not match ");
  }

  if ((number < atoi(allowed_values[0].c_str())) || (number > atoi(allowed_values[1].c_str())))
  {
    return std::make_tuple(false, "Value \"" + std::to_string(number) + "\" out of range");
  }
  return std::make_tuple(true, "");
}

std::tuple<bool, std::string> ArgumentDescriptor::check_int_format(std::vector<double> argument_values)
{
  int argument_dimension = argument_values.size();

  if (argument_dimension != this->dimensions)
  {
    return std::make_tuple(false, "Dimensions \"" + std::to_string(argument_dimension) + "\" do not match ");
  }

  for (int number : argument_values)
  {
    if ((number < atoi(allowed_values[0].c_str())) || (number > atoi(allowed_values[1].c_str())))
    {
      return std::make_tuple(false, "Value \"" + std::to_string(number) + "\" out of range");
    }
  }
  return std::make_tuple(true, "");
}

std::tuple<bool, std::string> ArgumentDescriptor::check_variable_format(std::vector<std::string> argument_variable)
{
  int argument_dimension = argument_variable.size();

  if (argument_dimension != this->dimensions)
  {
    return std::make_tuple(false, "Dimensions \"" + std::to_string(argument_dimension) + "\" do not match");
  }

  for (std::string variable : argument_variable)
  {
    if (variable.find("+") != std::string::npos)
    {
    }
  }
  return std::make_tuple(true, "");
}

/*Getters*/
std::string ArgumentDescriptor::getName() { return name; }

std::vector<std::string> ArgumentDescriptor::getAllowedValues() { return allowed_values; }

int ArgumentDescriptor::getDimensions() { return dimensions; }

/*Setters*/
void ArgumentDescriptor::setName(std::string name) { this->name = name; }

void ArgumentDescriptor::setAllowedValues(std::vector<std::string> allowed_values)
{
  this->allowed_values = allowed_values;
}

void ArgumentDescriptor::setDimensions(int dimensions) { this->dimensions = dimensions; }
