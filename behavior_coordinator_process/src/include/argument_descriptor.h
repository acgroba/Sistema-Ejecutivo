/*!*********************************************************************************
 *  \file       argument_descriptor.h
 *  \brief      ArgumentDescriptor definition file.
 *  \details    This file contains the ArgumentDescriptor declaration.
 *              To obtain more information about it's definition consult
 *              the argument_descriptor.cpp file.
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

#ifndef ARGUMENT_DESCRIPTOR_H
#define ARGUMENT_DESCRIPTOR_H

/*GNU/Linux*/
#include <algorithm>
#include <cstddef>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

/*Class definition*/
class ArgumentDescriptor
{
public: /*Constructor & Destructor*/
  ArgumentDescriptor();
  ArgumentDescriptor(std::string, std::vector<std::string>, int);
  ~ArgumentDescriptor();

private: /*Process variables*/
  std::string name;
  std::vector<std::string> allowed_values;
  int dimensions;

public: /*Process functions*/
  /*Functionality*/
  bool check_name(std::string);
  std::tuple<bool, std::string> check_string_format(std::string argument_values);
  std::tuple<bool, std::string> check_int_format(std::vector<double> argument_values);
  std::tuple<bool, std::string> check_int_format(double argument_value);
  std::tuple<bool, std::string> check_variable_format(std::vector<std::string> argument_variable);

  /*Getters*/
  std::string getName();
  std::vector<std::string> getAllowedValues();
  int getDimensions();

  /*Setters*/
  void setName(std::string);
  void setAllowedValues(std::vector<std::string>);
  void setDimensions(int);
};

#endif
