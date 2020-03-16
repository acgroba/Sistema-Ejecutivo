/*!*********************************************************************************
 *  \file       behavior_request_operator_assistance.h
 *  \brief      BehaviorRequestOperatorAssistance definition file.
 *  \details    This file contains the BehaviorRequestOperatorAssistance declaration. To obtain more information about
 *              it's definition consult the behavior_request_operator_assistance.cpp file.
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
#ifndef BEHAVIOR_REQUEST_OPERATOR_ASSISTANCE_H
#define BEHAVIOR_REQUEST_OPERATOR_ASSISTANCE_H

// System
#include <iostream>
#include <map>
#include <signal.h>
#include <string>
#include <tuple>
#include <yaml-cpp/yaml.h>

// Qt
#include <QApplication>

// ROS
#include <ros/ros.h>

// Aerostack libraries
#include <behavior_execution_controller.h>

// Msgs

#include <aerostack_msgs/BehaviorEvent.h>
#include <aerostack_msgs/AddBelief.h>
#include <nodelet/nodelet.h>

class OperatorAssistanceWidget;

class BehaviorRequestOperatorAssistance : public BehaviorExecutionController
{
public:
  struct Behavior
  {
    std::string name;
    std::string arguments;
  };
  void onExecute();
  void onConfigure();
public:
  BehaviorRequestOperatorAssistance(QApplication *);
  ~BehaviorRequestOperatorAssistance();

  void addBelief(std::string belief);
  bool belief_added;
private:
  std::string add_belief_str;

  ros::ServiceClient add_belief_srv;

  aerostack_msgs::AddBelief::Request add_belief_msg;

  OperatorAssistanceWidget *operator_widget;

  QApplication *app;

  // BehaviorProcess
private:
  void onActivate();
  void onDeactivate();
  bool checkSituation();
  void checkGoal();
  void checkProgress();
  void checkProcesses();
};

#endif
