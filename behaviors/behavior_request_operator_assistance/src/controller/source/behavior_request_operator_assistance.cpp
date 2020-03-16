
/*!*******************************************************************************************
 *  \file       behavior_request_operator_assistance.cpp
 *  \brief      BehaviorRequestOperatorAssistance implementation file.
 *  \details    This file implements the BehaviorRequestOperatorAssistance class.
 *  \authors    Abraham Carrera.
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

#include <behavior_request_operator_assistance.h>
#include <operator_assistance_widget.h>
#include <nodelet/nodelet.h>

BehaviorRequestOperatorAssistance::BehaviorRequestOperatorAssistance(QApplication *app) : BehaviorExecutionController()
{
  this->app = app;
   setName("request_operator_assistance");
}

BehaviorRequestOperatorAssistance::~BehaviorRequestOperatorAssistance() {}

void BehaviorRequestOperatorAssistance::onConfigure()
{   ros::NodeHandle nh = getNodeHandle();
  nh.param<std::string>("add_belief_srv", add_belief_str, "add_belief");
}

void BehaviorRequestOperatorAssistance::onActivate()
{ 
  ros::NodeHandle nh = getNodeHandle();
  std::string nspace = getNamespace(); 
  // Activate communications
  add_belief_srv = nh.serviceClient<aerostack_msgs::AddBelief>("/" + nspace + "/" +add_belief_str);
}

void BehaviorRequestOperatorAssistance::onDeactivate()
{ 
  add_belief_srv.shutdown();
}

void BehaviorRequestOperatorAssistance::onExecute()
{
    std::string args = getParameters();
    YAML::Node node = YAML::Load(args);

    std::string question = node["question"].as<std::string>();
    std::string belief_predicate_name = node["belief_predicate_name"].as<std::string>();
    std::vector<std::string> options = node["options"].as<std::vector<std::string>>();
    operator_widget = new OperatorAssistanceWidget(question, belief_predicate_name, this);
    operator_widget->setUpOptions(options);
    operator_widget->show();
    app->connect(app, SIGNAL(lastWindowClosed()), app, SLOT(quit()));
    app->exec();
      if (!belief_added)
      {
    BehaviorExecutionController::setTerminationCause(aerostack_msgs::BehaviorActivationFinished::WRONG_PROGRESS);
      }
      else {
    BehaviorExecutionController::setTerminationCause(aerostack_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
      }     
}

bool BehaviorRequestOperatorAssistance::checkSituation()
{
  return true;
}

void BehaviorRequestOperatorAssistance::checkGoal()
{


}

void BehaviorRequestOperatorAssistance::checkProgress() 
{ 
 
}


void BehaviorRequestOperatorAssistance::checkProcesses() 
{ 
 
}

void BehaviorRequestOperatorAssistance::addBelief(std::string belief)
{
  add_belief_msg.belief_expression = belief;
  aerostack_msgs::AddBelief::Response res;
  add_belief_srv.call(add_belief_msg, res);
  belief_added=res.success;
}
