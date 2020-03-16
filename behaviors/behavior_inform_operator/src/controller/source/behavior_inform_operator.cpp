
/*!*******************************************************************************************
 *  \file       behavior_inform_operator.cpp
 *  \brief      BehaviorInformOperator implementation file.
 *  \details    This file implements the BehaviorInformOperator class.
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

#include <behavior_inform_operator.h>
#include <inform_operator_widget.h>
#include <nodelet/nodelet.h>

BehaviorInformOperator::BehaviorInformOperator(QApplication *app) : BehaviorExecutionController()
{
  this->app = app;
   setName("inform_operator");
}

BehaviorInformOperator::~BehaviorInformOperator() {}

void BehaviorInformOperator::onConfigure()
{  
}

void BehaviorInformOperator::onActivate()
{

}

void BehaviorInformOperator::onDeactivate()
{

}

void BehaviorInformOperator::onExecute()
{
    std::string args = getParameters();
    YAML::Node node = YAML::Load(args);

    std::string message = node["message"].as<std::string>();

    operator_widget = new InformOperatorWidget(message, this);
    operator_widget->show();
    app->connect(app, SIGNAL(lastWindowClosed()), app, SLOT(quit()));
    app->exec();
    BehaviorExecutionController::setTerminationCause(aerostack_msgs::BehaviorActivationFinished::GOAL_ACHIEVED);
}

bool BehaviorInformOperator::checkSituation()
{
  return true;
}

void BehaviorInformOperator::checkGoal()
{


}

void BehaviorInformOperator::checkProgress() 
{ 
 
}


void BehaviorInformOperator::checkProcesses() 
{ 
 
}


