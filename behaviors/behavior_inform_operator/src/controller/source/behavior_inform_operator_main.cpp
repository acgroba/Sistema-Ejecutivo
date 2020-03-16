/*!*********************************************************************************
 *  \file       behavior_inform_operator_main.cpp
 *  \brief      BehaviorRequestOperatorAssistance main file.
 *  \details    This file implements the main function of the BehaviorRequestOperatorAssistance.
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

#include <behavior_inform_operator.h>
#include <nodelet/nodelet.h>
void signalhandler(int sig)
{
  if (sig == SIGINT || sig == SIGTERM)
    qApp->quit();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, ros::this_node::getName());
  QApplication app(argc, argv);
  signal(SIGINT, signalhandler);
  signal(SIGTERM, signalhandler);

  std::cout << ros::this_node::getName() << std::endl;

  ros::TimerEvent time_var;
  BehaviorInformOperator behavior(&app);
  behavior.configure();
  ros::AsyncSpinner as(1);
  as.start();

  ros::Rate rate(100);
  while (ros::ok())
  {
     behavior.execute(time_var);
    rate.sleep();
  }
  return 0;
}
