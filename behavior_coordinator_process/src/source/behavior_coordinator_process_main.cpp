/*!*********************************************************************************
 *  \file       behavior_coordinator_process_main.cpp
 *  \brief      BehaviorCoordinator main file.
 *  \details    This file implements the main function of the BehaviorCoordinator.
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

#include "../include/behavior_coordinator_process.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, ros::this_node::getName());

  std::cout << ros::this_node::getName() << std::endl;

  BehaviorCoordinator behavior_coordinator;
  behavior_coordinator.setUp();

  try
  {
    behavior_coordinator.start();
  }
  catch (std::exception &exception)
  {
    behavior_coordinator.notifyError(behavior_coordinator.SafeguardRecoverableError, 0, "ownStart()", exception.what());
    behavior_coordinator.stop();
  }

   ros::Rate r(100); // 100 hz
  while (ros::ok())
  {
   ros::spinOnce();
   r.sleep();
  }
  return 0;
}
