/*!*******************************************************************************************
 *  \copyright Copyright 2019 Universidad Politecnica de Madrid (UPM)
 *
 *             This program is free software: you can redistribute it and/or modify
 *             it under the terms of the GNU General Public License as published by
 *             the Free Software Foundation, either version 3 of the License, or
 *             (at your option) any later version.
 *
 *             This program is distributed in the hope that it will be useful,
 *             but WITHOUT ANY WARRANTY; without even the implied warranty of
 *             MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *             GNU General Public License for more details.
 *
 *             You should have received a copy of the GNU General Public License
 *             along with this program. If not, see http://www.gnu.org/licenses/.
 ********************************************************************************************/

#include "behavior_coordinator_process.h"
#include <boost/thread/thread.hpp>
#include <gtest/gtest.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>

/*Parameters*/
ros::NodeHandle *nh;
BehaviorCoordinator *behavior_coordinator;

void spinnerThread() { ros::spin(); }

/*  Main  */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, ros::this_node::getName());
  nh = new ros::NodeHandle;
  behavior_coordinator = new BehaviorCoordinator();

  system("bash "
         "$AEROSTACK_STACK/stack_devel/executive_system/aerostack_executive/behavior_coordinator_process/src/test/"
         "test.sh");
  std::cout << "WAIT WHILE LAUNCHING AEROSTACK" << std::endl;
  ros::Duration(15).sleep();

  std::thread thr(&spinnerThread);
  behavior_coordinator->setUp();
  behavior_coordinator->start();

  return RUN_ALL_TESTS();
  thr.join();
}

/* TESTS */
/**
 * Default activation when the drone is landed
 */

TEST(BehaviorActivation, defaultActivationLanded)
{
  std::cout << "[TEST]: Starting test 'defaultActivationLanded'" << std::endl;

  ros::Duration(2).sleep();

  BehaviorDescriptor behavior1;
  behavior1.setName("SELF_LOCALIZE_BY_ODOMETRY");

  BehaviorDescriptor behavior2;
  behavior2.setName("KEEP_HOVERING");
  EXPECT_TRUE(behavior_coordinator->getBehaviorActivationChanges().isActive(behavior1) &&
              !behavior_coordinator->getBehaviorActivationChanges().isActive(behavior2));
}

/**
 *  Activation withtout activation conditions
 */

TEST(BehaviorActivation, wrongActivationConditions)
{
  std::cout << "[TEST]: Starting test 'wrongActivationConditions'" << std::endl;

  std::string drone_id;
  std::string drone_id_namespace;
  std::string activate_behavior_str;
  std::string inhibit_behavior_str;

  ros::ServiceClient activate_behavior_srv;
  ros::ServiceClient inhibit_behavior_srv;

  nh->param<std::string>("drone_id", drone_id, "1");
  nh->param<std::string>("drone_id_namespace", drone_id_namespace, "drone" + drone_id);
  nh->param<std::string>("activate_behavior_srv", activate_behavior_str, "activate_behavior");
  nh->param<std::string>("inhibit_behavior_srv", inhibit_behavior_str, "inhibit_behavior");

  inhibit_behavior_srv = nh->serviceClient<aerostack_msgs::RequestBehaviorInhibition>("/drone1/" + inhibit_behavior_str);
  activate_behavior_srv = nh->serviceClient<aerostack_msgs::RequestBehaviorActivation>("/drone1/" + activate_behavior_str);

  std::cout << "[TEST]:Calling activate behavior  LAND with priority 2... ";

  aerostack_msgs::RequestBehaviorActivation activate_behavior;
  activate_behavior.request.behavior.name = "LAND";
  activate_behavior.request.behavior.arguments = "";
  activate_behavior.request.behavior.priority = 2;
  activate_behavior_srv.call(activate_behavior);
  // ros::Duration(2).sleep();
  BehaviorDescriptor behavior;
  behavior.setName(activate_behavior.request.behavior.name);
  EXPECT_TRUE(!behavior_coordinator->getBehaviorActivationChanges().isActive(behavior));
}

/**
 *  Activation with precedence present
 */

TEST(BehaviorActivation, precedencePresent)
{
  std::cout << "[TEST]: Starting test 'precedencePresent'" << std::endl;

  std::string drone_id;
  std::string drone_id_namespace;
  std::string activate_behavior_str;
  std::string inhibit_behavior_str;

  ros::ServiceClient activate_behavior_srv;
  ros::ServiceClient inhibit_behavior_srv;

  nh->param<std::string>("drone_id", drone_id, "1");
  nh->param<std::string>("drone_id_namespace", drone_id_namespace, "drone" + drone_id);
  nh->param<std::string>("activate_behavior_srv", activate_behavior_str, "activate_behavior");
  nh->param<std::string>("inhibit_behavior_srv", inhibit_behavior_str, "inhibit_behavior");

  inhibit_behavior_srv = nh->serviceClient<aerostack_msgs::RequestBehaviorInhibition>("/drone1/" + inhibit_behavior_str);
  activate_behavior_srv = nh->serviceClient<aerostack_msgs::RequestBehaviorActivation>("/drone1/" + activate_behavior_str);

  std::cout << "[TEST]:Calling activate behavior  TAKE_OFF with priority 2... ";

  aerostack_msgs::RequestBehaviorActivation activate_behavior;
  activate_behavior.request.behavior.name = "TAKE_OFF";
  activate_behavior.request.behavior.arguments = "";
  activate_behavior.request.behavior.priority = 2;
  activate_behavior_srv.call(activate_behavior);
  ros::Duration(6).sleep();
  BehaviorDescriptor behavior;
  behavior.setName(activate_behavior.request.behavior.name);
  EXPECT_TRUE(!behavior_coordinator->getBehaviorActivationChanges().isActive(behavior));
}

/**
 *  Default activation when the drone is flying
 */

TEST(BehaviorActivation, defaultActivationFlying)
{
  std::cout << "[TEST]: Starting test 'defaultActivationFlying'" << std::endl;

  // ros::Duration(2).sleep();
  BehaviorDescriptor behavior;
  behavior.setName("KEEP_HOVERING");
  EXPECT_TRUE(behavior_coordinator->getBehaviorActivationChanges().isActive(behavior));
}

/**
 *  Activation with activable precedence not  present
 */

TEST(BehaviorActivation, precedenceNotPresent)
{
  std::cout << "[TEST]: Starting test 'precedenceNotPresent'" << std::endl;

  std::string drone_id;
  std::string drone_id_namespace;
  std::string activate_behavior_str;
  std::string inhibit_behavior_str;

  ros::ServiceClient activate_behavior_srv;
  ros::ServiceClient inhibit_behavior_srv;

  nh->param<std::string>("drone_id", drone_id, "1");
  nh->param<std::string>("drone_id_namespace", drone_id_namespace, "drone" + drone_id);
  nh->param<std::string>("activate_behavior_srv", activate_behavior_str, "activate_behavior");
  nh->param<std::string>("inhibit_behavior_srv", inhibit_behavior_str, "inhibit_behavior");

  inhibit_behavior_srv = nh->serviceClient<aerostack_msgs::RequestBehaviorInhibition>("/drone1/" + inhibit_behavior_str);
  activate_behavior_srv = nh->serviceClient<aerostack_msgs::RequestBehaviorActivation>("/drone1/" + activate_behavior_str);

  std::cout << "[TEST]:Calling activate behavior  SELF_LOCALIZE_BY_VISUAL_MARKER with priority 2... ";

  aerostack_msgs::RequestBehaviorActivation activate_behavior;
  activate_behavior.request.behavior.name = "SELF_LOCALIZE_BY_VISUAL_MARKER";
  activate_behavior.request.behavior.arguments = "";
  activate_behavior.request.behavior.priority = 2;
  activate_behavior_srv.call(activate_behavior);
  // ros::Duration(2).sleep();
  BehaviorDescriptor behavior1;
  BehaviorDescriptor behavior2;
  behavior1.setName(activate_behavior.request.behavior.name);
  behavior2.setName("PAY_ATTENTION_TO_VISUAL_MARKERS");
  EXPECT_TRUE(behavior_coordinator->getBehaviorActivationChanges().isActive(behavior1) &&
              behavior_coordinator->getBehaviorActivationChanges().isActive(behavior2));
}

/**
 *  Exclusion with the same priority
 */

TEST(BehaviorActivation, exclusionSamePriority)
{
  std::cout << "[TEST]: Starting test 'exclusionSamePriority'" << std::endl;

  std::string drone_id;
  std::string drone_id_namespace;
  std::string activate_behavior_str;
  std::string inhibit_behavior_str;

  ros::ServiceClient activate_behavior_srv;
  ros::ServiceClient inhibit_behavior_srv;

  nh->param<std::string>("drone_id", drone_id, "1");
  nh->param<std::string>("drone_id_namespace", drone_id_namespace, "drone" + drone_id);
  nh->param<std::string>("activate_behavior_srv", activate_behavior_str, "activate_behavior");
  nh->param<std::string>("inhibit_behavior_srv", inhibit_behavior_str, "inhibit_behavior");

  inhibit_behavior_srv = nh->serviceClient<aerostack_msgs::RequestBehaviorInhibition>("/drone1/" + inhibit_behavior_str);
  activate_behavior_srv = nh->serviceClient<aerostack_msgs::RequestBehaviorActivation>("/drone1/" + activate_behavior_str);

  std::cout << "[TEST]:Calling activate behavior  SELF_LOCALIZE_BY_ODOMETRY with priority 2... ";

  aerostack_msgs::RequestBehaviorActivation activate_behavior;
  activate_behavior.request.behavior.name = "SELF_LOCALIZE_BY_ODOMETRY";
  activate_behavior.request.behavior.arguments = "";
  activate_behavior.request.behavior.priority = 2;
  activate_behavior_srv.call(activate_behavior);
  ros::Duration(2).sleep();
  BehaviorDescriptor behavior1;
  BehaviorDescriptor behavior2;
  behavior1.setName(activate_behavior.request.behavior.name);
  behavior2.setName("SELF_LOCALIZE_BY_VISUAL_MARKER");
  EXPECT_TRUE(behavior_coordinator->getBehaviorActivationChanges().isActive(behavior1) &&
              !behavior_coordinator->getBehaviorActivationChanges().isActive(behavior2));
}

/**
 *  Exclusion with the active behavior with less priority
 */

TEST(BehaviorActivation, exclusionLessPriority)
{
  std::cout << "[TEST]: Starting test 'exclusionLessPriority'" << std::endl;

  std::string drone_id;
  std::string drone_id_namespace;
  std::string activate_behavior_str;
  std::string inhibit_behavior_str;

  ros::ServiceClient activate_behavior_srv;
  ros::ServiceClient inhibit_behavior_srv;

  nh->param<std::string>("drone_id", drone_id, "1");
  nh->param<std::string>("drone_id_namespace", drone_id_namespace, "drone" + drone_id);
  nh->param<std::string>("activate_behavior_srv", activate_behavior_str, "activate_behavior");
  nh->param<std::string>("inhibit_behavior_srv", inhibit_behavior_str, "inhibit_behavior");

  inhibit_behavior_srv = nh->serviceClient<aerostack_msgs::RequestBehaviorInhibition>("/drone1/" + inhibit_behavior_str);
  activate_behavior_srv = nh->serviceClient<aerostack_msgs::RequestBehaviorActivation>("/drone1/" + activate_behavior_str);

  std::cout << "[TEST]:Calling activate behavior  SELF_LOCALIZE_BY_VISUAL_MARKER with priority 2... ";

  aerostack_msgs::RequestBehaviorActivation activate_behavior;
  activate_behavior.request.behavior.name = "SELF_LOCALIZE_BY_VISUAL_MARKER";
  activate_behavior.request.behavior.arguments = "";
  activate_behavior.request.behavior.priority = 3;
  activate_behavior_srv.call(activate_behavior);
  ros::Duration(2).sleep();
  BehaviorDescriptor behavior1;
  BehaviorDescriptor behavior2;
  behavior1.setName(activate_behavior.request.behavior.name);
  behavior2.setName("SELF_LOCALIZE_BY_ODOMETRY");
  EXPECT_TRUE(behavior_coordinator->getBehaviorActivationChanges().isActive(behavior1) &&
              !behavior_coordinator->getBehaviorActivationChanges().isActive(behavior2));
}

/**
 *  Stop behavior that is precedence of active behavior with more priority
 */

TEST(BehaviorActivation, inhibitionPrecedenceOfActiveMorePriority)
{
  std::cout << "[TEST]: Starting test 'inhibitionPrecedenceOfActiveMorePriority'" << std::endl;

  std::string drone_id;
  std::string drone_id_namespace;
  std::string activate_behavior_str;
  std::string inhibit_behavior_str;

  ros::ServiceClient activate_behavior_srv;
  ros::ServiceClient inhibit_behavior_srv;

  nh->param<std::string>("drone_id", drone_id, "1");
  nh->param<std::string>("drone_id_namespace", drone_id_namespace, "drone" + drone_id);
  nh->param<std::string>("activate_behavior_srv", activate_behavior_str, "activate_behavior");
  nh->param<std::string>("inhibit_behavior_srv", inhibit_behavior_str, "inhibit_behavior");

  inhibit_behavior_srv = nh->serviceClient<aerostack_msgs::RequestBehaviorInhibition>("/drone1/" + inhibit_behavior_str);
  activate_behavior_srv = nh->serviceClient<aerostack_msgs::RequestBehaviorActivation>("/drone1/" + activate_behavior_str);

  std::cout << "[TEST]:Calling deactivating behavior  PAY_ATTENTION_TO_VISUAL_MARKERS with priority 2... ";

  aerostack_msgs::RequestBehaviorInhibition deactivate_behavior;
  deactivate_behavior.request.behavior_uid = 5;

  inhibit_behavior_srv.call(deactivate_behavior);
  ros::Duration(2).sleep();
  BehaviorDescriptor behavior;
  behavior.setName("PAY_ATTENTION_TO_VISUAL_MARKERS");
  EXPECT_TRUE(behavior_coordinator->getBehaviorActivationChanges().isActive(behavior));
}

/**
 *  Stop behavior that is precedence of active behavior with same priority
 */

TEST(BehaviorActivation, inhibitionPrecedenceOfActiveSamePriority)
{
  std::cout << "[TEST]: Starting test 'inhibitionPrecedenceOfActiveSamePriority'" << std::endl;

  std::string drone_id;
  std::string drone_id_namespace;
  std::string activate_behavior_str;
  std::string inhibit_behavior_str;

  ros::ServiceClient activate_behavior_srv;
  ros::ServiceClient inhibit_behavior_srv;

  nh->param<std::string>("drone_id", drone_id, "1");
  nh->param<std::string>("drone_id_namespace", drone_id_namespace, "drone" + drone_id);
  nh->param<std::string>("activate_behavior_srv", activate_behavior_str, "activate_behavior");
  nh->param<std::string>("inhibit_behavior_srv", inhibit_behavior_str, "inhibit_behavior");

  inhibit_behavior_srv = nh->serviceClient<aerostack_msgs::RequestBehaviorInhibition>("/drone1/" + inhibit_behavior_str);
  activate_behavior_srv = nh->serviceClient<aerostack_msgs::RequestBehaviorActivation>("/drone1/" + activate_behavior_str);

  std::cout << "[TEST]:Calling deactivating behavior  SELF_LOCALIZE_BY_VISUAL_MARKER with priority 3... ";

  aerostack_msgs::RequestBehaviorInhibition deactivate_behavior;
  deactivate_behavior.request.behavior_uid = 8;

  inhibit_behavior_srv.call(deactivate_behavior);
  ros::Duration(2).sleep();

  std::cout << "[TEST]:Calling activate behavior  SELF_LOCALIZE_BY_VISUAL_MARKER with priority 2... ";

  aerostack_msgs::RequestBehaviorActivation activate_behavior;
  activate_behavior.request.behavior.name = "SELF_LOCALIZE_BY_VISUAL_MARKER";
  activate_behavior.request.behavior.arguments = "";
  activate_behavior.request.behavior.priority = 2;
  activate_behavior_srv.call(activate_behavior);
  ros::Duration(2).sleep();

  std::cout << "[TEST]:Calling deactivating behavior  PAY_ATTENTION_TO_VISUAL_MARKERS with priority 2... ";

  deactivate_behavior.request.behavior_uid = 5;

  inhibit_behavior_srv.call(deactivate_behavior);
  ros::Duration(2).sleep();
  BehaviorDescriptor behavior;
  behavior.setName("PAY_ATTENTION_TO_VISUAL_MARKERS");
  EXPECT_TRUE(!behavior_coordinator->getBehaviorActivationChanges().isActive(behavior));
}
