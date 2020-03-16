# Brief
This process is in charge of the coordination and execution of each behavior received.
For the coordination, the process checks if the behavior received is consistent with some beliefs
and with other behaviors executing at the same time.

# Services
- **request_behavior_activation** ([aerostack_msgs/RequestBehaviorActivation](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/srv/RequestBehavior.srv))  
Activates a given behavior.

- **request_behavior_inhibition** ([aerostack_msgs/RequestBehaviorInhibition](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/srv/InhibitBehavior.srv))  
Stops a given behavior.

- **consult_available_behaviors** ([aerostack_msgs/ConsultAvailableBehaviors](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/srv/ConsultAvailableBehaviors.srv))  
Returns a list with loaded behaviors.

- **check_behavior_format** ([aerostack_msgs/CheckBehaviorFormat](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/srv/CheckBehaviorFormat.srv))  
Checks the format of the behavior call and its arguments.

- **consult_incompatible_behaviors** ([aerostack_msgs/ConsultIncompatibleBehaviors](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/srv/ConsultIncompatibleBehaviors.srv))  
Returns a list with current active incompatible behaviors.

# Published topics
- **list_of_active_behaviors** ([aerostack_msgs/ListOfbehaviors](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/msg/ListOfBehaviors.msg))  
Publishes the list of active behaviors. This topic is updated when a behavior is activated or inhibited.

# Subscribed topics
- **behavior_event**  ([aerostack_msgs/BehaviorEvent](https://bitbucket.org/visionaerialrobotics/aerostack_msgs/src/master/msg/BehaviorEvent.msg))  
Informs about the result of the execution of a behavior that has stopped.

# Tests
The following tests are performed:

* **Test 1:** Default activation landed.
* **Test 2:** Wrong activation conditions.
* **Test 3:** Precedence active.
* **Test 4:** Default activation flying.
* **Test 5:** Inactive precedence.
* **Test 6:** Same priority exclusion active.
* **Test 7:** Less priority exclusion active.
* **Test 8:** Inhibit behavior precedence of active with more priority.
* **Test 9:** Inhibit behavior precedence of active with same priority.


---
# Contributors
**Maintainer:** Abraham Carrera Groba    
**Author:** Abraham Carrera Groba
