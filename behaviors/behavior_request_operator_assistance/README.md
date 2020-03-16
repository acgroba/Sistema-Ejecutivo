# behavior_request_operator_assistance

**Purpose**: This behavior shows a pop-up window with a question for the operator and several options to be selected as the answer. The robot will save the selected answer in its belief memory.

![request_operator_assistance.png](https://bitbucket.org/repo/rokr9B/images/3120408549-request_operator_assistance.png)

**Type of behavior:** Goal-based behavior.

**Arguments:** 

| Name    |   Format  |  Example |  
| :-----------| :---------| :--------|
| question |String with the question for the operator | question: "What is the level of light intensity in the environment where the aerial robot is flying?"|
| belief_predicate_name |String with the name of the predicate to be saved in the belief memory| belief_predicate_name: "light_intensity"|
| options | Array of strings with the possible answers to the question| options: ["Too low","Adequate","Too high"]|
	
# Contributors

**Code maintainer:** Abraham Carrera Groba

**Authors:** Abraham Carrera Groba