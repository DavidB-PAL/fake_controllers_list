### ROS Node: fake_controllers_list
Copyright (c) 2013, David Butterworth, PAL Robotics S.L. 
<br>
<br>
A fake /list_controllers Service.

The ROS Controller Manager provides the service <br>
/list_controllers <br>
which returns a list of those controllers that are loaded, <br>
and their state as 'stopped' or 'running'. 

e.g. <br>
controllers: ['head_traj_controller', 'left_arm_controller', 'left_hand_controller', ... ] <br>
state: ['running', 'running', 'running', ... ]

This allows other nodes to query the list, then call the /switch_controller Service.
<br>

<br>
The object_manipulator node from the Manipulation Pipeline calls the /list_controllers Service to check that the appropriate arm controller is running, before it attempts to grasp an object.

Therefore, for robots using controllers outside the Controller Manager system, this node provides a fake Service that returns a list of user-specified controller names, and returns their state as 'running'.
<br>

<br>
**Usage:** <br>
$ roslaunch fake_controllers_list list_controllers.launch <br>
$ rosservice call fake_controllers_list/list_controllers <br>

