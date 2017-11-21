# Force Dimension Sigma ROS Node

This ros node interfaces with a Force Dimension Sigma master device.

## Topics
The following topics under the name space /sigma/sigma<device_number>/ are 
published to:
* /pose  type:PoseStamped - Cartesian pose of the end effector.
* /buttons type:Joy - Contains the states of two buttons of sigma. First is the 
gripper button emulation, and second is the foot pedal.
* /twist  type:TwistStamped - Cartesian velocities of the end-effector

The following topic is subscribed to:
* /force_feedback type:WrenchStamped - Receives wrench information and when
the foot pedal is pressed it sends the wrenches to the sigma device. Note
that you can lock the orientation of the end-effector (useful in 
tele-operation) when the foot-pedal is released if you set the 
lock_orientation parameter as true.


## Parameters

* frequency defines the node's refresh rate
* enable_gripper_button enable the emulated gripper button of sigma
* lock_orientation flag to lock the orientation of the end-effector when the 
foot pedal is released.
* wrench_topic String defining the topic name for the wrench message to which
 this node subscribes
