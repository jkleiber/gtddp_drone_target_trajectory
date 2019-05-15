# gtddp_drone_target_trajectory

### Overview
This package provides target states along pre calculated target trajectories at certain timesteps along the path. The target states are derived using differential flatness in order to make the trajectories flyable by the drone.

This code can be adopted by any other drone system, even if it is not using the GT-DDP approach. The main functionality of the package is receiving a request from a ROS service, calculating the next target state, and then responding with that state to the calling ROS node. Many different trajectories are included, but more can be added quite easily, as the program uses an abstract class to represent many different types of trajectories.

### Trajectories
* Figure Eight
* Inclined Circle
* Straight Line
* Spin

### Dependencies
* [gtddp_drone_msgs](https://github.com/jkleiber/gtddp_drone_msgs/tree/master) - Contains the ROS srv file used by gtddp_drone_target_trajectory

### Installation
It is recommended to install this alongside the gtddp_drone_msgs package, as it has the srv file needed to request a target state. This is part of the [ARDrone_Control](https://github.com/jkleiber/ARDrone_Control) repository and is automatically installed in its installation instructions.

If you want this package to not be a part of the ARDrone_Control repository, just clone it to your workspace and replicate the appropriate srv file to the package you want.


