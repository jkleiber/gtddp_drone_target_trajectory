//Include system libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <sstream>
#include <vector>

//Include other libraries
#include <gtddp_drone/gtddp_lib/Constants.h>
#include <gtddp_drone_msgs/state_data.h>

//Include package classes
#include "gtddp_drone_target_trajectory/trajectory/target_trajectory.h"
#include "gtddp_drone_target_trajectory/trajectory/straight_line.h"


#define MAX_BUFFER 100
#define NUM_STATES 12

int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "gtddp_target_node");

    //Initialize this node
    ros::NodeHandle target_node;

    //Advertise control output and landing mode
    ros::Publisher target_pub = target_node.advertise<gtddp_drone_msgs::state_data>(target_node.resolveName("/gtddp_drone_target_trajectory/target_state"), MAX_BUFFER);
    
    //TODO: set up a timer for changing the target state

    //Establish a loop rate for the target node to run at
    ros::Rate loop_rate(1); //100 Hz

    //Create a state message
    gtddp_drone_msgs::state_data target_state;

    //Create an instance of the straight line trajectory
    TargetTrajectory *target_traj = new StraightLine(1.0, 0, 1.0, 100.0);

    //Choose an initial target state
    //TODO: make this dynamic
    std::vector<double> state_vector(NUM_STATES, 0.0);
    state_vector[0] = 1;
    state_vector[1] = 0;
    state_vector[2] = 1;

    //Add the values to the initial state
    //TODO: make a cleaner way of doing this when paths are to be followed
    for(int i = 0; i < NUM_STATES; ++i)
        target_state.states[i] = state_vector[i];

    //TODO: Add timers to accurately track horizon

    //Run the target state loop
    while(ros::ok())
    {
        //Handle ROS events
        ros::spinOnce();

        //TODO: get the target state from the abstract class
        //target_state = target_traj->get_target()

        //Send out the target state every loop
        target_pub.publish(target_state);

        //Sleep until it's time to run again
        loop_rate.sleep();
    }

    //Clear dynamic memory allocations
    free(target_traj);

    return 0;
}