//Include system libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <vector>

//Include other libraries
#include <gtddp_drone/gtddp_lib/Constants.h>
#include <gtddp_drone_msgs/state_data.h>

//Include package classes
#include "gtddp_drone_target_trajectory/trajectory/target_trajectory.h"
#include "gtddp_drone_target_trajectory/trajectory/straight_line.h"
#include "gtddp_drone_target_trajectory/gtddp_traj_constants.h"

int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "gtddp_target_node");

    //Initialize this node
    ros::NodeHandle target_node;

    //Advertise control output and landing mode
    ros::Publisher target_pub = target_node.advertise<gtddp_drone_msgs::state_data>(target_node.resolveName("/gtddp_drone_target_trajectory/target_state"), MAX_BUFFER);
    
    //Set up a time tracker for updating the target state
    ros::Time start_time;
    ros::Time cur_time;

    //Establish a loop rate for the target node to run at
    ros::Rate loop_rate(5); //5 Hz

    //Create an instance of the straight line trajectory, and choose a terminal state
    TargetTrajectory *target_traj = new StraightLine(1.0, 0, 1.0, 100.0);

    //Wait for subscribers before publishing state data or starting the timers
    while(target_pub.getNumSubscribers() < 1){}

    //Set the start time
    start_time = ros::Time::now();

    //Run the target state loop
    while(ros::ok())
    {
        //Handle ROS events
        ros::spinOnce();

        //Get current time
        cur_time = ros::Time::now();

        //Calculate and send out the target state every loop
        target_pub.publish(target_traj->get_target((cur_time.toSec() - start_time.toSec())));

        //Sleep until it's time to run again
        loop_rate.sleep();
    }

    //Clear dynamic memory allocations
    free(target_traj);

    return 0;
}