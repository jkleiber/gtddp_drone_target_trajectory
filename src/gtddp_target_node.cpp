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
#include "gtddp_drone_target_trajectory/trajectory/spin_around.h"
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
    //TargetTrajectory *target_traj = new StraightLine(0, 0, 5, 50.0);    //fly up 5 meters
    TargetTrajectory *target_traj = new StraightLine(0, 5, 1, 50.0);    //fly on y axis 5 meters
    //TargetTrajectory *target_traj = new StraightLine(5, 0, 1, 50.0);    //fly forward 5 meters
    //TargetTrajectory *target_traj = new StraightLine(0, 5, 5, 50.0);    //fly up 5 meters and sideways 5 meters
    //TargetTrajectory *target_traj = new SpinAround(2, 40);

    //Decide how long to wait before publishing target data
    target_traj->set_lead_time(5);

    //Wait for subscribers before publishing state data or starting the timers
    while(target_pub.getNumSubscribers() < 1){}

    //Wait for the clock topic to publish
    while(ros::Time::now().isZero()){}

    //Set the start time
    start_time = ros::Time::now();

    //Let the horizon point go a bit away from the origin
    sleep(target_traj->get_lead_time());

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