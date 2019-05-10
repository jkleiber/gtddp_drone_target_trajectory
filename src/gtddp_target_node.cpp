//Include system libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <vector>

//Include other libraries
#include <gtddp_drone/gtddp_lib/Constants.h>
#include <gtddp_drone_msgs/state_data.h>

//Service
#include "gtddp_drone_msgs/target.h"

//Include package classes
#include "gtddp_drone_target_trajectory/trajectory/target_trajectory.h"
#include "gtddp_drone_target_trajectory/trajectory/straight_line.h"
#include "gtddp_drone_target_trajectory/trajectory/spin_around.h"
#include "gtddp_drone_target_trajectory/trajectory/diff_flat_figure8.h"
#include "gtddp_drone_target_trajectory/trajectory/inclined_circle.h"

//Program constants
#define MAX_BUFFER 100

//Timing variables
double TIME_STEP;
double current_time = 0.0;

//TargetTrajectory *target_traj = new StraightLine(0, 0, 5, 10.0);    //fly up 5 meters
//TargetTrajectory *target_traj = new StraightLine(0, 5, 0, 100.0);    //fly on y axis 5 meters
//TargetTrajectory *target_traj = new StraightLine(3, 0, 0, 10.0);    //fly forward 5 meters
//TargetTrajectory *target_traj = new StraightLine(0, 5, 5, 1.5);    //fly up 5 meters and sideways 5 meters
//TargetTrajectory *target_traj = new SpinAround(2, 40);
TargetTrajectory *target_traj = new FigureEight(1.6, 1.6, 1.0, 0.2);  //fly a figure eight at 0.8 rad/sec
//TargetTrajectory *target_traj = new InclinedCircle(0.9, 0.8, 0.45, 0.98);

bool target_callback(gtddp_drone_msgs::target::Request &req, gtddp_drone_msgs::target::Response &resp)
{
    //Move forward in time for the duration of the flight
    current_time += TIME_STEP;

    //Calculate the new target
    resp.target_state = target_traj->get_target(current_time);

    //Complete the transaction
    return true;
}


void init_conds_callback(const gtddp_drone_msgs::state_data::ConstPtr& init_conds)
{
    target_traj->set_init_conds(init_conds->states[0], init_conds->states[1], init_conds->states[2]);
}

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "gtddp_target_node");

    //Initialize this node
    ros::NodeHandle target_node;

    //Get parameters from the server
    TIME_STEP = target_node.param("/gtddp_target_node/target_dt", 0.5); //Set the timestep

    //Set DT in the target trajectory
    target_traj->set_dt(TIME_STEP);

    //Advertise control output and landing mode
    //ros::Publisher target_pub = target_node.advertise<gtddp_drone_msgs::state_data>(target_node.resolveName("/gtddp_drone_target_trajectory/target_state"), MAX_BUFFER);
    
    //Subscribe to any initial conditions updates from the main controller
    ros::Subscriber init_sub = target_node.subscribe(target_node.resolveName("/gtddp_drone/initial_conditions"), MAX_BUFFER, &init_conds_callback);

    //Advertise the target state as a service
    ros::ServiceServer target_srv = target_node.advertiseService(target_node.resolveName("/gtddp_drone_target_trajectory/target_state"), target_callback);

    //Pump callbacks
    ros::spin();

    //Clear dynamic memory allocations
    free(target_traj);

    return 0;
}