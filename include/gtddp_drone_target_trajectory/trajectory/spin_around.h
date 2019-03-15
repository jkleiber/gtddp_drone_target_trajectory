#ifndef SPIN_AROUND_H
#define SPIN_AROUND_H

#include <ros/ros.h>
#include <ros/console.h>

#include "gtddp_drone_msgs/state_data.h"
#include "gtddp_drone_target_trajectory/trajectory/target_trajectory.h"
#include "gtddp_drone_target_trajectory/gtddp_traj_constants.h"

class SpinAround : public TargetTrajectory
{
    public:
        SpinAround();
        SpinAround(double yaw, double t);

        //Get the target data
        gtddp_drone_msgs::state_data get_target(double t);
    
    private:
        double target_time;

        //Target coordinates
        double target_yaw;
};

#endif