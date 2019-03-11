#ifndef STRAIGHT_LINE_H
#define STRAIGHT_LINE_H

#include "gtddp_drone_msgs/state_data.h"
#include "gtddp_drone_target_trajectory/trajectory/target_trajectory.h"
#include "gtddp_drone_target_trajectory/gtddp_traj_constants.h"

class StraightLine : public TargetTrajectory
{
    public:
        StraightLine();
        StraightLine(double x, double y, double z, double t);

        //Get the target data
        gtddp_drone_msgs::state_data get_target(double t);
    
    private:
        double target_time;

        //Target coordinates
        double target_x;
        double target_y;
        double target_z;
};

#endif