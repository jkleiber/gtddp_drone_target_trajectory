#ifndef STRAIGHT_LINE_H
#define STRAIGHT_LINE_H

#include <ros/ros.h>
#include <ros/console.h>

#include "gtddp_drone_msgs/state_data.h"
#include "gtddp_drone_target_trajectory/trajectory/target_trajectory.h"


class StraightLine : public TargetTrajectory
{
    public:
        StraightLine();
        StraightLine(double x, double y, double z, double t);

        //Init
        void set_init_conds(double xo, double yo, double zo);

        //Get the target data
        gtddp_drone_msgs::state_data get_target(double t);
    
    private:
        double target_time;

        //Initial position
        double x0;
        double y0;
        double z0;

        //Target coordinates
        double target_x;
        double target_y;
        double target_z;
};

#endif