#ifndef FIGURE_EIGHT_H
#define FIGURE_EIGHT_H

#include <ros/ros.h>
#include <ros/console.h>

#include "gtddp_drone_msgs/state_data.h"
#include "gtddp_drone_target_trajectory/trajectory/target_trajectory.h"
#include "gtddp_drone_target_trajectory/gtddp_traj_constants.h"

class FigureEight : public TargetTrajectory
{
    public:
        FigureEight();
        FigureEight(double alpha, double beta, double period);

        //Get the target data
        gtddp_drone_msgs::state_data get_target(double t);
    
    private:
        
        //Coeffs for Figure 8
        double alpha;
        double beta;
        double period;
};

#endif