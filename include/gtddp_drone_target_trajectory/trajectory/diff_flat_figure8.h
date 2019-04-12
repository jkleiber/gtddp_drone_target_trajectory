#ifndef FIGURE_EIGHT_H
#define FIGURE_EIGHT_H

#include <ros/ros.h>
#include <ros/console.h>

#include "gtddp_drone_msgs/state_data.h"
#include "gtddp_drone_target_trajectory/trajectory/target_trajectory.h"

//Only define NUM_STATES if needed
#ifndef NUM_STATES
#define NUM_STATES (gtddp_drone_msgs::state_data::NUM_STATES)
#endif

class FigureEight : public TargetTrajectory
{
    public:
        FigureEight();
        FigureEight(double alpha, double beta, double gamma, double freq);

        //Get the target data
        gtddp_drone_msgs::state_data get_target(double t);
    
    private:
        
        //Coeffs for Figure 8 trajectory
        double alpha;
        double beta;
        double gamma;
        double freq;

        //Numerical derivative stuff
        double last_theta;
        double last_phi;
};

#endif