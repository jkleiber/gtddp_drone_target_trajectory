#ifndef TARGET_TRAJECTORY_H
#define TARGET_TRAJECTORY_H

#include <gtddp_drone_msgs/state_data.h>

//Make sure these constants match their associated values in Constants.cpp in gtddp_lib
#define MASS (double)(1.25)
#define DT (double)(0.001)

class TargetTrajectory
{
    public:
        virtual gtddp_drone_msgs::state_data get_target(double t) = 0;

    private:
        int lead_time;
};

#endif