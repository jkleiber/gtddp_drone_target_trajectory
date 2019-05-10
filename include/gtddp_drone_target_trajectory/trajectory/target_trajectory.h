#ifndef TARGET_TRAJECTORY_H
#define TARGET_TRAJECTORY_H

#include <gtddp_drone_msgs/state_data.h>

//Make sure these constants match their associated values in Constants.cpp in gtddp_lib
#define MASS (double)(1.25)
#define NUM_STATES (gtddp_drone_msgs::state_data::NUM_STATES)


class TargetTrajectory
{
    public:
        virtual gtddp_drone_msgs::state_data get_target(double t) = 0;
        virtual void set_init_conds(double xo, double yo, double zo) = 0;
        void set_dt(double timestep);

        double DT;

    private:
        int lead_time;

        
};

#endif