#ifndef TARGET_TRAJECTORY_H
#define TARGET_TRAJECTORY_H

#include <gtddp_drone_msgs/state_data.h>

class TargetTrajectory
{
    public:
        virtual gtddp_drone_msgs::state_data get_target(double t) = 0;

        int get_lead_time();
        void set_lead_time(int lead_time);

    private:
        int lead_time;
};

#endif