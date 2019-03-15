#include "gtddp_drone_target_trajectory/trajectory/target_trajectory.h"

int TargetTrajectory::get_lead_time()
{
    return this->lead_time;
}

void TargetTrajectory::set_lead_time(int lead_time)
{
    this->lead_time = lead_time;
}
