#include "gtddp_drone_target_trajectory/trajectory/straight_line.h"


StraightLine::StraightLine()
{

}



StraightLine::StraightLine(double x, double y, double z, double t)
{
    this->target_x = x;
    this->target_y = y;
    this->target_z = z;
    this->target_time = t;
}



gtddp_drone_msgs::state_data StraightLine::get_target(double time)
{

}
