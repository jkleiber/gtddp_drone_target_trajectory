#include "gtddp_drone_target_trajectory/trajectory/straight_line.h"


StraightLine::StraightLine()
{
    //Default to the origin for initial conditions
    this->x0 = 0.0;
    this->y0 = 0.0;
    this->z0 = 0.0;
}



StraightLine::StraightLine(double x, double y, double z, double t)
{
    this->target_x = x;
    this->target_y = y;
    this->target_z = z;
    this->target_time = t;

    //Default to the origin for initial conditions
    this->x0 = 0.0;
    this->y0 = 0.0;
    this->z0 = 0.0;
}


void StraightLine::set_init_conds(double xo, double yo, double zo)
{
    this->x0 = xo;
    this->y0 = yo;
    this->z0 = zo;
}


gtddp_drone_msgs::state_data StraightLine::get_target(double cur_time)
{
    //Create a state message
    gtddp_drone_msgs::state_data target_state;

    //Create a state vector
    std::vector<double> state_vector(NUM_STATES, 0);

    //Find interpolation constant
    double interpolation = cur_time / this->target_time;

    //Do not allow interpolation to exceed 1
    interpolation = interpolation > 1 ? 1 : interpolation;

    //Calculate the interpolation of each coordinate
    state_vector[0] = (this->target_x * interpolation) + this->x0;
    state_vector[1] = (this->target_y * interpolation) + this->y0;
    state_vector[2] = (this->target_z * interpolation) + this->z0;   //The drone starts 1 meter off the ground, so set the offset accordingly

    for(int i = 0; i < NUM_STATES; ++i)
        target_state.state[i] = state_vector[i];

    return target_state;
}
