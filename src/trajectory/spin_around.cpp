#include "gtddp_drone_target_trajectory/trajectory/spin_around.h"


SpinAround::SpinAround()
{
    //Default to the origin for initial conditions
    this->x0 = 0.0;
    this->y0 = 0.0;
    this->z0 = 1.0;
}



SpinAround::SpinAround(double yaw, double t)
{
    this->target_yaw = yaw;
    this->target_time = t;

    //Default to the origin for initial conditions
    this->x0 = 0.0;
    this->y0 = 0.0;
    this->z0 = 1.0;
}


void SpinAround::set_init_conds(double xo, double yo, double zo)
{
    this->x0 = xo;
    this->y0 = yo;
    this->z0 = zo;
}


gtddp_drone_msgs::state_data SpinAround::get_target(double cur_time)
{
    //Create a state message
    gtddp_drone_msgs::state_data target_state;

    //Create a state vector
    std::vector<double> state_vector(NUM_STATES, 0);

    //Find interpolation constant
    double interpolation = cur_time / this->target_time;

    //Do not allow interpolation to exceed 1
    interpolation = interpolation > 1 ? 1 : interpolation;

    //Calculate the interpolation of yaw
    state_vector[2] = this->z0;
    state_vector[8] = this->target_yaw * interpolation;

    for(int i = 0; i < NUM_STATES; ++i)
        target_state.state[i] = state_vector[i];

    return target_state;
}
