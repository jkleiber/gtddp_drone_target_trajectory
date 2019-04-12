#include "gtddp_drone_target_trajectory/trajectory/spin_around.h"


SpinAround::SpinAround()
{

}



SpinAround::SpinAround(double yaw, double t)
{
    this->target_yaw = yaw;
    this->target_time = t;
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
    state_vector[2] = 1;
    state_vector[8] = this->target_yaw * interpolation;

    for(int i = 0; i < NUM_STATES; ++i)
        target_state.states[i] = state_vector[i];

    return target_state;
}
