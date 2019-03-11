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



gtddp_drone_msgs::state_data StraightLine::get_target(double cur_time)
{
    //Create a state message
    gtddp_drone_msgs::state_data target_state;

    //Create a state vector
    std::vector<double> state_vector(NUM_STATES, 0.0);

    //Find interpolation constant
    double interpolation = cur_time / this->target_time;

    //Do not allow interpolation to exceed 1
    interpolation = interpolation > 1 ? 1 : interpolation;

    //Calculate the interpolation of each coordinate
    state_vector[0] = this->target_x * interpolation;
    state_vector[1] = this->target_y * interpolation;
    state_vector[2] = ((this->target_z - 1) * interpolation) + 1;   //The drone starts 1 meter off the ground, so set the offset accordingly

    for(int i = 0; i < NUM_STATES; ++i)
        target_state.states[i] = state_vector[i];
}
