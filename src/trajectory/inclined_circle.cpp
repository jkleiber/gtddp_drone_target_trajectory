#include "gtddp_drone_target_trajectory/trajectory/inclined_circle.h"

/**
 * @brief Construct a new Figure Eight:: Figure Eight object
 * 
 */
InclinedCircle::InclinedCircle()
{

}

/**
 * @brief Construct a new Figure Eight:: Figure Eight object
 * 
 * @param alpha 
 * @param beta 
 * @param period 
 */
InclinedCircle::InclinedCircle(double alpha, double beta, double gamma, double freq)
{
    this->alpha = alpha;
    this->beta = beta;
    this->gamma = gamma;
    this->freq = freq;
}


/**
 * @brief 
 * 
 * @param t 
 * @return gtddp_drone_msgs::state_data 
 */
gtddp_drone_msgs::state_data InclinedCircle::get_target(double t)
{
    std::vector<double> target(12);
    gtddp_drone_msgs::state_data data;
    double thrust;

    //x, y, z, yaw
    target[0] = alpha * cos(freq * t);          //x
    target[1] = beta * sin(freq * t);           //y
    target[2] = gamma * cos(freq * t) + 2;      //z
    target[8] = 0;                              //yaw

    //linear velocity
    target[3] = -alpha * sin(freq / (2 * t)) * (-freq * 2) / pow(2 * t, 2); //x dot
    target[4] = beta * cos(freq * t) * freq;                                //y dot
    //target[5] = 0;                                                          //z dot
    //TODO: math for the circle
    //Thrust and moment calculations
    //thrust = 

    //roll, pitch
    target[6] = 0;
    target[7] = 0;

    //angular rates
    target[9] = 0;
    target[10] = 0;
    target[11] = 0;

    //Convert to state_data
    for(int i = 0; i < NUM_STATES; ++i)
    {
        data.states[i] = target[i];
    }

    return data;
}