#include "gtddp_drone_target_trajectory/trajectory/inclined_circle.h"


InclinedCircle::InclinedCircle()
{
    this->last_phi = -100;
    this->last_theta = -100;
}


InclinedCircle::InclinedCircle(double alpha, double beta, double gamma, double freq)
{
    this->alpha = alpha;
    this->beta = beta;
    this->gamma = gamma;
    this->freq = freq;

    this->last_phi = -100;
    this->last_theta = -100;
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
    double accel_x;
    double accel_y;
    double accel_z;
    double cp1, cp2, cp3;   //Squiggle snake vector (zeta?)
    double thrust;
    double theta_dot;
    double phi_dot;

    //x, y, z, yaw
    target[0] = alpha * cos(freq * t);          //x
    target[1] = beta * sin(freq * t) - beta;    //y
    target[2] = gamma * cos(freq * t) + 1;      //z
    target[8] = 0;                              //yaw

    //linear velocity
    target[3] = -alpha * freq * sin(freq * t);  //x dot
    target[4] = beta * freq * cos(freq * t);    //y dot
    target[5] = -gamma * freq * sin(freq * t);  //z dot
    
    //Calculate linear acceleration for intermediate control input
    accel_x = -alpha * pow(freq, 2) * cos(freq * t);
    accel_y = -beta * pow(freq, 2) * sin(freq * t);
    accel_z = -gamma * pow(freq, 2) * cos(freq * t) - 9.8;

    //Thrust and moment calculations
    thrust = sqrt(pow(accel_x, 2) + pow(accel_y, 2) + pow(accel_z, 2));

    //Calculate the intermediate angle stuff
    //NOTE: assuming rotation matrix is identity due to psi = 0
    cp1 = accel_x * MASS / thrust;
    cp2 = accel_y * MASS / thrust;
    cp3 = accel_z * MASS / thrust;

    //roll, pitch
    target[6] = -asin(cp2);
    target[7] = atan(cp1 / cp3);

    //Initialize the numerical derivative (Note: first point's derivative will always be 0)
    if(last_phi == -100 && last_theta == -100)
    {
        last_phi = target[6];
        last_theta = target[7];
    }

    //Calculate angular derivatives
    phi_dot = (target[6] - last_phi) / DT;
    theta_dot = (target[7] - last_theta) / DT;

    //angular rates
    target[9] = phi_dot;
    target[10] = theta_dot * cos(target[6]);
    target[11] = -theta_dot * sin(target[6]);

    //Convert to state_data
    for(int i = 0; i < NUM_STATES; ++i)
    {
        data.states[i] = target[i];
    }

    //Save last values
    this->last_phi = target[6];
    this->last_theta = target[7];

    return data;
}