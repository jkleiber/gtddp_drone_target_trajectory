#include "gtddp_drone_target_trajectory/trajectory/diff_flat_figure8.h"

/**
 * @brief Construct a new Figure Eight:: Figure Eight object
 * 
 */
FigureEight::FigureEight()
{
    this->last_phi = 0;
    this->last_theta = 0;

    //Default to the origin for initial conditions
    this->x0 = 0.0;
    this->y0 = 0.0;
    this->z0 = 1.0;
}

/**
 * @brief Construct a new Figure Eight:: Figure Eight object
 * 
 * @param alpha 
 * @param beta 
 * @param period 
 */
FigureEight::FigureEight(double alpha, double beta, double gamma, double freq)
{
    this->alpha = alpha;
    this->beta = beta;
    this->gamma = gamma;
    this->freq = freq;

    //Default to the origin for initial conditions
    this->x0 = 0.0;
    this->y0 = 0.0;
    this->z0 = 1.0;

    this->last_phi = -100;
    this->last_theta = -100;
}


void FigureEight::set_init_conds(double xo, double yo, double zo)
{
    this->x0 = xo;
    this->y0 = yo;
    this->z0 = zo;
}


/**
 * @brief 
 * 
 * @param t 
 * @return gtddp_drone_msgs::state_data 
 */
gtddp_drone_msgs::state_data FigureEight::get_target(double t)
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
    target[0] = alpha * cos((freq * t) / 2) + (this->x0 - alpha);   //x
    target[1] = beta * sin(freq * t) + this->y0;                    //y
    target[2] = this->z0;                                           //z
    target[8] = 0;                                                  //yaw

    //linear velocity
    target[3] = -(alpha * freq / 2 ) * sin((freq * t) / 2);         //x dot
    target[4] = beta * cos(freq * t) * freq;                        //y dot
    target[5] = 0;                                                  //z dot

    //Calculate linear acceleration for intermediate control input
    accel_x = (-1/4)*alpha*pow(freq, 2)*cos((freq * t) / 2);        //x accel
    accel_y = -beta * pow(freq, 2) * sin(freq * t);                 //y accel
    accel_z = -9.8;                                                 //z accel (gravity)

    //Thrust and moment calculations
    thrust = sqrt(pow(accel_x, 2) + pow(accel_y, 2) + pow(accel_z, 2));

    //Calculate the intermediate angle stuff
    //NOTE: assuming rotation matrix is identity due to psi = 0
    cp1 = accel_x * MASS / thrust;
    cp2 = accel_y * MASS / thrust;
    cp3 = accel_z * MASS / thrust;

    //roll, pitch
    target[6] = -asin(cp2);                                         //pitch
    target[7] = atan(cp1 / cp3);                                    //roll

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