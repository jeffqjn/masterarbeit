#include "angular_v.h"

//load parameters from yaml

void load_from_config(Parameters & parameters)
{
    YAML::Node config = YAML::LoadFile(YAML_PATH);

    //load paramters in class Parameters
    //Gyro-Scale-Factor
    parameters.set_GYRO_SCALE_FACTOR(config["Gyro-scale-factor"].as<double>());
    //Gyro-Bias
    parameters.set_GYRO_BIAS(config["Gyro-bias"].as<double>());
    //Gyro-Walking-Bias
    parameters.set_Gyro_WALKING_BIAS(config["Gyro-walking-bias"].as<double>());
    //Start-Compute-Time
    parameters.set_START_COMPUTE_TIME(config["start_compute_time"].as<double>());
    //End-Compute-Time
    parameters.set_END_COMPUTE_TIME(config["end_compute_time"].as<double>());
    //Bag_Path
    parameters.set_Bag_Path(config["Bag_Path"].as<string>());
    //Iterations
    parameters.set_Iterations(config["ICP_Iterations"].as<int>());
    //Acc-Scale-Factor
    parameters.set_ACC_SCALE_FACTOR(config["Acc-scale-factor"].as<double>());
    //Acc-Bias
    parameters.set_ACC_BIAS(config["Acc-bias"].as<double>());
    //Acc-Walking-Bias
    parameters.set_ACC_WALKING_BIAS(config["Acc-walking-bias"].as<double>());
}

IntervalMatrix calculate_rodrigues_rotation(IntervalVector angular_vel, double delta_t)
{
    IntervalMatrix skew_symmetric_matrix(3,3,0.0);
    IntervalMatrix skew_symmetric_matrix2(3,3,0.0);
    Interval theta;
    Interval term_a;
    Interval term_b;
    angular_vel[0]=angular_vel[0]*delta_t;
    angular_vel[1]=angular_vel[1]*delta_t;
    angular_vel[2]=angular_vel[2]*delta_t;
    //set up skew-symmetric matrix
    skew_symmetric_matrix[0][1]= -angular_vel[2];
    skew_symmetric_matrix[0][2]= angular_vel[1];
    skew_symmetric_matrix[1][0]= angular_vel[2];
    skew_symmetric_matrix[1][2]= -angular_vel[0];
    skew_symmetric_matrix[2][0]= -angular_vel[1];
    skew_symmetric_matrix[2][1]= angular_vel[0];
    skew_symmetric_matrix[0][0]= 0;
    skew_symmetric_matrix[1][1]= 0;
    skew_symmetric_matrix[2][2]= 0;

    //square the skew-symmetric matrix
    skew_symmetric_matrix2[0][0]=-sqr(angular_vel[2])-sqr(angular_vel[1]);
    skew_symmetric_matrix2[1][1]=-sqr(angular_vel[2])-sqr(angular_vel[0]);
    skew_symmetric_matrix2[2][2]=-sqr(angular_vel[1])-sqr(angular_vel[0]);
    skew_symmetric_matrix2[0][1]=angular_vel[0]*angular_vel[1];
    skew_symmetric_matrix2[1][0]=angular_vel[0]*angular_vel[1];
    skew_symmetric_matrix2[0][2]=angular_vel[0]*angular_vel[2];
    skew_symmetric_matrix2[2][0]=angular_vel[0]*angular_vel[2];
    skew_symmetric_matrix2[1][2]=angular_vel[2]*angular_vel[1];
    skew_symmetric_matrix2[2][1]=angular_vel[2]*angular_vel[1];

    theta=sqrt(sqr(angular_vel[0])+sqr(angular_vel[1])+sqr(angular_vel[2]));

    double term_a_lb=sin(theta.ub())/theta.ub();
    //Situation 1 when the lb of term a greater then the maximal feasible vaule of sin(theta)/theta
    if(term_a_lb>=1.)
    {
        term_a=Interval(1.);
    }
    else
    {
        //the maximum is at x=0, so need to know contain 0?
        if (theta.contains(0))
        {
            term_a=Interval(term_a_lb,1.);
        }
        else
        {
            term_a=Interval(term_a_lb,sin(theta.lb())/theta.lb());
        }
    }
    double term_b_lb=2*(sin(theta.ub()/2)*sin(theta.ub()/2))/(theta.ub()*theta.ub());
    //Situation 1 when the lb of term a greater then the maximal feasible vaule
    if(term_b_lb>=0.5)
    {
        term_b=Interval(0.5);
    }
    else
    {
        //the maximum is at x=0, so need to know contain 0?
        if (theta.contains(0.0))
        {
            term_b = Interval(term_b_lb, 0.5);
        }
        else
        {
            if(term_b_lb>(2*sin(theta.lb()/2)*sin(theta.lb()/2))/(theta.lb()*theta.lb()))
            {
                term_b=Interval((2*sin(theta.lb()/2)*sin(theta.lb()/2))/(theta.lb()*theta.lb()),term_b_lb);
            }
            else {
                term_b = Interval(term_b_lb,
                                  (2 * sin(theta.lb() / 2) * sin(theta.lb() / 2)) / (theta.lb() * theta.lb()));
            }
        }
    }
    IntervalMatrix eye=Matrix::eye(3);
    IntervalMatrix R=eye+term_a*skew_symmetric_matrix+term_b*skew_symmetric_matrix2;
    return R;
}
static void toEulerAngle(const Eigen::Quaterniond & q, double & roll, double & pitch , double & yaw)
{
    //roll
    double sinr_cosp=+2.0*(q.w()*q.x()+q.y()*q.z());
    double cosr_cosp=+1.0-2.0*(q.x()*q.x()+q.y()*q.y());
    roll=atan2(sinr_cosp,cosr_cosp)*180/M_PI;

    //pitch
    double sinp=+2.0*(q.w()*q.y()-q.z()*q.x());
    if (fabs(sinp)>=1)
    {
        pitch=copysign(M_PI/2,sinp)*180/M_PI;
    }
    else
    {
        pitch=asin(sinp);
    }

    //yaw
    double siny_cosp=+2.0*(q.w()*q.z()+q.x()*q.y());
    double cosy_cosp=+1.0-2.0*(q.y()*q.y()+q.z()*q.z());
    yaw=atan2(siny_cosp,cosy_cosp)*180/M_PI;
}
vector<ibex::Interval> IntervalrotationMatrixtoEulerAngle(IntervalMatrix & matrix)
{
    Interval roll ,pitch ,yaw;
    vector<Interval> temp;
    Interval sy= sqrt(matrix[0][0]*matrix[0][0]+matrix[1][0]*matrix[1][0]);
    if(sy.mid()<1e-6)
    {
        roll=atan2(-matrix[1][2],matrix[1][1]);
        pitch=atan2(-matrix[2][0],sy);
        yaw=0;

    }
    else
    {
        roll=atan2(matrix[2][1],matrix[2][2]);
        pitch=atan2(-matrix[2][0],sy);
        yaw=atan2(matrix[1][0],matrix[0][0]);
    }
    temp.push_back(roll*180/M_PI);
    temp.push_back(pitch*180/M_PI);
    temp.push_back(yaw*180/M_PI);
    return temp;
}


double double_round(double dVal, int iPlaces) {
    double dRetval;
    double dMod = 0.0000001;
    if (dVal<0.0) dMod=-0.0000001;
    dRetval=dVal;
    dRetval+=(5.0/pow(10.0,iPlaces+1.0));
    dRetval*=pow(10.0,iPlaces);
    dRetval=floor(dRetval+dMod);
    dRetval/=pow(10.0,iPlaces);
    return(dRetval);
//const double multiplier = std::pow(10.0,iPlaces);
//return std::ceil(dVal*multiplier) / multiplier;
}


//
// Created by noah on 23.01.21.
//

