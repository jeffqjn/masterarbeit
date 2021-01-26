//
// Created by noah on 18.01.21.
//
#include <angular_v.h>
IntervalMatrix IMU::vel2rotatation(double start_compute_time, double  end_compute_time)
{
    int stamp_index=0;
    Interval current_stamp;
    IntervalVector current_vel(3);
    IntervalMatrix overall_rotation=Matrix::eye(3);
    double delta_t;
    //find the corresponding stamp of imu
    while(!this->vel_data[stamp_index].first.contains(start_compute_time))
    {
        stamp_index++;
    }
    if(this->vel_data[stamp_index].first.contains(start_compute_time))
    {
        current_stamp=this->vel_data[stamp_index].first;
        current_vel=this->vel_data[stamp_index].second;
    }
    //calculate delta t, whether the required time is between two stamps
    do {
        delta_t=min(current_stamp.ub(),end_compute_time)-max(start_compute_time,current_stamp.lb());
        IntervalMatrix Matrix_temp(3,3);
        Matrix_temp=calculate_rodrigues_rotation(current_vel,delta_t);
        Matrix_temp &=IntervalMatrix(3,3,Interval(-1,1));
        overall_rotation*=Matrix_temp;
        stamp_index++;
        if(stamp_index>vel_data.size()) break;
        current_stamp=this->vel_data[stamp_index].first;
        current_vel=this->vel_data[stamp_index].second;
    }while (current_stamp.lb()<end_compute_time);
    //cout<<overall_rotation<<endl;
    overall_rotation &=IntervalMatrix(3,3,Interval(-1,1));
    return overall_rotation;
}
void IMU::add_vel_measurement(sensor_msgs::ImuConstPtr  imupointer, Parameters & parameters) {
    //In order to get the interval of current timestamp
    double start_current_timestamp, end_current_timestamp;
    double start_time;
    double GYRO_SCALE_FACTOR= parameters.get_GYRO_SCALE_FACTOR();
    double GYRO_BIAS= parameters.get_GYRO_BIAS();
    double GYRO_WALKING_BIAS= parameters.get_Gyro_WALKING_BIAS();
    if (this->vel_data.empty())
    {
        //To get initial angular velocity
        end_current_timestamp=imupointer->header.stamp.toSec();
        start_current_timestamp=end_current_timestamp;
        start_time=start_current_timestamp;
    }
        //There exists the previous imu data in the set
    else
    {
        start_current_timestamp=this->vel_data.back().first.ub();
        end_current_timestamp=imupointer->header.stamp.toSec();
    }
    if (start_current_timestamp<=end_current_timestamp) {
        Interval time_stamp(start_current_timestamp, end_current_timestamp);
        //Introduce noise, measurement noise ignored
        Interval scale_factor = (-GYRO_SCALE_FACTOR, GYRO_SCALE_FACTOR);
        IntervalVector bias(3, Interval(-GYRO_BIAS, GYRO_BIAS));
        IntervalVector walking_bias(3, Interval(-GYRO_WALKING_BIAS, GYRO_WALKING_BIAS));
        IntervalVector actual_vel(3);
        //add noise to each component
        actual_vel[0] = imupointer->angular_velocity.x + scale_factor * imupointer->angular_velocity.x + bias[0]/* +
                        walking_bias[0] * (end_current_timestamp - start_time)*/;
        actual_vel[1] = imupointer->angular_velocity.y + scale_factor * imupointer->angular_velocity.y + bias[1]/* +
                        walking_bias[1] * (end_current_timestamp - start_time)*/;
        actual_vel[2] = imupointer->angular_velocity.z + scale_factor * imupointer->angular_velocity.z + bias[2] /*+
                        walking_bias[2] * (end_current_timestamp - start_time)*/;
        this->vel_data.push_back(make_pair(time_stamp, actual_vel));
    }
    else
    {
        cout<<"IMU Data not correct,abort!!!"<<endl;
        exit(1);
    }
}
