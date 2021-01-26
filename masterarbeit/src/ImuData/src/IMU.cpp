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
//TODO input start_velocity
ibex::IntervalVector IMU::acc2pose(double start_compute_time, double  end_compute_time, LiDAR_PointCloud pointCloud)
{
    Interval stationary_time= pointCloud.get_stationary_time();
    IntervalVector v(3,Interval(0));
    IntervalVector r(3,Interval(0));
    int startIndex=0;
    for (; startIndex<acc_data.size();startIndex++)
    {
        if (stationary_time.intersects(acc_data[startIndex].first))
        {
            break;
        }
    }
    //v[k+1]=v[k]+Ta[k]
    //r[k+1]=r[k]+Tv[k]
    for(int i=startIndex;i<acc_data.size();i++)
    {
        r[0]=r[0]+(acc_data[i].first.ub()-acc_data[i].first.lb())*v[0];
        r[1]=r[1]+(acc_data[i].first.ub()-acc_data[i].first.lb())*v[1];
        r[2]=r[2]+(acc_data[i].first.ub()-acc_data[i].first.lb())*v[2];

        v[0]=v[0]+(acc_data[i].first.ub()-acc_data[i].first.lb())*acc_data[i].second[0];
        v[1]=v[1]+(acc_data[i].first.ub()-acc_data[i].first.lb())*acc_data[i].second[1];
        v[2]=v[2]+(acc_data[i].first.ub()-acc_data[i].first.lb())*acc_data[i].second[2];
    }
    cout<<r<<endl;
    return r;
}
void IMU::add_vel_measurement(sensor_msgs::ImuConstPtr  imupointer, Parameters & parameters) {

    //In order to get the interval of current timestamp
    double start_current_timestamp, end_current_timestamp;
    double start_time;
    double GYRO_SCALE_FACTOR= parameters.get_GYRO_SCALE_FACTOR();
    double GYRO_BIAS= parameters.get_GYRO_BIAS();
    double GYRO_WALKING_BIAS= parameters.get_Gyro_WALKING_BIAS();

    double ACC_SCALE_FACTOR= parameters.get_ACC_SCALE_FACTOR();
    double ACC_BIAS= parameters.get_ACC_BIAS();
    double ACC_WALKING_BIAS= parameters.get_ACC_WALKING_BIAS();

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
        //Gyroscope
        Interval gyro_scale_factor = (-GYRO_SCALE_FACTOR, GYRO_SCALE_FACTOR);
        IntervalVector gyro_bias(3, Interval(-GYRO_BIAS, GYRO_BIAS));
        IntervalVector gyro_walking_bias(3, Interval(-GYRO_WALKING_BIAS, GYRO_WALKING_BIAS));
        IntervalVector actual_vel(3);
        //add noise to each component
        actual_vel[0] = imupointer->angular_velocity.x + gyro_scale_factor * imupointer->angular_velocity.x + gyro_bias[0]/* +
                        gyro_walking_bias[0] * (end_current_timestamp - start_time)*/;
        actual_vel[1] = imupointer->angular_velocity.y + gyro_scale_factor * imupointer->angular_velocity.y + gyro_bias[1]/* +
                        gyro_walking_bias[1] * (end_current_timestamp - start_time)*/;
        actual_vel[2] = imupointer->angular_velocity.z + gyro_scale_factor * imupointer->angular_velocity.z + gyro_bias[2] /*+
                        gyro_walking_bias[2] * (end_current_timestamp - start_time)*/;
        this->vel_data.emplace_back(make_pair(time_stamp, actual_vel));
        //Accelerometer
        Interval acc_scale_factor = (-ACC_SCALE_FACTOR, ACC_SCALE_FACTOR);
        IntervalVector acc_bias(3, Interval(-ACC_BIAS, ACC_BIAS));
        IntervalVector acc_walking_bias(3, Interval(-ACC_WALKING_BIAS, ACC_WALKING_BIAS));
        IntervalVector actual_acc(3);
        //add noise to each component
        actual_acc[0] = imupointer->linear_acceleration.x + acc_scale_factor * imupointer->linear_acceleration.x + acc_bias[0]/* +
                        acc_walking_bias[0] * (end_current_timestamp - start_time)*/;
        actual_acc[1] = imupointer->linear_acceleration.y + acc_scale_factor * imupointer->linear_acceleration.y + acc_bias[1]/* +
                        acc_walking_bias[1] * (end_current_timestamp - start_time)*/;
        actual_acc[2] = imupointer->linear_acceleration.z + acc_scale_factor * imupointer->linear_acceleration.z + acc_bias[2] /*+
                        acc_walking_bias[2] * (end_current_timestamp - start_time)*/;
        acc_data.emplace_back(make_pair(time_stamp, actual_acc));

    }
    else
    {
        cout<<"IMU Data not correct,abort!!!"<<endl;
        exit(1);
    }
}
