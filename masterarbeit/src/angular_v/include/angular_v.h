#ifndef ANGULAR_V_ANGULAR_V_H
#define ANGULAR_V_ANGULAR_V_H
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <ibex.h>
#include <algorithm>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <eigen3/Eigen/Geometry>
#include <yaml.h>

//#define SCALE_FACTOR .00046875
//#define BIAS .0009375
//#define WALKING_BIAS 0
//#define START_COMPUTE_TIME 1557924743.397252747
//#define END_COMPUTE_TIME   1557924744.690659339
#define YAML_PATH "/home/noah/masterarbeit/src/angular_v/include/Configuration.yml"

using namespace std;
using namespace ibex;

class Parameters
{
private:
    double GYRO_SCALE_FACTOR;
    double GYRO_BIAS;
    double Gyro_WALKING_BIAS;
    double START_COMPUTE_TIME;
    double END_COMPUTE_TIME;
    string BAG_PATH;
public:
    void set_GYRO_SCALE_FACTOR(double value);
    void set_GYRO_BIAS(double value);
    void set_Gyro_WALKING_BIAS(double value);
    void set_START_COMPUTE_TIME(double value);
    void set_END_COMPUTE_TIME(double value);
    void set_Bag_Path(string path);
    double get_GYRO_SCALE_FACTOR();
    double get_GYRO_BIAS();
    double get_Gyro_WALKING_BIAS();
    double get_START_COMPUTE_TIME();
    double get_END_COMPUTE_TIME();
    string get_Bag_Path();

};

class IMU
{
private:
    std::vector<std::pair<ibex::Interval,ibex::IntervalVector>> vel_data;
public:
    void add_vel_measurement(sensor_msgs::ImuConstPtr imupointer,Parameters & parameters);
    ibex::IntervalMatrix vel2rotatation(double start_compute_time, double  end_compute_time);
};

class Measurement
{
private:
    vector<pair<Interval,Eigen::Matrix4d>> tf2imu;
    vector<pair<Interval,geometry_msgs::PoseStamped>> ground_truth;
    vector<pair<geometry_msgs::TransformStamped,geometry_msgs::TransformStamped>> tf_static;
public:
    void add_ground_truth (rosbag::MessageInstance  m);
    void add_tf_static (rosbag::MessageInstance  m, bool & tf_data_aquired);

    Eigen::Matrix4d tf_mms_cam();
    Eigen::Matrix4d tf_cam_imu();

    void transform_gt_imu(Eigen::Matrix4d tf_mms_cam, Eigen::Matrix4d tf_cam_imu, Parameters parameters);
    Eigen::Matrix4d calculate_relative_transformation_imu();
};

class Image
{
public:
    vector<pair<double,cv::Mat>> Raw_Image;
public:
    void add_raw_image(sensor_msgs::ImageConstPtr m, Parameters parameters);
    int find_stationary();
};


ibex::IntervalMatrix calculate_rodrigues_rotation(ibex::IntervalVector angular_vel, double delta_t);
static void toEulerAngle(const Eigen::Quaterniond & q, double & roll, double & pitch , double & yaw);
vector<ibex::Interval> IntervalrotationMatrixtoEulerAngle(IntervalMatrix & matrix);
void load_from_config(Parameters & parameters);
#endif //ANGULAR_V_ANGULAR_V_H
