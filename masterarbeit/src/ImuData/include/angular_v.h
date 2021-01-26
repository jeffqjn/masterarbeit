#ifndef ANGULAR_V_ANGULAR_V_H
#define ANGULAR_V_ANGULAR_V_H

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <iostream>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <ibex.h>
#include <algorithm>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <eigen3/Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/time.h>
#include <pcl/registration/icp.h>

//#include <opencv2/opencv.hpp>
//#include <opencv2/nonfree/features2d.hpp>

#include <yaml.h>

//#define SCALE_FACTOR .00046875
//#define BIAS .0009375
//#define WALKING_BIAS 0
//#define START_COMPUTE_TIME 1557924743.397252747
//#define END_COMPUTE_TIME   1557924744.690659339
#define YAML_PATH "/home/noah/masterarbeit/src/ImuData/include/Configuration.yml"

using namespace std;
using namespace ibex;

class Parameters
{
private:
    double GYRO_SCALE_FACTOR;
    double GYRO_BIAS;
    double Gyro_WALKING_BIAS;
    double ACC_SCALE_FACTOR;
    double ACC_BIAS;
    double ACC_WALKING_BIAS;
    double START_COMPUTE_TIME;
    double END_COMPUTE_TIME;
    string BAG_PATH;
    int Iterations;
public:
    void set_GYRO_SCALE_FACTOR(double value);
    void set_GYRO_BIAS(double value);
    void set_Gyro_WALKING_BIAS(double value);

    void set_ACC_SCALE_FACTOR(double value);
    void set_ACC_BIAS(double value);
    void set_ACC_WALKING_BIAS(double value);

    void set_START_COMPUTE_TIME(double value);
    void set_END_COMPUTE_TIME(double value);
    void set_Bag_Path(string path);
    void set_Iterations(int times);
    double get_GYRO_SCALE_FACTOR();
    double get_GYRO_BIAS();
    double get_Gyro_WALKING_BIAS();

    double get_ACC_SCALE_FACTOR();
    double get_ACC_BIAS();
    double get_ACC_WALKING_BIAS();

    double get_START_COMPUTE_TIME();
    double get_END_COMPUTE_TIME();
    string get_Bag_Path();
    int get_Iterations();

};
class LiDAR_PointCloud
{
public:
    vector<pair<double,pcl::PointCloud<pcl::PointXYZ>>> pointclouds;
    vector<Interval> velocity_0;
    Eigen::Matrix4d threshold_matrix= Eigen::Matrix4d::Identity ();
    Eigen::Matrix4d Template;
public:
    LiDAR_PointCloud(double leading_diagonal_threshold, double sub_diagonal_threshold, double tranlation_threshold);
    void add_pointcloud(sensor_msgs::PointCloud2ConstPtr m, Parameters parameters);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_convert(const boost::shared_ptr<const sensor_msgs::PointCloud2> & input);
    void compare_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc1, pcl::PointCloud<pcl::PointXYZ>::Ptr pc2, int Iteration);
    Eigen::Matrix4d transformation_matrix_round(Eigen::Matrix4d transformation_matrix);
    Interval get_stationary_time();
};
class IMU
{
private:
    std::vector<std::pair<ibex::Interval,ibex::IntervalVector>> vel_data;
    std::vector<std::pair<ibex::Interval,ibex::IntervalVector>> acc_data;
public:
    void add_vel_measurement(sensor_msgs::ImuConstPtr imupointer,Parameters & parameters);
    ibex::IntervalMatrix vel2rotatation(double start_compute_time, double  end_compute_time);
    ibex::IntervalVector acc2pose(double start_compute_time, double  end_compute_time, LiDAR_PointCloud pointCloud);
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



ibex::IntervalMatrix calculate_rodrigues_rotation(ibex::IntervalVector angular_vel, double delta_t);
static void toEulerAngle(const Eigen::Quaterniond & q, double & roll, double & pitch , double & yaw);
vector<ibex::Interval> IntervalrotationMatrixtoEulerAngle(IntervalMatrix & matrix);
void load_from_config(Parameters & parameters);
double double_round(double dVal, int iPlaces);
#endif //ANGULAR_V_ANGULAR_V_H
