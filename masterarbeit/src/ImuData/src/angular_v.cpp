#include "angular_v.h"
int main()
{
    Measurement measurement;
    rosbag::Bag bag;
    Parameters parameters;
    LiDAR_PointCloud points(0.999998,0.001,0.02);
    load_from_config(parameters);
    bag.open(parameters.get_Bag_Path());
    vector<string> topics;
    vector<pair<Interval,Eigen::Matrix4d>> tf2imu;
    vector<geometry_msgs::PoseStamped> ground_truth;
    vector<pair<geometry_msgs::TransformStamped,geometry_msgs::TransformStamped>> tf_static;
    IMU imu;
    bool static_data_aqcuired=false;
    IntervalMatrix rotation(3,3);
    IntervalVector state(3,Interval(0));
    geometry_msgs::TransformStamped end;
    geometry_msgs::Vector3 from_beginning_to_end_translation;
    topics.emplace_back("/ground_truth");
    topics.emplace_back("/tf_static");
    topics.emplace_back("/imu/data");
    topics.emplace_back("/velodyne_points");

    rosbag::View view(bag,rosbag::TopicQuery(topics));
    BOOST_FOREACH( rosbag::MessageInstance const  m, view) {
                    if (m.getTopic()=="/ground_truth") {
                        measurement.add_ground_truth(m);
                    }
                    else if (m.getTopic()=="/tf_static" && !static_data_aqcuired)
                    {
                        measurement.add_tf_static(m,static_data_aqcuired);
                    }
                    else if (m.getTopic()=="/imu/data")
                    {
                        imu.add_vel_measurement(m.instantiate<sensor_msgs::Imu>(),parameters);
                    }
                    else if (m.getTopic()=="/velodyne_points")
                    {
                        points.add_pointcloud(m.instantiate<sensor_msgs::PointCloud2>(),parameters);
                    }}
                    for( auto item : points.velocity_0)
                    {
                        cout<<item<<endl;
                    }

    //mms->camera
    Eigen::Matrix4d transform1=Eigen::Matrix4d::Identity();
    //camera->imu
    Eigen::Matrix4d transform2=Eigen::Matrix4d::Identity();
    //mms->camera
    transform1=measurement.tf_mms_cam();
    transform2=measurement.tf_cam_imu();
    measurement.transform_gt_imu(transform1,transform2,parameters);
    rotation=imu.vel2rotatation(parameters.get_START_COMPUTE_TIME(),parameters.get_END_COMPUTE_TIME());
    state=imu.acc2pose(parameters.get_START_COMPUTE_TIME(),parameters.get_END_COMPUTE_TIME(),points);
    //vector<Interval> b;
    //b=IntervalrotationMatrixtoEulerAngle(rotation);
    //cout<<"Interval roll:"<<b[0]<<endl;
    //cout<<"Interval pitch:"<<b[1]<<endl;
    //cout<<"Interval yaw:"<<b[2]<<endl;

    Eigen::Matrix4d relativ_transformation=Eigen::Matrix4d::Identity();
    relativ_transformation=measurement.calculate_relative_transformation_imu();

    Eigen::Matrix4d relativ_transformation_mms=Eigen::Matrix4d::Identity();
    relativ_transformation_mms=transform1.inverse()*relativ_transformation;
    relativ_transformation_mms=transform2.inverse()*relativ_transformation_mms;
    //Eigen::Matrix3d rotation_matrix=relativ_transformation.block(0,0,3,3);
    //Eigen::Quaterniond a(rotation_matrix);
    //double roll, pitch, yaw;
    //toEulerAngle(a,roll,pitch,yaw);
    //cout<<"roll:"<<roll<<endl;
    //cout<<"pitch:"<<pitch<<endl;
    //cout<<"yaw:"<<yaw<<endl;

    //Testing!!!
    bool erg;
    erg=rotation[0][0].contains(relativ_transformation(0,0))&rotation[0][1].contains(relativ_transformation(0,1))&rotation[0][2].contains(relativ_transformation(0,2))&rotation[1][0].contains(relativ_transformation(1,0))&rotation[1][1].contains(relativ_transformation(1,1))&rotation[1][2].contains(relativ_transformation(1,2))&rotation[2][0].contains(relativ_transformation(2,0))&rotation[2][1].contains(relativ_transformation(2,1))&rotation[2][2].contains(relativ_transformation(2,2));
    cout<<erg<<endl;
    bag.close();
    return 0;
}
//
// Created by noah on 11.01.21.
//