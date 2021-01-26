#include <angular_v.h>
LiDAR_PointCloud::LiDAR_PointCloud(double leading_diagonal_threshold, double sub_diagonal_threshold, double tranlation_threshold){
    //build threshold matrix
    threshold_matrix(0,0)=leading_diagonal_threshold;
    threshold_matrix(1,1)=leading_diagonal_threshold;
    threshold_matrix(2,2)=leading_diagonal_threshold;
    threshold_matrix(0,1)=sub_diagonal_threshold;
    threshold_matrix(0,2)=sub_diagonal_threshold;
    threshold_matrix(1,0)=sub_diagonal_threshold;
    threshold_matrix(1,2)=sub_diagonal_threshold;
    threshold_matrix(2,0)=sub_diagonal_threshold;
    threshold_matrix(2,1)=sub_diagonal_threshold;
    threshold_matrix(0,3)=tranlation_threshold;
    threshold_matrix(1,3)=tranlation_threshold;
    threshold_matrix(2,3)=tranlation_threshold;
    threshold_matrix(3,3)=1;
    threshold_matrix(3,0)=0;
    threshold_matrix(3,1)=0;
    threshold_matrix(3,2)=0;
    //build Template
    Template=Eigen::Matrix4d::Identity ();
}
pcl::PointCloud<pcl::PointXYZ>::Ptr LiDAR_PointCloud::cloud_convert(const boost::shared_ptr<const sensor_msgs::PointCloud2> & input) {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    return temp_cloud;
}

Interval LiDAR_PointCloud::get_stationary_time()
{
    return velocity_0[0];
}

void LiDAR_PointCloud::add_pointcloud(sensor_msgs::PointCloud2ConstPtr m, Parameters parameters) {
    if((m->header.stamp.toSec()>parameters.get_START_COMPUTE_TIME()&&m->header.stamp.toSec()<parameters.get_END_COMPUTE_TIME())||(m->header.stamp.toSec()>parameters.get_END_COMPUTE_TIME()&&m->header.stamp.toSec()-parameters.get_END_COMPUTE_TIME()<0.1))
    {
        pcl::PointCloud<pcl::PointXYZ> pc2;

        if(pointclouds.empty())
        {
            pc2=*cloud_convert(m);
            pointclouds.emplace_back(make_pair(m->header.stamp.toSec(),pc2));
        }
        else
        {
            pc2=*cloud_convert(m);
            compare_pc(pointclouds.back().second.makeShared(),pc2.makeShared(),parameters.get_Iterations());
            pointclouds.emplace_back(make_pair(m->header.stamp.toSec(),pc2));
        }

    }

}
Eigen::Matrix4d LiDAR_PointCloud::transformation_matrix_round(Eigen::Matrix4d transformation_matrix)
{
    Eigen::Matrix4d temp=Eigen::Matrix4d::Identity();
    temp=transformation_matrix.array().abs().matrix();
    if(temp(0,0)>threshold_matrix(0,0) && temp(1,1)>threshold_matrix(1,1) && temp(2,2)>threshold_matrix(2,2)&& temp(0,1)<threshold_matrix(0,1) && temp(0,2)<threshold_matrix(0,2) && temp(1,0)<threshold_matrix(1,0) && temp(1,2)<threshold_matrix(1,2) && temp(2,0)<threshold_matrix(2,0) &&temp(2,1)<threshold_matrix(2,1) && temp(0,3)<threshold_matrix(0,3) && temp(1,3)<threshold_matrix(1,3) && temp(2,3)<threshold_matrix(2,3))
    {
        return Template;
    }
    else
    {
        return transformation_matrix;
    }
}

void LiDAR_PointCloud::compare_pc(pcl::PointCloud<pcl::PointXYZ>::Ptr pc1, pcl::PointCloud<pcl::PointXYZ>::Ptr pc2, int Iteration) {
//The ICP algorithm
pcl::console::TicToc time;
pcl::PointCloud<pcl::PointXYZ> temp1;
pcl::PointCloud<pcl::PointXYZ> temp2;
Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
Eigen::Matrix4d round_transformation_matrix = Eigen::Matrix4d::Identity ();
temp1=*pc1;
temp2=*pc2;
//time.tic();
pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
icp.setMaximumIterations(Iteration);
icp.setInputSource(temp1.makeShared());
icp.setInputTarget(temp2.makeShared());
icp.align(temp1);
icp.setMaximumIterations(1);
//cout << "Applied " << 5 << " ICP iteration(s) in " << time.toc () << " ms" <<endl;

if (icp.hasConverged())
{
    //cout << "\nICP has converged, score is " << icp.getFitnessScore () << endl;
    //cout << "\nICP transformation " << 5 << " : cloud_icp -> cloud_in" << endl;
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    //set precision
    //cout<<transformation_matrix<<endl;
    round_transformation_matrix=transformation_matrix_round(transformation_matrix);
    //cout<<round_transformation_matrix<<endl;
    if(round_transformation_matrix.isIdentity())
    {
        velocity_0.push_back(Interval(pc1->header.stamp,pc2->header.stamp));
    }


}
else
{
    PCL_ERROR ("\nICP has not converged.\n");
}
}

//
// Created by noah on 25.01.21.
//

