//
// Created by nepgear on 2023/10/9.
//
#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl/io/pcd_io.h>//which contains the required definitions to load and store point clouds to PCD and other file formats.

#include "Eigen/Eigen"
#include "sophus/se3.hpp"
//DEFINE_bool(localtion, false,"judge if in robot or local");
int main (int argc, char **argv)
{
    ros::init (argc, argv, "UandBdetect");
    ros::NodeHandle nh;
    //gflags::ParseCommandLineFlags(&argc, &argv, true);
    std::cout<<"123"<<std::endl;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;

    Sophus::SO3f R;
    R = R * Sophus::SO3f::exp(Eigen::Vector3f(0,3.1415926/4 ,0));
    Eigen::Matrix3f R_m;
    R_m = R.matrix();
//    R_m.setOnes();
    std::string my_pcd = "/home/nepgear/archieve/src/FAST_LIO/PCD/scans.pcd",dest = "/home/dynamicx/rm_ws/src/FAST_LIO/PCD/scans.pcd";
    if(0)
    pcl::io::loadPCDFile (dest, cloud);
    else pcl::io::loadPCDFile (my_pcd, cloud);
    // your directory
    //Convert the cloud to ROS message
    //create a pitch vector

    for(auto &i : cloud.points)
    {
        using namespace Eigen;
        Vector3f xyz{i.x,i.y,i.z};
        xyz = xyz.eval()-Vector3f(-0.011, -0.0234, 0.044);
        xyz = R_m*xyz;
        Sophus::SO3f R0;
        R0 = R0 * Sophus::SO3f::exp(Eigen::Vector3f(0,0 ,-3.1415926/2));
        Eigen::Matrix3f R0_m;
        R0_m = R0.matrix();
        xyz = R0_m*xyz.eval();
        i.x = xyz.x();
        i.y = xyz.y();
        i.z = xyz.z();

//        else i.z=0;

    }
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "odom";
    pcl::io::savePCDFile("/home/nepgear/archieve/src/FAST_LIO/PCD/new_scans.pcd",cloud);
    //this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer
    ros::Rate loop_rate(1);

    return 0;
}

