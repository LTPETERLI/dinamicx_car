//
// Created by nepgear on 2024/1/16.
//
#include "LO.h"
#include "nav_msgs/Odometry.h"
npg::icp LO_icp;
ros::Publisher odom;
void cloudcallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    //msg2pc
    auto start = std::chrono::system_clock::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>), clouds(
            new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud_ptr);
    //pc2downsample
    clouds = LO_icp.icp_downsample(cloud_ptr);
    LO_icp.registerlaserdata(clouds);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    //std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    std::cout << "total time:" << elapsed_seconds.count() << std::endl;
    //GN_step
    //build kdtree
    return;
}

void livox_callback(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>), clouds(
            new pcl::PointCloud<pcl::PointXYZ>);
    for (auto i: msg->points) {
        pcl_cloud->points.push_back(pcl::PointXYZ(i.x, i.y, i.z));
    }
    auto start = std::chrono::system_clock::now();
    clouds = LO_icp.icp_downsample(pcl_cloud);
    LO_icp.registerlaserdata(clouds);
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    //std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    std::cout << "location is" << LO_icp.lidar_pose_.inverse().translation() << "\t";
    std::cout << "total time:" << elapsed_seconds.count() << std::endl;
    //GN_step
    //build kdtree
    nav_msgs::Odometry Odom_msg;
    Odom_msg.header.stamp=ros::Time::now();
    Odom_msg.header.frame_id="odom";
    Odom_msg.pose.pose.position.x=LO_icp.lidar_pose_.inverse().translation()(0);
    Odom_msg.pose.pose.position.y=LO_icp.lidar_pose_.inverse().translation()(1);
    Odom_msg.pose.pose.position.z=LO_icp.lidar_pose_.inverse().translation()(2);
    Odom_msg.child_frame_id="base_link";
    odom.publish(Odom_msg);
    return;
}

int main(int argc, char **argv) {
    //FLAGS_log_dir = "/home/nepgear/ws_livox/src/lio/log";
    google::InitGoogleLogging(*argv);
    Eigen::setNbThreads(16);
    Eigen::initParallel();
    ros::init(argc, argv, "LO");
    ros::NodeHandle n;
    ros::param::get("max_min_icp_distance", LO_icp.max_min_icp_distance_);
    ros::param::get("max_iterations", LO_icp.max_iterations_);
    ros::param::get("leaf_size", LO_icp.leaf_size_);
    ros::param::get("factor", LO_icp.factor_);
    LO_icp.deskewed_scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    LO_icp.last_scan_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    ros::Subscriber pc_sub = n.subscribe("/os1_cloud_node/points", 10000, cloudcallback,
                                         ros::TransportHints().tcpNoDelay());
    ros::Subscriber livox_sub = n.subscribe("/livox/lidar", 10000, livox_callback,
                                            ros::TransportHints().tcpNoDelay());
    odom = n.advertise<nav_msgs::Odometry>("this_odom",50);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}