//
// Created by nepgear on 2024/1/10.
//
#include "gtest/gtest.h"
#include "odom.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>

//todo: icp done , tbb or omp accelerate and add ros suport;
int main(int argc, char **argv) {
    FLAGS_log_dir = "/home/nepgear/ws_livox/src/lio/log";
    google::InitGoogleLogging(*argv);
    Eigen::setNbThreads(16);
    Eigen::initParallel();
    npg::icp test_icp;
    pcl::PointCloud<pcl::PointXYZ> cloud, new_cloud, last_cloud;
    std::string my_pcd = "/home/nepgear/archieve/src/FAST_LIO/PCD/scans.pcd";
    pcl::io::loadPCDFile(my_pcd, cloud);
    Eigen::Vector3d axis_rotate(0, 0, 3.1415926 / 8);
    Sophus::SE3d T_star(Sophus::SO3d::exp(axis_rotate), Eigen::Vector3d(1.5, 2.5, 4.5));
    for (auto i: cloud.points) {
        Eigen::Vector3d p(i.x, i.y, i.z);
        if (p.norm() <= 2) {
            auto p2 = T_star * p.eval();
            pcl::PointXYZ p3;
            p3.x = p2(0);
            p3.y = p2(1);
            p3.z = p2(2);
            new_cloud.points.push_back(p3);
            last_cloud.points.push_back(i);
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_ptr(new pcl::PointCloud<pcl::PointXYZ>), last_ptr(
            new pcl::PointCloud<pcl::PointXYZ>);
    *new_ptr = new_cloud;
    *last_ptr = last_cloud;
    *new_ptr = *test_icp.icp_downsample(new_ptr);
    *last_ptr = *test_icp.icp_downsample(last_ptr);
    test_icp.deskewed_scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    test_icp.last_scan_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    test_icp.registerlaserdata(last_ptr);

    test_icp.registerlaserdata(new_ptr);
    std::cout << test_icp.lidar_pose_.inverse().translation() << std::endl;
    std::cout << test_icp.ll_cost << std::endl;

}