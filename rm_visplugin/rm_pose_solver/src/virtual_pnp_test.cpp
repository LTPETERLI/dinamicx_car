//
// Created by kaxns on 11/7/21.
//

#include <cmath>
#include <string>
#include <utility>
#include <boost/shared_ptr.hpp>
#include <pluginlib/class_loader.h>
#include "rm_pose_solver/axis_mapping.h"
#include "rm_pose_solver/posesolver_plugins.h"
#include "rm_pose_solver/rotation_translation.h"

#if CV_MAJOR_VERSION == 3
int main()
{
  ROS_WARN("This code does not support OpenCV3.");
  return 0;
}
#elif CV_MAJOR_VERSION == 4

namespace rm_pose_solver_virtual_pnp_test
{
#define ENABLE_HZ_TEST 0  // HZ means Hertz

struct AxesMapping
{
  volatile static int x_;
  volatile static int y_;
};
volatile int AxesMapping::x_{ 1 };
volatile int AxesMapping::y_{ 2 };

auto axesMappingSrv(rm_pose_solver::axis_mapping::Request& req, rm_pose_solver::axis_mapping::Response& rsp)
{
  auto val1 = abs(req.x), val2 = abs(req.y);
  if (val1 != val2 && std::max<int>(val1, val2) <= 3 && val1 * val2 != 0)
  {
    AxesMapping::x_ = req.x;
    AxesMapping::y_ = req.y;
    rsp.result = "Set mapping to { " + std::to_string(req.x) + ", " + std::to_string(req.y) + " }.";
    rsp.notice = "Success";
  }
  else
  {
    rsp.result = "The request is wrong.";
    rsp.notice = "Abs of each element in the mapping should be unique and in range [1, 3].";
  }
  return true;
}

struct TranslationRotation
{
  volatile static cv::Point3d translation_vec_;
  volatile static cv::Point3d rotation_vec_;
};
volatile cv::Point3d TranslationRotation::translation_vec_{ 0, 0, 0.5 };
volatile cv::Point3d TranslationRotation::rotation_vec_{};

auto translationRotationSrv(rm_pose_solver::rotation_translation::Request& req,
                            rm_pose_solver::rotation_translation::Response& rsp)
{
  TranslationRotation::translation_vec_.x = req.t_x;
  TranslationRotation::translation_vec_.y = req.t_y;
  TranslationRotation::translation_vec_.z = req.t_z;

  rsp.result = "Translation is now = { " + std::to_string(req.t_x) + ", " + std::to_string(req.t_y) + ", " +
               std::to_string(req.t_z) + " }  Rotation is now = { x: " + std::to_string(req.r_x) +
               ", y: " + std::to_string(req.r_y) + ", z: " + std::to_string(req.r_z) + " }";

  // rotation: axis_with_angle_magnitude(degrees)
  TranslationRotation::rotation_vec_.x = req.r_x / 360 * CV_2PI;
  TranslationRotation::rotation_vec_.y = req.r_y / 360 * CV_2PI;
  TranslationRotation::rotation_vec_.z = req.r_z / 360 * CV_2PI;

  return true;
}

class VirtualCam
{
public:
  explicit VirtualCam(decltype(std::declval<sensor_msgs::CameraInfo>().height) const frame_height,
                      decltype(std::declval<sensor_msgs::CameraInfo>().width) const frame_width, double const fx,
                      double const fy,
                      decltype(std::declval<sensor_msgs::CameraInfo>().roi.x_offset) const x_offset = 0,
                      decltype(x_offset) y_offset = 0)
  {
    virtual_cam_info_.header.frame_id = std::string("camera_optical_frame");
    virtual_cam_info_.header.seq = 0;
    virtual_cam_info_.height = frame_height;
    virtual_cam_info_.width = frame_width;
    virtual_cam_info_.roi.x_offset = x_offset;
    virtual_cam_info_.roi.y_offset = y_offset;
    // clang-format off
    virtual_cam_info_.K = decltype(virtual_cam_info_.K){ fx, 0, static_cast<double>(frame_width / 2),
                                                       0, fy, static_cast<double>(frame_height / 2),
                                                       0, 0, 1 };
    // clang-format on
    virtual_cam_info_.D = decltype(virtual_cam_info_.D){ 0.0, 0.0, 0.0, 0.0, 0.0 };
  }

  sensor_msgs::CameraInfoConstPtr& getVirtualCamInfo()
  {
    virtual_cam_info_.header.seq += 1;
    virtual_cam_info_.header.stamp = ros::Time::now();
    virtual_cam_info_const_ptr_ = boost::make_shared<const sensor_msgs::CameraInfo>(virtual_cam_info_);
    return this->virtual_cam_info_const_ptr_;
  }

private:
  sensor_msgs::CameraInfoConstPtr virtual_cam_info_const_ptr_;
  sensor_msgs::CameraInfo virtual_cam_info_;
};

class VirtualImg
{
public:
  VirtualImg(VirtualCam& v_cam)
  {
    cv_bridge::CvImage virtual_img;
    virtual_img.header.frame_id = v_cam.getVirtualCamInfo()->header.frame_id;
    virtual_img.header.seq = 0;
    virtual_img.encoding = std::string("bgr8");
    virtual_img.image.create(v_cam.getVirtualCamInfo()->height, v_cam.getVirtualCamInfo()->width, CV_8UC3);
    virtual_img.image = cv::Scalar(100, 100, 100);
    this->virtual_img_ptr_ = boost::make_shared<cv_bridge::CvImage>(virtual_img);
  }

  void resetImg()
  {
    this->virtual_img_ptr_->image = cv::Scalar(100, 100, 100);
    cv::line(virtual_img_ptr_->image, cv::Point(0, virtual_img_ptr_->image.rows / 2),
             cv::Point(virtual_img_ptr_->image.cols, virtual_img_ptr_->image.rows / 2), cv::Scalar());
    cv::line(virtual_img_ptr_->image, cv::Point(virtual_img_ptr_->image.cols / 2, 0),
             cv::Point(virtual_img_ptr_->image.cols / 2, virtual_img_ptr_->image.rows), cv::Scalar());
  }

  cv_bridge::CvImagePtr& getVirtualImg()
  {
    return virtual_img_ptr_;
  }

private:
  cv_bridge::CvImagePtr virtual_img_ptr_;
};

void getRotationCam2World(cv::Matx33d* rot)
{
  cv::Vec3d r_vec{ TranslationRotation::rotation_vec_.x, TranslationRotation::rotation_vec_.y,
                   TranslationRotation::rotation_vec_.z };
  cv::Rodrigues(r_vec, *rot);
}

}  // namespace rm_pose_solver_virtual_pnp_test

int main(int argc, char** argv)
{
  using namespace rm_pose_solver_virtual_pnp_test;

  ros::init(argc, argv, "pnp_test");
  ros::NodeHandle parent_nh("~");
  ros::ServiceServer service1 = parent_nh.advertiseService("adjust_axis_mapping", axesMappingSrv);
  ros::ServiceServer service2 = parent_nh.advertiseService("translation_and_rotation", translationRotationSrv);

  // hz means Hertz
  ros::Publisher hz_test_pub = parent_nh.advertise<std_msgs::Header>("/rm_pose_solver/virtual_pnp_test_hz", 10);
  // create a ClassLoader that we'll use to load PnpPoseSolver plugin
  pluginlib::ClassLoader<rm_pose_solver::PnpPoseSolverPluginInterface> pnp_loader(
      "rm_pose_solver", "rm_pose_solver::PnpPoseSolverPluginInterface");
  try
  {
    boost::shared_ptr<rm_pose_solver::PnpPoseSolverPluginInterface> pnp_method =
        pnp_loader.createInstance("rm_pose_solver::PnpPoseSolverPluginImpl");

    VirtualCam virtual_camera(1024, 1280, 1274, 1275);
    sensor_msgs::CameraInfoConstPtr virtual_cam_info_const_ptr = virtual_camera.getVirtualCamInfo();
    VirtualImg virtual_img(virtual_camera);
    cv_bridge::CvImagePtr virtual_img_ptr = virtual_img.getVirtualImg();

    double height = 0.06;
    double width = 0.123;
    double vector_length = 0.01;
    parent_nh.setParam("armor_size/large_height", height);
    parent_nh.setParam("armor_size/large_width", width);
    parent_nh.setParam("vector_length", vector_length);
    parent_nh.setParam("input_mapping/first", 0);
    parent_nh.setParam("input_mapping/second", 1);
    parent_nh.setParam("input_mapping/third", 2);
    parent_nh.setParam("input_mapping/fourth", 3);

    typedef cv::Vec3d Point3DType;

    /*
     3D object points in the world coordinate system
              . +z
     1-----A-/---4
     |      /    |
     B     o-----D-------> x
     |     |     |
     2_____C_____3
           |
           |
           v +y
     * obj_points has { A, B, C, D }
     * input_vertex has { 1, 2, 3, 4 }
     */
    std::vector<Point3DType> obj_points, input_vertex;
    Point3DType p_a(0, -height / 2, 0);       // A
    Point3DType p_b(-width / 2, 0, 0);        // B
    Point3DType p_c(0, height / 2, 0);        // C
    Point3DType p_d(width / 2, 0, 0);         // D
    Point3DType origin(0, 0, 0);              // origin
    Point3DType n_vec(0, 0, -vector_length);  // normal vector
    obj_points = { p_a, p_b, p_c, p_d, origin, n_vec };

    Point3DType p1(-width / 2, -height / 2, 0);  // 1
    Point3DType p2(-width / 2, height / 2, 0);   // 2
    Point3DType p3(width / 2, height / 2, 0);    // 3
    Point3DType p4(width / 2, -height / 2, 0);   // 4
    input_vertex = { p1, p2, p3, p4 };

    cv::Matx33d cam_mat;  // camera intrinsics
    memcpy(&cam_mat.val[0], virtual_cam_info_const_ptr->K.data(), 9 * sizeof(double));

    // * START * /////////////////////////////////////////////////////////////////////////////////////////////////////
    // ***************************************************************************************************************
    // For typical cases including DEBUGGING and RACING,
    // it is recommended to use different overloaded functions for initialization and running.
    // Also, depending on the macro settings, the initialization function and the run function must match:
    // ---------------------------------------------------------------------------------------------------------------
    //       DEBUGGING
    // ---------------------------------------------------------------------------------------------------------------
    // enable ENABLE_POSE_SOLVER_RESULT_TEST
    // use  { initializeOnlyOnce(NodeHandle), initializeAndRun(Targets, CameraInfo, Img) }
    // or   { initializeOnlyOnce(NodeHandle, CameraInfo), initializeAndRun(Targets, CameraInfo, Img) }
    //
    // ---------------------------------------------------------------------------------------------------------------
    //       RACING
    // ---------------------------------------------------------------------------------------------------------------
    // disable ENABLE_POSE_SOLVER_RESULT_TEST
    // use  { initializeOnlyOnce(NodeHandle), initializeAndRun(Targets, CameraInfo) }
    // or   { initializeOnlyOnce(NodeHandle, CameraInfo), initializeAndRun(Targets, CameraInfo) }
    // ***************************************************************************************************************

#if ENABLE_HZ_TEST == 0    // This part is used to verify correctness.
    pnp_method->initializeOnlyOnce(parent_nh, virtual_cam_info_const_ptr);
    while (ros::ok())
    {
      cv::Vec3d t_vec(TranslationRotation::translation_vec_.x, TranslationRotation::translation_vec_.y,
                      TranslationRotation::translation_vec_.z);  // translation from camera to world
      // 标准照明距离, used to render plane lighting(平方反比定律)
      double standard_illumination_distance = t_vec.dot(t_vec);

      virtual_cam_info_const_ptr = virtual_camera.getVirtualCamInfo();
      parent_nh.setParam("axis_mapping/x", AxesMapping::x_);
      parent_nh.setParam("axis_mapping/y", AxesMapping::y_);

      pnp_method->paramReConfig();  // Update axis_mapping_, vector_length, etc.
      virtual_img.resetImg();

      cv::Matx33d rot;  // rotation from camera to world
      getRotationCam2World(&rot);

      rm_vision::ProcessorInterface::Object targets;
      targets.id = { 1 };
      targets.points.clear();
      std::vector<cv::Point2d> image_of_input_vertex;

      // Coordinate transformation and imaging of vertices
      for (unsigned int i = 0; i < input_vertex.size(); ++i)
      {
        Point3DType pt = rot * input_vertex[i] + t_vec;
        pt = cam_mat * pt;
        image_of_input_vertex.emplace_back(cv::Point2d(pt[0] / pt[2], pt[1] / pt[2]));
        cv::circle(virtual_img_ptr->image, image_of_input_vertex[i], 3, cv::Scalar(0, 255, 0), -1);
      }
      targets.points.emplace_back(image_of_input_vertex);

      // Rendering plane
      for (auto x = p1[0]; x < p3[0]; x += 0.0001)
      {
        for (auto y = p1[1]; y < p3[1]; y += 0.0001)
        {
          Point3DType pt = rot * Point3DType(x, y, 0) + t_vec;
          double dist = pt.dot(pt);
          pt = cam_mat * pt;
          auto value = 255 * (1.5 - (dist / standard_illumination_distance));
          value = value > 255 ? 255 : value;
          auto p = virtual_img_ptr->image.at<cv::Vec3b>(pt[1] / pt[2], pt[0] / pt[2]).val;
          p[0] = p[1] = value >= 0 ? static_cast<uchar>(value) : 0;
          p[2] = 0;
        }
      }

      std::vector<cv::Point2i> image_of_obj_points;
      for (auto pt : obj_points)
      {
        pt = rot * pt + t_vec;  // translate and rotate each 3D point
        pt = cam_mat * pt;
        image_of_obj_points.emplace_back(cv::Point(pt[0] / pt[2], pt[1] / pt[2]));
      }
      for (auto pt = image_of_obj_points.begin(); pt < image_of_obj_points.end() - 1; ++pt)
      {
        cv::circle(virtual_img_ptr->image, *pt, 3, cv::Scalar(0, 255, 255), -1);
      }
      cv::putText(virtual_img_ptr->image, "A", *image_of_obj_points.begin(), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar());
      cv::putText(virtual_img_ptr->image, "B", *(image_of_obj_points.begin() + 1), cv::FONT_HERSHEY_SIMPLEX, 1,
                  cv::Scalar());
      cv::putText(virtual_img_ptr->image, "C", *(image_of_obj_points.begin() + 2), cv::FONT_HERSHEY_SIMPLEX, 1,
                  cv::Scalar());
      cv::putText(virtual_img_ptr->image, "D", *(image_of_obj_points.begin() + 3), cv::FONT_HERSHEY_SIMPLEX, 1,
                  cv::Scalar());
      // Draw normal vector
      cv::line(virtual_img_ptr->image, *(image_of_obj_points.end() - 1), *(image_of_obj_points.end() - 2),
               cv::Scalar(0, 255, 255), 2);

      pnp_method->execute(targets, virtual_cam_info_const_ptr, virtual_img_ptr);
      ros::spinOnce();
    }
#elif ENABLE_HZ_TEST == 1  // This part is used to test the performance.
    rm_vision::ProcessorInterface::Object targets;
    std::vector<rm_vision::ProcessorInterface::Object> target_set{};
    cv::Matx33d rot;

    // Create a target_set of type std::vector<rm_vision::ProcessorInterface::Object>
    // to test the real-time performance of pose_solver.
    service1.shutdown();
    service2.shutdown();
    TranslationRotation::rotation_vec_.x = 0;
    TranslationRotation::rotation_vec_.y = 0;
    TranslationRotation::rotation_vec_.z = 0;
    while (TranslationRotation::rotation_vec_.x < 30)
    {
      TranslationRotation::rotation_vec_.x += 1;
      TranslationRotation::rotation_vec_.y += 1;
      TranslationRotation::rotation_vec_.z += 1;
      getRotationCam2World(&rot);

      targets.id.clear();
      targets.points.clear();
      std::vector<cv::Point2d> set_of_points;

      cv::Vec3d t_vec(TranslationRotation::translation_vec_.x, TranslationRotation::translation_vec_.y,
                      TranslationRotation::translation_vec_.z);  // translation from camera to world
      // Coordinate transformation and imaging of vertices
      for (auto& pt : input_vertex)
      {
        Point3DType temp = rot * pt + t_vec;
        temp = cam_mat * temp;
        set_of_points.emplace_back(cv::Point2d(temp[0] / temp[2], temp[1] / temp[2]));
      }
#define TARGET_NUMS 1
      for (int i = 0; i < TARGET_NUMS; ++i)
      {
        targets.points.emplace_back(set_of_points);
        targets.id.emplace_back(1);
      }
      target_set.emplace_back(targets);
    }

    pnp_method->initializeOnlyOnce(parent_nh, virtual_cam_info_const_ptr);

    while (ros::ok())
    {
      for (auto& it : target_set)
      {
        pnp_method->initializeAndRun(it, virtual_cam_info_const_ptr);
        hz_test_pub.publish(std_msgs::Header());
      }
    }
#endif
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
  return 0;
}

#endif