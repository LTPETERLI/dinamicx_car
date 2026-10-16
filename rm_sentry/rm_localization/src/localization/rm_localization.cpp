//
// Created by aung on 2023/3/4.
//

#include "rm_localization/localization/receiver.h"

int main(int argc, char** argv)
{
  std::shared_ptr<rm_localization::RmDetectionReceiver> rm_detection_receiver;
  std::shared_ptr<rm_localization::AprilTagReceiver> apriltag_receiver;
  tf2_ros::Buffer* tf_buffer;
  std::string camera_frame;
  std::string tags_type;
  int moving_average_num;

  ros::init(argc, argv, "rm_localization");
  ros::NodeHandle nh("~");

  tf_buffer = new tf2_ros::Buffer(ros::Duration(10));
  camera_frame = getParam(nh, "camera_frame", static_cast<std::string>("camera_optical_frame"));
  tags_type = getParam(nh, "tags_type", static_cast<std::string>("rm_tag"));
  moving_average_num = getParam(nh, "moving_average_num", 5);
  rm_detection_receiver = std::make_shared<rm_localization::RmDetectionReceiver>(nh, camera_frame, moving_average_num,
                                                                                 tf_buffer, "/tag_pnp_publisher");
  apriltag_receiver = std::make_shared<rm_localization::AprilTagReceiver>(nh, camera_frame, moving_average_num,
                                                                          tf_buffer, "/tag_detections");
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    if (tags_type == "rm_tag")
      rm_detection_receiver->mapToOdomBroadcaster();
    else if (tags_type == "apriltag")
      apriltag_receiver->mapToOdomBroadcaster();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
