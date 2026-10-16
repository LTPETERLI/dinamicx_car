//
// Created by yawara on 23-8-29.
//

#include "movegimbal.h"

namespace behavior_tree{
MoveGimbal::MoveGimbal(const std::string &name, const BT::NodeConfiguration &config, ros::NodeHandle &root_nh,
                       const ros::NodeHandle &decision_nh)
                        : BT::SyncActionNode(name, config) {

    gimbal_pub_ = std::make_unique<realtime_tools::RealtimePublisher<rm_msgs::GimbalCmd>>(
            root_nh, "/controllers/gimbal_controller/command", 1);
    ros::NodeHandle movegimabal_nh(decision_nh, "movegimbal");
    tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
}

void MoveGimbal::MoveGimbalRate() {
    try{
        base2yaw = tf_buffer_.lookupTransform("base_link","yaw",ros::Time(0));
        yaw2pitch = tf_buffer_.lookupTransform("yaw","pitch",ros::Time(0));
    }catch(tf2::TransformException& ex){
        ROS_WARN("%s", ex.what());
        return;
    }

//    double yaw_delta = yawFromQuat(base2yaw.transform.rotation);
    double roll_temp, pitch_delta, yaw_temp;
    quatToRPY(yaw2pitch.transform.rotation, roll_temp, pitch_delta, yaw_temp);

    if(gimbal_pub_->trylock()){
        gimbal_pub_->msg_.mode = gimbal_pub_->msg_.RATE;
        gimbal_pub_->msg_.stamp = ros::Time::now();
        if(pitch_delta > pitch_upper_limit_){
            gimbal_pub_->msg_.rate_pitch = -pitch_rate_;
        }else if(pitch_delta < pitch_lower_limit_){
            gimbal_pub_->msg_.rate_pitch = pitch_rate_;
        }
        gimbal_pub_->msg_.rate_yaw = yaw_rate_;
        gimbal_pub_->unlockAndPublish();
    }
}

BT::PortsList MoveGimbal::providedPorts(){
    BT::PortsList ports_list;
    return ports_list;
}

BT::NodeStatus MoveGimbal::tick(){
    MoveGimbalRate();
    return BT::NodeStatus::SUCCESS;
}

}// behavior_tree