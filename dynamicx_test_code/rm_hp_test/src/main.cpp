/*** 
 * @Author: wangzhaowu
 * @Date: 2023-10-27 15:03:32
 * @LastEditTime: 2023-10-27 20:39:15
 * @LastEditors: wangzhaowu
 * @Description: 
 * @FilePath: /catkin_workspace/src/rm_hp_test/src/main.cpp
 * @Verse
 */
#include <ros/ros.h>
#include "rm_msgs/GameRobotHp.h"
#include "rm_msgs/GameRobotStatus.h"
#include "rm_referee/common/protocol.h"


static void Robot_status_callback(const rm_msgs::GameRobotStatus::ConstPtr &hpMsg);

int main(int argc, char *argv[])
{
    // 初始化ROS节点
    ros::init(argc, argv, "rm_hp_test");
    ros::NodeHandle nh;

    // 创建一个订阅者，订阅名为 game_robot_status_subscriber 的话题
    ros::Subscriber game_robot_status_subscriber;

    game_robot_status_subscriber = nh.subscribe<rm_msgs::GameRobotStatus>("/rm_referee/game_robot_status", 10, Robot_status_callback);

    ROS_INFO("--------------------RM_HP_TEST--------------------");

    while(ros::ok()) {

        ros::spinOnce();
    }
}



static void Robot_status_callback(const rm_msgs::GameRobotStatus::ConstPtr &hpMsg)
{
    ROS_INFO("Robot Max hp:%d", hpMsg->max_hp);
    ROS_INFO("Robot Remain hp:%d", hpMsg->remain_hp);


}

