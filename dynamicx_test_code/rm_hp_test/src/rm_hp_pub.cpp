#include <ros/ros.h>
#include "rm_msgs/GameRobotHp.h"
#include "../../rm_referee/include/rm_referee/common/protocol.h"

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "rm_hp_pub_test");
    ros::NodeHandle nh;


    // 创建一个发布者，发布到名为 output_topic 的话题
    ros::Publisher pub = nh.advertise<rm_msgs::GameRobotHp>("/rm_referee/game_robot_hp", 10);

    ros::Rate rate(10);  // 控制发布频率

    while (ros::ok()) {
        rm_msgs::GameRobotHp game_robot_hp_data;
        // 填充 game_robot_hp_data 的数据
        game_robot_hp_data.blue_1_robot_hp = 100;
        game_robot_hp_data.blue_2_robot_hp = 100;
        game_robot_hp_data.blue_3_robot_hp = 100;
        game_robot_hp_data.blue_4_robot_hp = 100;
        game_robot_hp_data.blue_5_robot_hp = 100;
        game_robot_hp_data.blue_7_robot_hp = 100;
        game_robot_hp_data.blue_base_hp = 100;
        game_robot_hp_data.blue_outpost_hp = 150;
        game_robot_hp_data.red_1_robot_hp = 100;
        game_robot_hp_data.red_2_robot_hp = 100;
        game_robot_hp_data.red_3_robot_hp = 100;
        game_robot_hp_data.red_4_robot_hp = 100;
        game_robot_hp_data.red_5_robot_hp = 100;
        game_robot_hp_data.red_7_robot_hp = 100;
        game_robot_hp_data.red_base_hp = 100;
        game_robot_hp_data.red_outpost_hp = 150;
        game_robot_hp_data.stamp = ros::Time::now();
        // 发布消息到 output_topic
        pub.publish(game_robot_hp_data);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
