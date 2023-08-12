#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "my_utils/my_led_unique.h"
#include "my_utils/my_location.h"

bool utilsRunning = false;

ros::Publisher utils_led_pub;
ros::Publisher utils_pose_pub;
ros::Publisher utils_buzzer_pub;

typedef struct
{
    double x;
    double y;
    double z;
    double yaw;
} body_pose;

void utils_state_cb(const std_msgs::Bool::ConstPtr &msg)
{
    utilsRunning = msg->data;
}

enum LED_Color
{
    LED_Color_Off = 'f',

    LED_Color_Red = 'r',
    LED_Color_Green = 'g',
    LED_Color_Blue = 'b'
};

/**
 * 设置LED灯状态
 * color 选择颜色：
 *  LED_Color_Off 关
 *  LED_Color_Red 红色
 *  LED_Color_Green 绿色
 *  LED_Color_Blue 蓝色
 *
 */
void setLED(char color)
{
    my_utils::my_led_unique led;
    if (color == LED_Color::LED_Color_Off)
    {
        led.enable = false;
    }
    else
    {
        led.color = color;
        led.enable = true;
    }
    utils_led_pub.publish(led);
}

/**
 * 设置蜂鸣器状态
 */
void setBuzzer(bool enable)
{
    std_msgs::Bool data;
    data.data = enable;
    utils_buzzer_pub.publish(data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "utils_test_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    ros::Subscriber utils_state_sub = nh.subscribe<std_msgs::Bool>("/my_utils/state", 10, utils_state_cb);

    utils_led_pub = nh.advertise<my_utils::my_led_unique>("/my_utils/led_unique", 10);

    utils_pose_pub = nh.advertise<my_utils::my_location>("/my_utils/fire", 10);

    utils_buzzer_pub = nh.advertise<std_msgs::Bool>("/my_utils/buzzer", 10);

    ROS_WARN("Waiting Utils Node...");
    while (!utilsRunning)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ros::Duration(1).sleep();

    setLED(LED_Color::LED_Color_Red);
    ROS_WARN("Sent Red");
    ros::Duration(3).sleep();

    setLED(LED_Color::LED_Color_Green);
    ROS_WARN("Sent Green");
    ros::Duration(3).sleep();

    setLED(LED_Color::LED_Color_Blue);
    ROS_WARN("Sent Blue");
    ros::Duration(3).sleep();

    setLED(LED_Color::LED_Color_Off);
    ROS_WARN("Sent Off");

    my_utils::my_location pose;
    pose.pose_x = 1.00;
    pose.pose_y = 2.00;
    pose.pose_z = 1.80;
    utils_pose_pub.publish(pose);
    ros::Duration(3).sleep();
    ROS_WARN("Sent Fire");

    setBuzzer(true);
    ros::Duration(2).sleep();
    setBuzzer(false);

    ros::shutdown();

    return 0;
}