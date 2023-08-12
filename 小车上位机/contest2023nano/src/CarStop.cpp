#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

ros::Publisher car_head_pub;
ros::Publisher car_start_pub;

/**
 * 延时 精度 10 ms
 */
void sleep_ms(float ms_cnt)
{
    for (int i = 0; i < ms_cnt / 10; i++)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "contest2023nano_car_node");
    ros::NodeHandle nh;
    ros::Rate rate(10.0);

    car_head_pub = nh.advertise<std_msgs::Float32>("/my_utils/car_head", 10);
    car_start_pub = nh.advertise<std_msgs::Bool>("/my_utils/car_start", 10);

    sleep_ms(2000);

    std_msgs::Bool car_start_data;
    car_start_data.data = false;
    car_start_pub.publish(car_start_data);

    std_msgs::Float32 car_head_data;
    car_head_data.data = 0;
    car_head_pub.publish(car_head_data);

    return 0;
}