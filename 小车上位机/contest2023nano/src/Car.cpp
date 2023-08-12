#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include "T265Pose.hpp"

t265Pose t265_pose;

int runMode = 0;
bool utils_state = false;

ros::Publisher servo_angle;
ros::Publisher car_head_pub;
ros::Publisher car_start_pub;

void utils_state_cb(const std_msgs::Bool::ConstPtr &msg)
{
    utils_state = msg->data;
}

void t265_pose_cb(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    t265_pose.x = odom_msg->pose.pose.position.x; // 单位m
    t265_pose.y = odom_msg->pose.pose.position.y;
    t265_pose.z = odom_msg->pose.pose.position.z;
    t265_pose.vx = odom_msg->twist.twist.linear.x;
    t265_pose.vy = odom_msg->twist.twist.linear.y;
    t265_pose.vz = odom_msg->twist.twist.linear.z;

    double quatx = odom_msg->pose.pose.orientation.x;
    double quaty = odom_msg->pose.pose.orientation.y;
    double quatz = odom_msg->pose.pose.orientation.z;
    double quatw = odom_msg->pose.pose.orientation.w;
    tf::Quaternion q(quatx, quaty, quatz, quatw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    t265_pose.roll = roll;
    t265_pose.pitch = pitch;
    t265_pose.yaw = -yaw;
}

void car_mode_cb(const std_msgs::Int16::ConstPtr &msg)
{
    runMode = msg->data;
}

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

/**
 * 转动舵机
 * offset_angle > 0，往右边移动
 * offset_angle < 0，往左边移动
 */
void setAngleOffset(int offset_angle)
{
    std_msgs::Int16 offset;
    offset.data = offset_angle;
    servo_angle.publish(offset);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "contest2023nano_car_node");
    ros::NodeHandle nh;
    ros::Rate rate(10.0);

    ros::Subscriber state_sub = nh.subscribe<std_msgs::Bool>("/my_utils/state", 10, utils_state_cb);
    ros::Subscriber t265_sub = nh.subscribe<nav_msgs::Odometry>("/t265/odom/sample", 10, t265_pose_cb);
    ros::Subscriber car_mode_sub = nh.subscribe<std_msgs::Int16>("/my_utils/car_mode", 10, car_mode_cb);

    car_head_pub = nh.advertise<std_msgs::Float32>("/my_utils/car_head", 10);
    car_start_pub = nh.advertise<std_msgs::Int16>("/my_utils/car_start", 10);
    servo_angle = nh.advertise<std_msgs::Int16>("/my_utils/servo", 10);

    sleep_ms(5000);

    std_msgs::Float32 car_head_data;
    car_head_data.data = 0; // 回正朝向
    car_head_pub.publish(car_head_data);

    std_msgs::Int16 car_start_data;
    car_start_data.data = 0; // 停止电机
    car_start_pub.publish(car_start_data);

    // PID_incremental pid1(2, 0.65, 0.005);
    // float target = 0.0;
    // float actual = t265_pose.yaw;
    // float pid_increment = 0.0;

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();

        ROS_INFO("mode:%d, x:%f y:%f yaw:%f", runMode, t265_pose.x, t265_pose.y, t265_pose.yaw);

        switch (runMode)
        {
        case 0:
            break;
        case 1:
            // 前进
            if (t265_pose.x > 1.0)
            {
                car_start_data.data = 0;
                car_start_pub.publish(car_start_data);
                runMode = 2;
                break;
            }

            // pid_increment = pid1.pid_control(target, actual);
            // float actual = t265_pose.yaw;
            // car_head_data.data = t265_pose.yaw + pid_increment;

            // if (t265_pose.yaw < -0.005)
            // { // 向左偏移
            //     car_head_data.data = 2;
            // }
            // else if (t265_pose.yaw > 0.005)
            // { // 向右偏移
            //     car_head_data.data = -2;
            // }

            // if (t265_pose.yaw < -0.010)
            // {
            //     car_head_data.data = 4;
            // }
            // else if (t265_pose.yaw > 0.010)
            // {
            //     car_head_data.data = -4;
            // }

            // if (t265_pose.yaw < -0.015)
            // {
            //     car_head_data.data = 6;
            // }
            // else if (t265_pose.yaw > 0.015)
            // {
            //     car_head_data.data = -6;
            // }

            // if (t265_pose.yaw < -0.020)
            // {
            //     car_head_data.data = 8;
            // }
            // else if (t265_pose.yaw > 0.020)
            // {
            //     car_head_data.data = -8;
            // }

            // if (t265_pose.yaw < -0.025)
            // {
            //     car_head_data.data = 10;
            // }
            // else if (t265_pose.yaw > 0.025)
            // {
            //     car_head_data.data = -10;
            // }

            ROS_INFO("Head: %f", car_head_data.data);
            car_head_pub.publish(car_head_data);
            break;
        case 2:
            // 转动舵机
            setAngleOffset(-45);
            sleep_ms(1000);
            setAngleOffset(45);
            sleep_ms(1000);
            setAngleOffset(-45);
            sleep_ms(1000);
            setAngleOffset(45);
            sleep_ms(1000);
            runMode = 3;
            break;
        case 3:
            // 后退
            if (t265_pose.x < 0.2)
            {
                car_start_data.data = 0;
                car_start_pub.publish(car_start_data);
                runMode = 0;
                break;
            }

            car_start_data.data = -1;
            car_start_pub.publish(car_start_data);
            runMode = 4;
            break;
        case 4:
            ros::shutdown();
            break;
        }
    }

    return 0;
}