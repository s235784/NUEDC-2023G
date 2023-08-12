#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

#include "contest2023/camera_offset.h"
#include "contest2023/camera_yaw_fix.h"

cv::Mat camera;

int runMode = 2;

void mode_cb(const std_msgs::Int16::ConstPtr &msg)
{
    runMode = msg->data;
    ROS_INFO("Mode Changed: %d", runMode);
}

cv::Point findLargestRedBlob(const cv::Mat &image)
{
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // 红色的HSV范围
    // cv::Scalar lower_red(129, 74, 174);
    // cv::Scalar upper_red(180, 233, 255);
    cv::Scalar lower_red(0, 100, 100);
    cv::Scalar upper_red(10, 255, 255);

    cv::Mat mask;
    cv::inRange(hsv, lower_red, upper_red, mask);

    // 使用中值滤波平滑图像，以减少噪声
    cv::medianBlur(mask, mask, 5);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int max_area = 0;
    cv::Point max_area_center;

    for (const auto &contour : contours)
    {
        double area = cv::contourArea(contour);
        if (area > max_area)
        {
            max_area = static_cast<int>(area);
            cv::Moments mu = cv::moments(contour);
            max_area_center = cv::Point(static_cast<int>(mu.m10 / mu.m00), static_cast<int>(mu.m01 / mu.m00));
        }
    }

    return max_area_center;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "contest_camera_node");
    ros::NodeHandle nh;
    ros::Rate rate(30.0);

    cv::Mat frame;
    cv::VideoCapture capture("/dev/usbcam0");
    while (ros::ok())
    {
        if (!capture.isOpened())
        {
            ROS_ERROR("Can not open Camara, waiting 3 seconds……");
            ros::Duration(3).sleep();
        }
        else
            break;
    }

    // ros::Subscriber mode_sub = nh.subscribe<std_msgs::Int16>("/contest2023/camera/mode", 10, mode_cb);
    // ros::Publisher camera_yaw_pub = nh.advertise<contest2023::camera_yaw_fix>("/contest2023/camera/yaw_fix", 10);
    ros::Publisher camera_state_pub = nh.advertise<std_msgs::Bool>("/contest2023/camera/state", 10);
    ros::Publisher camera_fire_pub = nh.advertise<contest2023::camera_offset>("/contest2023/camera/fire", 10);

    while (ros::ok())
    {
        capture >> camera;

        std_msgs::Bool state;
        state.data = true;
        camera_state_pub.publish(state);

        // 效果不好
        if (runMode == 1)
        { // 检测起飞点矩形
            // 高斯模糊
            // cv::Mat blurred;
            // cv::GaussianBlur(camera, blurred, cv::Size(5, 5), 0);

            // // 将图像转换为灰度图像
            // cv::Mat gray;
            // cv::cvtColor(blurred, gray, cv::COLOR_BGR2GRAY);

            // // 使用阈值分割或其他图像处理方法（根据实际情况）来得到二值化图像
            // cv::Mat binary;
            // cv::threshold(gray, binary, 150, 255, cv::THRESH_BINARY_INV);

            // // 查找轮廓
            // std::vector<std::vector<cv::Point>> contours;
            // cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // // 找到面积最大的轮廓
            // size_t maxContourIdx = 0;
            // double maxContourArea = 0.0;
            // for (size_t i = 0; i < contours.size(); ++i)
            // {
            //     double area = cv::contourArea(contours[i]);
            //     if (area > maxContourArea)
            //     {
            //         maxContourArea = area;
            //         maxContourIdx = i;
            //     }
            // }

            // // 找到面积最大的轮廓的最小外接矩形
            // cv::RotatedRect minRect;
            // if (!contours.empty())
            // {
            //     minRect = cv::minAreaRect(contours[maxContourIdx]);

            //     // 计算拟合矩形相对于水平的旋转角度
            //     float angle = minRect.angle;
            //     if (angle < -45.0)
            //     {
            //         angle += 90.0;
            //     }

            //     // 绘制最小外接矩形
            //     cv::Point2f rectPoints[4];
            //     minRect.points(rectPoints);
            //     for (int j = 0; j < 4; ++j)
            //     {
            //         cv::line(camera, rectPoints[j], rectPoints[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
            //     }

            //     // 在图像中显示旋转角度
            //     std::stringstream ss;
            //     ss << "Angle: " << angle;
            //     cv::putText(camera, ss.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);

            //     contest2023::camera_yaw_fix yaw_fix;
            //     yaw_fix.yaw_offset = angle;
            //     camera_yaw_pub.publish(yaw_fix);
            // }

            // cv::imshow("thresh", binary);
            // cv::imshow("Camera", camera);
            // cv::waitKey(1);
        }
        else if (runMode == 2)
        { // 检测红色物体
            // 使用高斯滤波平滑图像，以减少噪声
            cv::Mat frame;
            cv::GaussianBlur(camera, frame, cv::Size(5, 5), 0);

            // 查找最大红色色块
            cv::Point red_blob_center = findLargestRedBlob(frame);

            // 获取画面中心坐标
            int center_frame_x = frame.cols / 2;
            int center_frame_y = frame.rows / 2;

            // 计算x和y轴偏移
            int offset_x = center_frame_x - red_blob_center.x;
            int offset_y = center_frame_y - red_blob_center.y;

            // 在图像上绘制最大色块中心点
            cv::circle(frame, red_blob_center, 5, cv::Scalar(0, 255, 0), -1);

            // 在图像上绘制画面中心点
            cv::circle(frame, cv::Point(center_frame_x, center_frame_y), 5, cv::Scalar(0, 0, 255), -1);

            // 显示偏移量信息
            cv::putText(frame, "X Offset: " + std::to_string(offset_x) + ", Y Offset: " + std::to_string(offset_y),
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);

            // 发送位移信息
            // 画面中心在火源上方，y<0
            // 画面中心在火源左边，x<0
            if (red_blob_center.x != 0 && red_blob_center.y != 0)
            {
                contest2023::camera_offset offset;
                offset.offset_x = offset_x;
                offset.offset_y = offset_y;
                camera_fire_pub.publish(offset);
            }
            
            
            cv::imshow("Frame", frame);
            cv::waitKey(1);
        }
    }

    cv::destroyAllWindows();
    capture.release();
}