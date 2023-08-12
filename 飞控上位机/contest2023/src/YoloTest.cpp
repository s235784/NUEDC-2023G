#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <detection_msgs/BoundingBox.h>
#include <detection_msgs/BoundingBoxes.h>

bool yoloRunning = false;

bool foundedFire = false;
ros::Time foundedFireTime;
detection_msgs::BoundingBox fire_box;

void yolo_state_cb(const std_msgs::Bool::ConstPtr &msg)
{
    yoloRunning = msg -> data;
}

void yolo_fire_cb(const detection_msgs::BoundingBoxes::ConstPtr& msg)
{
    // 寻找多个结果中可信度最高的打开的LED灯
    detection_msgs::BoundingBox bestBox;
    bool first = true;
    for (auto box : msg -> bounding_boxes)
    {
        if (box.Class == "on") // 标签为on的目标
        {
            if (first)
            {
                bestBox = box;
                first = false;
            }
            else if (box.probability > bestBox.probability)
            {
                bestBox = box;
            }
        }
    }

    if (!first)
    {
        fire_box = bestBox;
        foundedFireTime = ros::Time::now();
    }
}

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
    ros::init(argc, argv, "contest2023_yolotest_node");
    ros::NodeHandle nh;
    ros::Rate rate(30.0);

    ros::Subscriber yolo_output_sub = nh.subscribe<detection_msgs::BoundingBoxes>("/yolov5/detections/camara/0", 10, yolo_fire_cb);
    ros::Subscriber yolo_state_sub = nh.subscribe<std_msgs::Bool>("/yolov5/state/camara/0", 10, yolo_state_cb);

    ROS_WARN("Waiting Yolo Node...");
    while (!yoloRunning)
    {
        ros::spinOnce();
        rate.sleep();
    }
    sleep_ms(1000);

    while (ros::ok())
    {
        int offset_x = 320 - (fire_box.xmax + fire_box.xmin) / 2; // 横向坐标
        int offset_y = 240 - (fire_box.ymax + fire_box.ymin) / 2; // 纵向坐标

        ROS_INFO("Fire offset x:%d y:%d", offset_x, offset_y);

        ros::spinOnce();
        rate.sleep();
    }
}