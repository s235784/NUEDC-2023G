#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include "std_msgs/String.h"
#include "my_utils/my_led.h"

/**
 * 已弃用
*/
/**
 * led_index -1指激光，1至8指第n个LED
 * rgb 颜色，r 红色，g 绿色，b 蓝色
 * duration 持续时间，-1常亮，0关闭，单位秒
*/
my_utils::my_led led;

bool haveNewLEDMsg = false;

void led_cb(const my_utils::my_led::ConstPtr& msg) {
    led = *msg;
    haveNewLEDMsg = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "led_node");
	ros::NodeHandle nh;
    ros::Rate rate(20);

    ros::Subscriber sub = nh.subscribe("/my_utils/led", 10, led_cb);

    serial::Serial sp;
    serial::Timeout to = serial::Timeout::simpleTimeout(3000); // 超时时间
    sp.setPort("/dev/ttyUSB0"); // 串口名
    sp.setBaudrate(9600); // 波特率
    sp.setTimeout(to);

    while (ros::ok()) {
        try {
            sp.open(); //打开串口
        } catch(serial::IOException& e) {
            ROS_ERROR("%s", e.what());
            ROS_ERROR("Unable to open port. Wait 5 seconds and try again.");
            ros::Duration(3).sleep();
            continue;
        }

        if(sp.isOpen()) {
            ROS_INFO("Port is opened.");
            break;
        }
    }

    // std::string result;
    while (ros::ok()) {
        if (haveNewLEDMsg) {
            std::string commandColor;
            if (led.rgb == 'r') {
                commandColor = "RED";
            } else if (led.rgb == 'g') {
                commandColor = "GRN";
            } else if (led.rgb == 'b') {
                commandColor = "BLU";
            }

            std::string commandIndex;
            if (led.led_index == -1) {
                commandIndex = "9"; // 传到单片机时，9是指激光
                commandColor = "RED";
            } else if (led.led_index > 8) {
                ROS_WARN("Invalid LED Index");
                haveNewLEDMsg = false;
                continue;
            } else {
                commandIndex = std::to_string(led.led_index); // 1~8是LED
            }

            
            if (led.duration < 0) { // 常亮
                std::string cmd = commandIndex + commandColor;
                sp.write(cmd + "\\n");
                ROS_INFO("%s", cmd.c_str());
            } else if (led.duration == 0) { // 关闭
                std::string cmd = commandIndex + "CLO";
                sp.write(cmd + "\\n");
                ROS_INFO("%s", cmd.c_str());
            } else { // 亮指定时间
                std::string cmdOn = commandIndex + commandColor;
                sp.write(cmdOn + "\\n");
                ROS_INFO("%s", cmdOn.c_str());

                ros::Duration(led.duration).sleep();

                std::string cmdOff = commandIndex + "CLO";
                sp.write(cmdOff + "\\n");
                ROS_INFO("%s", cmdOff.c_str());
            }
            haveNewLEDMsg = false;
        }

        // result = sp.read( sp.available() );
        // if (!result.empty())
        //     ROS_INFO("Receive: %s", result.c_str());  

        rate.sleep();
    }
    
    sp.close();
    return 0;
}