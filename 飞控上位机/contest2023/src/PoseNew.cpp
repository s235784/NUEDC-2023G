#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/DebugValue.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <detection_msgs/BoundingBox.h>
#include <detection_msgs/BoundingBoxes.h>

#include "LidarPose.hpp"
#include "CtrlState.hpp"
#include "my_utils/my_location.h"
#include "my_utils/my_led_unique.h"
#include "my_utils/my_hook.h"
#include "contest2023/camera_offset.h"

#define PI 3.141592

typedef struct
{
    double x;
    double y;
    double z;
    double yaw;
} body_pose;

ros::Publisher camera_mode_pub;
ros::Publisher set_raw_pub;
ros::Publisher utils_led_pub;
ros::Publisher utils_buzzer_pub;
ros::Publisher utils_fire_pub;
ros::Publisher utils_hook_pub;

CtrlState ctrl_state;
mavros_msgs::State fcu_state;
geometry_msgs::PoseStamped fcu_pose; // 东北天
LidarPose lidar_pose;                // 前左上

tf2_ros::Buffer tfBuffer;

// contest2023::camera_offset fire_offset;

detection_msgs::BoundingBox fire_box;

// detection_msgs::BoundingBoxes yolo_d435_boxes;

int runMode = 0;
bool utilsRunning = false;
// bool cameraRunning = false;
bool yoloRunning = false;

bool foundedFire = false;
bool fireProcessed = false;
ros::Time foundedFireTime;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    fcu_state = *msg;
    // ROS_INFO("Mode: %s", fcu_state.mode.c_str());
}

void debug_state_cb(const mavros_msgs::DebugValue::ConstPtr &msg)
{
    ctrl_state.pos_mode = (int)msg->data[0];
    ctrl_state.alt_mode = (int)msg->data[1];
    ctrl_state.landed_state = (int)msg->data[2];
    ROS_INFO("Pos_mode: %d, Alt_mode: %d, LandedState: %d", ctrl_state.pos_mode, ctrl_state.alt_mode, ctrl_state.landed_state);
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    fcu_pose = *msg;
    tf2::Quaternion q(
        fcu_pose.pose.orientation.x,
        fcu_pose.pose.orientation.y,
        fcu_pose.pose.orientation.z,
        fcu_pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // ROS_INFO("FCU Local Position Pose x: %f y: %f z: %f yaw: %f", fcu_pose.pose.position.x, fcu_pose.pose.position.y, fcu_pose.pose.position.z, current_heading.data);
}

void local_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    nav_msgs::Odometry test_com = *msg;
    // ROS_INFO("FCU Global Position Pose  X :%.2f ,  Y %.2f ,  Z %.2f ", test_com.twist.twist.linear.x, test_com.twist.twist.linear.y, test_com.twist.twist.linear.z);
}

void lidar_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    lidar_pose.x = msg->pose.pose.position.x; // 单位m
    lidar_pose.y = msg->pose.pose.position.y;
    lidar_pose.z = msg->pose.pose.position.z;
    lidar_pose.vx = msg->twist.twist.linear.x;
    lidar_pose.vy = msg->twist.twist.linear.y;
    lidar_pose.vz = msg->twist.twist.linear.z;

    double quatx = msg->pose.pose.orientation.x;
    double quaty = msg->pose.pose.orientation.y;
    double quatz = msg->pose.pose.orientation.z;
    double quatw = msg->pose.pose.orientation.w;
    tf2::Quaternion q(quatx, quaty, quatz, quatw);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    lidar_pose.roll = roll;
    lidar_pose.pitch = pitch;
    lidar_pose.yaw = -yaw;
    // ROS_INFO("Livox Position Pose x: %.3f y: %.3f z: %.3f yaw: %.3f", lidar_pose.x, lidar_pose.y, lidar_pose.z, lidar_pose.yaw);
}

void mode_cb(const std_msgs::Int16::ConstPtr &msg)
{
    runMode = msg->data;
}

void utils_state_cb(const std_msgs::Bool::ConstPtr &msg)
{
    utilsRunning = msg->data;
}

// void camera_state_cb(const std_msgs::Bool::ConstPtr &msg)
// {
//     cameraRunning = msg->data;
// }

void yolo_state_cb(const std_msgs::Bool::ConstPtr &msg)
{
    yoloRunning = msg->data;
}

void yolo_fire_cb(const detection_msgs::BoundingBoxes::ConstPtr &msg)
{
    // 寻找多个结果中可信度最高的打开的LED灯
    detection_msgs::BoundingBox bestBox;
    bool first = true;
    for (auto box : msg->bounding_boxes)
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
        foundedFire = true;
    }
}

// void fire_offset_cb(const contest2023::camera_offset::ConstPtr &msg)
// {
//     fire_offset = *msg;
//     foundedFireTime = ros::Time::now();
// }

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

int set_offboard(ros::NodeHandle &nh)
{
    ros::ServiceClient set_mode_client_i = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.custom_mode = "offboard";
    if (set_mode_client_i.call(srv_setMode) && srv_setMode.response.mode_sent)
        ROS_WARN("offboard mode sent");
    else
    {
        ROS_ERROR("Failed setting mode");
        return -1;
    }
    return 0;
}

/**
 * 记录起飞时的位置与偏航
 * 记录的数据将在set_pose_local_offset_takeoff中用到
 */
void send_tf_snapshot_takeoff()
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped pose_snapshot = fcu_pose;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "my_map";
    transformStamped.child_frame_id = "snapshot_start";
    transformStamped.transform.translation.x = pose_snapshot.pose.position.x;
    transformStamped.transform.translation.y = pose_snapshot.pose.position.y;
    transformStamped.transform.translation.z = pose_snapshot.pose.position.z;

    tf2::Quaternion q;
    tf2::fromMsg(pose_snapshot.pose.orientation, q);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    tf2::Quaternion q_yaw;
    q_yaw.setRPY(0, 0, yaw); // strip pitch and roll info to avoid height changes.

    transformStamped.transform.rotation = tf2::toMsg(q_yaw);
    br.sendTransform(transformStamped);

    ROS_WARN("Takeoff Snapshot Sended");
}

/**
 * 移动到相较于起飞时的指定坐标（坐标系：起飞时的前左上）
 * xyz 相较于起飞时的相对坐标，单位 m
 * yaw 相较于起飞时的偏航，向左为正，单位 rad
 * z和yaw为0时则保持当前状态
 * 不允许不断调用，只能调用一次等待完成后再调用
 */
int set_pose_local_offset_takeoff(float x, float y, float z, float yaw)
{
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PointStamped initial_pt, transformed_pt;
    try
    {
        transformStamped = tfBuffer.lookupTransform("my_map", "snapshot_start", ros::Time(0));
        initial_pt.point.x = x;
        initial_pt.point.y = y;
        initial_pt.point.z = z;
        tf2::doTransform(initial_pt, transformed_pt, transformStamped);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return -1;
    }

    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    raw_target.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                           mavros_msgs::PositionTarget::IGNORE_VY |
                           mavros_msgs::PositionTarget::IGNORE_VZ |
                           mavros_msgs::PositionTarget::IGNORE_AFX |
                           mavros_msgs::PositionTarget::IGNORE_AFY |
                           mavros_msgs::PositionTarget::IGNORE_AFZ |
                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    if (fabs(z) < 1e-3)
        raw_target.type_mask |= mavros_msgs::PositionTarget::IGNORE_PZ;

    raw_target.position.x = transformed_pt.point.x;
    raw_target.position.y = transformed_pt.point.y;
    raw_target.position.z = transformed_pt.point.z;
    raw_target.velocity.y = 0.5; // XY移动速度

    tf2::Quaternion q = tf2::Quaternion(transformStamped.transform.rotation.x,
                                        transformStamped.transform.rotation.y,
                                        transformStamped.transform.rotation.z,
                                        transformStamped.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, takeoff_yaw;
    m.getRPY(roll, pitch, takeoff_yaw);
    if (fabs(yaw) < 1e-6)
    {
        raw_target.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
    }
    else
    {
        float newYaw;
        if (takeoff_yaw + yaw > PI)
        {
            newYaw = yaw + takeoff_yaw - (PI * 2);
        }
        else if (takeoff_yaw + yaw < -PI)
        {
            newYaw = PI - fabs(PI + (yaw + takeoff_yaw));
        }
        else
        {
            newYaw = takeoff_yaw + yaw;
        }
        raw_target.yaw = newYaw;
    }

    set_raw_pub.publish(raw_target);

    ros::spinOnce();
    ros::Duration(0.1).sleep();
    ROS_WARN("Local pose set to x:%f y:%f z:%f yaw:%f", raw_target.position.x, raw_target.position.y, raw_target.position.z, raw_target.yaw);
    return 0;
}

/**
 * 移动相较于当前位置/方向的距离/偏航（坐标系：当前的前左上）
 * xyz 前左上，单位 m
 * yaw 向左为正，单位 rad
 * x y z yaw为0时保持当前状态
 * 不允许不断调用，只能调用一次等待完成后再调用
 */
void set_pose_body(double x, double y, double z, double yaw)
{
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED;
    raw_target.type_mask = mavros_msgs::PositionTarget::IGNORE_VZ |
                           mavros_msgs::PositionTarget::IGNORE_AFX |
                           mavros_msgs::PositionTarget::IGNORE_AFY |
                           mavros_msgs::PositionTarget::IGNORE_AFZ |
                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    if (fabs(yaw) < 1e-6)
        raw_target.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW;
    else
        raw_target.yaw = yaw;

    if (fabs(z) < 1e-3)
        raw_target.type_mask |= mavros_msgs::PositionTarget::IGNORE_PZ;
    else
        raw_target.position.z = z;

    if (fabs(x) < 1e-3 && fabs(y) < 1e-3)
    {
        raw_target.type_mask |= mavros_msgs::PositionTarget::IGNORE_PX;
        raw_target.type_mask |= mavros_msgs::PositionTarget::IGNORE_PY;
    }
    else
    {
        raw_target.position.x = x;
        raw_target.position.y = y;
        raw_target.velocity.x = 0.5; // XY移动速度
    }

    set_raw_pub.publish(raw_target);

    ros::spinOnce();
    ros::Duration(0.1).sleep();
    ROS_WARN("Body pose set to x:%f y:%f z:%f yaw:%f", raw_target.position.x, raw_target.position.y, raw_target.position.z, raw_target.yaw);
}

/**
 * 设置移动速率
 * vx vy vz 前左上，单位 m/s
 * yaw_rate 向左为正，单位 rad/s
 * 调用频率需要大于1hz，调用结束后需要手动调用刹车
 */
void set_speed_body(double vx, double vy, double vz, double yaw_rate)
{
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED;
    raw_target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                           mavros_msgs::PositionTarget::IGNORE_PY |
                           mavros_msgs::PositionTarget::IGNORE_PZ |
                           mavros_msgs::PositionTarget::IGNORE_AFX |
                           mavros_msgs::PositionTarget::IGNORE_AFY |
                           mavros_msgs::PositionTarget::IGNORE_AFZ |
                           mavros_msgs::PositionTarget::IGNORE_YAW;
    if (fabs(yaw_rate) < 1e-6)
        raw_target.type_mask |= mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    if (fabs(vz) < 1e-3)
        raw_target.type_mask |= mavros_msgs::PositionTarget::IGNORE_VZ;

    raw_target.velocity.x = vx;
    raw_target.velocity.y = vy;
    raw_target.velocity.z = vz;
    raw_target.yaw_rate = yaw_rate;
    set_raw_pub.publish(raw_target);

    // ros::spinOnce();
    // ros::Duration(0.1).sleep();
    // ROS_WARN("Body speed set to vx:%f vy:%f vz:%f yaw_rate:%f", raw_target.velocity.x, raw_target.velocity.y, raw_target.velocity.z, raw_target.yaw_rate);
}

/**
 * 设置刹车
 */
void set_break()
{
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED;
    raw_target.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                           mavros_msgs::PositionTarget::IGNORE_VY |
                           mavros_msgs::PositionTarget::IGNORE_VZ |
                           mavros_msgs::PositionTarget::IGNORE_AFX |
                           mavros_msgs::PositionTarget::IGNORE_AFY |
                           mavros_msgs::PositionTarget::IGNORE_AFZ |
                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    raw_target.position.x = 0;
    raw_target.position.y = 0;
    raw_target.position.z = 0;
    raw_target.yaw = 0;
    set_raw_pub.publish(raw_target);

    // ros::spinOnce();
    // ros::Duration(0.1).sleep();
    ROS_WARN("Body speed set break");
}

/**
 * 绕圈旋转
 * r 半径
 * vy 旋转速度，大于0时顺时针旋转
 * vxDelta x轴速度的微调量，用于PID控制
 * rateDelta 旋转角速度的微调量，用于PID控制
 * 调用频率需要大于1hz，调用结束后需要手动调用刹车
 */
void round_circle(float r, float vy, float vxDelta, float rateDelta)
{
    float rate = vy / r;
    set_speed_body(vxDelta, vy, 0, -(rate + rateDelta));
}

/**
 * 解锁
 */
int arm_drone(ros::NodeHandle &nh)
{
    ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv_arm_i;
    srv_arm_i.request.value = true;
    if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success)
        ROS_WARN("ARM sent %d", srv_arm_i.response.success);
    else
    {
        ROS_ERROR("Failed arming");
        return -1;
    }
    return 0;
}

/**
 * 起飞， height 单位米
 */
int takeoff(ros::NodeHandle &nh, double height)
{
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = height;
    if (takeoff_cl.call(srv_takeoff))
    {
        ROS_WARN("takeoff sent %d", srv_takeoff.response.success);
    }
    else
    {
        ROS_ERROR("Takeoff failed");
        return -1;
    }
    return 0;
}

/**
 * 降落
 */
int land(ros::NodeHandle &nh)
{
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    if (land_client.call(srv_land) && srv_land.response.success)
        ROS_WARN("Land sent %d", srv_land.response.success);
    else
    {
        ROS_ERROR("Land failed");
        ros::shutdown();
        return -1;
    }
    return 0;
}

/**
 * 阻塞解锁飞机
 */
void arm_drone_block(ros::NodeHandle &nh, ros::Rate &rate)
{
    arm_drone(nh);
    while (!fcu_state.armed)
    {
        ros::spinOnce();
        rate.sleep();
    }
    sleep_ms(1000);
}

/**
 * 阻塞起飞
 */
void takeoff_block(ros::NodeHandle &nh, ros::Rate &rate, double height)
{
    ROS_WARN("Takeoff, height: %f", height);
    takeoff(nh, height);
    sleep_ms(1000);
    while (ctrl_state.landed_state != CtrlState::Landed_State_In_Air)
    { // 正在起飞
        ros::spinOnce();
        rate.sleep();
    }
    ROS_WARN("Takeoff Done");
}

/**
 * 阻塞降落
 */
void land_block(ros::NodeHandle &nh, ros::Rate &rate)
{
    ROS_WARN("Land");
    land(nh);
    sleep_ms(1000);
    while (ctrl_state.landed_state == CtrlState::Landed_State_Landing)
    {
        ros::spinOnce();
        rate.sleep();
    }
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

/**
 * 是否已经过去了second秒
 * start_time 开始时间
 * second 时间间隔
 */
bool isPassedSecond(ros::Time start_time, double second)
{
    return ros::Time::now() - start_time > ros::Duration(second);
}

/**
 * 两个浮点数是否相近
 */
bool isAround(double a, double b)
{
    return fabs(a - b) < 0.1;
}

/**
 * 通过速度控制到达指定坐标
 * target_X 前左上坐标系下的X轴坐标
 * start_Y 开始移动时的Y轴坐标
 */
bool controlXSpeed_YLimit(double target_X, double start_X, double start_Y)
{
    double speed_x = 0;
    if (target_X - start_X > 0)
    { // 向前，速度为正
        speed_x = 0.3;
        if (lidar_pose.x > target_X - 0.01)
        {
            set_break();
            sleep_ms(1000);
            ROS_WARN("Break");
            return true;
        }
    }
    else if (target_X - start_X < 0)
    { // 向后，速度为负
        speed_x = -0.3;
        if (lidar_pose.x < target_X + 0.01)
        {
            set_break();
            sleep_ms(1000);
            ROS_WARN("Break");
            return true;
        }
    }

    if (lidar_pose.y - start_Y > 0.05)
    { // 向左偏
        set_speed_body(speed_x, -0.1, 0, 0);
    }
    else if (lidar_pose.y - start_Y < -0.05)
    { // 向右偏
        set_speed_body(speed_x, 0.1, 0, 0);
    }
    else
    {
        set_speed_body(speed_x, 0, 0, 0);
    }
    return false;
}

bool controlYSpeed_XLimit(double target_Y, double start_X, double start_Y)
{
    double speed_y = 0;
    if (target_Y - start_Y > 0)
    { // 向左，速度为正
        speed_y = 0.3;
        if (lidar_pose.y > target_Y - 0.01)
        {
            set_break();
            sleep_ms(1000);
            return true;
        }
    }
    else if (target_Y - start_Y < 0)
    { // 向右，速度为负
        speed_y = -0.3;
        if (lidar_pose.y < target_Y + 0.01)
        {
            set_break();
            sleep_ms(1000);
            return true;
        }
    }

    if (lidar_pose.x - start_X > 0.05)
    { // 向前偏
        set_speed_body(-0.1, speed_y, 0, 0);
    }
    else if (lidar_pose.x - start_X < -0.05)
    { // 向后偏
        set_speed_body(0.1, speed_y, 0, 0);
    }
    else
    {
        set_speed_body(0, speed_y, 0, 0);
    }
    return false;
}

/**
 * 遍历地图使用的路径
 * 需要不断调用
 * 返回是否完成整个遍历过程
 */
bool raverseMapBySpeed(int &mode, double &start_x, double &start_y)
{
    switch (mode)
    {
    case 1: // 前进至（3.2，0）处
        if (controlXSpeed_YLimit(3.2, start_x, start_y))
        {
            ROS_WARN("Arrive (3.2,0)");
            start_x = lidar_pose.x;
            start_y = lidar_pose.y;
            mode++;
        }
        break;
    case 2: // 前进至（3.2，-0.8）处
        if (controlYSpeed_XLimit(-0.8, start_x, start_y))
        {
            ROS_WARN("Arrive (3.2,-0.8)");
            start_x = lidar_pose.x;
            start_y = lidar_pose.y;
            mode++;
        }
        break;
    case 3: // 前进至（0.8，-0.8）处
        if (controlXSpeed_YLimit(0.8, start_x, start_y))
        {
            ROS_WARN("Arrive (0.8,-0.8)");
            start_x = lidar_pose.x;
            start_y = lidar_pose.y;
            mode++;
        }
        break;
    case 4: // 前进至（0.8，-1.6）处
        if (controlYSpeed_XLimit(-1.6, start_x, start_y))
        {
            ROS_WARN("Arrive (0.8,-1.6)");
            start_x = lidar_pose.x;
            start_y = lidar_pose.y;
            mode++;
        }
        break;
    case 5: // 前进至（3.2，-1.6）处
        if (controlXSpeed_YLimit(3.2, start_x, start_y))
        {
            ROS_WARN("Arrive (3.2,-1.6)");
            start_x = lidar_pose.x;
            start_y = lidar_pose.y;
            mode++;
        }
        break;
    case 6: // 前进至（3.2，-2.4）处
        if (controlYSpeed_XLimit(-2.4, start_x, start_y))
        {
            ROS_WARN("Arrive (3.2,-2.4)");
            start_x = lidar_pose.x;
            start_y = lidar_pose.y;
            mode++;
        }
        break;
    case 7: // 前进至（0.8，-2.4）处
        if (controlXSpeed_YLimit(0.8, start_x, start_y))
        {
            ROS_WARN("Arrive (0.8,-2.4)");
            start_x = lidar_pose.x;
            start_y = lidar_pose.y;
            mode++;
        }
        break;
    case 8: // 前进至（0.8，-3.2）处
        if (controlYSpeed_XLimit(-3.2, start_x, start_y))
        {
            ROS_WARN("Arrive (0.8,-3.2)");
            start_x = lidar_pose.x;
            start_y = lidar_pose.y;
            mode++;
        }
        break;
    case 9: // 前进至（3.2，-3.2）处
        if (controlXSpeed_YLimit(3.2, start_x, start_y))
        {
            ROS_WARN("Arrive (3.2,-3.2)");
            start_x = lidar_pose.x;
            start_y = lidar_pose.y;
            mode++;
        }
        break;
    case 10: // 前进至（3.2，-4.0）处
        if (controlYSpeed_XLimit(-4.0, start_x, start_y))
        {
            ROS_WARN("Arrive (3.2,-4.0)");
            start_x = lidar_pose.x;
            start_y = lidar_pose.y;
            mode++;
        }
        break;
    case 11: // 前进至（0，-4.0）处
        if (controlXSpeed_YLimit(0, start_x, start_y))
        {
            ROS_WARN("Arrive (0,-4.0)");
            start_x = lidar_pose.x;
            start_y = lidar_pose.y;
            mode++;
        }
        break;
    case 12: // 前进至（0，0）处
        if (controlYSpeed_XLimit(0, start_x, start_y))
        {
            ROS_WARN("Arrive (0,0)");
            start_x = lidar_pose.x;
            start_y = lidar_pose.y;
            return true;
        }
        break;
    }
    return false;
}

/**
 * 速度控制遍历地图
 */ 
void raverseMapBySpeed_Block(ros::Rate &rate)
{
    int mode = 1;
    double start_x = lidar_pose.x;
    double start_y = lidar_pose.y;
    while (true)
    {
        ros::spinOnce();

        if (raverseMapBySpeed(mode, start_x, start_y))
        {
            return;
        }

        ROS_INFO("mode %d, has reached x: %.3f y: %.3f", mode, lidar_pose.x, lidar_pose.y);
        rate.sleep();
    }
}

/**
 * 打开脱钩器
 */
void unlook_hook()
{
    my_utils::my_hook hook;
    hook.unlock = true;
    utils_hook_pub.publish(hook);
}

/**
 * 发现火源时的处理
*/
bool fireProcess_Block(ros::Rate &rate)
{
    // 画面中心在火源上方，y<0
    // 画面中心在火源左边，x<0
    double orinalX = lidar_pose.x;
    double orinalY = lidar_pose.y;

    // 靠近火源
    while (true)
    {
        ros::spinOnce();

        if (isPassedSecond(foundedFireTime, 1.5))
        { // 超过1.5s没有更新坐标
            // 丢失目标，回到巡航位置
            ROS_ERROR("Loss target!");
            set_pose_local_offset_takeoff(orinalX, orinalY, 0, 0);
            sleep_ms(1000);
            while (ctrl_state.pos_mode == CtrlState::Position_ControlMode_RouteLine)
            {
                ros::spinOnce();
                rate.sleep();
            }
            sleep_ms(1000);

            foundedFire = false;

            return false;
        }

        int offset_x = 320 - (fire_box.xmax + fire_box.xmin) / 2; // 横向坐标
        int offset_y = 240 - (fire_box.ymax + fire_box.ymin) / 2; // 纵向坐标
        if (offset_y < 15 && offset_y > -15 &&
            offset_x < 15 && offset_x > -15)
        { // 已在正上方
            set_break();
            break;
        }

        double speed_x = 0;
        double speed_y = 0;
        if (offset_x > 15)
        { // 物体中心在画面中心左边，向左移动
            speed_y = 0.1;
        }
        else if (offset_x < -15)
        { // 向右移动
            speed_y = -0.1;
        }

        if (offset_y > 15)
        { // 物体中心在画面中心上面，向上移动
            speed_x = 0.1;
        }
        else if (offset_y < -15)
        { // 向后移动
            speed_x = -0.1;
        }

        set_speed_body(speed_x, speed_y, 0, 0);

        ROS_INFO("Fire offset x:%d y:%d, set vx:%f vy:%f", offset_x, offset_y, speed_x, speed_y);

        ros::spinOnce();
        rate.sleep();
    }

    // 已靠近火源
    // 刹车
    set_break();

    ROS_WARN("Get above the fire");

    // 亮灯
    setLED(LED_Color::LED_Color_Red);
    sleep_ms(1000);
    setLED(LED_Color::LED_Color_Off);
    sleep_ms(1000);
    setLED(LED_Color::LED_Color_Red);
    sleep_ms(1000);
    setLED(LED_Color::LED_Color_Off);
    ROS_WARN("End of flashing lights");

    // 降落至1m左右
    double heightChange = lidar_pose.z - 1.1;
    set_pose_body(0, 0, -heightChange, 0);
    sleep_ms(1000);
    while (ctrl_state.pos_mode == CtrlState::Position_ControlMode_RouteLine)
    {
        ros::spinOnce();
        rate.sleep();
    }
    // 等待3s
    sleep_ms(3000);

    // 抛洒灭火包
    unlook_hook();
    sleep_ms(1000);
    ROS_WARN("Fire extinguisher pack thrown");

    // 将坐标发送给小车
    my_utils::my_location fire_location;
    fire_location.pose_x = lidar_pose.x;
    fire_location.pose_y = lidar_pose.y;
    utils_fire_pub.publish(fire_location);
    ROS_WARN("Coordinates sent");

    // 回到原本高度
    set_pose_body(0, 0, heightChange, 0);
    sleep_ms(1000);
    while (ctrl_state.pos_mode == CtrlState::Position_ControlMode_RouteLine)
    {
        ros::spinOnce();
        rate.sleep();
    }
    sleep_ms(1000);

    // 回到巡航位置
    set_pose_local_offset_takeoff(orinalX, orinalY, 0, 0);
    sleep_ms(1000);
    while (ctrl_state.pos_mode == CtrlState::Position_ControlMode_RouteLine)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_WARN("Reach the coordinates before the fire was found");
    sleep_ms(1000);

    return true;
}

/**
 * 遍历地图，并寻找火源
*/
void raverseMapAndFindFire_Block(ros::Rate &rate)
{
    int mode = 1;
    double start_x = lidar_pose.x;
    double start_y = lidar_pose.y;
    while (true)
    {
        ros::spinOnce();

        if (mode != 1 && foundedFire &&
            !fireProcessed && !isPassedSecond(foundedFireTime, 0.5)) // 避免前进的时候去找灯
        {
            // 找到火源，下面的代码需要阻塞运行
            if (fireProcess_Block(rate))
            { // 完成投掷
                // 继续巡航
                fireProcessed = true;
            }
        }

        // 没有找到火源，继续巡航
        if (raverseMapBySpeed(mode, start_x, start_y))
        {
            return;
        }

        ROS_INFO("mode %d, has reached x: %.3f y: %.3f", mode, lidar_pose.x, lidar_pose.y);
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "contest2023_pose_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber debug_state_sub = nh.subscribe<mavros_msgs::DebugValue>("/mavros/debug_value/debug_vector", 10, debug_state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber lidar_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 10, lidar_cb);
    ros::Subscriber utils_mode_sub = nh.subscribe<std_msgs::Int16>("/my_utils/mode", 10, mode_cb);
    ros::Subscriber utils_state_sub = nh.subscribe<std_msgs::Bool>("/my_utils/state", 10, utils_state_cb);
    // ros::Subscriber yolo_d435_sub = nh.subscribe<detection_msgs::BoundingBoxes>("/yolov5/detections/d435", 10, yolo_d435_cb);
    // ros::Subscriber camera_state_sub = nh.subscribe<std_msgs::Bool>("/contest2023/camera/state", 10, camera_state_cb);
    // ros::Subscriber camera_fire_sub = nh.subscribe<contest2023::camera_offset>("/contest2023/camera/fire", 10, fire_offset_cb);

    ros::Subscriber yolo_output_sub = nh.subscribe<detection_msgs::BoundingBoxes>("/yolov5/detections/camara/0", 10, yolo_fire_cb);
    ros::Subscriber yolo_state_sub = nh.subscribe<std_msgs::Bool>("/yolov5/state/camara/0", 10, yolo_state_cb);

    // camera_mode_pub = nh.advertise<std_msgs::Int16>("/contest2023/camera/mode", 10);
    set_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    utils_led_pub = nh.advertise<my_utils::my_led_unique>("/my_utils/led_unique", 10);
    utils_fire_pub = nh.advertise<my_utils::my_location>("/my_utils/fire_location", 10);
    utils_buzzer_pub = nh.advertise<std_msgs::Bool>("/my_utils/buzzer", 10);
    utils_hook_pub = nh.advertise<my_utils::my_hook>("/my_utils/hook", 10);

    tf2_ros::TransformListener tfListener(tfBuffer);

    ROS_WARN("Waiting Utils Node...");
    while (!utilsRunning)
    {
        ros::spinOnce();
        rate.sleep();
    }
    sleep_ms(1000);

    ROS_WARN("Waiting Yolo Node...");
    while (!yoloRunning)
    {
        ros::spinOnce();
        rate.sleep();
    }
    sleep_ms(1000);

    ROS_WARN("Waiting FCU Connection...");
    while (ros::ok() && !fcu_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    ROS_WARN("Waiting OFFBOARD...");
    set_offboard(nh);
    while (fcu_state.mode != "OFFBOARD")
    {
        ros::spinOnce();
        rate.sleep();
    }
    sleep_ms(1000);

    setLED(LED_Color::LED_Color_Green);

    double takeoffHeight = 1.6;

    while (ros::ok())
    {
        ros::spinOnce();
        if (runMode == 1)
        {
            ROS_WARN("Mode 1");
            setLED(LED_Color::LED_Color_Off);
            setBuzzer(true);
            sleep_ms(3000);
            setBuzzer(false);

            arm_drone_block(nh, rate);

            ros::spinOnce();
            rate.sleep();
            send_tf_snapshot_takeoff();

            takeoff_block(nh, rate, takeoffHeight);
            sleep_ms(1000);

            // 遍历地图
            raverseMapBySpeed_Block(rate);

            land_block(nh, rate);

            runMode = 3;
        }
        else if (runMode == 2)
        {
            ROS_WARN("Mode 2");
            setBuzzer(true);
            sleep_ms(3000);
            setLED(LED_Color::LED_Color_Off);
            setBuzzer(false);

            arm_drone_block(nh, rate);

            ros::spinOnce();
            rate.sleep();
            send_tf_snapshot_takeoff();

            takeoff_block(nh, rate, takeoffHeight);
            sleep_ms(1000);

            // 巡航+寻找火源
            raverseMapAndFindFire_Block(rate);

            land_block(nh, rate);

            runMode = 3;
        }
        else if (runMode == 3)
        {
            sleep_ms(1000);
            continue;
        }
    }

    // 这是mode1的废弃代码
    // 转向控制
    // body_pose pose[] = {
    //     {1.2, 0, 0, 0},
    //     {0, 0, 0, -PI / 2},
    //     {3.8, 0, 0, 0},
    //     {0, 0, 0, PI / 2},
    //     {-0.6, 0, 0, 0},
    //     {2.1, 0, 0, 0},
    //     {0, 0, 0, PI / 2},
    //     {3.8, 0, 0, 0},
    //     {0, 0, 0, PI / 2},
    //     {2.7, 0, 0, 0},
    // };
    // for (int i = 0; i < sizeof(pose) / sizeof(body_pose); ++i)
    // {
    //     set_pose_body(pose[i].x, pose[i].y, pose[i].z, pose[i].yaw);
    //     sleep_ms(1000);
    //     while (ctrl_state.pos_mode == CtrlState::Position_ControlMode_RouteLine)
    //     {
    //         ros::spinOnce();
    //         rate.sleep();
    //     }
    //     ROS_WARN("Arrive Pose x:%.2f y:%.2f z:%.2f yaw:%f", pose[i].x, pose[i].y, pose[i].z, pose[i].yaw);
    //     sleep_ms(1000);
    // }

    // 不带转向控制
    // body_pose pose[] = {
    //     {1.2, 0, 0, 0},
    //     {0  , -3.8, 0, 0},
    //     {-0.6, 0, 0, 0},
    //     {2.1, 0, 0, 0},
    //     {0  , 3.8, 0, 0},
    //     {-2.7, 0, 0, 0},
    // };
    // for (int i = 0; i < sizeof(pose)/sizeof(body_pose); ++i)
    // {
    //     set_pose_body(pose[i].x, pose[i].y, pose[i].z, pose[i].yaw);
    //     sleep_ms(1000);
    //     while (ctrl_state.pos_mode == CtrlState::Position_ControlMode_RouteLine)
    //     {
    //         ros::spinOnce();
    //         rate.sleep();
    //     }
    //     ROS_WARN("Arrive Pose x:%.2f y:%.2f z:%.2f yaw:%f", pose[i].x, pose[i].y, pose[i].z, pose[i].yaw);
    //     sleep_ms(1000);
    // }

    // 速度控制
    // ROS_WARN("Speed Control");
    // double start_y = lidar_pose.y;
    // while (true)
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    //     if (lidar_pose.x > 0.98 || lidar_pose.y > 0.7)
    //     {
    //         ROS_WARN("RESIVED:::::");
    //         set_break();
    //         break;
    //     }
    //     if (lidar_pose.y - start_y > 0.1)
    //     { // 向左偏
    //         set_speed_body(0.2, -0.1, 0, 0);
    //     } else if (lidar_pose.y - start_y < -0.1)
    //     { // 向右偏
    //         set_speed_body(0.2, 0.1, 0, 0);
    //     } else
    //     {
    //         set_speed_body(0.2, 0, 0, 0);
    //     }

    //     ROS_INFO("has reached x: %.3f y: %.3f", lidar_pose.x, lidar_pose.y);
    // }

    // 绕圈
    // ROS_WARN("Round Circle");
    // float start_yaw = lidar_pose.yaw;
    // ros::Time start_time = ros::Time::now();
    // while (true)
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    //     if (isPassedSecond(start_time, 2) && isAround(start_yaw, lidar_pose.yaw))
    //     {
    //         ROS_WARN("RESIVED:::::");
    //         set_break();
    //         break;
    //     }
    //     round_circle(0.5, 0.2, 0, 0);
    //     ROS_INFO("has reached x: %.3f y: %.3f yaw:%.3f", lidar_pose.x, lidar_pose.y, lidar_pose.yaw);
    // }
    // sleep_ms(1000);

    // 移动到指定位置 Local
    // body_pose pose[] = {
    // 	{0.5 ,0   ,0   ,  0},
    // 	{0.5 ,0   ,0   ,  PI / 2},
    //     {0.5 ,0.5 ,0   ,  0}
    // };
    // for (int i = 0; i < 3; ++i) {
    //     set_pose_local_offset_takeoff(pose[i].x, pose[i].y, pose[i].z, pose[i].yaw);
    //     sleep_ms(1000);
    //     while (ctrl_state.pos_mode == CtrlState::Position_ControlMode_RouteLine)
    //     {
    //         ros::spinOnce();
    //         rate.sleep();
    //     }
    //     ROS_WARN("Arrive Pose x:%.2f y:%.2f z:%.2f yaw:%f", pose[i].x, pose[i].y, pose[i].z, pose[i].yaw);
    //     sleep_ms(1000);
    // }

    // 移动到指定位置 Body
    // body_pose pose[] = {
    // 	{0.5 ,0   ,0   ,  0},
    // 	{0   ,0   ,0   ,  PI / 2},
    //     {0.5 ,0   ,0   ,  0}
    // };
    // for (int i = 0; i < 3; ++i)
    // {
    //     set_pose_body(pose[i].x, pose[i].y, pose[i].z, pose[i].yaw);
    //     sleep_ms(1000);
    //     while (ctrl_state.pos_mode == CtrlState::Position_ControlMode_RouteLine)
    //     {
    //         ros::spinOnce();
    //         rate.sleep();
    //     }
    //     ROS_WARN("Arrive Pose x:%.2f y:%.2f z:%.2f yaw:%f", pose[i].x, pose[i].y, pose[i].z, pose[i].yaw);
    //     sleep_ms(1000);
    // }

    ros::shutdown();
    return 0;
}
