#pragma once

// 以下反向均以T265在机尾处
class t265Pose {
    public:
        double x = 0; // 机头为X轴正方向
        double y = 0; // 机左为y轴正方向
        double z = 0; // 上方为z轴正方向
        double vx = 0;
        double vy = 0;
        double vz = 0;
        double roll = 0;
        double pitch = 0;
        double yaw = 0; // 顺时针旋转为正，顺时针转得越多值越大，初始方向为0，范围为0~3.14 （跳变） -3.14~0（归位）

        /**
         * 获得归一化后的yaw
         * 一定大于0，初始方向为0
         * 顺时针方向旋转值为0~6.48
        */
        double getYawSTD() {
            double result = 0;
            if (yaw < 0) {
                return 2 * 3.141592 + yaw;
            } else {
                return yaw;
            }
        }
};