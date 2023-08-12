#pragma once

class CtrlState
{
public:
    int alt_mode; // Z轴控制状态
    int pos_mode; // XY轴控制状态
    enum Position_ControlMode
    {
        Position_ControlMode_Null = 255, // 控制锁定

        Position_ControlMode_ManualCircle = 15, // 2D绕圈模式
        Position_ControlMode_Position = 12,     // 位置锁定模式
        Position_ControlMode_Velocity = 11,     // 速度控制模式
        Position_ControlMode_Locking = 10,      // 刹车后锁位置
        Position_ControlMode_OffBoard = 8,

        Position_ControlMode_Takeoff = 20,   // 起飞模式
        Position_ControlMode_RouteLine = 22, // 巡线模式

        Position_ControlMode_RouteLine3D = 52, // 巡线模式
        Position_ControlMode_Circle3D = 55,
    };

    int landed_state; // 飞行状态
    enum Landed_State
    {
        Landed_State_Undefined = 0, // 状态未知
        Landed_State_On_Ground = 1, // 在地上
        Landed_State_In_Air = 2,    // 在空中
        Landed_State_Takeoff = 3,   // 正在起飞过程中
        Landed_State_Landing = 4,   // 正在降落过程中
    };
};