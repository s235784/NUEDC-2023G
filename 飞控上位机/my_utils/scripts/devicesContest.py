#!/usr/bin/env python
import os
import rospy
import serial
import Jetson.GPIO as GPIO
from std_msgs.msg import Bool, Int16
from nav_msgs.msg import Odometry
from adafruit_servokit import ServoKit
from my_utils.msg import my_led_unique, my_location, my_hook

Wireless_UART = "/dev/ttyUSB0"

led_enable = True
CHANNEL_LED_R = "GP40_SPI3_CS1_N" # BOARD:16
CHANNEL_LED_B = "GP39_SPI3_CS0_N" # BOARD:18
CHANNEL_LED_G = "GP49_SPI1_MOSI" # BOARD:19

buzzer_enable = True
CHANNEL_BUZZER = "GP37_SPI3_MISO" #BOARD:22

uart_enable = True
uart_ser = None

mode_pub = None

servo_enable = True
servo_kit = None
servo_unlock_angle = 120
servo_lock_angle = 60

pose_x = 0;
pose_y = 0;
pose_z = 0;
last_pose_time = None

fire_sent = False
fire_x = None
fire_y = None

runMode = 0


def buzzer_callback(buzzer_msg):
    GPIO.output(CHANNEL_BUZZER, GPIO.HIGH if buzzer_msg.data else GPIO.LOW)


def servo_process(servo_msg):
    global servo_kit
    if servo_msg.unlock:
        servo_kit.servo[0].angle = servo_unlock_angle
    else:
        servo_kit.servo[0].angle = servo_lock_angle


def led_callback(led_msg):
    rospy.loginfo("LED Receive: {}".format(led_msg))
    if not led_msg.enable: # 关
        GPIO.output(CHANNEL_LED_B, GPIO.HIGH)
        GPIO.output(CHANNEL_LED_G, GPIO.HIGH)
        GPIO.output(CHANNEL_LED_R, GPIO.HIGH)
    elif led_msg.color == 114: # r 红色
        GPIO.output(CHANNEL_LED_B, GPIO.HIGH)
        GPIO.output(CHANNEL_LED_G, GPIO.HIGH)
        GPIO.output(CHANNEL_LED_R, GPIO.LOW)
    elif led_msg.color == 103: # g 绿色
        GPIO.output(CHANNEL_LED_B, GPIO.HIGH)
        GPIO.output(CHANNEL_LED_G, GPIO.LOW)
        GPIO.output(CHANNEL_LED_R, GPIO.HIGH)
    elif led_msg.color == 98: # b 蓝色
        GPIO.output(CHANNEL_LED_B, GPIO.LOW)
        GPIO.output(CHANNEL_LED_G, GPIO.HIGH)
        GPIO.output(CHANNEL_LED_R, GPIO.HIGH)


def odometry_callback(odometry_msg):
    global pose_x, pose_y, pose_z
    pose_x = odometry_msg.pose.pose.position.x
    pose_y = odometry_msg.pose.pose.position.y
    pose_z = odometry_msg.pose.pose.position.z


def fire_callback(location_msg):
    global fire_x, fire_y
    fire_x = location_msg.pose_x
    fire_y = location_msg.pose_y


# 无人机与小车通过无线串口传输信息
# 无人机发送：
#   1.当前坐标 pose(x:1.00,y:1.00,z:1.80)\n
#   2.火源坐标 fire(x:1.50,y:1.50)\n
# 无人机接收：
#   1.行动模式 mode(1)\n
uart_buffer = "";
def wireless_uart_process():
    global uart_buffer, runMode
    if uart_ser.inWaiting():
        readB = uart_ser.read(1)
        read = readB.decode()
        if read != "\n": # 还没发送完毕
            uart_buffer += read
        else: # 接收完毕
            uart_buffer += read
            rospy.loginfo(uart_buffer)
            # 切换模式命令
            if str(uart_buffer).startswith("mode("): # 读取行动模式
                runMode = int(uart_buffer[5:len(uart_buffer)-2]) # 去掉最后的 ")\n"
                # 关机命令
                if runMode == 99:
                    rospy.logwarn("Shutdown!!!!!!")
                    rospy.sleep(1.0)
                    os.system("bash /home/jetson/shutdown.sh")
                else:
                    global mode_pub
                    mode_pub.publish(runMode)
                    rospy.loginfo("Run Mode: %d", runMode)
                
            uart_buffer = ""
        

    # 发送当前坐标
    global last_pose_time
    if rospy.get_rostime() - last_pose_time > rospy.Duration(1):
        pose = "pose[x({:.2f})y({:.2f})z({:.2f})]\n".format(pose_x, pose_y, pose_z)
        uart_ser.write(pose.encode())
        last_pose_time = rospy.get_rostime()
        # rospy.loginfo(pose)
        # rospy.loginfo("{} {} {}".format(pose_x, pose_y, pose_z))
    
    # 发送火源坐标
    global fire_sent
    if not fire_sent and not (fire_x is None) and not (fire_y is None):
        fire = "fire[x({:.2f})y({:.2f})]\n".format(fire_x, fire_y)
        uart_ser.write(fire.encode())
        fire_sent = True


def initDevices():
    rospy.init_node('devices_node', anonymous=True)
    rate = rospy.Rate(10)

    GPIO.setmode(GPIO.TEGRA_SOC)

    state_pub = rospy.Publisher('/my_utils/state', Bool, queue_size=10)

    global mode_pub
    mode_pub = rospy.Publisher('/my_utils/mode', Int16, queue_size=10)

    # 初始化无线通信
    if uart_enable:
        while not rospy.is_shutdown():
            try:
                global uart_ser
                uart_ser = serial.Serial(Wireless_UART, 9600, timeout=3)
                if uart_ser.is_open:
                    break
            except Exception:
                rospy.logerr("Unable to open Wireless UART port. Wait 5 seconds and try again.")
                rospy.sleep(5)
                continue
        
        rospy.Subscriber("/Odometry", Odometry, odometry_callback);
        rospy.Subscriber("/my_utils/fire_location", my_location, fire_callback);
        global last_pose_time
        last_pose_time = rospy.get_rostime()
        rospy.loginfo("Wireless UART Initialized")
    
    # 初始化拖钩器
    if servo_enable:
        global servo_kit
        servo_kit = ServoKit(channels=16)
        servo_kit.servo[0].angle = servo_lock_angle
        rospy.Subscriber("/my_utils/hook", my_hook, servo_process)

    # 初始化LED
    if led_enable:
        GPIO.setup(CHANNEL_LED_B, GPIO.OUT)
        GPIO.setup(CHANNEL_LED_G, GPIO.OUT)
        GPIO.setup(CHANNEL_LED_R, GPIO.OUT)
        GPIO.output(CHANNEL_LED_B, GPIO.HIGH)
        GPIO.output(CHANNEL_LED_G, GPIO.HIGH)
        GPIO.output(CHANNEL_LED_R, GPIO.HIGH)
        rospy.Subscriber("/my_utils/led_unique", my_led_unique, led_callback)
        rospy.loginfo("LED Initialized")

    # 初始化蜂鸣器
    if buzzer_enable:
        GPIO.setup(CHANNEL_BUZZER, GPIO.OUT)
        GPIO.output(CHANNEL_BUZZER, GPIO.LOW)
        rospy.Subscriber("/my_utils/buzzer", Bool, buzzer_callback)
        rospy.loginfo("Buzzer Initialized")

    rospy.loginfo("Devices Node Running……")
    while not rospy.is_shutdown():
        state_pub.publish(True)

        if uart_enable:
            wireless_uart_process()

        rate.sleep()
    

    GPIO.cleanup()
    if uart_enable:
        uart_ser.close()


if __name__ == '__main__':
    try:
        initDevices()
    except rospy.ROSInterruptException:
        pass
