#!/usr/bin/env python
import os
import rospy
import serial
import Jetson.GPIO as GPIO
from std_msgs.msg import Bool, Float32, Int16
from nav_msgs.msg import Odometry
from adafruit_servokit import ServoKit

# 舵机
servo_enable = True
servo_kit = None
servo_x_now = None
servo_center = 100

# Button
btn_enable = True
channel_btn_1 = "SPI2_CS1" # BOARD:16
channel_btn_2 = "SPI2_CS0" # BOARD:18
channel_btn_3 = "SPI1_MOSI" # BOARD:19
btn1_pressed = False
btn2_pressed = False
btn3_pressed = False
btn1_sent = False
btn2_sent = False
btn3_sent = False
btn1_time = None
btn2_time = None
btn3_time = None

# 与飞机通信
wireless_enable = True
wireless_ser = None
Wireless_UART = "/dev/UARTWireless"

# 向小车传数据
usb_enable = True
usb_ser = None
USB_UART = "/dev/UARTUSB"

# 向串口屏传数据
board_enable = True
Board_UART = "/dev/ttyTHS1"
board_ser = None

runMode = None
backMode = None

drone_pose_x = 0.00
drone_pose_y = 0.00
drone_pose_z = 0.00
cumulative_voyage = 0.00

fire_pose_x = None
fire_pose_y = None

t265_started = False;

mode_pub = None


def t265_callback(t265_msg):
    global t265_started
    t265_started = True;


def servo_process(servo_msg):
    global servo_kit, servo_x_now
    servo_x_now += servo_msg.data
    servo_kit.servo[0].angle = servo_x_now
    rospy.loginfo("Servo set to:" + servo_x_now)


def button_process():
    global btn1_pressed, btn2_pressed, btn3_pressed
    global btn1_sent, btn2_sent, btn3_sent, btn1_time, btn2_time, btn3_time

    btn1_input = GPIO.input(channel_btn_1);
    btn2_input = GPIO.input(channel_btn_2);
    btn3_input = GPIO.input(channel_btn_3);

    if btn1_input == GPIO.LOW:
        if (not btn1_pressed):
            btn1_time = rospy.get_rostime()
        btn1_pressed = True
        # rospy.loginfo("Button 1 Pressed")
    else:
        btn1_pressed = False
        btn1_sent = False

    if btn2_input == GPIO.LOW:
        if (not btn2_pressed):
            btn2_time = rospy.get_rostime()
        btn2_pressed = True
        # rospy.loginfo("Button 2 Pressed")
    else:
        btn2_pressed = False
        btn2_sent = False

    if btn3_input == GPIO.LOW:
        if (not btn3_pressed):
            btn3_time = rospy.get_rostime()
        btn3_pressed = True
        rospy.loginfo("Button 3 Pressed")
    else:
        btn3_pressed = False
        btn3_sent = False

    global runMode
    if btn1_pressed and (not btn1_sent) and (rospy.get_rostime() - btn1_time > rospy.Duration(2)):
        # 按了3秒
        btn1_sent = True
        runMode = 1
        rospy.loginfo("Mode 1")

    if btn2_pressed and (not btn2_sent) and (rospy.get_rostime() - btn2_time > rospy.Duration(2)):
        btn2_sent = True
        runMode = 2
        rospy.loginfo("Mode 2")

    if btn3_pressed and (not btn3_sent) and (rospy.get_rostime() - btn3_time > rospy.Duration(5)):
        # 按了6秒，这是关机命令
        btn3_sent = True
        runMode = 99
        rospy.loginfo("Mode 99")

        # rospy.loginfo("Lock:{} 1:{} 2:{} 3:{}".format(btnLock_input, btn1_input, btn2_input, btn3_input))

# 无人机与小车通过无线串口传输信息
# 小车接收：
#   1.当前坐标 pose(x:1.00,y:1.00,z:1.80)\n
#   2.火源坐标 fire(x:1.50,y:1.50)\n
# 小车发送：
#   1.行动模式 mode(1)\n
uart_buffer = "";
def wireless_process(event):
    global uart_buffer
    if wireless_ser.in_waiting:
        readB = wireless_ser.read(1)
        read = readB.decode()
        if read != "\n": # 还没发送完毕
            uart_buffer += read
        else:
            uart_buffer += read
            rospy.loginfo(uart_buffer)
            if str(uart_buffer).startswith("pose["): # 当前坐标
                global drone_pose_x, drone_pose_y, drone_pose_z, cumulative_voyage
                x_temp = uart_buffer[uart_buffer.index('(') + 1:]
                x = x_temp[:x_temp.index(')')]
                y_temp = x_temp[x_temp.index('(') + 1:]
                y = y_temp[:y_temp.index(')')]
                z_temp = y_temp[y_temp.index('(') + 1:]
                z = z_temp[:z_temp.index(')')]

                cumulative_voyage += abs(float(x) - drone_pose_y) + abs(-float(y) - drone_pose_x)

                drone_pose_x = -float(y)
                drone_pose_y = float(x)
                drone_pose_z = float(z)
                # rospy.loginfo("Drone X: {}, Y: {}, Z: {}".format(drone_pose_x, drone_pose_y, drone_pose_z))

            elif str(uart_buffer).startswith("fire["): # 火源坐标
                global fire_pose_x, fire_pose_y
                fire_x_temp = uart_buffer[uart_buffer.index('(') + 1:]
                fire_x = fire_x_temp[:fire_x_temp.index(')')]
                fire_y_temp = fire_x_temp[fire_x_temp.index('(') + 1:]
                fire_y = fire_y_temp[:fire_y_temp.index(')')]
                fire_pose_x = -float(fire_y) # 显示坐标系和飞机返回的坐标系不一致
                fire_pose_y = float(fire_x)
                rospy.loginfo("Fire X: {}, Y: {}".format(fire_pose_x, fire_pose_y))

                global mode_pub
                mode_pub.publish(1)

            uart_buffer = ""

    global runMode, backMode
    if not (runMode is None) and backMode != runMode: # 发送运行模式
        mode = "mode({})\n".format(runMode)
        wireless_ser.write(mode.encode())
        backMode = runMode
        rospy.loginfo("Sent Run Mode: " + mode)

        if runMode == 99: # 关机
            if board_enable:
                board_ser.write("mode.txt=\"99\"".encode("utf-8"))
                board_ser.write(bytes.fromhex('ff ff ff'))
            rospy.logwarn("Shutdown!!!!!!")
            rospy.sleep(3.0)
            os.system("bash /home/jetson/shutdown.sh")

# 串口屏通信
track_start_x = 178
track_start_y = 247
def usb_process(event):
    global drone_pose_x, drone_pose_y, drone_pose_z, cumulative_voyage
    board_ser.write("pose_x.txt=\"{:.2f}\"".format(drone_pose_x).encode("utf-8"))
    board_ser.write(bytes.fromhex('ff ff ff'))

    board_ser.write("pose_y.txt=\"{:.2f}\"".format(drone_pose_y).encode("utf-8"))
    board_ser.write(bytes.fromhex('ff ff ff'))

    board_ser.write("pose_z.txt=\"{:.2f}\"".format(drone_pose_z).encode("utf-8"))
    board_ser.write(bytes.fromhex('ff ff ff'))

    # 累计航程
    board_ser.write("total.txt=\"{:.2f}\"".format(cumulative_voyage).encode("utf-8"))
    board_ser.write(bytes.fromhex('ff ff ff'))

    global runMode
    if not runMode is None:
        board_ser.write("mode.txt=\"{}\"".format(runMode).encode("utf-8"))
        board_ser.write(bytes.fromhex('ff ff ff'))
    elif t265_started: # 通过-1提示系统启动已完成
        board_ser.write("mode.txt=\"-1\"".encode("utf-8"))
        board_ser.write(bytes.fromhex('ff ff ff'))
    else: # T265未启动显示-2
        board_ser.write("mode.txt=\"-2\"".encode("utf-8"))
        board_ser.write(bytes.fromhex('ff ff ff'))

    # 绘制轨迹
    # 图片开始坐标是x=155，图片高度272
    track_next_x = 325 / 4.8 * drone_pose_x + track_start_x
    track_next_y = track_start_y - (272 / 4 * drone_pose_y)
    board_ser.write("cirs {},{},2,1024".format(int(track_next_x), int(track_next_y)).encode("utf-8"))
    board_ser.write(bytes.fromhex('ff ff ff'))
    # rospy.loginfo("cirs {},{},2,1024".format(int(track_next_x), int(track_next_y)))
    # rospy.loginfo("Drone X: {}, Y: {}, Z: {}".format(drone_pose_x, drone_pose_y, drone_pose_z))

    # 绘制火源点
    global fire_pose_x, fire_pose_y
    if not (fire_pose_x is None) and not (fire_pose_y is None):
        track_fire_x = 325 / 4.8 * fire_pose_x + track_start_x
        track_fire_y = track_start_y - (272 / 4 * fire_pose_y)
        board_ser.write("cirs {},{},4,63488".format(int(track_fire_x), int(track_fire_y)).encode("utf-8"))
        board_ser.write(bytes.fromhex('ff ff ff'))
        fire_pose_x = None
        fire_pose_y = None

# 小车通信
def car_head_callback(car_msg):
    # if car_msg.data > 0 and car_msg.data <= 5:
    #     usb_ser.write("n".encode()) # 右转5度
    # elif car_msg.data > 5 and car_msg.data <= 15:
    #     usb_ser.write("o".encode())
    # elif car_msg.data > 15 and car_msg.data <= 25:
    #     usb_ser.write("p".encode())
    # elif car_msg.data > 25 and car_msg.data <= 35:
    #     usb_ser.write("q".encode())
    # elif car_msg.data > 35:
    #     usb_ser.write("r".encode())
    
    # elif car_msg.data > -5 and car_msg.data <= 0:
    #     usb_ser.write("h".encode()) # 左转5度
    # elif car_msg.data > -15 and car_msg.data <= -5:
    #     usb_ser.write("i".encode())
    # elif car_msg.data > -25 and car_msg.data <= -15:
    #     usb_ser.write("j".encode())
    # elif car_msg.data > -35 and car_msg.data <= -25:
    #     usb_ser.write("k".encode())
    # elif car_msg.data < -35:
    #     usb_ser.write("l".encode())


    if car_msg.data > 0 and car_msg.data <= 2:
        usb_ser.write("n".encode()) # 右转5度
    elif car_msg.data > 2 and car_msg.data <= 4:
        usb_ser.write("o".encode())
    elif car_msg.data > 4 and car_msg.data <= 6:
        usb_ser.write("p".encode())
    elif car_msg.data > 6 and car_msg.data <= 8:
        usb_ser.write("q".encode())
    elif car_msg.data > 8:
        usb_ser.write("r".encode())
    
    elif car_msg.data > -2 and car_msg.data <= 0:
        usb_ser.write("h".encode()) # 左转5度
    elif car_msg.data > -4 and car_msg.data <= -2:
        usb_ser.write("i".encode())
    elif car_msg.data > -6 and car_msg.data <= -4:
        usb_ser.write("j".encode())
    elif car_msg.data > -8 and car_msg.data <= -6:
        usb_ser.write("k".encode())
    elif car_msg.data < -8:
        usb_ser.write("l".encode())


def car_start_callback(car_msg):
    if car_msg.data == 1: # 前进
        usb_ser.write("f".encode())
    elif car_msg.data == -1: # 后退
        usb_ser.write("b".encode())
    else: # 停止
        usb_ser.write("s".encode())
    rospy.loginfo("Car start change")


def initDevices():
    rospy.init_node('devices_node', anonymous=True)
    rate = rospy.Rate(10)

    state_pub = rospy.Publisher('/my_utils/state', Bool, queue_size=10)

    global mode_pub
    mode_pub = rospy.Publisher('/my_utils/car_mode', Int16, queue_size=10)

    rospy.Subscriber('/t265/odom/sample', Odometry, t265_callback)

    GPIO.setmode(GPIO.TEGRA_SOC)

    # 初始化舵机
    if servo_enable:
        global servo_kit, servo_x_now
        servo_kit = ServoKit(channels=16)
        servo_kit.servo[0].angle = servo_center
        servo_kit.servo[1].angle = servo_center
        servo_x_now = servo_center
        rospy.Subscriber("/my_utils/servo", Int16, servo_process)

    # 初始化按钮
    if btn_enable:
        global button_enable
        GPIO.setup(channel_btn_1, GPIO.IN)
        GPIO.setup(channel_btn_2, GPIO.IN)
        GPIO.setup(channel_btn_3, GPIO.IN)

        btn1_input = GPIO.input(channel_btn_1);
        btn2_input = GPIO.input(channel_btn_2);
        btn3_input = GPIO.input(channel_btn_3);

        if btn1_input == GPIO.LOW or btn2_input == GPIO.LOW or btn3_input == GPIO.LOW:
            rospy.logerr("Button connects incorrectly!")
            button_enable = False

    # 初始化无线通信
    if wireless_enable:
        while not rospy.is_shutdown():
            try:
                global wireless_ser
                wireless_ser = serial.Serial(Wireless_UART, 9600, timeout=3)
                if wireless_ser.is_open:
                    break
            except Exception:
                rospy.logerr("Unable to open Wireless UART. Wait 5 seconds and try again.")
                rospy.sleep(5)
                continue

    # 初始化小车通信
    if usb_enable:
        while not rospy.is_shutdown():
            try:
                global usb_ser
                usb_ser = serial.Serial(USB_UART, 115200, timeout=3)
                if usb_ser.is_open:
                    break
            except Exception:
                rospy.logerr("Unable to open USB UART. Wait 5 seconds and try again.")
                rospy.sleep(5)
                continue
        rospy.Subscriber("/my_utils/car_head", Float32, car_head_callback)
        rospy.Subscriber("/my_utils/car_start", Int16, car_start_callback)

    # 初始化串口屏通信
    if board_enable:
        while not rospy.is_shutdown():
            try:
                global board_ser
                board_ser = serial.Serial(Board_UART, 115200, timeout=3)
                if board_ser.is_open:
                    break
            except Exception:
                rospy.logerr("Unable to open BOARD UART. Wait 5 seconds and try again.")
                rospy.sleep(5)
                continue

    rospy.loginfo("Devices Node Running……")
    if usb_enable: # 避免串口发送太多数据，所以在这里用定时器更新数据
        rospy.Timer(rospy.Duration(0.5), usb_process)
    if wireless_enable:
        rospy.Timer(rospy.Duration(0.01), wireless_process)
    while not rospy.is_shutdown():
        state_pub.publish(True)

        if btn_enable:
            button_process()

        rate.sleep()

    GPIO.cleanup()
    if wireless_enable:
        wireless_ser.close()

    if usb_enable:
        usb_ser.close()

    if board_enable:
        board_ser.close()


if __name__ == '__main__':
    try:
        initDevices()
    except rospy.ROSInterruptException:
        pass
