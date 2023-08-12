#!/usr/bin/env python
import rospy
import serial
import Jetson.GPIO as GPIO
from std_msgs.msg import Int16, Bool
from my_utils.msg import my_led, my_servo
from adafruit_servokit import ServoKit

# adafruit_servokit强制使用了TEGRA_SOC，所以以下针脚都使用该配置
# BOARD与TEGRA_SOC的对照关系见
# https://github.com/NVIDIA/jetson-gpio/blob/master/lib/python/Jetson/GPIO/gpio_pin_data.py

button_enable = True
CHANNEL_BUTTON_LOCK = "GP122" # BOARD:12
CHANNEL_BUTTON_1 = "GP40_SPI3_CS1_N" # BOARD:16
CHANNEL_BUTTON_2 = "GP39_SPI3_CS0_N" # BOARD:18
CHANNEL_BUTTON_3 = "GP37_SPI3_MISO" # BOARD:22
button_pub = None
btn1_pressed = False
btn2_pressed = False
btn3_pressed = False
btn1_sent = False
btn2_sent = False
btn3_sent = False
btn1_time = None
btn2_time = None
btn3_time = None

buzzer_enable = True
CHANNEL_BUZZER = "GP51_SPI1_CS1_N" # BOARD:26

led_enable = True
led_ser = None

servo_enable = False
servo_kit = None
servo_center = 100
servo_x_index = 0
servo_y_index = 1
servo_x_now = None
servo_y_now = None


def buzzer_process(buzzer_msg):
    GPIO.output(CHANNEL_BUZZER, GPIO.HIGH if buzzer_msg.data else GPIO.LOW)


def led_process(led_msg):
    rospy.loginfo("LED Receive: {}".format(led_msg))
    if led_msg.led_index > 4 or led_msg.led_index < -2:
        rospy.logwarn("Invalid LED Index")
        return
        
    cmd = None # 下面是屎山代码
    if led_msg.led_index == 1:
        if not led_msg.enable:
            cmd = "a"
        elif led_msg.rgb == 114: # r 红色
            cmd = "1"
        elif led_msg.rgb == 103: # g 绿色
            cmd = "2"
        elif led_msg.rgb == 98: # b 蓝色
            cmd = "3"
    elif led_msg.led_index == 2:
        if not led_msg.enable:
            cmd = "b"
        elif led_msg.rgb == 114:
            cmd = "4"
        elif led_msg.rgb == 103:
            cmd = "5"
        elif led_msg.rgb == 98:
            cmd = "6"
    elif led_msg.led_index == 3:
        if not led_msg.enable:
            cmd = "c"
        elif led_msg.rgb == 114:
            cmd = "7"
        elif led_msg.rgb == 103:
            cmd = "8"
        elif led_msg.rgb == 98:
            cmd = "9"
    elif led_msg.led_index == 4:
        if not led_msg.enable:
            cmd = "d"
        elif led_msg.rgb == 114:
            cmd = "A"
        elif led_msg.rgb == 103:
            cmd = "B"
        elif led_msg.rgb == 98:
            cmd = "C"
    elif led_msg.led_index == -1: # 红色激光
        if not led_msg.enable:
            cmd = "e"
        else:
            cmd = "RED"
    elif led_msg.led_index == -2: # 绿色激光
        if not led_msg.enable:
            cmd = "f"
        else:
            cmd = "GREEN"

    if cmd is None:
        rospy.logerr("LED Receive data error!")
        return

    led_ser.write(cmd.encode())
    rospy.loginfo("UART Send: {}".format(cmd))

        # commandColor = None
        # if led_msg.rgb == "r":
        #     commandColor = "RED"
        # elif led_msg.rgb == "g":
        #     commandColor = "GRN"
        # elif led_msg.rgb == "b":
        #     commandColor = "BLU"
        
        # commandIndex = None
        # if led_msg.led_index == -1: # 指激光
        #     commandIndex = "1";
        #     commandColor = "LAS";
        # elif led_msg.led_index > 8:
        #     rospy.logwarn("Invalid LED Index")
        #     led_flag = False
        #     return
        # else:
        #     commandIndex = str(led_msg.led_index)

        # if led_msg.duration < 0: # 常亮
        #     cmd = commandIndex + commandColor
        #     led_ser.write(cmd + "\n")
        #     rospy.loginfo("UART Send: {}".format(cmd))
        # elif led_msg.duration == 0: # 关闭
        #     cmd = commandIndex + "CLO"
        #     led_ser.write(cmd + "\n")
        #     rospy.loginfo("UART Send: {}".format(cmd))
        # else:
        #     cmdOn = commandIndex + commandColor
        #     led_ser.write(cmdOn + "\n")
        #     rospy.loginfo("UART Send: {}".format(cmdOn))

        #     rospy.sleep(led_msg.duration)

        #     cmdOff = commandIndex + "CLO"
        #     led_ser.write(cmdOff + "\n")
        #     rospy.loginfo("UART Send: {}".format(cmdOff))


def button_process():
    global button_pub, btn1_pressed, btn2_pressed, btn3_pressed
    global btn1_sent, btn2_sent, btn3_sent, btn1_time, btn2_time, btn3_time

    btnLock_input = GPIO.input(CHANNEL_BUTTON_LOCK)
    btn1_input = GPIO.input(CHANNEL_BUTTON_1);
    btn2_input = GPIO.input(CHANNEL_BUTTON_2);
    btn3_input = GPIO.input(CHANNEL_BUTTON_3);

    if btnLock_input == GPIO.HIGH:
        # 被按下
        rospy.loginfo("Button Lock Pressed")

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
        # rospy.loginfo("Button 3 Pressed")
    else:
        btn3_pressed = False
        btn3_sent = False

    if btn1_pressed and (not btn1_sent) and (rospy.get_rostime() - btn1_time > rospy.Duration(2)):
        # 按了3秒
        btn1_sent = True
        button_pub.publish(1)
        rospy.loginfo("Mode 1")

    if btn2_pressed and (not btn2_sent) and (rospy.get_rostime() - btn2_time > rospy.Duration(2)):
        btn2_sent = True
        button_pub.publish(2)
        rospy.loginfo("Mode 2")
        
    if btn3_pressed and (not btn3_sent) and (rospy.get_rostime() - btn3_time > rospy.Duration(2)):
        btn3_sent = True
        button_pub.publish(3)
        rospy.loginfo("Mode 3")

        # rospy.loginfo("Lock:{} 1:{} 2:{} 3:{}".format(btnLock_input, btn1_input, btn2_input, btn3_input))

def servo_process(servo_msg):
    global servo_kit, servo_x_now, servo_y_now
    servo_x_now += servo_msg.x_offset
    servo_y_now += servo_msg.y_offset
    servo_kit.servo[servo_x_index].angle = servo_x_now
    servo_kit.servo[servo_y_index].angle = servo_y_now


def initDevices():
    rospy.init_node('devices_node', anonymous=True)
    rate = rospy.Rate(10)

    state_pub = rospy.Publisher('/my_utils/state', Bool, queue_size=10)
    GPIO.setmode(GPIO.TEGRA_SOC)

    # 初始化LED
    if led_enable:
        rospy.Subscriber("/my_utils/led", my_led, led_process)
        while not rospy.is_shutdown():
            try:
                global led_ser
                led_ser = serial.Serial("/dev/ttyUSB0", 9600, timeout=3)
                if led_ser.is_open:
                    break
            except Exception:
                rospy.logerr("Unable to open port. Wait 5 seconds and try again.")
                rospy.sleep(5)
                continue

    # 初始化按钮
    global button_enable
    if button_enable:
        GPIO.setup(CHANNEL_BUTTON_LOCK, GPIO.IN)
        GPIO.setup(CHANNEL_BUTTON_1, GPIO.IN)
        GPIO.setup(CHANNEL_BUTTON_2, GPIO.IN)
        GPIO.setup(CHANNEL_BUTTON_3, GPIO.IN)
        # 检测当前状态
        btn1_input = GPIO.input(CHANNEL_BUTTON_1);
        btn2_input = GPIO.input(CHANNEL_BUTTON_2);
        btn3_input = GPIO.input(CHANNEL_BUTTON_3);

        if btn1_input == GPIO.LOW or btn2_input == GPIO.LOW or btn3_input == GPIO.LOW:
            rospy.logerr("Button connects incorrectly!")
            button_enable = False
        else:
            global button_pub
            button_pub = rospy.Publisher('/my_utils/mode', Int16, queue_size=10)

    # 初始化蜂鸣器
    if buzzer_enable:
        GPIO.setup(CHANNEL_BUZZER, GPIO.OUT)
        GPIO.output(CHANNEL_BUZZER, GPIO.LOW)
        rospy.Subscriber("/my_utils/buzzer", Bool, buzzer_process)

    # 初始化舵机
    if servo_enable:
        global servo_kit, servo_x_now, servo_y_now
        servo_kit = ServoKit(channels=16)
        servo_kit.servo[servo_x_index].angle = servo_center
        servo_kit.servo[servo_y_index].angle = servo_center
        servo_x_now = servo_center
        servo_y_now = servo_center
        rospy.Subscriber("/my_utils/servo", my_servo, servo_process)


    rospy.loginfo("Devices Node Running……")
    while not rospy.is_shutdown():
        if button_enable:
            button_process()

        state_pub.publish(True)
        rate.sleep()
    
    GPIO.cleanup()
    
    if led_enable and not (led_ser is None):
        led_ser.close()

if __name__ == '__main__':
    try:
        initDevices()
    except rospy.ROSInterruptException:
        pass