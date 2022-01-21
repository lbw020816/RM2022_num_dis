#!/usr/bin/python3
from scipy import optimize
import math
import rospy
from std_msgs.msg import Float32
from serial_common.msg import serialWrite, gimbal
from exp import exp
import numpy as np

lead_cal = False  # 提前量计算开关
width = 640
height = 480
horizontal_angle = 0.6981317
vertical_angle = horizontal_angle / width * height
pitch2muzzle = -0.1    # 距离产生偏差调这个
thetaX = math.tan(horizontal_angle/2)
thetaY = math.tan(vertical_angle/2)
phy_pitch = 0.
vx = 0.
vy = 0.
wz = 0.
yaw = 0.
serial_publisher = None
last_xl = 0.
last_yl = 0.
last_yaw = 0.
last_time = 0.
dt = 100000000.


def pixel2angle(x, y):
    ax = math.atan((x / (width / 2) - 1) * thetaX) - 0.025
    ay = math.atan((y / (height / 2) - 1) * thetaY) + 0.04    # 稳态偏差调这个
    return [ax, ay]


def get_length(length, yaw, pitch, real_pitch):
    length1 = length * math.cos(yaw)
    # print(length, length1)
    angle = pitch + real_pitch
    lx1 = length * math.cos(angle)
    ly1 = length * math.sin(angle)
    lx2 = pitch2muzzle * math.cos(real_pitch)
    ly2 = pitch2muzzle * math.sin(real_pitch)
    # print(angle,pitch,real_pitch, lx1, ly1, ly2)
    return [lx1+0.1, ly1+ly2, math.atan2(lx1+lx2, ly1+ly2)]


def solve(x, y, length):
    global last_xl, last_yl, last_yaw
    yaw, pitch = pixel2angle(x, y)
    return [yaw, pitch + phy_pitch]


def pitch_callback(res):
    global phy_pitch, vx, vy, wz
    vx = res.vx
    vy = res.vy
    wz = res.wz
    phy_pitch = res.pitch


def identify_callback(res):
    global dt, last_time, last_xl, last_yl, last_yaw
    x = res.xlocation
    y = res.ylocation
    l = res. depth / 1000
    msg = serialWrite()
    if x > 2000 or y > 2000:
        msg.xlocation = 30000
        msg.ylocation = 30000
        msg.depth = 1
        last_time = 0.
        last_xl = 0.
        last_yl = 0.
        last_yaw = 0.
    else:
        if last_time != 0:
            dt = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9 - last_time
        last_time = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9
        yaw, pitch = solve(x, y, l)
        msg.xlocation = int(yaw * 1000)
        msg.ylocation = int(pitch * 1000)
        msg.depth = int(l * 1000)
    serial_publisher.publish(msg)


if __name__ == '__main__':
    node = rospy.init_node('ballistic_node')
    pitch_sub = rospy.Subscriber('/robot_status', gimbal, pitch_callback)
    identify_sub = rospy.Subscriber('/write_pixel', serialWrite, identify_callback)
    serial_publisher = rospy.Publisher('/write_angle', serialWrite, queue_size=5)
    rospy.spin()
