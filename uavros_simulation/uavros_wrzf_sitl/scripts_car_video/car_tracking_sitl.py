#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将发布turtle1/cmd_vel话题，消息类型geometry_msgs::Twist
import os
import cv2
import time
import numpy as np
import math
from cir2_sitl import findcir
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def car_tracking():
    # ROS节点初始化
    rospy.init_node('car_tracking', anonymous=True)
    # 创建一个Publisher，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
    rospy.Subscriber('/iris/usb_cam/image_raw', Image, image_callback, queue_size=1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
    '''
    # 设置循环的频率
    rate = rospy.Rate(30)
    cap = cv2.VideoCapture(0)  # 捕获摄像头的帧
    #cap.set(3, 1280)  # 设置分辨率
    #cap.set(4, 720)
    success, frame = cap.read()
    while not success:
        print("camera ERROR!")
    Dxy = 0
    a1 = frame.shape[0]  # 高
    b1 = frame.shape[1]  # 宽
    target_angle = Point()
    # greeen = (0, 255, 0)
    while not rospy.is_shutdown():
        # 初始化geometry_msgs::Twist类型的消息
        # vel_msg = Twist()

        #success, frame = cap.read()
        # font = cv2.FONT_HERSHEY_SIMPLEX
        # cv2.putText(frame, str(round(Dxy, 2)), (55, 60), font, 2, greeen, 2)
        flag, rect = findcir(frame)
        if flag:
            Dx = math.atan(2.0 * (rect[0] - b1 / 2) * math.tan(35.0 * math.pi / 180.0) / b1) * 180.0 / math.pi
            Dy = - math.atan(2.0 * (rect[1] - a1 / 2) * math.tan(35.0 * math.pi / 180.0) / a1) * 180.0 / math.pi
        target_angle.x = Dy
        target_angle.y = - Dx
        target_angle.z = flag

        # 发布消息
        target_angle_pub.publish(target_angle)
        print(target_angle)
        # rospy.loginfo("Publsh car angle command",q)

        # 按照循环频率延时
        # success, frame = cap.read()
        rate.sleep()
        '''

def image_callback(imgmsg):
    # target_angle_pub = rospy.Publisher('/target_angle', Point, queue_size=1)
    target_angle_pub = rospy.Publisher('/target_angle', Quaternion, queue_size=1)
    bridge = CvBridge()  # Cvbridge can change between ROS image topic and cv frame format!
    target_angle = Point()
    Dx = 0
    Dy = 0
    try:
        frame = bridge.imgmsg_to_cv2(imgmsg, "bgr8")
        a1 = frame.shape[0]  # 高
        b1 = frame.shape[1]  # 宽
        target_angle = Quaternion()
        # greeen = (0, 255, 0)
        flag, rect = findcir(frame)
        if flag:
            Dx = math.atan(2.0 * (rect[0] - b1 / 2) * math.tan(35.0 * math.pi / 180.0) / b1) * 180.0 / math.pi
            Dy = - math.atan(2.0 * (rect[1] - a1 / 2) * math.tan(35.0 * math.pi / 180.0) / a1) * 180.0 / math.pi
        else:
            print('LOST!')
        target_angle.x = Dy
        target_angle.y = - Dx
        target_angle.z = 0
        target_angle.w = flag
        target_angle_pub.publish(target_angle)
        # 发布消息
        print(target_angle)
    except CvBridgeError as e:  # error reminder
        print(e)


if __name__ == '__main__':

    try:
        car_tracking()
    except rospy.ROSInterruptException:
        pass
