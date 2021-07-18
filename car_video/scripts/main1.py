#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
@Author  : Li-Jinjie
@Time    : 2021/7/17 15:03
@File    : main
@Email   : lijinjie362@outlook.com
@Description:  None
'''
import cv2
from detecting.detect_car import CarDetector
import rospy
from geometry_msgs.msg import Quaternion
import math
# from tracking import


def main():
    rospy.init_node('car_tracking', anonymous=True)
    target_angle_pub = rospy.Publisher('/target_angle', Quaternion, queue_size=1)
    rate = rospy.Rate(30)
    vc = cv2.VideoCapture(0)  # 捕获摄像头的帧
    vc.set(3, 1280)  # 设置分辨率
    vc.set(4, 720)
    success, frame = vc.read()
    car_detector = CarDetector()
    while not success:
        print("camera ERROR!")
        time.sleep(1)
    Dx = 0
    Dy = 0
    yaw = 0
    flag = 0
    a1 = frame.shape[0]  # 高
    b1 = frame.shape[1]  # 宽
    target_angle = Quaternion()
    while not rospy.is_shutdown():  # 循环读取视频帧
        rval, img = vc.read()
        if rval is False:
            break

        # 选择矩形
        box, center, side_length, degree = car_detector.detect_car(img)

        img_quads_detect = img.copy()
        if box is not None:
            cv2.drawContours(img_quads_detect, [box], 0, (0, 0, 255), 2)
            print(center[0][0], center[0][1], degree[0])
            Dx = math.atan(2.0 * (center[0][0] - b1 / 2) * math.tan(35.0 * math.pi / 180.0) / b1) * 180.0 / math.pi
            Dy = - math.atan(2.0 * (center[0][1] - a1 / 2) * math.tan(35.0 * math.pi / 180.0) / a1) * 180.0 / math.pi
            yaw = degrees_list[0]
            flag = 1
        else:
            flag = 0
            print("LOST!!")

        target_angle.x = Dy
        target_angle.y = - Dx
        target_angle.z = yaw
        target_angle.w = flag
        print(target_angle)
        target_angle_pub.publish(target_angle)

        cv2.imshow("quads detection", img_quads_detect)
        cv2.waitKey(10)

    vc.release()


if __name__ == "__main__":
    main()
