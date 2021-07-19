#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 无人争锋 飞控to决策
import os
import time
import numpy as np
import math
import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
from std_msgs.msg import Int32
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from car_control.msg import LinktrackAnchorframe0
from sensor_msgs.msg import Imu
import time
import json

f = 50

class message:
    def __init__(self):
        rate = rospy.Rate(f)
        # Main while loop.
        while not rospy.is_shutdown():
            try:
                data1 =  self.read_from_fk()
                print('data1: ', data1)
            except:
                print('txt-WRONG!')
            try:
                position = self.get_position()
                print('position: ', position)
            except:
                print('json-WRONG!')

            rate.sleep()

    def read_from_fk(self):
        filename = "/home/szx/VideoTarget_ws/src/message/scripts/from_fk.txt"
        with open(filename, 'r') as f:
            data = f.read()
            return data

    def get_position(self):
        filename = "/home/szx/VideoTarget_ws/src/message/scripts/position.json"
        with open(filename, 'r') as f:
            data = json.load(f)
            print(data)
            return [data["x"], data["y"], data["z"]]


if __name__ == '__main__':
    # ROS节点初始化
    rospy.init_node('message', anonymous=True)
    try:
        message()
    except rospy.ROSInterruptException:
        pass
