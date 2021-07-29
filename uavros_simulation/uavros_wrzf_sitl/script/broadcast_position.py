#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 无人争锋 广播位置
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
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
import time
import json

f = 20

class broadcast_position:
    def __init__(self):
        global latitude, longitude, z, vx, vy, vz, armed, mode, yaw
        latitude, longitude, z, vx, vy, vz, yaw = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        armed = "false"
        mode = ""
        rate = rospy.Rate(f)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_position_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.local_pose_callback)
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.local_velocity_callback)
        rospy.Subscriber('/yaw_east_to_head_deg', Float32, self.yaw_callback)
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        json_path = rospy.get_param('~json_filepath', './position.json')

        # Main while loop.
        while not rospy.is_shutdown():
            self.send_position(json_path)
            try:
                self.send_position(json_path)
                # print('position: ', position)
            except:
                print('json-WRONG!')
                pass

            rate.sleep()

    def send_position(self, filename):
        global latitude, longitude, z, vx, vy, vz, armed, mode, yaw
        with open(filename, 'w') as f:
            save = {"latitude": latitude, "longitude": longitude, "z": z, "vx": vx, "vy": vy, "vz": vz, "armed": armed,
                    "mode": mode, "yaw": yaw}
            saveb = json.dumps(save)
            f.write(saveb)
            return True

    def global_position_callback(self, msg):
        global latitude, longitude
        latitude = msg.latitude
        longitude = msg.longitude

    def local_pose_callback(self, msg):
        global z
        z = msg.pose.position.z
        # print(z)

    def local_velocity_callback(self, msg):
        global vx, vy, vz
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        vz = msg.twist.linear.z

    def yaw_callback(self, msg):
        global yaw
        yaw = 90 - msg.data
        if yaw < 0:
            yaw = yaw + 360
        elif yaw > 360:
            yaw = yaw - 360

    def state_callback(self, msg):
        global armed, mode
        armed = msg.armed
        mode = msg.mode
        # print(armed, mode)

if __name__ == '__main__':
    # ROS节点初始化
    rospy.init_node('broadcast_position', anonymous=True)
    try:
        broadcast_position()
    except rospy.ROSInterruptException:
        pass
