#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 协同制导
import os
import cv2
import time
import numpy as np
import math
# from cir2 import findcir
import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf
from std_msgs.msg import Int32
from gazebo_msgs.msg import ModelState
import time
from time import sleep

set_pose = [	0,10,
				-3,-10,
	   			3,-10,
	   			0,-9]
set_angle_target = [0,0,0]
set_angle = [0,0,math.pi/2]
wp = 4
leader_angle_except = [0,-10,10]
f = 5
leader_vel = 0.5
follower_vel = 0.55
dh_q3 = 0
h_q = [0,0,0,0,0]
dh_q = [0,0,0,0]
kp1 = 0.4
kp2 = 0.03
kd = 0.3




class single_control():
	def __init__(self):
		#设置循环的频率
		start_time = time.time()
		flagg = 1
		rate = rospy.Rate(f)
		self.command = 0
		q1_tmp = 0
		reset_flag = 1
		car1_vel = Twist()
		car1_vel_pub = rospy.Publisher('/car2/cmd_vel', Twist, queue_size=10)
		command_sub = rospy.Subscriber('/command', Int32 , self.command_callback)
		pose_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
		car1_pose = ModelState()
		car1_pose.model_name = 'car2'
		# Main while loop.
		while not rospy.is_shutdown():
			t = time.time() - start_time
			car1_position = rospy.wait_for_message('/car2/odom', Odometry, timeout=5)
			car1_x = car1_position.pose.pose.position.x
			car1_y = car1_position.pose.pose.position.y
			# QuatToEuler

			(car1_r, car1_p, car1_yaw) = tf.transformations.euler_from_quaternion(
				[car1_position.pose.pose.orientation.x, car1_position.pose.pose.orientation.y,
				 car1_position.pose.pose.orientation.z, car1_position.pose.pose.orientation.w])



		
			if (self.command == 4):
				print('run')
				car1_vel.linear.x = 0.8
				car1_vel.angular.z = 0.0349
				car1_vel_pub.publish(car1_vel)
	def command_callback(self,msg):
		self.command = msg.data
		print("Recieve command  ",msg.data)
if __name__ == '__main__':
	# ROS节点初始化
	rospy.init_node('single_control', anonymous=True)
	try:
		single_control()
	except rospy.ROSInterruptException:
		pass


