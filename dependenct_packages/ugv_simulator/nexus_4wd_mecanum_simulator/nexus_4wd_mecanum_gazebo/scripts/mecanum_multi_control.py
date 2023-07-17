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




class multi_control():
	def __init__(self):
		#设置循环的频率
		start_time = time.time()
		flagg = 1
		rate = rospy.Rate(f)
		self.command = 0
		q1_tmp = 0
		reset_flag = 1
		car1_vel = Twist()
		car2_vel = Twist()
		car3_vel = Twist()
		car1_vel_pub = rospy.Publisher('/car2/cmd_vel', Twist, queue_size=10)
		car2_vel_pub = rospy.Publisher('/car3/cmd_vel', Twist, queue_size=10)
		car3_vel_pub = rospy.Publisher('/car4/cmd_vel', Twist, queue_size=10)
		command_sub = rospy.Subscriber('/command', Int32 , self.command_callback)
		pose_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
		car1_pose = ModelState()
		car1_pose.model_name = 'car2'
		car2_pose = ModelState()
		car2_pose.model_name = 'car3'
		car3_pose = ModelState()
		car3_pose.model_name = 'car4'
		# Main while loop.
		while not rospy.is_shutdown():
			t = time.time() - start_time
			car1_position = rospy.wait_for_message('/car2/odom', Odometry, timeout=5)
			car2_position = rospy.wait_for_message('/car3/odom', Odometry, timeout=5)
			car3_position = rospy.wait_for_message('/car4/odom', Odometry, timeout=5)
			car1_x = car1_position.pose.pose.position.x
			car1_y = car1_position.pose.pose.position.y
			car2_x = car2_position.pose.pose.position.x
			car2_y = car2_position.pose.pose.position.y
			car3_x = car3_position.pose.pose.position.x
			car3_y = car3_position.pose.pose.position.y
			# QuatToEuler

			(car1_r, car1_p, car1_yaw) = tf.transformations.euler_from_quaternion(
				[car1_position.pose.pose.orientation.x, car1_position.pose.pose.orientation.y,
				 car1_position.pose.pose.orientation.z, car1_position.pose.pose.orientation.w])
			(car2_r, car2_p, car2_yaw) = tf.transformations.euler_from_quaternion(
				[car2_position.pose.pose.orientation.x, car2_position.pose.pose.orientation.y,
				 car2_position.pose.pose.orientation.z, car2_position.pose.pose.orientation.w])
			(car3_r, car3_p, car3_yaw) = tf.transformations.euler_from_quaternion(
				[car3_position.pose.pose.orientation.x, car3_position.pose.pose.orientation.y,
				 car3_position.pose.pose.orientation.z, car3_position.pose.pose.orientation.w])


		
			if (self.command == 4):
				print('run')
				car1_vel.linear.x = 0.8
				car1_vel.angular.z = 0.0349
				car1_vel_pub.publish(car1_vel)
				if t > 5 :
					if flagg == 1 :
						car2_pose.pose.position.y = 10
						pose_pub.publish(car2_pose)
						flagg = 2
					car2_vel.linear.x = 0.8
					car2_vel.angular.z = 0.0349
					car2_vel_pub.publish(car2_vel)
					if t > 10 :
						if flagg == 2 :
							car3_pose.pose.position.y = 10
							pose_pub.publish(car3_pose)
							flagg = 3
						car3_vel.linear.x = 0.8
						car3_vel.angular.z = 0.0349
						car3_vel_pub.publish(car3_vel)

	def command_callback(self,msg):
		self.command = msg.data
		print("Recieve command  ",msg.data)
if __name__ == '__main__':
	# ROS节点初始化
	rospy.init_node('multi_control', anonymous=True)
	try:
		multi_control()
	except rospy.ROSInterruptException:
		pass


