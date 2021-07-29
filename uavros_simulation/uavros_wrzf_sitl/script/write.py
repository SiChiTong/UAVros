#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 无人争锋 决策to飞控
import os

#from numpy.lib.function_base import vectorize
import rospy
import json
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
f = 10



class message:
    def __init__(self):
        rate = rospy.Rate(f)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.pos_callback)
        rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self.vel_callback)
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/yaw',Float32, self.yaw_callback)
        # Main while loop.
        while not rospy.is_shutdown():
            try:
                # print(jc_data)
                filename = "./tojson.json"
                fsave = open(filename, 'w')
                save = {"latitude": latitude, "longitude": longitude, "altitude": altitude,"vx":vx,"vy":vy,"vz":vz,"state":state,"yaw":yaw,"unlocked":unlocked}
                saveb = json.dumps(save)
                fsave.write(saveb)
                fsave.write('\n')
            except:
                print('txt-WRONG!')

            rate.sleep()
    def pos_callback(self,pos):
        global latitude,longitude,altitude
        latitude=pos.latitude
        longitude=pos.longitude
        altitude=pos.altitude
    def  vel_callback(self,vel):
        global vx,vy,vz
        vx=vel.twist.linear.x
        vy=vel.twist.linear.y
        vz=vel.twist.linear.z

    def state_callback(self,stat):
        global state,unlocked
        state=stat.mode
        unlocked=stat.armed
    def yaw_callback(self,yaw_):
        global yaw
        yaw=yaw_.data

if __name__ == '__main__':
    # ROS节点初始化
    rospy.init_node('message', anonymous=True)
    try:
        message()
        
    except rospy.ROSInterruptException:
        pass
