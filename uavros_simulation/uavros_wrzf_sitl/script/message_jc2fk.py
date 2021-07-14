#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 无人争锋 决策to飞控
import os
import rospy
import json
from std_msgs.msg import Int32

f = 20



class message:
    def __init__(self):
        rate = rospy.Rate(f)
        jc_data = 'wait'
        jc_data_pub = 0
        # Main while loop.
        while not rospy.is_shutdown():
            try:
                jc_data = self.read_from_jc()
                # print(jc_data)
                #print(str(jc_data))
            except:
                print('txt-WRONG!')
            '''
            try:
                position = self.get_position()
                print('position: ', position)
            except:
                print('json-WRONG!')
            '''
            if jc_data == 'wait':
                jc_data_pub = 0
            elif jc_data == 'takeoff':
                jc_data_pub = 1
            elif jc_data == 'return':
                jc_data_pub = 6

            jc_cmd_pub.publish(jc_data_pub)

            rate.sleep()

    def read_from_jc(self):
        filename = txt_path
        with open(filename, 'r') as f:
            data = f.read()
            data = data.strip('\n')
            return data


if __name__ == '__main__':
    # ROS节点初始化
    rospy.init_node('message', anonymous=True)
    txt_path = rospy.get_param('~txt_filepath','./jc2fk.txt')
    print(txt_path)
    try:
        jc_cmd_pub = rospy.Publisher('/jc_cmd', Int32, queue_size=1)
        message()
    except rospy.ROSInterruptException:
        pass
