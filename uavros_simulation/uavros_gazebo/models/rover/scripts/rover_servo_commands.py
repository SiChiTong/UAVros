#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
import math

flag_move = 0

def set_throttle_steer(data):

    global flag_move
    wheel_radius = 0.3175
    #wheel_base = 2.196 #distance between front and rear
    wheel_base = 2.054 #distance between front and rear
    wheel_dist = 1.43124 #distance between left and right 

    pub_vel_left_rear_wheel = rospy.Publisher('left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('right_steering_hinge_position_controller/command', Float64, queue_size=1)

    #throttle = data.drive.speed*13.95348
    # this geometry analysis is by spx
    v = data.drive.speed
    steer = data.drive.steering_angle
    w_rl = v*(1-wheel_dist*math.tan(steer)/2/wheel_base)/wheel_radius #rear left
    w_rr = v*(1+wheel_dist*math.tan(steer)/2/wheel_base)/wheel_radius #rear right
    w_fl = v*math.sqrt(math.pow((1-wheel_dist*math.tan(steer)/2/wheel_base),2)+math.pow(math.tan(steer),2))/wheel_radius #front left
    w_fr = v*math.sqrt(math.pow((1+wheel_dist*math.tan(steer)/2/wheel_base),2)+math.pow(math.tan(steer),2))/wheel_radius #front right
    steer_fl = math.atan2(math.tan(steer),1-wheel_dist*math.tan(steer)/2/wheel_base)
    steer_fr = math.atan2(math.tan(steer),1+wheel_dist*math.tan(steer)/2/wheel_base)

    pub_vel_left_rear_wheel.publish(w_rl)
    pub_vel_right_rear_wheel.publish(w_rr)
    pub_vel_left_front_wheel.publish(w_fl)
    pub_vel_right_front_wheel.publish(w_fr)
    pub_pos_left_steering_hinge.publish(steer_fl)
    pub_pos_right_steering_hinge.publish(steer_fr)

def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)

    rospy.Subscriber("ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
