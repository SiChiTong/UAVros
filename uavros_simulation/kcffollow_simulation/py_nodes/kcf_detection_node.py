#!/usr/bin/env python
import numpy as np 
import cv2
import sys
from time import time
import kcftracker
import yaml
import math
import os

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from uavros_msgs.msg import KcfDetectioninfo

selectingObject = False
initTracking = False
onTracking = False
ix, iy, cx, cy = -1, -1, -1, -1
w, h = 0, 0
fx,fy,x0,y0 = 0, 0, 0, 0 #camera matrix arguments

inteval = 1 
duration = 0.01

# mouse callback function
def draw_boundingbox(event, x, y, flags, param):
	global selectingObject, initTracking, onTracking, ix, iy, cx,cy, w, h
	if event == cv2.EVENT_LBUTTONDOWN:
		selectingObject = True
		onTracking = False
		ix, iy = x, y
		cx, cy = x, y
	
	elif event == cv2.EVENT_MOUSEMOVE:
		cx, cy = x, y
	
	elif event == cv2.EVENT_LBUTTONUP:
		selectingObject = False
		if(abs(x-ix)>10 and abs(y-iy)>10):
			w, h = abs(x - ix), abs(y - iy)
			ix, iy = min(x, ix), min(y, iy)
			initTracking = True
		else:
			onTracking = False
	
	elif event == cv2.EVENT_RBUTTONDOWN:
		onTracking = False
		if(w>0):
			ix, iy = x-w//2, y-h//2
			initTracking = True

def image_callback(imgmsg):
	global selectingObject, initTracking, onTracking, ix, iy, cx,cy, w, h
	global fx,fy,x0,y0,duration,inteval
	target_info = KcfDetectioninfo()
	bridge = CvBridge() # Cvbridge can change between ROS image topic and cv frame format!
	try:
		frame = bridge.imgmsg_to_cv2(imgmsg, "bgr8")	
	except CvBridgeError as e: # error reminder
		print(e)

	if(selectingObject):
		cv2.rectangle(frame,(ix,iy), (cx,cy), (0,255,255), 1)
	elif(initTracking):
		cv2.rectangle(frame,(ix,iy), (ix+w,iy+h), (0,255,255), 2)

		tracker.init([ix,iy,w,h], frame)

		initTracking = False
		onTracking = True		
	elif(onTracking):
		t0 = time()
		boundingbox = tracker.update(frame)
		t1 = time()

		boundingbox = list(map(int, boundingbox))
		cv2.rectangle(frame,(boundingbox[0],boundingbox[1]), (boundingbox[0]+boundingbox[2],boundingbox[1]+boundingbox[3]), (0,255,255), 1)
		duration = 0.8*duration + 0.2*(t1-t0)
		#duration = t1-t0
		cv2.putText(frame, 'FPS: '+str(1/duration)[:4].strip('.'), (8,20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
		centerx = float(boundingbox[0]) + float(boundingbox[2]) / 2
		centery = float(boundingbox[1]) + float(boundingbox[3]) / 2
		height = float(boundingbox[3])
		width = float(boundingbox[2])
		target_info.angle_x = math.atan((centerx-x0)/fx)*180/math.pi #degree
		target_info.angle_y = math.atan((centery-y0)/fy)*180/math.pi #degree
		target_info.angle_h = math.atan(height/fy)*180/math.pi #degree
		target_info.angle_w = math.atan(width/fx)*180/math.pi #degree
	target_info.detected = onTracking
	pub.publish(target_info) # publish the target information
	cv2.imshow('tracking', frame) #"imshow" must be followed by "waitkey"!!!
	c = cv2.waitKey(inteval)& 0xFF #inteval is manually set
	if c==27 or c==ord('q'):
		cv2.destroyAllWindows() 
		# however,in the next image_callback loop,windows shall appear again


def main():
	## some initialization set up:
	global selectingObject, initTracking, onTracking, ix, iy, cx,cy, w, h
	global fx,fy,x0,y0,duration,tracker,pub

	rospy.init_node('kcf_detection_node', anonymous=True)
	pub = rospy.Publisher('/kcfdetection/target_info',KcfDetectioninfo,queue_size=10)

	image_topic_name = rospy.get_param('~subscriber', '/usb_cam/image_raw')
	rospy.Subscriber(image_topic_name, Image, image_callback)

	config = rospy.get_param('~config', 'camera_param.yaml')	
	yaml_config_fn = os.path.dirname(os.path.abspath(__file__)) + '/../config/' + config
	yaml_config = yaml.load(open(yaml_config_fn))	
	print('Input config file: {}'.format(config))
	fx = yaml_config['fx']
	fy = yaml_config['fy']
	x0 = yaml_config['x0']
	y0 = yaml_config['y0']
	print('fx:',fx)
	print('fy:',fy)
	print('x0:',x0)
	print('y0:',y0)

	tracker = kcftracker.KCFTracker(True, True, True)  # hog, fixed_window, multiscale
	#if you use hog feature, there will be a short pause after you draw a first boundingbox, that is due to the use of Numba.

	cv2.namedWindow('tracking')
	cv2.setMouseCallback('tracking',draw_boundingbox)

	rospy.spin() # rospy.spin() just serves to keep the node from shutting down
	# there is no "rospy.spinonce()" in rospy!

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
	cv2.destroyAllWindows()


