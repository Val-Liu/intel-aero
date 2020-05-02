#!/usr/bin/env python2
import cv2
import rospy,roslib
import numpy as np
import argparse
import mavros
import time
import sys
from sensor_msgs.msg import NavSatFix
from cv_bridge import CvBridge,CvBridgeError 
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker,MarkerArray


global out ,out2 
fourcc = cv2.VideoWriter_fourcc('X','V','I','D')
out = cv2.VideoWriter('record_usb_cam.avi', fourcc, 20.0, (640, 480),0)
out2 = cv2.VideoWriter('record_realsense.avi', fourcc, 20.0, (640, 480))
def main():
    rospy.init_node('h264_listener')
    rospy.Subscriber("/camera/mono/image_raw", Image, call, buff_size = 2**24, queue_size = 1)
    rospy.Subscriber("/camera/rgb/image_raw", Image, call_realsense, buff_size = 2**24, queue_size = 1)
    rospy.Subscriber("/aruco_single/pose", PoseStamped, pose_cb, queue_size = 10)
    rospy.Subscriber("/aruco_single/marker", Marker, marker_cb, queue_size = 10)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, gpsstate_cb)
    rospy.spin()

def pose_cb(pose):
	print("p %s"%pose.pose.position.x)
	
def marker_cb(mark):
	print("m %s"%mark.pose.position.x)

def gpsstate_cb(gps_state):
	global gps_state_
	gps_state_ = gps_state
	if gps_state_.status == 0:
		print("gps status: %s" %gps_state_.status)	
		print("current latitude: %s"%gps_state_.latitude)
	else:
		print("Gps lost")
		print("gps status: %s" %gps_state_.status)
		print("current latitude: %s"%gps_state_.latitude)
	

def call(msg):
	try:
    		img = CvBridge().imgmsg_to_cv2(msg, "mono8")
		rotated = rotate(img,90)
		out.write(rotated)
		#cv2.imshow('show',rotated)
		cv2.waitKey(1)
    	except CvBridgeError, e:
        	print e
def call_realsense(msg1):
	try:
    		img = CvBridge().imgmsg_to_cv2(msg1, "bgr8")
		out2.write(img)
		#cv2.imshow('show',img)
		cv2.waitKey(1)
    	except CvBridgeError, e:
        	print e

def rotate(image, angle, center=None, scale=1.0):
	(h, w) = image.shape[:2]
	if center is None:
		center = (w / 2, h / 2)
	M = cv2.getRotationMatrix2D(center, angle, scale)
	rotated = cv2.warpAffine(image, M, (w, h))
	return rotated


if __name__ == '__main__':
    try:
    	main()
    except:
	pass
    finally:
	cv2.destroyAllWindows()
