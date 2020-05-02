#!/usr/bin/env python2
import cv2
import rospy,roslib
import numpy as np
import time
import sys
from cv_bridge import CvBridge,CvBridgeError 
from sensor_msgs.msg import Image

global out
fourcc = cv2.VideoWriter_fourcc('X','V','I','D')
out = cv2.VideoWriter('record.avi', fourcc, 30.0, (480, 360))
def depth_callback(msg):
	depth_image = CvBridge().imgmsg_to_cv2(msg, "32FC1")
	depth_array = np.array(depth_image, dtype=np.float32)
	cv_image_norm = cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
	cv_image_resized = cv2.resize(cv_image_norm, (480,360), interpolation = cv2.INTER_CUBIC)
	u = (msg.width)/2
	v = (msg.height)/2
	centerIdx = u + msg.width * v
	rospy.loginfo("Center distance : %g m", depth_array[180,240])
	#out.write(np.uint8(cv_image_norm))
	cv2.imshow('show',cv_image_resized)
	cv2.waitKey(1)

def main():
	rospy.init_node('depth_image')
	depth_sub = rospy.Subscriber("camera/depth/image_raw",Image,depth_callback)
	#rospy.Subscriber("/camera/rgb/image_raw", Image, call, buff_size = 2**24, queue_size = 1)
	rospy.spin()
if __name__ == '__main__':
	main()
