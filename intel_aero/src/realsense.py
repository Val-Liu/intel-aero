#!/usr/bin/env python2
import cv2
import rospy,roslib
import time
import sys
from cv_bridge import CvBridge,CvBridgeError 
from sensor_msgs.msg import Image


global out
fourcc = cv2.VideoWriter_fourcc('X','V','I','D')
out = cv2.VideoWriter('record.avi', fourcc, 20.0, (640, 480))
def main():
    rospy.init_node('h264_listener2')
    rospy.Subscriber("/camera/rgb/image_raw", Image, call, buff_size = 2**24, queue_size = 1)
    rospy.spin()

def call(msg):
	try:
    		img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
		out.write(img)
		cv2.imshow('show',img)
		cv2.waitKey(1)
    	except CvBridgeError, e:
        	print e

if __name__ == '__main__':
    try:
    	main()
    except:
	pass
    finally:
	cv2.destroyAllWindows()
