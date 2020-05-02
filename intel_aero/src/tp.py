#!/usr/bin/env python2
import cv2
import rospy,roslib
import numpy as np
import time
import sys
from cv_bridge import CvBridge,CvBridgeError 
from sensor_msgs.msg import Image



global c
c = 1
def main():
    rospy.init_node('h264_listener')
    rospy.Subscriber("/camera/mono/image_raw", Image, call, buff_size = 2**24, queue_size = 1)
    rospy.spin()
def call(msg):
	try:
		global c
		time_f = 100
    		img = CvBridge().imgmsg_to_cv2(msg, "mono8")
		#print("img :%s" %img)
		if (c % time_f == 0):
			cv2.imwrite('image/'+str(c) + '.PNG',img)
		c = c+1
		#cv2.imshow('show',img)
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
    


