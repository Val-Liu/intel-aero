#!/usr/bin/env python2
import cv2
import rospy,roslib
import numpy as np
import time
import sys
import argparse
import os
from cv_bridge import CvBridge,CvBridgeError 
from sensor_msgs.msg import Image 

DIR = 'query/'
print (len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))]))
global c
global query_count
c = 0
query_count = 0

def main():
    rospy.init_node('h264_listener')
    rospy.Subscriber("/camera/rgb/image_raw", Image, call, buff_size = 2**24, queue_size = 1)
    rospy.spin()

def call(msg):
	global c,query_count
	img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
	query_count = query_count + 1
	img1 = cv2.imread('query/' + str(query_count) + '.png',cv2.IMREAD_GRAYSCALE) #queryImage
	lap1 = cv2.Laplacian(img1,cv2.CV_64F)
	lap1 = np.uint8(np.absolute(lap1))
	#cv2.imshow("1",lap1)
	#cv2.imshow("edge",lap1)
	c = c + 1
	cv2.imwrite('image/'+str(c) + '.PNG',img)
	img2 = cv2.imread('image/'+str(c) + '.PNG',cv2.IMREAD_GRAYSCALE) #trainImage
	lap2 = cv2.Laplacian(img2,cv2.CV_64F)
	lap2 = np.uint8(np.absolute(lap2))
	# Initiate SIFT detector
	sift = cv2.xfeatures2d.SIFT_create()
	# find the keypoints and descriptors with SIFT
	kp1, des1 = sift.detectAndCompute(lap1,None)
	kp2, des2 = sift.detectAndCompute(lap2, None)
	# BFMatcher with default params
	bf = cv2.BFMatcher()
	t = rospy.get_time()
	while rospy.get_time() - t <= 2:
		matches = bf.knnMatch(des1,des2,k=2)
		# Apply ratio test
		good = []
		for m,n in matches:
			if m.distance < 0.55*n.distance:
				good.append([m])
	print("threshold: %s" %len(good))
	if len(good) >= 10:
		print("match")
	
				
	else:	
		print("continue")
	


if __name__ == '__main__':
    try:
    	main()
    except:
	pass
    finally:
	cv2.destroyAllWindows()
    
