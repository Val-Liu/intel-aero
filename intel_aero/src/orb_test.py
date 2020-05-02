#!/usr/bin/env python2
import cv2
import rospy,roslib
import time
import sys
import math
from math import *
from cv_bridge import CvBridge,CvBridgeError 
from sensor_msgs.msg import Image,PointCloud2
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2

global dist
dist = 5
def main():
    rospy.init_node('h264_listener2')
    rospy.Subscriber("/camera/rgb/image_raw", Image, call, buff_size = 2**24, queue_size = 1)
    rospy.Subscriber("/orb_slam2_mono/pose", PoseStamped, orb_pose)
    #rospy.Subscriber("/orb_slam2_mono/map_points", PointCloud2, pointcloud_cb)
    rospy.spin()

def orb_pose(pose):
	global dist
	posx = pose.pose.position.x
	posy = pose.pose.position.y
	posz = pose.pose.position.z
	print("d: %s"%dist)
	pos_dist = 0.26 * dist
	print("x: %s"%posx)
	if math.fabs(pos_dist - posx) >= 0.001:
		print("dist: %s"%math.fabs(pos_dist - posx))
		pass 
	else:
		print("achieve")
	dist = dist - 1
	
	
'''def pointcloud_cb(data):
	assert isinstance(data,PointCloud2)
	gen = point_cloud2.read_points(data)
	time.sleep(1)
	print(type(gen))
	lst = list(gen)
	for p in lst:
		print("x: %.3f y: %.3f z: %.3f" %(p[0],p[1],p[2]))'''

def call(msg):
	try:
    		img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
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
