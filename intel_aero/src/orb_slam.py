#!/usr/bin/env python2
import cv2
import rospy,roslib
import time
import sys
import math
from cv_bridge import CvBridge,CvBridgeError 
from sensor_msgs.msg import Image,PointCloud2
from geometry_msgs.msg import PoseStamped
from sensor_msgs import point_cloud2

global out
fourcc = cv2.VideoWriter_fourcc('X','V','I','D')
out = cv2.VideoWriter('record.avi', fourcc, 30.0, (640, 480))
global scale
scale = 0.004
pose_ = PoseStamped()
def main():
    rospy.init_node('h264_listener2')
    rospy.Subscriber("/camera/rgb/image_raw", Image, call, buff_size = 2**24, queue_size = 1)
    rospy.Subscriber("/orb_slam2_mono/pose", PoseStamped, orb_pose)
    rospy.Subscriber("/orb_slam2_mono/map_points", PointCloud2, pointcloud_cb)
    #local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
    rospy.spin()

def orb_pose(pose):
	pose_ = pose
	#print("z : %s" %pose.pose.position.z) 
	pass
def pointcloud_cb(data):
	pose_x = pose_.pose.position.x
	pose_y = pose_.pose.position.y
	pose_z = pose_.pose.position.z
	assert isinstance(data,PointCloud2)
	gen = point_cloud2.read_points(data,field_names=("x","y","z"),skip_nans=True)
	time.sleep(1)
	lst = list(gen)
	print(len(lst))
	total_points = [(lst[i][0],lst[i][1],lst[i][2]) for i in range(len(lst)) if math.fabs(lst[i][0] - pose_x) <= scale*1.5 and math.fabs(lst[i][1] - pose_y) <= scale*1.5 and math.fabs(lst[i][2] - pose_z) <= scale*1.5] 
	print(len(total_points))
	if len(total_points) >= 150:
		print("True")
	else:
		print("False")
	
	
			

def call(msg):
	try:
    		img = CvBridge().imgmsg_to_cv2(msg, "bgr8")
		out.write(img)
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
