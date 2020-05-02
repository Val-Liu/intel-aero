#!/usr/bin/env python
import rospy,roslib
import time,sys
import mavros
import math
import geopy
import cv2
import numpy as np
from cv_bridge import CvBridge,CvBridgeError 
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker,MarkerArray
from math import *
from mavros.utils import *
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Vector3, Pose
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import HomePosition, GlobalPositionTarget, PositionTarget , Altitude
from sensor_msgs.msg import NavSatFix
from math import sin, cos, atan2, degrees, acos, radians
from geopy.distance import vincenty
from mavros_msgs.msg import State
from std_msgs.msg import Float64
from behave import *

rospy.init_node('mavros_takeoff_python')
mavros.set_namespace()
global px,py,pz
px = 0
py = 0
pz = 0
rate = rospy.Rate(20)
pose_ = PoseStamped()
global out
fourcc = cv2.VideoWriter_fourcc('X','V','I','D')
out = cv2.VideoWriter('usb_cam.avi', fourcc, 20.0, (640, 480),0)
home_ = HomePosition()
def setHomeGeoPointCB(home):
		home_= home
		home_set_ = True
		rospy.loginfo("Received Home (WGS84 datum): %lf, %lf, %lf" % (home_.geo.latitude, home_.geo.longitude, home_.geo.altitude))
home_sub = rospy.Subscriber("/mavros/home_position/home",HomePosition,setHomeGeoPointCB)
def ALT_HOLD():
  print "\nchange_mode"
  rospy.wait_for_service('/mavros/set_mode')
  try:
      change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
      response = change_mode(custom_mode="ALT_HOLD")
      rospy.loginfo(response)
  except rospy.ServiceException as e:
      print("Set mode failed: %s" %e)

# Arm
def Arm():
  print "\nArming"
  rospy.wait_for_service('/mavros/cmd/arming')
  try:
      arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
      response = arming_cl(value = True)
      rospy.loginfo(response)
  except rospy.ServiceException as e:
      print("Arming failed: %s" %e)



def guide():
  rospy.wait_for_service('/mavros/set_mode')
  try:
      change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
      response = change_mode(custom_mode="GUIDED")
      #rospy.loginfo(response)
  except rospy.ServiceException as e:
      print("Set mode failed: %s" %e)

# Takeoff
def Takeoff():
  print "\nTaking off"
  rospy.wait_for_service('/mavros/cmd/takeoff')
  try:
      takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
      response = takeoff_cl(altitude=10, latitude = home_.geo.latitude, longitude = home_.geo.longitude, min_pitch=0, yaw=0)
      rospy.loginfo(response)
  except rospy.ServiceException as e:
      print("Takeoff failed: %s" %e)

'''pose = PoseStamped()
pose.pose.position.x = 0
pose.pose.position.y = 0
pose.pose.position.z = 10'''
def pose_cb(pose):
	#print("x %s"%pose.pose.position.x)
	#print("y %s"%pose.pose.position.y)
	global pose_
	pose_ = pose
	px = pose_.pose.position.x
	#print("%s"%px)
	py = pose_.pose.position.y
	pz = pose_.pose.position.z
def marker_cb(mark):
	#print("m %s"%mark.id)
	a = mark.id
def call(msg):
	try:
    		img = CvBridge().imgmsg_to_cv2(msg, "mono8")
		rotated = rotate(img,90)
		out.write(rotated)
		#cv2.imshow('show',rotated)
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

def pose_detect():
    	rospy.Subscriber("/camera/mono/image_raw", Image, call, buff_size = 2**24, queue_size = 1)
    	rospy.Subscriber("/aruco_single/pose", PoseStamped, pose_cb, queue_size = 10)
    	rospy.Subscriber("/aruco_single/marker", Marker, marker_cb, queue_size = 10)
	d_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
	t1 = rospy.get_time()
	while rospy.get_time() - t1 <= 5 :
		pass
	current_px = pose_.pose.position.x
	current_py = pose_.pose.position.y
	current_pz = pose_.pose.position.z
	print("x %s"%current_px)
	while math.fabs(current_px * 100) >= 1 or math.fabs(current_py * 100) >= 1:
		msg = TwistStamped()
		if current_px * 100 > 0 :
			msg.twist.linear.y = 1
		elif current_py * 100 > 0 :
			msg.twist.linear.x = 1
		elif current_px * 100 < 0 :
			msg.twist.linear.y = -1
		elif current_py * 100 < 0:
			msg.twist.linear.x = -1
		elif current_px * 100 < 0 and current_py * 100 < 0:
			msg.twist.linear.y = -1
			msg.twist.linear.x = -1
		elif current_px * 100 < 0 and current_py * 100 > 0:
			msg.twist.linear.y = -1
			msg.twist.linear.x = 1
		elif current_px * 100 > 0 and current_py * 100 > 0:
			msg.twist.linear.y = 1
			msg.twist.linear.x = 1
		elif current_px * 100 > 0 and current_py * 100 < 0:
			msg.twist.linear.y = 1
			msg.twist.linear.x = -1
		d_pub.publish(msg)
		current_px = pose_.pose.position.x
		current_py = pose_.pose.position.y
		current_pz = pose_.pose.position.z
	print("correct")
	
	
 	
	

# Land
def Land():
  print "\nLanding"
  rospy.wait_for_service('/mavros/cmd/land')
  try:
      takeoff_cl = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
      response = takeoff_cl(altitude=10, latitude=home_.geo.latitude, longitude=home_.geo.longitude, min_pitch=0, yaw=0)
      rospy.loginfo(response)
  except rospy.ServiceException as e:
      print("Landing failed: %s" %e)
  #time.sleep(10)

# Disarm
def Disarm():
  print "\nDisarming"
  rospy.wait_for_service('/mavros/cmd/arming')
  try:
      arming_cl = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
      response = arming_cl(value = False)
      rospy.loginfo(response)
  except rospy.ServiceException as e:
      print("Disarming failed: %s" %e)



current_state = State() 
offb_set_mode = SetMode
def state_cb(state):
    global current_state
    current_state = state

local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
state_sub = rospy.Subscriber(mavros.get_topic('state'), State, state_cb)
arming_client = rospy.ServiceProxy(mavros.get_topic('cmd', 'arming'), CommandBool)
set_mode_client = rospy.ServiceProxy(mavros.get_topic('set_mode'), SetMode)

def check_mode():
	if current_state == "RTL":
		guide()
		
if __name__ == '__main__':
	try:
		ALT_HOLD()
		Arm()
		time.sleep(3)
		guide()
		Takeoff()
		time.sleep(5)
		check_mode()
		pose_detect()
		Land()
		time.sleep(3)
		Disarm()
	except rospy.ROSInterruptException:
		pass
