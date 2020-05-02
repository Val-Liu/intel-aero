#!/usr/bin/env python
import rospy,roslib
import time,sys
import mavros
import thread
import threading
import math
import geopy
import cv2
import numpy as np
from cv_bridge import CvBridge,CvBridgeError
from sensor_msgs.msg import Image,PointCloud2
from sensor_msgs import point_cloud2
from math import *
from mavros.utils import *
from mavros import setpoint as SP
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist, Vector3, Pose
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from mavros_msgs.msg import HomePosition, GlobalPositionTarget, PositionTarget , Altitude
from sensor_msgs.msg import NavSatFix
from math import sin, cos, atan2, degrees, acos, radians
from geopy.distance import vincenty
from mavros_msgs.msg import State
from std_msgs.msg import Float64
from behave import *

home_ = HomePosition()
def setHomeGeoPointCB(home):
	home_= home
	home_set_ = True
	rospy.loginfo("Received Home (WGS84 datum): %lf, %lf, %lf" % (home_.geo.latitude, home_.geo.longitude, home_.geo.altitude))
home_sub = rospy.Subscriber("/mavros/home_position/home",HomePosition,setHomeGeoPointCB)
def ALT_HOLD():
	print "\nALT_HOLD"
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


class bt_missions:
	rospy.init_node('aero_pose_python')
	mavros.set_namespace()
	rate = rospy.Rate(20)
	orb_ = PoseStamped()
	dirct_ = Float64()
	points_ = PointCloud2()
	home_ = HomePosition()
	current_state = State()
	current_ = NavSatFix()
	alt_ = Float64()
	fourcc = cv2.VideoWriter_fourcc('X','V','I','D')
	out = cv2.VideoWriter('realsense.avi', fourcc, 20.0, (640, 480))
	home_set_ = False
	isContinue = True
	orb_scale = False
	target = [[24.982550663278158,121.57233949235275],[24.982091,121.571846],[24.981758,121.571963]]
	index = 0
	current_posx = 0
	dist_first = True
	start_heading = 0
	p0 = 0
	p1 = 0
	brng = 0
	dLon = 0
	y = 0
	x = 0 
	current_yaw = 0
	current_alt = 0
	dist = 0
	pose_dist = 0
	heading = 0
	lat = 0
	lon = 0
	cnt = 0
	alt_count = 0
	alt_count2 = 0
	ah_alt = 0
	z1 = 0
	z2 = 0
	z3 = 0
	z4 = 0
	scale = 0
	ah_flag=False
	img_count = 0
	query_count = 0
	pose_flag = True
	move_flag = False
	record_img = np.array([])
	set_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
	set_v_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget,queue_size=10)
	local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)

	def __init__(self):
		self.orb_sub = rospy.Subscriber("/orb_slam2_mono/pose", PoseStamped, self.orb_pose)
		self.hdg_sub = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64,self.compass)
		self.rel_alt= rospy.Subscriber("/mavros/global_position/rel_alt", Float64,self.altitude)
		self.home_sub = rospy.Subscriber("/mavros/home_position/home",HomePosition,self.setHomeGeoPointCB)
		self.gps_sub = rospy.Subscriber("/mavros/global_position/global",NavSatFix,self.current_position_cb)
		self.state_sub = rospy.Subscriber(mavros.get_topic('state'), State, self.state_cb)
		self.img_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.call, buff_size = 2**24, queue_size = 1)
		self.point_sub = rospy.Subscriber("/orb_slam2_mono/map_points", PointCloud2, self.pointcloud_cb)
		#self.mono_sub = rospy.Subscriber("/camera/mono/image_raw", Image, self.mono_cb, buff_size = 2**24, queue_size = 1)
		self.tree = self.correction
#(self.achieve | ((self.achieve_alt | self.up) >> (self.yaw_correction | self.correction) >> (self.get_scale | self.up_scale) >> self.forward)) >> (self.ah | (self.isdetect >> self.change_alt))
		


	def orb_pose(self, pose):
		bt_missions.orb_ = pose

	def pointcloud_cb(self, points):
		bt_missions.points_ = points

	def compass(self, dirct):
		bt_missions.dirct_ = dirct
		bt_missions.heading = bt_missions.dirct_.data
	
	def state_cb(self, state):
		bt_missions.current_state = State()
		bt_missions.current_state = state

	def altitude(self, alt):
		bt_missions.alt_ = alt
		bt_missions.current_alt = bt_missions.alt_.data
		rospy.loginfo("Received current altitude : %s" %bt_missions.current_alt)

	def setHomeGeoPointCB(self,home):
		bt_missions.home_= home
		bt_missions.home_set_ = True
		rospy.loginfo("Received Home (WGS84 datum): %lf, %lf, %lf" % (bt_missions.home_.geo.latitude, bt_missions.home_.geo.longitude, bt_missions.home_.geo.altitude))

	def current_position_cb(self,current):
		bt_missions.current_  = current
		bt_missions.lat = bt_missions.current_.latitude
		bt_missions.lon = bt_missions.current_.longitude
		rospy.loginfo("Received current GPS : %lf, %lf" % (bt_missions.lat, bt_missions.lon))
	
	def call(self,callimage):
    		bt_missions.record_img = CvBridge().imgmsg_to_cv2(callimage, "bgr8")
		bt_missions.out.write(bt_missions.record_img)
	

	@condition
	def ah(self):
		if bt_missions.ah_flag == True:
			return False
		else:
			return True

	@condition
	def isdetect(self):
		bt_missions.query_count = bt_missions.query_count + 1
		img1 = cv2.imread('query/' + str(bt_missions.query_count) + '.PNG',cv2.IMREAD_GRAYSCALE) #queryImage
		bt_missions.img_count = bt_missions.img_count + 1
		cv2.imwrite('image/'+str(bt_missions.img_count) + '.PNG',bt_missions.record_img)
		img2 = cv2.imread('image/'+str(bt_missions.img_count) + '.PNG',cv2.IMREAD_GRAYSCALE) #trainImage
		# Initiate SIFT detector
		sift = cv2.xfeatures2d.SIFT_create()
		# find the keypoints and descriptors with SIFT
		kp1, des1 = sift.detectAndCompute(img1, None)
		kp2, des2 = sift.detectAndCompute(img2, None)
		# BFMatcher with default params
		bf = cv2.BFMatcher()
		t = rospy.get_time()
		while (rospy.get_time() - t <= 2):
			matches = bf.knnMatch(des1,des2,k=2)
			# Apply ratio test
			good = []
			for m,n in matches:
				if m.distance < 0.55*n.distance:
					good.append([m])
		print("threshold: %s" %len(good))
		if len(good) >= 20:
			bt_missions.index = bt_missions.index + 1
			bt_missions.img_count = 0
			return True
				
		else:
			return False

	@condition
	def lose_tracking(self):
		if bt_missions.orb_.pose.position.z == 0:
			return False
		else:
			return True

	@condition
	def get_scale(self):
		if bt_missions.orb_scale == True :
			return True
		else:
			return False

	@condition
	def achieve_alt(self):
		if bt_missions.current_alt > 15:
			print("altitude enough")
			return True
		else:
			print("altitude not enough")
			return False

	
		

	@condition
	def achieve(self):
		if bt_missions.index == 0 and bt_missions.dist_first == True:
			bt_missions.dist_first = False
			#GPS distance
			bt_missions.p0 =(bt_missions.home_.geo.latitude,bt_missions.home_.geo.longitude)
			bt_missions.p1 = (bt_missions.target[bt_missions.index][0],bt_missions.target[bt_missions.index][1])
			bt_missions.dist = vincenty(bt_missions.p0,bt_missions.p1).meters
			print("current Gps dist: %s" %bt_missions.dist)
			#Pose distance
			bt_missions.pose_dist = bt_missions.scale * bt_missions.dist
			print("current pose dist: %s" %bt_missions.pose_dist)
			
			if bt_missions.pose_dist >= 0.02: 
				
				return False
			elif bt_missions.pose_dist == 0:
				return False
			else:
				bt_missions.index = bt_missions.index + 1
		elif bt_missions.index == 0 and bt_missions.dist_first == False:
			bt_missions.p0 = (bt_missions.current_.latitude, bt_missions.current_.longitude)
			bt_missions.p1 = (bt_missions.target[bt_missions.index][0],bt_missions.target[bt_missions.index][1])
			bt_missions.dist = vincenty(bt_missions.p0,bt_missions.p1).meters
			print("current Gps dist: %s" %bt_missions.dist)
			#Pose distance
			bt_missions.pose_dist = bt_missions.scale * bt_missions.dist
			print("current pose dist: %s" %bt_missions.pose_dist)

			if bt_missions.pose_dist >= 0.02:
				return False
			elif bt_missions.pose_dist == 0:
				return False
			else:
				print("achieve first point")
				bt_missions.move_flag = False
				rospy.loginfo("first point's GPS : %lf, %lf, %lf" % (bt_missions.lat, bt_missions.lon, bt_missions.current_alt))
				bt_missions.index = bt_missions.index + 1
				return True
		elif bt_missions.index == 1 and bt_missions.dist_first == False:
			bt_missions.p0 = (bt_missions.current_.latitude, bt_missions.current_.longitude)
			bt_missions.p1 = (bt_missions.target[bt_missions.index][0],bt_missions.target[bt_missions.index][1])
			bt_missions.dist = vincenty(bt_missions.p0,bt_missions.p1).meters
			print("current Gps dist: %s" %bt_missions.dist)
			#Pose distance
			bt_missions.pose_dist = bt_missions.scale * bt_missions.dist
			print("current pose dist: %s" %bt_missions.pose_dist)
			
			if bt_missions.pose_dist >= 0.02:
				return False
			else:
				print("achieve key point")
				bt_missions.move_flag = False
				bt_missions.ah_flag = True
				rospy.loginfo("keypoint point's GPS : %lf, %lf, %lf" % (bt_missions.lat, bt_missions.lon, bt_missions.current_alt))
				return True


	@condition
	def yaw_correction(self):
		if bt_missions.index == 0:
			bt_missions.dLon = (bt_missions.target[bt_missions.index][1] - bt_missions.home_.geo.longitude)
			bt_missions.y = math.sin(bt_missions.dLon) * math.cos(bt_missions.target[bt_missions.index][0])
			bt_missions.x = math.cos(bt_missions.home_.geo.latitude) * math.sin(bt_missions.target[bt_missions.index][0]) - math.sin(bt_missions.home_.geo.latitude) * cos(bt_missions.target[bt_missions.index][0]) * math.cos(bt_missions.dLon)
			bt_missions.brng = math.atan2(bt_missions.y,bt_missions.x)
			bt_missions.brng = math.degrees(bt_missions.brng)
			bt_missions.brng = (bt_missions.brng + 360) % 360
			bt_missions.brng = 360 - bt_missions.brng
			bt_missions.brng = math.radians(bt_missions.brng)
			bt_missions.brng = math.degrees(bt_missions.brng)
			bt_missions.current_yaw = bt_missions.heading
			print("correction heading: %s" %bt_missions.current_yaw)
                        print("correction brng: %s" %bt_missions.brng)
			if bt_missions.current_yaw >(360 - bt_missions.brng) + 2.0 or bt_missions.current_yaw <(360 - bt_missions.brng) -2.0:
				return False
			else:
				return True
		else:
			bt_missions.dLon = (bt_missions.target[bt_missions.index][1] - bt_missions.current_.longitude)
			bt_missions.y = math.sin(bt_missions.dLon) * math.cos(bt_missions.target[bt_missions.index][0])
			bt_missions.x = math.cos(bt_missions.current_.latitude) * math.sin(bt_missions.target[bt_missions.index][0]) - math.sin(bt_missions.current_.latitude) * cos(bt_missions.target[bt_missions.index][0]) * math.cos(bt_missions.dLon)
			bt_missions.brng = math.atan2(bt_missions.y,bt_missions.x)
			bt_missions.brng = math.degrees(bt_missions.brng)
			bt_missions.brng = (bt_missions.brng + 360) % 360
			bt_missions.brng = 360 - bt_missions.brng
			bt_missions.brng = math.radians(bt_missions.brng)
			bt_missions.brng = math.degrees(bt_missions.brng)
			bt_missions.current_yaw = bt_missions.heading
			print("correction heading: %s" %bt_missions.current_yaw)
                        print("correction brng: %s" %bt_missions.brng)
			if bt_missions.current_yaw >(360 - bt_missions.brng) + 2.0 or bt_missions.current_yaw <(360 - bt_missions.brng) -2.0:
				return False
			else:
				return True 
	
	@action
	def up_scale(self):
		bt_missions.start_heading = bt_missions.heading
		msg = TwistStamped()
		msg.header.stamp = rospy.Time.now()
		msg.twist.angular.x = 0.0
		msg.twist.linear.x = 0.0
		msg.twist.angular.y = 0.0
		msg.twist.linear.y = 0.0
		msg.twist.angular.z = 0.0
		msg.twist.linear.z = 0.0
		t = rospy.get_time()
		while rospy.get_time() - t <= 2:
			change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                  	response = change_mode(custom_mode="GUIDED")
			bt_missions.set_vel_pub.publish(msg)
			bt_missions.rate.sleep()
		bt_missions.alt_count = bt_missions.current_alt
		bt_missions.z1 = bt_missions.orb_.pose.position.z
		print("pose z1 :%s" %bt_missions.z1)
		msg = TwistStamped()
		msg.header.stamp = rospy.Time.now()
		msg.twist.angular.z = 0.0 
		msg.twist.linear.z = 0.5
		while bt_missions.current_alt - bt_missions.alt_count <= 5:
			change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                  	response = change_mode(custom_mode="GUIDED")
			bt_missions.set_vel_pub.publish(msg)
			bt_missions.rate.sleep()

		msg = TwistStamped()
		msg.header.stamp = rospy.Time.now()
		msg.twist.angular.x = 0.0
		msg.twist.linear.x = 0.0
		msg.twist.angular.y = 0.0
		msg.twist.linear.y = 0.0
		msg.twist.angular.z = 0.0
		msg.twist.linear.z = 0.0
		t = rospy.get_time()
		while rospy.get_time() - t <= 2 :
			change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                  	response = change_mode(custom_mode="GUIDED")
			bt_missions.set_vel_pub.publish(msg)
			bt_missions.rate.sleep()
		bt_missions.alt_count2 = bt_missions.current_alt
		print("current altitude : %s" %bt_missions.alt_count2)
		bt_missions.z2 = bt_missions.orb_.pose.position.z
		print("pose z2 :%s" %bt_missions.z2)
		bt_missions.scale = math.fabs(bt_missions.z2 - bt_missions.z1) /(bt_missions.alt_count2 - bt_missions.alt_count)
		print("scale one: %s" %bt_missions.scale)
		bt_missions.orb_scale = True
		

	@action 
	def correction(self):
		#orb lost tracking
		if bt_missions.orb_.pose.position.z == 0 and bt_missions.orb_.pose.position.y == 0 and bt_missions.orb_.pose.position.x == 0:
			bt_missions.current_yaw = bt_missions.heading
			
		msg = TwistStamped()
		msg.header.stamp = rospy.Time.now()
		msg.twist.linear.z = 0
		bt_missions.current_yaw = bt_missions.heading
		if 360 - bt_missions.brng - bt_missions.current_yaw > 0 and 360 - bt_missions.brng - bt_missions.current_yaw < 180 :
			msg.twist.angular.z = -0.1
		elif 360 - bt_missions.brng - bt_missions.current_yaw > 0 and 360 - bt_missions.brng - bt_missions.current_yaw >= 180:  
			msg.twist.angular.z = 0.1
		elif 360 - bt_missions.brng - bt_missions.current_yaw < 0 and math.fabs(360 - bt_missions.brng - bt_missions.current_yaw) >= 180:
			msg.twist.angular.z = -0.1
		elif 360 - bt_missions.brng - bt_missions.current_yaw < 0 and math.fabs(360 - bt_missions.brng - bt_missions.current_yaw) < 180:
			msg.twist.angular.z = 0.1
		bt_missions.current_yaw = bt_missions.heading
		while (bt_missions.current_yaw >(360 - bt_missions.brng) + 2.0 or bt_missions.current_yaw <(360 - bt_missions.brng) -2.0) and (bt_missions.orb_.pose.position.z != 0 and bt_missions.orb_.pose.position.y != 0 and bt_missions.orb_.pose.position.x != 0):
			change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)	
			response = change_mode(custom_mode="GUIDED")
			bt_missions.set_vel_pub.publish(msg)
			bt_missions.current_yaw = bt_missions.heading
			bt_missions.rate.sleep()
			print("first correction heading : %s" %bt_missions.current_yaw)
		
		if bt_missions.orb_.pose.position.z == 0 and bt_missions.orb_.pose.position.y == 0 and bt_missions.orb_.pose.position.x == 0:
			print("orb lost tracking")
			bt_missions.current_yaw = bt_missions.heading
			bt_missions.dLon = (bt_missions.target[bt_missions.index-1][1] - bt_missions.current_.longitude)
			bt_missions.y = math.sin(bt_missions.dLon) * math.cos(bt_missions.target[bt_missions.index-1][0])
			bt_missions.x = math.cos(bt_missions.current_.latitude) * math.sin(bt_missions.target[bt_missions.index-1][0]) - math.sin(bt_missions.current_.latitude) * cos(bt_missions.target[bt_missions.index][0]) * math.cos(bt_missions.dLon)
			bt_missions.brng = math.atan2(bt_missions.y,bt_missions.x)
			bt_missions.brng = math.degrees(bt_missions.brng)
			bt_missions.brng = (bt_missions.brng + 360) % 360
			bt_missions.brng = 360 - bt_missions.brng
			bt_missions.brng = math.radians(bt_missions.brng)
			bt_missions.brng = math.degrees(bt_missions.brng)

			msg = TwistStamped()
			msg.header.stamp = rospy.Time.now()
			msg.twist.linear.z = 0
			bt_missions.current_yaw = bt_missions.heading
			if 360 - bt_missions.brng - bt_missions.current_yaw > 0 and 360 - bt_missions.brng - bt_missions.current_yaw < 180 :
				msg.twist.angular.z = -0.1
			elif 360 - bt_missions.brng - bt_missions.current_yaw > 0 and 360 - bt_missions.brng - bt_missions.current_yaw >= 180:  
				msg.twist.angular.z = 0.1
			elif 360 - bt_missions.brng - bt_missions.current_yaw < 0 and math.fabs(360 - bt_missions.brng - bt_missions.current_yaw) >= 180:
				msg.twist.angular.z = -0.1
			elif 360 - bt_missions.brng - bt_missions.current_yaw < 0 and math.fabs(360 - bt_missions.brng - bt_missions.current_yaw) < 180:
				msg.twist.angular.z = 0.1
			bt_missions.current_yaw = bt_missions.heading
			while bt_missions.current_yaw >(360 - bt_missions.brng) + 2.0 or bt_missions.current_yaw <(360 - bt_missions.brng) -2.0:
				change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)	
				response = change_mode(custom_mode="GUIDED")
				bt_missions.set_vel_pub.publish(msg)
				bt_missions.current_yaw = bt_missions.heading
				bt_missions.rate.sleep()
				print("orb correction heading : %s" %bt_missions.current_yaw)

	@action
	def up(self):
		msg = TwistStamped()
		msg.header.stamp = rospy.Time.now()
		msg.twist.linear.z = 0.8
		while bt_missions.current_alt <= 15:
			change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                  	change_mode(custom_mode="GUIDED")
			bt_missions.set_vel_pub.publish(msg)
			bt_missions.rate.sleep()
			print("current altitude : %s" %bt_missions.current_alt)

	@action
	def forward(self):	
		bt_missions.move_flag = True
		#body_frame
		msg = TwistStamped()
		msg.header.stamp = rospy.Time.now()
		msg.twist.angular.z = 0.0
		print("forward")
		vel = PositionTarget()
		vel.header.stamp = rospy.Time.now()
		vel.coordinate_frame = PositionTarget.FRAME_BODY_NED
		vel.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.FORCE + PositionTarget.IGNORE_YAW +PositionTarget.IGNORE_YAW_RATE
		vel.velocity.x = 0.0
		vel.velocity.y = 1.0
		vel.velocity.z = 0.0
		forward_t = rospy.get_time()
		while rospy.get_time() - forward_t <= bt_missions.dist*0.1:
		  	change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		  	response = change_mode(custom_mode="GUIDED")
			bt_missions.set_vel_pub.publish(msg)
			bt_missions.set_v_pub.publish(vel)
			bt_missions.rate.sleep()
		
		
	@action
	def change_alt(self):
		msg = TwistStamped()
		msg.header.stamp = rospy.Time.now()
		msg.twist.linear.z = 0.8
		while bt_missions.current_alt <= 35:
			change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                  	response = change_mode(custom_mode="GUIDED")
			bt_missions.set_vel_pub.publish(msg)
			bt_missions.rate.sleep()
			print("current altitude : %s" %bt_missions.current_alt)

	def run(self):
		while True:
			if bt_missions.isContinue == False:
				break
			bt_count = self.tree.blackboard(1)
			bt_state = bt_count.tick()
			print("bt state = %s\n" % bt_state)
			while bt_count == RUNNING:
				bt_state = bt_count.tick()
				print("state = %s\n" %bt_state)
			assert bt_state == SUCCESS or bt_state == FAILURE


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

def main():
	print("start BT:")
	btCm_n = bt_missions()
	try:
		btCm_n.run()
    	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == '__main__':
	try:
		ALT_HOLD()
		Arm()
		time.sleep(3)
		guide()
		Takeoff()
		time.sleep(5)
		main()
		Land()
		time.sleep(3)
		Disarm()
	except rospy.ROSInterruptException:
		pass
