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
from math import *
from mavros.utils import *
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
      response = takeoff_cl(altitude=20, latitude = home_.geo.latitude, longitude = home_.geo.longitude, min_pitch=0, yaw=0)
      rospy.loginfo(response)
  except rospy.ServiceException as e:
      print("Takeoff failed: %s" %e)



class bt_missions:
	rospy.init_node('mavros_takeoff_python')
	mavros.set_namespace()
	rate = rospy.Rate(20)
	dirct_ = Float64()
	home_ = HomePosition()
	current_state = State()
	current_ = NavSatFix()
	alt_ = Float64()
	img = Image()
	fourcc = cv2.VideoWriter_fourcc('X','V','I','D')
	out = cv2.VideoWriter('realsense.avi', fourcc, 20.0, (640, 480))
	out1 = cv2.VideoWriter('usb_cam.avi', fourcc, 20.0, (640, 480),0)
	home_set_ = False
	isContinue = True
	dist_first = True
	kp_count = True
	ah_flag = False
	forward_flag = False
	target = [[24.982550663278158,121.57233949235275],[24.981975,121.57176],[24.981758,121.571963]]
	forward_t = 0
	correction_t = 0
	index = 0
	time_index = 0
	p0 = 0
	p1 = 0
	brng = 0
	dLon = 0
	y = 0
	x = 0
	current_yaw = 0
	current_alt = 0
	dist = 0
	heading = 0
	lat = 0
	lon = 0
	cnt = 0
	img_count = 0
	query_count = 0
	monoq_count = 0
	record_img = np.array([])
	record_mono = np.array([])
	set_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
	set_v_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget,queue_size=10)
	local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)

	def __init__(self):
		self.hdg_subscriber = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64,self.compass)
		self.rel_alt= rospy.Subscriber("/mavros/global_position/rel_alt", Float64,self.altitude)
		self.home_sub = rospy.Subscriber("/mavros/home_position/home",HomePosition,self.setHomeGeoPointCB)
		self.gps_sub = rospy.Subscriber("/mavros/global_position/global",NavSatFix,self.current_position_cb)
		self.state_sub = rospy.Subscriber(mavros.get_topic('state'), State, self.state_cb)
		self.img_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.call, buff_size = 2**24, queue_size = 1)
		self.mono_sub = rospy.Subscriber("/camera/mono/image_raw", Image, self.mono_cb, buff_size = 2**24, queue_size = 1)
		self.tree = (self.achieve | ((self.achieve_alt | self.up) >> (self.yaw_correction | self.correction) >> self.forward)) >> (self.ah | (self.isdetect >> self.change_alt)) #>> (self.realsense_check | self.change_alt | (self.mono_detect >> self.change_alt))

	def compass(self, dirct):
		bt_missions.dirct_ = dirct
		bt_missions.heading = bt_missions.dirct_.data
	
	def state_cb(self, state):
		bt_missions.current_state = State()
		bt_missions.current_state = state

	def altitude(self, alt):
		bt_missions.alt_ = alt
		bt_missions.current_alt = bt_missions.alt_.data

	def setHomeGeoPointCB(self,home):
		bt_missions.home_= home
		bt_missions.home_set_ = True
		rospy.loginfo("Received Home (WGS84 datum): %lf, %lf, %lf" % (bt_missions.home_.geo.latitude, bt_missions.home_.geo.longitude, bt_missions.home_.geo.altitude))

	def current_position_cb(self,current):
		bt_missions.current_  = current
		bt_missions.lat = bt_missions.current_.latitude
		bt_missions.lon = bt_missions.current_.longitude
	
	def call(self,callimage):
    		bt_missions.record_img = CvBridge().imgmsg_to_cv2(callimage, "bgr8")
		bt_missions.out.write(bt_missions.record_img)

	def mono_cb(self,monoimage):
		bt_missions.record_mono = CvBridge().imgmsg_to_cv2(monoimage, "mono8")
		bt_missions.out1.write(bt_missions.record_mono)
    		
	@condition
	def ah(self):
		if bt_missions.ah_flag == True:
			return False
		else:
			return True

	'''@condition
	def realsense_check(self):
		if bt_missions.forward_flag == True:
			return False
		else:
			return True'''
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
		kp1, des1 = sift.detectAndCompute(img1,None)
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
		if len(good) >= 10:
			bt_missions.index = bt_missions.index + 1
			bt_missions.img_count = 0
			bt_missions.kp_count = False
			return True
				
		else:
			bt_missions.forward_flag = True
			return False
	
	@condition
	def mono_detect(self):
		bt_missions.monoq_count = bt_missions.monoq_count + 1
		img1 = cv2.imread('query/' + str(bt_missions.monoq_count) + '.PNG',cv2.IMREAD_GRAYSCALE) #queryImage
		bt_missions.img_count = bt_missions.img_count + 1
		cv2.imwrite('image/'+str(bt_missions.img_count) + '.PNG',bt_missions.record_img)
		img2 = cv2.imread('image/'+str(bt_missions.img_count) + '.PNG',cv2.IMREAD_GRAYSCALE) #trainImage
		# Initiate SIFT detector
		sift = cv2.xfeatures2d.SIFT_create()
		# find the keypoints and descriptors with SIFT
		kp1, des1 = sift.detectAndCompute(img1,None)
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
		if len(good) >= 10:
			bt_missions.index = bt_missions.index + 1
			bt_missions.img_count = 0
			bt_missions.kp_count = False
			return True
				
		else:
			bt_missions.forward_flag = False
			return False
			
	@condition
	def achieve(self):
		if bt_missions.index == 0 and bt_missions.dist_first == True:
			bt_missions.dist_first = False
			bt_missions.p0 = (bt_missions.home_.geo.latitude,bt_missions.home_.geo.longitude)
			bt_missions.p1 = (bt_missions.target[bt_missions.index][0],bt_missions.target[bt_missions.index][1])
			bt_missions.dist = vincenty(bt_missions.p0,bt_missions.p1).meters
			print("current dist: %s" %bt_missions.dist)
			if bt_missions.dist >= 5:
				return False
			else:
				bt_missions.index = bt_missions.index + 1

		elif bt_missions.index == 0 and bt_missions.dist_first == False:
			bt_missions.p0 = (bt_missions.current_.latitude, bt_missions.current_.longitude)
			bt_missions.p1 = (bt_missions.target[bt_missions.index][0],bt_missions.target[bt_missions.index][1])
			bt_missions.dist = vincenty(bt_missions.p0,bt_missions.p1).meters
			print("current dist: %s" %bt_missions.dist)
			if bt_missions.dist >= 5:
				return False
			else:
				print("achieve first point")
				bt_missions.kp_count == True
				bt_missions.index = bt_missions.index + 1
				return True

		elif bt_missions.index == 1 and bt_missions.dist_first == False:
			bt_missions.p0 = (bt_missions.current_.latitude, bt_missions.current_.longitude)
			bt_missions.p1 = (bt_missions.target[bt_missions.index][0],bt_missions.target[bt_missions.index][1])
			bt_missions.dist = vincenty(bt_missions.p0,bt_missions.p1).meters
			print("current dist: %s" %bt_missions.dist)
			if bt_missions.dist >= 5:
				return False
			else:
				print("achieve key point")
				bt_missions.ah_flag = True
				bt_missions.kp_count == True
				#bt_missions.index = bt_missions.index + 1
				return True
		
		elif bt_missions.index == 2 and bt_missions.kp_count == False:
			bt_missions.p0 = (bt_missions.current_.latitude, bt_missions.current_.longitude)
			bt_missions.p1 = (bt_missions.target[bt_missions.index][0],bt_missions.target[bt_missions.index][1])
			bt_missions.dist = vincenty(bt_missions.p0,bt_missions.p1).meters
			print("current dist: %s" %bt_missions.dist)
			if bt_missions.dist >= 5:
				return False
			else:
				print("achieve final point")
				#bt_missions.ah_flag = True
				bt_missions.kp_count == True
				return True

	@condition
	def achieve_alt(self):
		if bt_missions.current_alt > 15:
			print("altitude enough")
			return True
		else:
			print("altitude not enough")
			return False
	
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
			if bt_missions.current_yaw >(360 - bt_missions.brng) + 2 or bt_missions.current_yaw <(360 - bt_missions.brng) -2:
				return False
			else:
				return True
		else:
			bt_missions.dLon = (bt_missions.target[bt_missions.index][1] - bt_missions.current_.longitude)
			bt_missions.y = math.sin(bt_missions.dLon) * math.cos(bt_missions.target[bt_missions.index][0])
			bt_missions.x = math.cos(bt_missions.home_.geo.latitude) * math.sin(bt_missions.target[bt_missions.index][0]) - math.sin(bt_missions.current_.latitude) * cos(bt_missions.target[bt_missions.index][0]) * math.cos(bt_missions.dLon)
			bt_missions.brng = math.atan2(bt_missions.y,bt_missions.x)
			bt_missions.brng = math.degrees(bt_missions.brng)
			bt_missions.brng = (bt_missions.brng + 360) % 360
			bt_missions.brng = 360 - bt_missions.brng
			bt_missions.brng = math.radians(bt_missions.brng)
			bt_missions.brng = math.degrees(bt_missions.brng)
			bt_missions.current_yaw = bt_missions.heading
			print("correction heading: %s" %bt_missions.current_yaw)
                        print("correction brng: %s" %bt_missions.brng)
			if bt_missions.current_yaw >(360 - bt_missions.brng) + 2 or bt_missions.current_yaw <(360 - bt_missions.brng) -2:
				return False
			else:
				return True

	@action
	def up(self):
		msg = TwistStamped()
		msg.header.stamp = rospy.Time.now()
		msg.twist.linear.z = 0.8
		while bt_missions.current_alt <= 15:
			#rospy.wait_for_service('/mavros/set_mode')
			change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
                  	response = change_mode(custom_mode="GUIDED")
			bt_missions.set_vel_pub.publish(msg)
			bt_missions.rate.sleep()
			print("current altitude : %s" %bt_missions.current_alt)


	@action 
	def correction(self):
		msg = TwistStamped()
		msg.header.stamp = rospy.Time.now()
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
		while bt_missions.current_yaw >(360 - bt_missions.brng) + 2 or bt_missions.current_yaw <(360 - bt_missions.brng) -2:
			#rospy.wait_for_service('/mavros/set_mode')
		  	change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)	
			response = change_mode(custom_mode="GUIDED")
			bt_missions.set_vel_pub.publish(msg)
			bt_missions.current_yaw = bt_missions.heading
			bt_missions.rate.sleep()
			print("first correction heading : %s" %bt_missions.current_yaw)
		
	@action
	def forward(self):
		msg = TwistStamped()
		msg.header.stamp = rospy.Time.now()
		msg.twist.angular.z = 0
		print("forward")
		vel = PositionTarget()
		vel.header.stamp = rospy.Time.now()
		vel.coordinate_frame = PositionTarget.FRAME_BODY_NED
		vel.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.FORCE + PositionTarget.IGNORE_YAW +PositionTarget.IGNORE_YAW_RATE
		vel.velocity.x = 0
		vel.velocity.y = 1.0
		vel.velocity.z = 0
		bt_missions.p0 = (bt_missions.current_.latitude, bt_missions.current_.longitude)
		bt_missions.p1 = (bt_missions.target[bt_missions.index][0],bt_missions.target[bt_missions.index][1])
		bt_missions.dist = vincenty(bt_missions.p0,bt_missions.p1).meters
		forward_t = rospy.get_time()
		while rospy.get_time() - forward_t <= bt_missions.dist*0.1:	#bt_missions.dist *1.25*0.25:
			#rospy.wait_for_service('/mavros/set_mode')
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
			#rospy.wait_for_service('/mavros/set_mode')
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
		check_mode()
		main()
		Land()
		time.sleep(3)
		Disarm()
	except rospy.ROSInterruptException:
		pass
