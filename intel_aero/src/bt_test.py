#!/usr/bin/env python

import rospy
import time
import mavros
import math
import geopy
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


#rospy.loginfo("Received Home (WGS84 datum): %lf, %lf, %lf" % (home_.geo.latitude, home_.geo.longitude, home_.geo.altitude))

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
      response = takeoff_cl(altitude=7, latitude = home_.geo.latitude, longitude = home_.geo.longitude, min_pitch=0, yaw=0)
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
	current_ = GlobalPositionTarget()
	home_set_ = False
	isContinue = True
	dist_first = True
	dis_count = True
	#triangle
	target = [[24.985059,121.572934],[24.985161,121.572858]]
	forward_t = rospy.get_time()
	index = 0
	time_index = 0
	p0 = 0
	p1 = 0
	brng = 0
	dLon = 0
	y = 0
	x = 0
	current_yaw = 0
	dist = 0
	heading = 0
	lat = 0
	lon = 0
	cnt = 0
	set_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
	set_v_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget,queue_size=10)
	local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
	
		
	
	def __init__(self):
		self.hdg_subscriber = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64,self.compass)
		self.home_sub = rospy.Subscriber("/mavros/home_position/home",HomePosition,self.setHomeGeoPointCB)
		self.gps_sub = rospy.Subscriber("/mavros/global_position/global",NavSatFix,self.current_position_cb)
		state_sub = rospy.Subscriber(mavros.get_topic('state'), State, self.state_cb)
		self.tree = self.acheive |((self.isGuided | (self.notGuided >> self.set_guided)) >> (self.yaw_correction | self.notGuided | self.notGuided | self.correction) >> self.forward)
		#(self.yaw_discorrection | self.notGuided | (self.yaw_correction >> 
	def compass(self, dirct):
		bt_missions.dirct_ = dirct
		bt_missions.heading = bt_missions.dirct_.data
	
	def state_cb(self, state):
		bt_missions.current_state = State()
		bt_missions.current_state = state
	
	def angleFromCoordinate(self,lat0, long0, lat1, long1):
		bt_missions.dLon = (long1 - long0)
		bt_missions.y = math.sin(bt_missions.dLon) * math.cos(bt_missions.lat1)
		bt_missions.x = math.cos(lat0) * math.sin(lat1) - math.sin(lat0) * cos(lat1) * math.cos(bt_missions.dLon)
		bt_missions.brng = math.atan2(bt_missions.y,bt_missions.x)
		bt_missions.brng = math.degrees(bt_missions.brng)
		bt_missions.brng = (bt_missions.brng + 360) % 360
		bt_missions.brng = 360 - bt_missions.brng
		bt_missions.brng = math.radians(bt_missions.brng)

	def setHomeGeoPointCB(self,home):
		bt_missions.home_= home
		bt_missions.home_set_ = True
		rospy.loginfo("Received Home (WGS84 datum): %lf, %lf, %lf" % (bt_missions.home_.geo.latitude, bt_missions.home_.geo.longitude, bt_missions.home_.geo.altitude))

	def current_position_cb(self,current):
		bt_missions.current_  = current
		bt_missions.lat = bt_missions.current_.latitude
		bt_missions.lon = bt_missions.current_.longitude
	
	
	
	@condition
	def acheive(self):
		if bt_missions.index == 0 and bt_missions.dist_first == True:
			bt_missions.dist_first = False
			bt_missions.p0 = (bt_missions.home_.geo.latitude,bt_missions.home_.geo.longitude)
			bt_missions.p1 = (bt_missions.target[bt_missions.index][0],bt_missions.target[bt_missions.index][1])
			bt_missions.dist = vincenty(bt_missions.p0,bt_missions.p1).meters
			print(bt_missions.dist)
			if bt_missions.dist >= 1.5:
				return False
			else:
				bt_missions.index = bt_missions.index + 1
		elif bt_missions.index == 0 and bt_missions.dist_first == False:
			bt_missions.p0 = (bt_missions.current_.latitude, bt_missions.current_.longitude)
			bt_missions.p1 = (bt_missions.target[bt_missions.index][0],bt_missions.target[bt_missions.index][1])
			bt_missions.dist = vincenty(bt_missions.p0,bt_missions.p1).meters
			print(bt_missions.dist)
			if bt_missions.dist >= 1.5:
				return False
			else:
				bt_missions.index = bt_missions.index + 1
		elif bt_missions.index == 1:
			bt_missions.p0 = (bt_missions.current_.latitude, bt_missions.current_.longitude)
			bt_missions.p1 = (bt_missions.target[bt_missions.index][0],bt_missions.target[bt_missions.index][1])
			bt_missions.dist = vincenty(bt_missions.p0,bt_missions.p1).meters
			print(bt_missions.dist)
			if bt_missions.dist >= 1.5:
				return False
			else:
				return True

	@condition
	def isGuided(self):
		if bt_missions.current_state == "GUIDED":
			return True
		else:
			return False
	
	@condition
	def notGuided(self):
		if bt_missions.current_state == "GUIDED":
			return False
		else:
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
			if bt_missions.current_yaw >(360 - bt_missions.brng) + 2 or bt_missions.current_yaw <(360 - bt_missions.brng) -2:
				return False
			else:
				return True
		else:
			bt_missions.dis_count = True
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
			if bt_missions.current_yaw >(360 - bt_missions.brng) + 2 or bt_missions.current_yaw <(360 - bt_missions.brng) -2:
				return False
			else:
				return True

	@condition
	def yaw_discorrection(self):
		if bt_missions.index == 0:
			bt_missions.current_yaw = bt_missions.heading
			print("discorrection heading: %s" %bt_missions.current_yaw)
			print("discorrection brng: %s" %bt_missions.brng)
			if bt_missions.current_yaw >(360 - bt_missions.brng) + 2 or bt_missions.current_yaw <(360 - bt_missions.brng) -2:
				bt_missions.current_yaw = bt_missions.heading
				return True
			else:
				return False

		else:
			bt_missions.current_yaw = bt_missions.heading
			print("discorrection heading: %s" %bt_missions.current_yaw)
			print("discorrection brng: %s" %bt_missions.brng)
			if bt_missions.current_yaw >(360 - bt_missions.brng) + 2 or bt_missions.current_yaw <(360 - bt_missions.brng) -2:
				bt_missions.current_yaw = bt_missions.heading
				return True
			else:
				return False
	
	@action
	def set_guided(self):
	  rospy.wait_for_service('/mavros/set_mode')
	  try:
		  change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
		  response = change_mode(custom_mode="GUIDED")
	  except rospy.ServiceException as e:
		  print("Set mode failed: %s" %e)
		
	
	@action 
	def correction(self):
		if bt_missions.dis_count == True:
			bt_missions.current_yaw = bt_missions.heading
			while bt_missions.current_yaw >(360 - bt_missions.brng) + 2 or bt_missions.current_yaw <(360 - bt_missions.brng) -2:
				msg = TwistStamped()
				if 360 - bt_missions.brng - bt_missions.current_yaw > 0 and 360 - bt_missions.brng - bt_missions.current_yaw < 180 :
					msg.twist.angular.z = -0.1
				elif 360 - bt_missions.brng - bt_missions.current_yaw > 0 and 360 - bt_missions.brng - bt_missions.current_yaw >= 180:  
					msg.twist.angular.z = 0.1
				elif 360 - bt_missions.brng - bt_missions.current_yaw < 0 and math.fabs(360 - bt_missions.brng - bt_missions.current_yaw) >= 180:
					msg.twist.angular.z = -0.1
				elif 360 - bt_missions.brng - bt_missions.current_yaw < 0 and math.fabs(360 - bt_missions.brng - bt_missions.current_yaw) < 180:
					msg.twist.angular.z = 0.1
				bt_missions.set_vel_pub.publish(msg)
				bt_missions.current_yaw = bt_missions.heading
				#print("first correction : %s" %bt_missions.current_yaw)
		else:
			#pass
			bt_missions.current_yaw = bt_missions.heading
			while bt_missions.current_yaw >(360 - bt_missions.brng) + 2 or bt_missions.current_yaw <(360 - bt_missions.brng) -2: #and int(rospy.get_time() -bt_missions.forward_t) % 10 == 0:
					msg = TwistStamped()
					#print("forward correction: %s" %bt_missions.current_yaw)
					msg.twist.angular.z = math.radians(bt_missions.current_yaw - 360 + bt_missions.brng)
					bt_missions.set_vel_pub.publish(msg)
					bt_missions.current_yaw = bt_missions.heading
	
	@action 
	def forward(self):
		print(123)
		forward_t = rospy.get_time()
		#while rospy.get_time() - forward_t <= 5:
		bt_missions.dis_count = False
		vel = PositionTarget()
		vel.header.stamp = rospy.Time.now()
		vel.coordinate_frame = PositionTarget.FRAME_BODY_NED
		vel.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.FORCE + PositionTarget.IGNORE_YAW +PositionTarget.IGNORE_YAW_RATE
		vel.velocity.x = 0
		vel.velocity.y = 0.8
		vel.velocity.z = 0
		bt_missions.set_v_pub.publish(vel)
		bt_missions.rate.sleep()
		
	
	
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
