#!/usr/bin/env python

import rospy
import thread
import threading
import time
import mavros
import math
import geopy
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


rospy.init_node('mavros_takeoff_python')
mavros.set_namespace()
rate = rospy.Rate(10)
dirct_ = Float64()
home_ = HomePosition()
current_ = NavSatFix()
gps_state_ = NavSatFix()
home_set_ = False
global lat1,long1,lat2,long2
lat1  = 24.984105
long1= 121.572514
lat2 = 24.98404 
long2 = 121.572775
lat3 = 24.984139 
long3 = 121.57305
global dist
global lat, lon
global brng
lat = 0
lon = 0
def setHomeGeoPointCB(home):
  global home_
  home_= home
  global home_set_
  home_set_ = True
  rospy.loginfo("Received Home (WGS84 datum): %lf, %lf, %lf" % (home_.geo.latitude, home_.geo.longitude, home_.geo.altitude))
#home_sub = rospy.Subscriber("/mavros/home_position/home",HomePosition,setHomeGeoPointCB)

def ALT_HOLD():
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


rospy.loginfo("Received Home (WGS84 datum): %lf, %lf, %lf" % (home_.geo.latitude, home_.geo.longitude, home_.geo.altitude))

def guide():
  rospy.wait_for_service('/mavros/set_mode')
  try:
      change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
      response = change_mode(custom_mode="GUIDED")
      rospy.loginfo(response)
  except rospy.ServiceException as e:
      print("Set mode failed: %s" %e)

def RTL():
  rospy.wait_for_service('/mavros/set_mode')
  try:
      change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
      response = change_mode(custom_mode="RTL")
      rospy.loginfo(response)
  except rospy.ServiceException as e:
      print("Set mode failed: %s" %e)

# Takeoff
def Takeoff():
  print "\nTaking off"
  rospy.wait_for_service('/mavros/cmd/takeoff')
  try:
      takeoff_cl = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
      response = takeoff_cl(altitude=10, latitude=24.984256, longitude=121.572751, min_pitch=0, yaw=0)
      rospy.loginfo(response)
	
  except rospy.ServiceException as e:
      print("Takeoff failed: %s" %e)


def alt_cb(alt):
  cnt = 0
  global alt_
  alt_ = alt
  global altitude_amsl
  altitude = 0
  global altitude_received_
  altitude_amsl = alt_.amsl
  
  cnt = cnt + 1
  if cnt % 6000 == 0 : 
  	rospy.loginfo("altitude: %lf" % altitude_amsl)

def compass(dirct):
	cnt = 0
        global dirct_
	dirct_ = dirct
	global current_heading
	current_heading = dirct_.data
	cnt = cnt +1
	if cnt%100 == 0:	
		rospy.loginfo("Current heading is %f",current_heading)


	

def angleFromCoordinate(lat0, long0, lat1, long1):
	global brng
	brng = 0
	dLon = (long1 - long0)
	y = math.sin(dLon) * math.cos(lat1)
	x = math.cos(lat0) * math.sin(lat1) - math.sin(lat0) * cos(lat1) * math.cos(dLon)
	brng = math.atan2(y,x)
	brng = math.degrees(brng)
	brng = (brng + 360) % 360
	brng = 360 - brng
	brng = math.radians(brng)
	return brng
	

def distance(lat0, long0, lat1, long1):
	p0 = (lat0,long0)
	p1 = (lat1,long1)
	dist = vincenty(p0,p1).meters
	return dist

def current_position(current):
	global current_
	current_  = current
	lat = current_.latitude
	lon = current_.longitude
	
def gpsstate_cb(gps_cb):
	global gps_state_
	gps_state_ = gps_cb
	#print("gps: %s" %gps_state_.status)

def global_position():
	global home_
	global home_set_
        global current_
	global brng
	global dirct,scale
	global current_heading
	global current_yaw 
	guide()
	hdg_subscriber = rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, compass)
	home_sub = rospy.Subscriber("/mavros/home_position/home",HomePosition,setHomeGeoPointCB)
	set_vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
	set_v_pub = rospy.Publisher('/mavros/setpoint_yaw/local', PositionTarget,queue_size=10)
        gps_sub = rospy.Subscriber("/mavros/global_position/global",NavSatFix,current_position)
	rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, gpsstate_cb)
	gps_pub = rospy.Publisher('/mavros/global_position/raw/gps_vel',TwistStamped,queue_size=10)
	#altitude_sub = rospy.Subscriber('/mavros/altitude', Altitude, alt_cb)
	local_pos_pub = rospy.Publisher(mavros.get_topic('setpoint_position', 'local'), PoseStamped, queue_size=10)
	time_now = rospy.get_time()
	while rospy.get_time() - time_now <= 3:
		pass
	print("gps now: %s" %gps_state_.status)
        if gps_state_.status == -1:
		RTL()
	else:
		msg = TwistStamped()
		msg.header.stamp = rospy.Time.now()
		msg.twist.angular.z = 0
		print("forward")
		vel = PositionTarget()
		vel.header.stamp = rospy.Time.now()
		vel.coordinate_frame = PositionTarget.FRAME_BODY_NED
		vel.type_mask = PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.FORCE + PositionTarget.IGNORE_YAW +PositionTarget.IGNORE_YAW_RATE
		vel.velocity.x = 0
		vel.velocity.y = 1
		vel.velocity.z = 0
		forward_t = rospy.get_time()
		while rospy.get_time() - forward_t <= 5:
			set_vel_pub.publish(msg)
			set_v_pub.publish(vel)
			rate.sleep()
			

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
  time.sleep(10)

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



if __name__ == '__main__':
    try:
	ALT_HOLD()
	Arm()
	time.sleep(3)
	guide()
	Takeoff()
	time.sleep(3)
        global_position()
	time.sleep(3)
	#down()
	time.sleep(5)
	Land()
	time.sleep(3)
	Disarm()
    except rospy.ROSInterruptException:
        passcurrent_heading
