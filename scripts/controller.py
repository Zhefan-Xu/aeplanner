#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from tf.transformations import euler_from_quaternion
import message_filters
from math import atan2

# Start
sx = 0
sy = 0
sz = 0
s_roll = 0
s_pitch = 0
s_yaw = 0

# Goal
gx = 0
gy = 0
gz = 0
g_roll = 0
g_pitch = 0
g_yaw = 0


def callback(odom, goal):
	global sx
	global sy
	global sz
	global s_roll
	global s_pitch
	global s_yaw
	global gx
	global gy
	global gz
	global g_roll
	global g_pitch
	global g_yaw
	rospy.loginfo("callback")
	sx = odom.pose.pose.position.x
	sy = odom.pose.pose.position.y
	sz = odom.pose.pose.position.z
	rot_q = odom.pose.pose.orientation
	(s_roll, s_pitch, s_yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

	gx = goal.pose.pose.position.x
	gy = goal.pose.pose.position.y
	gz = goal.pose.pose.position.z
	rot_goal = goal.pose.pose.orientation
	(g_roll, g_pitch, g_yaw) = euler_from_quaternion([rot_goal.x,rot_goal.y,rot_goal.z, rot_goal.w])

	# gx = goal.x
	# gy = goal.y
	# gz = goal.z

rospy.init_node("speed_controller")
odom_sub = message_filters.Subscriber("odom", Odometry)
# goal_sub = message_filters.Subscriber("get_goal", Point)
goal_sub = message_filters.Subscriber("get_goal", Odometry)

ts = message_filters.ApproximateTimeSynchronizer([odom_sub, goal_sub], 10, 0.1, allow_headerless=True)
ts.registerCallback(callback)

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
speed = Twist()
adjust_heading = True
while not rospy.is_shutdown():
	dx = gx - sx
	dy = gy - sy
	dz = gz - sz
	angle_to_goal = atan2(dy, dx)
	

	# First Adjust Heading
	if abs(angle_to_goal - s_yaw) > 0.3 and adjust_heading:
		rospy.loginfo("Adjust Heading: %s", abs(angle_to_goal - s_yaw))
		# speed.angular.z = 0.1 * abs(angle_to_goal - s_yaw)
		speed.angular.z = 0.3 * (angle_to_goal-s_yaw)/abs(angle_to_goal-s_yaw)
		# speed.angular.z = 0.5
		speed.linear.x = 0
		speed.linear.z = 0

	# Move to Position
	# elif abs(dx) > 0.3 or abs(dy) > 0.3 or abs(dz) > 0.3:
	# 	rospy.loginfo("Move to Position: %s, %s, %s", abs(dx), abs(dy), abs(dz))
	# 	total = 0.3 * (dx**2+dy**2+dz**2) ** 0.5
	# 	speed.angular.z = 0
	# 	speed.linear.x = total * (dx**2 + dy**2)**0.5/((dx**2 + dy**2 + dz**2)**0.5+0.000001)
	# 	speed.linear.z = total * (dz) / ((dx**2 + dy**2 + dz**2)**0.5+0.000001)
		
	# else:
	# 	# Adjust View Angle 
	# 	adjust_heading = False
	# 	if abs(g_yaw - s_yaw) > 0.3:
	# 		rospy.loginfo("Adjust View Angle: %s",abs(g_yaw - s_yaw))
	# 		rospy.loginfo("goal yaw: %s", g_yaw)
	# 		rospy.loginfo("current yaw: %s", s_yaw)
	# 		speed.angular.z = 0.3 * abs(g_yaw - s_yaw)
	# 		speed.linear.x = 0
	# 		speed.linear.z = 0
	# 	else:
	# 		rospy.loginfo("Reach")
	# 		speed.angular.z = 0
	# 		speed.linear.x = 0
	# 		speed.linear.z = 0
	# 		adjust_heading = True
	else:
		rospy.loginfo("Move to Position: %s, %s, %s", abs(dx), abs(dy), abs(dz))
		# total = 0.3 * (dx**2+dy**2+dz**2) ** 0.5
		total = 0.3 
		speed.angular.z = 0
		speed.linear.x = total * (dx**2 + dy**2)**0.5/((dx**2 + dy**2 + dz**2)**0.5+0.000001)
		speed.linear.z = total * (dz) / ((dx**2 + dy**2 + dz**2)**0.5+0.000001)
	# if abs(dx) > 0.3 or abs(dy) > 0.3 or abs(dz) > 0.3:
	# 	rospy.loginfo("move to position")
	# 	total = 0.3
	# 	speed.angular.z = 0
	# 	speed.linear.x = total * (dx)/((dx**2 + dy**2 + dz**2)**0.5+0.000001)
	# 	speed.linear.y = total * (dy)/((dx**2 + dy**2 + dz**2)**0.5+0.000001)
	# 	speed.linear.z = total * (dz)/((dx**2 + dy**2 + dz**2)**0.5+0.000001)
	# else:
	# 	if abs(g_yaw - s_yaw) > 0.3:
	# 		rospy.loginfo("Adjust View Angle")
	# 		speed.angular.z = 0.3
	# 		speed.linear.x = 0
	# 		speed.linear.y = 0
	# 		speed.linear.z = 0
	# 	else:
	# 		rospy.loginfo("reach")
	# 		speed.angular.z = 0
	# 		speed.linear.x = 0
	# 		speed.linear.y = 0
	# 		speed.linear.z = 0



	pub.publish(speed)
