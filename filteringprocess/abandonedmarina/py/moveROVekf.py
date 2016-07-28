#!/usr/bin/env python
import rospy
import roslib

import ekf
import math
#import outlier_filter
from geometry_msgs.msg import Twist, Vector3, Pose, PoseStamped, PointStamped, TransformStamped, Transform
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from abandonedmarina.msg import amDVL, amMTi
from numpy import mean, std
import sys
import numpy as np
import random
import tf
from tf.transformations import euler_from_quaternion, quaternion_matrix

def sonar_callback(pos):
	if sonar_callback.initialized is False:
		sonar_callback.H = np.matrix([[1,0,0,0,0,0,0,0],
									  [0,1,0,0,0,0,0,0],
									  [0,0,1,0,0,0,0,0],
									  [0,0,0,1,0,0,0,0]])
		sonar_callback.R = np.matrix([[1000,0,0,0],
									  [0,1000,0,0],
									  [0,0,1000,0],
									  [0,0,0,1000]])
		sonar_callback.initialized = True

	if pos.rotation.w == 0:
		print "bad quaternion"
		return

	global xpos,ypos,zpos,yawpos,xbuff,ybuff,zbuff,yawbuff

	# Create matrix from geometry_msgs.msg.Transform
	quat = [pos.rotation.x, pos.rotation.y, pos.rotation.z, pos.rotation.w]
	a = quaternion_matrix(quat)
	a[0,3] = pos.translation.x
	a[1,3] = pos.translation.y
	b = np.array([xpos, ypos, zpos, 1])
	c = a.dot(b)

	# print "before: ", xpos, ypos, zpos
	# print "after:", c
	# print "quat:", quat
	# print "euler:", tf.transformations.euler_from_quaternion(quat)

	c[3] = yawpos - tf.transformations.euler_from_quaternion(quat)[2]


	# xpos,ypos,zpos,yawpos = ekffilter.returnState()
	
	measurement = np.zeros((4,1))
	measurement[0] = c[0]	
	measurement[1] = c[1]	
	measurement[2] = c[2]	
	measurement[3] = c[3]

	ekffilter.update(measurement, sonar_callback.H,sonar_callback.R)
	xposT,yposT,zposT,yawposT = ekffilter.returnState()
	xbuff   -= xposT-xpos
	ybuff   -= yposT-ypos
	zbuff   -= zposT-zpos
	yawbuff -= yawposT-yawpos

	xpos,ypos,zpos,yawpos = xposT,yposT,zposT,yawposT

	broadcaster.sendTransform( (xpos,ypos,zpos), tf.transformations.quaternion_from_euler(0, 0, yawpos),rospy.Time.now(), "body","odom")
	
	broadcaster.sendTransform((xbuff, ybuff, zbuff), 
								tf.transformations.quaternion_from_euler(0, 0, yawbuff),
								rospy.Time.now(),"odom","buffer")


	path_now = PoseStamped()
	path_now.pose.position.x = xpos
	path_now.pose.position.y = ypos
	path_now.pose.position.z = 0
	travelPath.poses.append(path_now)
	path.publish(travelPath)



def dvl_callback(data):
	if dvl_callback.initialized is False:
		dvl_callback.H = np.matrix([[0,0,0,0,1,0,0,0],
									[0,0,0,0,0,1,0,0],
									[0,0,0,0,0,0,1,0],
									[0,0,1,0,0,0,0,0]])
		dvl_callback.initialized = True

	global xpos,ypos,zpos,yawpos,xbuff,ybuff,zbuff,yawbuff

	ekffilter.predict(data.header.stamp)

	ps = PointStamped()
	ps.header.frame_id = "dvl"
	if data.bottomok == 1:
		ps.point.x = data.vbx / 100.0
		ps.point.y = data.vby / 100.0
	elif data.waterok == 1:
		ps.point.x = data.vwx / 100.0
		ps.point.y = data.vwy / 100.0
	else:
		print "no dvl measurement"
		return

	ps = listener.transformPoint("body", ps)

	measurement = np.zeros((4,1))
	measurement[0] = ps.point.y
	measurement[1] = ps.point.x
 
	ekffilter.update(measurement, dvl_callback.H)
	xpos,ypos,zpos,yawpos = ekffilter.returnState()

	broadcaster.sendTransform( (xpos,ypos,zpos), tf.transformations.quaternion_from_euler(0, 0, yawpos),rospy.Time.now(), "body","odom")
	
	broadcaster.sendTransform((xbuff, ybuff, zbuff), 
								tf.transformations.quaternion_from_euler(0, 0, yawbuff),
								rospy.Time.now(),"odom","buffer")


	path_now = PoseStamped()
	path_now.pose.position.x = xpos
	path_now.pose.position.y = ypos
	path_now.pose.position.z = 0
	travelPath.poses.append(path_now)
	path.publish(travelPath)

	# print state.T

def mti_callback(data):
	if mti_callback.initialized is False:
		mti_callback.H = np.matrix([[0,0,0,1,0,0,0,0]])
		mti_callback.initialized = True

	global xpos,ypos,zpos,yawpos,xbuff,ybuff,zbuff,yawbuff

	measurement = np.matrix([data.Yaw + yawbuff])



	# print "first: ", ekffilter.returnState().T
	ekffilter.predict(data.header.stamp)
	
	# print "predict: ", ekffilter.returnState().T
	ekffilter.update(measurement, mti_callback.H)
	
	# print "update: ", ekffilter.returnState().T
	xpos,ypos,zpos,yawpos = ekffilter.returnState()
	
	# print "variables: ", x,y,z,theta
	# print "finished: ", ekffilter.returnState().T
	# print " "
	# print " "
	# print " "

	broadcaster.sendTransform( (xpos,ypos,zpos), tf.transformations.quaternion_from_euler(0, 0, yawpos),rospy.Time.now(), "body","odom")
	
	broadcaster.sendTransform((xbuff, ybuff, zbuff), 
								tf.transformations.quaternion_from_euler(0, 0, yawbuff),
								rospy.Time.now(),"odom","buffer")

	# path_now = PoseStamped()
	# path_now.pose.position.x = xpos
	# path_now.pose.position.y = ypos
	# path_now.pose.position.z = 0
	# travelPath.poses.append(path_now)
	# path.publish(travelPath)

if __name__ == '__main__':
	#set up the node
	rospy.init_node('moveROVekf', anonymous=True)

	# tn = rospy.Time.now()

	broadcaster = tf.TransformBroadcaster()
	listener = tf.TransformListener()

	ekffilter = ekf.ekf()

	dvl_callback.initialized   = False
	mti_callback.initialized   = False
	sonar_callback.initialized = False

	xpos = 0
	ypos = 0
	zpos = 0
	yawpos = 0

	xbuff = 0
	ybuff = 0
	zbuff = 0
	yawbuff = 0
	travelPath = Path()
	travelPath.header.frame_id="odom"

	broadcaster.sendTransform( (0,0,0), (0,0,0,1),rospy.Time.now(), "test", "odom")
	path = rospy.Publisher('pathekf', Path, queue_size=1)

	rospy.Subscriber("/abandonedMarina/DVLData", amDVL, dvl_callback);
	rospy.Subscriber("/abandonedMarina/MTiData", amMTi, mti_callback);
	rospy.Subscriber("/mapBuilderAB/trans_update", Transform, sonar_callback);
	rospy.spin()
