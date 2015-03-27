#! /usr/bin/python

import rospy
import tf
from numpy import linalg as LA
from math import *
from sensor_msgs.msg import LaserScan, JointState
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2
import math

# assuming there is already a ros node, do not init one here

# P control is sufficient for this function


class baseScan:
	def __init__(self, verbose=False):
		self.rangeData = LaserScan()
		self.scan_sub = rospy.Subscriber("/base_scan", LaserScan, self.callback)
		self.laser_projector = LaserProjection()
		self.pc = []
		self.leg1 = []
		self.leg2 = []
		self.br = tf.TransformBroadcaster()
		self.rate = rospy.Rate(4.0)


	def callback(self,data):
		self.rangeData = data

	def refreshRangeData(self):
		self.rangeData = LaserScan() # flush
		while len(self.rangeData.ranges) == 0:
			rospy.sleep(0.1)


	def getCloud(self):

		self.refreshRangeData()
		cloud2 = self.laser_projector.projectLaser(self.rangeData)
		xyz = pc2.read_points(cloud2, skip_nans=True, field_names=("x", "y", "z"))
		self.pc = []
		while True:
			try:
				self.pc.append(xyz.next())
			except:
				break
		return self.pc

	def findLegs(self):
		# Assuming there is nothing between the robot and the shelf
		pc = self.getCloud()
		x = []
		y= []
		for i in range(len(pc)):
			x.append(pc[i][0])
			y.append(pc[i][1])
		radius = []
		for i in range(len(pc)):
			radius.append(math.sqrt(x[i]**2 + y[i]**2))
		n = radius.index(min(radius))
		x2 = [x[i] for i in range(len(x)) if math.sqrt( (x[i]-x[n])**2 + (y[i]-y[n])**2 ) > 0.1]
		y2 = [y[i] for i in range(len(y)) if math.sqrt( (x[i]-x[n])**2 + (y[i]-y[n])**2 ) > 0.1]
		radius2 = []
		for i in range(len(x2)):
			radius2.append(math.sqrt(x2[i]**2 + y2[i]**2))
		n2 = radius2.index(min(radius2))

		self.leg1 = [x[n], y[n]]
		self.leg2 = [x2[n2], y2[n2]]
		return [self.leg1, self.leg2]

	def getShelfFrame(self):
		# with respect to the frame of /base_scan
		legs = self.findLegs()
		ori_x = (legs[0][0] + legs[1][0]) / 2.
		ori_y = (legs[0][1] + legs[1][1]) / 2.

		left_leg = legs[0]
		if legs[0][1] < legs[1][1]:
			left_leg = legs[1]

		rotAngle = atan2(ori_x - left_leg[0], left_leg[1] - ori_y)

		return [ori_x, ori_y], rotAngle

	def publish2TF(self):
		while not rospy.is_shutdown():
			shelfOri, shelfRot = self.getShelfFrame()
			self.br.sendTransform((shelfOri[0], shelfOri[1], 0),
	                         tf.transformations.quaternion_from_euler(0, 0, shelfRot),
	                         rospy.Time.now(),
	                         "/shelf_frame",     # child
	                         "/base_laser_link"      # parent
	                         )
			self.rate.sleep()