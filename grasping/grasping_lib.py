#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys
import tf
import PyKDL as kdl
from tf_conversions import posemath


def getApprochVec(bin_frame, obj_frame):

	'''
	this function assumes everything is represented in the quaternions in the /base_link frame
	'''

	F_bin_frame = posemath.fromTf(bin_frame)
	F_obj_frame = posemath.fromTf(obj_frame)

	objRed = F_obj_frame.M * kdl.Vector(1.0, 0.0, 0.0)
	objGreen = F_obj_frame.M * kdl.Vector(0.0, 1.0, 0.0)
	objBlue = F_obj_frame.M * kdl.Vector(0.0, 0.0, 1.0)
	
	binRed = F_bin_frame.M * kdl.Vector(1.0, 0.0, 0.0)
	binGreen = F_bin_frame.M * kdl.Vector(0.0, 1.0, 0.0)
	binBlue = F_bin_frame.M * kdl.Vector(0.0, 0.0, 1.0)
	
	rRProj = kdl.dot(objRed , binRed)
	gRProj = kdl.dot(objGreen, binRed)
	bRProj = kdl.dot(objBlue, binRed)

	tmp1 = [abs(rRProj), abs(gRProj), abs(bRProj)]
	tmp2 = [rRProj, gRProj, bRProj]
	axis = tmp1.index(max(tmp1))

	return axis, tmp2[axis]/abs(tmp2[axis])


def getGraspingAxis(approchVec, bin_frame, obj_frame):

	F_bin_frame = posemath.fromTf(bin_frame)
	F_obj_frame = posemath.fromTf(obj_frame)

	objRed = F_obj_frame.M * kdl.Vector(1.0, 0.0, 0.0)
	objGreen = F_obj_frame.M * kdl.Vector(0.0, 1.0, 0.0)
	objBlue = F_obj_frame.M * kdl.Vector(0.0, 0.0, 1.0)
	

	binRed = F_bin_frame.M * kdl.Vector(1.0, 0.0, 0.0)
	binGreen = F_bin_frame.M * kdl.Vector(0.0, 1.0, 0.0)
	binBlue = F_bin_frame.M * kdl.Vector(0.0, 0.0, 1.0)

	objAxes = [objRed, objGreen, objBlue]
	tmp1 = []
	for i in range(3):
		if i == approchVec:
			tmp1.append(0)
			continue
		tmp1.append(kdl.dot(objAxes[i], binBlue))

	tmp2 = [abs(t) for t in tmp1]

	axis = tmp2.index(max(tmp2))

	return axis, tmp1[axis]/abs(tmp1[axis])



def publishGraspingFrame(bin_frame, obj_frame):
	approchVec, approchDir = getApprochVec(bin_frame, obj_frame)
	graspingVec, graspingDir = getGraspingAxis(approchVec, bin_frame, obj_frame)

	F_bin_frame = posemath.fromTf(bin_frame)
	F_obj_frame = posemath.fromTf(obj_frame)

	objRed = F_obj_frame.M * kdl.Vector(1.0, 0.0, 0.0)
	objGreen = F_obj_frame.M * kdl.Vector(0.0, 1.0, 0.0)
	objBlue = F_obj_frame.M * kdl.Vector(0.0, 0.0, 1.0)

	objAxes = [objRed, objGreen, objBlue]

	x = [a*approchDir for a in objAxes[approchVec]]
	z = [g*graspingDir for g in objAxes[graspingVec]]

	y = [a*b for a,b in zip(z,x)]
	

	br = tf.TransformBroadcaster()

	r = kdl.Rotation()
	for i in range(3):
		for j in range(3):
			if i == 0:
				r[j,i] = x[j]
			elif i == 1:
				r[j,i] = y[j]
			else:
				r[j,i] = z[j]

	t = F_obj_frame.p - F_bin_frame.p

	f = kdl.Frame(r, t)
	graspFrame = posemath.toTf(f)

	while not rospy.is_shutdown():
		br.sendTransform(graspFrame[0], graspFrame[1], \
                 rospy.Time.now(), \
                 "/grasp_frame",  \
                 "/shelf_bin_H" \
                 )


def getGraspFrame(shelf_bin, object_name):
	listener = tf.TransformListener()
	while not rospy.is_shutdown():
		try:
			bin_frame = listener.lookupTransform('/base_link', shelf_bin, rospy.Time(0))
			obj_frame = listener.lookupTransform('/base_link', object_name, rospy.Time(0))
			break
		except:
			continue


	approchVec, approchDir = getApprochVec(bin_frame, obj_frame)
	graspingVec, graspingDir = getGraspingAxis(approchVec, bin_frame, obj_frame)

	F_bin_frame = posemath.fromTf(bin_frame)
	F_obj_frame = posemath.fromTf(obj_frame)

	objRed = F_obj_frame.M * kdl.Vector(1.0, 0.0, 0.0)
	objGreen = F_obj_frame.M * kdl.Vector(0.0, 1.0, 0.0)
	objBlue = F_obj_frame.M * kdl.Vector(0.0, 0.0, 1.0)

	objAxes = [objRed, objGreen, objBlue]

	x = [a*approchDir for a in objAxes[approchVec]]
	z = [g*graspingDir for g in objAxes[graspingVec]]

	y = [a*b for a,b in zip(z,x)]
	

	r = kdl.Rotation()
	for i in range(3):
		for j in range(3):
			if i == 0:
				r[j,i] = x[j]
			elif i == 1:
				r[j,i] = y[j]
			else:
				r[j,i] = z[j]

	t = F_obj_frame.p - F_bin_frame.p

	f = kdl.Frame(r, t)
	graspFrame = posemath.toTf(f)

	return graspFrame




if __name__ == "__main__":

	rospy.init_node('grasping_lib_test')
	listener = tf.TransformListener()
	while not rospy.is_shutdown():
		try:
			bin_frame = listener.lookupTransform('/base_link', "/shelf_bin_H", rospy.Time(0))
			obj_frame = listener.lookupTransform('/base_link', "/amazon_cheeze", rospy.Time(0))
			break
		except:
			continue


	publishGraspingFrame(bin_frame, obj_frame)