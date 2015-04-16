#!/usr/bin/env python
import moveit_commander

import rospy

import actionlib

import amazon_challenge_bt_actions.msg

from std_msgs.msg import String
import sys

import tf


class superDetector(object):
    # create messages that are used to publish feedback/result
    _feedback = amazon_challenge_bt_actions.msg.DetectorFeedback()
    _result   = amazon_challenge_bt_actions.msg.DetectorResult()

    def __init__(self, name):
        self._action_name = name
        rospy.init_node(self._action_name)
        self._as = actionlib.SimpleActionServer(self._action_name, amazon_challenge_bt_actions.msg.DetectorAction,\
            execute_cb=self.receive_update, auto_start = False)

        self._as.start()
        self.pub_rate = rospy.Rate(30)
        self.listener = tf.TransformListener()
        rospy.Subscriber("/amazon_next_task", String, self.get_task)
        self._item = ""
        self._bin = ""
        self.trials = 100
        self.br = tf.TransformBroadcaster()
        self.tp = []

    def flush(self):
        self._item = ""
        self._bin = ""




    def my_pub(self):
        # publish info to the console for the user
        rospy.loginfo('Starting Detecting')



        while not rospy.is_shutdown():
            if len(self._item) == 0 or len(self._bin) == 0:
                continue
            try:
                self.br.sendTransform(self.tp[0], self.tp[1], rospy.Time.now(),\
                                         "/" + self._item + "_detector",   \
                                         "/base_link")
                self.pub_rate.sleep()
            except:
                continue



    def get_task(self, msg):
        text = msg.data
        text = text.replace('[','')
        text = text.replace(']','')
        words = text.split(',')
        self._bin = words[0]
        self._item = words[1]

    def receive_update(self,goal):
        rospy.loginfo('Goal Received')
        while not rospy.is_shutdown():
            try:
                self.tp = self.listener.lookupTransform('/base_link', '/' + self._item, rospy.Time(0))
                rospy.loginfo('object pose UPDATED')
                self._feedback.status = 0 # a status 0 means that there are still objects in the list
                self._result.status = self._feedback.status
                self._as.set_succeeded(self._result)
                break
            except:
                continue



if __name__ == '__main__':
    rospy.init_node('detector')
    pubDetector = superDetector('detector')
    pubDetector.my_pub()
    rospy.spin()
