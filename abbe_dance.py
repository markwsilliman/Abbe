#!/usr/bin/env python

#
# Give the right hand something to do before we have 2x grippers
#

import math
import rospy
import argparse
import struct
import sys
from abbe_ik import Abbe_IK
import threading
import time

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from tf import transformations
from std_msgs.msg import Header, UInt16
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

import baxter_interface
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from sensor_msgs.msg import Range

class Abbe_Dance(object):

	def __init__(self):
		self._right = baxter_interface.Limb('right')
	
	def dance(self):
		angles = self._right.joint_angles()
		angles['right_s0']=-0.2
		angles['right_s1']=0.0
		angles['right_e0']=0.0
		angles['right_e1']=0.0
		angles['right_w0']=0.0
		angles['right_w1']=0.0
		angles['right_w2']=0.0

		self._right.move_to_joint_positions(angles)
		print self._right.joint_angles()

	def dance_2(self):
		angles = self._right.joint_angles()
		angles['right_s0']=-0.125
		angles['right_s1']=-1
		angles['right_e0']=-1.184
		angles['right_e1']=0
		angles['right_w0']=0
		angles['right_w1']=0
		angles['right_w2']=-0.782

		self._right.move_to_joint_positions(angles)
		print self._right.joint_angles()

	def dance_3(self):
		angles = self._right.joint_angles()
		angles['right_s0']=-1.700
		angles['right_s1']=-0.7324
		angles['right_e0']=-0.606
		angles['right_e1']=0.8532
		angles['right_w0']= 2.0045
		angles['right_w1']=-0.849
		angles['right_w2']=-1.4469

		self._right.move_to_joint_positions(angles)
		print self._right.joint_angles()

	def dance_4(self):
		angles = self._right.joint_angles()
		angles['right_s0']=-1.700
		angles['right_s1']=0.7324
		angles['right_e0']=-0.606
		angles['right_e1']=0.8532
		angles['right_w0']= 2.0045
		angles['right_w1']=-0.849
		angles['right_w2']=-1.4469

		self._right.move_to_joint_positions(angles)
		print self._right.joint_angles()

	def print_pos(self):
		print self._right.joint_angles()

if __name__ == '__main__':
    rospy.init_node('Abbe_Table', anonymous=True)
    n = Abbe_Dance()
    while not rospy.is_shutdown():
    	n.dance()
    	n.dance_4()
    	n.dance_2()
    	n.dance_3()
