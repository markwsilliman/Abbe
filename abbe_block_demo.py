#!/usr/bin/env python

#
# Block demo
#

import random
import time
import numpy as np
import math
from square import Square

import cv2
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.distance import euclidean

import rospy
from sensor_msgs.msg import Image, Range
import baxter_interface
from abbe_table import Abbe_Table
from abbe_ik import Abbe_IK
from abbe_block_finder import Abbe_Block_Finder
from abbe_gripper import Abbe_Gripper


class Abbe_Block_Demo(object):
	TOLERANCE_FOR_BLOCK_POSE = 15	
	_Y_BACKSTEP = 0.01
	_X_BACKSTEP = 0.01

	def __init__(self):
		self._table = Abbe_Table()
		self._table.determine_height_of_table()
		self._ik = Abbe_IK()	
		self._block_finder = Abbe_Block_Finder()
		self._left_gripper = Abbe_Gripper()

	def pickup_block(self):
		if(len(self._block_finder.BLOCKS) > 0):
			print "block found"
			block_c = self._block_finder.BLOCKS[0].pose()
			if self.center_on_block(block_c): #returns True when centered
				print "pick up now"
				self._left_gripper.open()
				self._table.moveto_height_for_blockpickup("left")
				self._left_gripper.close()
				self._ik.set_speed("left") #reset speed
				self._table.default()
				time.sleep(10)					
				
		return True
	
	def center_on_block(self,block_pose,limb = "left"):
		_x = block_pose[0]
		_y = block_pose[1]
		_hand_pose = self._ik.get_pose(limb)
		self._ik.set_speed(limb,0.02)

		#check how far we're off with X
		print "X is: " + str(_x)
		print "ABS is: " + str(abs(_x - (640/2)))
		if(abs(_x - (640/2)) > self.TOLERANCE_FOR_BLOCK_POSE):
			if(_x > (640/2)):
				print "adjust y +"
				self._ik.set_left(_hand_pose.x,_hand_pose.y + self._Y_BACKSTEP,_hand_pose.z)	
			else:
				print "adjust y -"
				self._ik.set_left(_hand_pose.x,_hand_pose.y - self._Y_BACKSTEP,_hand_pose.z)
		else:
			#x is good so now check Y
			print "Y is: " + str(_y)
			if(abs(_y - (400/2)) > self.TOLERANCE_FOR_BLOCK_POSE):
				if(_y > (400/2)):
					print "adjust x +"
					self._ik.set_left(_hand_pose.x + self._X_BACKSTEP,_hand_pose.y,_hand_pose.z)	
				else:
					print "adjust x -"
					self._ik.set_left(_hand_pose.x - self._X_BACKSTEP,_hand_pose.y,_hand_pose.z)
			else:	
				return True #we're locked on!

		
		return False


if __name__ == '__main__':
    rospy.init_node('Abbe_Block_Demo', anonymous=True)
    abd = Abbe_Block_Demo()
    while(not rospy.is_shutdown()):
	abd.pickup_block()
