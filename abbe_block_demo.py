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
from abbe_face import Abbe_Face


class Abbe_Block_Demo(object):
	TOLERANCE_FOR_BLOCK_POSE = 15	
	_Y_BACKSTEP = 0.01
	_X_BACKSTEP = 0.01
	_BLOCK_STACK_COUNT = 0

	def __init__(self):
		self._table = Abbe_Table()
		self._table.determine_height_of_table()
		self._ik = Abbe_IK()	
		self._block_finder = Abbe_Block_Finder()
		self._table.default_right()
		self._left_gripper = Abbe_Gripper()
		self._face = Abbe_Face()	

	def pickup_block(self):
		if self.load_a_new_block(): #len(self._block_finder.BLOCKS) > 0
			#print "block found"
			#block_c = self._block_finder.BLOCKS[0].pose()
			if True: #returns True when centered -- self.center_on_block(block_c)
				self._face.nod()
				#print "pick up now"
				self._left_gripper.open()
				self._ik.set_speed("left",0.5)				
				self._table.moveto_height_for_blockpickup("left")
				self._left_gripper.close()	
				_did_we_grip_block = self.did_the_gripper_pickup_a_block()

															

				if(_did_we_grip_block):	
					self._table.reload() #non-blocking	
					self._table.move_straight_up_height_default("left") #go straight up to avoid hitting anything else			
					self.stack_block("left")
																	
					

				self._ik.set_speed("left") #reset speed			
				
		return True

	def load_a_new_block(self):
		#move left hand out of the way

		success = True

		if self._left_gripper.gripping(False):	
			return True

		self._table.droppoint() #out of the way for a

		#load block using blocking methods
		if not self._left_gripper.gripping(False):	
			self._left_gripper.open(False)
			self._face.awake()
			self._face.right()
			self._table.load_block_so_were_ready()
			self._left_gripper.close(False)
				
		if self._left_gripper.gripping(False):						
			self._table.move_right_hand_up_some_after_grab()
			self._face.center()
			self._table.default_right_dropoff_initial()
			self._table.moveto_height_for_block_reload()
			self._left_gripper.open(False)
			#get right out  of the way
			self._face.right()
			self._table.move_right_straight_up_height_default()
			self._table.default_right()
		else:
			self._table.move_right_straight_up_height_default()
			self._face.confused()
			success = False

		if success:
			self._face.center()
			self._table.default()
		else:
			self._table.move_right_hand_up_some_after_grab()

		return success

	def did_the_gripper_pickup_a_block(self):
		return self._left_gripper.gripping()

	def stack_block(self,limb):
		self._face.left()
		self._ik.set_speed(limb,0.7)
		self._table.droppoint(self._BLOCK_STACK_COUNT)
		self._ik.set_speed(limb,0.2)
		_success = self._table.moveto_height_for_blockdropoff("left",self._BLOCK_STACK_COUNT)	
		self._BLOCK_STACK_COUNT = self._BLOCK_STACK_COUNT + 1		
		self._left_gripper.open()
		time.sleep(1) #make sure were open all the way before moving fast
		
		self._table.move_straight_up_height_default("left") #go straight up to avoid hitting any blocks
		self._face.center()
		self._ik.set_speed(limb)

	def center_on_block(self,block_pose,limb = "left"):
		return True
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
