#!/usr/bin/env python

#
# Detect Table Height
#

import math
import rospy
import argparse
import struct
import sys
from abbe_ik import Abbe_IK
import threading

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

from baxter_interface import RobotEnable, CameraController, Limb
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from sensor_msgs.msg import Range



class Abbe_Table(object):
	_Z_START = 0.0
	_Z_BACKSTEP = 0.01
	_Z_ACCEPT_FORCE = -10.0
	_Z_TABLE_HEIGHT = 0.0
	_Z_CURRENT_HEIGHT_FROM_IR_SENSOR = 0.0

	def __init__(self):
		self._ik = Abbe_IK()
		rospy.Subscriber("/robot/range/left_hand_range/state", Range, self.left_range_callback)
		
	def default(self):
		self._ik.set_left(0.5,0.0,0.0)			

	def default_right_dropoff_initial(self):
		self._ik.set_right(0.516,0.015,0.0)	

	def default_right(self):
		self._ik.set_right(-0.5,-0.7,0.0)		

	def left_range_callback(self,data):
		self._Z_CURRENT_HEIGHT_FROM_IR_SENSOR = data.range

	def droppoint(self,block_count = 10):
		_tmp_height = 0.1
		if(block_count <= 2):
			_tmp_height = 0.0

		self._ik.set_left(0.5,0.5,_tmp_height) #-0.18		

	def load_block(self):
		#self._load_block = threading.Timer(0, self.load_block_so_were_ready)
		#self._load_block.start()
		self.load_block_so_were_ready()
		
	def load_block_so_were_ready(self):
		self._ik.set_speed("right",0.7)
		self.default_right()
		self._ik.set_speed("right",0.3)
		self.moveto_height_for_blockpickup("right")
						

	def reload(self):
		self._sch_reload = threading.Timer(0, self.grab_the_next_block)
		self._sch_reload.start()

	def grab_the_next_block(self):
		return False
		self._ik.set_speed("right",0.7)		
		
		#only do the following if you aren't loaded yet
		self._ik.set_right(0.5,0.0,0.0)
		self._ik.set_right(0.5,-0.5,0.2)

	def ret_height_of_table(self):
		if(self._Z_TABLE_HEIGHT == 0.0):
			print "you probably didn't determine the height of the table yet!"
		return self._Z_TABLE_HEIGHT

	def ret_height_for_block_pickup(self):
		return self.ret_height_of_table() + 0.03

	def move_right_hand_up_some_after_grab(self):
		limb = "right"
		_pose = self._ik.get_pose(limb)
		self._ik.set_speed(limb,0.7)
		self._ik.set_right(_pose.x,_pose.y,_pose.z + 0.3)

	def move_straight_up_height_default(self,limb):
		_pose = self._ik.get_pose(limb)
		self._ik.set_speed(limb,0.1)
		self._ik.set_left(_pose.x,_pose.y,_pose.z + 0.1) #slowly up a little so we dont knock blocks
		self._ik.set_speed(limb,0.7)
		self._ik.set_left(_pose.x,_pose.y,0.5)

	def move_right_straight_up_height_default(self,limb = "right"):
		_pose = self._ik.get_pose(limb)
		self._ik.set_speed(limb,0.1)
		self._ik.set_right(_pose.x,_pose.y,_pose.z + 0.1) #slowly up a little so we dont knock blocks
		self._ik.set_speed(limb,0.7)
		self._ik.set_right(_pose.x,_pose.y,0.5)

	def moveto_height_for_blockpickup(self,limb):
		_pose = self._ik.get_pose(limb)
		if(limb == "left"):
			self._ik.set_left(_pose.x,_pose.y,self.ret_height_for_block_pickup())
		else:
			self._ik.set_right(_pose.x,_pose.y,self.ret_height_for_block_pickup() - 0.02)

	def moveto_height_for_blockdropoff(self,limb,block_count):
		_pose = self._ik.get_pose(limb)

		#slow down
		self._ik.set_speed("left",0.1)
		_back_step = self._Z_BACKSTEP		
		
		_table_height = _pose.z
		
		_success = True
		while _success and not rospy.is_shutdown() and self._ik.get_force("left").z > -3:
			#print self._Z_CURRENT_HEIGHT_FROM_IR_SENSOR

			_table_height = _table_height - _back_step
			_success = self._ik.set_left(_pose.x,_pose.y,_table_height,True,0.5)	

	def moveto_height_for_block_reload(self,limb = "right"):
		_pose = self._ik.get_pose(limb)

		_table_height = _pose.z
		self._ik.set_speed("right",0.1)
		_back_step = self._Z_BACKSTEP
		
		_success = True
		while _success and not rospy.is_shutdown() and self._ik.get_force("right").z > -3:
			_table_height = _table_height - _back_step
			_success = self._ik.set_right(_pose.x,_pose.y,_table_height,True,1)	


	def determine_height_of_table(self):
		self.default()

		#next 2 lines are a hack to save time... remove them to recalibrate
		self._Z_TABLE_HEIGHT = -0.17
		return True

		_table_height = self._Z_START
		
		#slow down
		self._ik.set_speed("left",0.1)

		print "Determine table height"

		_success = True
		while _success and not rospy.is_shutdown() and self._ik.get_force("left").z > self._Z_ACCEPT_FORCE:
			_table_height = _table_height - self._Z_BACKSTEP
			_success = self._ik.set_left(0.5,0.0,_table_height)		

		self._ik.set_timeout("left") #default
		self._Z_TABLE_HEIGHT = _table_height

		print "Table Height determined at: "  + str(self._Z_TABLE_HEIGHT)
		self._ik.set_speed("left") #reset speed to default
	
#if __name__ == '__main__':
#    rospy.init_node('Abbe_Table', anonymous=True)
#    at = Abbe_Table()
#    at.default()
#    at.determine_height_of_table()
#    at.default()
#    at.moveto_height_for_blockpickup("left")
#    at.default()
#    at.moveto_height_for_blockpickup("left")
