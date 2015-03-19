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

class Abbe_Table(object):
	_Z_START = 0.0
	_Z_BACKSTEP = 0.01
	_Z_ACCEPT_FORCE = -10.0
	_Z_TABLE_HEIGHT = 0.0

	def __init__(self):
		self._ik = Abbe_IK()	
		
	def default(self):
		self._ik.set_left(0.5,0.0,0.0)

	def droppoint(self,block_count = 10):
		_tmp_height = 0.2
		if(block_count <= 2):
			_tmp_height = 0.0

		self._ik.set_left(0.5,0.5,_tmp_height) #-0.18

	def ret_height_of_table(self):
		if(self._Z_TABLE_HEIGHT == 0.0):
			print "you probably didn't determine the height of the table yet!"
		return self._Z_TABLE_HEIGHT

	def ret_height_for_block_pickup(self):
		return self.ret_height_of_table() + 0.03

	def move_straight_up_height_default(self,limb):
		_pose = self._ik.get_pose(limb)
		self._ik.set_left(_pose.x,_pose.y,0.5)

	def moveto_height_for_blockpickup(self,limb):
		_pose = self._ik.get_pose(limb)
		self._ik.set_left(_pose.x,_pose.y,self.ret_height_for_block_pickup())

	def moveto_height_for_blockdropoff(self,limb,block_count):
		_pose = self._ik.get_pose(limb)

		#slow down
		self._ik.set_speed("left",0.4)
		_back_step = self._Z_BACKSTEP		

		self._ik.set_speed("left",0.1)

		_table_height = _pose.z
		
		_success = True
		while _success and not rospy.is_shutdown() and self._ik.get_force("left").z > -3:
			_table_height = _table_height - _back_step
			_success = self._ik.set_left(_pose.x,_pose.y,_table_height,True,1)	

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
