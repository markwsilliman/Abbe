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
		self._ik.set_left(0.5,0.0,0)

	def ret_height_of_table(self):
		if(self._Z_TABLE_HEIGHT == 0.0):
			print "you probably didn't determine the height of the table yet!"
		return self._Z_TABLE_HEIGHT

	def ret_height_for_block_pickup(self):
		return self.ret_height_of_table() + 0.01

	def moveto_height_for_blockpickup(self,limb):
		_pose = self._ik.get_pose(limb)
		self._ik.set_left(_pose.x,_pose.y,self.ret_height_for_block_pickup())

	def determine_height_of_table(self):
		_table_height = self._Z_START
		
		#slow down
		self._ik.set_speed("left",0.1)

		print "Determine table height"

		_success = True
		while _success and not rospy.is_shutdown() and self._ik.get_force("left").z > self._Z_ACCEPT_FORCE:
			_table_height = _table_height - self._Z_BACKSTEP
			_success = self._ik.set_left(0.5,0.0,_table_height)		

		self._Z_TABLE_HEIGHT = _table_height

		print "Table Height determined at: "  + str(self._Z_TABLE_HEIGHT)
		self._ik.set_speed("left") #reset speed to default
	
if __name__ == '__main__':
    rospy.init_node('Abbe_Table', anonymous=True)
    at = Abbe_Table()
    at.default()
    at.determine_height_of_table()
    at.default()
    at.moveto_height_for_blockpickup("left")
    at.default()
    at.moveto_height_for_blockpickup("left")
