#!/usr/bin/env python

#
# Controls the pan / tilt of Abbe's face
#

import random
import numpy as np
import math
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image, Range
from baxter_core_msgs.msg import ITBState
from baxter_interface import (
	RobotEnable,
	AnalogIO,
	DigitalIO,
	Gripper
)

class Abbe_Gripper(object):
	
	def __init__(self):
		self._gripper = Gripper('left')
		self._gripper.calibrate()
	def close(self):
		self._gripper.close(block=True)

	def open(self):
		self._gripper.open(block=True)


if __name__ == '__main__':
	rospy.init_node("abbe_gripper")
	gripper = Abbe_Gripper()
	gripper.close()
