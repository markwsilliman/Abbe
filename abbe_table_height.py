#!/usr/bin/env python
import numpy as np
import math
import yaml
import cv2
from cv_bridge import CvBridge

import rospy
from sensor_msgs.msg import Image
from baxter_ikhelper import IKHelper
from baxter_core_msgs.msg import ITBState
from baxter_interface import (
    RobotEnable,
    DigitalIO,
    Gripper
)

class Abbe_Table_Height(object):
	_Z_START = 0.0
	
	def __init__(self):
		self._z = self._Z_START
		self.ik = IKHelper()

	def _reset(self):
	        self._z = self._Z_START
	        self.ik.set_right(0.5, 0.0, self._z, True)	

if __name__ == '__main__':
    rospy.init_node('Abbe_Table_Height', anonymous=True)
    Abbe_TH = Abbe_Table_Height()	
    Abbe_TH._reset()
