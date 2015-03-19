#!/usr/bin/env python

#
# Controls the pan / tilt of Abbe's face
#

import argparse
import random
import rospy
import baxter_interface
from abbe_errors import Abbe_Error
from abbe_emotion import Abbe_Emotions
import threading

class Abbe_Face(object):
	

	def __init__(self):
		self._pan_value = 0		
		self._head = baxter_interface.Head()		
		self._emotion = Abbe_Emotions()
		self._error = Abbe_Error() #for throwing excpetions
		self.pan(0.0) #default
	
	def pan_a_little_to_the_left(self):
		if (self._pan_value + 0.05 > 0.7):
			return False
		self.pan(self._pan_value + 0.05)
		return True

	def pan_a_little_to_the_right(self):
		if (self._pan_value - 0.1 < -0.7):
			return False
		self.pan(self._pan_value - 0.1)
		return True
	
	def confused(self):
		self._emotion.emotion("confused")
		
	def nod_do_it(self):
		self._emotion.emotion("happy")
		self._head.command_nod()
		self._emotion.emotion("awake")

	def nod(self):
		if(not rospy.is_shutdown()):
			self._sch_nod = threading.Timer(0, self.nod_do_it)
			self._sch_nod.start()

	def pan(self,angle):
		control_rate = rospy.Rate(100)
		command_rate = rospy.Rate(1)

		while (not rospy.is_shutdown() and not (abs(self._head.pan() - angle) <= baxter_interface.HEAD_PAN_ANGLE_TOLERANCE)):
			self._head.set_pan(angle, speed=30, timeout=0)
			control_rate.sleep()
		self._pan_value = angle
		command_rate.sleep()

	def left(self):
		if(not rospy.is_shutdown()):
			self._sch_left = threading.Timer(0, self.left_do_it)
			self._sch_left.start()	

	def left_do_it(self):
		self._emotion.eye_direction("left")
		self.pan(0.7)
		self._emotion.eye_direction("center")

	def right(self):
		if(not rospy.is_shutdown()):
			self._sch_right = threading.Timer(0, self.right_do_it)
			self._sch_right.start()

	def right_do_it(self):
		self._emotion.eye_direction("right")
		self.pan(-0.7)
		self._emotion.eye_direction("center")

	def center(self):
		if(not rospy.is_shutdown()):
			self._sch_center = threading.Timer(0, self.center_do_it)
			self._sch_center.start()

	def center_do_it(self):
		self.pan(0.0)

	
		

#if __name__ == '__main__':
#	rospy.init_node('Abbe_Face', anonymous=True)
#	face = Abbe_Face()
#	face.nod()
#	face.left()
#	face.nod()
#	face.center()
#	face.nod()
#	face.right()	
#	face.nod()
#	rospy.spin()
		
