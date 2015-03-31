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
from abbe_face import Abbe_Face

if __name__ == '__main__':
	rospy.init_node('Abbe_Face', anonymous=True)
	face = Abbe_Face()
	while not rospy.is_shutdown():
		face.nod_do_it()
		face. left_do_it()
		face.nod_do_it()
		face.center_do_it()
		face.nod_do_it()
		face.right_do_it()	
