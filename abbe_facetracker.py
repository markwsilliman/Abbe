# Heavily inspired by OpenCV Computer Vision with Python by Joseph Howse
#!/usr/bin/env python

import random

import numpy as np
import math

import cv2
from cv_bridge import CvBridge, CvBridgeError

import rospy
from sensor_msgs.msg import Image, Range

from baxter_core_msgs.msg import ITBState
import baxter_interface
from abbe_errors import Abbe_Error
from abbe_face import Abbe_Face

class Abbe_FaceTracker(object):
	
	def __init__(self):
		
		self._cvbr = CvBridge()
		self.left_img = None
		self._bridge = CvBridge()
		self._abbe_face = Abbe_Face()
		cv2.namedWindow('Head Camera')
		self._currently_panning = False #avoid duplicate pan requests

		# Load all HAARs for facial recognition	
		self._faceClassifier = cv2.CascadeClassifier(
        	    'cascades/haarcascade_frontalface_alt.xml')
		

		# Instantiate all three cameras
		# Only 2 cameras may be on at one time
		try:
			self._right_camera = baxter_interface.CameraController('right_hand_camera')
			self._right_camera.close()
		except Exception:
			pass

		self._head_camera = baxter_interface.CameraController('head_camera')		
		self._head_camera.open()
		self._head_camera.resolution = [640, 400]

		self._head_camera_sub = rospy.Subscriber(
           		 '/cameras/head_camera/image', 
            		Image,
            		self.on_head_imagemsg_received)

	def on_head_imagemsg_received(self, data):
		try:
			cv_image = self._bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
		except CvBridgeError, e:
			print e

	        img = cv2.resize(cv_image, (640, 400))

		if cv2.waitKey(1) == -1:
			cv2.imshow('Head Camera', img)
		
		self.find_a_face(img)

	def find_a_face(self, img):
		if(self._currently_panning):
			return False
		
		faceRects = self._faceClassifier.detectMultiScale(
            		img, 1.2, 2, cv2.cv.CV_HAAR_SCALE_IMAGE,
		        self.widthHeightDividedBy(img, 8))

		if faceRects is not None:
		    for faceRect in faceRects:
		        
		        face = Face()
		        face.faceRect = faceRect
		        
		        x, y, w, h = faceRect

			# add 1/2 w to x?
			_ratio = float((float(x) + (float(w)/2.0)) / 640.0 * 100.0)
			self._currently_panning = True
			if(_ratio < 48):
				self._abbe_face.pan_a_little_to_the_left()
			if(_ratio > 52):
				self._abbe_face.pan_a_little_to_the_right()
			self._currently_panning = False
	
	def widthHeightDividedBy(self,image, divisor):
	    """Return an image's dimensions, divided by a value."""
	    h, w = image.shape[:2]
	    return (w/divisor, h/divisor)        

class Face(object):
    """Data on facial features: face, eyes, nose, mouth."""
    
    def __init__(self):
        self.faceRect = None
        self.leftEyeRect = None
        self.rightEyeRect = None
        self.noseRect = None
        self.mouthRect = None


if __name__ == '__main__':
    rospy.init_node('Abbe_FaceTracker', anonymous=True)
    Abbe_FT = 	Abbe_FaceTracker()	
    rospy.spin()
