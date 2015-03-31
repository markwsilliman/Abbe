# https://github.com/Knifa/Glasgow-Baxter/blob/795038e951a592c7345f59cbb0d559b304e28427/src/glasgow_baxter/scripts/block_stacker/perception.py
#!/usr/bin/env python

import random

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

class Abbe_Block_Finder(object):
	SHAPE_STD_LIMIT = 3.0
	SHAPE_AREA_PERCENT_LIMIT = 0.2
	SHAPE_MIN_DISTANCE = 32
	SHAPE_ARC = 0.025
	RES_PERCENT = 0.66
	CANNY_DILATE = 3
	BLOCKS = []

	def __init__(self):
		self._blocks_found = 0
		self._bridge = CvBridge()
		self._table = Abbe_Table()
		self._table.default()
		

	def init_b(self):
		cv2.namedWindow('Left Arm')
		cv2.namedWindow('Right Arm')
		# Instantiate all three cameras
		# Only 2 cameras may be on at one time
		try:
			self._head_camera = baxter_interface.CameraController('head_camera')
			self._head_camera.close()
		except Exception:
			pass

		self._left_camera = baxter_interface.CameraController('left_hand_camera')		
		self._left_camera.open()

		self._left_camera_sub = rospy.Subscriber(
           		 '/cameras/left_hand_camera/image', 
            		Image,
            		self.left_img_received)	

		self._right_camera = baxter_interface.CameraController('right_hand_camera')		
		self._right_camera.open()

		self._right_camera_sub = rospy.Subscriber(
           		 '/cameras/right_hand_camera/image', 
            		Image,
            		self.right_img_received)	

	def right_img_received(self,data):
		try:
			img = self._bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
		except CvBridgeError, e:
			print e

		img = cv2.resize(img, (640, 400))
		if cv2.waitKey(1) == -1:
			cv2.imshow('Right Arm', img)

	def left_img_received(self,data):
		try:
			img = self._bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
		except CvBridgeError, e:
			print e

		img = cv2.resize(img, (640, 400))
		img = cv2.bilateralFilter(img, 3, 15, 15)
		if cv2.waitKey(1) == -1:
			cv2.imshow('Left Arm', cv2.Canny(img, 50, 150))

		self._find_blocks(img)

	def _find_blocks(self, img):
		contours = self._find_contours(img)
		squares = self._find_squares_from_contours(contours)
		self.BLOCKS = squares

	def _find_contours(self, img):
		# Detect contours in the image.
		canny_img = cv2.Canny(img, 50, 150)
		canny_img = cv2.dilate(canny_img, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, 
		    (self.CANNY_DILATE, self.CANNY_DILATE)))
		(contours, _) = cv2.findContours(canny_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

		# Smooth out the contours a little.
		approx_contours = []
		for c in contours:
		    arc = cv2.arcLength(c, True)
		    approx = cv2.approxPolyDP(c, self.SHAPE_ARC * arc, True)
		    approx_contours.append(approx)

		return approx_contours	
	
	def _find_squares_from_contours(self, contours):
		squares = []

		for c in contours:
		    # Drop anything that doesn't have four corners (obviously.)	             

		    if not len(c) == 4:
		        continue

		    # Find the distances between each side
		    sides = []
		    #for i in range(len(c)):
		    #    sides.append((c[i][0], c[(i + 1) % len(c)][0]))
		    #d = [euclidean(s[0], s[1]) for s in sides]

		    # Drop any contours that doesn't have about the same size sides.
		    #d_std = np.std(d)
		    #if not (d_std > 0.0 and d_std < self.SHAPE_STD_LIMIT):
		    #    continue

		    # Drop any contours where the area isn't about the same value as a square with sides
		    # equaling to the average side length.
		    #area = cv2.contourArea(c) 
		    #square_area = np.mean(d) ** 2
		    #area_dif = abs(square_area - area)
		    #if not area_dif <= square_area * self.SHAPE_AREA_PERCENT_LIMIT:
		    #    continue
			
		    squares.append(Square(c))

		return squares

#if __name__ == '__main__':
#    rospy.init_node('Abbe_Block_finder', anonymous=True)
#    Abbe_BF = 	Abbe_Block_Finder()	
#    rospy.spin()
