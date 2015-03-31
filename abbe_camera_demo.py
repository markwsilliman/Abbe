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

class Abbe_Camera_Demo(object):

	def __init__(self):
		self._bridge = CvBridge()
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

if __name__ == '__main__':
    rospy.init_node('Abbe_Camera_Demo', anonymous=True)
    Abbe_BF = 	Abbe_Camera_Demo()	
    rospy.spin()
