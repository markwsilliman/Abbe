#!/usr/bin/env python

import cv2
import numpy as np

class Square(object):
	def __init__(self, contour):
		self.contour = np.int0(contour.reshape(4, 2))
		self.box = np.int0(cv2.cv.BoxPoints(cv2.minAreaRect(self.contour)))
		self.moments = cv2.moments(np.float32([self.box]))
		self.center = np.array([
			self.moments['m10'] / self.moments['m00'],
			self.moments['m01'] / self.moments['m00']])
		self.x = self.center[0] - 60
		self.y = self.center[1] + 100

	def pose(self):
		return [self.x,self.y]
