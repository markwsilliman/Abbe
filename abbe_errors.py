#!/usr/bin/env python

class Abbe_Error(object):

	def error(self,msg):
		raise Exception("Abbe Error: " + msg)
