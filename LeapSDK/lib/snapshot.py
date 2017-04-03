import cv2, Leap, math, ctypes
import numpy as np
import cPickle as pickle 

from supportFunctions import *

#Custom class to magage leap motion code:
class leap(object):
	def __init__(self):
		self.controller = Leap.Controller()
		#Allow access to raw images:
		self.controller.set_policy(Leap.Controller.POLICY_IMAGES)
		self.images = []

	def takeSnaphot(self):
		frame = self.controller.frame()
		#Just grab left image
		image = frame.images[0]

		#wrap image data in numpy array
		i_address = int(image.data_pointer)
		ctype_array_def = ctypes.c_ubyte * image.height * image.width
		# as ctypes array
		as_ctype_array = ctype_array_def.from_address(i_address)
		# as numpy array
		as_numpy_array = np.ctypeslib.as_array(as_ctype_array)
		rawImage = np.reshape(as_numpy_array, (image.height, image.width))

		left_coordinates, left_coefficients = convert_distortion_maps(frame.images[0])

		destination = cv2.remap(rawImage, left_coordinates, left_coefficients, interpolation = cv2.INTER_LINEAR)
		destination = cv2.resize(destination,(400, 400), 0, 0, cv2.INTER_LINEAR)

		self.images.append(destination)

		print str(len(self.images)) + ' images captured!'


	def pickleImages(self, fileName):
		pickleFileName = fileName + ".pickle"
		pickleFile = open(pickleFileName, 'wb')
		pickle.dump(images, pickleFile, pickle.HIGHEST_PROTOCOL)
		pickleFile.close()