import cv2, Leap, math, ctypes
import numpy as np
import time
import cPickle as pickle 

from supportFunctions import *

controller = Leap.Controller()
#Allow access to raw images:
controller.set_policy(Leap.Controller.POLICY_IMAGES)

#Chill for a tenth of a second:
time.sleep(0.1)

numImages = 100
rawImages = []
images = []


startTime = time.time()

for i in range(numImages):
	frame = controller.frame()
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

	rawImages.append((left_coordinates, left_coefficients, rawImage))

elapsedTime = time.time()-startTime
print 'Done Capturing! ' + str(len(images)) + ' images captured in ' + str(elapsedTime) + ' seconds.'
print 'Effective Frame Rate = ' + str(float(len(images))/elapsedTime) + ' images/second.'

print 'Processing Images...'

for i in range(len(rawImages)):
	(left_coordinates, left_coefficients, rawImage) = rawImages[i]
	destination = cv2.remap(rawImage, left_coordinates, left_coefficients, interpolation = cv2.INTER_LINEAR)
	destination = cv2.resize(destination,(400, 400), 0, 0, cv2.INTER_LINEAR)
	images.append(destination)


pickleName = 'video'
pickleFileName = pickleName + ".pickle"
pickleFile = open(pickleFileName, 'wb')
pickle.dump(images, pickleFile, pickle.HIGHEST_PROTOCOL)
pickleFile.close()

elapsedTime = time.time()-startTime
print 'Done!' + ' Total time = ' + str(elapsedTime) + ' s.'