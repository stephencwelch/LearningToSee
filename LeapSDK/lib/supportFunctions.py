import cv2, Leap, math, ctypes
import numpy as np
from time import sleep
import cPickle as pickle 

#Custom class to control leap motion code:
class LeapController(object):
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
		pickle.dump(self.images, pickleFile, pickle.HIGHEST_PROTOCOL)
		pickleFile.close()



def convert_distortion_maps(image):

	distortion_length = image.distortion_width * image.distortion_height
	xmap = np.zeros(distortion_length/2, dtype=np.float32)
	ymap = np.zeros(distortion_length/2, dtype=np.float32)

	for i in range(0, distortion_length, 2):
			xmap[distortion_length/2 - i/2 - 1] = image.distortion[i] * image.width
			ymap[distortion_length/2 - i/2 - 1] = image.distortion[i + 1] * image.height

	xmap = np.reshape(xmap, (image.distortion_height, image.distortion_width/2))
	ymap = np.reshape(ymap, (image.distortion_height, image.distortion_width/2))

	#resize the distortion map to equal desired destination image size
	resized_xmap = cv2.resize(xmap,
														(image.width, image.height),
														0, 0,
														cv2.INTER_LINEAR)
	resized_ymap = cv2.resize(ymap,
														(image.width, image.height),
														0, 0,
														cv2.INTER_LINEAR)

	#Use faster fixed point maps
	coordinate_map, interpolation_coefficients = cv2.convertMaps(resized_xmap,
																															 resized_ymap,
																															 cv2.CV_32FC1,
																															 nninterpolation = False)

	return coordinate_map, interpolation_coefficients

def undistort(image, coordinate_map, coefficient_map, width, height):
	destination = np.empty((width, height), dtype = np.ubyte)

	#wrap image data in numpy array
	i_address = int(image.data_pointer)
	ctype_array_def = ctypes.c_ubyte * image.height * image.width
	# as ctypes array
	as_ctype_array = ctype_array_def.from_address(i_address)
	# as numpy array
	as_numpy_array = np.ctypeslib.as_array(as_ctype_array)
	img = np.reshape(as_numpy_array, (image.height, image.width))

	#remap image to destination
	destination = cv2.remap(img,
													coordinate_map,
													coefficient_map,
													interpolation = cv2.INTER_LINEAR)

	#resize output to desired destination size
	destination = cv2.resize(destination,
													 (width, height),
													 0, 0,
													 cv2.INTER_LINEAR)
	return destination

def undistortNumpyArray(img, coordinate_map, coefficient_map, width, height):
	destination = np.empty((width, height), dtype = np.ubyte)
	#remap image to destination
	destination = cv2.remap(img,
													coordinate_map,
													coefficient_map,
													interpolation = cv2.INTER_LINEAR)

	#resize output to desired destination size
	destination = cv2.resize(destination,
													 (width, height),
													 0, 0,
													 cv2.INTER_LINEAR)

	return destination

def convertRawImage(img, frame, image):
	maps_initialized = False

	if image.is_valid:
	    if not maps_initialized:
		   left_coordinates, left_coefficients = convert_distortion_maps(frame.images[0])
		   right_coordinates, right_coefficients = convert_distortion_maps(frame.images[1])
		   maps_initialized = True

	    undistorted_left = undistortNumpyArray(img, left_coordinates, left_coefficients, 400, 400)
	    return undistorted_left
	
	else:
		return 0


def convertImages(frame, image):
	maps_initialized = False

	if image.is_valid:
	    if not maps_initialized:
		   left_coordinates, left_coefficients = convert_distortion_maps(frame.images[0])
		   right_coordinates, right_coefficients = convert_distortion_maps(frame.images[1])
		   maps_initialized = True

	    undistorted_left = undistort(image, left_coordinates, left_coefficients, 400, 400)
	    undistorted_right = undistort(image, right_coordinates, right_coefficients, 400, 400)

	    return undistorted_left, undistorted_right
	
	else:
		return 0

def snapNSave(picCount = 0, directory = 'pictures/'):
	controller = Leap.Controller()
	#Allow access to raw images:
	controller.set_policy(Leap.Controller.POLICY_IMAGES)

	sleep(0.1)

	frame = controller.frame()
	image = frame.images[0]

	[left, right] = convertImages(frame,image)
	cv2.imwrite(directory + 'left' + str(picCount) + '.png', left)
	cv2.imwrite(directory + 'right' + str(picCount) + '.png', right)
	picCount = picCount +1
	
	return picCount


def getModelParts(frame):

	finger_names = ['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']
	bone_names = ['Metacarpal', 'Proximal', 'Intermediate', 'Distal']

	# Get hands
	for hand in frame.hands:

	    handType = "Left hand" if hand.is_left else "Right hand"

	    print "  %s, id %d, position: %s" % (
	        handType, hand.id, hand.palm_position)

	    # Get the hand's normal vector and direction
	    normal = hand.palm_normal
	    direction = hand.direction

	    # Calculate the hand's pitch, roll, and yaw angles
	    print "  pitch: %f degrees, roll: %f degrees, yaw: %f degrees" % (
	        direction.pitch * Leap.RAD_TO_DEG,
	        normal.roll * Leap.RAD_TO_DEG,
	        direction.yaw * Leap.RAD_TO_DEG)

	    # Get arm bone
	    arm = hand.arm
	    print "  Arm direction: %s, wrist position: %s, elbow position: %s" % (
	        arm.direction,
	        arm.wrist_position,
	        arm.elbow_position)

	    # Get fingers
	    for finger in hand.fingers:

	        print "    %s finger, id: %d, length: %fmm, width: %fmm" % (
	            finger_names[finger.type],
	            finger.id,
	            finger.length,
	            finger.width)

	        # Get bones
	        for b in range(0, 4):
	            bone = finger.bone(b)
	            print "      Bone: %s, start: %s, end: %s, direction: %s" % (
	                bone_names[bone.type],
	                bone.prev_joint,
	                bone.next_joint,
	                bone.direction)

	# Get tools
	for tool in frame.tools:

	    print "  Tool id: %d, position: %s, direction: %s" % (
	        tool.id, tool.tip_position, tool.direction)

def takeSnapshotWithFinger(numFingers=1, picCount = 0, directory = 'Pickles/'):

	controller = Leap.Controller()
	#Allow access to raw images:
	controller.set_policy(Leap.Controller.POLICY_IMAGES)

	#Chill for a tenth of a second:
	sleep(0.1)

	frame = controller.frame()
	image = frame.images[0]

	# [left, right] = convertImages(frame,image)

	#wrap image data in numpy array
	i_address = int(image.data_pointer)
	ctype_array_def = ctypes.c_ubyte * image.height * image.width
	# as ctypes array
	as_ctype_array = ctype_array_def.from_address(i_address)
	# as numpy array
	as_numpy_array = np.ctypeslib.as_array(as_ctype_array)
	rawImage = np.reshape(as_numpy_array, (image.height, image.width))

	left_coordinates, left_coefficients = convert_distortion_maps(frame.images[0])
	right_coordinates, right_coefficients = convert_distortion_maps(frame.images[1])

	

	trackingIndices = {}

	for fingerNum in np.arange(1, numFingers+1):
		#Find Points at end of finger and base:
		horizontal_slope = -1*(frame.hands[0].fingers[fingerNum].tip_position.x-20)/frame.hands[0].fingers[fingerNum].tip_position.y
		vertical_slope = frame.hands[0].fingers[fingerNum].tip_position.z/frame.hands[0].fingers[fingerNum].tip_position.y

		pixel = image.warp(Leap.Vector(horizontal_slope, vertical_slope, 0))
		print pixel.x, pixel.y
		pixelIndices = [np.round(pixel.y), np.round(pixel.x)]

		temp = np.zeros((image.height, image.width))
		temp[pixelIndices[0], pixelIndices[1]] = 255

		tempMapped = cv2.remap(temp, left_coordinates, left_coefficients, interpolation = cv2.INTER_LINEAR)
		tempMapped = cv2.resize(tempMapped,(400, 400), 0, 0, cv2.INTER_LINEAR)

		tipIndices = np.where(tempMapped>1)

		trackingIndices[str(fingerNum)] = {}
		trackingIndices[str(fingerNum)]['tipIndices'] = tipIndices

		#Find bone at other end of finger:
		bone = frame.hands[0].fingers[fingerNum].bone(2).prev_joint
		horizontal_slope = -1*(bone.x-20)/bone.y
		vertical_slope = bone.z/bone.y

		pixel = image.warp(Leap.Vector(horizontal_slope, vertical_slope, 0))
		print pixel.x, pixel.y
		pixelIndices = [np.round(pixel.y), np.round(pixel.x)]

		temp = np.zeros((image.height, image.width))
		temp[pixelIndices[0], pixelIndices[1]] = 255

		tempMapped = cv2.remap(temp, left_coordinates, left_coefficients, interpolation = cv2.INTER_LINEAR)
		tempMapped = cv2.resize(tempMapped,(400, 400), 0, 0, cv2.INTER_LINEAR)

		baseIndices = np.where(tempMapped>1)
		trackingIndices[str(fingerNum)]['baseIndices'] = baseIndices


	destination = cv2.remap(rawImage, left_coordinates, left_coefficients, interpolation = cv2.INTER_LINEAR)
	destination = cv2.resize(destination,(400, 400), 0, 0, cv2.INTER_LINEAR)
	#cv2.imwrite('mergedImage.png', mergedImage)
	#cv2.imwrite(directory + str(numFingers) + 'f' + str(picCount) + '.png', destination)
	#cv2.imwrite('cleanLabeledImage.png', cleanLabeledImage)

	#Wrap things up nicely
	imageDict = {}
	imageDict['image'] = destination
	imageDict['trackingIndices'] = trackingIndices
	imageDict['picCount']= picCount
	imageDict['numFingers']= numFingers

	pickleName = str(numFingers) + 'f' + str(picCount)
	pickleFileName = directory + pickleName + ".pickle"
	pickleFile = open(pickleFileName, 'wb')
	pickle.dump(imageDict, pickleFile, pickle.HIGHEST_PROTOCOL)
	pickleFile.close()

	return picCount +1

















