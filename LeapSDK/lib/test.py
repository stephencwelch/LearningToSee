import Leap, cv2
from supportFunctions import *
from time import sleep
import numpy as np

controller = Leap.Controller()
#Allow access to raw images:
controller.set_policy(Leap.Controller.POLICY_IMAGES)

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



#getModelParts(frame)

#For left camera:
# horizontal_slope = -1*(frame.hands[0].fingers[1].tip_position.x-20)/frame.hands[0].fingers[1].tip_position.y
# vertical_slope = -1*(frame.hands[0].fingers[1].tip_position.x-20)/frame.hands[0].fingers[1].tip_position.z

horizontal_slope = -1*(frame.hands[0].fingers[1].tip_position.x-20)/frame.hands[0].fingers[1].tip_position.y
vertical_slope = frame.hands[0].fingers[1].tip_position.z/frame.hands[0].fingers[1].tip_position.y

pixel = image.warp(Leap.Vector(horizontal_slope, vertical_slope, 0))

print pixel.x, pixel.y

if(pixel.x >= 0 and pixel.y >= 0 and pixel.x <= image.width and pixel.y <= image.height):
	print 'In Range, Converting!'
	pixelIndices = [math.floor(pixel.y), math.floor(pixel.x)]

	#Put images together into single colored image:
	mergedImage = np.zeros((image.height, image.width, 3))
	mergedImage[:, :, 0] = rawImage
	mergedImage[:, :, 1] = rawImage
	mergedImage[:, :, 2] = rawImage

	mergedImage[pixelIndices[0], pixelIndices[1], 2] = 255
	mergedImage[pixelIndices[0], pixelIndices[1], 0] = 0
	mergedImage[pixelIndices[0], pixelIndices[1], 1] = 0

	destination = cv2.remap(mergedImage, left_coordinates, left_coefficients, interpolation = cv2.INTER_LINEAR)
	destination = cv2.resize(destination,(400, 400), 0, 0, cv2.INTER_LINEAR)

	#mergedImage[pixelIndices[1], pixelIndices[0], 0] = 255

	cv2.imwrite('mergedImage.png', mergedImage)
	cv2.imwrite('mergedMapped.png', destination)
	#cv2.imwrite('cleanLabeledImage.png', cleanLabeledImage)



