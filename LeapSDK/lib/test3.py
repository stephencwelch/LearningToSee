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

endPixel = image.warp(Leap.Vector(horizontal_slope, vertical_slope, 0))
endPixelIndices = [math.floor(endPixel.y), math.floor(endPixel.x)]

#Find bone at other end of finger:
bone = frame.hands[0].fingers[1].bone(2).prev_joint
horizontal_slope = -1*(bone.x-20)/bone.y
vertical_slope = bone.z/bone.y

startPixel = image.warp(Leap.Vector(horizontal_slope, vertical_slope, 0))
startPixelIndices = [math.floor(startPixel.y), math.floor(startPixel.x)]

fingerMask = np.zeros((image.height, image.width))
fingerMask[startPixelIndices[0], startPixelIndices[1]] = 255
fingerMask[endPixelIndices[0], endPixelIndices[1]] = 255

mappedMask = cv2.remap(fingerMask, left_coordinates, left_coefficients, interpolation = cv2.INTER_LINEAR)
mappedMask = cv2.resize(mappedMask,(400, 400), 0, 0, cv2.INTER_LINEAR)



#Shifting these endpoints into a region that represents the finger is going to be tough!
grayScaleImage = cv2.remap(rawImage, left_coordinates, left_coefficients, interpolation = cv2.INTER_LINEAR)
grayScaleImage = cv2.resize(grayScaleImage,(400, 400), 0, 0, cv2.INTER_LINEAR)

#mergedImage[pixelIndices[1], pixelIndices[0], 0] = 255

cv2.imwrite('grayScaleImage.png', grayScaleImage)




