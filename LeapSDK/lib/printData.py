import Leap, cv2
from supportFunctions import *
from time import sleep
import numpy as np

controller = Leap.Controller()
#Allow access to raw images:
controller.set_policy(Leap.Controller.POLICY_IMAGES)

sleep(0.1)



#getModelParts(frame)

for i in range(100):
	frame = controller.frame()
	image = frame.images[0]

	for hand in frame.hands:
		#For left camera:
		horizontal_slope = -1*(hand.fingers[1].tip_position.x+20)/hand.fingers[1].tip_position.y
		vertical_slope = hand.fingers[1].tip_position.z/hand.fingers[1].tip_position.y

		ray = [horizontal_slope*image.ray_scale_x + image.ray_offset_x, \
			vertical_slope*image.ray_scale_y + image.ray_offset_y]

		pixel = [ray[0]*400, ray[1]*400]
		
		# vertical_slope = hand.fingers[1].tip_position.y/(hand.fingers[1].tip_position.x+20)
		# horizontal_slope = hand.fingers[1].tip_position.z/(hand.fingers[1].tip_position.x+20)

		#pixel = image.warp(Leap.Vector(horizontal_slope, vertical_slope, 0))

		print 'x-position: ' + str(frame.hands[0].fingers[1].tip_position.x)
		print 'y-position: ' + str(frame.hands[0].fingers[1].tip_position.y)
		print 'z-position: ' + str(frame.hands[0].fingers[1].tip_position.z)

		print 'Horizontal Slope: ' + str(horizontal_slope)
		print 'Vertical Slope: ' + str(vertical_slope)

		print 'Pixel Location x: ' + str(pixel[0])
		print 'Pixel Location y: ' + str(pixel[1])

	sleep(1)