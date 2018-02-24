import time
import argparse

import cv2
import serial
from numpy import array
from matplotlib import pyplot as pl

from config.environment import RealWorld
from tracking.ball import Ball
from tracking.path import Path
from physics.models import free_fall
from actuators.motor import Motor


def main(source, real_world):
	############################
	#   MOTOR INITIALISATION   #
	############################

	arduino_console = serial.Serial('COM3', 9600, timeout=1, write_timeout=2)

	belt_motor = Motor(
		arduino_console, 
		max_speed=real_world.motor_max_speed
	)
	# avoids some bugs with serial
	time.sleep(0.5)


	###########################
	#  CAMERA INITIALISATION  #
	###########################

	WHITE_LOWER = (240, 240, 240)
	WHITE_UPPER = (255, 255, 255)

	ORANGE_LOWER = (255, 122, 0)
	ORANGE_UPPER = (255, 200, 0)

	upper = array([x + 20 for x in real_world.object_color])
	lower = array([x - 20 for x in real_world.object_color])

	ball = Ball(source, (lower, upper), max_retries=1000)
	ball.start_positionning()
	rail_origin = real_world.dist_origin_rails


	###################
	#  BALL TRACKING  #
	###################
	
	i = 0
	positions = []
	x_falls = []

	# Wait for the ball to appear
	while not ball.is_in_range:
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	print("BALL FOUND")

	# Get the first position (to compute initial speed)
	while not ball.is_in_range:
		pass

	positions.append(ball.position)

	while ball.is_in_range:
		positions.append(ball.position)

		# Create the functions describing the position of the
		# ball with x axis
		f = free_fall(
			[positions[-1], positions[-2]],
			real_world.px_m_ratio
		)
		path = Path(f)

		x_fall = path.falling_point(ball.window)
		x_falls.append((x_fall/real_world.px_m_ratio, time.clock()))
		belt_motor.speed = (x_fall/real_world.px_m_ratio - rail_origin)*5

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break


	##########################
	#  TEMP: DRAW POSITIONS  #
	##########################
	print("Number of positions:", len(positions))
	x_falls = array(x_falls)
	X = x_falls[:,1]
	Y = x_falls[:,0]

	pl.plot(X, x_falls)
	pl.legend(['Abscysse du point de chute anticipé en fonction du temps'])
	pl.show()


	# while 1:
	# 	pos = ball.position

	# 	print(pos)

	# 	# WARNING ! DO NOT DELETE
	# 	if cv2.waitKey(1) & 0xFF == ord('q'):
	# 		break

	ball.stop_positionning()


parser = argparse.ArgumentParser()
parser.add_argument(
	'--config-distances', 
	help='Enter the distances configuration menu',
	action='store_true')
parser.add_argument(
	'--source',
	help="""Specifies the video source to use with opencv, can be a path to a 
			video file or the index of the camera')"""
)
parser.add_argument(
	'--config-color',
	help='Enter the ball color configuration menu',
	action='store_true'
)

args = parser.parse_args()
source = 0
real_world = RealWorld()

if args.source:
	if args.source.isnumeric():
		source = int(args.source)
	else:
		source = args.source


####################
# DISTANCES CONFIG #
####################

if args.config_distances:
	print('DISTANCES CONFIG')
	capture = cv2.VideoCapture(source)

	print('Enter the dimensions of the reference paper : ')
	w = float(input('width (m) : '))
	h = float(input('height (m) : '))

	while 1:
		ret, frame = capture.read()

		cv2.putText(
			frame, 
			'Place the reference paper landscape mode then press the d key',
			(0, 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 3
		)
		cv2.imshow('CONFIG', frame)

		if cv2.waitKey(1) & 0xFF == ord('d'):
			break

	ret, frame = capture.read()

	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	print(real_world.config_distances(gray, w, h), 'px/m')
	real_world.save()
	exit()


##################
#  COLOR CONFIG  #
##################
if args.config_color:
	print('COLOR CONFIG')
	capture = cv2.VideoCapture(source)

	while 1:
		ret, frame = capture.read()

		cv2.putText(
			frame, 
			'Place the object behind in the center then press the d key',
			(0, 20),
			cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 3
		)

		height = len(frame)
		width = len(frame[0])
		cv2.circle(frame, (width//2, height//2), 3, (0, 255, 0))


		cv2.imshow('CONFIG', frame)

		if cv2.waitKey(1) & 0xFF == ord('d'):
			break

	ret, frame = capture.read()

	rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
	
	print(real_world.config_colors(rgb_frame))
	real_world.save()

else:
	main(source, real_world)