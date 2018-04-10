import os
import time
import argparse

import cv2
import serial
from numpy import array
from matplotlib import pyplot as pl

from tests.tests import list_tests

from config.environment import RealWorld
from tracking.ball import Ball
from tracking.path import Path
from physics.models import free_fall
from actuators.motor import Motor


def clear_cmd():
	"""
	Utility function that clear the shell
	"""

	try:
		os.system('clear')
	except:
		pass
	
	try:
		os.system('cls')
	except:
		pass

################################################################################
#                                     MAIN                                     #
################################################################################
def main(source, port, real_world, debug):
	############################
	#   MOTOR INITIALISATION   #
	############################

	arduino_console = serial.Serial(port, 230400, timeout=0.01)

	belt_motor = Motor(
		arduino_console, 
		max_speed=real_world.motor_max_speed,
		debug=debug
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

	ball = Ball(source, (lower, upper), max_retries=1000, debug=debug)
	ball.start_positionning()
	rail_origin = real_world.dist_origin_rails


	###################
	#  BALL TRACKING  #
	###################
	
	i = 0
	positions = []
	models = []
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
		models.append(f)
		path = Path(f)

		x_fall = path.falling_point(ball.window)
		x_falls.append((x_fall, time.clock()))
		
		t = time.clock()

		success = False
		while not success:
			try:
				belt_motor.position = int((x_fall/real_world.px_m_ratio)*real_world.encoder_ratio*1000)
				success = True
			except:
				success = False
				
		print('WROTE IN ', time.clock() - t)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break


	##########################
	#  TEMP: DRAW POSITIONS  #
	##########################
	print("Number of positions:", len(positions))
	print(positions)
	
	positions = array(positions)
	X = positions[:,0]
	Y = positions[:,1]

	# for point in x_falls:
	# 	pl.plot(point[0], 0, 'ro')

	for model in models:
		pl.plot(X, [model(x) for x in X])

	pl.plot(X, Y)
	#pl.plot(X, [f(x) for x in X])
	pl.legend(['position']+[str(i) for i in range(len(models))])
	pl.show()


	# while 1:
	# 	pos = ball.position

	# 	print(pos)

	# 	# WARNING ! DO NOT DELETE
	# 	if cv2.waitKey(1) & 0xFF == ord('q'):
	# 		break

	ball.stop_positionning()


################################################################################
#                                    TESTS                                     #
################################################################################
def run_tests(source, port, realworld, debug):
	clear_cmd()
	print('TESTS UTILITY')
	print()
	print()

	tests = list_tests()
	nb_tests = len(tests)

	print('Available tests:')
	for i, test in enumerate(tests):
		print('\t[{}] '.format(i), test.__name__)
	
	answer = input(
		'Execute test (you can enter one or more, coma separated) id : '
	)
	choices = list(map(int, answer.split(',')))
	
	for choice in choices:
		if choice in range(nb_tests):
			tests[choice]()
	


################################################################################
#                               DISTANCES CONFIG                               #
################################################################################
def config_distances(source, real_world, debug):
	clear_cmd()
	print('DISTANCES CONFIG')
	capture = cv2.VideoCapture(source)

	print('Enter the dimensions of the reference paper : ')
	w = float(input('width (m) : '))
	h = float(input('height (m) : '))

	while 1:
		ret, frame = capture.read()

		cv2.putText(
			frame, 
			'Place the reference paper behind the center then press the d key',
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

	img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
	print(real_world.config_distances(img, w, h), 'px/m')
	real_world.save()


################################################################################
#                                 CONFIG COLOR                                 #
################################################################################
def config_color(source, realworld, debug):
	clear_cmd()
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
		center = (height//2, width//2)
		square_size = 15

		y,x = (center[0] - square_size//2, center[1] - square_size//2)

		cv2.circle(frame, (width//2, height//2), 3, (0, 255, 0))
		cv2.rectangle(frame, (x, y), (x + square_size, y + square_size), 255, 2)

		cv2.imshow('CONFIG', frame)

		if cv2.waitKey(1) & 0xFF == ord('d'):
			break

	ret, frame = capture.read()

	rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
	
	print(real_world.config_colors(rgb_frame, square_size))
	real_world.save()


################################################################################
#                              CLI INTERFACE                                   #
################################################################################
if __name__ == '__main__':
	parser = argparse.ArgumentParser()

	parser.add_argument(
		'mode',
		help="""Select what you want to do : test or play
				Authorized values:
					- run (R)
					- test (T)""",
		type=str
	)

	parser.add_argument(
		'--source',
		help="""Specifies the video source to use with opencv, can be a path to a 
				video file or the index of the camera"""
	)
	parser.add_argument(
		'--port',
		help='Specifies the port to use to connect to the arduino'
	)
	parser.add_argument(
		'--config-distances', 
		help='Enter the distances configuration menu',
		action='store_true')
	parser.add_argument(
		'--config-color',
		help='Enter the ball color configuration menu',
		action='store_true'
	)
	parser.add_argument(
		'--debug',
		help='Switch debug mode on /!\\ Verbose /!\\',
		action='store_true'
	)

	args = parser.parse_args()
	source = 0
	port = 'COM0'
	debug = False
	real_world = RealWorld()

	if args.debug:
		debug = True

	if args.source:
		if args.source.isnumeric():
			source = int(args.source)
		else:
			source = str(args.source)

	if args.port:
		if 'COM' in args.port and args.port.replace('COM', '').isnumeric():
			port = args.port

	if args.config_distances:
		config_distances(source, real_world, debug)

	if args.config_color:
		config_color(source, real_world, debug)
	
	if args.mode in ['run', 'R']:
		main(source, port, real_world, debug)
	elif args.mode in ['test', 'T']:
		run_tests(source, port, real_world, debug)
	else:
		raise ValueError(
			'The chosen mode must be an authorized one, type --help for help'
		)