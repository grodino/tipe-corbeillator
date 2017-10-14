import argparse

import cv2
from numpy import array
from matplotlib import pyplot as pl

from config.environment import RealWorld
from tracking.ball import Ball
from tracking.path import Path
from physics.models import free_fall


def main(source, real_world):	
	WHITE_LOWER = (240, 240, 240)
	WHITE_UPPER = (255, 255, 255)

	ORANGE_LOWER = (255, 122, 0)
	ORANGE_UPPER = (255, 200, 0)

	ball = Ball(source, (WHITE_LOWER, WHITE_UPPER))
	ball.start_positionning()

	i = 0
	positions = []

	# Wait for the ball to appear
	while not ball.is_in_range:
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	print("BALL FOUND")

	while ball.is_in_range:
		try:
			positions.append(ball.position)
		except:
			break

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	print("Number of positions:", len(positions))

	positions = array(positions)
	Y = positions[:, 1]
	X = positions[:, 0]

	pos_y = free_fall(positions[:2], real_world.ratio)
	path = Path(pos_y)
	Y2 = [path.pos_y(x) for x in X]

	pl.plot(X, Y)
	pl.plot(X, Y2)
	pl.legend(['Courbe expérimentale', 'courbe théorique'])
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
	'--config', 
	help='Enter the config menu',
	action='store_true')
parser.add_argument(
	'--source',
	help="""Specifies the video source to use with opencv, can be a path to a 
			video file or the index of the camera')"""
)

args = parser.parse_args()
source = 0
real_world = RealWorld()

if args.source:
	if args.source.isnumeric():
		source = int(args.source)
	else:
		source = args.source

if args.config:
	print('CONFIG')
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
else:
	main(source, real_world)