import cv2

from tracking.ball import Ball

BLACK_LOWER = (0, 10, 0)
BLACK_UPPER = (20, 50, 60)

ball = Ball(0, (BLACK_LOWER, BLACK_UPPER))
ball.start_positionning()

while 1:
	print(ball.position)

	# WARNING ! DO NOT DELETE
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

ball.stop_positionning()