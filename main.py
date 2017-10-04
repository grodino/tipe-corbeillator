import cv2
from numpy import array
from matplotlib import pyplot as pl

from tracking.ball import Ball
from tracking.path import Path
from physics.models import free_fall

WHITE_LOWER = (240, 240, 240)
WHITE_UPPER = (255, 255, 255)

ball = Ball(2, (WHITE_LOWER, WHITE_UPPER), (1.34, 1))
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

model = free_fall
path = Path(model, positions[:2])
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

