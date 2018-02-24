import time

import serial
from matplotlib import pyplot as pl

from actuators.motor import Motor
from config.environment import RealWorld


# def constraint(val, a, b):
# 	if val < a:
# 		return a
# 	elif val > b:
# 		return b
	
# 	return val

# ################################
# # OPEN LOOP MOTOR DRIVING TEST #
# ################################
# real_world = RealWorld()
# arduino_console = serial.Serial('COM6', 9600, timeout=1, write_timeout=2)

# motor = Motor(
# 	arduino_console, 
# 	max_speed=real_world.motor_max_speed,
# 	debug=False
# )
# # avoids some bugs with serial
# time.sleep(1)
# measures = []
# motor.speed = 5

# for i in range(0, 500):
# 	measures.append(motor.speed)


# time.sleep(5)
# motor.speed = 0

# X = list(range(len(measures)))
# pl.plot(X, measures)
# pl.legend(['nb tops/s'])
# pl.show()

real_world = RealWorld()
print(real_world.object_color)
print(real_world.dist_origin_rails)
real_world.dist_origin_rails = 6
print(real_world.dist_origin_rails)
real_world.save()