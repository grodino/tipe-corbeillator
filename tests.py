import time

import serial

from actuators.motor import Motor
from config.environment import RealWorld


################################
# OPEN LOOP MOTOR DRIVING TEST #
################################
real_world = RealWorld()
arduino_console = serial.Serial('COM3', 9600, timeout=1, write_timeout=2)

motor = Motor(
	arduino_console, 
	max_speed=real_world.motor_max_speed
)
# avoids some bugs with serial
time.sleep(0.5)

#motor.exec(b'90255')
motor.speed = 10
time.sleep(10)
motor.speed = 0
#motor.exec(b'90000')