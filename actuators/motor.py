import time

import serial

from actuators.interface import SerialActuator


class Motor(SerialActuator):
	pwm = 0
	rotation = 0 # 0: ANTICLOCKWISE, 1: CLOCKWISE
	max_speed = 200 # rad/s
	max_acceleration = 200 # rad/s^2

	_prev_speed_measure = (0, 0)
	
	def __init__(self, serial_console, max_speed=200, max_acceleration=200):
		"""
		params:
			- serial_console: the serial.Serial object used to communicate
			- max_speed: the maximum rotation speed of the motor in rad/s
			- max_acceleration: the maximum rotation acceleration of the motor
				in rad/s^2
			- ratio : the rotation to translation ratio in m/rad
		"""

		self._pwm = 0
		self._rotation = 0
		self.max_speed = max_speed
		self.max_acceleration = max_acceleration
		self._prev_speed_measure = (0, time.clock())

		super().__init__(serial_console)

	@property
	def speed(self):
		"""
		Computes and returns the current speed of the motor in rad/s

		TODO : use a sensor instead of a stored value
		"""

		val = (self._pwm/255)*self.max_speed

		self._prev_speed_measure = (
			val,
			time.clock()
		)

		return val

	@speed.setter
	def speed(self, value):
		"""
		Sets the speed asked if in the range the the maxspeed value
		of the motor considering the maximum acceleration with a 
		trapezoïd command.

		Raises a ValueError of the speed asked is too high

		params:
			- value : speed in rad/s
		"""

		if not type(value) in [int, float]:
			raise ValueError('The speed must be an int or a float, not a ' + str(type(value)))
		elif abs(value) > self.max_speed:
			raise ValueError('The speed must be under the maximum speed of the motor')

		t = time.clock()
		prev_speed, prev_t = self._prev_speed_measure
		accel_asked = (value - prev_speed)/(t - prev_t) # acceleration asked

		if abs(accel_asked) > self.max_acceleration:
			value = (accel_asked/abs(accel_asked)) * self.max_acceleration * (t - prev_t)


		pwm = int((value/self.max_speed)*255)
		# Fill with zeros so that the number of characters is always 3
		pwm = '0' * (3 - len(str(pwm))) + str(pwm)
		rotation = (1, -1)[value < 0]

		order = bytes('9' + str(rotation) + pwm, 'ascii')

		if self._debug:
			print('[SERIAL MOTOR] SENT ORDER : ', order)
		
		result = self.exec(order)

# arduino_console = serial.Serial('COM3', 9600, timeout=1, write_timeout=2)

# belt_motor = Motor(arduino_console)
# # avoids some bugs
# time.sleep(0.5)

# belt_motor.speed = 200
# time.sleep(1)
# belt_motor.speed = 0