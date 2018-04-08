import time
import threading

import serial

from actuators.interface import SerialActuator

# TODO : plutôt que de d'envoyer et attendre une réponse, faire en sorte que
# l'arduino envoie en continue sont état, le stocker et comme ça juste lire la
# variable stockée quand on veut la valeur
# => C'est ici qu'il faut le thread différent !

class Motor(SerialActuator):
	pwm = 0
	rotation = 0 # 0: ANTICLOCKWISE, 1: CLOCKWISE
	max_speed = 200 # rad/s
	max_acceleration = 200 # rad/s^2
	pos_digit_number = 6 # number of digits in the pos order

	_read_thread = None
	_prev_speed_measure = (0, 0)
	_state = {
		'pos' : 0,
		'speed': 0
	}
	
	def __init__(self, serial_console, max_speed=200, max_acceleration=200, pos_digit_number=6, debug=False):
		"""
		params:
			- serial_console: the serial.Serial object used to communicate
			- max_speed: the maximum rotation speed of the motor in rad/s
			- max_acceleration: the maximum rotation acceleration of the motor
				in rad/s^2
			- pos_digit_number : number of digits (not including the minus sign) in the position value
			- debug: if true, the object becomes verbose
		"""

		self._pwm = 0
		self._rotation = 0
		self.max_speed = max_speed
		self.max_acceleration = max_acceleration
		self._prev_speed_measure = (0, time.clock())
		self.pos_digit_number = pos_digit_number

		# Initiate a separate thread to read value from serial console
		#self._read_thread = threading.Thread(target=self._update_motor_state, daemon=True)

		super().__init__(serial_console, debug)

		#self._read_thread.start()

	def _update_motor_state(self):
		"""
		Reads serial input and store values in the state dict
		"""

		success = False

		value = self.get_value()

		if self._debug:
			print('[SERIAL MOTOR] VALUE ', value)
		
		if value.startswith('[') and value.endswith(']'):
			values = value.replace('[', '').replace(']', '').split()
			
			if len(values) == 2:
				pos, speed = values

				if pos.replace('.', '').replace('-', '').isdigit() \
					and speed.replace('.', '').replace('-', '').isdigit():

					self._state['pos'] = int(float(pos))
					self._state['speed'] = float(speed)

					success = True

					if self._debug:
						print('[SERIAL MOTOR] STATE UPDATE : ', self._state)
		
		if self._debug and not success:
			print('[SERIAL MOTOR] COULD NOT UPDATE STATE')


	@property
	def speed(self):
		"""
		Computes and returns the current speed of the motor in rad/s
		"""

		self._update_motor_state()

		return self._state['speed']

	@speed.setter
	def speed(self, value):
		"""
		Sets the speed asked if in the range the maxspeed value
		of the motor or set it to max

		params:
			- value : speed in pwm [-255, 255]
		"""

		if not type(value) in [int, float]:
			raise ValueError('The speed must be an int or a float, not a ' + str(type(value)))


		pwm = min(abs(value), 255)

		# Fill with zeros so that the number of characters is always 3
		pwm = '0' * (3 - len(str(pwm))) + str(pwm)
		rotation = (0, 1)[value < 0]

		order = bytes('9' + str(rotation) + pwm, 'ascii')

		if self._debug:
			print('[SERIAL MOTOR] SENT ORDER : ', order)
		
		result = self.exec(order)
	
	@property
	def position(self):
		self._update_motor_state()

		return self._state['pos']
	
	@position.setter
	def position(self, value):
		"""
		Send a position order to the serial port
		"""

		if type(value) != int:
			raise TypeError('Position must be an int')

		nb_digits = len(str(abs(value)))
		if nb_digits > self.pos_digit_number:
			raise ValueError('Position value too big: ' + str(nb_digits) + ' greater than ' + int(self.pos_digit_number))
		
		order = '8'
		order += ('+', '-')[value < 0]
		order += '0'*(self.pos_digit_number - nb_digits)
		order += str(abs(value))

		order = bytes(order, 'ascii')

		if self._debug:
			print('[SERIAL MOTOR] SENT ORDER : ', order)
		
		self.exec(order)
	


# arduino_console = serial.Serial('COM3', 9600, timeout=1, write_timeout=2)

# belt_motor = Motor(arduino_console)
# # avoids some bugs
# time.sleep(0.5)

# belt_motor.speed = 200
# time.sleep(1)
# belt_motor.speed = 0