import time


class SerialActuator(object):
	"""
	Abstracts the serial interface between python and the arduino
	"""

	_console = None
	_debug = False

	def __init__(self, console, debug=False):
		"""
		params:
			- console: a Serial object
		"""

		self._console = console
		self._debug = debug

		if self._debug:
			print('[SERIAL] CONNECTING')

		# HANDSHAKE
		while self.get_value() != 'READY':
			time.sleep(0.5)

		while self.get_value() == 'READY':
			self._console.write(b'1')
			time.sleep(0.5)
		
		time.sleep(3) # Avoid bugs

		if self._debug:
			print('[SERIAL] CONNEXION READY')

	def exec(self, order):
		"""
		Writes the command to the serial console and returns

		params:
			- order : a bytes string containing the order to send
		"""

		if type(order) != bytes:
			raise ValueError('You must provide a bytes string for the order')

		self._console.write(order)

		if self._debug:
			print('[SERIAL] WROTE COMMAND ' + str(order))

	def exec_and_get(self, order):
		"""
		Writes the command to the serial console and waits for potentially multiple 
		answer as a lines. Returns a list of str

		params:
			- order : a bytes string containing the order to send
		"""

		self.exec(order)

		# wait for the answer
		while self._console.in_waiting <= 0:
			pass

		answer = []
		message_returned = self.get_value()

		# Read all the lines returned by the serial connexion
		while message_returned != '':
			answer.append(message_returned)
			message_returned = str(self._console.readline().decode('ascii')).replace('\r\n', '')

		# Tweak because sometimes, this string appeared in the results
		answer = list(filter(lambda x: x != '17497', answer))

		return answer
	
	def get_value(self):
		"""
		Returns the first value in the buffer
		"""

		self._console.reset_input_buffer()
		read_value = self._console.readline()

		if self._debug:
			print('[SERIAL] READ VALUE : ', read_value)
			print('[SERIAL] IN WAITING : ', self._console.in_waiting)

		return str(read_value.decode('ascii')).replace('\r\n', '')