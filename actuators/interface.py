class SerialActuator:
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

		# HANDSHAKE
		while self._console.in_waiting <= 0:
			self._console.write(b'0')

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
		message_returned = str(self._console.readline().decode('ascii')).replace('\r\n', '')

		# Read all the lines returned by the serial connexion
		while message_returned != '':
			answer.append(message_returned)
			message_returned = str(self._console.readline().decode('ascii')).replace('\r\n', '')

		# Tweak because sometimes, this string appeared in the results
		answer = list(filter(lambda x: x != '17497', answer))

		return answer