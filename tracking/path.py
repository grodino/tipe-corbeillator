class Path:
	"""
	From a array of initial positions and a model, predicts the path of an objet
	"""

	model = None
	pos_y = None
	initial_positions = []

	def __init__(self, model, initial_positions):
		"""
		params:
			- model : a function that accepts an array of initial positions
			and returns a function of x coordinate that returns the y coordinate
		"""

		self.model = model
		self.initial_positions = initial_positions

		self.pos_y = self.model(initial_positions)

	def falling_point(self, window, h_limit, precision):
		"""
		Returns the x value of the falling point (when y == h_limit) or None
		if it does no fall within the view of the camera
		
		Uses  a bisect method assuming that there only one zero (or none) in 
		the	function (studied models are free falling objects so it is 
		justified)
		"""

		left, right = 0, window['width']
		middle = (left + right) / 2

		while abs(right - left) >= precision:
			if pos_y(right) > 0:
				return None

			if pos_y(left)*pos_y(middle) < h_limit:
				right = middle
			else:
				left = middle

			middle = (left + right) / 2

		return middle