class Path:
	"""
	From a array of initial positions and a model, predicts the path of an objet
	"""

	pos_y = None

	def __init__(self, pos_y):
		"""
		params:
			- pos_y : a function that give the y coordinate in function of the x coordinate
		"""

		self.pos_y = pos_y

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