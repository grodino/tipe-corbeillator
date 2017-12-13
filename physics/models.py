from numpy import array


def free_fall(initial_positions, ratio):
		"""
		Free fall of an object with initial speed
		Repère ascendant

		/!\ The position is in pixels and speed in pixels/s
		"""

		g = array([0, -9.81*ratio])
		x0, y0, t0 = initial_positions[0]
		x1, y1, t1 = initial_positions[1]

		initial_speed = array([
			(x1 - x0) / (t1 - t0),
			(y1 - y0) / (t1 - t0)
		])
		initial_pos = array([x0, y0])

		def pos_y(x):
			t = (x - x0) / initial_speed[0]

			return ((g/2)*t**2 + (initial_speed)*t + initial_pos)[1]

		return pos_y