import json

import cv2


class RealWorld:
	ratio = None

	def __init__(self):
		with open('C:/Users/agodi/Desktop/tipe-corbeillator/config/config.json') as file:
			data = json.load(file)

		self.ratio = data['px_m_ratio']

	def save(self):
		"""
		Saves all config vars to the config.json file
		"""

		with open('C:/Users/agodi/Desktop/tipe-corbeillator/config/config.json') as file:
			data = json.load(file)

		with open('C:/Users/agodi/Desktop/tipe-corbeillator/config/config.json', 'w') as file:
			data['px_m_ratio'] = self.ratio

			json.dump(data, file)


	def config_distances(self, image, real_w, real_h):
		"""
		Protocol:
			- The operator places a rectangle in landscape mode of a known height and width
			- The function detects the rectangle and calculates it's dimensions
			- Returns the ratio px/m avering it in height and width
		
		Params:
			- image : a matrix of pixels in grayscale

		Saves the value of the ratio into config.json
		"""

		# Blur the image in order not to have to much details
		blurred = cv2.GaussianBlur(image, (5, 5), 0)
		thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]

		# find all the contours in the image
		cnts = cv2.findContours(
			thresh.copy(),
			cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)[-2]

		# keep only the rectangles
		def is_rect(c):
			""" Approximation : 4 vertices = rectangle """
			perimeter = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.04 * perimeter, True)

			return len(approx) == 4

		rects = list(filter(is_rect, cnts))

		if len(rects) <= 0:
			raise ValueError('Could not find the reference rectangle')

		biggest_rect = rects[0]
		peri = cv2.arcLength(biggest_rect, True)
		approx = cv2.approxPolyDP(biggest_rect, 0.04 * peri, True)
		x, y, w, h = cv2.boundingRect(approx)
		biggest_area = w * h

		# Detect the biggest rectangle (by the area) (assuming it is our reference)
		for c in rects:
			peri = cv2.arcLength(c, True)
			approx = cv2.approxPolyDP(c, 0.04 * peri, True)

			x, y, w, h = cv2.boundingRect(approx)
			area = w * h

			if area > biggest_area:
				biggest_rect = c
				biggest_area = area

		vertical_ratio = w / real_w
		horizontal_ratio = h / real_h

		# cv2.rectangle(thresh, (x, y), (x + w, y + h), (0, 255, 0), 3)
		# cv2.imshow('Shape Detection', thresh)

		# while 1:
		# 	# WARNING ! DO NOT DELETE
		# 	if cv2.waitKey(1) & 0xFF == ord('q'):
		# 		break

		self.ratio = (vertical_ratio + horizontal_ratio)/2

		return self.ratio