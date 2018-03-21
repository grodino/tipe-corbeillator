import json

import cv2

import numpy as np


class RealWorld(object):
    """
    Reads, computes and manages config vars
    """

    def __init__(self):
        with open('config/config.json') as file:
            self._config_data = json.load(file)
        
        self._keys = [var["name"] for var in self._config_data]
        
        docstring = 'Config vars: \n'
        for var in self._config_data:
            docstring += var['name']
            docstring += ' : '
            docstring += var['description']
            docstring += '\n'
            docstring += '    units: '
            docstring += var['unit']
            docstring += '\n\n'
        
        if self.__doc__:
            self.__doc__ += '\n' + docstring
        else:
            self.__doc__ = '\n' + docstring

    def __getattr__(self, name):
        """
        Checks if the name is in the config data, if yes returns its value
        else, use default behaviour
        """

        if name in self._keys:
            return self._config_data[self._keys.index(name)]['value']
        else:
            raise AttributeError('Attribute ' + name + ' was not found in class ' + str(self))

    def __setattr__(self, name, value):
        """
        Checks if the name is in the config data, if yes saves it in the config data
        else, use default behaviour
        """

        if not name.startswith('_'):
            if name in self._keys:
                self._config_data[self._keys.index(name)]['value'] = value
        else:
            super().__setattr__(name, value)

    def save(self):
        """
        Saves all config vars to the config.json file
        """

        with open('config/config.json', 'w') as file:
            json.dump(self._config_data, file, indent=4)

    def config_distances(self, image, real_w, real_h):
        """
        Protocol:
            - The operator places a rectangle in landscape mode of a known height and width
            - The function detects the rectangle and calculates it's dimensions
            - Returns the ratio px/m avering it in height and width
        
        Params:
            - image : a matrix of pixels in RGB

        Saves the value of the ratio into config.json
        """

        # Blur the image in order not to have to much details
        color = image[len(image)//2, len(image[0])//2]
        unit_vect = np.array([1, 1, 1])

        mask = cv2.inRange(image, color - 15*unit_vect, color + 15*unit_vect)
        mask = cv2.erode(mask, None, iterations=2)

        # find all the contours in the image
        cnts = cv2.findContours(
            mask.copy(),
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

        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 165, 0), 3)
        cv2.imshow('Shape Detection', image)

        while 1:
            # WARNING ! DO NOT DELETE
            if cv2.waitKey(1) & 0xFF == ord('d'):
                break

        self.px_m_ratio = (vertical_ratio + horizontal_ratio)/2

        return self.px_m_ratio

    def config_colors(self, image):
        """
        Get the rgb value of the pixel in the center of the image and 
        returns it

        params:
            - image : RGB color image

        """

        height = len(image)
        width = len(image[0])

        self.object_color = list(map(int, image[height//2, width//2]))

        return self.object_color
