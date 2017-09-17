from time import sleep, clock

import numpy as np
import cv2

class Ball:
    """
    The postition of the ball, and methods to get it
    The theoricall falling point of the ball and methods to get it
    """

    postitions = []

    _video_source = None
    _video_capture = None
    _color_range = ((-1, -1, -1), (-1, -1, -1))

    def __init__(self, video_source, color_range):
        """
        Params:
            - video_source : opencv video source (camera of file)
            - color_range : tuple of two colors (who are 3-uples)
                ex : ((0,0,0), (0, 0, 50))
        """
        self._video_source = video_source
        self.color_range = color_range

    def start_positionning(self):
        """
        Start the video capture or the reading of the video
        """
        self._video_capture = cv2.VideoCapture(self._video_source)

    def _get_new_position(self):
        """
        Read a new image and determines (x,y) and return it
        """

        if not self._video_capture or not self._video_capture.isOpened():
            raise ValueError('There is no opened video capture')

        ret, frame = self._video_capture.read()


def track():
    cap = cv2.VideoCapture(0)
    #'VID_20170317_213254244.mp4'

    RED_LOWER = (0, 10, 0)
    RED_UPPER = (20, 50, 60)

    first_frame = cap.read()[1]
    height = len(first_frame)
    width = len(first_frame[0])

    drawing = first_frame
    point = None
    tic = clock()

    while(cap.isOpened()):
        ret, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Filter the image to retrieve only the green pixels
        mask = cv2.inRange(rgb, RED_LOWER, RED_UPPER)
        mask = cv2.erode(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(
            mask.copy(),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )[-2]
        center = None

        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)

            if M["m00"] != 0 and M["m00"] != 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                # Add the center to the drawing
                try:
                    cv2.circle(drawing, center, 2, (0, 255, 0), -1)
                    cv2.line(drawing, center, point, (0, 255, 0), 1)
                    point = center
                except:
                    pass

        tac = clock()
        delta_t = tac - tic
        tic = tac

        cv2.putText(frame, str(1/delta_t)[:5] + ' FPS', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 3)

        cv2.imshow('frame', frame)

        # WARNING ! DO NOT DELETE
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    while 1:
        cv2.imshow('final drawing', drawing)

        # WARNING ! DO NOT DELETE
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
