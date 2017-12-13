from time import sleep, clock

import numpy as np
import cv2

class Ball:
    """
    The position of the ball, and methods to get it
    The theoricall falling point of the ball and methods to get it
    """

    positions = []
    MAX_GET_POS_RETRIES = 20

    _video_source = None
    _video_capture = None
    _color_range = ((-1, -1, -1), (-1, -1, -1))
    _last_frame = (None, None)
    _window = {'width': -1, 'height': -1}

    def __init__(self, video_source, color_range, max_retries=20):
        """
        Params:
            - video_source : opencv video source (camera or file)
            - color_range : tuple of two colors (who are 3-uples)
                ex : ((0,0,0), (0, 0, 50))
            - dimentions : 'real' height and width of the window in meters
        """
        self._video_source = video_source
        self.color_range = color_range
        self.MAX_GET_POS_RETRIES = max_retries

    def start_positionning(self):
        """
        Start the video capture or the reading of the video and reads the first
        frame
        """

        if self._last_frame != (None, None):
            raise ValueError("Positionning has already started !")

        self._video_capture = cv2.VideoCapture(self._video_source)
        ret, self._last_frame = self._video_capture.read()

        self._window['height'] = len(self._last_frame)
        self._window['width'] = len(self._last_frame[0])

        print(self._window)

    def stop_positionning(self):
        """
        Ends positionning and cuts the video feed
        """
        self._video_capture.release()
        cv2.destroyAllWindows()

    @property
    def is_in_range(self):
        """
        Returns if the camera (and the softare detected the ball)
        """

        current_pos = self._get_new_position()
        self.positions.append(current_pos)

        if current_pos[0] == -1 and current_pos[1] == -1:
            return False

        return True

    @property
    def position(self):
        """
        Returns the position of the ball, always returns a position
        If no position is found, raises an error
        Best practice is to call is_in_range before
        """

        pos = self._get_new_position()
        retries = 0

        while pos[0] == -1 and pos[1] == -1 and retries <= self.MAX_GET_POS_RETRIES:
            pos = self._get_new_position()
            retries += 1

        if retries > self.MAX_GET_POS_RETRIES:
            raise ValueError("Tracking failed")

        self.positions.append(pos)

        return pos


    def _get_new_position(self):
        """
        Read a new image and determines (x,y) and returns it
        If no position is found, returns (-1, -1)
        """

        if not self._video_capture or not self._video_capture.isOpened():
            raise ValueError('There is no opened video capture')

        ret, frame = self._video_capture.read()
        self._last_frame = frame

        t_grabed = clock()
        pos = (-1, -1, t_grabed)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Filter only the black pixels
        mask = cv2.inRange(rgb, self.color_range[0], self.color_range[1])
        mask = cv2.erode(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(
            mask.copy(),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )[-2]

        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = None

            if M["m00"] != 0 and M["m00"] != 0:
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            else:
                return (-1, -1, t_grabed)

            if radius > 1:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                # Add the center to the drawing
                try:
                    cv2.circle(frame, center, 2, (0, 255, 0), -1)
                except:
                    print('OUCH')
                    pass

            pos = (center[0], self._window['height'] - center[1], clock())

        cv2.imshow('frame', frame)

        return pos

    @property
    def window(self):
        return self._window


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
                    cv2.circle(frame, center, 2, (0, 255, 0), -1)
                    cv2.line(frame, center, point, (0, 255, 0), 1)
                    point = center
                except:
                    pass

        tac = clock()
        delta_t = tac - tic
        tic = tac

        cv2.putText(frame, str(1/delta_t)[:5] + ' FPS', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

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
