from time import clock

import cv2
from numpy import array

from config.environment import RealWorld

def detect(capture, color_range):
    if not capture or not capture.isOpened():
        raise ValueError('There is no opened video capture')

    ret, frame = capture.read()

    t_grabed = clock()
    pos = (-1, -1, 0, t_grabed)

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Filter only the pixels in the right range of colors
    mask = cv2.inRange(rgb, array(color_range[0]), array(color_range[1]))
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
            return (-1, -1, 0, t_grabed)

        if radius > 1:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                        (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            # Add the center to the drawing
            try:
                cv2.circle(frame, center, 2, (0, 255, 0), -1)
                cv2.circle(frame, (0, 0), 2, (0, 255, 0), -1)
            except:
                print('OUCH')
                pass

        pos = (center[0], center[1], radius, clock())

    cv2.imshow('frame', frame)

    return pos


def main():
    real_world = RealWorld()
    capture = cv2.VideoCapture(1)

    color_range = real_world.color_range

    x, y, r, t = detect(capture, color_range)
    k = cv2.waitKey(60)

    while x == -1 and y == -1 and not(cv2.waitKey(1) & 0xFF == ord('d')) or r < 10:
        x, y, r, t = detect(capture, color_range)
    
    r = 50
    track_window = (x, y, int(r) + 10, int(r) + 10)
    ret,frame = capture.read()

    roi = frame[x:x+int(r),y:y+int(r)]
    rgb_roi =  cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mask = cv2.inRange(rgb_roi, color_range[0], color_range[1])
    roi_hist = cv2.calcHist([rgb_roi],[0],mask,[180],[0,180])
    cv2.normalize(roi_hist,roi_hist,0,255,cv2.NORM_MINMAX)

    # Setup the termination criteria, either 10 iteration or move by atleast 1 pt
    term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )

    while(1):
        ret ,frame = capture.read()

        if ret == True:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180],1)

            # apply meanshift to get the new location
            ret, track_window = cv2.meanShift(dst, track_window, term_crit)

            # Draw it on image
            x,y,w,h = track_window
            img2 = cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
            cv2.imshow('frame',img2)
            cv2.imshow('dst', dst)

            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break

        else:
            break

    cv2.destroyAllWindows()
    capture.release()

if __name__ == '__main__':
    main()



