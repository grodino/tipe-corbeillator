from time import clock

import cv2
from numpy import array

from tracking.ball import Ball
from tracking.ball import display_info
from config.environment import RealWorld

def detect(color_range):
    ball = Ball(1, color_range)
    ball.start_positionning()
    
    t = clock()

    while not ball.is_in_range:
        dt = clock() - t
        t = clock()

        frame = ball._last_frame
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        fps = str(1/dt)
        
        display_info(frame, fps, color_range)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            return
    
    pos = ball.positions[-1] + (t,)

    ball.stop_positionning()

    return pos


def main():
    real_world = RealWorld()

    color_range = real_world.color_range

    x, y, r, t = detect(color_range)
    print(x,y,r,t)

    # while x == -1 and y == -1 and not(cv2.waitKey(1) & 0xFF == ord('d')) or r < 10:
    #     print(x,y,r,t)
    #     x, y, r, t = detect(color_range)
    
    capture = cv2.VideoCapture(1)

    r = 100
    track_window = (x - int(r), y - int(r), int(r), int(r))
    x, y, w, h = track_window
    ret,frame = capture.read()

    roi = frame[y:y+h, x:x+w]
    rgb_roi =  cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mask = cv2.inRange(rgb_roi, color_range[0], color_range[1])

    while 1:
        cv2.imshow('roi', roi)
        cv2.imshow('mask', mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    roi_hist = cv2.calcHist([rgb_roi],[0],mask,[180],[0,180])
    print(roi_hist)
    cv2.normalize(roi_hist,roi_hist,0,255,cv2.NORM_MINMAX)

    # Setup the termination criteria, either 10 iteration or move by atleast 1 pt
    term_crit = ( cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1 )

    last_t = clock()

    while(1):
        ret ,frame = capture.read()
        t = clock()
        dt = t - last_t

        if ret == True:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            dst = cv2.calcBackProject([hsv],[0],roi_hist,[0,180],1)

            # apply meanshift to get the new location
            ret, track_window = cv2.meanShift(dst, track_window, term_crit)

            # Draw it on image
            x,y,w,h = track_window
            frame = cv2.rectangle(frame, (x,y), (x+w,y+h), 255,2)
            display_info(frame, str(1/dt), color_range)
            cv2.imshow('frame',frame)
            cv2.imshow('dst', dst)

            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break

        else:
            break
        
        last_t = clock()

    cv2.destroyAllWindows()
    capture.release()

if __name__ == '__main__':
    main()



