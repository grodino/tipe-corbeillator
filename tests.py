import time

import cv2
import serial
import numpy as np
from matplotlib import pyplot as pl

from tracking.path import Path
from actuators.motor import Motor
from physics.models import free_fall
from config.environment import RealWorld
from tracking.ball import Ball, display_info


def constraint(val, a, b):
    if val < a:
        return a
    elif val > b:
        return b
    
    return val


def sample_time(times):
    """
    Returns the mean sample time of the array of time instants given
    
    params:
        - times : array of instants
    """

    n = len(times)
    diffs = [times[i+1] - times[i] for i in range(n - 1)]

    return sum(diffs)/len(diffs)


################################
# OPEN LOOP MOTOR DRIVING TEST #
################################
def open_loop_motor_test():
    real_world = RealWorld()
    arduino_console = serial.Serial('COM6', 230400, timeout=1, write_timeout=2)

    motor = Motor(
        arduino_console, 
        max_speed=real_world.motor_max_speed,
        debug=False
    )
    # avoids some bugs with serial
    time.sleep(1)

    print(motor.position)

    ############
    # MEASURES #
    ############
    sine_freq = 0.1 # Hz
    sine_amplitude = 255 # pwm
    exp_duration = 15 # s

    results = []

    for i in range(8):
        sine_freq += 0.1

        speed_measures = []
        pos_measures = []
        consignes = []
        times = []

        t_start = time.clock()
        t = t_start
        
        while t - t_start < exp_duration:
            times.append(t - t_start)
            speed_measures.append(motor.speed)
            pos_measures.append(motor.position)
            
            order = int(sine_amplitude*np.sin(2*np.pi*sine_freq*(t - t_start)))
            motor.speed = order
            consignes.append(order)
            t = time.clock()
        
        # pos_measures = np.array(pos_measures)
        # pos_spectrum = np.fft.rfft(pos_measures)

        # speed_measures = np.array(speed_measures)
        # speed_spectrum = np.fft.rfft(speed_measures)

        results.append({
            "order_freq": sine_freq,
            "times": times,
            "speed_measures": speed_measures,
            "pos_measures": pos_measures
        })

    ############
    # ANALYSIS #
    ############

    n_experiments = len(results)

    # plot all the experiments measures
    fig, axes = pl.subplots(n_experiments)

    for i in range(n_experiments):
        axes[i].plot(results[i]['times'], results[i]['pos_measures'])
        axes[i].legend(['position mesurée (inc) '])
        
        ax2 = axes[i].twinx()
        ax2.plot(results[i]['times'], results[i]['speed_measures'])
        ax2.legend(['vitesse mesurée (inc/ms)'])
    
    # ax1.semilogx(
    #     results[:,0],
    #     20*np.log10(results[:,1])
    # )

    # ax1.semilogx(
    #     results[:,0],
    #     results[:,2]
    # )


    # fig, ax1 = pl.subplots()
    # ax2 = ax1.twinx()

    # ax1.plot(times, pos_measures, color='r')
    # ax1.plot(times, consignes)
    # ax1.set_xlabel('time (s)')
    # ax1.set_ylabel('position (inc)')
    # ax1.legend(['position (inc)', 'consigne'], loc=1)

    # ax2.plot(times, speed_measures, color='b')
    # ax2.set_ylabel('speed (inc/s)')
    # ax2.legend(['speed (inc/s)'], loc=4)


    #pl.title('Réponse du moteur à un sinus de vitesse')
    pl.show()


######################
# TRACKING TIME TEST #
######################
def tracking_time_test():
    real_world = RealWorld()
    color_range = real_world.color_range

    ball = Ball(0, color_range, max_retries=500, debug=True)

    ball.start_positionning()
    i = 0
    positions = []
    models = []
    x_falls = []

    t = time.clock()

    # Wait for the ball to appear
    while not ball.is_in_range:
        # frame = ball._last_frame
        # frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        # cv2.imshow('frame', frame)

        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
        dt = time.clock() - t
        t = time.clock()

        frame = ball._last_frame
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        fps = str(1/dt)
        
        display_info(frame, fps, color_range)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print('Object found')

    while ball.is_in_range:
        dt = time.clock() - t
        t = time.clock()
        frame = ball._last_frame
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        fps = str(1/dt)
        
        display_info(frame, fps, color_range)

        cv2.imshow('frame', frame)
        cv2.imshow('mask', ball._mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    

################################
# TRAJECTORY ANTICIPATION TEST #
################################
def trajectory_anticipation_test():
    real_world = RealWorld()
    color_range = real_world.color_range

    ball = Ball(0, color_range, max_retries=100, debug=True)

    ball.start_positionning()
    i = 0
    positions = []
    models = []
    x_falls = []

    t = time.clock()

    # Wait for the ball to appear
    while not ball.is_in_range:
        pass

    positions.append(ball.position)

    while ball.is_in_range:
        dt = time.clock() - t
        t = time.clock()

        positions.append(ball.position)

        f = free_fall(
            [positions[-1], positions[-2]],
            real_world.px_m_ratio
        )

        models.append(f)
        path = Path(f)

        x_fall = path.falling_point(ball.window)
        x_falls.append((x_fall, time.clock()))

        frame = ball._last_frame
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        fps = str(1/dt)
        
        display_info(frame, fps, color_range)

        cv2.imshow('frame', frame)
        cv2.imshow('mask', ball._mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("Number of positions:", len(positions))
    print(positions)
        
    positions = np.array(positions)
    X = positions[:,0]
    Y = positions[:,1]

    # for point in x_falls:
    # 	pl.plot(point[0], 0, 'ro')

    # for model in models:
    #     pl.plot(X, [model(x) for x in X])

    pl.plot(X, Y)
    #pl.plot(X, [f(x) for x in X])
    # pl.legend(['position']+[str(i) for i in range(len(models))])
    pl.show()

if __name__ == '__main__':
    #tracking_time_test()
    #trajectory_anticipation_test()
    open_loop_motor_test()
