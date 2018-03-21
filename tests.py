import time

import serial
import numpy as np
from matplotlib import pyplot as pl

from actuators.motor import Motor
from config.environment import RealWorld


def constraint(val, a, b):
    if val < a:
        return a
    elif val > b:
        return b
    
    return val

################################
# OPEN LOOP MOTOR DRIVING TEST #
################################
if __name__ == '__main__':
    real_world = RealWorld()
    arduino_console = serial.Serial('COM3', 230400, timeout=1, write_timeout=2)

    motor = Motor(
        arduino_console, 
        max_speed=real_world.motor_max_speed,
        debug=True
    )
    # avoids some bugs with serial
    time.sleep(1)

    print(motor.position)
    #motor.position = 30_000
    sine_wave = 5_000*np.sin(np.linspace(0, 100, num=500))

    speed_measures = []
    pos_measures = []
    consignes = []
    times = []

    for value in sine_wave:
        motor.position = int(35_000 + value)
        consignes.append(int(35_000 + value))
        times.append(time.clock())
        speed_measures.append(motor.speed)
        pos_measures.append(motor.position)


    time.sleep(5)

    fig, ax1 = pl.subplots()
    ax2 = ax1.twinx()

    ax1.plot(times, pos_measures, color='r')
    ax1.plot(times, consignes)
    ax1.set_xlabel('time (s)')
    ax1.set_ylabel('position (inc)')
    ax1.legend(['position (inc)', 'consigne'], loc=1)

    ax2.plot(times, speed_measures, color='b')
    ax2.set_ylabel('speed (inc/s)')
    ax2.legend(['speed (inc/s)'], loc=4)


    pl.title('Réponse du moteur à un échellon de position')
    pl.show()