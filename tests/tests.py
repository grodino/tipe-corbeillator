import time

import serial
import numpy as np
from matplotlib import pyplot as pl

from tracking.path import Path
from tracking.ball import Ball, display_info

from tests.utils import amplitude
from tests.utils import constraint
from tests.utils import sample_time
from tests.utils import ask_context
from tests.utils import response_time

from actuators.motor import Motor
from physics.models import free_fall
from config.environment import RealWorld
from tests.experimenting import Experiment


################################################################################
#                             OPEN LOOP SINE INPUT                             #
################################################################################
def open_loop_sine_input(source, port, real_world, debug):
    arduino_console = serial.Serial(port, 230400, timeout=1, write_timeout=2)
    experiment = Experiment.new(real_world.data_folder)

    motor = Motor(
        arduino_console,
        debug=debug
    )
    # avoids some bugs with serial
    time.sleep(1)
    context = ask_context()
    experiment.add_data(['context'], context)
    print('Move the basket next to the motor (the origin)')
    input('Ready ?')

    ############
    # MEASURES #
    ############
    START_FREQ = 0.05 # Hz
    END_FREQ = 50 # Hz
    NB_POINTS = 15

    SINE_AMPLITUDE = 200 # pwm
    EXP_DURATION = 15 # s
    CENTER = int(
        (real_world.rail_length/2)*real_world.inc_m_ratio
    )# inc

    motor.position = CENTER
    time.sleep(2)

    exp_freqs = np.geomspace(
        START_FREQ,
        END_FREQ,
        NB_POINTS
    )
    results = []

    for sine_freq in exp_freqs:
        motor.position = CENTER
        time.sleep(5)
        sine_freq += 0.1

        speed_measures = []
        pos_measures = []
        consignes = []
        times = []

        t_start = time.clock()
        t = t_start
        
        while t - t_start < EXP_DURATION:
            times.append(t - t_start)
            speed_measures.append(motor.speed)
            pos_measures.append(motor.position)

            order = int(SINE_AMPLITUDE*np.sin(2*np.pi*sine_freq*(t - t_start)))

            # limit the amplitude of the movement
            if pos_measures[-1] <= 1500 and order <= 0:
                motor.speed = 0
                consignes.append(0)
            elif pos_measures[-1] >= 2*CENTER - 1500 and order >= 0:
                motor.speed = 0
                consignes.append(0)
            else:
                motor.speed = order
                consignes.append(order)

            t = time.clock()
        
        motor.speed = 0
        # pos_measures = np.array(pos_measures)
        # pos_spectrum = np.fft.rfft(pos_measures)

        # speed_measures = np.array(speed_measures)
        # speed_spectrum = np.fft.rfft(speed_measures)

        results.append({
            "order_freq": sine_freq,
            "order_amplitude": SINE_AMPLITUDE,
            "times": times,
            "speed_orders": consignes,
            "speed_measures": speed_measures,
            "pos_measures": pos_measures
        })

    ############
    # ANALYSIS #
    ############
    experiment.add_data(['entree_sinus'], {'measures': results})
    experiment.save()
    n_experiments = len(results)

    # plot all the experiments measures
    fig, axes = pl.subplots(n_experiments)

    for i in range(n_experiments):
        axes[i].plot(results[i]['times'], results[i]['pos_measures'], color='r')
        axes[i].legend(['position mesurée (inc) '])
        
        ax2 = axes[i].twinx()
        ax2.plot(results[i]['times'], results[i]['speed_measures'], color='b')
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


################################################################################
#                             OPEN LOOP STEP INPUT                             #
################################################################################
def open_loop_step_input(source, port, real_world, debug):
    arduino_console = serial.Serial(port, 230400, timeout=1, write_timeout=2)
    experiment = Experiment.new(real_world.data_folder)

    motor = Motor(
        arduino_console,
        debug=debug
    )
    # avoids some bugs with serial
    time.sleep(1)
    context = ask_context()
    experiment.add_data(['context'], context)

    EXP_DURATION = 1.7 # s
    SPEED_VALUE = 255 # pwm

    positions = []
    speeds = []
    times = []

    t_start = time.clock()
    t = t_start

    while t - t_start < EXP_DURATION:
        motor.speed = SPEED_VALUE
        times.append(t - t_start)
        positions.append(motor.position)
        speeds.append(motor.speed)
        t = time.clock()
    
    motor.speed = 0
    
    experiment.add_data(
        ['entree_echelon'],
        {
            'name': 'Réponse temporelle du système en boucle ouverte à une entrée en echellon',
            'speed_order': SPEED_VALUE,
            'times': times,
            'speeds': speeds,
            'positions': positions
        }
    )
    experiment.save()
        

################################################################################
#                       CLOSED LOOP POSITION STEP INPUT                        #
################################################################################
def closed_loop_pos_step_input(source, port, real_world, debug):
    arduino_console = serial.Serial(port, 230400, timeout=1, write_timeout=2)
    experiment = Experiment.new(real_world.data_folder)

    motor = Motor(
        arduino_console,
        debug=debug
    )
    # avoids some bugs with serial
    time.sleep(1)
    context = ask_context()
    experiment.add_data(['context'], context)

    EXP_DURATION = 3 # s
    POS_VALUE = int((real_world.rail_length/2)*real_world.inc_m_ratio) # inc

    positions = []
    speeds = []
    times = []

    t_start = time.clock()
    t = t_start

    while t - t_start < EXP_DURATION:
        motor.position = POS_VALUE
        times.append(t - t_start)
        positions.append(motor.position)
        speeds.append(motor.speed)
        t = time.clock()
    
    motor.speed = 0
    
    experiment.add_data(
        ['entree_echellon'],
        {
            'name': 'Réponse temporelle du système en boucle fermée à une entrée en echelon de position',
            'pos_order': POS_VALUE,
            'times': times,
            'speeds': speeds,
            'positions': positions
        }
    )
    experiment.save()

    pl.plot(times, positions)
    pl.legend(['position'])
    pl.show()


################################################################################
#                            TRACKING TIME TEST                                #
################################################################################
def tracking_time_test(source, port, real_world, debug):
    import cv2
    
    color_range = real_world.color_range

    ball = Ball(source, color_range, max_retries=500, debug=debug)

    ball.start_positionning()
    i = 0
    positions = []
    models = []
    x_falls = []

    t = time.clock()

    # Wait for the ball to appear
    while not ball.is_in_range:
        dt = time.clock() - t
        t = time.clock()

        frame = ball._last_frame
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        fps = str(1/dt)
        
        display_info(frame, fps, color_range)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            return

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
    

################################################################################
#                          TRAJECTORY ANTICIPATION TEST                        #
################################################################################
def trajectory_anticipation_test(source, port, real_world, debug):
    import cv2

    color_range = real_world.color_range

    ball = Ball(source, color_range, max_retries=100, debug=debug)

    ball.start_positionning()
    i = 0
    positions = []
    models = []
    x_falls = []

    t = time.clock()
    bgr = cv2.cvtColor(ball._last_frame, cv2.COLOR_RGB2BGR)

    height = len(bgr)
    width = len(bgr[0])

    x_rail_origin = int(real_world.dist_origin_rails)
    line = np.array(
           [[x_rail_origin, 0], [x_rail_origin, height - 1]], np.int32
    ).reshape((-1,1,2))

    # Wait for the ball to appear
    while not ball.is_in_range:
        dt = time.clock() - t
        t = time.clock()

        bgr = cv2.cvtColor(ball._last_frame, cv2.COLOR_RGB2BGR)
        
        display_info(bgr, str(1/dt), color_range)
        cv2.polylines(bgr,[line],False,(0, 0, 255))

        cv2.imshow('camera', bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return

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

        x_fall = path.falling_point(ball.window, ball.window['width']//2)
        x_falls.append((x_fall, time.clock()))

        frame = ball._last_frame
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        fps = str(1/dt)
        
        display_info(frame, fps, color_range)

        cv2.imshow('camera', frame)
        cv2.imshow('mask', ball._mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    for i, x_fall in enumerate(x_falls):
        cv2.circle(
            bgr, 
            (int(x_fall[0]), height//2), 
            3, 
            (int((i/len(x_falls))*255), 0, 0)
        )

    for position in positions:
        cv2.circle(bgr, (position[0], height - position[1]), 3, (255, 0, 0))

    n = len(models)

    for i in range(0, n, 2):
        f = models[i]

        line = np.array(
            [[x,int(height - f(x))] for x in range(0, width, int(width/50))], np.int32
        ).reshape((-1,1,2))
        
        cv2.polylines(bgr,[line],False,(0, int(255*(i/n)), 0))

    x_rail_origin = int(real_world.dist_origin_rails)
    line = np.array(
           [[x_rail_origin, 0], [x_rail_origin, height - 1]], np.int32
        ).reshape((-1,1,2))
    cv2.polylines(bgr,[line],False,(0, 0, 255))
        
    while 1:
        cv2.imshow('camera', bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("Number of positions:", len(positions))
    print(positions)
        
    # positions = np.array(positions)
    # X = positions[:,0]
    # Y = positions[:,1]

    # for point in x_falls:
    # 	pl.plot(point[0], 0, 'ro')

    # for model in models:
    #     pl.plot(X, [model(x) for x in X])

    # pl.plot(X, Y)
    # pl.plot(X, [f(x) for x in X])
    # pl.legend(['position']+[str(i) for i in range(len(models))])
    # pl.show()


################################################################################
#                               BODE DATA ANALYSIS                             #
################################################################################
def bode_data_analysis(source, port, real_world, debug):
    """
    Takes the data stored in the experiments and draws a bode diagram
    """

    exp_id = input('EXPRERIMENT ID : ')
    experiment = Experiment.from_id(exp_id, real_world.data_folder)
    n_experiments = len(experiment.data['entree_sinus']['measures'])
    amplitudes = []
    freqs = []
    
    # fig, axes = pl.subplots(n_experiments)

    for i in range(n_experiments):
        pos_measures = np.array(
            list(map(float, experiment.data['entree_sinus']['measures'][i]['pos_measures']))
        )
        speed_measures = np.array(
            list(map(float, experiment.data['entree_sinus']['measures'][i]['speed_measures']))
        )
        times = np.array(
            list(map(float, experiment.data['entree_sinus']['measures'][i]['times']))
        )
        order_freq = float(experiment.data['entree_sinus']['measures'][i]['order_freq'])
        order_amplitude = float(experiment.data['entree_sinus']['measures'][i]['order_amplitude'])

        pos_spectrum = np.fft.rfft(pos_measures)
        speed_spectrum = np.fft.rfft(speed_measures)
        freq_range = np.fft.rfftfreq(times.size, sample_time(times))

        amplitudes.append(amplitude(
            times, 
            speed_measures, 
            1/order_freq
        ))
        freqs.append(order_freq)

    freqs = np.array(freqs)
    amplitudes = np.array(amplitudes)*real_world.pwm_incs_ratio # convert in pwm

    pl.title('Gain en vitesse de la corbeille en fonction de la fréquence')
    pl.semilogx(
        freqs,
        20*np.log10(amplitudes/order_amplitude)
    )
    pl.xlabel('Fréquence (Hz)')
    pl.ylabel('Gain (dB)')
    pl.grid(True, color='0.7', linestyle='-', which='both', axis='both')
    pl.show()


################################################################################
#                            STEP INPUT DATA ANALYSIS                          #
################################################################################
def step_input_data_analysis(source, port, real_world, debug):
    exp_id = int(input('EXPERIMENT ID : '))
    experiment = Experiment.from_id(exp_id, real_world.data_folder)

    EXP_TYPES = ['POSITION', 'SPEED']
    EXP_TYPES_TO_FR = {'POSITION': 'position', 'SPEED': 'vitesse'}

    print('Experiment types : ')
    for i, exp_type in enumerate(EXP_TYPES):
        print('\t [{}] {}'.format(i, exp_type))
    
    exp_type = EXP_TYPES[int(input('Choose the type : '))]
    
    if exp_type == 'SPEED':
        order = experiment.data['entree_echelon']['speed_order']
    else:
        order = experiment.data['entree_echelon']['pos_order']
    times = experiment.data['entree_echelon']['times']
    speeds = experiment.data['entree_echelon']['speeds']
    positions = experiment.data['entree_echelon']['positions']

    if exp_type == 'SPEED':
        response_t, lower, upper = response_time(times, speeds)
    else:
        response_t, lower, upper = response_time(times, positions)

    fig, ax1 = pl.subplots()
    pl.title('Réponse à un échellon de {}'.format(
        EXP_TYPES_TO_FR[exp_type]
    ))
    
    ax1.plot(times, positions, color='red')
    ax1.set_ylabel('position (inc)')

    ax2 = ax1.twinx()
    ax2.plot(times, speeds, color='blue')
    ax2.set_ylabel('vitesse (inc/s)')

    ax2.axhline(y=lower, color='0.7', linestyle='--', linewidth=1)
    ax2.axhline(y=upper, color='0.7', linestyle='--', linewidth=1)
    ax2.text(0, upper, '$' + str(upper)[:4] + ' +5\%$')
    ax2.text(0, lower, '$' + str(lower)[:4] + ' -5\%$')
    
    ax2.axvline(x=response_t, color='0.7', linestyle='--', linewidth=1)
    ax2.text(response_t, 0, '$t_{5\%}= '+ str(response_t)[:3]+' s$')

    pl.show()

TESTS_LIST = [
    open_loop_sine_input,
    open_loop_step_input,
    closed_loop_pos_step_input,
    tracking_time_test,
    trajectory_anticipation_test,
    bode_data_analysis,
    step_input_data_analysis
]

def list_tests():
    return TESTS_LIST