from math import pi, acos

import numpy as np


def zero_cross_down(values, start=0, end=-1):
    """
    Returns the indexes of the values so that values[i] > 0 and values[i+1] < 0

    params:
        - values: list of values to consider
        - start: index of the first value to consider
        - end: index of the last value to consider (can be < 0)
    """

    n = len(values)
    res = []
    
    for i in range(start, end % n - 1):
        if values[i]*values[i+1] < 0 and values[i] > 0:
            res.append(i)
    
    return res


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


def amplitude(times, signal, period):
    """
    Computes the mean amplitude of a periodic signal

    params:
        - times: instants when the measures were taken
        - signal: values of the measure
        - period: period of the signal
    """

    n = len(times)
    if not len(signal) == len(times):
        raise ValueError(
            'signal and times must have the same length (a measure for each time)'
        )

    points = []
    amp_sum = 0
    current_max = 0
    i = 0
    count = 0

    for i in range(n):
        if times[i] < (count + 1)*period:
            current_max = max(current_max, abs(signal[i]))
        else:
            amp_sum += current_max
            current_max = 0
            count += 1
    
    # print('MAX ', max(np.abs(signal)))
    # print('MEAN', mean_sum/count)
    
    return amp_sum/count


def phase1(times, signal, ref_signal, period):
    """
    Computes the phase of an harmonic signal relative to an harmonic reference 
    signal, modulo 2*pi

    params:
        - times: instants when the measures were taken
        - signal: values of the measure
        - ref_signal: values of the reference signal
        - period: period of the signals
    """
    import matplotlib.pyplot as pl

    am = amplitude(times, signal, period)
    am_ref = amplitude(times, ref_signal, period)
    
    produit = (signal*ref_signal)/(am*am_ref)
    
    pl.subplot('211')
    pl.plot(ref_signal, signal)
    
    pl.subplot('212')
    pl.plot(times, produit)
    pl.show()
    
    # Moyenne sur un nombre entier de périodes
    sampling_time = sum(
        [times[i+1] - times[i] for i in range(len(times) - 1)]
    )/(len(times) - 1)
    
    n = int((period/sampling_time)*(times[-1]/period))
    
    dsf = np.fft.rfft(produit[:n])
    
    cos_phi = 2*(abs(dsf[0])/n)
    print('COS_PHI : ', cos_phi)
    
    return 0 #-acos(cos_phi)
    

def phase(times, signal, ref_signal, period):
    """
    Computes the phase of an harmonic signal relative to an harmonic reference 
    signal, modulo 2*pi

    params:
        - times: instants when the measures were taken
        - signal: values of the measure
        - ref_signal: values of the reference signal
        - period: period of the signals
    """
    import matplotlib.pyplot as pl
    

    #pl.plot(times, ref_signal)
    pl.plot(times, signal)
    
    recompozed = np.fft.irfft(np.fft.rfft(signal)[:2])
    n = min(len(times), len(recompozed))
    pl.plot(times[:n], recompozed[:n])
            
    pl.show()
    
    return 0 #-acos(cos_phi)
    


def response_time(times, signal, percentage=0.05, nb_points_mean=10):
    """
    Returns response time, lower and upper boundaries, defined by the first 
    instant from which the signal stays within end_value -+ percentage*end_value
    
    params:
        - times: array of instants
        - signal: array of scalar values, must be the same dimension as times
        - percentage: float in [0;1]
        - nb_points_mean: int that sets the number of points used to calculate 
            the mean end value
    """

    n = len(times)
    end_value = sum(signal[n-nb_points_mean:])/(nb_points_mean)
    lower = end_value*(1 - percentage)
    upper = end_value*(1 + percentage)

    min_t = times[0]

    for i in range(n):
        if upper <= signal[i] or signal[i] <= lower:
            min_t = times[i]
    
    return min_t, lower, upper


def ask_context():
    print('\n\n')
    print('CONTEXT INFORMATION')

    voltage = float(input('Voltage given to chopper controller in V : '))
    print()

    basket_mounted = {
        'Y': True, 
        'N': False
    }[input('Basket mounted (Y or N) : ').upper()]
    print()

    remarks = input('Any other remarks : ')

    return {
        'voltage' : voltage,
        'basket_mounted': basket_mounted,
        'remarks': remarks
    }


if __name__ == '__main__':
    import matplotlib.pyplot as pl
    
    n_points = 1000
    a, b = 0, 5
    f = 2
    T_ech = (b - a)/n_points
    
    # pl.plot(T, COS_REF)
    # pl.plot(T, COS)
    # pl.legend(['ref', 'signal'])
    # pl.show()
    
    for phi in np.linspace(1, 2*pi, 1):
        T = np.linspace(a, b, n_points)
        COS_REF = np.cos(2*pi*f*T)
        COS = np.cos(2*pi*f*T + phi)
        
        print('TARGET PHASE : ', phi)
        mes = phase(T, COS, COS_REF, 1/f)
        print('PHASE : ', mes)
        print('ERROR : ', abs(mes - phi))
        print()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    