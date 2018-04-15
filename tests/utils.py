import numpy as np


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
    mean_sum = 0
    current_max = 0
    i = 0
    count = 0

    for i in range(n):
        if times[i] < (count + 1)*period:
            current_max = max(current_max, abs(signal[i]))
        else:
            mean_sum += current_max
            current_max = 0
            count += 1
    
    print('MAX ', max(np.abs(signal)))
    print('MEAN', mean_sum/count)
    
    return mean_sum/count


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