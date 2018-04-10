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
