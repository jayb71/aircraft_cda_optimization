import numpy as np
from .constants import *

def speed_correction(t, x, STA, V_nominal, t1=1000):
    s_max = V_nominal * 0.2
    if t >= STA:
        return 0
    s_cor = x / (STA - t + 1e-6) if t <= t1 else x / (STA - t1 + 1e-6)
    return min(s_cor, s_max)

def simulate_uncertainty(t, V_nominal, STA_times, x_0=1.3 * 1852):
    x = np.zeros_like(t)
    s = np.zeros_like(t)
    x[0] = x_0
    for i in range(1, len(t)):
        dt = t[i] - t[i-1]
        STA = STA_times[min(len(STA_times)-1, np.searchsorted(STA_times, t[i]))]
        s[i] = speed_correction(t[i], x[i-1], STA, V_nominal[i-1])
        x[i] = x[i-1] + (w_0 - s[i]) * dt
    return x, s