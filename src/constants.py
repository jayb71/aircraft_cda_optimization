import numpy as np

# Constants (A320-like aircraft)
g = 9.81  # m/s^2
rho_0 = 1.225  # kg/m^3
S = 122.6  # m^2
m0 = 68000  # kg
C_D0 = 0.031
k = 0.045
T_max = 2 * 75000  # N
C_ff1, C_ff2, C_ff3, C_ff_ch = 0.01, 0.01, 0.01, 1e-8  # Adjusted for A320 CDA
V_min, V_max = 150, 250  # m/s
gamma_max = np.radians(3)
x_req = 3 * 1852  # 3 NM
w_0 = 5  # m/s
t_f = 3600  # s
N = 20  # Number of intervals
