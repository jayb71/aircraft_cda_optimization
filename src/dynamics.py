import casadi as ca
from .constants import *

def dynamics(x, u):
    V = x[0]
    d = x[1]
    h = x[2]
    m = x[3]
    T = u[0]
    gamma = u[1]
    
    T = ca.fmax(T, 0)
    V = ca.fmax(V, V_min)
    m = ca.fmax(m, 0.8 * m0)
    gamma = ca.fmin(ca.fmax(gamma, -gamma_max), gamma_max)
    
    rho = rho_0 * (1 - 2.255e-5 * h) ** 4.258
    denom = 0.5 * rho * V**2 * S + 1e-6
    C_L = m * g * ca.cos(gamma) / denom
    C_D = C_D0 + k * C_L**2
    D = C_D * 0.5 * rho * V**2 * S
    T_0 = T_max * (rho / rho_0)
    F_fuel = C_ff3 * (T / T_0)**3 + C_ff2 * (T / T_0)**2 + C_ff1 * (T / T_0) + C_ff_ch * T * h
    F_fuel = ca.fmax(F_fuel, 0)
    
    dV_dt = (T - D) / m - g * ca.sin(gamma)
    dd_dt = V * ca.cos(gamma)
    dh_dt = V * ca.sin(gamma)
    dm_dt = -F_fuel
    
    return ca.vertcat(dV_dt, dd_dt, dh_dt, dm_dt), F_fuel