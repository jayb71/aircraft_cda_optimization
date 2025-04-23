import numpy as np
from scipy.optimize import minimize
from .constants import *
from .uncertainty import simulate_uncertainty

def compute_link_metrics(t, V, x, s, h, T):
    V_GS = V + w_0 - s
    t_sep = (2 * x + x_req) / (V_GS + 1e-6)
    P_link = 1 / np.max(t_sep)
    fuel = 0
    for i in range(len(t)-1):
        dt = t[i+1] - t[i]
        rho = rho_0 * (1 - 2.255e-5 * h[i]) ** 4.258
        T_0 = T_max * (rho / rho_0)
        T_val = T[i] if i < len(T) else T[-1]
        F_fuel = C_ff3 * (T_val / T_0)**3 + C_ff2 * (T_val / T_0)**2 + C_ff1 * (T_val / T_0) + C_ff_ch * T_val * h[i]
        fuel += F_fuel * dt
    return P_link, fuel

def optimize_sta_waypoints(N_links, t_total, V, h, t, T):
    if N_links == 1:
        return np.array([0, t_total])
    def objective(t_durations):
        STA_times = np.cumsum([0] + list(t_durations))
        x, s = simulate_uncertainty(t, V, STA_times)
        P_links = []
        fuels = []
        for i in range(N_links-1):
            idx = (t >= STA_times[i]) & (t <= STA_times[i+1])
            if np.sum(idx) < 2:
                return 1e6
            P_link, fuel = compute_link_metrics(t[idx], V[idx], x[idx], s[idx], h[idx], T[idx])
            P_links.append(P_link)
            fuels.append(fuel)
        idx = (t >= STA_times[N_links-1]) & (t <= t_total)
        if np.sum(idx) >= 2:
            P_link, fuel = compute_link_metrics(t[idx], V[idx], x[idx], s[idx], h[idx], T[idx])
            P_links.append(P_link)
            fuels.append(fuel)
        alpha = 10
        return sum(fuels) - alpha * min(P_links)
    
    constraints = [
        {"type": "eq", "fun": lambda t_durations: sum(t_durations) - t_total},
        {"type": "ineq", "fun": lambda t_durations: min(t_durations) - 50},
        {"type": "ineq", "fun": lambda t_durations: t_total - max(t_durations)}
    ]
    bounds = [(50, t_total)] * (N_links-1)
    result = minimize(
        objective, [t_total / N_links] * (N_links-1),
        constraints=constraints, bounds=bounds, method="SLSQP",
        options={"maxiter": 100, "disp": True}
    )
    if not result.success:
        print("STA optimization failed:", result.message)
        return np.linspace(0, t_total, N_links)
    return np.cumsum([0] + list(result.x))