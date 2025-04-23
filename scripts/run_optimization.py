import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
from src.optimization import setup_and_solve_optimization
from src.dynamics import dynamics
from src.uncertainty import simulate_uncertainty
from src.metrics import compute_link_metrics, optimize_sta_waypoints
from src.plotting import plot_results
from src.constants import *

def main():
    t, V, d, h, m, T, gamma = setup_and_solve_optimization()
    dt = t_f / N
    fuel_flow = np.zeros(N+1)
    cumulative_fuel = np.zeros(N+1)
    for k in range(N):
        x_k = np.array([V[k], d[k], h[k], m[k]])
        u_k = np.array([T[k], gamma[k]])
        _, F_fuel_k = dynamics(x_k, u_k)
        x_kp1 = np.array([V[k+1], d[k+1], h[k+1], m[k+1]])
        _, F_fuel_kp1 = dynamics(x_kp1, u_k)
        fuel_flow[k] = F_fuel_k
        fuel_flow[k+1] = F_fuel_kp1
        cumulative_fuel[k+1] = cumulative_fuel[k] + 0.5 * dt * (F_fuel_k + F_fuel_kp1)
    d_total = d[-1]
    dist_to_go = d_total - d
    baseline_STA_times = [0, t_f]
    baseline_x, baseline_s = simulate_uncertainty(t, V, baseline_STA_times)
    baseline_P_link, baseline_fuel = compute_link_metrics(t, V, baseline_x, baseline_s, h, T)
    N_values = [1, 2, 3, 4, 5]
    uncertainties = {}
    STA_times_dict = {}
    P_link_dict = {}
    for N_links in N_values:
        STA_times = optimize_sta_waypoints(N_links, t[-1], V, h, t, T)
        x, s = simulate_uncertainty(t, V, STA_times)
        P_link = []
        for i in range(N_links-1):
            idx = (t >= STA_times[i]) & (t <= STA_times[i+1])
            if np.sum(idx) >= 2:
                P_link_i, _ = compute_link_metrics(t[idx], V[idx], x[idx], s[idx], h[idx], T[idx])
                P_link.append(P_link_i)
        idx = (t >= STA_times[N_links-1]) & (t <= t[-1])
        if np.sum(idx) >= 2:
            P_link_i, _ = compute_link_metrics(t[idx], V[idx], x[idx], s[idx], h[idx], T[idx])
            P_link.append(P_link_i)
        uncertainties[N_links] = x / 1852
        STA_times_dict[N_links] = STA_times
        P_link_dict[N_links] = max(P_link) if P_link else baseline_P_link
    plot_results(t, V, d, h, m, T, gamma, fuel_flow, cumulative_fuel, dist_to_go, uncertainties, STA_times_dict, baseline_P_link, baseline_fuel)
    print("Total Fuel (kg):", cumulative_fuel[-1])
    print("Baseline Fuel (kg):", baseline_fuel)
    print("Baseline Throughput (aircraft/s):", baseline_P_link)
    print(P_link_dict)

    for N_links in N_values:
        print(f"Max Path Uncertainty (NM) for N={N_links}:", np.max(uncertainties[N_links]))
        print(f"Throughput (aircraft/s) for N={N_links}:", P_link_dict[N_links])
        print(f"STA Times for N={N_links}:", STA_times_dict[N_links])
        sta_distances = [d_total - np.interp(t_sta, t, d) for t_sta in STA_times_dict[N_links]]
        print(f"STA Distances (NM) for N={N_links}:", [-sd / 1852 for sd in sta_distances])
    np.savez("results/cda_results.npz", t=t, V=V, d=d, h=h, m=m, T=T, gamma=gamma, x=x, s=s, fuel_flow=fuel_flow, cumulative_fuel=cumulative_fuel, dist_to_go=dist_to_go, uncertainties=uncertainties)

if __name__ == "__main__":
    main()