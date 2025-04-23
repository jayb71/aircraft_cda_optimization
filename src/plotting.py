import matplotlib.pyplot as plt
import numpy as np
from .constants import *
from .metrics import compute_link_metrics

def plot_results(t, V, d, h, m, T, gamma, fuel_flow, cumulative_fuel, dist_to_go, uncertainties, STA_times_dict, baseline_P_link, baseline_fuel):
    plt.figure(figsize=(12, 8))
    
    # Distance to Go vs. Fuel Flow Rate
    plt.subplot(2, 2, 1)
    plt.plot(-dist_to_go / 1852, fuel_flow * 60)
    plt.xlabel("Distance to Go (NM)")
    plt.ylabel("Fuel Flow Rate (kg/min)")
    plt.title("Fuel Flow Rate vs. Distance to Go")
    
    # Distance to Go vs. Total Fuel Consumption
    plt.subplot(2, 2, 2)
    plt.plot(-dist_to_go / 1852, cumulative_fuel)
    plt.xlabel("Distance to Go (NM)")
    plt.ylabel("Total Fuel Consumption (kg)")
    plt.title("Total Fuel Consumption vs. Distance to Go")
    
    # Path Uncertainty vs. Distance to Go
    plt.subplot(2, 2, 3)
    for N_links in [1, 2, 3, 4, 5]:
        plt.plot(-dist_to_go / 1852, uncertainties[N_links], label=f"N = {N_links}")
    plt.xlabel("Distance to Go (NM)")
    plt.ylabel("Path Uncertainty (NM)")
    plt.title("Path Uncertainty vs. Distance to Go")
    plt.legend()
    
    # Throughput vs. Fuel Consumption
    P_links = []
    fuels = []
    for i in range(5-1):
        idx = (t >= STA_times_dict[5][i]) & (t <= STA_times_dict[5][i+1])
        if np.sum(idx) < 2:
            continue
        P_link, fuel = compute_link_metrics(t[idx], V[idx], uncertainties[5][idx]*1852, np.zeros_like(t[idx]), h[idx], T[idx])
        P_links.append(P_link)
        fuels.append(fuel)
    idx = (t >= STA_times_dict[5][4]) & (t <= t[-1])
    if np.sum(idx) >= 2:
        P_link, fuel = compute_link_metrics(t[idx], V[idx], uncertainties[5][idx]*1852, np.zeros_like(t[idx]), h[idx], T[idx])
        P_links.append(P_link)
        fuels.append(fuel)
    
    plt.subplot(2, 2, 4)
    plt.scatter(fuels, P_links, c='blue', label="Optimized STAs")
    plt.scatter([baseline_fuel], [baseline_P_link], c='red', label="Baseline (No STAs)")
    plt.xlabel("Fuel Consumption per Link (kg)")
    plt.ylabel("Throughput (aircraft/s)")
    plt.title("Throughput vs. Fuel Consumption")
    plt.legend()
    
    plt.tight_layout()
    plt.savefig("results/cda_paper_results.png")
    plt.close()