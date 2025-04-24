import casadi as ca
import numpy as np
from .constants import *
from .dynamics import dynamics

def setup_and_solve_optimization():
    opti = ca.Opti()
    
    # Variables
    X = opti.variable(4, N+1)  # States: [V, d, h, m]
    U = opti.variable(2, N)    # Controls: [T, gamma]
    dt = t_f / N
    
    # States and controls
    V = X[0, :]
    d = X[1, :]
    h = X[2, :]
    m = X[3, :]
    T = U[0, :]
    gamma = U[1, :]
    
    # Objective: Minimize fuel
    J = 0
    for k in range(N):
        x_k = X[:, k]
        u_k = U[:, k]
        x_kp1 = X[:, k+1]
        f_k, F_fuel_k = dynamics(x_k, u_k)
        f_kp1, F_fuel_kp1 = dynamics(x_kp1, u_k)
        J += 0.5 * dt * (F_fuel_k + F_fuel_kp1)
    
    opti.minimize(J)
        
    # Constraints: State and control boundary conditions
    opti.subject_to(V[0] == 200)  # Initial velocity condition
    opti.subject_to(d[0] == 0)    # Initial distance condition
    opti.subject_to(h[0] == 37000 * 0.3048)  # Initial altitude in meters (converted)
    opti.subject_to(m[0] == m0)   # Initial mass condition

    # Relaxed final altitude constraint
    opti.subject_to(h[N] >= 5000 * 0.3048 - 200 * 0.3048)  # Relaxed lower bound
    opti.subject_to(h[N] <= 5000 * 0.3048 + 200 * 0.3048)  # Relaxed upper bound

    # Distance constraints at final timestep (relaxed bounds)
    opti.subject_to(d[N] >= 200 * 1852 - 2 * 1852)  # Relaxed lower bound
    opti.subject_to(d[N] <= 200 * 1852 + 2 * 1852)  # Relaxed upper bound
    
    for k in range(N+1):
        opti.subject_to(opti.bounded(V_min, V[k], V_max))
        opti.subject_to(m[k] >= 0.8 * m0)
        opti.subject_to(V[k] > 0)
        opti.subject_to(h[k] > 0)
    for k in range(N):
        opti.subject_to(opti.bounded(0, T[k], 0.2 * T_max))  # Relaxed
        opti.subject_to(opti.bounded(-gamma_max, gamma[k], gamma_max))
    
    # Dynamics constraints
    for k in range(N):
        x_k = X[:, k]
        x_kp1 = X[:, k+1]
        u_k = U[:, k]
        f_k, _ = dynamics(x_k, u_k)
        f_kp1, _ = dynamics(x_kp1, u_k)
        opti.subject_to(x_kp1 == x_k + 0.5 * dt * (f_k + f_kp1))
    
    # Initial guess
    V_init = np.linspace(200, 180, N+1)
    d_init = np.linspace(0, 200 * 1852, N+1)
    h_init = np.linspace(37000 * 0.3048, 5000 * 0.3048, N+1)
    m_init = np.linspace(m0, 0.95 * m0, N+1)
    T_init = 0.05 * T_max * np.ones(N)
    gamma_init = -np.radians(3) * np.ones(N)  # Constant 3-degree descent
    opti.set_initial(V, V_init)
    opti.set_initial(d, d_init)
    opti.set_initial(h, h_init)
    opti.set_initial(m, m_init)
    opti.set_initial(T, T_init)
    opti.set_initial(gamma, gamma_init)
    
    # Solve
    opti.solver("ipopt", {
        "ipopt.print_user_options": "yes",
        "ipopt.print_info_string": "yes",
        "ipopt.max_iter": 1000,
        "ipopt.tol": 1e-6,
        "ipopt.acceptable_tol": 1e-4,
        "ipopt.print_level": 5
    })
    try:
        sol = opti.solve()
        t = np.linspace(0, t_f, N+1)
        V = sol.value(V)
        d = sol.value(d)
        h = sol.value(h)
        m = sol.value(m)
        T = sol.value(T)
        gamma = sol.value(gamma)
        # Pad T and gamma to length N+1 by repeating the last value
        T = np.pad(T, (0, 1), mode='edge')  # [T[0], ..., T[N-1], T[N-1]]
        gamma = np.pad(gamma, (0, 1), mode='edge')
    except Exception as e:
        print(f"Optimization failed: {str(e)}")
        print("Debugging values:")
        print("V:", opti.debug.value(V))
        print("d:", opti.debug.value(d))
        print("h:", opti.debug.value(h))
        print("m:", opti.debug.value(m))
        print("T:", opti.debug.value(T))
        print("gamma:", opti.debug.value(gamma))
        t = np.linspace(0, t_f, N+1)
        V = np.linspace(200, 180, N+1)
        d = np.linspace(0, 200 * 1852, N+1)
        h = np.linspace(37000 * 0.3048, 5000 * 0.3048, N+1)
        m = np.linspace(m0, 0.95 * m0, N+1)
        T = 0.05 * T_max * np.ones(N+1)  # Changed to N+1
        gamma = -0.02 * np.ones(N+1)    # Changed to N+1
    
    return t, V, d, h, m, T, gamma