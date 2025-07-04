#!/usr/bin/env python3

import numpy as np
from IK_dynamics_ur5 import inverse_dynamics_ur5
from K_dynamics_ur5 import direct_dynamics_ur5
from ur5_dynamic_equations import ur5_matrices
from initial_params import get_initial_params
two_pi = 2 * np.pi

""" 
Este script simula a modelagem dinâmica do robô UR5, calculando a matriz de massa, 
vetor de gravidade e termos de Coriolis/Centrífuga.
"""
params = get_initial_params()  # Get initial parameters from the initial_params module
# Test with some example joint states
# q_test = np.array([0.0, -np.pi/2, np.pi/2, 0.0, 0.0, 0.0]) # UR5 home pose
q_test = params['q_initial']  # Get initial parameters from the initial_params module
# dq_test = np.zeros(6)  # Joint velocities (0 rad/s)
# dq_test = np.array([0.1, 0.05, -0.02, 0.0, 0.0, 0.0])      # joint velocities (0 rad/s)
dq_test = params['dq_initial']  # Get initial parameters from the initial_params module

'''  
# Calculate the mass matrix, gravity vector, and Coriolis/Centrifugal terms
M_matrix, G_vector, CDq_vector = ur5_matrices(q_test, dq_test)

print("--- Mass Matrix (M) ---")
print(M_matrix)
print("\n--- Gravity Vector (G) ---")
print(G_vector)
print("\n--- Coriolis/Centrifugal Term Vector (CDq) ---")
print(CDq_vector)
'''

'''  
print("\n--- Inverse Dynamics ---")
desired_q_ddot = np.array([0.5, 0.2, 0.1, 0.0, 0.0, 0.0]) # Desired accelerations

tau_required = inverse_dynamics_ur5(q_test, dq_test, desired_q_ddot)
print("Desired q_ddot:")
print(desired_q_ddot)
print("\nCalculated Torques (tau) for desired acceleration:")
print(tau_required)
'''

print("\n--- Direct Dynamics ---")
# importar o torque aplicado do arquivo salvo "gazebo_sim_results.npz"

# applied_tau = np.array([-73.54215138984566, 74.81404639938762, 77.51897977306223, 15.978459680192621, 0.5520043982634526, 9.997587733944673]) # applied torques
applied_tau = params['tau_apply'] # Get initial parameters from the initial_params module

resulting_q_ddot = direct_dynamics_ur5(q_test, dq_test, applied_tau)
print("Applied Torques (tau):")
print(applied_tau)
print("\nCalculated Resulting Accelerations (q_ddot):")
print(resulting_q_ddot)

# Example of direct dynamics in action (simple integration loop)
print("\n--- Simple Direct Dynamics Simulation Loop ---")
sim_q = q_test
sim_dq = dq_test 
sim_dt = 0.01
sim_duration = 1.0

sim_times = []
sim_q_hist = []
sim_dq_hist = []

# torque_pulse = np.array([10.0, 20.0, 5.0, 1.0, 0.5, 0.1]) # Apply a pulse on first few joints
torque_pulse = applied_tau 

current_sim_time = 0.0
while current_sim_time <= sim_duration:
    # Apply torque for first 0.1 seconds, then zero
    current_tau = torque_pulse if current_sim_time < 0.1 else np.zeros(6)
    
    sim_q_ddot = direct_dynamics_ur5(sim_q, sim_dq, current_tau).flatten() # Flatten to 1D array
    
    # Euler integration
    sim_dq += sim_q_ddot * sim_dt
    sim_q += sim_dq * sim_dt

    # Redução para o intervalo [-2π, 2π], preservando o sinal
    sim_q_reduced = sim_q % two_pi  # módulo padrão
    sim_q_reduced = np.where(sim_q < 0, sim_q_reduced - two_pi, sim_q_reduced)  # Corrige os negativos que estavam além de -2π
        
    sim_times.append(current_sim_time)
    sim_q_hist.append(sim_q_reduced.copy())
    sim_dq_hist.append(sim_dq.copy())
    
    current_sim_time += sim_dt

print("Simulation completed. Last joint positions (rad):")
print(sim_q_reduced)
print("Last joint velocities (rad/s):")
print(sim_dq)

# Plotting position and velocity over time
try:
    import matplotlib.pyplot as plt
    # plot of position over time
    plt.figure(figsize=(10, 6))
    for i in range(6):
        # plt.plot(sim_times, np.degrees(np.array(sim_q_hist)[:, i]), label=f'Joint {i+1} Position (deg)')
        plt.plot(sim_times, np.array(sim_q_hist)[:, i], label=f'Joint {i+1} Position (deg)')
    plt.title('Simulated Joint Positions over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (deg)')
    plt.legend()
    plt.grid(True)
    
    # plot of velocity over time
    plt.figure(figsize=(10, 6))
    for i in range(6):
        plt.plot(sim_times, np.array(sim_dq_hist)[:, i], label=f'Joint {i+1} velocity (rad/s)')
    plt.title('Simulated Joint Velocity over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (rad/s)')
    plt.legend()
    plt.grid(True)
    plt.show()

except ImportError:
    print("\nMatplotlib not found. Install it with 'pip install matplotlib' to see plots.")
