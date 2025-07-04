#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import os # Import the os module

def compare_simulations():
    rospy.init_node('simulation_comparator', anonymous=True)

    # --- Define the shared directory where results are saved ---
    results_directory = os.path.expanduser('~/miguel_ws/src/trajectory_pkg/scripts/dynamic_study_new')

    # --- Load KDL Simulation Results ---
    kdl_file_path = os.path.join(results_directory, 'kdl_sim_results.npz')
    try:
        kdl_data = np.load(kdl_file_path)
        kdl_times = kdl_data['times']
        kdl_q = kdl_data['q']
        kdl_q_dot = kdl_data['q_dot']
        kdl_tau_applied = kdl_data['tau_applied'] 
        rospy.loginfo(f"KDL simulation results loaded from '{kdl_file_path}'.")
    except FileNotFoundError:
        rospy.logerr(f"File '{kdl_file_path}' not found. Ensure 'ur5_dynamic_equations.py' was run and saved results to this path.")
        return
    except Exception as e:
        rospy.logerr(f"Error loading KDL simulation results from '{kdl_file_path}': {e}")
        return
    
    # --- Load Gazebo Simulation Results ---
    gazebo_file_path = os.path.join(results_directory, 'gazebo_sim_results.npz')
    try:
        gazebo_data = np.load(gazebo_file_path)
        gazebo_times = gazebo_data['times']
        gazebo_q = gazebo_data['q']
        gazebo_q_dot = gazebo_data['q_dot']
        # You might also save 'tau_applied' from Gazebo if you logged it there
        # For now, we'll assume it's the same applied torque as KDL
        rospy.loginfo(f"Gazebo simulation results loaded from '{gazebo_file_path}'.")
    except FileNotFoundError:
        rospy.logerr(f"File '{gazebo_file_path}' not found. Ensure 'send_torques_to_gazebo.py' was run and saved results to this path.")
        return
    except Exception as e:
        rospy.logerr(f"Error loading Gazebo simulation results from '{gazebo_file_path}': {e}")
        return

    # --- Plot and Compare ---
    plt.figure(figsize=(15, 18)) 
    joint_names = ['Shoulder Pan', 'Shoulder Lift', 'Elbow', 'Wrist 1', 'Wrist 2', 'Wrist 3']

    for i in range(kdl_q.shape[1]):
        # Position and Velocity Comparison
        plt.subplot(6, 2, 2*i + 1) 
        
        plt.plot(kdl_times, np.degrees(kdl_q[:, i]), label='KDL Pos (deg)', color='blue', linewidth=2)
        plt.plot(kdl_times, np.degrees(kdl_q_dot[:, i]), label='KDL Vel (deg/s)', color='green', linestyle='--', linewidth=2)
        
        # Interpolate Gazebo data to KDL times for smoother comparison
        q_gazebo_interp = np.interp(kdl_times, gazebo_times, gazebo_q[:, i])
        q_dot_gazebo_interp = np.interp(kdl_times, gazebo_times, gazebo_q_dot[:, i])

        plt.plot(kdl_times, np.degrees(q_gazebo_interp), label='Gazebo Pos (deg)', color='red', linestyle=':', linewidth=2)
        plt.plot(kdl_times, np.degrees(q_dot_gazebo_interp), label='Gazebo Vel (deg/s)', color='orange', linestyle=':', linewidth=2)

        plt.title(f'Junta {i+1} ({joint_names[i]}) - Posição/Velocidade')
        plt.xlabel('Time (s)')
        plt.ylabel('Value')
        plt.grid(True)
        if i == 0: 
            plt.legend(fontsize=8)
        
        # Applied Torque Comparison (plotting the same applied torque from KDL for both)
        plt.subplot(6, 2, 2*i + 2) 
        plt.plot(kdl_times, kdl_tau_applied[:, i], label='Applied Torque (Nm)', color='purple', linewidth=2)
        # Assuming the same torque profile was applied in Gazebo as calculated by KDL
        plt.plot(kdl_times, kdl_tau_applied[:, i], label='Gazebo Applied Torque (Nm)', color='darkblue', linestyle=':', linewidth=2, alpha=0.7)
        
        plt.title(f'Junta {i+1} ({joint_names[i]}) - Torque Aplicado')
        plt.xlabel('Time (s)')
        plt.ylabel('Torque (Nm)')
        plt.grid(True)
        if i == 0:
            plt.legend(fontsize=8)
        
    plt.tight_layout(rect=[0, 0.03, 1, 0.95]) 
    plt.suptitle('Comparação da Dinâmica do UR5: Modelo Explícito vs. Gazebo Simulador', fontsize=16, y=0.98)
    plt.show()

if __name__ == '__main__':
    try:
        compare_simulations()
    except rospy.ROSInterruptException:
        pass